"""Autonomous rectangle loop: robot odometry + LiDAR in, ``/cmd_vel`` out.

    # dry run -- computes and reports commands, publishes NOTHING
    python3 -m go2_survey_loop.loop_node --ros-args -p length:=4.0 -p width:=3.0
    ros2 service call /go2_loop/start std_srvs/srv/Trigger

    # live -- only with go2_survey_bridge running and a person on the remote
    python3 -m go2_survey_loop.loop_node --ros-args \
        -p length:=4.0 -p width:=3.0 -p dry_run:=false

It never moves on its own. Three things must all be true before a single
non-zero velocity leaves this node:

1. it was launched with ``dry_run:=false`` (the default is true),
2. a loop size was given (there is no default size -- zero is refused),
3. someone called ``~/start``, and at that moment odometry and LiDAR were
   fresh, the floor had been found, and nothing was in the way.

Everything it publishes goes to /cmd_vel, so the bridge's clamp, slew limit,
300 ms watchdog and e-stop all still apply underneath. This node adds the
mission-level stops: obstacle, stale odometry, stale LiDAR, geofence, time
limit. See follower.py for their precedence.

Inputs default to the Go2's own LiDAR stack: /utlidar/robot_odom (pose) and
/utlidar/cloud_deskewed, which that stack already publishes in the odom
frame. Parameters must be typed as their defaults: ``length:=4.0``, not
``length:=4``.

Services: ``~/start``, ``~/stop`` (abort), ``~/resume`` (after a pause).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from go2_survey_loop.cmu import (
    CARROT_M, PASS_LATERAL_M, SENSOR_OFFSET_X, SPORT_STOP_MOVE_ID, STOP_ALL,
    effective_cloud_stride, step_cmu, vehicle_pose,
)
from go2_survey_loop.explore import ExploreMission, ExploreParams
from go2_survey_loop.route import DetourParams, RouteMission
from go2_survey_loop.follower import FollowerParams, Mission, SafetyParams, State
from go2_survey_loop.geometry import (
    Pose2D, plan_local, route_extent, to_world, yaw_from_quaternion,
)
from go2_survey_loop.obstacles import (
    ObstacleMemory, ObstacleParams, classify, floor_from_scan, floor_plane_from_scan, floor_xy,
    median, obstacle_xy, ray_points, ring_count,
)
try:
    from go2_survey_bridge.sport_api import build_stop_move_request as _build_stop_move
except ImportError:
    _build_stop_move = None

_POINTFIELD = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def _age(seconds: float) -> str:
    return "never received" if math.isinf(seconds) else f"{seconds:.2f}s old"


def cloud_points(msg: PointCloud2, stride: int):
    """PointCloud2 -> list of (x, y, z), every ``stride``-th finite point."""
    by = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if not {"x", "y", "z"} <= set(by):
        return []
    order = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [order + _POINTFIELD[by[k][1]] for k in "xyz"],
        "offsets": [by[k][0] for k in "xyz"],
        "itemsize": msg.point_step,
    })
    n = len(msg.data) // msg.point_step
    arr = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)[::max(1, stride)]
    xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return [tuple(p) for p in xyz.tolist()]


class LoopNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_loop")
        d = self.declare_parameter
        d("shape", "rectangle")             # or "out_and_back", or "explore" (no route:
                                            # greedy legs into open space, then home)
        d("explore_legs", 8)
        d("explore_leg_max", 1.5)           # m
        d("explore_radius", 4.0)            # m from the start; geofence = this + margin
        d("explore_strategy", "frontier")   # or "greedy" (explore.py)
        # routes: blocked ahead this long -> a clear path round (route.py)
        d("route_detour", True)
        d("detour_wait_s", 3.0)
        d("detour_clearance", 0.80)         # m from any obstacle cell
        d("length", 0.0)
        d("width", 0.0)
        d("turn", "left")
        d("laps", 1)
        d("dry_run", True)
        # "internal" (default): publish Twist on cmd_vel_topic for go2_survey_bridge.
        # "cmu" (D5, guide 05): publish PointStamped on waypoint_topic and Int8 on
        # stop_topic for the CMU Go2 autonomy stack, plus StopMove on halt.
        d("planner", "internal")
        d("waypoint_topic", "/way_point")
        d("stop_topic", "/stop")
        d("cmu_sensor_offset_x", SENSOR_OFFSET_X)   # m, L1 ahead of body centre
        d("cmu_carrot_m", CARROT_M)                 # m beyond the waypoint (pathFollower slows inside 0.75 m)
        d("cmu_pass_lateral_m", PASS_LATERAL_M)     # m: crossed the waypoint's line within this -> reached
        d("cmd_vel_topic", "/cmd_vel")
        d("odom_topic", "/utlidar/robot_odom")
        d("cloud_topic", "/utlidar/cloud_deskewed")
        # a second LiDAR already in the odom frame, e.g. /mid360/cloud_odom
        # (tools/mid360_to_odom.py); "" = none. Adds obstacles, never required.
        d("extra_cloud_topic", "")
        d("extra_cloud_stride", 8)          # the Mid-360 sends ~10x the points
        d("rate_hz", 10.0)
        d("cloud_stride", 4)
        d("max_vx", 0.30)
        d("min_vx", 0.30)                   # walking, short of a waypoint (follower.py)
        d("stall_s", 5.0)                   # pause if commanded but not moving; 0 = off
        d("max_vyaw", 1.00)
        d("min_vyaw", 0.70)                 # turning in place never commands less (follower.py)
        d("reach_tol", 0.20)
        d("lookahead", 0.6)                 # m along the route line; 0 = aim at corners
        d("stop_distance", 0.50)
        d("lane_half_width", 0.35)
        d("clearance_radius", 0.55)
        # a turn on the spot waits while anything seen in the last memory_s
        # seconds -- behind it included -- is within turn_clearance of centre
        d("turn_clearance", 0.55)
        d("memory_s", 10.0)
        d("memory_range", 2.5)              # m: turn memory only needs near-field obstacles
        d("body_front", 0.35)
        d("body_rear", 0.40)
        d("body_half_width", 0.20)
        # 0.13, not 0.10 -- walking, the built-in cloud tilts ~3 deg (floor
        # -0.07 underfoot, +0.03 two metres ahead). Combined with floor_plane
        # below, this prevents carpet returns ahead from reading as obstacles.
        d("obstacle_z_min", 0.13)
        # the floor as a plane fitted to each built-in scan (obstacles.
        # floor_plane_from_scan) instead of one height: handles gait pitch tilt.
        d("floor_plane", True)
        d("obstacle_z_max", 1.00)
        d("min_obstacle_points", 5)
        d("odom_timeout_s", 0.5)
        d("cloud_timeout_s", 1.0)
        d("max_duration_s", 300.0)
        d("geofence_margin_m", 1.0)
        d("auto_resume_s", 0.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dry_run = bool(g("dry_run"))
        self.planner = str(g("planner"))
        self.follow = FollowerParams(max_vx=g("max_vx"), min_vx=g("min_vx"),
                                     max_vyaw=g("max_vyaw"),
                                     min_vyaw=g("min_vyaw"), reach_tol=g("reach_tol"),
                                     lookahead=g("lookahead"))
        self.obst = ObstacleParams(
            body_front=g("body_front"), body_rear=g("body_rear"),
            body_half_width=g("body_half_width"), stop_distance=g("stop_distance"),
            lane_half_width=g("lane_half_width"), clearance_radius=g("clearance_radius"),
            z_min=g("obstacle_z_min"), z_max=g("obstacle_z_max"))
        self.stride = effective_cloud_stride(self.planner, int(g("cloud_stride")))

        self._pose: Pose2D | None = None
        self._pose_z: float = 0.0
        self._pose_t: float | None = None
        self._odom_frame = ""
        self._cloud_t: float | None = None
        self._front = self._near = 0
        self._front2 = self._near2 = 0          # the second LiDAR's, if any
        self._plane, self._plane_t = None, 0.0  # latest floor plane (floor_plane)
        self._extra_t: float | None = None
        self.extra_stride = int(g("extra_cloud_stride"))
        # ~2 s of scans' obstacle (x, y) for exploration to plan from: one scan
        # is sparse (thin legs, desk edges); planning happens standing still
        self._obst_scans: deque = deque(maxlen=30)
        self._memory = ObstacleMemory(keep_s=float(g("memory_s")))
        self._turn_clearance = float(g("turn_clearance"))
        self._turn_near = 0
        self._pruned_t = 0.0
        self._explore_scans = 0
        self._floor_hist: deque = deque(maxlen=15)
        self._frame_warned = False
        self.mission: Mission | None = None
        self._last_state = None
        self._cmu_active = False                # a /way_point was published last tick
        self._ticks = 0

        odom_topic = str(g("odom_topic"))
        cloud_topic = str(g("cloud_topic"))
        if self.planner == "cmu":
            if odom_topic == "/utlidar/robot_odom":
                odom_topic = "/state_estimation"
            if cloud_topic == "/utlidar/cloud_deskewed":
                cloud_topic = "/registered_scan"

        self.cmd_pub = self.create_publisher(Twist, g("cmd_vel_topic"), 10)
        self.wp_pub = self.create_publisher(PointStamped, g("waypoint_topic"), 10) if self.planner == "cmu" else None
        self.stop_pub = self.create_publisher(Int8, g("stop_topic"), 10) if self.planner == "cmu" else None
        self.sport_pub = None
        if self.planner == "cmu":
            try:
                from unitree_api.msg import Request as SportRequest
                self._sport_req_cls = SportRequest
                self.sport_pub = self.create_publisher(SportRequest, "/api/sport/request", 10)
            except ImportError:
                self._sport_req_cls = None
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud,
                                 qos_profile_sensor_data)
        if g("extra_cloud_topic"):
            self.create_subscription(PointCloud2, g("extra_cloud_topic"), self._on_extra_cloud,
                                     qos_profile_sensor_data)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_service(Trigger, "~/resume", self._srv_resume)
        self.create_timer(1.0 / max(1.0, g("rate_hz")), self._tick)

        out_desc = (f"{g('waypoint_topic')} + {g('stop_topic')} (CMU planner)"
                    if self.planner == "cmu" else str(g("cmd_vel_topic")))
        mode = "DRY RUN -- nothing will be published" if self.dry_run \
            else f"LIVE -- publishing {out_desc}"
        route = (f"explore: {g('explore_legs')} legs of <= {g('explore_leg_max')} m "
                 f"within {g('explore_radius')} m, then home"
                 if g("shape") == "explore" else
                 f"route {g('shape')}: length {g('length')} m"
                 + (f", width {g('width')} m, turn {g('turn')}" if g("shape") == "rectangle" else "")
                 + f", laps {g('laps')}")
        self.get_logger().info(
            f"go2_loop up, {mode}\n"
            f"  {route}\n"
            f"  speed <= {self.follow.max_vx} m/s, {self.follow.max_vyaw} rad/s; "
            f"stop {self.obst.stop_distance} m ahead / {self.obst.clearance_radius} m around\n"
            f"  waiting for ~/start")

    # ------------------------------------------------------------- inputs

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self.planner == "cmu":
            # Point-LIO's /state_estimation is the L1 LiDAR's pose; shift back
            # to the body centre so our body box, clearance ring and waypoints
            # match localPlanner / pathFollower (sensorOffsetX).
            pose = vehicle_pose(pose, float(self.get_parameter("cmu_sensor_offset_x").value))
        self._pose = pose
        self._pose_z = float(p.z)
        self._pose_t = self._now()          # receipt time: robot clocks may differ
        self._odom_frame = msg.header.frame_id

    def _on_cloud(self, msg: PointCloud2) -> None:
        pose = self._pose
        if pose is None:
            return
        if self._odom_frame and msg.header.frame_id != self._odom_frame:
            # Not refreshing _cloud_t means the mission sees the LiDAR as
            # stale and stays paused: a cloud we cannot place is no cloud.
            if not self._frame_warned:
                self.get_logger().error(
                    f"cloud frame '{msg.header.frame_id}' != odom frame "
                    f"'{self._odom_frame}' -- refusing to use it")
                self._frame_warned = True
            return
        pts = cloud_points(msg, self.stride)
        f = floor_from_scan(pts, pose)
        if f is not None:
            self._floor_hist.append(f)
        floor_z = median(self._floor_hist)
        if floor_z is None:
            return                          # can't tell floor from obstacle yet
        floor = floor_z
        if self.get_parameter("floor_plane").value:
            plane = floor_plane_from_scan(pts, pose)
            if plane is not None:
                floor, self._plane, self._plane_t = plane, plane, self._now()
        self._front, self._near = classify(pts, pose, floor, self.obst)
        self._ingest(pts, pose, floor)
        self._cloud_t = self._now()

    def _on_extra_cloud(self, msg: PointCloud2) -> None:
        """A second LiDAR, already in the odom frame (the Mid-360, via
        tools/mid360_to_odom.py). Its obstacles count too -- it sees behind
        the robot and far denser -- but the built-in LiDAR stays the one that
        must be fresh to move and that finds the floor."""
        pose, floor_z = self._pose, median(self._floor_hist)
        if pose is None or floor_z is None:
            return
        if self._odom_frame and msg.header.frame_id != self._odom_frame:
            return
        if self._plane is not None and self._now() - self._plane_t <= 0.5:
            floor_z = self._plane           # the built-in scan's floor plane, if fresh
        pts = cloud_points(msg, self.extra_stride)
        self._front2, self._near2 = classify(pts, pose, floor_z, self.obst)
        self._extra_t = self._now()
        self._ingest(pts, pose, floor_z)

    def _counts(self, now: float):
        """Obstacle counts from the built-in LiDAR, and from the second one
        while it is fresh: the larger of each."""
        front, near = self._front, self._near
        if self._extra_t is not None and now - self._extra_t <= 0.5:
            front, near = max(front, self._front2), max(near, self._near2)
        return front, near

    def _ingest(self, pts, pose: Pose2D, floor_z: float) -> None:
        """Turn memory and exploration's map, from either LiDAR."""
        now = self._now()
        obst = obstacle_xy(pts, pose, floor_z, self.obst)
        fl = floor_xy(pts, pose, floor_z)
        mem_range = float(self.get_parameter("memory_range").value)
        self._memory.add(obst, now, pose=pose, max_range=mem_range, floor_xy=fl, p=self.obst)
        if now - self._pruned_t > 1.0:
            self._memory.prune(now)
            self._pruned_t = now
        self._turn_near = ring_count(self._memory.points(), pose, self._turn_clearance, self.obst)
        if self.get_parameter("shape").value == "explore":
            # kept from before ~/start too: the first leg is planned at once
            self._obst_scans.append(obst)
        if isinstance(self.mission, (ExploreMission, RouteMission)):
            self._explore_scans += 1
            rays = (ray_points(pts, pose, floor_z, self.obst)
                    if self._explore_scans % 3 == 0 else ())         # CPU: every third scan
            self.mission.observe(fl, obst, (pose.x, pose.y), rays)

    def _ages(self, now: float):
        oa = math.inf if self._pose_t is None else now - self._pose_t
        ca = math.inf if self._cloud_t is None else now - self._cloud_t
        return oa, ca

    # -------------------------------------------------------------- control

    def _publish(self, vx: float, vyaw: float) -> None:
        if self.dry_run:
            return
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(vyaw)
        self.cmd_pub.publish(t)

    def _publish_cmu(self, cmu_out) -> None:
        if self.dry_run:
            return
        if self.stop_pub is not None:
            self.stop_pub.publish(Int8(data=int(cmu_out.stop_mask)))
        if cmu_out.waypoint is not None and self.wp_pub is not None:
            pt = PointStamped()
            pt.header.frame_id = self._odom_frame or "map"
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = cmu_out.waypoint
            self.wp_pub.publish(pt)
        if cmu_out.send_sport_stop:
            self._send_sport_stop()

    def _send_sport_stop(self) -> None:
        if self.dry_run or self.sport_pub is None or self._sport_req_cls is None:
            return
        if _build_stop_move is not None:
            req = _build_stop_move(self._sport_req_cls)
        else:
            req = self._sport_req_cls()
            req.header.identity.api_id = SPORT_STOP_MOVE_ID
        self.sport_pub.publish(req)

    def _halt_output(self) -> None:
        self._publish(0.0, 0.0)
        if self.planner == "cmu" and not self.dry_run:
            if self.stop_pub is not None:
                self.stop_pub.publish(Int8(data=STOP_ALL))
            self._send_sport_stop()
        self._cmu_active = False

    def _tick(self) -> None:
        now = self._now()
        oa, ca = self._ages(now)
        floor_z = median(self._floor_hist)
        front, near = self._counts(now)
        extra = ("" if self._extra_t is None else
                 f", mid360 {_age(now - self._extra_t)}")
        sensors = (f"odom {_age(oa)}, lidar {_age(ca)}{extra}, floor "
                   f"{'?' if floor_z is None else f'{floor_z:.2f}'}, "
                   f"front {front}, near {near}")
        self._ticks += 1

        if self.mission is None:
            if self._ticks % 10 == 0:
                self._status(f"idle | {sensors}")
            return

        m = self.mission
        obst = [p for scan in self._obst_scans for p in scan] if isinstance(m, ExploreMission) else ()
        if self.planner == "cmu":
            cmd, cmu_out = step_cmu(
                m, now, self._pose, oa, ca, front, near, obst,
                pose_z=self._pose_z, was_running=self._cmu_active,
                carrot_m=float(self.get_parameter("cmu_carrot_m").value),
                pass_lateral=float(self.get_parameter("cmu_pass_lateral_m").value))
            if cmd.publish:
                self._publish_cmu(cmu_out)
            self._cmu_active = (cmu_out.waypoint is not None)
        else:
            if isinstance(m, ExploreMission):
                cmd = m.step(now, self._pose, oa, ca, front, near, obst, self._turn_near)
            else:
                cmd = m.step(now, self._pose, oa, ca, front, near, self._turn_near)
            if cmd.publish:
                self._publish(cmd.vx, cmd.vyaw)

        if cmd.state != self._last_state:
            # One call site per severity. rclpy's logger raises "Logger
            # severity cannot be changed between calls" if a single line logs
            # at different levels -- which crashed this node on its first
            # pause or stop when the level was chosen in a variable.
            text = f"{cmd.state.value}: {cmd.reason}  ({sensors})"
            if cmd.state in (State.PAUSED, State.ABORTED):
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
            self._last_state = cmd.state

        tag = "would send" if self.dry_run else "send"
        if self.planner == "cmu":
            wp_str = ("none" if cmu_out.waypoint is None
                      else f"({cmu_out.waypoint[0]:.2f},{cmu_out.waypoint[1]:.2f})")
            self._status(f"{cmd.state.value} | {cmd.reason} | {tag} wp={wp_str} "
                         f"stop={cmu_out.stop_mask} | {sensors}")
        else:
            self._status(f"{cmd.state.value} | {cmd.reason} | {tag} vx={cmd.vx:.2f} "
                         f"vyaw={cmd.vyaw:.2f} | {sensors}")

    def _status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    # ------------------------------------------------------------- services

    def _srv_start(self, _req, res):
        if self.mission is not None and self.mission.state in (State.RUNNING, State.PAUSED):
            res.success, res.message = False, f"mission already {self.mission.state.value}"
            return res
        now = self._now()
        oa, ca = self._ages(now)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        k = int(g("min_obstacle_points"))
        problems = []
        if self._pose is None or oa > g("odom_timeout_s"):
            problems.append(f"no fresh odometry ({_age(oa)})")
        if ca > g("cloud_timeout_s"):
            problems.append(f"no fresh LiDAR ({_age(ca)})")
        if median(self._floor_hist) is None:
            problems.append("floor not found yet")
        front, near = self._counts(now)
        # In CMU mode the local planner steers round the front lane (just as
        # ~/resume already allows); only the clearance ring refuses a start.
        if near >= k or (self.planner != "cmu" and front >= k):
            problems.append(f"path blocked (front={front}, near={near})")
        shape = str(g("shape"))
        local = []
        if shape == "explore":
            if not self._obst_scans:
                problems.append("no LiDAR obstacle picture yet")
        else:
            try:
                local = plan_local(shape, float(g("length")), float(g("width")),
                                   str(g("turn")), int(g("laps")))
            except ValueError as exc:
                problems.append(str(exc))
        if problems:
            res.success, res.message = False, "refused: " + "; ".join(problems)
            self.get_logger().warning(res.message)
            return res

        start = self._pose
        extent = (float(g("explore_radius")) if shape == "explore"
                  else route_extent(shape, g("length"), g("width")))
        safety = SafetyParams(
            odom_timeout_s=g("odom_timeout_s"), cloud_timeout_s=g("cloud_timeout_s"),
            max_duration_s=g("max_duration_s"),
            geofence_m=extent + g("geofence_margin_m"),
            min_obstacle_points=k, auto_resume_s=g("auto_resume_s"),
            # a dry run publishes nothing, so the robot never moves: not a stall
            stall_s=0.0 if self.dry_run else g("stall_s"))
        if shape == "explore":
            ep = ExploreParams(legs=int(g("explore_legs")), leg_max=float(g("explore_leg_max")),
                               radius=float(g("explore_radius")),
                               strategy=str(g("explore_strategy")))
            self.mission = ExploreMission(start, now, self.follow, safety, ep, self.obst)
            what = f"exploring {ep.legs} legs within {ep.radius:.1f} m"
        else:
            if g("route_detour"):
                dp = DetourParams(wait_s=float(g("detour_wait_s")),
                                  path_clearance=float(g("detour_clearance")))
                self.mission = RouteMission(to_world(start, local), start, now, self.follow,
                                            safety, dp, self.obst)
                what = f"{len(local)} waypoints, going round obstacles"
            else:
                self.mission = Mission(to_world(start, local), start, now, self.follow, safety)
                what = f"{len(local)} waypoints"
        self._last_state = None
        self._cmu_active = False
        mode = "DRY RUN (nothing published)" if self.dry_run else "LIVE"
        res.success = True
        res.message = (f"started {mode}: {what} from "
                       f"({start.x:.2f}, {start.y:.2f}) heading {math.degrees(start.yaw):.0f} deg")
        self.get_logger().info(res.message)
        return res

    def _srv_stop(self, _req, res):
        if self.mission is None:
            res.success, res.message = True, "no mission"
            return res
        self.mission.abort(self._now(), "stopped by operator")
        self._halt_output()
        res.success = True
        res.message = ("mission aborted (dry run: nothing was being published)"
                       if self.dry_run else "mission aborted, zero velocity sent")
        self.get_logger().warning(res.message)
        return res

    def _srv_resume(self, _req, res):
        if self.mission is None:
            res.success, res.message = False, "no mission"
            return res
        now = self._now()
        oa, ca = self._ages(now)
        front, near = self._counts(now)
        ok, msg = self.mission.resume(oa, ca, 0 if self.planner == "cmu" else front, near)
        res.success, res.message = ok, msg
        if ok:                                   # separate sites: see _tick
            self.get_logger().info(f"resume: {msg}")
        else:
            self.get_logger().warning(f"resume: {msg}")
        return res

    def destroy_node(self) -> bool:
        try:
            self._halt_output()
        except Exception:
            pass
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = LoopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
