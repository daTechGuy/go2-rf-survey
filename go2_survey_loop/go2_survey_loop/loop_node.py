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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from go2_survey_loop.explore import ExploreMission, ExploreParams
from go2_survey_loop.follower import FollowerParams, Mission, SafetyParams, State
from go2_survey_loop.geometry import (
    Pose2D, plan_local, route_extent, to_world, yaw_from_quaternion,
)
from go2_survey_loop.obstacles import (
    ObstacleMemory, ObstacleParams, classify, floor_from_scan, floor_xy, median, obstacle_xy,
    ray_points, ring_count,
)

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
        d("length", 0.0)
        d("width", 0.0)
        d("turn", "left")
        d("laps", 1)
        d("dry_run", True)
        d("cmd_vel_topic", "/cmd_vel")
        d("odom_topic", "/utlidar/robot_odom")
        d("cloud_topic", "/utlidar/cloud_deskewed")
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
        d("body_front", 0.35)
        d("body_rear", 0.40)
        d("body_half_width", 0.20)
        d("obstacle_z_min", 0.10)
        d("obstacle_z_max", 1.00)
        d("min_obstacle_points", 5)
        d("odom_timeout_s", 0.5)
        d("cloud_timeout_s", 1.0)
        d("max_duration_s", 300.0)
        d("geofence_margin_m", 1.0)
        d("auto_resume_s", 0.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dry_run = bool(g("dry_run"))
        self.follow = FollowerParams(max_vx=g("max_vx"), min_vx=g("min_vx"),
                                     max_vyaw=g("max_vyaw"),
                                     min_vyaw=g("min_vyaw"), reach_tol=g("reach_tol"),
                                     lookahead=g("lookahead"))
        self.obst = ObstacleParams(
            body_front=g("body_front"), body_rear=g("body_rear"),
            body_half_width=g("body_half_width"), stop_distance=g("stop_distance"),
            lane_half_width=g("lane_half_width"), clearance_radius=g("clearance_radius"),
            z_min=g("obstacle_z_min"), z_max=g("obstacle_z_max"))
        self.stride = int(g("cloud_stride"))

        self._pose: Pose2D | None = None
        self._pose_t: float | None = None
        self._odom_frame = ""
        self._cloud_t: float | None = None
        self._front = self._near = 0
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
        self._ticks = 0

        self.cmd_pub = self.create_publisher(Twist, g("cmd_vel_topic"), 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(Odometry, g("odom_topic"), self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, g("cloud_topic"), self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_service(Trigger, "~/resume", self._srv_resume)
        self.create_timer(1.0 / max(1.0, g("rate_hz")), self._tick)

        mode = "DRY RUN -- nothing will be published" if self.dry_run \
            else f"LIVE -- publishing {g('cmd_vel_topic')}"
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
        self._pose = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
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
        self._front, self._near = classify(pts, pose, floor_z, self.obst)
        now = self._now()
        obst = obstacle_xy(pts, pose, floor_z, self.obst)
        self._memory.add(obst, now)
        if now - self._pruned_t > 1.0:
            self._memory.prune(now)
            self._pruned_t = now
        self._turn_near = ring_count(self._memory.points(), pose, self._turn_clearance, self.obst)
        if self.get_parameter("shape").value == "explore":
            # kept from before ~/start too: the first leg is planned at once
            self._obst_scans.append(obst)
            if isinstance(self.mission, ExploreMission):
                self._explore_scans += 1
                rays = (ray_points(pts, pose, floor_z, self.obst)
                        if self._explore_scans % 3 == 0 else ())     # CPU: every third scan
                self.mission.observe(floor_xy(pts, pose, floor_z), obst, (pose.x, pose.y), rays)
        self._cloud_t = now

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

    def _tick(self) -> None:
        now = self._now()
        oa, ca = self._ages(now)
        floor_z = median(self._floor_hist)
        sensors = (f"odom {_age(oa)}, lidar {_age(ca)}, floor "
                   f"{'?' if floor_z is None else f'{floor_z:.2f}'}, "
                   f"front {self._front}, near {self._near}")
        self._ticks += 1

        if self.mission is None:
            if self._ticks % 10 == 0:
                self._status(f"idle | {sensors}")
            return

        m = self.mission
        if isinstance(m, ExploreMission):
            obst = [p for scan in self._obst_scans for p in scan]
            cmd = m.step(now, self._pose, oa, ca, self._front, self._near, obst,
                         self._turn_near)
        else:
            cmd = m.step(now, self._pose, oa, ca, self._front, self._near, self._turn_near)
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
        if self._front >= k or self._near >= k:
            problems.append(f"path blocked (front={self._front}, near={self._near})")
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
            self.mission = Mission(to_world(start, local), start, now, self.follow, safety)
            what = f"{len(local)} waypoints"
        self._last_state = None
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
        self._publish(0.0, 0.0)
        res.success = True
        res.message = ("mission aborted (dry run: nothing was being published)"
                       if self.dry_run else "mission aborted, zero velocity sent")
        self.get_logger().warning(res.message)
        return res

    def _srv_resume(self, _req, res):
        if self.mission is None:
            res.success, res.message = False, "no mission"
            return res
        oa, ca = self._ages(self._now())
        ok, msg = self.mission.resume(oa, ca, self._front, self._near)
        res.success, res.message = ok, msg
        if ok:                                   # separate sites: see _tick
            self.get_logger().info(f"resume: {msg}")
        else:
            self.get_logger().warning(f"resume: {msg}")
        return res

    def destroy_node(self) -> bool:
        try:
            self._publish(0.0, 0.0)
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
