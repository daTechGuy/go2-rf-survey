"""Adapter and readiness checks for the CMU Go2 autonomy stack (D5).
Pure, stdlib-only, no ROS, no clock.

The CMU stack (jizhang-cmu/autonomy_stack_go2, branch ``foxy-humble``) runs
Point-LIO, ``terrainAnalysis``, ``localPlanner`` (343 precomputed paths) and
``pathFollower`` on the robot:

* Pose:  ``/state_estimation`` (``nav_msgs/Odometry``, frame ``camera_init``
  -- Point-LIO's own name; the launch adds an identity ``map`` -> ``camera_init``
  TF. It is the pose of the **L1 LiDAR**, ``sensorOffsetX`` = 0.3 m ahead of
  the body centre: ``vehicle_pose`` undoes that, exactly as CMU's nodes do.)
* Cloud: ``/registered_scan``  (``sensor_msgs/PointCloud2``, frame ``camera_init``)
* Goal:  ``/way_point``        (``geometry_msgs/PointStamped``; the frame is ignored)
* Stop:  ``/stop``             (``std_msgs/Int8``: bit 1 forward, 2 back,
                                4 left turn, 8 right turn -> ``15`` blocks all)

Read from the source (``pathFollower.cpp``), not assumed: inside
``slowDwnDisThre`` (0.75 m) of its goal pathFollower slows in proportion
(0.2 m/s at 0.5 m -- the Go2's forward sway zone) and stops at
``stopDisThre`` (0.3 m). Aimed straight at a waypoint the robot therefore
never comes within ``reach_tol`` (0.2 m) of it and the mission pauses
"stalled". So the goal sent is a **carrot**: the waypoint pushed
``carrot_m`` further along the leg, walked through at full speed, and the
waypoint counts as reached once the robot has **crossed** the line through
it, square to the leg (``passed``). The final waypoint ends the mission the
same way: DONE sends ``/stop 15`` + ``StopMove`` as it crosses home.

When ``loop_node`` runs with ``planner:="cmu"``, our mission state machine
(``Mission``, ``RouteMission``, ``ExploreMission``) keeps every one of its
safety layers -- fresh odometry and LiDAR, body clearance ring, geofence, time
limit, stall detection, operator ``~/start`` / ``~/stop`` / ``~/resume`` and
the handheld remote trigger -- while delegating local obstacle steering to
CMU's ``localPlanner`` via ``/way_point`` and ``/stop``. Because ``pathFollower``
publishes ``Move`` straight to ``/api/sport/request`` (bypassing our bridge),
every transition out of ``RUNNING`` emits both ``/stop = 15`` and a one-shot
``StopMove`` (sport API 1003).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from go2_survey_loop.explore import ExploreMission
from go2_survey_loop.follower import Command, Mission, State
from go2_survey_loop.geometry import Pose2D
from go2_survey_loop.route import RouteMission

STOP_FREE = 0
STOP_ALL = 15               # bits 1 (fwd) | 2 (back) | 4 (left) | 8 (right)
SPORT_STOP_MOVE_ID = 1003   # unitree_sdk2 sport_api.hpp StopMove

STATE_EST_MIN_HZ = 20.0
REG_SCAN_MIN_HZ = 3.0
TERRAIN_MIN_HZ = 3.0
PATH_MIN_HZ = 2.0
# Point-LIO stamps /state_estimation and /registered_scan "camera_init"
# (laserMapping.cpp); system_real_robot.launch publishes map -> camera_init as
# an identity, so either name is the same frame.
CMU_FRAMES = ("camera_init", "map")
CMU_FRAME = "map"               # what we stamp on /way_point (localPlanner ignores it)
SENSOR_OFFSET_X = 0.3           # system_real_robot.launch sensorOffsetX: L1 ahead of centre
# pathFollower walks at full speed only while its goal is > slowDwnDisThre
# (0.75 m) away; the carrot keeps it beyond that until the waypoint is crossed
CARROT_M = 0.8
PASS_LATERAL_M = 0.6            # crossed the waypoint's line within this of it -> reached
MAX_LIVE_SPEED = 0.5            # m/s: refuse a live stack faster than this
SWAY_SPEED = 0.3                # m/s: slower walk requests only sway the Go2 (2026-10-31)


@dataclass(frozen=True)
class CmuOutput:
    """What to publish to the CMU stack on this tick."""
    waypoint: tuple[float, float, float] | None
    stop_mask: int
    send_sport_stop: bool


def vehicle_pose(sensor: Pose2D, offset_x: float = SENSOR_OFFSET_X,
                 offset_y: float = 0.0) -> Pose2D:
    """Body centre from the L1 LiDAR's pose on ``/state_estimation``.

    The same transform as CMU's ``localPlanner`` / ``pathFollower``
    ``odometryHandler`` (``sensorOffsetX``/``Y``), so our waypoints, reach
    checks, body box and clearance ring use the point CMU steers.
    """
    c, s = math.cos(sensor.yaw), math.sin(sensor.yaw)
    return Pose2D(sensor.x - c * offset_x + s * offset_y,
                  sensor.y - s * offset_x - c * offset_y,
                  sensor.yaw)


def driving_mission(mission) -> Mission | None:
    """The inner ``Mission`` that holds the current waypoint list, or None."""
    if mission is None:
        return None
    if isinstance(mission, ExploreMission):
        return mission.leg
    if isinstance(mission, RouteMission):
        return mission.route
    return mission


def active_target(mission: Mission | RouteMission | ExploreMission | None) -> tuple[float, float] | None:
    """Current (x, y) waypoint of ``mission``, or None when between legs or idle."""
    if mission is None:
        return None
    if isinstance(mission, ExploreMission):
        if mission.leg is None or mission.leg.state != State.RUNNING:
            return None
        return mission.leg.target
    if isinstance(mission, RouteMission):
        return mission.route.target
    return mission.target


def _unit(dx: float, dy: float):
    n = math.hypot(dx, dy)
    if n < 1e-6 or not math.isfinite(n):
        return None
    return dx / n, dy / n


def carrot(origin, target, pose_xy, extend: float):
    """``target`` pushed ``extend`` metres further along origin -> target.

    A zero-length leg (origin on the target) uses the direction from the
    robot instead; with neither, or ``extend <= 0``, the target itself.
    """
    if extend <= 0:
        return target
    u = _unit(target[0] - origin[0], target[1] - origin[1]) if origin is not None else None
    if u is None and pose_xy is not None:
        u = _unit(target[0] - pose_xy[0], target[1] - pose_xy[1])
    if u is None:
        return target
    return target[0] + u[0] * extend, target[1] + u[1] * extend


def passed(origin, target, pose_xy, lateral_max: float) -> bool:
    """Has the robot crossed the line through ``target`` square to the leg
    origin -> target, within ``lateral_max`` of the target? False for a
    zero-length leg or ``lateral_max <= 0``."""
    if origin is None or pose_xy is None or lateral_max <= 0:
        return False
    sx, sy = target[0] - origin[0], target[1] - origin[1]
    seg = math.hypot(sx, sy)
    if seg < 1e-6:
        return False
    rx, ry = pose_xy[0] - target[0], pose_xy[1] - target[1]
    beyond = (rx * sx + ry * sy) / seg           # > 0: past the target along the leg
    lateral = abs(rx * sy - ry * sx) / seg
    return beyond >= 0.0 and lateral <= lateral_max


def cmu_command(state: State, target: tuple[float, float] | None,
                pose_z: float = 0.0, was_running: bool = False) -> CmuOutput:
    """Translate mission ``state`` and ``target`` into ``/way_point`` and ``/stop``.

    ``send_sport_stop`` is True on the tick the mission leaves ``RUNNING`` so the
    caller also sends a one-shot ``StopMove`` to ``/api/sport/request``.
    """
    z = float(pose_z) if math.isfinite(pose_z) else 0.0
    if (state == State.RUNNING and target is not None
            and math.isfinite(target[0]) and math.isfinite(target[1])):
        return CmuOutput(
            waypoint=(float(target[0]), float(target[1]), z),
            stop_mask=STOP_FREE,
            send_sport_stop=False,
        )
    return CmuOutput(
        waypoint=None,
        stop_mask=STOP_ALL,
        send_sport_stop=bool(was_running),
    )


def step_cmu(mission: Mission | RouteMission | ExploreMission,
             now: float, pose: Pose2D | None, odom_age: float, cloud_age: float,
             front: int, near: int, obstacles_xy=(),
             pose_z: float = 0.0, was_running: bool = False,
             carrot_m: float = 0.0, pass_lateral: float = 0.0) -> tuple[Command, CmuOutput]:
    """Step ``mission`` in CMU waypoint mode and return ``(cmd, cmu_out)``.

    Front-lane obstacles and turn-sweep memory are delegated to CMU's
    ``localPlanner`` (so ``front=0, turn_near=0`` are passed to the inner
    follower), while ``near`` (the body clearance ring), sensor staleness,
    geofence, duration and stall checks remain enforced.

    ``pass_lateral`` > 0: a waypoint the robot has crossed (``passed``) counts
    as reached even if it never came within ``reach_tol`` -- CMU swerves.
    ``carrot_m`` > 0: the goal sent is the waypoint pushed that far along the
    leg (``carrot``), so pathFollower does not slow down short of it.
    ``was_running``: a waypoint was sent last tick (``StopMove`` edge).

    If ``ExploreMission`` stalls during an explore leg while ``front`` is
    blocked (CMU's planner stopped short of an impassable obstacle), the
    unreachable frontier goal is marked spent and exploration re-plans
    instead of latching a permanent stall pause.
    """
    inner = driving_mission(mission)
    if (pass_lateral > 0 and inner is not None and inner.state == State.RUNNING
            and pose is not None
            and all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw))
            and odom_age <= inner.safety.odom_timeout_s
            and passed(inner.origin, inner.target, (pose.x, pose.y), pass_lateral)):
        inner.pass_waypoint(now)

    if isinstance(mission, ExploreMission):
        cmd = mission.step(now, pose, odom_age, cloud_age, 0, near, obstacles_xy, 0)
        if (cmd.state == State.PAUSED and mission.phase == "explore"
                and "stalled" in cmd.reason
                and front >= mission.safety.min_obstacle_points
                and near < mission.safety.min_obstacle_points
                and mission.leg is not None):
            mission._spent_goals.append(mission.leg.waypoints[-1])
            if getattr(mission, "_leg_goal", None) is not None:
                mission._spent_goals.append(mission._leg_goal)
            mission.legs_done += 1
            mission.leg = None
            mission._hold_since = None
            mission.state = State.RUNNING
            mission.reason = f"CMU planner blocked ahead (front={front}) -- choosing another way ({mission.progress()})"
            cmd = Command(0.0, 0.0, True, State.RUNNING, mission.reason)
    else:
        cmd = mission.step(now, pose, odom_age, cloud_age, 0, near, 0)

    target = active_target(mission) if cmd.state == State.RUNNING else None
    if target is not None and carrot_m > 0:
        inner = driving_mission(mission)
        pxy = (pose.x, pose.y) if pose is not None else None
        target = carrot(inner.origin if inner is not None else None, target, pxy, carrot_m)
    out = cmu_command(cmd.state, target, pose_z=pose_z, was_running=was_running)
    return cmd, out


def assess_cmu(snap: dict, sensing_only: bool = True) -> tuple[bool, list[str]]:
    """Judge a snapshot of the CMU stack's topics and parameters (guide 05, S1/S3).

    Keys in ``snap``:
        state_est_hz, state_est_frame,
        reg_scan_hz, reg_scan_frame,
        terrain_hz, path_hz,
        waypoint_sub_count, stop_sub_count, joy_pub_count, sport_pub_count,
        autonomy_mode, autonomy_speed, max_speed, is_real_robot (all optional:
        checked when present; live mode fails on unsafe values).
    """
    lines: list[str] = []
    ready = True

    def ok(msg: str) -> None:
        lines.append(f"OK   {msg}")

    def fail(msg: str) -> None:
        nonlocal ready
        lines.append(f"FAIL {msg}")
        ready = False

    def warn(msg: str) -> None:
        lines.append(f"WARN {msg}")

    se_hz = float(snap.get("state_est_hz") or 0.0)
    if se_hz >= STATE_EST_MIN_HZ:
        ok(f"/state_estimation {se_hz:.0f} Hz")
    else:
        fail(f"/state_estimation {se_hz:.1f} Hz (want >= {STATE_EST_MIN_HZ:.0f} Hz -- is Point-LIO up?)")

    se_frame = snap.get("state_est_frame")
    if se_hz > 0:
        if se_frame in CMU_FRAMES:
            ok(f"/state_estimation frame is '{se_frame}'")
        else:
            fail(f"/state_estimation frame is {se_frame!r}, expected one of {CMU_FRAMES}")

    rs_hz = float(snap.get("reg_scan_hz") or 0.0)
    if rs_hz >= REG_SCAN_MIN_HZ:
        ok(f"/registered_scan {rs_hz:.1f} Hz")
    else:
        fail(f"/registered_scan {rs_hz:.1f} Hz (want >= {REG_SCAN_MIN_HZ:.0f} Hz)")

    rs_frame = snap.get("reg_scan_frame")
    if rs_hz > 0 and rs_frame and rs_frame not in CMU_FRAMES:
        fail(f"/registered_scan frame is {rs_frame!r}, expected one of {CMU_FRAMES}")

    ter_hz = float(snap.get("terrain_hz") or 0.0)
    if ter_hz >= TERRAIN_MIN_HZ:
        ok(f"/terrain_map {ter_hz:.1f} Hz")
    else:
        fail(f"/terrain_map {ter_hz:.1f} Hz (want >= {TERRAIN_MIN_HZ:.0f} Hz -- is terrainAnalysis up?)")

    path_hz = float(snap.get("path_hz") or 0.0)
    if path_hz >= PATH_MIN_HZ:
        ok(f"/path {path_hz:.1f} Hz (localPlanner active)")
    else:
        fail(f"/path {path_hz:.1f} Hz (want >= {PATH_MIN_HZ:.0f} Hz -- is localPlanner up?)")

    wp_subs = snap.get("waypoint_sub_count")
    if wp_subs is not None:
        if wp_subs >= 1:
            ok(f"/way_point has {wp_subs} subscriber(s)")
        else:
            fail("/way_point has 0 subscribers (localPlanner not listening)")

    stop_subs = snap.get("stop_sub_count")
    if stop_subs is not None:
        if stop_subs >= 1:
            ok(f"/stop has {stop_subs} subscriber(s)")
        else:
            fail("/stop has 0 subscribers (pathFollower not listening)")

    # system_real_robot.launch starts joy_node; any /joy message with L2
    # unpressed (axes[2] > -0.1) flips autonomyMode to false in both nodes
    # and sets joySpeed = 0 (localPlanner.cpp:236, pathFollower.cpp:183).
    joy_pubs = snap.get("joy_pub_count")
    if joy_pubs:
        warn(f"/joy has {joy_pubs} publisher(s) -- an idle joystick resets autonomyMode to false")

    sport_pubs = snap.get("sport_pub_count")
    if sport_pubs is not None:
        if sensing_only and sport_pubs > 0:
            warn(f"/api/sport/request has {sport_pubs} publisher(s) -- for S1 sensing-only, launch with is_real_robot:=false")
        elif not sensing_only and sport_pubs == 0:
            fail("/api/sport/request has 0 publishers -- for live motion, launch with is_real_robot:=true")

    if not sensing_only:
        am = snap.get("autonomy_mode")
        if am is False:
            fail("autonomyMode is false -- launch local_planner with autonomyMode:=true (default is false: /way_point is ignored)")
        elif am is True:
            ok("autonomyMode is true")

        real = snap.get("is_real_robot")
        if real is False:
            fail("pathFollower is_real_robot is false -- it will not publish to /api/sport/request")

        for key, val in (("autonomySpeed", snap.get("autonomy_speed")),
                         ("maxSpeed", snap.get("max_speed"))):
            if val is None:
                continue
            v = float(val)
            if v > MAX_LIVE_SPEED:
                fail(f"{key} is {v:.2f} m/s (> {MAX_LIVE_SPEED:.2f} m/s -- launch with {key}:={SWAY_SPEED})")
            elif v < SWAY_SPEED - 1e-3:
                warn(f"{key} is {v:.2f} m/s (< {SWAY_SPEED:.2f} m/s -- Go2 only sways below {SWAY_SPEED} m/s)")
            else:
                ok(f"{key} is {v:.2f} m/s")

    return ready, lines

