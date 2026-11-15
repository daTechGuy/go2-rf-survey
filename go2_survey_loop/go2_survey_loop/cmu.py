"""Adapter and readiness checks for the CMU Go2 autonomy stack (D5).
Pure, stdlib-only, no ROS, no clock.

The CMU stack (jizhang-cmu/autonomy_stack_go2, branch ``foxy-humble``) runs
Point-LIO, ``terrainAnalysis``, ``localPlanner`` (343 precomputed paths) and
``pathFollower`` on the robot:

* Pose:  ``/state_estimation`` (``nav_msgs/Odometry``, frame ``map``)
* Cloud: ``/registered_scan``  (``sensor_msgs/PointCloud2``, frame ``map``)
* Goal:  ``/way_point``        (``geometry_msgs/PointStamped``, frame ``map``)
* Stop:  ``/stop``             (``std_msgs/Int8``: bit 1 forward, 2 back,
                                4 left turn, 8 right turn -> ``15`` blocks all)

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
CMU_FRAME = "map"


@dataclass(frozen=True)
class CmuOutput:
    """What to publish to the CMU stack on this tick."""
    waypoint: tuple[float, float, float] | None
    stop_mask: int
    send_sport_stop: bool


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
             pose_z: float = 0.0, was_running: bool = False) -> tuple[Command, CmuOutput]:
    """Step ``mission`` in CMU waypoint mode and return ``(cmd, cmu_out)``.

    Front-lane obstacles and turn-sweep memory are delegated to CMU's
    ``localPlanner`` (so ``front=0, turn_near=0`` are passed to the inner
    follower), while ``near`` (the body clearance ring), sensor staleness,
    geofence, duration and stall checks remain enforced.

    If ``ExploreMission`` stalls during an explore leg while ``front`` is
    blocked (CMU's planner stopped short of an impassable obstacle), the
    unreachable frontier goal is marked spent and exploration re-plans
    instead of latching a permanent stall pause.
    """
    if isinstance(mission, ExploreMission):
        cmd = mission.step(now, pose, odom_age, cloud_age, 0, near, obstacles_xy, 0)
        if (cmd.state == State.PAUSED and mission.phase == "explore"
                and "stalled" in cmd.reason
                and front >= mission.safety.min_obstacle_points
                and near < mission.safety.min_obstacle_points
                and mission.leg is not None):
            mission._spent_goals.append(mission.leg.waypoints[-1])
            mission.legs_done += 1
            mission.leg = None
            mission._hold_since = None
            mission.state = State.RUNNING
            mission.reason = f"CMU planner blocked ahead (front={front}) -- choosing another way ({mission.progress()})"
            cmd = Command(0.0, 0.0, True, State.RUNNING, mission.reason)
    else:
        cmd = mission.step(now, pose, odom_age, cloud_age, 0, near, 0)

    target = active_target(mission) if cmd.state == State.RUNNING else None
    out = cmu_command(cmd.state, target, pose_z=pose_z, was_running=was_running)
    return cmd, out


def assess_cmu(snap: dict, sensing_only: bool = True) -> tuple[bool, list[str]]:
    """Judge a snapshot of the CMU stack's topics (guide 05, S1/S3).

    Keys in ``snap``:
        state_est_hz, state_est_frame,
        reg_scan_hz, reg_scan_frame,
        terrain_hz, path_hz,
        sport_pub_count (optional: publishers on /api/sport/request)
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
        if se_frame == CMU_FRAME:
            ok(f"/state_estimation frame is '{CMU_FRAME}'")
        else:
            fail(f"/state_estimation frame is {se_frame!r}, expected '{CMU_FRAME}'")

    rs_hz = float(snap.get("reg_scan_hz") or 0.0)
    if rs_hz >= REG_SCAN_MIN_HZ:
        ok(f"/registered_scan {rs_hz:.1f} Hz")
    else:
        fail(f"/registered_scan {rs_hz:.1f} Hz (want >= {REG_SCAN_MIN_HZ:.0f} Hz)")

    rs_frame = snap.get("reg_scan_frame")
    if rs_hz > 0 and rs_frame and rs_frame != CMU_FRAME:
        fail(f"/registered_scan frame is {rs_frame!r}, expected '{CMU_FRAME}'")

    ter_hz = float(snap.get("terrain_hz") or 0.0)
    if ter_hz >= TERRAIN_MIN_HZ:
        ok(f"/terrain_map {ter_hz:.1f} Hz")
    else:
        fail(f"/terrain_map {ter_hz:.1f} Hz (want >= {TERRAIN_MIN_HZ:.0f} Hz -- is terrainAnalysis up?)")

    path_hz = float(snap.get("path_hz") or 0.0)
    if path_hz >= PATH_MIN_HZ:
        ok(f"/path {path_hz:.1f} Hz (localPlanner active)")
    else:
        fail(f"/path {path_hz:.1f} Hz (want >= {PATH_MIN_HZ:.0f} Hz -- is localPlanner up, or still in 15s startup delay?)")

    sport_pubs = snap.get("sport_pub_count")
    if sport_pubs is not None:
        if sensing_only and sport_pubs > 0:
            warn(f"/api/sport/request has {sport_pubs} publisher(s) -- for S1 sensing-only, launch with is_real_robot:=false")
        elif not sensing_only and sport_pubs == 0:
            warn("/api/sport/request has 0 publishers -- for live motion, launch with is_real_robot:=true")

    return ready, lines
