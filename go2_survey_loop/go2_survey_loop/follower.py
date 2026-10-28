"""Waypoint following and the mission state machine. Pure, no ROS, no clock.

``compute_command`` is the steering law: turn in place towards the next
corner if it is well off the nose, otherwise drive at it with a heading
correction, slowing as it closes. Hysteresis between the "turn" and "drive"
thresholds stops it chattering between the two near the boundary.

It steers at a point ``lookahead`` metres along the *route line* (previous
corner -> next corner) ahead of the robot, not at the corner itself. Aiming
at the corner leaves any sideways offset uncorrected: in the Gazebo test a
turn on the spot shifted the robot 12-14 cm sideways and it walked the whole
return leg off-centre, into the desks' clearance ring. Aiming along the line
pulls it back to the centre within about a metre. ``lookahead=0`` restores
plain corner-aiming.

``Mission`` owns everything that decides *whether* to move, in order of
precedence:

1. **terminal** -- DONE or ABORTED: hold zero briefly, then go silent. A
   silent /cmd_vel trips the bridge watchdog, which issues StopMove.
2. **abort** -- past ``max_duration_s``, or further than ``geofence_m`` from
   the start. Odometry that has wandered off is not something to keep
   following.
3. **pause** -- odometry stale, LiDAR stale (driving blind), or an obstacle.
   A pause holds zero velocity and *stays* paused until ``resume()`` -- or,
   only for obstacles and only if ``auto_resume_s`` is set, until the way has
   been clear that long. Nothing about a pause clearing itself should surprise
   the person holding the remote.
4. **run** -- steer to the current waypoint; advance when within
   ``reach_tol``; DONE after the last one.

Every call takes ``now`` and the ages of its inputs as arguments, so every
timeout above is unit-testable. An untestable stop condition is not a safety
feature.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from go2_survey_loop.geometry import Pose2D, wrap_angle


@dataclass(frozen=True)
class FollowerParams:
    max_vx: float = 0.30        # m/s -- deliberately half the bridge's ceiling
    max_vyaw: float = 1.00      # rad/s
    # rad/s while turning in place. Was 0.15. On 2026-10-28 the Go2 (firmware
    # 'mcf') only swayed for gentle turn requests -- from the remote too --
    # and stepped round properly above that: a 1.0 rad/s turn test delivered
    # 58%, 0.2-0.5 delivered 8-39%. So turns in place stay well above it.
    min_vyaw: float = 0.70
    k_lin: float = 0.6          # 1/s, slows inside ~0.5 m of a corner
    k_yaw: float = 1.2          # 1/s
    reach_tol: float = 0.20     # m
    turn_enter: float = 0.35    # rad (~20 deg): start turning in place
    turn_exit: float = 0.12     # rad (~7 deg): resume driving
    lookahead: float = 0.6      # m along the route line; 0 = aim at the corner


@dataclass(frozen=True)
class SafetyParams:
    odom_timeout_s: float = 0.5
    cloud_timeout_s: float = 1.0
    max_duration_s: float = 300.0
    geofence_m: float = 10.0
    min_obstacle_points: int = 5
    auto_resume_s: float = 0.0      # 0 = obstacles need an explicit resume
    tail_s: float = 1.0             # hold zero this long after DONE/ABORTED


class State(str, Enum):
    RUNNING = "running"
    PAUSED = "paused"
    DONE = "done"
    ABORTED = "aborted"


@dataclass(frozen=True)
class Command:
    vx: float
    vyaw: float
    publish: bool
    state: State
    reason: str = ""


def _clamp(v: float, limit: float) -> float:
    if not math.isfinite(v):
        return 0.0
    limit = abs(limit)
    return max(-limit, min(limit, v))


def aim_point(origin, target, pose: Pose2D, lookahead: float):
    """The point to steer at: ``lookahead`` metres along the line origin ->
    target beyond the robot's projection onto it, never past the target.
    With no origin, no lookahead, or a zero-length segment: the target."""
    if origin is None or lookahead <= 0:
        return target
    ax, ay = origin
    bx, by = target
    sx, sy = bx - ax, by - ay
    seg = math.hypot(sx, sy)
    if seg < 1e-6:
        return target
    along = ((pose.x - ax) * sx + (pose.y - ay) * sy) / seg
    s = max(0.0, along) + lookahead
    if s >= seg:
        return target
    return ax + sx * s / seg, ay + sy * s / seg


def compute_command(pose: Pose2D, target, p: FollowerParams, turning: bool, origin=None):
    """Steering law. Returns (vx, vyaw, turning, distance_to_target).

    ``origin`` is the previous corner; with it the robot steers along the
    route line (see aim_point), and still slows and stops on ``target``.
    A non-finite pose or target commands nothing. That is checked explicitly
    rather than left to how NaN happens to propagate through min/max.
    """
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, target[0], target[1])):
        return 0.0, 0.0, False, math.inf
    dist = math.hypot(target[0] - pose.x, target[1] - pose.y)
    ax, ay = aim_point(origin, target, pose, p.lookahead)
    err = wrap_angle(math.atan2(ay - pose.y, ax - pose.x) - pose.yaw)

    turning = abs(err) > (p.turn_exit if turning else p.turn_enter)
    vyaw = _clamp(p.k_yaw * err, p.max_vyaw)

    if turning:
        if abs(vyaw) < p.min_vyaw:
            vyaw = math.copysign(p.min_vyaw, err)
        return 0.0, vyaw, True, dist

    vx = min(p.max_vx, p.k_lin * dist) * max(0.0, math.cos(err))
    return vx, vyaw, False, dist


class Mission:
    def __init__(self, waypoints, start: Pose2D, now: float,
                 follow: FollowerParams = FollowerParams(),
                 safety: SafetyParams = SafetyParams()) -> None:
        if not waypoints:
            raise ValueError("mission needs at least one waypoint")
        self.waypoints = list(waypoints)
        self.start = start
        self.t0 = now
        self.follow = follow
        self.safety = safety
        self.index = 0
        self.state = State.RUNNING
        self.reason = "started"
        self._turning = False
        self._turn_dir = 0.0            # +1 CCW / -1 CW once a turn starts
        self._ended_at: float | None = None
        self._clear_since: float | None = None

    # ---------------------------------------------------------------- queries

    @property
    def target(self):
        return self.waypoints[min(self.index, len(self.waypoints) - 1)]

    @property
    def origin(self):
        """Start of the current route line: the previous corner, or the start."""
        i = min(self.index, len(self.waypoints) - 1)
        return self.waypoints[i - 1] if i > 0 else (self.start.x, self.start.y)

    def progress(self) -> str:
        return f"waypoint {min(self.index + 1, len(self.waypoints))}/{len(self.waypoints)}"

    def _blocked(self, front: int, near: int) -> bool:
        k = self.safety.min_obstacle_points
        return front >= k or near >= k

    # -------------------------------------------------------------- transitions

    def _end(self, state: State, reason: str, now: float) -> Command:
        if self.state not in (State.DONE, State.ABORTED):
            self.state = state
            self.reason = reason
            self._ended_at = now
        return Command(0.0, 0.0, True, self.state, self.reason)

    def _pause(self, reason: str) -> Command:
        self.state = State.PAUSED
        self.reason = reason
        self._clear_since = None
        self._turning = False
        self._turn_dir = 0.0
        return Command(0.0, 0.0, True, State.PAUSED, reason)

    def _latch_turn(self, pose: Pose2D, vyaw: float) -> float:
        """Commit to one turn direction near the +/-pi seam.

        A target directly behind the robot -- the far end of an out-and-back --
        sits at +/-pi of heading error, where a few hundredths of a radian of
        odometry noise decide whether the short way round is left or right.
        Without a latch the turn could reverse tick to tick. Once a turn has
        started, keep its direction while the error is still near the seam.
        """
        if not self._turning:
            self._turn_dir = 0.0
            return vyaw
        if self._turn_dir == 0.0:
            self._turn_dir = math.copysign(1.0, vyaw) if vyaw else 0.0
            return vyaw
        tx, ty = aim_point(self.origin, self.target, pose, self.follow.lookahead)
        err = wrap_angle(math.atan2(ty - pose.y, tx - pose.x) - pose.yaw)
        if abs(err) > math.pi - self.follow.turn_enter \
                and math.copysign(1.0, vyaw) != self._turn_dir:
            return -vyaw
        return vyaw

    def abort(self, now: float, reason: str = "stopped by operator") -> Command:
        return self._end(State.ABORTED, reason, now)

    def resume(self, odom_age: float, cloud_age: float, front: int, near: int):
        """Operator resume. Returns (ok, message). Refuses unless it is
        currently safe to move -- resuming into an obstacle is not a resume."""
        if self.state != State.PAUSED:
            return False, f"not paused (state={self.state.value})"
        if odom_age > self.safety.odom_timeout_s:
            return False, f"odometry stale ({odom_age:.2f}s)"
        if cloud_age > self.safety.cloud_timeout_s:
            return False, f"lidar stale ({cloud_age:.2f}s)"
        if self._blocked(front, near):
            return False, f"still blocked (front={front}, near={near})"
        self.state = State.RUNNING
        self.reason = "resumed"
        self._turning = False
        self._turn_dir = 0.0
        return True, "resumed"

    # ------------------------------------------------------------------- step

    def step(self, now: float, pose: Pose2D | None, odom_age: float,
             cloud_age: float, front: int, near: int) -> Command:
        s = self.safety

        # 1. terminal
        if self.state in (State.DONE, State.ABORTED):
            publish = self._ended_at is not None and now - self._ended_at < s.tail_s
            return Command(0.0, 0.0, publish, self.state, self.reason)

        # Non-finite odometry is treated exactly like missing odometry.
        if pose is not None and not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
            pose = None

        # 2. abort
        if now - self.t0 > s.max_duration_s:
            return self._end(State.ABORTED, f"exceeded {s.max_duration_s:.0f}s limit", now)
        if pose is not None and math.hypot(pose.x - self.start.x,
                                           pose.y - self.start.y) > s.geofence_m:
            return self._end(State.ABORTED, f"left the {s.geofence_m:.1f} m geofence", now)

        blocked = self._blocked(front, near)
        fresh = pose is not None and odom_age <= s.odom_timeout_s \
            and cloud_age <= s.cloud_timeout_s

        # 3. pause
        if self.state == State.RUNNING:
            if pose is None or odom_age > s.odom_timeout_s:
                return self._pause("odometry stale")
            if cloud_age > s.cloud_timeout_s:
                return self._pause("lidar stale -- driving blind")
            if blocked:
                return self._pause(f"obstacle (front={front}, near={near})")

        if self.state == State.PAUSED:
            if s.auto_resume_s > 0 and self.reason.startswith("obstacle") \
                    and fresh and not blocked:
                if self._clear_since is None:
                    self._clear_since = now
                if now - self._clear_since >= s.auto_resume_s:
                    self.state, self.reason = State.RUNNING, "auto-resumed"
                    self._turning = False
            else:
                self._clear_since = None
            if self.state == State.PAUSED:
                return Command(0.0, 0.0, True, State.PAUSED, self.reason)

        # 4. run
        vx, vyaw, self._turning, dist = compute_command(
            pose, self.target, self.follow, self._turning, self.origin)
        while dist < self.follow.reach_tol:
            self.index += 1
            self._turning = False
            self._turn_dir = 0.0
            if self.index >= len(self.waypoints):
                return self._end(State.DONE, "loop complete", now)
            vx, vyaw, self._turning, dist = compute_command(
                pose, self.target, self.follow, self._turning, self.origin)

        vyaw = self._latch_turn(pose, vyaw)
        self.reason = self.progress()
        return Command(vx, vyaw, True, State.RUNNING, self.reason)
