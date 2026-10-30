"""Greedy exploration: walk to open space it hasn't visited, then come home.
Pure, no ROS, no clock.

Why: a classroom's layout changes as desks and chairs move, so a fixed route
planned for one layout walks into the next. The map comes from whatever the
LiDAR sees on the way; this module only decides where to walk next, from what
the LiDAR sees *now*.

Each **leg** starts with the robot standing still. ``choose_leg`` looks along
``headings`` directions around it. For each it measures how far a corridor as
wide as the robot is clear, and plans a leg of up to ``leg_max`` that stops
``margin`` short of the first obstacle -- outside the stop zone, so a leg that
ends next to a desk does not trip the obstacle stop. Each candidate is scored:

    novelty (how far its end is from everywhere already walked, capped)
    + length_weight * leg length
    - turn_cost * |turn needed|

and the best one is walked with the ordinary route follower (follower.py
``Mission``, one waypoint), so the steering, the brisk turns and every stop
condition are the ones already tested on the robot.

When something blocks a leg the follower pauses as usual. Here, instead of
waiting for the operator, the robot stands for ``replan_hold_s`` and then
picks another way from the new scan -- but only once nothing is inside the
clearance ring (someone beside it). After ``legs`` legs, or when no direction
is open, it walks **home** by retracing the points where its legs began: the
straight lines between them were walked already, so they were passable. On
the way home an obstacle pauses it like any route, until the remote resumes.

The time limit and the geofence (``radius`` around the start) apply to the
whole exploration, home included. Everything stops exactly as a route does.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from go2_survey_loop.follower import (
    Command, FollowerParams, Mission, SafetyParams, State,
)
from go2_survey_loop.geometry import Pose2D
from go2_survey_loop.obstacles import ObstacleParams


@dataclass(frozen=True)
class ExploreParams:
    legs: int = 8                   # explore legs before heading home
    leg_max: float = 1.5            # m
    leg_min: float = 0.6            # m: a shorter opening isn't worth walking
    headings: int = 16              # candidate directions, evenly spaced
    corridor_half: float = 0.60     # m: wider than the 0.55 m clearance ring,
                                    # or a leg along a wall stalls on it
    margin: float = 1.0             # m centre-to-obstacle kept at a leg's end
                                    # (nose 0.35 + stop 0.5 + 0.15 slack)
    end_clearance: float = 0.8      # m from a leg's end to anything, any direction:
                                    # it must be able to turn there (ring 0.55).
                                    # Sim 2026-10-31: legs ending by a chair stalled
    radius: float = 4.0             # m: legs end within this of the start
    novelty_cap: float = 3.0        # m
    length_weight: float = 0.3
    turn_cost: float = 0.3          # per radian of turn
    replan_hold_s: float = 1.0      # stand this long after a block, then re-plan
    home_skip_m: float = 0.6        # on the way home, a turn point this close
                                    # with something just beyond it counts as reached
    max_range: float = 8.0          # m: further points don't matter for one leg
    trail_step: float = 0.3         # m between remembered trail points


@dataclass(frozen=True)
class Leg:
    target: tuple
    heading_rel: float              # rad, relative to the robot's heading
    length: float
    free: float                     # m clear along that heading
    score: float


def robot_frame(points_xy, pose: Pose2D, body: ObstacleParams, max_range: float):
    """Odom-frame (x, y) -> robot-frame (forward, left), without the robot's
    own footprint (its legs are in every scan) and beyond ``max_range``."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    cells = {}                      # one point per 5 cm cell: a real scan has
    r2 = max_range * max_range      # thousands, planning needs the shape
    for x, y in points_xy:
        dx, dy = x - pose.x, y - pose.y
        if dx * dx + dy * dy > r2:
            continue
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        if -body.body_rear <= xr <= body.body_front and abs(yr) <= body.body_half_width:
            continue
        cells.setdefault((round(xr / 0.05), round(yr / 0.05)), (xr, yr))
    return list(cells.values())


def free_distance(obst_robot, heading_rel: float, corridor_half: float,
                  max_range: float) -> float:
    """Distance from the robot's centre to the first obstacle point inside a
    corridor of half-width ``corridor_half`` along ``heading_rel``."""
    c, s = math.cos(heading_rel), math.sin(heading_rel)
    best = max_range
    for xr, yr in obst_robot:
        along = c * xr + s * yr
        if along <= 0.0 or along >= best:
            continue
        if abs(-s * xr + c * yr) <= corridor_half:
            best = along
    return best


def choose_leg(pose: Pose2D, obstacles_xy, visited, start: Pose2D,
               p: ExploreParams = ExploreParams(),
               body: ObstacleParams = ObstacleParams()):
    """The best next leg, or None if no direction is open enough."""
    obst = robot_frame(obstacles_xy, pose, body, p.max_range)
    best = None
    c2 = p.end_clearance * p.end_clearance
    for k in range(p.headings):
        rel = -math.pi + 2.0 * math.pi * k / p.headings
        free = free_distance(obst, rel, p.corridor_half, p.max_range)
        longest = min(p.leg_max, free - p.margin)
        # the longest leg that ends somewhere it can turn round, in 0.3 m steps
        length, ex, ey = longest, None, None
        while length >= p.leg_min:
            fx, fy = length * math.cos(rel), length * math.sin(rel)     # robot frame
            if all((xr - fx) ** 2 + (yr - fy) ** 2 > c2 for xr, yr in obst):
                a = pose.yaw + rel
                ex, ey = pose.x + length * math.cos(a), pose.y + length * math.sin(a)
                break
            length -= 0.3
        if ex is None:
            continue
        if math.hypot(ex - start.x, ey - start.y) > p.radius:
            continue
        novelty = min([math.hypot(ex - vx, ey - vy) for vx, vy in visited]
                      + [p.novelty_cap])
        score = novelty + p.length_weight * length - p.turn_cost * abs(rel)
        if best is None or score > best.score:
            best = Leg((ex, ey), rel, length, free, score)
    return best


class ExploreMission:
    """Same interface as follower.Mission: ``step`` -> Command, ``abort``,
    ``resume``, ``state``, ``reason``, ``progress()``. ``step`` also takes the
    latest obstacle points (odom-frame x, y in the obstacle height band)."""

    def __init__(self, start: Pose2D, now: float,
                 follow: FollowerParams = FollowerParams(),
                 safety: SafetyParams = SafetyParams(),
                 explore: ExploreParams = ExploreParams(),
                 body: ObstacleParams = ObstacleParams()) -> None:
        self.start, self.t0 = start, now
        self.follow, self.safety, self.p, self.body = follow, safety, explore, body
        self.phase = "explore"
        self.legs_done = 0
        self.leg_starts: list = []
        self.visited = [(start.x, start.y)]
        self.leg: Mission | None = None
        self.state = State.RUNNING
        self.reason = "started"
        self._hold_since: float | None = None
        self._ended_at: float | None = None
        self._latched = False           # paused for a reason only resume() clears

    # ------------------------------------------------------------- queries

    def progress(self) -> str:
        if self.phase == "home":
            return "home " + (self.leg.progress() if self.leg else "")
        return f"explore leg {min(self.legs_done + 1, self.p.legs)}/{self.p.legs}"

    def _leg_safety(self) -> SafetyParams:
        # the leg's own time limit and geofence are never the binding ones:
        # this class applies both to the whole exploration
        return replace(self.safety, max_duration_s=1e9, geofence_m=1e9,
                       auto_resume_s=self.safety.auto_resume_s if self.phase == "home" else 0.0)

    # --------------------------------------------------------- transitions

    def _end(self, state: State, reason: str, now: float) -> Command:
        if self.state not in (State.DONE, State.ABORTED):
            self.state, self.reason, self._ended_at = state, reason, now
        return Command(0.0, 0.0, True, self.state, self.reason)

    def _hold(self, reason: str, latch: bool) -> Command:
        self.state, self.reason = State.PAUSED, reason
        self._latched = self._latched or latch
        return Command(0.0, 0.0, True, State.PAUSED, reason)

    def abort(self, now: float, reason: str = "stopped by operator") -> Command:
        return self._end(State.ABORTED, reason, now)

    def resume(self, odom_age: float, cloud_age: float, front: int, near: int):
        if self.state != State.PAUSED:
            return False, f"not paused (state={self.state.value})"
        if self.leg is not None and self.leg.state == State.PAUSED:
            ok, msg = self.leg.resume(odom_age, cloud_age, front, near)
            if not ok:
                return ok, msg
        elif odom_age > self.safety.odom_timeout_s or cloud_age > self.safety.cloud_timeout_s:
            return False, "sensors stale"
        self.state, self.reason, self._latched = State.RUNNING, "resumed", False
        self._hold_since = None
        return True, "resumed"

    def _go_home(self, pose: Pose2D, now: float) -> None:
        self.phase = "home"
        wps = list(reversed(self.leg_starts)) or [(self.start.x, self.start.y)]
        self.leg = Mission(wps, pose, now, self.follow, self._leg_safety())

    # ---------------------------------------------------------------- step

    def step(self, now: float, pose: Pose2D | None, odom_age: float,
             cloud_age: float, front: int, near: int, obstacles_xy=()) -> Command:
        s = self.safety
        if self.state in (State.DONE, State.ABORTED):
            publish = self._ended_at is not None and now - self._ended_at < s.tail_s
            return Command(0.0, 0.0, publish, self.state, self.reason)

        if pose is not None and not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
            pose = None
        if now - self.t0 > s.max_duration_s:
            return self._end(State.ABORTED, f"exceeded {s.max_duration_s:.0f}s limit", now)
        if pose is not None and math.hypot(pose.x - self.start.x,
                                           pose.y - self.start.y) > s.geofence_m:
            return self._end(State.ABORTED, f"left the {s.geofence_m:.1f} m geofence", now)

        if pose is not None:
            lx, ly = self.visited[-1]
            if math.hypot(pose.x - lx, pose.y - ly) >= self.p.trail_step:
                self.visited.append((pose.x, pose.y))

        # between legs: plan the next one
        if self.leg is None:
            if self.state == State.PAUSED and self._latched:
                return Command(0.0, 0.0, True, State.PAUSED, self.reason)
            if pose is None or odom_age > s.odom_timeout_s:
                return self._hold("odometry stale", latch=True)
            if cloud_age > s.cloud_timeout_s:
                return self._hold("lidar stale -- driving blind", latch=True)
            if near >= s.min_obstacle_points:
                return self._hold(f"something beside me (near={near}) -- waiting", latch=False)
            if not obstacles_xy:
                # an empty picture is no picture: a real room always has walls
                return self._hold("no LiDAR picture to plan from -- waiting", latch=False)
            self.state = State.RUNNING
            if self.legs_done < self.p.legs:
                leg = choose_leg(pose, obstacles_xy, self.visited, self.start, self.p, self.body)
                if leg is not None:
                    self.leg_starts.append((pose.x, pose.y))
                    self.leg = Mission([leg.target], pose, now, self.follow,
                                       self._leg_safety())
                    self.reason = (f"leg {self.legs_done + 1}: {leg.length:.1f} m at "
                                   f"{math.degrees(leg.heading_rel):+.0f} deg")
            if self.leg is None:
                if not self.leg_starts:
                    return self._end(State.ABORTED, "nowhere open to explore", now)
                self._go_home(pose, now)

        cmd = self.leg.step(now, pose, odom_age, cloud_age, front, near)

        if cmd.state == State.DONE:
            if self.phase == "home":
                return self._end(State.DONE, f"explored {self.legs_done} legs, home", now)
            self.legs_done += 1
            self.leg, self._hold_since = None, None
            return Command(0.0, 0.0, True, State.RUNNING, self.progress())

        if cmd.state == State.ABORTED:
            return self._end(State.ABORTED, cmd.reason, now)

        if cmd.state == State.PAUSED:
            if self.phase == "explore" and cmd.reason.startswith("obstacle"):
                # blocked mid-leg: stand, then choose another way
                if self._hold_since is None:
                    self._hold_since = now
                if now - self._hold_since >= self.p.replan_hold_s \
                        and near < s.min_obstacle_points:
                    self.legs_done += 1
                    self.leg, self._hold_since = None, None
                    self.state = State.RUNNING
                    return Command(0.0, 0.0, True, State.RUNNING,
                                   f"blocked -- choosing another way ({self.progress()})")
                self.state, self.reason = State.PAUSED, f"{cmd.reason} -- re-planning"
                return Command(0.0, 0.0, True, State.PAUSED, self.reason)
            if self.phase == "home" and cmd.reason.startswith("obstacle") and pose is not None:
                # Home retraces where legs began, but approaches each from a
                # new direction: something just beyond one can fill the front
                # lane before it is reached. Close enough counts.
                tx, ty = self.leg.target
                if math.hypot(tx - pose.x, ty - pose.y) <= self.p.home_skip_m \
                        and self.leg.skip_waypoint(now, near):
                    if self.leg.state == State.DONE:
                        return self._end(State.DONE, f"explored {self.legs_done} legs, home", now)
                    self.state = State.RUNNING
                    return Command(0.0, 0.0, True, State.RUNNING,
                                   f"home: turn point next to an obstacle, skipped ({self.progress()})")
            # stale sensors, or blocked on the way home: the leg holds its own
            # pause (operator resume, or its auto-resume); mirror it
            self.state, self.reason = State.PAUSED, cmd.reason
            return cmd

        self.state, self._hold_since = State.RUNNING, None
        self.reason = self.progress()
        return Command(cmd.vx, cmd.vyaw, True, State.RUNNING, self.reason)
