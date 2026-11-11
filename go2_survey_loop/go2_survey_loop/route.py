"""A route that goes round what blocks it. Pure, no ROS, no clock.

A plain route (follower.Mission) stops in front of anything in its way and
waits for the operator. On the robot (2026-11-11) a rectangle in a classroom
stopped for a chair on its second side, though there was room to step
round it. ``RouteMission`` keeps the same route and the same safety layers,
plus a coverage grid of what the LiDARs have seen (coverage.py, as when
exploring). When the way ahead stays blocked for ``wait_s`` -- a person may
just be passing -- it plans a clear path round to the waypoint and splices
it into the route:

* only for something **ahead** (the front lane). Something inside the
  clearance ring still stops everything, as before;
* the path keeps ``path_clearance`` from every obstacle cell, over floor
  that was actually seen, all the way: it ends on the clear cell by the
  waypoint (within one cell), not on the waypoint; one that is itself too
  close to the obstacle is skipped and the next one aimed for (``max_skip``);
* **re-checked while walking it**: when it was planned, only the near side
  of the obstacle had been seen. Every ``recheck_s`` the rest of the detour
  is checked against the map as it is now; if newly seen obstacle comes
  within the clearance, it re-plans from where it stands (sim 2026-11-11:
  the last leg cut past the chair's unseen far corner, 0.45 m off);
* at most ``max_detours``; each no longer than ``max_ratio`` times the
  straight way plus ``slack_m``, and inside the geofence;
* no clear way round: it stays paused and looks again every ``retry_s``.
  The remote works as always.

Every step of a detour goes through the same Mission.step: front-lane stop,
clearance ring, turn check, stall check, geofence, time limit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from go2_survey_loop import coverage
from go2_survey_loop.follower import Command, FollowerParams, Mission, SafetyParams, State
from go2_survey_loop.geometry import Pose2D
from go2_survey_loop.obstacles import ObstacleParams


@dataclass(frozen=True)
class DetourParams:
    enabled: bool = True
    wait_s: float = 3.0             # blocked this long before going round
    # "the way cleared" only after this long with nothing ahead: robot
    # 2026-11-11, a chair at the edge of the front lane flickered 2-8 points
    # per scan (threshold 5); one clear reading resumed it, it stepped and
    # stopped again, over and over, and never went round
    clear_s: float = 1.0
    retry_s: float = 2.0            # no way round: look again this often
    # The route's 0.55 m ring + up to 0.18 m a 0.25 m cell can hide + wobble.
    # Exploring uses 0.95 with its 0.70 ring.
    path_clearance: float = 0.80
    recheck_s: float = 1.0          # the rest of a detour against the newest map
    recheck_skip_m: float = 0.5     # ...from this far ahead (it stands near the obstacle)
    max_replans: int = 8            # per detour
    max_detours: int = 4
    # waypoints a detour may skip (one inside the obstacle's clearance, e.g. a
    # chair by a corner). More would quietly drop the route: a wall of desks
    # across it once sent the test robot straight home.
    max_skip: int = 1
    max_ratio: float = 3.0
    slack_m: float = 1.0
    grid_cell: float = 0.25
    lidar_h: float = 0.35           # m above the floor: where rays start


def path_length(start, waypoints) -> float:
    d, (px, py) = 0.0, start
    for x, y in waypoints:
        d += math.hypot(x - px, y - py)
        px, py = x, y
    return d


class RouteMission:
    """Mission's interface (step, abort, resume, state, reason, progress),
    plus ``observe`` for the grid, as ExploreMission."""

    def __init__(self, waypoints, start: Pose2D, now: float,
                 follow: FollowerParams = FollowerParams(),
                 safety: SafetyParams = SafetyParams(),
                 detour: DetourParams = DetourParams(),
                 body: ObstacleParams = ObstacleParams()) -> None:
        self.route = Mission(waypoints, start, now, follow, safety)
        self.start, self.follow, self.safety, self.p, self.body = start, follow, safety, detour, body
        self.grid = coverage.Grid(start.x, start.y, safety.geofence_m, detour.grid_cell)
        self.observations = 0
        self.detours = 0
        self.replans = 0
        self.detour_paths: list = []        # every planned detour path (logs, tests)
        self._blocked_since: float | None = None
        self._last_blocked: float | None = None     # last reading with something ahead
        self._tried_at: float | None = None
        # the detour being walked: index of its last waypoint in the route's
        # list, the route waypoint it stands in for, its re-plans so far
        self._detour = None
        self._checked_at = -math.inf

    # ------------------------------------------------------------ delegation
    @property
    def state(self) -> State:
        return self.route.state

    @property
    def reason(self) -> str:
        return self.route.reason

    @property
    def waypoints(self):
        return self.route.waypoints

    def progress(self) -> str:
        return self.route.progress() + (" (detour)" if self._detour else "")

    def abort(self, now: float, reason: str = "stopped by operator") -> Command:
        return self.route.abort(now, reason)

    def resume(self, odom_age: float, cloud_age: float, front: int, near: int):
        self._blocked_since = self._tried_at = None
        return self.route.resume(odom_age, cloud_age, front, near)

    def observe(self, floor_xy, obstacle_xy, origin=None, rays=()) -> None:
        """One scan into the grid (loop_node, as for exploring)."""
        self.grid.mark(floor_xy, obstacle_xy)
        if origin is not None and rays:
            self.grid.mark_rays(origin, rays, self.p.lidar_h, (self.body.z_min, self.body.z_max))
        self.observations += 1

    # ----------------------------------------------------------------- plan
    def _path_to(self, pose: Pose2D, cost, parent, clear, start_xy, target):
        """Clear path from the robot to the cell by ``target`` (within one
        cell), or None if there is none, or it is too long or leaves the
        fence. ``start_xy``: where the straight way would have started."""
        g, p = self.grid, self.p
        goal = g.cell_of(*target)
        if goal is None:
            return None
        near = [c for c in cost if max(abs(c[0] - goal[0]), abs(c[1] - goal[1])) <= 1]
        if not near:
            return None
        best = min(near, key=cost.get)
        # ends on the clear cell by the waypoint, not the waypoint itself:
        # sim 2026-11-11, a corner 0.75 m from the chair
        path = coverage.simplify(g, clear, coverage.path_to(parent, best), p.path_clearance)
        straight = path_length(start_xy, [target])
        if path_length((pose.x, pose.y), path) > p.max_ratio * straight + p.slack_m:
            return None
        fence = self.safety.geofence_m - 0.3
        if any(math.hypot(x - self.start.x, y - self.start.y) > fence for x, y in path):
            return None
        return path

    def _search(self, pose: Pose2D):
        g = self.grid
        here = g.cell_of(pose.x, pose.y)
        if here is None:
            return None
        clear = g.clearance()
        cost, parent = coverage.costs_from(g, here, clear, self.p.path_clearance)
        return cost, parent, clear

    def plan_detour(self, pose: Pose2D):
        """(new waypoint list, index to carry on from, message) or None:
        a clear path from here to the current waypoint -- or, if that one
        is too close to the obstacle, the first later one it can reach."""
        found = self._search(pose)
        if found is None:
            return None
        cost, parent, clear = found
        wps, i = self.route.waypoints, self.route.index
        for j in range(i, min(len(wps), i + self.p.max_skip + 1)):
            path = self._path_to(pose, cost, parent, clear, (pose.x, pose.y), wps[j])
            if path is None:
                continue
            self.detour_paths.append(path)
            new = wps[:i] + [(pose.x, pose.y)] + path + wps[j + 1:]
            self._detour = {"end": i + len(path), "target": wps[j], "replans": 0}
            skipped = j - i
            msg = (f"detour {self.detours + 1}: round an obstacle, {len(path)} waypoint(s), "
                   f"{path_length((pose.x, pose.y), path):.1f} m"
                   + (f", skipping {skipped} blocked waypoint(s)" if skipped else ""))
            return new, i + 1, msg            # the robot's own spot starts the line
        return None

    def _rest_clear(self, pose: Pose2D) -> bool:
        """The rest of the detour -- from ``recheck_skip_m`` ahead of the
        robot to its end -- still keeps the clearance on the newest map."""
        g, p = self.grid, self.p
        clear = g.clearance()
        i, end = self.route.index, self._detour["end"]
        pts = [(pose.x, pose.y)] + self.route.waypoints[i:end + 1]
        walked = 0.0
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            seg = math.hypot(bx - ax, by - ay)
            n = max(1, int(seg / 0.1))
            for k in range(n + 1):
                if walked + seg * k / n < p.recheck_skip_m:
                    continue
                c = g.cell_of(ax + (bx - ax) * k / n, ay + (by - ay) * k / n)
                if c is None or g.state(c) == coverage.OCC \
                        or clear.get(c, 3.0) < p.path_clearance - 1e-6:
                    return False
            walked += seg
        return True

    def _recheck(self, now: float, pose: Pose2D):
        """Walking a detour: re-plan its rest if the map now says it's too
        close to something. Returns a Command to send instead, or None."""
        d = self._detour
        if self.route.index > d["end"]:
            self._detour = None                     # walked it: back on the route
            return None
        if now - self._checked_at < self.p.recheck_s:
            return None
        self._checked_at = now
        if self._rest_clear(pose):
            return None
        found = self._search(pose) if d["replans"] < self.p.max_replans else None
        path = None
        if found is not None:
            cost, parent, clear = found
            path = self._path_to(pose, cost, parent, clear, (pose.x, pose.y), d["target"])
        if path is None:
            # carry on as planned: the front lane and the ring still stop it
            return None
        wps, i = self.route.waypoints, self.route.index
        self.detour_paths.append(path)
        self.replans += 1
        new = wps[:i] + [(pose.x, pose.y)] + path + wps[d["end"] + 1:]
        self._detour = {"end": i + len(path), "target": d["target"], "replans": d["replans"] + 1}
        msg = f"detour {self.detours}: re-planned with what it now sees, {len(path)} waypoint(s)"
        self.route.reroute(new, i + 1, msg)
        return None

    # ----------------------------------------------------------------- step
    def step(self, now: float, pose: Pose2D | None, odom_age: float,
             cloud_age: float, front: int, near: int, turn_near: int = 0) -> Command:
        fresh = pose is not None and odom_age <= self.safety.odom_timeout_s \
            and cloud_age <= self.safety.cloud_timeout_s
        if self._detour is not None and fresh and self.route.state == State.RUNNING:
            self._recheck(now, pose)
        cmd = self.route.step(now, pose, odom_age, cloud_age, front, near, turn_near)
        k = self.safety.min_obstacle_points
        if not (self.p.enabled and cmd.state == State.PAUSED
                and self.route.reason.startswith("obstacle")):
            if cmd.state != State.PAUSED:
                self._blocked_since = self._tried_at = None
            return cmd
        if not fresh or near >= k:
            # something beside it, or blind: not a moment to plan anything
            self._blocked_since = None
            return cmd
        if self._blocked_since is None:
            self._blocked_since = self._last_blocked = now
        if front >= k:
            self._last_blocked = now
        if now - self._blocked_since < self.p.wait_s:
            return cmd
        if now - self._last_blocked >= self.p.clear_s:
            # nothing ahead for clear_s: it moved off -- carry on as it was
            ok, _ = self.route.resume(odom_age, cloud_age, front, near)
            self._blocked_since = self._last_blocked = self._tried_at = None
            if ok:
                self.route.reason = "the way cleared -- carrying on"
            return Command(0.0, 0.0, True, self.route.state, self.route.reason)
        if self.detours >= self.p.max_detours:
            self.route.reason = (f"obstacle (front={front}) -- {self.detours} detours used, "
                                 "waiting for the operator")
            return Command(0.0, 0.0, True, State.PAUSED, self.route.reason)
        if self._tried_at is not None and now - self._tried_at < self.p.retry_s:
            return Command(0.0, 0.0, True, State.PAUSED, self.route.reason)
        self._tried_at = now
        plan = self.plan_detour(pose)
        if plan is None:
            self.route.reason = f"obstacle (front={front}) -- no clear way round, waiting"
            return Command(0.0, 0.0, True, State.PAUSED, self.route.reason)
        new, index, msg = plan
        self.detours += 1
        self.route.reroute(new, index, msg)
        self._blocked_since = self._tried_at = None
        self._checked_at = now
        return Command(0.0, 0.0, True, State.RUNNING, msg)
