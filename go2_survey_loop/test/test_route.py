"""Tests for routes that go round what blocks them (route.py). No ROS.

On the robot (2026-11-11) a rectangle stopped for a chair on its second side
and waited. Here the same: a closed loop with the 2D LiDAR model from the
frontier tests, a rectangle, and a chair on the way.

    python go2_survey_loop/test/test_route.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from go2_survey_loop.follower import FollowerParams, Mission, SafetyParams, State  # noqa: E402
from go2_survey_loop.geometry import Pose2D, plan_local, route_extent, to_world  # noqa: E402
from go2_survey_loop.obstacles import ObstacleParams, classify, ring_count  # noqa: E402
from go2_survey_loop.route import DetourParams, RouteMission  # noqa: E402
from test_explore_frontier import Lidar, box  # noqa: E402

FP = FollowerParams()
BODY = ObstacleParams()                 # a route's: stop 0.5 ahead, 0.55 ring
FRESH = 0.05
DT = 0.1
ROOM = box(-1.5, -1.5, 5.5, 3.5)        # 7 x 5 m, plenty of room round the route


def chair(x, y, half=0.25):
    return box(x - half, y - half, x + half, y + half)


def route_mission(length=3.0, width=2.0, detour=DetourParams(), cls=RouteMission):
    wps = to_world(Pose2D(0, 0, 0), plan_local("rectangle", length, width))
    safety = SafetyParams(max_duration_s=600.0,
                          geofence_m=route_extent("rectangle", length, width) + 1.0)
    if cls is Mission:
        return Mission(wps, Pose2D(0, 0, 0), 0.0, FP, safety)
    return RouteMission(wps, Pose2D(0, 0, 0), 0.0, FP, safety, detour, BODY)


def run(m, room, steps=6000, moving=None, yaw_gain=1.0):
    """Closed loop. ``moving(t)`` -> extra obstacle points at time t (a person);
    ``yaw_gain`` < 1: turns deliver less than commanded, like the sim's 0.5."""
    lidar_static = room
    pose, t, closest, log = Pose2D(0, 0, 0), 0.0, math.inf, []
    for i in range(steps):
        extra = moving(t) if moving else []
        pts = lidar_static + extra
        if i % 5 == 0:
            floor, hits = Lidar(pts).scan(pose)
            if hasattr(m, "observe"):
                m.observe(floor, hits)
        pts3 = [(x, y, 0.5) for x, y in pts if math.hypot(x - pose.x, y - pose.y) < 3.0]
        front, near = classify(pts3, pose, 0.0, BODY)
        turn_near = ring_count(pts, pose, 0.55, BODY)
        cmd = m.step(t, pose, FRESH, FRESH, front, near, turn_near)
        log.append((t, pose, cmd))
        if cmd.publish and cmd.state == State.RUNNING:
            yaw = pose.yaw + yaw_gain * cmd.vyaw * DT
            pose = Pose2D(pose.x + cmd.vx * DT * math.cos(yaw),
                          pose.y + cmd.vx * DT * math.sin(yaw), yaw)
        closest = min(closest, min(math.hypot(x - pose.x, y - pose.y) for x, y in pts))
        t += DT
        if cmd.state in (State.DONE, State.ABORTED) and not cmd.publish:
            break
        if cmd.state == State.PAUSED and i > 50 and all(c.state == State.PAUSED
                                                      for _, _, c in log[-300:]):
            break                                       # 30 s paused: it's waiting for us
    return pose, log, closest


class TestGoesRound(unittest.TestCase):
    """A chair in the middle of the second side."""

    @classmethod
    def setUpClass(cls):
        cls.obstacle = chair(3.0, 1.0)
        cls.m = route_mission()
        cls.end, cls.log, cls.closest = run(cls.m, ROOM + cls.obstacle)

    def test_finishes_the_route(self):
        self.assertEqual(self.m.state, State.DONE, self.m.reason)
        self.assertLess(math.hypot(self.end.x, self.end.y), FP.reach_tol + 0.1)

    def test_went_round(self):
        self.assertGreaterEqual(self.m.detours, 1)
        self.assertTrue(any("detour" in c.reason for _, _, c in self.log))

    def test_never_inside_the_clearance_ring(self):
        self.assertGreater(self.closest, BODY.clearance_radius)

    def test_waited_first(self):
        # blocked -> stood at least wait_s before the detour
        t_pause = next(t for t, _, c in self.log if c.state == State.PAUSED)
        t_det = next(t for t, _, c in self.log if "detour" in c.reason)
        self.assertGreaterEqual(t_det - t_pause, DetourParams().wait_s - DT)

    def test_sluggish_turning(self):
        # sim 2026-11-11 (YAW_GAIN 0.5): the detour ended on the corner, 0.75 m
        # from the chair, and a sluggish robot drifted inside the ring
        m = route_mission()
        end, log, closest = run(m, ROOM + self.obstacle, yaw_gain=0.5)
        self.assertEqual(m.state, State.DONE, m.reason)
        self.assertGreater(closest, BODY.clearance_radius)

    def test_detour_keeps_its_clearance_to_the_end(self):
        g = self.m.grid
        clear = g.clearance()
        self.assertTrue(self.m.detour_paths)
        for path in self.m.detour_paths:
            for x, y in path:                      # the last one included
                self.assertGreaterEqual(clear.get(g.cell_of(x, y), 3.0),
                                        DetourParams().path_clearance - 1e-6,
                                        f"detour waypoint {(x, y)}")

    def test_round_the_far_side_of_an_unseen_corner(self):
        # sim 2026-11-11: round the OUTSIDE of the chair; planned when only
        # its near side was seen, the last leg cut past its far corner 0.45 m
        # off. A cupboard inside the rectangle forces the outside way here.
        cupboard = box(1.6, 0.6, 2.2, 1.6)
        for cx, cy in [(3.0, 1.0), (3.1, 0.9), (2.95, 1.15)]:
            for gain in (1.0, 0.5):
                with self.subTest(chair=(cx, cy), yaw_gain=gain):
                    m = route_mission()
                    _, log, closest = run(m, ROOM + cupboard + chair(cx, cy), yaw_gain=gain)
                    self.assertGreater(closest, BODY.clearance_radius)
                    self.assertEqual(m.state, State.DONE, m.reason)

    def test_a_plain_route_just_waits(self):
        m = route_mission(cls=Mission)
        _, log, _ = run(m, ROOM + self.obstacle)
        self.assertEqual(log[-1][2].state, State.PAUSED)
        self.assertTrue(log[-1][2].reason.startswith("obstacle"))


class TestOtherCases(unittest.TestCase):
    def test_a_person_passing_is_waited_for_not_detoured(self):
        # someone on the second side walks off 1.5 s after the robot stops
        person = chair(3.0, 1.0, half=0.15)
        m = route_mission()
        seen = {"paused_at": None}

        def moving(t):
            if m.state == State.PAUSED and seen["paused_at"] is None:
                seen["paused_at"] = t
            gone = seen["paused_at"] is not None and t > seen["paused_at"] + 1.5
            return [] if gone else person

        _, log, _ = run(m, ROOM, moving=moving)
        self.assertIsNotNone(seen["paused_at"])                  # it did stop for them
        self.assertEqual(m.state, State.DONE, m.reason)
        self.assertEqual(m.detours, 0)                           # and carried on as it was
        self.assertTrue(any("cleared" in c.reason for _, _, c in log))

    def test_no_way_round_waits(self):
        # a wall of desks right across the room on the second side
        wall = [(x / 20, 1.0) for x in range(-30, 111)]
        m = route_mission()
        _, log, closest = run(m, ROOM + wall)
        self.assertEqual(m.state, State.PAUSED)
        self.assertIn("no clear way round", m.reason)
        self.assertGreater(closest, BODY.clearance_radius - 0.05)

    def test_disabled_is_a_plain_route(self):
        m = route_mission(detour=DetourParams(enabled=False))
        _, log, _ = run(m, ROOM + chair(3.0, 1.0))
        self.assertEqual(m.state, State.PAUSED)
        self.assertEqual(m.detours, 0)

    def test_something_beside_it_is_never_planned_round(self):
        m = route_mission()
        for _ in range(12):
            m.observe(*Lidar(ROOM).scan(Pose2D(0, 0, 0)))
        t = 0.0
        while t < 10.0:
            cmd = m.step(t, Pose2D(0, 0, 0), FRESH, FRESH, 9, 9)
            self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))
            t += DT
        self.assertEqual(m.detours, 0)
        self.assertEqual(m.state, State.PAUSED)

    def test_the_robots_own_size_route(self):
        # 2026-11-11's 1.5 x 1.0 m rectangle, chair 0.9 m along its second side:
        # no room inside to pass at 0.8 m, so round the outside or wait --
        # never into the ring
        m = route_mission(1.5, 1.0)
        _, log, closest = run(m, ROOM + chair(1.5, 0.9))
        self.assertGreater(closest, BODY.clearance_radius)
        self.assertIn(m.state, (State.DONE, State.PAUSED))


if __name__ == "__main__":
    unittest.main(verbosity=2)
