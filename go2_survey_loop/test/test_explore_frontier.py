"""Tests for frontier exploring: the coverage grid and the closed loop. No ROS.

The closed-loop runs use a small 2D LiDAR: rays from the robot stop at the
first wall or desk, so what is behind stays unseen -- the thing frontier
exploring is for. The hall here has rows of desks like sim/worlds hall.world,
scaled down so the test runs in seconds.

    python go2_survey_loop/test/test_explore_frontier.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_loop import coverage  # noqa: E402
from go2_survey_loop.coverage import FREE, OCC, UNKNOWN, Grid  # noqa: E402
from go2_survey_loop.explore import ExploreMission, ExploreParams  # noqa: E402
from go2_survey_loop.follower import FollowerParams, SafetyParams, State  # noqa: E402
from go2_survey_loop.geometry import Pose2D  # noqa: E402
from go2_survey_loop.obstacles import ObstacleParams, classify, ring_count  # noqa: E402

FP = FollowerParams()
BODY = ObstacleParams(clearance_radius=0.70, stop_distance=0.70, lane_half_width=0.45)
FRESH = 0.05
DT = 0.1


def segment(x0, y0, x1, y1, step=0.05):
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def box(x0, y0, x1, y1):
    return (segment(x0, y0, x1, y0) + segment(x1, y0, x1, y1)
            + segment(x1, y1, x0, y1) + segment(x0, y1, x0, y0))


def hall():
    """14 x 10 m, three rows of desks either side of a 2.5 m centre aisle."""
    pts = box(-1.5, -5.0, 12.5, 5.0)
    for x in (3.0, 6.0, 9.0):
        for y0 in (1.25, -2.45):                       # desks 1.2 m wide, 0.6 deep
            pts += box(x - 0.3, y0, x + 0.3, y0 + 1.2)
        for y0 in (2.75, -3.95):
            pts += box(x - 0.3, y0, x + 0.3, y0 + 1.2)
    return pts


class Lidar:
    """Rays from the robot, stopping at the first obstacle point."""

    def __init__(self, obstacles, cell=0.1, rays=90, max_range=8.0):
        self.cell, self.rays, self.max_range = cell, rays, max_range
        self.occ = {(round(x / cell), round(y / cell)) for x, y in obstacles}

    def scan(self, pose):
        floor, hits = [], []
        c = self.cell
        for k in range(self.rays):
            a = pose.yaw + 2 * math.pi * k / self.rays
            ca, sa = math.cos(a), math.sin(a)
            r = 0.45                                   # beyond its own body
            while r < self.max_range:
                x, y = pose.x + r * ca, pose.y + r * sa
                if (round(x / c), round(y / c)) in self.occ:
                    hits.append((x, y))
                    break
                floor.append((x, y))
                r += 0.2
        return floor, hits


def mission(legs=12, radius=8.0, strategy="frontier", **safety):
    s = dict(max_duration_s=900.0, geofence_m=radius + 1.0)
    s.update(safety)
    return ExploreMission(Pose2D(0, 0, 0), 0.0, FP, SafetyParams(**s),
                          ExploreParams(legs=legs, radius=radius, strategy=strategy), BODY)


def run(m, room, steps=12000, yaw_gain=1.0):
    """Closed loop with the LiDAR model; observe every 0.5 s, as a coarse
    stand-in for loop_node feeding every scan."""
    lidar = Lidar(room)
    pose, t, log, closest = Pose2D(0, 0, 0), 0.0, [], math.inf
    floor, hits = lidar.scan(pose)
    for i in range(steps):
        if i % 5 == 0:
            floor, hits = lidar.scan(pose)
            m.observe(floor, hits)
        pts3 = [(x, y, 0.5) for x, y in hits]
        front, near = classify(pts3, pose, 0.0, BODY)
        turn_near = ring_count(room, pose, 0.70, BODY)
        cmd = m.step(t, pose, FRESH, FRESH, front, near, hits, turn_near)
        log.append((t, pose, cmd, m.phase))
        if cmd.publish and cmd.state == State.RUNNING:
            yaw = pose.yaw + yaw_gain * cmd.vyaw * DT
            pose = Pose2D(pose.x + cmd.vx * DT * math.cos(yaw),
                          pose.y + cmd.vx * DT * math.sin(yaw), yaw)
        closest = min(closest, min(math.hypot(x - pose.x, y - pose.y) for x, y in room))
        t += DT
        if cmd.state in (State.DONE, State.ABORTED) and not cmd.publish:
            break
    return pose, log, closest


def walked(log, phase=None):
    d = 0.0
    for (_, a, _, ph), (_, b, _, _) in zip(log, log[1:]):
        if phase is None or ph == phase:
            d += math.hypot(b.x - a.x, b.y - a.y)
    return d


# --------------------------------------------------------------------------
class TestGrid(unittest.TestCase):
    def test_states(self):
        g = Grid(0, 0, 2.0, 0.25)
        g.mark(floor_xy=[(0.1, 0.1)])
        self.assertEqual(g.state(g.cell_of(0.1, 0.1)), FREE)
        self.assertEqual(g.state(g.cell_of(1.0, 1.0)), UNKNOWN)
        g.mark(obstacle_xy=[(1.0, 1.0)])
        self.assertEqual(g.state(g.cell_of(1.0, 1.0)), UNKNOWN)     # one stray hit
        g.mark(floor_xy=[(1.0, 1.0)], obstacle_xy=[(1.01, 1.01)])
        self.assertEqual(g.state(g.cell_of(1.0, 1.0)), OCC)         # obstacle beats floor

    def test_outside_is_ignored(self):
        g = Grid(0, 0, 1.0)
        g.mark(floor_xy=[(50, 50)], obstacle_xy=[(-50, 0)] * 3)
        self.assertEqual(g.counts()[:2], (0, 0))

    def test_clearance(self):
        g = Grid(0, 0, 3.0, 0.25)
        g.mark(obstacle_xy=[(1.0, 0.0)] * 2)
        clear = g.clearance()
        self.assertEqual(clear[g.cell_of(1.0, 0.0)], 0.0)
        self.assertAlmostEqual(clear[g.cell_of(0.0, 0.0)], 1.0, delta=0.01)

    def test_frontier_is_the_edge_of_what_was_seen(self):
        g = Grid(0, 0, 3.0, 0.25)
        g.mark(floor_xy=[(x / 10, y / 10) for x in range(-10, 11) for y in range(-10, 11)])
        patches = g.frontiers(min_cells=4)
        self.assertEqual(len(patches), 1)                            # a ring round the patch
        for c in patches[0]:
            x, y = g.centre(c)
            self.assertGreater(max(abs(x), abs(y)), 0.7)

    def test_path_keeps_clear_and_stays_on_seen_floor(self):
        g = Grid(0, 0, 4.0, 0.25)
        g.mark(floor_xy=[(x / 10, y / 10) for x in range(-35, 36) for y in range(-35, 36)])
        wall = [(1.5, y / 10) for y in range(-35, 16)]               # gap above y = 1.5
        g.mark(obstacle_xy=wall * 2)
        clear = g.clearance()
        here, goal = g.cell_of(0, 0), g.cell_of(3.0, 0)
        cost, parent = coverage.costs_from(g, here, clear, 0.75)
        self.assertIn(goal, cost)
        cells = coverage.path_to(parent, goal)
        self.assertTrue(all(clear.get(c, 3.0) >= 0.75 for c in cells[1:]))
        self.assertTrue(any(g.centre(c)[1] > 1.5 for c in cells))   # around, not through
        wps = coverage.simplify(g, clear, cells, 0.75)
        self.assertLess(len(wps), len(cells))
        pts = [(0.0, 0.0)] + wps
        for a, b in zip(pts, pts[1:]):
            self.assertTrue(coverage.line_clear(g, clear, a, b, 0.75))

    def test_unknown_is_never_planned_through(self):
        g = Grid(0, 0, 3.0, 0.25)
        g.mark(floor_xy=[(x / 10, 0.0) for x in range(-5, 6)])       # a seen strip only
        cost, _ = coverage.costs_from(g, g.cell_of(0, 0), g.clearance(), 0.0)
        self.assertNotIn(g.cell_of(0.0, 1.0), cost)

    def test_line_of_sight(self):
        g = Grid(0, 0, 3.0, 0.25)
        g.mark(obstacle_xy=[(1.0, y / 10) for y in range(-5, 6)] * 2)   # a desk side-on
        a, behind, beside = g.cell_of(0, 0), g.cell_of(2.0, 0), g.cell_of(0, 2.0)
        self.assertFalse(coverage.sees(g, a, behind))
        self.assertTrue(coverage.sees(g, a, beside))          # unknown doesn't block

    def test_rays_mark_the_floor_they_cross_at_desk_height(self):
        g = Grid(0, 0, 8.0, 0.25)
        # a wall 2 m up, 7 m away: the ray is under 1.0 m for its first ~2.7 m
        g.mark_rays((0.0, 0.0), [(7.0, 0.0, 2.0)], sensor_h=0.35, band=(0.10, 1.00))
        self.assertEqual(g.state(g.cell_of(1.0, 0.0)), FREE)
        self.assertEqual(g.state(g.cell_of(2.5, 0.0)), FREE)
        self.assertEqual(g.state(g.cell_of(5.0, 0.0)), UNKNOWN)   # ray above desk height
        # a desk-high return 4 m away: clear all the way, not the hit itself
        g.mark_rays((0.0, 0.0), [(0.0, 4.0, 0.5)])
        self.assertEqual(g.state(g.cell_of(0.0, 3.5)), FREE)
        self.assertEqual(g.state(g.cell_of(0.0, 4.0)), UNKNOWN)

    def test_truncate(self):
        out = coverage.truncate([(2.0, 0.0), (2.0, 3.0)], (0.0, 0.0), 3.0)
        self.assertEqual(out[0], (2.0, 0.0))
        self.assertAlmostEqual(out[-1][1], 1.0)


class TestFrontierClosedLoop(unittest.TestCase):
    """In the hall: deeper than greedy, home by a short way, never close."""

    @classmethod
    def setUpClass(cls):
        cls.room = hall()
        cls.m = mission()
        cls.end, cls.log, cls.closest = run(cls.m, cls.room)
        cls.g = mission(strategy="greedy")
        cls.g_end, cls.g_log, _ = run(cls.g, cls.room)

    def test_finishes_at_home(self):
        self.assertEqual(self.m.state, State.DONE, self.m.reason)
        self.assertLess(math.hypot(self.end.x, self.end.y), FP.reach_tol + 0.15)

    def test_never_close_to_desks_or_walls(self):
        self.assertGreater(self.closest, 0.6)

    def test_sees_the_hall_for_much_less_walking(self):
        # in a hall the LiDAR can see down most of, the win is efficiency: about
        # what greedy sees, then home -- instead of wandering out its legs
        seen, g_seen = self.m.grid.counts()[0], self.g.grid.counts()[0]
        self.assertGreaterEqual(seen, g_seen)                    # 3836 vs 3690
        self.assertLess(walked(self.log), 0.8 * walked(self.g_log))   # 20 m vs 29 m

    def test_reaches_into_the_room(self):
        self.assertGreater(max(p.x for _, p, _, _ in self.log), 6.0)

    def test_home_is_a_direct_route(self):
        # not a retrace of every leg: close to the straight line home
        at = next(p for _, p, _, ph in self.log if ph == "home")
        self.assertLess(walked(self.log, "home"), 1.5 * math.hypot(at.x, at.y) + 0.5)

    def test_sluggish_turning(self):
        m = mission(legs=6)
        end, _, closest = run(m, self.room, yaw_gain=0.5)
        self.assertEqual(m.state, State.DONE, m.reason)
        self.assertGreater(closest, 0.6)


class TestStuck(unittest.TestCase):
    def test_something_inside_the_ring_for_too_long_ends_the_run(self):
        m = mission()
        lidar = Lidar(hall())
        for _ in range(12):
            m.observe(*lidar.scan(Pose2D(0, 0, 0)))
        t, cmd = 0.0, None
        while t < 25.0:
            cmd = m.step(t, Pose2D(0, 0, 0), FRESH, FRESH, 0, 9, hall())
            self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))
            if cmd.state == State.ABORTED:
                break
            t += DT
        self.assertEqual(cmd.state, State.ABORTED)
        self.assertIn("stuck", cmd.reason)
        self.assertGreaterEqual(t, m.p.stuck_s - 1e-6)

    def test_a_person_stepping_away_in_time_is_not_stuck(self):
        m = mission()
        lidar = Lidar(hall())
        for _ in range(12):
            m.observe(*lidar.scan(Pose2D(0, 0, 0)))
        for i in range(100):                                   # 10 s beside it
            m.step(i * DT, Pose2D(0, 0, 0), FRESH, FRESH, 0, 9, hall())
        cmd = m.step(10.1, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, hall())
        self.assertEqual(cmd.state, State.RUNNING)


class TestNothingLeftToSee(unittest.TestCase):
    def test_room_seen_whole_from_the_start_is_done_not_a_failure(self):
        # sim furnished room: from the start the LiDAR saw nearly all of it,
        # and the first version ended "nowhere open to explore" (aborted)
        room = box(-1.5, -2.5, 4.0, 2.5)
        m = mission()
        _, log, _ = run(m, room, steps=600)
        self.assertEqual(m.state, State.DONE, m.reason)
        self.assertLess(walked(log), 3.0)


class TestFrontierStart(unittest.TestCase):
    def test_looks_around_before_planning(self):
        m = mission()
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, hall())
        self.assertEqual(cmd.state, State.RUNNING)          # not a pause to resume from
        self.assertIn("looking around", cmd.reason)
        self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))
        self.assertIsNone(m.leg)

    def test_no_room_to_turn_falls_back_to_straight_legs(self):
        m = mission()
        lidar = Lidar(hall())
        for _ in range(12):
            m.observe(*lidar.scan(Pose2D(0, 0, 0)))
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, hall(), turn_near=20)
        self.assertIsNotNone(m.leg)
        tx, ty = m.leg.waypoints[-1]
        self.assertLess(abs(math.atan2(ty, tx)), m.p.straight_turn + 1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
