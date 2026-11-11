"""Tests for the obstacle stop check. No ROS, no numpy.

    python go2_survey_loop/test/test_obstacles.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_loop.geometry import Pose2D  # noqa: E402
from go2_survey_loop.obstacles import (  # noqa: E402
    ObstacleMemory, ObstacleParams, classify, floor_from_scan, median, obstacle_xy,
    ring_count,
)

FLOOR = 0.0
P = ObstacleParams()
ORIGIN = Pose2D(0.0, 0.0, 0.0)


def pt(x, y, h=0.5):
    return (x, y, FLOOR + h)


class TestFrontLane(unittest.TestCase):
    def test_point_ahead_in_lane(self):
        self.assertEqual(classify([pt(0.6, 0.0)], ORIGIN, FLOOR)[0], 1)

    def test_point_beyond_stop_distance_ignored(self):
        self.assertEqual(classify([pt(2.0, 0.0)], ORIGIN, FLOOR), (0, 0))

    def test_point_beside_lane_not_front(self):
        self.assertEqual(classify([pt(0.6, 0.8)], ORIGIN, FLOOR)[0], 0)

    def test_point_behind_not_front(self):
        self.assertEqual(classify([pt(-0.6, 0.0)], ORIGIN, FLOOR)[0], 0)


class TestRing(unittest.TestCase):
    def test_point_beside_robot_is_near(self):
        self.assertEqual(classify([pt(0.0, 0.45)], ORIGIN, FLOOR), (0, 1))

    def test_point_behind_close_is_near(self):
        self.assertEqual(classify([pt(-0.45, 0.0)], ORIGIN, FLOOR), (0, 1))


class TestMemory(unittest.TestCase):
    """The turn check's memory: things now behind the robot still count."""

    def test_remembers_what_it_can_no_longer_see(self):
        mem = ObstacleMemory(keep_s=10.0)
        mem.add([(-0.6, 0.0)], now=0.0)       # seen earlier, now 0.6 m behind
        mem.add([], now=5.0)                  # current scan: nothing
        self.assertEqual(ring_count(mem.points(), ORIGIN, 0.70, P), 1)

    def test_forgets_after_keep_s(self):
        mem = ObstacleMemory(keep_s=10.0)
        mem.add([(-0.6, 0.0)], now=0.0)
        mem.prune(now=11.0)
        self.assertEqual(len(mem), 0)

    def test_seen_again_stays(self):
        mem = ObstacleMemory(keep_s=10.0)
        mem.add([(-0.6, 0.0)], now=0.0)
        mem.add([(-0.6, 0.0)], now=9.0)
        mem.prune(now=11.0)
        self.assertEqual(len(mem), 1)

    def test_one_point_per_cell(self):
        mem = ObstacleMemory(cell=0.05)
        mem.add([(1.0, 1.0), (1.01, 1.01), (1.2, 1.0)], now=0.0)
        self.assertEqual(len(mem), 2)

    def test_ring_count_ignores_own_body_and_far_points(self):
        pts = [(0.2, 0.1), (-0.3, -0.1),      # inside the footprint: its own legs
               (0.0, 0.65), (2.0, 0.0)]       # beside it (counts); far (doesn't)
        self.assertEqual(ring_count(pts, ORIGIN, 0.70, P), 1)

    def test_obstacle_xy_band_and_range(self):
        pts = [pt(1.0, 0.0, 0.05), pt(1.0, 0.0, 0.5), pt(1.0, 0.0, 1.5), pt(20.0, 0.0, 0.5)]
        self.assertEqual(obstacle_xy(pts, ORIGIN, FLOOR, P), [(1.0, 0.0)])

    def test_obstacle_xy_drops_its_own_legs(self):
        # sim 2026-10-31: remembered leg points boxed it in wherever it walked
        legs = [pt(0.30, 0.18), pt(-0.38, -0.18), pt(0.42, 0.25)]   # incl. a swinging foot
        self.assertEqual(obstacle_xy(legs, ORIGIN, FLOOR, P), [])
        pose = Pose2D(2.0, 1.0, math.pi / 2)                         # same, rotated
        rot = [pt(2.0 - 0.18, 1.0 + 0.30), pt(2.0 + 0.18, 1.0 - 0.38)]
        self.assertEqual(obstacle_xy(rot, pose, FLOOR, P), [])

    def test_walking_leaves_no_ghosts_behind(self):
        mem = ObstacleMemory()
        for i in range(20):                                          # walks 2 m along x
            pose = Pose2D(0.1 * i, 0.0, 0.0)
            legs = [pt(pose.x + 0.3, 0.18), pt(pose.x - 0.38, -0.18)]
            mem.add(obstacle_xy(legs, pose, FLOOR, P), now=0.1 * i)
        self.assertEqual(ring_count(mem.points(), Pose2D(1.9, 0.0, 0.0), 0.70, P), 0)


class TestFilters(unittest.TestCase):
    def test_own_body_ignored(self):
        legs = [pt(0.2, 0.1, 0.2), pt(-0.3, -0.15, 0.3), pt(0.0, 0.0, 0.5)]
        self.assertEqual(classify(legs, ORIGIN, FLOOR), (0, 0))

    def test_floor_ignored(self):
        self.assertEqual(classify([pt(0.6, 0.0, 0.02)], ORIGIN, FLOOR), (0, 0))

    def test_ceiling_ignored(self):
        self.assertEqual(classify([pt(0.6, 0.0, 2.5)], ORIGIN, FLOOR), (0, 0))

    def test_floor_offset_applied(self):
        # same physical point; floor at z=-0.3 makes it 0.5 m tall, not 0.2
        self.assertEqual(classify([(0.6, 0.0, 0.2)], ORIGIN, -0.3)[0], 1)
        self.assertEqual(classify([(0.6, 0.0, 0.2)], ORIGIN, 0.15)[0], 0)


class TestPose(unittest.TestCase):
    def test_rotated_robot(self):
        # robot facing +y: a point 0.6 m along +y is ahead of it
        facing_left = Pose2D(0.0, 0.0, math.pi / 2)
        self.assertEqual(classify([pt(0.0, 0.6)], facing_left, FLOOR)[0], 1)
        self.assertEqual(classify([pt(0.6, 0.0)], facing_left, FLOOR)[0], 0)

    def test_translated_robot(self):
        there = Pose2D(10.0, 5.0, 0.0)
        self.assertEqual(classify([pt(10.6, 5.0)], there, FLOOR)[0], 1)
        self.assertEqual(classify([pt(0.6, 0.0)], there, FLOOR), (0, 0))


class TestFloor(unittest.TestCase):
    def test_floor_estimate(self):
        pts = [(math.cos(i) * 1.5, math.sin(i) * 1.5, -0.3) for i in range(100)]
        pts += [(1.0, 0.0, 0.8)] * 20            # some furniture
        self.assertAlmostEqual(floor_from_scan(pts, ORIGIN), -0.3)

    def test_far_points_ignored(self):
        pts = [(5.0, 0.0, -2.0)] * 100 + [(0.5, 0.0, -0.3)] * 100
        self.assertAlmostEqual(floor_from_scan(pts, ORIGIN), -0.3)

    def test_too_few_points_is_none(self):
        self.assertIsNone(floor_from_scan([(0.5, 0.0, -0.3)] * 10, ORIGIN))

    def tilted_scan(self, slope_deg=3.0, seed=5):
        """Carpet tilting up ahead (+x) as the walking robot saw it, a chair
        1.0 m to the side and a 12 cm box 2 m ahead."""
        import random
        rng = random.Random(seed)
        g = math.tan(math.radians(slope_deg))
        floor = lambda x, y: -0.05 + g * x                        # noqa: E731
        pts = []
        for _ in range(4000):
            x, y = rng.uniform(-2.5, 2.5), rng.uniform(-2.5, 2.5)
            if math.hypot(x, y) > 0.4:
                pts.append((x, y, floor(x, y) + rng.gauss(0, 0.017)))
        pts += [(0.3, 1.0, floor(0.3, 1.0) + z / 10) for z in range(1, 9)] * 20     # chair
        pts += [(2.0, 0.0, floor(2.0, 0.0) + 0.12)] * 10                           # low box
        return pts, floor

    def test_floor_plane_follows_the_tilt(self):
        from go2_survey_loop.obstacles import floor_plane_from_scan
        pts, floor = self.tilted_scan()
        pl = floor_plane_from_scan(pts, ORIGIN)
        self.assertIsNotNone(pl)
        self.assertAlmostEqual(pl.gx, math.tan(math.radians(3.0)), delta=0.01)
        self.assertAlmostEqual(pl.gy, 0.0, delta=0.01)
        for x, y in [(0.0, 0.0), (2.0, 0.0), (-2.0, 1.0)]:
            self.assertAlmostEqual(pl.at(x, y), floor(x, y), delta=0.01)

    def test_floor_plane_ends_the_carpet_ghosts(self):
        # one floor height: high carpet 2 m ahead reads as obstacle; the plane: not
        from go2_survey_loop.obstacles import floor_plane_from_scan
        pts, floor = self.tilted_scan()
        carpet = [p for p in pts if abs(p[2] - floor(p[0], p[1])) < 0.06]
        flat = floor_from_scan(pts, ORIGIN)
        pl = floor_plane_from_scan(pts, ORIGIN)
        self.assertGreater(len(obstacle_xy(carpet, ORIGIN, flat)), 20)
        self.assertEqual(obstacle_xy(carpet, ORIGIN, pl), [])
        # and the real things stay: the chair and the 12 cm box
        found = obstacle_xy(pts, ORIGIN, pl)
        self.assertTrue(any(abs(x - 2.0) < 0.01 for x, y in found))
        self.assertTrue(any(abs(y - 1.0) < 0.01 for x, y in found))

    def test_floor_plane_takes_a_number_elsewhere(self):
        from go2_survey_loop.obstacles import FloorPlane
        level = FloorPlane(-0.3, 0.0, 0.0, 0.0, 0.0)
        pts = [pt(0.6, 0.0), pt(0.0, 0.4), (1.0, 0.0, -0.3)]
        self.assertEqual(classify(pts, ORIGIN, level), classify(pts, ORIGIN, -0.3))
        self.assertEqual(obstacle_xy(pts, ORIGIN, level), obstacle_xy(pts, ORIGIN, -0.3))

    def test_floor_plane_refuses_a_ramp(self):
        from go2_survey_loop.obstacles import floor_plane_from_scan
        pts, _ = self.tilted_scan(slope_deg=15.0)
        pl = floor_plane_from_scan(pts, ORIGIN)
        self.assertTrue(pl is None or (pl.gx, pl.gy) == (0.0, 0.0))

    def test_carpet_floor_is_its_middle(self):
        # the robot 2026-11-11: carpet returns spread ~7 cm. The floor must
        # be the carpet's middle, not its low tail, or carpet reads as obstacle.
        import random
        rng = random.Random(3)
        pts = [(rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5), -0.024 + rng.gauss(0, 0.017))
               for _ in range(3000)]
        pts += [(1.0, 0.5, z / 10) for z in range(1, 9)] * 30      # a chair
        f = floor_from_scan(pts, ORIGIN)
        self.assertAlmostEqual(f, -0.024, delta=0.008)
        carpet_high = -0.024 + 3 * 0.017                           # a 3-sigma carpet return
        self.assertEqual(obstacle_xy([(0.6, 0.0, carpet_high)], ORIGIN, f), [])
        self.assertEqual(len(obstacle_xy([(0.6, 0.0, -0.024 + 0.12)], ORIGIN, f)), 1)

    def test_median(self):
        self.assertEqual(median([3, 1, 2]), 2)
        self.assertEqual(median([4, 1, 2, 3]), 2.5)
        self.assertIsNone(median([]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
