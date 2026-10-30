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

    def test_median(self):
        self.assertEqual(median([3, 1, 2]), 2)
        self.assertEqual(median([4, 1, 2, 3]), 2.5)
        self.assertIsNone(median([]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
