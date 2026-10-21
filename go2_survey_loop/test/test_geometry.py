"""Tests for loop geometry. No ROS.

    python go2_survey_loop/test/test_geometry.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_loop.geometry import (  # noqa: E402
    Pose2D, out_and_back_local, plan_local, rectangle_local, route_extent,
    to_robot, to_world, wrap_angle, yaw_from_quaternion,
)


class TestWrap(unittest.TestCase):
    def test_range(self):
        for a in (-10.0, -math.pi, 0.0, math.pi, 7.5, 100.0):
            w = wrap_angle(a)
            self.assertGreaterEqual(w, -math.pi)
            self.assertLess(w, math.pi)

    def test_identity_inside_range(self):
        self.assertAlmostEqual(wrap_angle(1.0), 1.0)
        self.assertAlmostEqual(wrap_angle(-1.0), -1.0)

    def test_just_past_pi_wraps_negative(self):
        self.assertAlmostEqual(wrap_angle(math.pi + 0.1), -math.pi + 0.1)


class TestQuaternion(unittest.TestCase):
    def test_identity(self):
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, 0, 1), 0.0)

    def test_quarter_turn(self):
        h = math.sqrt(0.5)
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, h, h), math.pi / 2)

    def test_half_turn(self):
        self.assertAlmostEqual(abs(yaw_from_quaternion(0, 0, 1, 0)), math.pi)


class TestRectangle(unittest.TestCase):
    def test_left_loop_returns_to_start(self):
        wps = rectangle_local(4.0, 3.0, "left")
        self.assertEqual(wps, [(4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)])

    def test_right_loop_goes_negative_y(self):
        wps = rectangle_local(4.0, 3.0, "right")
        self.assertEqual(wps[1], (4.0, -3.0))
        self.assertEqual(wps[-1], (0.0, 0.0))

    def test_laps_repeat(self):
        self.assertEqual(len(rectangle_local(2, 1, laps=3)), 12)

    def test_zero_or_negative_size_refused(self):
        for l, w in ((0, 3), (4, 0), (-1, 3), (4, -2)):
            with self.assertRaises(ValueError):
                rectangle_local(l, w)

    def test_bad_turn_refused(self):
        with self.assertRaises(ValueError):
            rectangle_local(2, 2, "sideways")

    def test_zero_laps_refused(self):
        with self.assertRaises(ValueError):
            rectangle_local(2, 2, laps=0)


class TestOutAndBack(unittest.TestCase):
    def test_out_and_back(self):
        self.assertEqual(out_and_back_local(3.0), [(3.0, 0.0), (0.0, 0.0)])

    def test_laps(self):
        self.assertEqual(len(out_and_back_local(3.0, laps=2)), 4)

    def test_zero_length_refused(self):
        for bad in (0.0, -1.0):
            with self.assertRaises(ValueError):
                out_and_back_local(bad)

    def test_plan_dispatch(self):
        self.assertEqual(plan_local("out_and_back", 3.0), out_and_back_local(3.0))
        self.assertEqual(plan_local("rectangle", 4.0, 3.0), rectangle_local(4.0, 3.0))

    def test_out_and_back_ignores_width(self):
        self.assertEqual(plan_local("out_and_back", 3.0, width=99.0), out_and_back_local(3.0))

    def test_unknown_shape_refused(self):
        with self.assertRaises(ValueError):
            plan_local("figure_eight", 3.0, 3.0)

    def test_route_extent(self):
        self.assertAlmostEqual(route_extent("rectangle", 3.0, 4.0), 5.0)
        self.assertAlmostEqual(route_extent("out_and_back", 3.0, 99.0), 3.0)


class TestTransforms(unittest.TestCase):
    def test_to_world_identity_start(self):
        self.assertEqual(to_world(Pose2D(0, 0, 0), [(1, 2)]), [(1, 2)])

    def test_to_world_rotated_start(self):
        # robot started facing +y at (5, 5): "1 m ahead" is (5, 6)
        (x, y), = to_world(Pose2D(5, 5, math.pi / 2), [(1, 0)])
        self.assertAlmostEqual(x, 5.0)
        self.assertAlmostEqual(y, 6.0)

    def test_to_robot_inverts_to_world(self):
        start = Pose2D(2.0, -1.0, 0.7)
        for lx, ly in [(1, 0), (0, 1), (-2, 3.5)]:
            (wx, wy), = to_world(start, [(lx, ly)])
            rx, ry = to_robot(start, wx, wy)
            self.assertAlmostEqual(rx, lx)
            self.assertAlmostEqual(ry, ly)


if __name__ == "__main__":
    unittest.main(verbosity=2)
