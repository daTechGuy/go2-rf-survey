"""Tests for the live-view trail decimation. Stdlib only.

    python tools/tests/test_trail.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from trail import Trail  # noqa: E402


class TestTrail(unittest.TestCase):
    def test_first_point_kept(self):
        t = Trail()
        self.assertTrue(t.add(0, 0, 0))

    def test_standing_still_adds_nothing(self):
        t = Trail()
        t.add(0, 0, 0)
        for _ in range(1000):
            self.assertFalse(t.add(0.001, -0.001, 0.01))
        self.assertEqual(len(t.points), 1)

    def test_moving_adds_at_step_spacing(self):
        t = Trail(step_m=0.05)
        for i in range(301):                       # 3 m in 1 cm steps
            t.add(i * 0.01, 0, 0)
        self.assertAlmostEqual(len(t.points), 61, delta=2)
        self.assertAlmostEqual(t.length(), 3.0, delta=0.06)

    def test_turning_in_place_is_kept(self):
        t = Trail(turn_rad=0.15)
        t.add(0, 0, 0)
        self.assertTrue(t.add(0, 0, 0.2))

    def test_turn_across_pi_measured_short_way(self):
        t = Trail(turn_rad=0.15)
        t.add(0, 0, math.pi - 0.05)
        self.assertFalse(t.add(0, 0, -math.pi + 0.05))   # 0.1 rad apart, not 6.18

    def test_non_finite_never_kept(self):
        t = Trail()
        self.assertFalse(t.add(math.nan, 0, 0))
        self.assertFalse(t.add(0, math.inf, 0))
        self.assertEqual(t.points, [])

    def test_bounded_and_keeps_start(self):
        t = Trail(step_m=0.01, max_points=100)
        for i in range(1000):
            t.add(i * 0.02, 0, 0)
        self.assertLessEqual(len(t.points), 101)
        self.assertEqual(t.points[0], (0.0, 0, 0))

    def test_reset(self):
        t = Trail()
        t.add(0, 0, 0)
        t.reset()
        self.assertEqual(t.points, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
