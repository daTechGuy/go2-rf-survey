"""Tests for the did-the-feet-step check. Stdlib only.

    python tools/tests/test_gait_analysis.py
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gait_analysis import FEET, format_steps, liftoffs, steps_between  # noqa: E402

STAND_F, STAND_Z = 120, -0.30     # standing force (robot units) and foot z (m)


def rows(n, dt=0.02, t0=0.0, force=None, z=None):
    """n rows; force(i, foot) / z(i, foot) override the standing values."""
    out = []
    for i in range(n):
        r = {"t": t0 + i * dt}
        for k in range(4):
            r[f"ff{k}"] = STAND_F if force is None else force(i, k)
            r[f"fz{k}"] = STAND_Z if z is None else z(i, k)
        out.append(r)
    return out


def trot(i, k):
    """Diagonal pairs (FR+RL, FL+RR) alternate swing every 10 samples."""
    swing_pair = (i // 10) % 2
    in_swing = (k in (0, 3)) if swing_pair == 0 else (k in (1, 2))
    return 5 if in_swing else 2 * STAND_F


def trot_z(i, k):
    swing_pair = (i // 10) % 2
    in_swing = (k in (0, 3)) if swing_pair == 0 else (k in (1, 2))
    return STAND_Z + (0.07 if in_swing else 0.0)


class TestLiftoffs(unittest.TestCase):
    def test_counts_each_lift_once(self):
        self.assertEqual(liftoffs([100, 10, 10, 100, 100, 5, 100], 100), 2)

    def test_hysteresis_ignores_jitter(self):
        # dips to 50% are weight shifting, not a lift-off
        self.assertEqual(liftoffs([100, 50, 100, 45, 100], 100), 0)

    def test_no_baseline(self):
        self.assertEqual(liftoffs([0, 0, 0], 0), 0)
        self.assertEqual(liftoffs([1, 2], None), 0)

    def test_none_samples_skipped(self):
        self.assertEqual(liftoffs([100, None, 10, None, 100], 100), 1)


class TestSteps(unittest.TestCase):
    def test_trot_is_stepping(self):
        data = rows(50) + rows(100, t0=1.0, force=trot, z=trot_z)
        rep = steps_between(data, 1.0, 3.0)
        self.assertTrue(rep.stepped)
        for foot in FEET:
            self.assertGreaterEqual(rep.liftoffs[foot], 2)
            self.assertAlmostEqual(rep.lift_m[foot], 0.07, places=6)

    def test_twist_on_planted_feet_is_not(self):
        # what 2026-10-28 looked like: weight shifts, one rear foot nudged 1 cm
        def shift(i, k):
            return STAND_F * (0.7 if (i // 7) % 2 else 1.2)

        def nudge(i, k):
            return STAND_Z + (0.01 if k == 2 and 20 < i < 30 else 0.0)
        data = rows(50) + rows(100, t0=1.0, force=shift, z=nudge)
        rep = steps_between(data, 1.0, 3.0)
        self.assertFalse(rep.stepped)
        self.assertIn("NO STEPS", format_steps(rep))

    def test_one_foot_lifting_is_not_a_gait(self):
        def one(i, k):
            return 0 if k == 2 and 10 < i < 20 else STAND_F
        data = rows(50) + rows(100, t0=1.0, force=one)
        rep = steps_between(data, 1.0, 3.0)
        self.assertEqual(rep.liftoffs["RR"], 1)
        self.assertFalse(rep.stepped)

    def test_force_only_still_judges(self):
        # foot z missing (older log): force alone decides
        data = [dict(r, fz0=None, fz1=None, fz2=None, fz3=None)
                for r in rows(50) + rows(100, t0=1.0, force=trot)]
        rep = steps_between(data, 1.0, 3.0)
        self.assertTrue(rep.stepped)
        self.assertIsNone(rep.lift_m["FR"])
        self.assertIn("STEPPED", format_steps(rep))

    def test_all_zero_feet_are_no_data_not_planted(self):
        # the Gazebo sim leaves foot force and position at zero
        rep = steps_between(rows(100, force=lambda i, k: 0, z=lambda i, k: 0.0), 1.0, 2.0)
        self.assertFalse(rep.has_data)
        self.assertIn("no foot data", format_steps(rep))
        self.assertNotIn("NO STEPS", format_steps(rep))

    def test_no_standing_rows_uses_whole_log(self):
        rep = steps_between(rows(100, force=trot, z=trot_z), 0.0, 2.0)
        self.assertTrue(rep.stepped)


if __name__ == "__main__":
    unittest.main(verbosity=2)
