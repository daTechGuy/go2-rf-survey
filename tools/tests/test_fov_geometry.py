"""Tests for the Mid-360 mount geometry.

These pin the numbers that appear in docs/plan.md and the README. If the
maths and the docs ever disagree, this fails -- which is the point, because
the mount angle is drilled into a bracket exactly once.

    python tools/tests/test_fov_geometry.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fov_geometry import (  # noqa: E402
    FOV_LOWER_DEG,
    FOV_UPPER_DEG,
    MIN_RANGE_M,
    blind_radius_m,
    ground_contact_m,
    lowest_ray_deg,
    recommend_pitch,
    upper_fov_deg,
)


class TestSensorConstants(unittest.TestCase):
    def test_fov_matches_datasheet(self) -> None:
        self.assertEqual(FOV_LOWER_DEG, -7.0)
        self.assertEqual(FOV_UPPER_DEG, 52.0)
        self.assertEqual(FOV_UPPER_DEG - FOV_LOWER_DEG, 59.0)   # quoted as 59 deg

    def test_fov_is_biased_upward(self) -> None:
        # The whole reason this module exists. If this ever reads symmetric,
        # someone has "corrected" the constants and the mount advice is void.
        self.assertLess(abs(FOV_LOWER_DEG), abs(FOV_UPPER_DEG))


class TestDocumentedTable(unittest.TestCase):
    """The exact figures published in docs/plan.md, at h = 0.40 m."""

    H = 0.40

    def test_level_mount_is_3_26_m(self) -> None:
        self.assertAlmostEqual(ground_contact_m(0.0, self.H), 3.26, places=2)

    def test_fifteen_degrees(self) -> None:
        self.assertAlmostEqual(ground_contact_m(15.0, self.H), 0.99, places=2)

    def test_twenty_two_degrees_is_the_chosen_default(self) -> None:
        self.assertAlmostEqual(ground_contact_m(22.0, self.H), 0.72, places=2)

    def test_twenty_five_degrees(self) -> None:
        self.assertAlmostEqual(ground_contact_m(25.0, self.H), 0.64, places=2)

    def test_thirty_degrees(self) -> None:
        self.assertAlmostEqual(ground_contact_m(30.0, self.H), 0.53, places=2)

    def test_upper_fov_column(self) -> None:
        for pitch, expected in [(0, 52), (15, 37), (22, 30), (25, 27), (30, 22)]:
            with self.subTest(pitch=pitch):
                self.assertAlmostEqual(upper_fov_deg(pitch), expected, places=6)


class TestGroundContact(unittest.TestCase):
    def test_steeper_pitch_sees_closer(self) -> None:
        prev = math.inf
        for pitch in range(0, 61, 5):
            d = ground_contact_m(float(pitch), 0.40)
            self.assertLess(d, prev)
            prev = d

    def test_scales_linearly_with_height(self) -> None:
        # d = h / tan(...), so doubling the mount height doubles the blind
        # distance. Worth pinning: it is why a taller mount is not free.
        self.assertAlmostEqual(
            ground_contact_m(22.0, 0.80), 2 * ground_contact_m(22.0, 0.40), places=9
        )

    def test_matches_closed_form(self) -> None:
        for pitch in (0.0, 7.5, 22.0, 33.3):
            with self.subTest(pitch=pitch):
                expected = 0.40 / math.tan(math.radians(pitch + 7.0))
                self.assertAlmostEqual(ground_contact_m(pitch, 0.40), expected, places=9)

    def test_negative_pitch_past_the_fov_never_reaches_the_floor(self) -> None:
        # Tilted nose-UP by more than 7 deg: the lowest ray points at or above
        # horizontal, so it never meets the floor. Infinity is the honest
        # answer; returning a large finite number would invite a silent bug.
        self.assertEqual(ground_contact_m(-7.0, 0.40), math.inf)
        self.assertEqual(ground_contact_m(-20.0, 0.40), math.inf)

    def test_straight_down_is_zero(self) -> None:
        self.assertEqual(ground_contact_m(90.0, 0.40), 0.0)


class TestLowestRay(unittest.TestCase):
    def test_level(self) -> None:
        self.assertEqual(lowest_ray_deg(0.0), -7.0)

    def test_documented_values(self) -> None:
        for pitch, expected in [(15, -22), (22, -29), (25, -32), (30, -37)]:
            with self.subTest(pitch=pitch):
                self.assertAlmostEqual(lowest_ray_deg(pitch), expected, places=6)


class TestBlindRadius(unittest.TestCase):
    def test_floors_at_the_sensor_blind_sphere(self) -> None:
        # However steeply pitched, the Mid-360 still cannot see inside 0.1 m.
        self.assertEqual(blind_radius_m(89.0, 0.40), MIN_RANGE_M)

    def test_matches_ground_contact_in_the_normal_range(self) -> None:
        for pitch in (10.0, 22.0, 30.0):
            with self.subTest(pitch=pitch):
                self.assertAlmostEqual(
                    blind_radius_m(pitch, 0.40), ground_contact_m(pitch, 0.40)
                )


class TestRecommendation(unittest.TestCase):
    def test_default_constraints_land_near_the_chosen_22(self) -> None:
        rec = recommend_pitch(0.40, max_blind_m=0.8, min_upper_fov_deg=25.0)
        self.assertIsNotNone(rec)
        self.assertGreaterEqual(rec, 15.0)
        self.assertLessEqual(rec, 25.0)

    def test_recommendation_satisfies_both_constraints(self) -> None:
        for h in (0.30, 0.40, 0.50):
            with self.subTest(height=h):
                rec = recommend_pitch(h, 0.8, 25.0)
                if rec is None:
                    continue
                self.assertLessEqual(blind_radius_m(rec, h), 0.8 + 1e-9)
                self.assertGreaterEqual(upper_fov_deg(rec), 25.0 - 1e-9)

    def test_returns_shallowest_workable_pitch(self) -> None:
        # Shallowest keeps the most upper FOV for SLAM features, so anything
        # 0.1 deg shallower must fail the blind-radius constraint.
        rec = recommend_pitch(0.40, 0.8, 25.0)
        self.assertGreater(blind_radius_m(rec - 0.1, 0.40), 0.8)

    def test_impossible_constraints_return_none(self) -> None:
        # A very tall mount with a tight blind budget cannot be satisfied
        # without pitching so far down that SLAM loses its upward features.
        self.assertIsNone(recommend_pitch(2.0, max_blind_m=0.2, min_upper_fov_deg=45.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
