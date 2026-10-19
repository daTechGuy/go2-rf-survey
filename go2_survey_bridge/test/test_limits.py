"""Tests for the velocity limiter and command watchdog.

No ROS, no robot, no clock. The watchdog takes its timestamps as arguments
precisely so this file can exist -- an untestable watchdog is not a safety
feature.

    python go2_survey_bridge/test/test_limits.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_bridge.limits import (  # noqa: E402
    Limits,
    Velocity,
    VelocityLimiter,
    Watchdog,
    clamp,
)


class TestClamp(unittest.TestCase):
    def test_within_limit_passes_through(self) -> None:
        self.assertEqual(clamp(0.3, 0.6), 0.3)
        self.assertEqual(clamp(-0.3, 0.6), -0.3)

    def test_clamps_both_signs(self) -> None:
        self.assertEqual(clamp(5.0, 0.6), 0.6)
        self.assertEqual(clamp(-5.0, 0.6), -0.6)

    def test_negative_limit_treated_as_magnitude(self) -> None:
        self.assertEqual(clamp(5.0, -0.6), 0.6)

    def test_nan_becomes_zero_not_passed_through(self) -> None:
        # A NaN reaching the robot is unpredictable. "No number" can only
        # safely mean "do not move".
        self.assertEqual(clamp(float("nan"), 0.6), 0.0)

    def test_infinity_clamps_to_limit_with_sign(self) -> None:
        self.assertEqual(clamp(float("inf"), 0.6), 0.6)
        self.assertEqual(clamp(float("-inf"), 0.6), -0.6)


class TestLimitsClamping(unittest.TestCase):
    def test_each_axis_uses_its_own_ceiling(self) -> None:
        lim = Limits(max_vx=0.6, max_vy=0.4, max_vyaw=0.8)
        out = lim.clamp_velocity(Velocity(10.0, 10.0, 10.0))
        self.assertEqual((out.vx, out.vy, out.vyaw), (0.6, 0.4, 0.8))

    def test_defaults_are_conservative(self) -> None:
        # Guards against someone "helpfully" raising these to the robot's
        # actual capability. Surveying wants a slow, steady platform.
        lim = Limits()
        self.assertLessEqual(lim.max_vx, 1.0)
        self.assertLessEqual(lim.max_vy, 1.0)
        self.assertLessEqual(lim.max_vyaw, 1.5)


class TestVelocityLimiter(unittest.TestCase):
    def setUp(self) -> None:
        self.lim = VelocityLimiter(Limits(max_vx=0.6, accel_xy=0.8, accel_yaw=1.6))

    def test_starts_at_rest(self) -> None:
        self.assertTrue(self.lim.current.is_zero)

    def test_ramps_rather_than_stepping(self) -> None:
        # 0.8 m/s^2 over 0.1 s = 0.08 m/s, not the full 0.6 requested.
        out = self.lim.step(Velocity(vx=0.6), dt=0.1)
        self.assertAlmostEqual(out.vx, 0.08, places=6)

    def test_reaches_target_eventually(self) -> None:
        for _ in range(100):
            out = self.lim.step(Velocity(vx=0.6), dt=0.05)
        self.assertAlmostEqual(out.vx, 0.6, places=6)

    def test_never_overshoots_target(self) -> None:
        prev = 0.0
        for _ in range(200):
            out = self.lim.step(Velocity(vx=0.5), dt=0.05)
            self.assertLessEqual(out.vx, 0.5 + 1e-9)
            self.assertGreaterEqual(out.vx, prev - 1e-9)
            prev = out.vx

    def test_clamps_before_slewing(self) -> None:
        # Asking for 10 m/s must ramp toward the 0.6 ceiling, not toward 10.
        for _ in range(500):
            out = self.lim.step(Velocity(vx=10.0), dt=0.02)
        self.assertAlmostEqual(out.vx, 0.6, places=6)

    def test_ramps_down_symmetrically(self) -> None:
        for _ in range(100):
            self.lim.step(Velocity(vx=0.6), dt=0.05)
        out = self.lim.step(Velocity(vx=0.0), dt=0.1)
        self.assertAlmostEqual(out.vx, 0.6 - 0.08, places=6)

    def test_yaw_uses_its_own_acceleration(self) -> None:
        out = self.lim.step(Velocity(vyaw=1.0), dt=0.1)
        self.assertAlmostEqual(out.vyaw, 0.16, places=6)   # 1.6 rad/s^2 * 0.1

    def test_zero_and_negative_dt_are_noops(self) -> None:
        self.lim.step(Velocity(vx=0.6), dt=0.1)
        before = self.lim.current
        self.assertEqual(self.lim.step(Velocity(vx=0.6), dt=0.0), before)
        self.assertEqual(self.lim.step(Velocity(vx=0.6), dt=-1.0), before)

    def test_stop_immediately_bypasses_the_ramp(self) -> None:
        # Ramping down over half a second is exactly wrong when the reason
        # for stopping is that something has gone wrong.
        for _ in range(100):
            self.lim.step(Velocity(vx=0.6), dt=0.05)
        self.assertTrue(self.lim.stop_immediately().is_zero)
        self.assertTrue(self.lim.current.is_zero)

    def test_reset_clears_history(self) -> None:
        for _ in range(100):
            self.lim.step(Velocity(vx=0.6), dt=0.05)
        self.lim.reset()
        out = self.lim.step(Velocity(vx=0.6), dt=0.1)
        self.assertAlmostEqual(out.vx, 0.08, places=6)   # ramps from rest again

    def test_nan_target_decays_to_zero(self) -> None:
        for _ in range(100):
            self.lim.step(Velocity(vx=0.6), dt=0.05)
        for _ in range(100):
            out = self.lim.step(Velocity(vx=float("nan")), dt=0.05)
        self.assertAlmostEqual(out.vx, 0.0, places=6)


class TestVelocity(unittest.TestCase):
    def test_is_zero(self) -> None:
        self.assertTrue(Velocity().is_zero)
        self.assertFalse(Velocity(vx=0.01).is_zero)
        self.assertFalse(Velocity(vyaw=-0.01).is_zero)

    def test_speed_ignores_yaw(self) -> None:
        self.assertAlmostEqual(Velocity(3.0, 4.0, 99.0).speed(), 5.0)


class TestWatchdog(unittest.TestCase):
    def test_does_not_trip_before_any_command(self) -> None:
        # Nothing has been commanded, so there is nothing to guard. Tripping
        # here would fire a StopMove at every idle robot on startup.
        wd = Watchdog(timeout_s=0.3)
        self.assertFalse(wd.check(1000.0))
        self.assertFalse(wd.tripped)

    def test_fresh_commands_do_not_trip(self) -> None:
        wd = Watchdog(timeout_s=0.3)
        t = 100.0
        for _ in range(20):
            wd.feed(t)
            t += 0.1
            self.assertFalse(wd.check(t))
        self.assertFalse(wd.tripped)

    def test_trips_after_timeout(self) -> None:
        wd = Watchdog(timeout_s=0.3)
        wd.feed(100.0)
        self.assertFalse(wd.check(100.2))
        self.assertTrue(wd.check(100.4))
        self.assertTrue(wd.tripped)

    def test_trip_is_edge_triggered(self) -> None:
        # One StopMove per outage, not one per tick for as long as the
        # planner stays dead.
        wd = Watchdog(timeout_s=0.3)
        wd.feed(100.0)
        self.assertTrue(wd.check(100.4))
        for t in (100.5, 100.6, 101.0, 200.0):
            self.assertFalse(wd.check(t))
        self.assertEqual(wd.trip_count, 1)

    def test_recovery_clears_the_trip(self) -> None:
        wd = Watchdog(timeout_s=0.3)
        wd.feed(100.0)
        wd.check(100.4)
        self.assertTrue(wd.feed(101.0))     # True == this cleared a trip
        self.assertFalse(wd.tripped)

    def test_feed_returns_false_when_not_tripped(self) -> None:
        wd = Watchdog(timeout_s=0.3)
        self.assertFalse(wd.feed(100.0))
        self.assertFalse(wd.feed(100.1))

    def test_can_trip_again_after_recovery(self) -> None:
        wd = Watchdog(timeout_s=0.3)
        wd.feed(100.0)
        wd.check(100.4)
        wd.feed(101.0)
        self.assertTrue(wd.check(101.5))
        self.assertEqual(wd.trip_count, 2)

    def test_age_is_none_before_first_command(self) -> None:
        self.assertIsNone(Watchdog().age(100.0))

    def test_age_tracks_elapsed(self) -> None:
        wd = Watchdog()
        wd.feed(100.0)
        self.assertAlmostEqual(wd.age(100.25), 0.25)

    def test_rejects_nonpositive_timeout(self) -> None:
        with self.assertRaises(ValueError):
            Watchdog(timeout_s=0.0)
        with self.assertRaises(ValueError):
            Watchdog(timeout_s=-1.0)

    def test_exactly_at_timeout_does_not_trip(self) -> None:
        # Boundary pinned explicitly: trips strictly *after* the timeout, so
        # a command arriving exactly on the period boundary is still good.
        wd = Watchdog(timeout_s=0.3)
        wd.feed(100.0)
        self.assertFalse(wd.check(100.3))
        self.assertTrue(wd.check(100.3 + 1e-6))


if __name__ == "__main__":
    unittest.main(verbosity=2)
