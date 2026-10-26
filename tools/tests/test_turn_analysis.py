"""Tests for the Stage 1 turn-test analysis. Stdlib only, no ROS.

Each scenario synthesises the CSV the robot-side logger writes and checks the
analysis tells the stories apart -- including a reconstruction of the first
real Stage 1 run (little rotation while commanded, a swing after the stop).

    python tools/tests/test_turn_analysis.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from turn_analysis import MOVE, STOP_MOVE, analyse, format_report, passed  # noqa: E402

DT = 0.02          # 50 Hz logger
VYAW = 0.2
T_CMD0, T_CMD1, T_END = 1.0, 3.0, 7.0


def synth(rate_fn, odom_extra=lambda t: 0.0, cmd=VYAW, send_move=True,
          yaw0=0.3, t_end=T_END):
    """Rows for a test where the robot's true yaw rate is rate_fn(t).
    The bridge sends Move(cmd) on [T_CMD0, T_CMD1) then StopMove."""
    rows, yaw, t = [], yaw0, 0.0
    while t <= t_end + 1e-9:
        if send_move and T_CMD0 <= t < T_CMD1:
            c, api = cmd, MOVE
        elif send_move and t >= T_CMD1:
            c, api = 0.0, STOP_MOVE
        else:
            c, api = 0.0, None
        rate = rate_fn(t)
        rows.append({"t": round(t, 4), "cmd": c, "api": api,
                     "imu_yaw": (yaw + math.pi) % (2 * math.pi) - math.pi,
                     "odom_yaw": (yaw + odom_extra(t) + math.pi) % (2 * math.pi) - math.pi,
                     "yaw_speed": rate})
        yaw += rate * DT
        t += DT
    return rows


def prompt(t):         # follows the command with a 0.1 s lag
    return VYAW if T_CMD0 + 0.1 <= t < T_CMD1 + 0.1 else 0.0


class TestScenarios(unittest.TestCase):
    def test_clean_turn_passes(self):
        r = analyse(synth(prompt))
        self.assertTrue(passed(r), format_report(r))
        self.assertAlmostEqual(r.commanded_deg, math.degrees(VYAW * 2.0), delta=1.5)
        self.assertAlmostEqual(r.imu_during_deg, math.degrees(VYAW * 2.0), delta=2.0)
        self.assertLess(r.latency_s, 0.2)

    def test_first_real_run_pattern_fails_with_the_right_reasons(self):
        # 2026-10-21: ~3 deg while commanded, then a swing to ~+26 and back to
        # ~+8 starting ~1 s after the stop. Heading sources agree (real motion).
        def late(t):
            if T_CMD0 <= t < T_CMD1:
                return 0.026                       # ~3 deg over 2 s
            if T_CMD1 + 1.0 <= t < T_CMD1 + 1.5:
                return 0.7                         # swing out
            if T_CMD1 + 1.5 <= t < T_CMD1 + 2.5:
                return -0.31                       # swing back
            return 0.0
        r = analyse(synth(late, t_end=9.0))        # real run settled ~3 s after
        self.assertFalse(passed(r))
        joined = " ".join(r.verdicts)
        self.assertIn("UNDER-RESPONSE", joined)
        self.assertIn("MOVED AFTER STOP", joined)
        self.assertNotIn("DISAGREE", joined)       # both sources saw it: it was real
        self.assertIn("STOPPED", joined)           # ...and it did settle
        # the swing is what matters, not the small net change
        self.assertGreater(r.imu_after_peak_deg, 15.0)
        self.assertLess(abs(r.imu_after_deg), 5.0)

    def test_net_change_alone_would_hide_a_swing(self):
        # out 20 deg and straight back after the stop: net ~0, peak ~20
        def swing(t):
            if T_CMD0 + 0.1 <= t < T_CMD1 + 0.1:
                return VYAW
            if T_CMD1 + 0.5 <= t < T_CMD1 + 1.0:
                return 0.7
            if T_CMD1 + 1.0 <= t < T_CMD1 + 1.5:
                return -0.7
            return 0.0
        r = analyse(synth(swing, t_end=9.0))
        self.assertLess(abs(r.imu_after_deg), 3.0)
        self.assertIn("MOVED AFTER STOP", " ".join(r.verdicts))
        self.assertFalse(passed(r))

    def test_odometry_artifact_detected(self):
        # robot genuinely still; LiDAR odometry jumps 20 deg when legs move
        jump = lambda t: math.radians(20) if T_CMD1 + 1.0 <= t < T_CMD1 + 2.0 else 0.0
        r = analyse(synth(lambda t: 0.0, odom_extra=jump))
        self.assertFalse(passed(r))
        joined = " ".join(r.verdicts)
        self.assertIn("DISAGREE", joined)
        self.assertIn("NO RESPONSE", joined)

    def test_lag_detected(self):
        late_start = lambda t: VYAW if T_CMD0 + 0.9 <= t < T_CMD1 + 0.9 else 0.0
        r = analyse(synth(late_start))
        self.assertIn("LAG", " ".join(r.verdicts))
        self.assertAlmostEqual(r.latency_s, 0.9, delta=0.05)
        self.assertFalse(passed(r))

    def test_still_rotating_at_end_fails(self):
        runaway = lambda t: VYAW if t >= T_CMD0 + 0.1 else 0.0
        r = analyse(synth(runaway))
        self.assertIn("STILL ROTATING", " ".join(r.verdicts))
        self.assertFalse(passed(r))

    def test_no_move_sent(self):
        r = analyse(synth(lambda t: 0.0, send_move=False))
        self.assertIn("NO MOVE SENT", " ".join(r.verdicts))
        self.assertFalse(passed(r))

    def test_clockwise_command(self):
        cw = lambda t: -VYAW if T_CMD0 + 0.1 <= t < T_CMD1 + 0.1 else 0.0
        r = analyse(synth(cw, cmd=-VYAW))
        self.assertTrue(passed(r), format_report(r))
        self.assertLess(r.imu_during_deg, 0)

    def test_crossing_pi_is_not_a_full_turn(self):
        # start just below +pi; a 23 deg CCW turn crosses into negative angles
        r = analyse(synth(prompt, yaw0=math.pi - 0.1))
        self.assertTrue(passed(r), format_report(r))
        self.assertAlmostEqual(r.imu_during_deg, math.degrees(VYAW * 2.0), delta=2.0)

    def test_missing_samples_tolerated(self):
        rows = synth(prompt)
        for i, row in enumerate(rows):
            if i % 7 == 0:
                row["odom_yaw"] = None
            if i % 11 == 0:
                row["yaw_speed"] = None
        self.assertTrue(passed(analyse(rows)))

    def test_empty_refused(self):
        with self.assertRaises(ValueError):
            analyse([])

    def test_report_is_printable(self):
        text = format_report(analyse(synth(prompt)))
        self.assertIn("STAGE 1: PASS", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
