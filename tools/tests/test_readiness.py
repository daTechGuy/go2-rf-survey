"""Tests for the session-preflight verdicts. Stdlib only.

    python tools/tests/test_readiness.py
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from readiness import assess, summary  # noqa: E402

# What the robot actually reported on 2026-10-21, standing at the door.
GOOD = {"odom_hz": 150.0, "cloud_hz": 15.4, "cloud_frame": "odom", "sport_hz": 50.0,
        "mode_name": "normal", "avoid_enabled": True, "sport_mode": 0, "gait_type": 0,
        "error_code": 100, "body_height": 0.32}


def with_(**kw):
    s = dict(GOOD)
    s.update(kw)
    return s


def fails(lines):
    return [l for l in lines if l.startswith("FAIL")]


class TestVerdicts(unittest.TestCase):
    def test_all_good(self):
        sensing, motion, lines = assess(GOOD)
        self.assertTrue(sensing)
        self.assertTrue(motion)
        self.assertEqual(fails(lines), [])

    def test_breathing_reminder_always_present(self):
        for s in (GOOD, {}):
            self.assertTrue(any("breathing" in l for l in assess(s)[2]))

    def test_empty_snapshot_is_not_ready(self):
        sensing, motion, _ = assess({})
        self.assertFalse(sensing)
        self.assertFalse(motion)

    def test_no_odometry_blocks_everything(self):
        sensing, motion, _ = assess(with_(odom_hz=0.0))
        self.assertFalse(sensing)
        self.assertFalse(motion)

    def test_slow_lidar_blocks_everything(self):
        self.assertFalse(assess(with_(cloud_hz=2.0))[0])

    def test_wrong_cloud_frame_blocks_everything(self):
        sensing, motion, lines = assess(with_(cloud_frame="utlidar_lidar"))
        self.assertFalse(sensing)
        self.assertTrue(any("frame" in l for l in fails(lines)))

    def test_wrong_mode_blocks_motion_only(self):
        sensing, motion, lines = assess(with_(mode_name="ai"))
        self.assertTrue(sensing)          # can still dry-run and map
        self.assertFalse(motion)
        self.assertTrue(any("'ai'" in l and "blocks motion" in l for l in lines))

    def test_silent_switcher_blocks_motion_only(self):
        sensing, motion, _ = assess(with_(mode_name=None))
        self.assertTrue(sensing)
        self.assertFalse(motion)

    def test_lying_down_blocks_motion_only(self):
        sensing, motion, _ = assess(with_(body_height=0.08))
        self.assertTrue(sensing)
        self.assertFalse(motion)

    def test_no_sport_state_blocks_motion_only(self):
        sensing, motion, _ = assess(with_(sport_hz=0.0))
        self.assertTrue(sensing)
        self.assertFalse(motion)

    def test_avoidance_off_warns_not_blocks(self):
        sensing, motion, lines = assess(with_(avoid_enabled=False))
        self.assertTrue(sensing and motion)
        self.assertTrue(any(l.startswith("WARN") and "avoidance OFF" in l for l in lines))

    def test_summary_text(self):
        self.assertIn("motion: NOT READY", summary(True, False))
        self.assertIn("sensing/dry-run/mapping: READY", summary(True, False))


if __name__ == "__main__":
    unittest.main(verbosity=2)
