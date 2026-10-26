"""Tests for the staged-rollout gate. Stdlib only.

    python tools/tests/test_stage_gate.py
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import stage_gate  # noqa: E402
from stage_gate import gate  # noqa: E402

NOW = 1_800_000_000.0
H = 3600.0


def mark(age_h=1.0, backend="sport"):
    return {"epoch": NOW - age_h * H, "backend": backend}


class TestGate(unittest.TestCase):
    def test_nothing_recorded_refuses(self):
        ok, reasons = gate(1.0, "sport", None, None, NOW)
        self.assertFalse(ok)
        self.assertIn("Stage 1", reasons[0])

    def test_stage1_allows_one_metre(self):
        self.assertTrue(gate(1.0, "sport", mark(), None, NOW)[0])

    def test_stage1_alone_refuses_three_metres(self):
        ok, reasons = gate(3.0, "sport", mark(), None, NOW)
        self.assertFalse(ok)
        self.assertTrue(any("Stage 2" in r for r in reasons))

    def test_both_allow_three_metres(self):
        self.assertTrue(gate(3.0, "sport", mark(), mark(), NOW)[0])

    def test_stale_pass_refused(self):
        ok, reasons = gate(1.0, "sport", mark(age_h=30), None, NOW)
        self.assertFalse(ok)
        self.assertIn("h old", reasons[0])

    def test_other_backend_pass_does_not_count(self):
        ok, reasons = gate(1.0, "avoid", mark(backend="sport"), None, NOW)
        self.assertFalse(ok)
        self.assertIn("backend", reasons[0])

    def test_future_dated_marker_refused(self):
        self.assertFalse(gate(1.0, "sport", mark(age_h=-5), None, NOW)[0])

    def test_garbage_marker_refused(self):
        self.assertFalse(gate(1.0, "sport", {"nonsense": 1}, None, NOW)[0])


class TestCli(unittest.TestCase):
    def test_record_then_check(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / ".stage1_pass").write_text(json.dumps(
                {"epoch": __import__("time").time(), "backend": "sport"}))
            self.assertEqual(stage_gate.main(["check", "--length", "3.0", "--dir", d]), 1)
            self.assertEqual(stage_gate.main(["record-stage2", "--dir", d]), 0)
            self.assertEqual(stage_gate.main(["check", "--length", "3.0", "--dir", d]), 0)

    def test_corrupt_marker_file_refused(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / ".stage1_pass").write_text("{not json")
            self.assertEqual(stage_gate.main(["check", "--length", "1.0", "--dir", d]), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
