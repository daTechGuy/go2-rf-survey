"""Tests for the per-run report. Stdlib only, no robot.

    python tools/tests/test_run_report.py
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from run_report import build, parse_demolog, render, verdict  # noqa: E402

# trimmed from room_auto4_20261031_014434.demolog (the robot, 1 m route)
ROUTE = """\
== LIVE run: the robot will walk a 1.0 m out_and_back on its own (backend sport)
  route out_and_back: length 1.0 m, laps 1
  speed <= 0.3 m/s, 1.0 rad/s; stop 0.5 m ahead / 0.55 m around
   01:45:38 remote start (loop idle) -> start
   01:45:40 running | waypoint 1/2 | send vx=0.30 vyaw=-0.01 | odom 0.01s old, lidar 0.04s old, floor -0.07, front 0, near 0
   01:45:42 running | waypoint 1/2 | send vx=0.30 vyaw=-0.01 | odom 0.00s old, lidar 0.03s old, floor -0.07, front 0, near 0
   01:45:45 running | waypoint 2/2 | send vx=0.00 vyaw=1.00 | odom 0.01s old, lidar 0.01s old, floor -0.08, front 0, near 1
   01:45:51 done | loop complete | send vx=0.00 vyaw=0.00 | odom 0.00s old, lidar 0.01s old, floor -0.08, front 0, near 0
extent     15.9 x 11.2 m (317x224 cells)
loop       path 1.9 m, start-end gap 0.18 m (9.5% -- only meaningful if you finished where you started)
== 01:46:35 demo finished: done
"""

# shaped like the exploration run's log, with pauses, crossing midnight
EXPLORE = """\
== LIVE run: the robot will explore on its own, up to 8 legs within 4.0 m, then walk home (backend sport)
  explore: 8 legs of <= 1.5 m within 4.0 m, then home
   23:59:50 remote start (loop idle) -> start
   23:59:51 running | explore leg 1/8 | send vx=0.30 vyaw=-0.01 | odom 0.00s old, lidar 0.01s old, floor -0.08, front 0, near 0
   00:00:20 paused | obstacle (front=5, near=0) -- re-planning | send vx=0.00 vyaw=0.00 | odom 0.0s old, lidar 0.0s old, floor -0.08, front 5, near 0
   00:00:21 paused | obstacle (front=6, near=0) -- re-planning | send vx=0.00 vyaw=0.00 | odom 0.0s old, lidar 0.0s old, floor -0.08, front 6, near 0
   00:00:22 running | blocked -- choosing another way (explore leg 3/8) | send vx=0.00 vyaw=-1.00 | odom 0.0s old, lidar 0.0s old, floor -0.08, front 0, near 1
   00:01:40 done | explored 8 legs, home | send vx=0.00 vyaw=0.00 | odom 0.0s old, lidar 0.0s old, floor -0.08, front 0, near 0
loop       path 16.4 m, start-end gap 0.16 m (1.0%)
== 00:02:30 demo finished: done
"""

STOPPED = """\
== LIVE run: the robot will walk a 1.0 m out_and_back on its own (backend sport)
   01:27:05 remote start (loop idle) -> start
   01:27:06 running | waypoint 1/2 | send vx=0.30 vyaw=-0.01 | odom 0.00s old, lidar 0.02s old, floor -0.08, front 0, near 0
== 01:29:16 demo finished: stopped early (Ctrl-C / demo-stop)
"""


class TestParse(unittest.TestCase):
    def test_route(self):
        r = parse_demolog(ROUTE)
        self.assertEqual(r["outcome"], "done")
        self.assertEqual(r["duration_s"], 13)
        self.assertEqual((r["path_m"], r["gap_m"]), (1.9, 0.18))
        self.assertEqual(r["extent"], "15.9 x 11.2 m")
        self.assertEqual(r["mode"], "route")
        self.assertEqual([s for _, s, _ in r["timeline"]], ["running", "running", "done"])
        self.assertEqual(r["pauses"], [])

    def test_explore_pauses_dedup_and_midnight(self):
        r = parse_demolog(EXPLORE)
        self.assertEqual(r["mode"], "explore")
        self.assertEqual(r["legs"], 8)
        self.assertEqual(r["duration_s"], 110)          # 23:59:50 -> 00:01:40
        # two pause lines differing only in counts are one pause
        self.assertEqual(len(r["pauses"]), 1)
        self.assertIn("re-planning", r["pauses"][0][1])

    def test_stopped_early(self):
        r = parse_demolog(STOPPED)
        self.assertTrue(r["outcome"].startswith("stopped early"))
        self.assertEqual(r["duration_s"], 131)          # until it was stopped
        self.assertIsNone(r["path_m"])

    def test_empty_log(self):
        r = parse_demolog("")
        self.assertIsNone(r["outcome"])
        self.assertEqual(verdict(None)[1], "warn")


class TestRender(unittest.TestCase):
    def test_numbers_and_escaping(self):
        r = parse_demolog(EXPLORE)
        r["timeline"].append(("00:02:00", "paused", "<script>x</script>"))
        page = render("room_explore_test", r)
        self.assertIn("16.4 m", page)
        self.assertIn("Legs explored", page)
        self.assertIn("&lt;script&gt;", page)
        self.assertNotIn("<script>x", page)
        self.assertIn("prefers-color-scheme: dark", page)

    def test_build_from_files(self):
        with tempfile.TemporaryDirectory() as d:
            run = Path(d) / "room_auto4_20261031_014434"
            run.with_name(run.name + ".demolog").write_text(ROUTE)
            run.with_name(run.name + "_map-topdown.png").write_bytes(b"\x89PNG fake")
            out = build(run)
            page = out.read_text(encoding="utf-8")
            self.assertTrue(out.name.endswith("_report.html"))
            self.assertIn("data:image/png;base64,", page)
            self.assertIn("Completed", page)
            self.assertNotIn("Robot camera", page)       # no camera stills this run

    def test_missing_log(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(FileNotFoundError):
                build(Path(d) / "nothing_here")


if __name__ == "__main__":
    unittest.main(verbosity=2)
