"""Tests for choosing which robot results to copy. Stdlib only, no robot.

    python tools/tests/test_get_results.py
"""

from __future__ import annotations

import datetime as dt
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from get_results import parse_listing, plan, select  # noqa: E402

TODAY = dt.date(2026, 10, 31)
LISTING = "\n".join([
    "room_explore_20261031_023814/room_explore_20261031_023814_0.db3\t1100000000",
    "room_explore_20261031_023814/metadata.yaml\t2000",
    "room_explore_20261031_023814_map-topdown.png\t120000",
    "room_explore_20261031_023814.demolog\t12000",
    "room_20261029_024705_map.pcd\t2000000",
    "turn_19700102_041243.csv\t34000",            # no clock: stamped 1970
    "manual_20261001_101010/manual_20261001_101010_0.db3\t900000000",   # a month old
    "go2loop.log\t40000",
    "sim_gazebo.log\t9000000",                     # undated and big: skipped
    "garbage line without a tab",
]) + "\n"


def paths(entries):
    return [p for p, _ in entries]


class TestSelect(unittest.TestCase):
    def setUp(self):
        self.entries = parse_listing(LISTING)

    def test_parse_skips_junk(self):
        self.assertEqual(len(self.entries), 9)
        self.assertIn(("go2loop.log", 40000), self.entries)

    def test_this_week_plus_1970_plus_small_logs(self):
        got = paths(select(self.entries, TODAY, 7))
        self.assertIn("turn_19700102_041243.csv", got)
        self.assertIn("go2loop.log", got)
        self.assertNotIn("sim_gazebo.log", got)
        self.assertFalse(any(p.startswith("manual_20261001") for p in got))

    def test_floorplans_first_recordings_last(self):
        got = paths(select(self.entries, TODAY, 7))
        self.assertTrue(got[0].endswith("topdown.png"))
        self.assertTrue(got[-1].endswith(".db3"))

    def test_no_recordings_keeps_bag_metadata(self):
        got = paths(select(self.entries, TODAY, 7, recordings=False))
        self.assertFalse(any(p.endswith(".db3") for p in got))
        self.assertIn("room_explore_20261031_023814/metadata.yaml", got)

    def test_one_run(self):
        got = paths(select(self.entries, TODAY, 7, run="room_explore_20261031"))
        self.assertEqual(len(got), 4)
        self.assertTrue(all(p.startswith("room_explore_20261031") for p in got))

    def test_more_days_reaches_older_runs(self):
        got = paths(select(self.entries, TODAY, 31))
        self.assertTrue(any(p.startswith("manual_20261001") for p in got))


class TestPlan(unittest.TestCase):
    def test_skips_files_already_here_whole(self):
        chosen = [("a.png", 100), ("b.db3", 1000), ("c.log", 50)]
        here = {"a.png": 100, "b.db3": 400}        # b arrived only partly
        todo, done = plan(chosen, here.get)
        self.assertEqual(paths(done), ["a.png"])
        self.assertEqual(paths(todo), ["b.db3", "c.log"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
