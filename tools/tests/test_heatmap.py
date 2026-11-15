"""Unit tests for the pure helpers in ``tools/heatmap.py``."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from heatmap import collapse_per_position, geodesic_distance, load_survey  # noqa: E402


class TestHeatmapHelpers(unittest.TestCase):
    def test_load_survey_parses_and_skips_bad_rows(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "survey.csv"
            p.write_text(
                "x,y,rssi,bssid,ssid,band,sample_id\n"
                "1.0,2.0,-55,aa:bb:cc:dd:ee:01,Corp,5,s1\n"
                "bad,2.0,-60,aa:bb:cc:dd:ee:01,Corp,5,s2\n"
                "1.5,2.5,-62,aa:bb:cc:dd:ee:02,Guest,2.4,s3\n",
                encoding="utf-8",
            )
            rows = load_survey(p)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["bssid"], "aa:bb:cc:dd:ee:01")
        self.assertAlmostEqual(rows[0]["rssi"], -55.0)
        self.assertEqual(rows[1]["band"], "2.4")

    def test_load_survey_rejects_missing_columns(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.csv"
            p.write_text("x,y,rssi\n1,2,-50\n", encoding="utf-8")
            with self.assertRaises(SystemExit):
                load_survey(p)

    def test_collapse_per_position_keeps_strongest_per_bssid(self) -> None:
        rows = [
            {"x": 1.001, "y": 2.002, "bssid": "aa", "ssid": "S", "rssi": -70.0},
            {"x": 1.004, "y": 2.001, "bssid": "aa", "ssid": "S", "rssi": -52.0},
            {"x": 1.002, "y": 2.003, "bssid": "aa", "ssid": "S", "rssi": -61.0},
            {"x": 1.002, "y": 2.003, "bssid": "bb", "ssid": "S", "rssi": -65.0},
        ]
        collapsed = collapse_per_position(rows)
        by_bssid = {r["bssid"]: r["rssi"] for r in collapsed}
        self.assertEqual(by_bssid, {"aa": -52.0, "bb": -65.0})

    def test_geodesic_distance_blocked_by_solid_wall(self) -> None:
        # 5x5 grid with a solid vertical wall down column 2.
        free = [
            [True, True, False, True, True],
            [True, True, False, True, True],
            [True, True, False, True, True],
            [True, True, False, True, True],
            [True, True, False, True, True],
        ]
        dist = geodesic_distance(free, [(2, 1)], cell_m=0.1, max_reach_m=3.0)
        self.assertAlmostEqual(dist[2][1], 0.0)
        self.assertAlmostEqual(dist[2][0], 0.1)
        # Column 3 is only 0.2 m away in Euclidean terms, but behind the wall.
        for r in range(5):
            self.assertTrue(math.isinf(dist[r][3]))
            self.assertTrue(math.isinf(dist[r][4]))

    def test_geodesic_distance_reaches_through_doorway(self) -> None:
        # Same wall in column 2, with a 1-cell doorway at row 4.
        free = [
            [True, True, False, True, True],
            [True, True, False, True, True],
            [True, True, False, True, True],
            [True, True, False, True, True],
            [True, True, True,  True, True],
        ]
        dist = geodesic_distance(free, [(0, 1)], cell_m=1.0, max_reach_m=10.0)
        # Straight across the wall at (0, 3) must travel down to row 4, through
        # (4, 2), and back up to (0, 3) -> strictly longer than Euclidean 2.0 m.
        self.assertTrue(math.isfinite(dist[0][3]))
        self.assertGreater(dist[0][3], 5.0)
        # And if max_reach_m is 3.0 m, (0, 3) is out of reach even though its
        # Euclidean distance from (0, 1) is only 2.0 m.
        capped = geodesic_distance(free, [(0, 1)], cell_m=1.0, max_reach_m=3.0)
        self.assertTrue(math.isinf(capped[0][3]))

    def test_geodesic_distance_does_not_squeeze_diagonal_wall_corners(self) -> None:
        # Diagonal wall blocking (0,0) -> (1,1): cells (0,1) and (1,0) are walls.
        free = [
            [True,  False],
            [False, True],
        ]
        dist = geodesic_distance(free, [(0, 0)], cell_m=1.0, max_reach_m=5.0)
        self.assertAlmostEqual(dist[0][0], 0.0)
        self.assertTrue(math.isinf(dist[1][1]))


if __name__ == "__main__":
    unittest.main()
