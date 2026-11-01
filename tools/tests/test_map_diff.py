"""Tests for 'what moved between two visits'. Stdlib only.

A synthetic room is mapped twice -- the second time from another start spot
and heading, with one table moved -- and the diff must line them up and find
just that table: gone where it was, new where it is.

    python tools/tests/test_map_diff.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from map_diff import align, diff, transform  # noqa: E402

STEP = 0.05


def rect_outline(x0, y0, x1, y1):
    pts = []
    n = int((x1 - x0) / STEP)
    m = int((y1 - y0) / STEP)
    pts += [(x0 + i * STEP, y0) for i in range(n + 1)] + [(x0 + i * STEP, y1) for i in range(n + 1)]
    pts += [(x0, y0 + j * STEP) for j in range(m + 1)] + [(x1, y0 + j * STEP) for j in range(m + 1)]
    return pts


def room(tables):
    """(occupied, free) points of an 8 x 6 m room with an alcove and tables."""
    occ = rect_outline(0, 0, 8, 6) + rect_outline(6, 4, 8, 6)        # alcove: breaks symmetry
    for x, y in tables:
        occ += [(x + i * STEP, y + j * STEP) for i in range(17) for j in range(11)]
    taken = {(round(x / STEP), round(y / STEP)) for x, y in occ}
    free = [(i * STEP, j * STEP) for i in range(1, 160) for j in range(1, 120)
            if (i, j) not in taken and not (i * STEP > 6 and j * STEP > 4)]
    return occ, free


class TestMapDiff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a_occ, cls.a_free = room([(1.5, 1.5), (4.0, 2.5)])
        b_occ, b_free = room([(1.5, 1.5), (4.0, 4.6)])                # second table moved
        # the second visit's own frame: started elsewhere, facing another way
        th, tx, ty = math.radians(35.0), 1.3, -0.8
        inv = lambda pts: transform(transform(pts, 0.0, -tx, -ty), -th, 0.0, 0.0)  # noqa: E731
        cls.b_occ, cls.b_free = inv(b_occ), inv(b_free)
        cls.truth = (th, tx, ty)

    def test_aligns_the_two_visits(self):
        th, tx, ty, score = align(self.a_occ, self.b_occ)
        self.assertAlmostEqual(math.degrees(th) % 360, 35.0, delta=1.0)
        self.assertAlmostEqual(tx, 1.3, delta=0.08)
        self.assertAlmostEqual(ty, -0.8, delta=0.08)
        self.assertGreater(score, 0.75)        # the moved table (~17% of points) can't overlap

    def test_finds_the_moved_table_and_nothing_else(self):
        th, tx, ty, _ = align(self.a_occ, self.b_occ)
        new, gone = diff(self.a_occ, self.a_free, transform(self.b_occ, th, tx, ty),
                         transform(self.b_free, th, tx, ty))
        self.assertEqual(len(gone), 1)
        self.assertEqual(len(new), 1)
        gx, gy, garea, _ = gone[0]
        nx, ny, narea, _ = new[0]
        self.assertLess(math.hypot(gx - 4.4, gy - 2.75), 0.3)         # where it was
        self.assertLess(math.hypot(nx - 4.4, ny - 4.85), 0.3)         # where it is
        self.assertGreater(garea, 0.2)

    def test_unseen_floor_is_not_a_change(self):
        # the second visit never saw the room's left half: the table there
        # isn't "gone" just because it isn't in the second map
        th, tx, ty = self.truth
        b_occ = [(x, y) for x, y in self.b_occ if transform([(x, y)], th, tx, ty)[0][0] > 3.0]
        b_free = [(x, y) for x, y in self.b_free if transform([(x, y)], th, tx, ty)[0][0] > 3.0]
        new, gone = diff(self.a_occ, self.a_free, transform(b_occ, th, tx, ty),
                         transform(b_free, th, tx, ty))
        self.assertTrue(all(g[0] > 3.0 for g in gone))                # nothing reported in the unseen half

    def test_same_room_same_furniture_no_changes(self):
        new, gone = diff(self.a_occ, self.a_free, self.a_occ, self.a_free)
        self.assertEqual((new, gone), ([], []))


if __name__ == "__main__":
    unittest.main(verbosity=2)
