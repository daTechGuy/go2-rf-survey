"""Tests for finding the Mid-360's mount from the floor. Stdlib only.

A synthetic sensor, pitched nose-down like the real mount, looks at a floor,
four walls and a desk; the fit must recover its pitch, roll and height.

    python tools/tests/test_mid360_mount.py
"""

from __future__ import annotations

import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mid360_mount import (apply, fit_floor, mount_from_floor, rpy_matrix,  # noqa: E402
                          wall_yaw_offset)


def world_scene(rng, noise=0.005):
    """Points in a level frame centred under the sensor: floor z = 0, walls
    of a 6 x 4 m room, a desk."""
    pts = []
    for _ in range(3000):                                    # floor within 4 m
        r, a = 0.5 + 3.5 * math.sqrt(rng.random()), rng.uniform(-math.pi, math.pi)
        pts.append((r * math.cos(a), r * math.sin(a), rng.gauss(0, noise)))
    for _ in range(1500):                                    # walls, 0-2 m high
        side = rng.randrange(4)
        z = rng.uniform(0.0, 2.0)
        if side == 0:
            pts.append((4.0, rng.uniform(-2, 2), z))
        elif side == 1:
            pts.append((-2.0, rng.uniform(-2, 2), z))
        elif side == 2:
            pts.append((rng.uniform(-2, 4), 2.0, z))
        else:
            pts.append((rng.uniform(-2, 4), -2.0, z))
    for _ in range(300):                                     # a desk top at 0.75 m
        pts.append((rng.uniform(1.5, 2.7), rng.uniform(-0.5, 0.1), 0.75))
    return pts


def seen_by_sensor(pts, roll, pitch, yaw, height):
    """The same points in the sensor's own frame (sensor at (0, 0, height))."""
    R = rpy_matrix(roll, pitch, yaw)
    Rt = tuple(zip(*R))                                       # inverse rotation
    return [apply(Rt, (0, 0, 0), (x, y, z - height)) for x, y, z in pts]


class TestRotation(unittest.TestCase):
    def test_positive_pitch_is_nose_down(self):
        R = rpy_matrix(0.0, math.radians(22), 0.0)
        fx, _, fz = apply(R, (0, 0, 0), (1.0, 0.0, 0.0))      # sensor's +x in base
        self.assertGreater(fx, 0.9)
        self.assertLess(fz, -0.3)                             # tilted towards the floor


class TestFloorFit(unittest.TestCase):
    def check(self, roll_deg, pitch_deg, height):
        rng = random.Random(7)
        seen = seen_by_sensor(world_scene(rng), math.radians(roll_deg),
                              math.radians(pitch_deg), 0.3, height)
        fit = fit_floor(seen)
        self.assertIsNotNone(fit)
        roll, pitch, h = mount_from_floor(fit[0], fit[1])
        self.assertAlmostEqual(math.degrees(pitch), pitch_deg, delta=0.5)
        self.assertAlmostEqual(math.degrees(roll), roll_deg, delta=0.5)
        self.assertAlmostEqual(h, height, delta=0.01)

    def test_design_mount(self):
        self.check(0.0, 22.0, 0.45)

    def test_a_little_crooked(self):
        self.check(2.5, 17.0, 0.52)

    def test_level(self):
        self.check(-1.0, 0.0, 0.40)

    def test_desk_and_walls_are_not_the_floor(self):
        # a big desk-top plane must not win: it is not the lowest wide plane
        rng = random.Random(3)
        pts = world_scene(rng) + [(rng.uniform(0.5, 3.0), rng.uniform(-1.5, 1.5), 0.75)
                                  for _ in range(800)]
        seen = seen_by_sensor(pts, 0.0, math.radians(22), 0.0, 0.45)
        _, _, h = mount_from_floor(*fit_floor(seen)[:2])
        self.assertAlmostEqual(h, 0.45, delta=0.02)

    def test_too_few_points(self):
        self.assertIsNone(fit_floor([(0, 0, -0.4)] * 10))


class TestYawCheck(unittest.TestCase):
    def walls(self, yaw_deg, rng):
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        pts = []
        for _ in range(600):
            side = rng.randrange(4)
            x, y = [(4.0, rng.uniform(-2, 2)), (-2.0, rng.uniform(-2, 2)),
                    (rng.uniform(-2, 4), 2.0), (rng.uniform(-2, 4), -2.0)][side]
            pts.append((c * x - s * y, s * x + c * y))
        return pts

    def test_finds_the_rotation(self):
        rng = random.Random(5)
        off = wall_yaw_offset(self.walls(0.0, rng), self.walls(-6.0, rng))
        self.assertAlmostEqual(off, 6.0, delta=1.5)

    def test_aligned(self):
        rng = random.Random(6)
        self.assertAlmostEqual(wall_yaw_offset(self.walls(0, rng), self.walls(0, rng)), 0.0,
                               delta=1.5)

    def test_not_enough_walls(self):
        self.assertIsNone(wall_yaw_offset([(0, 0)], [(1, 1)]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
