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

from mid360_mount import (apply, fit_floor, floor_below, icp_2d, level_peak,  # noqa: E402
                          matrix_rpy, mount_from_floor, mount_in_body, rpy_matrix,
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


def ceiling_scene(rng, ceiling=2.9, noise=0.005):
    """As the robot saw it 2026-11-11: a ceiling everywhere, walls, and only
    a sliver of floor far ahead (the Mid-360 looks no lower than -7 deg)."""
    pts = []
    for _ in range(4000):
        r, a = 0.3 + 4.0 * math.sqrt(rng.random()), rng.uniform(-math.pi, math.pi)
        pts.append((r * math.cos(a), r * math.sin(a), ceiling + rng.gauss(0, noise)))
    for _ in range(2500):
        side = rng.randrange(2)
        pts.append(((2.9, -3.7)[side], rng.uniform(-2, 2), rng.uniform(0.0, ceiling)))
    for _ in range(300):
        pts.append((rng.uniform(1.8, 2.8), rng.uniform(-0.6, 0.6), rng.gauss(0, noise)))
    return pts


class TestCeilingFit(unittest.TestCase):
    def test_robot_like_mount(self):
        rng = random.Random(11)
        seen = seen_by_sensor(ceiling_scene(rng), math.radians(-2.2), math.radians(12.8),
                              0.1, 0.62)
        fit = fit_floor(seen, ceiling=True)
        self.assertIsNotNone(fit)
        roll, pitch, d = mount_from_floor(fit[0], fit[1])
        self.assertAlmostEqual(math.degrees(pitch), 12.8, delta=0.5)
        self.assertAlmostEqual(math.degrees(roll), -2.2, delta=0.5)
        self.assertAlmostEqual(d, 2.9 - 0.62, delta=0.01)       # distance up to the ceiling
        floor = fit_floor(seen)
        self.assertTrue(floor is None or floor[2] < fit[2])     # so the ceiling is chosen
        self.assertAlmostEqual(floor_below(seen, fit[0]), 0.62, delta=0.02)

    def test_floor_not_chair_seats(self):
        rng = random.Random(13)
        pts = ceiling_scene(rng) + [(rng.uniform(1.0, 2.0), rng.uniform(-1, 1), 0.45)
                                    for _ in range(900)]         # more seat than floor
        seen = seen_by_sensor(pts, 0.0, math.radians(13), 0.0, 0.62)
        fit = fit_floor(seen, ceiling=True)
        self.assertAlmostEqual(floor_below(seen, fit[0]), 0.62, delta=0.02)
        self.assertIsNone(floor_below(seen[:10], fit[0]))

    def test_walls_are_not_the_ceiling(self):
        # the robot's bad run: a plane ~59 deg off took the floor's place.
        # Without a ceiling, only chance planes remain -- far smaller.
        rng = random.Random(12)
        walls = [p for p in ceiling_scene(rng) if p[2] < 2.8]
        seen = seen_by_sensor(walls, 0.0, math.radians(13), 0.0, 0.62)
        fit = fit_floor(seen, ceiling=True)
        self.assertTrue(fit is None or fit[2] < 0.05 * len(seen))


class TestMountInBody(unittest.TestCase):
    def test_rpy_round_trip(self):
        for r, p, y in [(0.1, 0.2, 0.3), (-0.04, 0.23, -1.2), (0.0, 0.0, 3.0)]:
            self.assertEqual([round(v, 9) for v in matrix_rpy(rpy_matrix(r, p, y))],
                             [round(v, 9) for v in (r, p, y)])

    def test_body_tilt_is_not_counted_twice(self):
        # the robot 2026-11-11: body rolled -2.2, pitched -1.3; the ceiling
        # gave the sensor against level. Body tilt then the body mount must
        # give back the level mount, for points and the offset alike.
        level, t = (math.radians(-2.3), math.radians(12.9), math.radians(3.6)), (0.226, 0.0, 0.114)
        br, bp = math.radians(-2.2), math.radians(-1.3)
        (r, p, y), tb = mount_in_body(*level, t, br, bp)
        B = rpy_matrix(br, bp, 0.0)
        for pt in [(1.0, 0.0, 0.0), (0.3, -2.0, 0.5)]:
            via_body = apply(B, (0, 0, 0), apply(rpy_matrix(r, p, y), tb, pt))
            direct = apply(rpy_matrix(*level), t, pt)
            for a, b in zip(via_body, direct):
                self.assertAlmostEqual(a, b, places=9)
        self.assertAlmostEqual(math.degrees(r), -0.1, delta=0.3)     # the roll was the body's

    def test_level_body_changes_nothing(self):
        (r, p, y), t = mount_in_body(0.01, 0.2, 0.05, (0.2, 0.0, 0.1), 0.0, 0.0)
        self.assertAlmostEqual(p, 0.2, places=9)
        self.assertEqual(tuple(round(v, 9) for v in t), (0.2, 0.0, 0.1))


class TestIcp2d(unittest.TestCase):
    def test_finds_rotation_and_shift(self):
        rng = random.Random(21)
        walls = [(2.5, rng.uniform(-2, 1.5)) for _ in range(800)]
        walls += [(rng.uniform(0.5, 2.5), -2.0) for _ in range(500)]
        walls += [(rng.uniform(0.8, 1.6), rng.uniform(0.4, 0.6)) for _ in range(150)]   # a desk
        th, sx, sy = math.radians(-0.6), -0.06, 0.03                 # what the sensor is off by
        c, s = math.cos(th), math.sin(th)
        seen = [(c * x - s * y + sx + rng.gauss(0, 0.005), s * x + c * y + sy + rng.gauss(0, 0.005))
                for x, y in walls]
        seen += [(rng.uniform(-4, -3), rng.uniform(-2, 3)) for _ in range(600)]   # only it sees
        fit = icp_2d(walls[::2], seen)
        self.assertIsNotNone(fit)
        dth, dx, dy, n, gap = fit
        # applying the fit to the sensor's points undoes its error
        for x, y in [(2.5, 0.0), (1.0, -2.0)]:
            px, py = c * x - s * y + sx, s * x + c * y + sy
            cx, cy = math.cos(dth), math.sin(dth)
            self.assertAlmostEqual(cx * px - cy * py + dx, x, delta=0.01)
            self.assertAlmostEqual(cy * px + cx * py + dy, y, delta=0.01)
        self.assertLess(gap, 0.01)

    def test_nothing_shared(self):
        self.assertIsNone(icp_2d([(0.0, 0.0)] * 10, [(5.0, 5.0)] * 100))


class TestLevelPeak(unittest.TestCase):
    def test_ceiling_over_lights(self):
        rng = random.Random(4)
        zs = [rng.gauss(2.90, 0.01) for _ in range(2000)]
        zs += [rng.uniform(2.3, 2.6) for _ in range(150)]          # hanging lights, walls
        self.assertAlmostEqual(level_peak(zs), 2.90, delta=0.01)

    def test_nothing_up_there(self):
        self.assertIsNone(level_peak([]))
        self.assertIsNone(level_peak([2.0, 2.5, 3.0]))


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
