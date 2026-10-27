"""The obstacle check's body box and clearance ring, held against the Go2's
measured footprint. Stdlib only.

Numbers from dancher00/go2-nav2-wifi (MIT), docs/GO2-FOOTPRINT.md and
go2_nav2/config/go2_footprint.yaml, commit 4a8a8b9: the standing robot's
URDF visual meshes projected over 33 live joint samples (2026-09-11), and
the padded Nav2 envelope also used by go2_ros2_sdk and BotBrain. All in
base_link, metres. Standing pose only -- not the swept volume of every gait.

    python go2_survey_loop/test/test_footprint.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_loop.geometry import Pose2D  # noqa: E402
from go2_survey_loop.obstacles import ObstacleParams, classify  # noqa: E402

MEASURED_X = (-0.35345, 0.33196)
MEASURED_Y = (-0.17027, 0.19290)
ENVELOPE_X = (-0.45, 0.36)
ENVELOPE_Y = (-0.20, 0.20)
ENVELOPE_PADDING = 0.03

P = ObstacleParams()


def corners(xs, ys, pad=0.0):
    return [(x + math.copysign(pad, x), y + math.copysign(pad, y)) for x in xs for y in ys]


def furthest(points):
    return max(math.hypot(x, y) for x, y in points)


class TestBodyBox(unittest.TestCase):
    def test_body_box_contains_the_measured_robot(self):
        # Otherwise the LiDAR sees its own legs as obstacles and the robot
        # refuses to move (or pauses at random as the legs swing).
        self.assertGreaterEqual(P.body_front, MEASURED_X[1])
        self.assertGreaterEqual(P.body_rear, -MEASURED_X[0])
        self.assertGreaterEqual(P.body_half_width, max(-MEASURED_Y[0], MEASURED_Y[1]))

    def test_body_box_stays_inside_the_nav_envelope(self):
        # Bigger than the envelope would hide real obstacles next to the robot.
        self.assertLessEqual(P.body_front, ENVELOPE_X[1])
        self.assertLessEqual(P.body_rear, -ENVELOPE_X[0])
        self.assertLessEqual(P.body_half_width, ENVELOPE_Y[1])

    def test_measured_outline_is_not_an_obstacle(self):
        pts = [(x, y, 0.3) for x, y in corners(MEASURED_X, MEASURED_Y)] * 10
        self.assertEqual(classify(pts, Pose2D(0, 0, 0), 0.0, P), (0, 0))


class TestClearanceRing(unittest.TestCase):
    def test_ring_clears_the_turn_in_place_sweep(self):
        # Turning on the spot, the body sweeps a circle out to its furthest
        # corner. The ring must reach past it, or the robot turns into things
        # the check never looked at.
        self.assertGreater(P.clearance_radius, furthest(corners(MEASURED_X, MEASURED_Y)))
        self.assertGreaterEqual(P.clearance_radius,
                                furthest(corners(ENVELOPE_X, ENVELOPE_Y, ENVELOPE_PADDING)))

    def test_margins_as_documented(self):
        # docs/autonomous-loop.md quotes these; keep the doc honest.
        self.assertAlmostEqual(furthest(corners(MEASURED_X, MEASURED_Y)), 0.40, places=2)
        self.assertAlmostEqual(furthest(corners(ENVELOPE_X, ENVELOPE_Y)), 0.49, places=2)
        self.assertAlmostEqual(furthest(corners(ENVELOPE_X, ENVELOPE_Y, ENVELOPE_PADDING)),
                               0.53, places=2)

    def test_point_just_outside_the_sweep_counts(self):
        r = furthest(corners(ENVELOPE_X, ENVELOPE_Y, ENVELOPE_PADDING)) + 0.01
        pts = [(0.0, r, 0.3)] * 10                    # beside the robot
        self.assertEqual(classify(pts, Pose2D(0, 0, 0), 0.0, P)[1], 10)


class TestFrontLane(unittest.TestCase):
    def test_lane_wider_than_the_body(self):
        self.assertGreaterEqual(P.lane_half_width - ENVELOPE_Y[1], 0.10)


if __name__ == "__main__":
    unittest.main(verbosity=2)
