"""Tests for sport-mode request construction.

Request building is a pure function, so the wire format can be pinned without
a robot. The axis-convention test is the important one here: a silent sign
error between Twist and the Move JSON drives the robot sideways into things,
and that is not a failure you want to discover on carpet.

    python go2_survey_bridge/test/test_sport_api.py
"""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_bridge import sport_api  # noqa: E402


class TestApiIds(unittest.TestCase):
    """Pinned against unitree_sdk2's sport_api.hpp.

    These are a wire protocol, not an implementation detail: if upstream
    renumbers them, this fails loudly instead of the robot doing something
    unexpected.
    """

    def test_ids_used_by_the_bridge(self) -> None:
        self.assertEqual(sport_api.DAMP, 1001)
        self.assertEqual(sport_api.BALANCE_STAND, 1002)
        self.assertEqual(sport_api.STOP_MOVE, 1003)
        self.assertEqual(sport_api.STAND_UP, 1004)
        self.assertEqual(sport_api.STAND_DOWN, 1005)
        self.assertEqual(sport_api.RECOVERY_STAND, 1006)
        self.assertEqual(sport_api.EULER, 1007)
        self.assertEqual(sport_api.MOVE, 1008)

    def test_ids_are_distinct(self) -> None:
        ids = [
            sport_api.DAMP, sport_api.BALANCE_STAND, sport_api.STOP_MOVE,
            sport_api.STAND_UP, sport_api.STAND_DOWN,
            sport_api.RECOVERY_STAND, sport_api.EULER, sport_api.MOVE,
            sport_api.SIT, sport_api.RISE_SIT, sport_api.SPEED_LEVEL,
        ]
        self.assertEqual(len(ids), len(set(ids)))

    def test_topic_name(self) -> None:
        self.assertEqual(sport_api.SPORT_REQUEST_TOPIC, "/api/sport/request")


class TestMoveParameter(unittest.TestCase):
    def test_axis_mapping_matches_twist_exactly(self) -> None:
        # Both conventions are body-frame right-handed with Z up, so this is
        # a straight field copy: Twist.linear.x -> "x", .linear.y -> "y",
        # .angular.z -> "z". No sign flips, no renaming.
        d = json.loads(sport_api.move_parameter(0.5, -0.25, 0.75))
        self.assertEqual(d, {"x": 0.5, "y": -0.25, "z": 0.75})

    def test_forward_is_positive_x(self) -> None:
        self.assertGreater(json.loads(sport_api.move_parameter(1.0, 0, 0))["x"], 0)

    def test_left_is_positive_y(self) -> None:
        self.assertGreater(json.loads(sport_api.move_parameter(0, 1.0, 0))["y"], 0)

    def test_ccw_yaw_is_positive_z(self) -> None:
        self.assertGreater(json.loads(sport_api.move_parameter(0, 0, 1.0))["z"], 0)

    def test_zero_velocity_is_explicit_not_omitted(self) -> None:
        d = json.loads(sport_api.move_parameter(0.0, 0.0, 0.0))
        self.assertEqual(set(d), {"x", "y", "z"})
        self.assertEqual(list(d.values()), [0.0, 0.0, 0.0])

    def test_ints_are_coerced_to_float(self) -> None:
        # json.dumps would otherwise emit `1` where the robot expects `1.0`.
        d = json.loads(sport_api.move_parameter(1, 0, 0))
        self.assertIsInstance(d["x"], float)

    def test_output_is_valid_json(self) -> None:
        json.loads(sport_api.move_parameter(0.1, 0.2, 0.3))   # must not raise


class TestOtherParameters(unittest.TestCase):
    def test_euler(self) -> None:
        d = json.loads(sport_api.euler_parameter(0.1, -0.2, 0.3))
        self.assertEqual(d, {"x": 0.1, "y": -0.2, "z": 0.3})

    def test_speed_level(self) -> None:
        self.assertEqual(json.loads(sport_api.speed_level_parameter(-1)), {"data": -1})

    def test_speed_level_coerces_to_int(self) -> None:
        self.assertIsInstance(json.loads(sport_api.speed_level_parameter(0))["data"], int)


class TestApiNames(unittest.TestCase):
    def test_known_ids_are_named(self) -> None:
        self.assertEqual(sport_api.api_name(sport_api.MOVE), "Move")
        self.assertEqual(sport_api.api_name(sport_api.STOP_MOVE), "StopMove")

    def test_unknown_id_degrades_readably(self) -> None:
        self.assertEqual(sport_api.api_name(9999), "api_id=9999")


if __name__ == "__main__":
    unittest.main(verbosity=2)
