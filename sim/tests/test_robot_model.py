"""Tests for the simulated Go2's request/remote model. Stdlib only.

    python sim/tests/test_robot_model.py
"""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_sim_adapter.robot_model import (  # noqa: E402
    AV_MOVE, AV_SWITCH_GET, AV_USE_REMOTE, BALANCE_STAND, DAMP, KEY, MODE_DAMP,
    MODE_STAND, MODE_WALK, MOVE, MS_CHECK_MODE, STOP_MOVE, SWITCH_JOYSTICK, RobotModel,
)


def keys(*names):
    return sum(1 << KEY[n] for n in names)


def mv(x=0.0, y=0.0, z=0.0):
    return json.dumps({"x": x, "y": y, "z": z})


class TestSport(unittest.TestCase):
    def test_move_drives(self):
        m = RobotModel()
        m.sport(MOVE, mv(0.2, 0, 0.1), 0.0)
        self.assertEqual(m.command(0.0), (0.2, 0.0, 0.1))
        self.assertEqual(m.mode(0.0), MODE_WALK)

    def test_move_holds_until_stopmove_by_default(self):
        # worst case on purpose: only our bridge's watchdog can stop it
        m = RobotModel()
        m.sport(MOVE, mv(0.2), 0.0)
        self.assertEqual(m.command(100.0)[0], 0.2)
        m.sport(STOP_MOVE, "", 100.0)
        self.assertEqual(m.command(100.0), (0.0, 0.0, 0.0))
        self.assertEqual(m.mode(100.0), MODE_STAND)

    def test_optional_move_timeout(self):
        m = RobotModel(move_timeout_s=0.5)
        m.sport(MOVE, mv(0.2), 0.0)
        self.assertEqual(m.command(0.6), (0.0, 0.0, 0.0))

    def test_damp_ignores_move_until_stand(self):
        m = RobotModel()
        m.sport(DAMP, "", 0.0)
        m.sport(MOVE, mv(0.3), 0.1)
        self.assertEqual(m.command(0.1), (0.0, 0.0, 0.0))
        self.assertEqual(m.mode(0.1), MODE_DAMP)
        self.assertLess(m.body_height(), 0.25)        # preflight sees "not standing"
        m.sport(BALANCE_STAND, "", 0.2)
        m.sport(MOVE, mv(0.3), 0.3)
        self.assertEqual(m.command(0.3)[0], 0.3)

    def test_switch_joystick(self):
        m = RobotModel()
        m.sport(SWITCH_JOYSTICK, json.dumps({"data": False}), 0.0)
        m.remote(0, 0.0, 1.0, 0.0, 0.0)               # full stick forward
        self.assertEqual(m.command(0.0), (0.0, 0.0, 0.0))
        m.sport(SWITCH_JOYSTICK, json.dumps({"data": True}), 0.0)
        m.remote(0, 0.0, 1.0, 0.0, 0.0)
        self.assertEqual(m.command(0.0)[0], m.max_stick_v)

    def test_bad_json_is_harmless(self):
        m = RobotModel()
        self.assertEqual(m.sport(MOVE, "{not json", 0.0), (0, ""))
        self.assertEqual(m.command(0.0), (0.0, 0.0, 0.0))


class TestRemote(unittest.TestCase):
    def test_l2_start_damps_even_with_sticks_off_and_api_holding(self):
        m = RobotModel()
        m.sport(SWITCH_JOYSTICK, json.dumps({"data": False}), 0.0)
        m.avoid(AV_USE_REMOTE, json.dumps({"is_remote_commands_from_api": True}), 0.0)
        m.sport(MOVE, mv(0.3), 0.0)
        m.remote(keys("L2", "START"), 0, 0, 0, 0)
        self.assertTrue(m.damped)
        self.assertEqual(m.command(0.0), (0.0, 0.0, 0.0))

    def test_l2_a_stands_again(self):
        m = RobotModel()
        m.remote(keys("L2", "START"), 0, 0, 0, 0)
        m.remote(keys("L2", "A"), 0, 0, 0, 0)
        self.assertFalse(m.damped)

    def test_stick_overrides_api_like_the_real_robot(self):
        m = RobotModel()
        m.sport(MOVE, mv(0.3), 0.0)
        m.remote(0, 0.0, -0.5, 0.0, 0.0)               # pull back
        self.assertLess(m.command(0.0)[0], 0.0)
        m.remote(0, 0.0, 0.0, 0.0, 0.0)                # let go: API again
        self.assertEqual(m.command(0.0)[0], 0.3)

    def test_api_holding_ignores_sticks(self):
        m = RobotModel()
        m.avoid(AV_USE_REMOTE, json.dumps({"is_remote_commands_from_api": True}), 0.0)
        m.avoid(AV_MOVE, json.dumps({"x": 0.2, "y": 0.0, "yaw": 0.0, "mode": 0}), 0.0)
        m.remote(0, 0.0, 1.0, 0.0, 0.0)
        self.assertEqual(m.command(0.0)[0], 0.2)

    def test_stick_noise_ignored(self):
        m = RobotModel()
        m.remote(0, 0.05, -0.05, 0.02, 0.0)
        self.assertIsNone(m.stick_vel)


class TestServices(unittest.TestCase):
    def test_check_mode_normal(self):
        code, data = RobotModel.motion_switcher(MS_CHECK_MODE, "", 0.0)
        self.assertEqual((code, json.loads(data)["name"]), (0, "normal"))

    def test_avoid_switch_get(self):
        code, data = RobotModel().avoid(AV_SWITCH_GET, "", 0.0)
        self.assertTrue(json.loads(data)["enable"])

    def test_avoid_move_uses_yaw_field(self):
        m = RobotModel()
        m.avoid(AV_MOVE, json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.4, "mode": 0}), 0.0)
        self.assertEqual(m.command(0.0)[2], 0.4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
