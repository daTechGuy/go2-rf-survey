"""Tests for the bridge's motion backends. No ROS, no robot.

The avoid backend's sequencing is safety-relevant: it takes control from the
handheld remote, so it must claim only when moving and always hand control
back on release.

    python go2_survey_bridge/test/test_motion_backend.py
"""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_bridge.motion_backend import (  # noqa: E402
    AVOID, AVOID_MOVE, AVOID_SWITCH_SET, AVOID_USE_REMOTE_COMMAND_FROM_API,
    SPORT, SPORT_FREE_WALK, SPORT_MOVE, SPORT_STOP_MOVE, SPORT_SWITCH_JOYSTICK,
    AvoidBackend, SportBackend, SportPrepBackend, make_backend,
)


def params(reqs):
    return [(svc, api, json.loads(p) if p else {}) for svc, api, p in reqs]


class TestSport(unittest.TestCase):
    def test_move_is_sport_move_with_z_yaw(self):
        (svc, api, p), = params(SportBackend().move(0.2, 0.0, -0.3))
        self.assertEqual((svc, api), (SPORT, SPORT_MOVE))
        self.assertEqual(p, {"x": 0.2, "y": 0.0, "z": -0.3})

    def test_no_claim(self):
        self.assertEqual(SportBackend().claim(), [])

    def test_stop_and_release_are_stopmove(self):
        b = SportBackend()
        self.assertEqual(b.stop(), [(SPORT, SPORT_STOP_MOVE, "")])
        self.assertEqual(b.release(), [(SPORT, SPORT_STOP_MOVE, "")])


class TestAvoid(unittest.TestCase):
    def test_first_move_claims_then_moves(self):
        reqs = params(AvoidBackend().move(0.3, 0.0, 0.1))
        self.assertEqual([r[:2] for r in reqs], [
            (AVOID, AVOID_SWITCH_SET),
            (AVOID, AVOID_USE_REMOTE_COMMAND_FROM_API),
            (AVOID, AVOID_MOVE),
        ])
        self.assertEqual(reqs[0][2], {"enable": True})
        self.assertEqual(reqs[1][2], {"is_remote_commands_from_api": True})
        self.assertEqual(reqs[2][2], {"x": 0.3, "y": 0.0, "yaw": 0.1, "mode": 0})

    def test_later_moves_do_not_reclaim(self):
        b = AvoidBackend()
        b.move(0.3, 0, 0)
        self.assertEqual([r[1] for r in b.move(0.2, 0, 0)], [AVOID_MOVE])

    def test_yaw_field_is_yaw_not_z(self):
        # sport uses "z" for yaw rate, obstacles_avoid uses "yaw": a silent
        # mix-up here would drop the turn command entirely
        (_, _, p), = params(AvoidBackend().stop())
        self.assertIn("yaw", p)
        self.assertNotIn("z", p)

    def test_stop_keeps_control(self):
        b = AvoidBackend()
        b.move(0.3, 0, 0)
        reqs = b.stop()
        self.assertEqual([r[1] for r in reqs], [AVOID_MOVE])
        self.assertTrue(b.holding)
        self.assertEqual(json.loads(reqs[0][2])["x"], 0.0)

    def test_release_zeroes_then_hands_back_to_remote(self):
        b = AvoidBackend()
        b.move(0.3, 0, 0)
        reqs = params(b.release())
        self.assertEqual([r[1] for r in reqs], [AVOID_MOVE, AVOID_USE_REMOTE_COMMAND_FROM_API])
        self.assertEqual(reqs[0][2]["x"], 0.0)
        self.assertEqual(reqs[1][2], {"is_remote_commands_from_api": False})
        self.assertFalse(b.holding)

    def test_release_without_claim_still_stops(self):
        reqs = AvoidBackend().release()
        self.assertEqual([r[1] for r in reqs], [AVOID_MOVE])        # zero, no hand-back needed

    def test_release_twice_hands_back_once(self):
        b = AvoidBackend()
        b.move(0.3, 0, 0)
        b.release()
        self.assertEqual([r[1] for r in b.release()], [AVOID_MOVE])

    def test_reclaims_after_release(self):
        b = AvoidBackend()
        b.move(0.3, 0, 0)
        b.release()
        self.assertEqual([r[1] for r in b.move(0.3, 0, 0)][:2],
                         [AVOID_SWITCH_SET, AVOID_USE_REMOTE_COMMAND_FROM_API])

    def test_nan_becomes_zero(self):
        (_, _, p), = params(SportBackend().move(float("nan"), 0, float("inf")))
        self.assertEqual((p["x"], p["z"]), (0.0, 0.0))
        b = AvoidBackend()
        p = params(b.move(float("nan"), 0, 0))[-1][2]
        self.assertEqual(p["x"], 0.0)


class TestSportPrep(unittest.TestCase):
    """Walk prep from dancher00/go2-nav2-wifi; the sticks must come back."""

    def test_claim_switches_sticks_off_then_free_walk(self):
        reqs = params(SportPrepBackend().claim())
        self.assertEqual([r[:2] for r in reqs],
                         [(SPORT, SPORT_SWITCH_JOYSTICK), (SPORT, SPORT_FREE_WALK)])
        self.assertEqual(reqs[0][2], {"data": False})

    def test_claims_on_start_others_do_not(self):
        self.assertTrue(SportPrepBackend().claim_on_start)
        self.assertFalse(getattr(SportBackend(), "claim_on_start", False))
        self.assertFalse(getattr(AvoidBackend(), "claim_on_start", False))

    def test_move_is_plain_sport_move_once_claimed(self):
        b = SportPrepBackend()
        b.claim()
        (svc, api, p), = params(b.move(0.0, 0.0, 0.2))
        self.assertEqual((svc, api, p), (SPORT, SPORT_MOVE, {"x": 0.0, "y": 0.0, "z": 0.2}))

    def test_unclaimed_move_claims_first(self):
        self.assertEqual([r[1] for r in SportPrepBackend().move(0.2, 0, 0)],
                         [SPORT_SWITCH_JOYSTICK, SPORT_FREE_WALK, SPORT_MOVE])

    def test_stop_keeps_sticks_off(self):
        b = SportPrepBackend()
        b.claim()
        self.assertEqual(b.stop(), [(SPORT, SPORT_STOP_MOVE, "")])
        self.assertTrue(b.holding)

    def test_release_stops_then_gives_sticks_back(self):
        b = SportPrepBackend()
        b.claim()
        reqs = params(b.release())
        self.assertEqual([r[1] for r in reqs], [SPORT_STOP_MOVE, SPORT_SWITCH_JOYSTICK])
        self.assertEqual(reqs[1][2], {"data": True})
        self.assertFalse(b.holding)

    def test_release_without_claim_still_stops(self):
        self.assertEqual(SportPrepBackend().release(), [(SPORT, SPORT_STOP_MOVE, "")])

    def test_reclaims_after_release(self):
        b = SportPrepBackend()
        b.claim()
        b.release()
        self.assertEqual([r[1] for r in b.move(0.1, 0, 0)][:2],
                         [SPORT_SWITCH_JOYSTICK, SPORT_FREE_WALK])


class TestFactory(unittest.TestCase):
    def test_names(self):
        self.assertEqual(make_backend("sport").name, "sport")
        self.assertEqual(make_backend("avoid").name, "avoid")
        self.assertEqual(make_backend("sport_prep").name, "sport_prep")

    def test_unknown_refused(self):
        with self.assertRaises(ValueError):
            make_backend("teleport")


if __name__ == "__main__":
    unittest.main(verbosity=2)
