"""Tests for starting/aborting the loop from the Go2 remote. Stdlib only.

    python tools/tests/test_remote_keys.py
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from remote_keys import (KEY_BITS, RemoteTrigger, action_for, decode,  # noqa: E402
                         parse_combo, state_of)


def keys(*names):
    return sum(1 << KEY_BITS[n] for n in names)


START = keys("SELECT", "UP")


def run(trig, samples, dt=0.02, t0=0.0):
    """Feed (keys, sticks) samples at dt; return [(t, event)] for non-None events."""
    out = []
    for i, s in enumerate(samples):
        k, st = s if isinstance(s, tuple) else (s, (0.0, 0.0, 0.0, 0.0))
        t = t0 + i * dt
        e = trig.feed(t, k, st)
        if e:
            out.append((round(t, 3), e))
    return out


class TestDecode(unittest.TestCase):
    def test_decode_bits(self):
        self.assertEqual(decode(0), frozenset())
        self.assertEqual(decode(1 << 2 | 1 << 5), {"START", "L2"})
        self.assertEqual(decode(0xFFFF), set(KEY_BITS))

    def test_parse_combo(self):
        self.assertEqual(parse_combo("select + Up"), {"SELECT", "UP"})
        with self.assertRaises(ValueError):
            parse_combo("select+jump")
        with self.assertRaises(ValueError):
            parse_combo(" + ")

    def test_start_containing_abort_rejected(self):
        with self.assertRaises(ValueError):
            RemoteTrigger(start="L2+START+A")
        with self.assertRaises(ValueError):
            RemoteTrigger(start="SELECT+DOWN+X", abort="SELECT+DOWN")


class TestStart(unittest.TestCase):
    def test_hold_fires_once(self):
        ev = run(RemoteTrigger(hold_s=1.0), [START] * 150)   # 3 s held
        self.assertEqual([e for _, e in ev], ["start"])
        self.assertAlmostEqual(ev[0][0], 1.0, places=2)

    def test_short_press_does_nothing(self):
        self.assertEqual(run(RemoteTrigger(hold_s=1.0), [START] * 40 + [0] * 100), [])

    def test_extra_button_is_not_the_combo(self):
        extra = START | keys("A")
        self.assertEqual(run(RemoteTrigger(hold_s=1.0), [extra] * 150), [])

    def test_release_rearms(self):
        ev = run(RemoteTrigger(hold_s=0.5), [START] * 50 + [0] * 5 + [START] * 50)
        self.assertEqual([e for _, e in ev], ["start", "start"])

    def test_flicker_restarts_hold(self):
        # 0.8 s + 0.8 s held with one released sample between: never 1 s continuous
        self.assertEqual(run(RemoteTrigger(hold_s=1.0), [START] * 40 + [0] + [START] * 40), [])

    def test_stream_gap_restarts_hold(self):
        trig = RemoteTrigger(hold_s=1.0, max_gap_s=0.5)
        self.assertIsNone(trig.feed(0.0, START))
        self.assertIsNone(trig.feed(0.1, START))
        self.assertIsNone(trig.feed(1.5, START))      # 1.4 s gap: hold restarts here
        self.assertIsNone(trig.feed(2.0, START))
        self.assertIsNone(trig.feed(2.4, START))      # 0.9 s since the restart
        self.assertEqual(trig.feed(2.6, START), "start")

    def test_clock_going_backwards_restarts_hold(self):
        trig = RemoteTrigger(hold_s=1.0)
        trig.feed(10.0, START)
        trig.feed(10.2, START)
        self.assertIsNone(trig.feed(5.0, START))
        self.assertIsNone(trig.feed(5.4, START))


class TestAbort(unittest.TestCase):
    def test_abort_combo_immediate(self):
        ev = run(RemoteTrigger(), [0, keys("SELECT", "DOWN")] + [0] * 3)
        self.assertEqual(ev, [(0.02, "abort")])

    def test_hard_stop_aborts_even_with_extras(self):
        # L2+B is the robot's damping stop (verified 2026-10-28)
        ev = run(RemoteTrigger(), [keys("L2", "B", "R1")] * 10)
        self.assertEqual([e for _, e in ev], ["abort"])   # once, not every sample

    def test_mode_toggle_also_aborts(self):
        # L2+START doesn't damp the robot, but still means "operator wants control"
        ev = run(RemoteTrigger(), [keys("L2", "START")] * 10)
        self.assertEqual([e for _, e in ev], ["abort"])

    def test_stick_aborts(self):
        ev = run(RemoteTrigger(stick_abort=0.5), [(0, (0.0, 0.7, 0.0, 0.0))])
        self.assertEqual([e for _, e in ev], ["abort"])
        ev = run(RemoteTrigger(stick_abort=0.5), [(0, (0.0, 0.0, -0.6, 0.0))])
        self.assertEqual([e for _, e in ev], ["abort"])

    def test_stick_noise_ignored(self):
        self.assertEqual(run(RemoteTrigger(), [(0, (0.05, -0.1, 0.02, 0.0))] * 50), [])

    def test_abort_beats_start_and_resets_hold(self):
        trig = RemoteTrigger(hold_s=1.0)
        samples = [START] * 40 + [(START, (0.9, 0.0, 0.0, 0.0))] + [START] * 40
        ev = run(trig, samples)
        self.assertEqual([e for _, e in ev], ["abort"])   # hold restarted after the abort

    def test_abort_refires_after_release(self):
        ab = keys("SELECT", "DOWN")
        ev = run(RemoteTrigger(), [ab, ab, 0, ab])
        self.assertEqual([e for _, e in ev], ["abort", "abort"])


class TestActions(unittest.TestCase):
    def test_start_actions(self):
        self.assertEqual(action_for("start", "idle"), "start")
        self.assertEqual(action_for("start", "paused"), "resume")
        self.assertIsNone(action_for("start", "running"))
        self.assertIsNone(action_for("start", "done"))
        self.assertIsNone(action_for("start", None))      # never start blind

    def test_abort_actions(self):
        for s in ("running", "paused", "idle", None):
            self.assertEqual(action_for("abort", s), "stop")
        self.assertIsNone(action_for("abort", "done"))
        self.assertIsNone(action_for("abort", "aborted"))

    def test_state_of(self):
        self.assertEqual(state_of("running | leg 1 | send vx=0.20"), "running")
        self.assertEqual(state_of("idle | odom 0.1 s"), "idle")
        self.assertIsNone(state_of(""))


if __name__ == "__main__":
    unittest.main(verbosity=2)
