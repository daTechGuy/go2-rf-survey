#!/usr/bin/env python3
"""Start and abort the autonomous loop from the Go2's own remote. Pure logic.

The handheld remote talks to the robot by radio, so it works anywhere in the
room -- unlike the laptop, which needs the Ethernet cable. The robot already
republishes the remote on /wirelesscontroller (unitree_go/WirelessController:
lx, ly, rx, ry sticks and a 16-bit ``keys`` field). remote_trigger.py feeds
those messages to RemoteTrigger and turns its events into loop services.

Rules, chosen so a stray press cannot start a walk:
  * START = the start combo held EXACTLY (no other buttons) for hold_s, with
    no gap in the stream. Fires once; the combo must be released to re-arm.
    Pressed again while the mission is paused, it resumes.
  * ABORT = the abort combo, OR L2+B (the robot's own damping stop, which the
    operator already reaches for), OR L2+START (a walking-mode toggle on the
    robot -- not a stop, but a sign the operator wants control), OR any stick
    pushed past stick_abort. Fires on the press, no hold.
    Extra buttons do not mask an abort. Abort beats start in the same sample.

Key bits follow unitree_sdk2's xKeySwitchUnion. They are verified on the
real remote with `go2loop.sh remote-keys` (read-only) before first use.
Stdlib only, no clock: the caller passes time in.
"""

from __future__ import annotations

KEY_BITS = {
    "R1": 0, "L1": 1, "START": 2, "SELECT": 3, "R2": 4, "L2": 5, "F1": 6, "F2": 7,
    "A": 8, "B": 9, "X": 10, "Y": 11, "UP": 12, "RIGHT": 13, "DOWN": 14, "LEFT": 15,
}

DEFAULT_START = "SELECT+UP"
DEFAULT_ABORT = "SELECT+DOWN"
HARD_STOP = "L2+B"          # damps: verified on the robot 2026-10-28
MODE_TOGGLE = "L2+START"    # does NOT damp on the robot (2026-10-28)


def decode(keys: int) -> frozenset:
    """Names of the buttons set in a ``keys`` bitfield."""
    return frozenset(n for n, b in KEY_BITS.items() if keys >> b & 1)


def parse_combo(text: str) -> frozenset:
    """'select+up' -> {'SELECT', 'UP'}; raises ValueError on unknown names."""
    names = frozenset(p.strip().upper() for p in text.split("+") if p.strip())
    if not names:
        raise ValueError(f"empty button combo: {text!r}")
    unknown = sorted(names - KEY_BITS.keys())
    if unknown:
        raise ValueError(f"unknown button(s) {unknown} in {text!r}; "
                         f"known: {', '.join(KEY_BITS)}")
    return names


class RemoteTrigger:
    def __init__(self, start: str = DEFAULT_START, abort: str = DEFAULT_ABORT,
                 hold_s: float = 1.5, stick_abort: float = 0.5,
                 max_gap_s: float = 0.5) -> None:
        self.start = parse_combo(start)
        self.aborts = (parse_combo(abort), parse_combo(HARD_STOP),
                       parse_combo(MODE_TOGGLE))
        for a in self.aborts:
            if a <= self.start:
                raise ValueError(f"start combo {'+'.join(sorted(self.start))} contains "
                                 f"abort combo {'+'.join(sorted(a))}")
        self.hold_s = hold_s
        self.stick_abort = stick_abort
        self.max_gap_s = max_gap_s
        self._held_since: float | None = None
        self._fired = False          # start fired; waiting for release
        self._aborting = False       # abort fired; waiting for release
        self._last_t: float | None = None

    def feed(self, t: float, keys: int, sticks=(0.0, 0.0, 0.0, 0.0)):
        """One remote sample -> 'start', 'abort' or None."""
        gap = self._last_t is not None and (t - self._last_t > self.max_gap_s
                                            or t < self._last_t)
        self._last_t = t
        pressed = decode(keys)

        abort_now = (any(a <= pressed for a in self.aborts)
                     or max(abs(float(s)) for s in sticks) > self.stick_abort)
        if abort_now:
            self._held_since = None
            if not self._aborting:
                self._aborting = True
                return "abort"
            return None
        self._aborting = False

        if pressed != self.start:
            self._held_since = None
            self._fired = False
            return None
        if self._fired:
            return None
        if self._held_since is None or gap:
            self._held_since = t     # a gap in the stream restarts the hold
            return None
        if t - self._held_since >= self.hold_s:
            self._fired = True
            return "start"
        return None


def action_for(event, state):
    """Loop service to call for an event, given the loop's state (or None).

    Start never acts blind (unknown state -> nothing). Abort always calls
    stop unless the mission is already over: stop is harmless when idle.
    """
    if event == "start":
        return {"idle": "start", "paused": "resume"}.get(state)
    if event == "abort":
        return None if state in ("done", "aborted") else "stop"
    return None


def state_of(status: str):
    """'running | ...' -> 'running'; None for an empty status."""
    head = status.split("|", 1)[0].strip()
    return head or None
