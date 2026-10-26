"""Which Unitree service the bridge drives the robot through. Pure: each
method returns the requests to publish as (service, api_id, parameter)
tuples, so the sequencing is unit-testable without a robot.

Two backends:

* ``sport`` (default) -- sport-mode Move (1008) on /api/sport/request. What
  the bridge has always done. Unitree's built-in obstacle avoidance is not
  known to filter it.
* ``avoid`` -- Unitree's obstacle-avoidance Move (1003) on
  /api/obstacles_avoid/request, so the robot's own avoidance applies on top of
  our LiDAR stop. Sequence from Unitree's official example,
  unitree_sdk2_python example/obstacles_avoid/obstacles_avoid_move.py:
      SwitchSet(true); UseRemoteCommandFromApi(true); Move(x, y, yaw, mode 0)...;
      Move(0, 0, 0); UseRemoteCommandFromApi(false)

``UseRemoteCommandFromApi(true)`` lets the API *override the handheld
remote*. So the avoid backend claims control only when it actually has motion
to send, and releases it -- handing the robot back to the remote -- on every
stop that is not just a pause: watchdog, e-stop, damp, shutdown. Whether the
remote's L2+START damping still works while the API holds control is not
documented; it must be verified on the robot before this backend moves it.
"""

from __future__ import annotations

import json

SPORT = "sport"
AVOID = "obstacles_avoid"

# sport (unitree_sdk2 go2/sport/sport_api.hpp)
SPORT_MOVE = 1008
SPORT_STOP_MOVE = 1003

# obstacles_avoid (unitree_sdk2 go2/obstacles_avoid/obstacles_avoid_api.hpp)
AVOID_SWITCH_SET = 1001
AVOID_MOVE = 1003
AVOID_USE_REMOTE_COMMAND_FROM_API = 1004
AVOID_MODE_VELOCITY = 0


def _f(v: float) -> float:
    v = float(v)
    return v if v == v and abs(v) != float("inf") else 0.0   # NaN/inf -> 0


class SportBackend:
    name = "sport"

    def claim(self):
        return []

    def move(self, vx, vy, vyaw):
        return [(SPORT, SPORT_MOVE,
                 json.dumps({"x": _f(vx), "y": _f(vy), "z": _f(vyaw)}))]

    def stop(self):
        return [(SPORT, SPORT_STOP_MOVE, "")]

    def release(self):
        return self.stop()


class AvoidBackend:
    """Tracks whether it currently holds control so claim/release are
    idempotent: claiming twice sends nothing new; releasing when not held
    still sends a zero Move (stopping is never skipped)."""

    name = "avoid"

    def __init__(self) -> None:
        self.holding = False

    def claim(self):
        if self.holding:
            return []
        self.holding = True
        return [
            (AVOID, AVOID_SWITCH_SET, json.dumps({"enable": True})),
            (AVOID, AVOID_USE_REMOTE_COMMAND_FROM_API,
             json.dumps({"is_remote_commands_from_api": True})),
        ]

    def move(self, vx, vy, vyaw):
        return self.claim() + [(AVOID, AVOID_MOVE, json.dumps(
            {"x": _f(vx), "y": _f(vy), "yaw": _f(vyaw), "mode": AVOID_MODE_VELOCITY}))]

    def stop(self):
        """Zero velocity, keep control (a pause between commands)."""
        return [(AVOID, AVOID_MOVE, json.dumps(
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": AVOID_MODE_VELOCITY}))]

    def release(self):
        """Zero velocity, then hand the robot back to the remote."""
        out = self.stop()
        if self.holding:
            out.append((AVOID, AVOID_USE_REMOTE_COMMAND_FROM_API,
                        json.dumps({"is_remote_commands_from_api": False})))
            self.holding = False
        return out


def make_backend(name: str):
    if name == "sport":
        return SportBackend()
    if name == "avoid":
        return AvoidBackend()
    raise ValueError(f"backend must be 'sport' or 'avoid', got {name!r}")
