"""Which Unitree service the bridge drives the robot through. Pure: each
method returns the requests to publish as (service, api_id, parameter)
tuples, so the sequencing is unit-testable without a robot.

Three backends:

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

* ``sport_prep`` -- sport-mode Move, preceded by the walk preparation that
  dancher00/go2-nav2-wifi (MIT) sends before driving a Go2 Edu from /cmd_vel,
  ws/scripts/robot_sport_bridge.py:
      BalanceStand; RecoveryStand; SwitchJoystick(false); FreeWalk
  with the note "Release RC / app joystick and enable velocity walk (required
  after RC use)" and, in robot-check-cmdvel.sh, "If hz>0 but robot jerks:
  ... (FreeWalk + SwitchJoystick)" -- the symptom of our first Stage 1 run.
  RecoveryStand is left out (it can stand a robot up: motion we did not ask
  for); BalanceStand is already sent by the bridge at startup. They never
  hand the sticks back; this backend does, with SwitchJoystick(true), on
  every release -- same claim/release discipline as ``avoid``. Whether
  L2+START still damps while the sticks are switched off must be verified on
  the robot first (``go2loop.sh joystick hold``).
"""

from __future__ import annotations

import json

SPORT = "sport"
AVOID = "obstacles_avoid"

# sport (unitree_sdk2 go2/sport/sport_api.hpp)
SPORT_MOVE = 1008
SPORT_STOP_MOVE = 1003
SPORT_SWITCH_JOYSTICK = 1027     # {"data": false} = ignore the remote's sticks
SPORT_FREE_WALK = 2045

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


class SportPrepBackend(SportBackend):
    """Sport Move after SwitchJoystick(false) + FreeWalk. Holds the sticks
    only between claim and release, like AvoidBackend holds control.

    ``claim_on_start``: the bridge claims right after its startup
    BalanceStand, so the walk-mode switch is done before the first Move
    (a turn test measures the response to that first Move). The first
    release -- watchdog, e-stop, damp, shutdown -- hands the sticks back."""

    name = "sport_prep"
    claim_on_start = True

    def __init__(self) -> None:
        self.holding = False

    def claim(self):
        if self.holding:
            return []
        self.holding = True
        return [
            (SPORT, SPORT_SWITCH_JOYSTICK, json.dumps({"data": False})),
            (SPORT, SPORT_FREE_WALK, ""),
        ]

    def move(self, vx, vy, vyaw):
        return self.claim() + super().move(vx, vy, vyaw)

    def release(self):
        """StopMove, then give the sticks back to the remote."""
        out = self.stop()
        if self.holding:
            out.append((SPORT, SPORT_SWITCH_JOYSTICK, json.dumps({"data": True})))
            self.holding = False
        return out


BACKENDS = ("sport", "avoid", "sport_prep")


def make_backend(name: str):
    if name == "sport":
        return SportBackend()
    if name == "avoid":
        return AvoidBackend()
    if name == "sport_prep":
        return SportPrepBackend()
    raise ValueError(f"backend must be one of {', '.join(BACKENDS)}, got {name!r}")
