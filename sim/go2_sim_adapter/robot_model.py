"""What the simulated Go2 does with Unitree API requests and the remote. Pure.

The Gazebo Go2 (CHAMP controller) only takes a velocity. The real robot
takes Unitree service requests and its handheld remote. This model sits
between them so our real code -- bridge, loop, turn test, demo -- runs
against the sim unchanged. It is a *stand-in*, not Unitree's firmware:

  * sport (/api/sport/request): Damp, BalanceStand, StopMove, StandUp/Down,
    RecoveryStand, Move, SwitchJoystick, FreeWalk/ClassicWalk
  * obstacles_avoid: SwitchSet/Get, UseRemoteCommandFromApi, Move
  * motion_switcher CheckMode -> "normal"
  * the remote: L2+START damps (always, as we assume the real one does),
    L2+A stands, sticks walk unless switched off or the API holds control

Deliberate worst case: a Move holds until StopMove (``move_timeout_s=0``),
so only *our* watchdog stops the sim robot when commands cease -- the thing
the turn test is meant to prove. Real-robot behaviour that the sim cannot
answer (walking-mode quirks, the Stage 1 failure) stays a robot question.
Stdlib only.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

# sport
DAMP, BALANCE_STAND, STOP_MOVE, STAND_UP, STAND_DOWN, RECOVERY_STAND = 1001, 1002, 1003, 1004, 1005, 1006
MOVE, SWITCH_JOYSTICK, FREE_WALK, CLASSIC_WALK = 1008, 1027, 2045, 2049
# obstacles_avoid
AV_SWITCH_SET, AV_SWITCH_GET, AV_MOVE, AV_USE_REMOTE = 1001, 1002, 1003, 1004
# motion_switcher
MS_CHECK_MODE = 1001

KEY = {"R1": 0, "L1": 1, "START": 2, "SELECT": 3, "R2": 4, "L2": 5,
       "A": 8, "B": 9, "X": 10, "Y": 11, "UP": 12, "RIGHT": 13, "DOWN": 14, "LEFT": 15}

# SportModeState.mode values used here (0 idle, 1 balance stand, 3 locomotion, 7 damping)
MODE_IDLE, MODE_STAND, MODE_WALK, MODE_DAMP = 0, 1, 3, 7


def _json(p: str) -> dict:
    try:
        v = json.loads(p) if p else {}
        return v if isinstance(v, dict) else {}
    except ValueError:
        return {}


def _held(keys: int, *names) -> bool:
    return all(keys >> KEY[n] & 1 for n in names)


@dataclass
class RobotModel:
    max_stick_v: float = 0.5            # m/s at full stick
    max_stick_w: float = 0.8            # rad/s at full stick
    stick_deadzone: float = 0.1
    move_timeout_s: float = 0.0         # 0 = a Move holds until StopMove
    standing: bool = True               # the sim spawns standing
    damped: bool = False
    joystick_on: bool = True
    api_holds: bool = False             # obstacles_avoid UseRemoteCommandFromApi
    avoid_enabled: bool = True
    api_vel: tuple = (0.0, 0.0, 0.0)
    api_t: float = 0.0
    stick_vel: tuple | None = None
    log: list = field(default_factory=list)

    # ------------------------------------------------------------ requests
    def sport(self, api_id: int, parameter: str, now: float):
        """Returns (status_code, response_data_json)."""
        p = _json(parameter)
        if api_id == DAMP:
            self._damp("sport Damp")
        elif api_id in (BALANCE_STAND, STAND_UP, RECOVERY_STAND):
            self.damped, self.standing = False, True
        elif api_id == STAND_DOWN:
            self.standing, self.api_vel = False, (0.0, 0.0, 0.0)
        elif api_id == STOP_MOVE:
            self.api_vel = (0.0, 0.0, 0.0)
        elif api_id == MOVE:
            self._api_move(p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0), now)
        elif api_id == SWITCH_JOYSTICK:
            self.joystick_on = bool(p.get("data", True))
            self.log.append(f"joystick {'on' if self.joystick_on else 'off'}")
        return 0, ""

    def avoid(self, api_id: int, parameter: str, now: float):
        p = _json(parameter)
        if api_id == AV_SWITCH_SET:
            self.avoid_enabled = bool(p.get("enable", True))
        elif api_id == AV_SWITCH_GET:
            return 0, json.dumps({"enable": self.avoid_enabled})
        elif api_id == AV_USE_REMOTE:
            self.api_holds = bool(p.get("is_remote_commands_from_api", False))
        elif api_id == AV_MOVE:
            self._api_move(p.get("x", 0.0), p.get("y", 0.0), p.get("yaw", 0.0), now)
        return 0, ""

    @staticmethod
    def motion_switcher(api_id: int, parameter: str, now: float):
        if api_id == MS_CHECK_MODE:
            return 0, json.dumps({"name": "normal", "form": "0"})
        return 0, ""

    def _api_move(self, x, y, z, now):
        if self.damped or not self.standing:
            return                      # a lying / limp robot ignores Move
        self.api_vel, self.api_t = (float(x), float(y), float(z)), now

    def _damp(self, why: str):
        if not self.damped:
            self.log.append(f"damped ({why})")
        self.damped, self.api_vel, self.stick_vel = True, (0.0, 0.0, 0.0), None

    # -------------------------------------------------------------- remote
    def remote(self, keys: int, lx: float, ly: float, rx: float, ry: float):
        if _held(keys, "L2", "START"):
            self._damp("remote L2+START")
            return
        if _held(keys, "L2", "A"):
            self.damped, self.standing = False, True
        sticks_count = self.joystick_on and not self.api_holds
        if sticks_count and max(abs(lx), abs(ly), abs(rx)) > self.stick_deadzone:
            self.stick_vel = (ly * self.max_stick_v, -lx * self.max_stick_v,
                              -rx * self.max_stick_w)
        else:
            self.stick_vel = None

    # ------------------------------------------------------------- outputs
    def command(self, now: float):
        """Velocity for the sim's controller right now."""
        if self.damped or not self.standing:
            return (0.0, 0.0, 0.0)
        if self.stick_vel is not None:
            return self.stick_vel       # a hand on the sticks wins, as on the robot
        if self.move_timeout_s > 0 and now - self.api_t > self.move_timeout_s:
            return (0.0, 0.0, 0.0)
        return self.api_vel

    def mode(self, now: float) -> int:
        if self.damped:
            return MODE_DAMP
        if not self.standing:
            return MODE_IDLE
        return MODE_WALK if any(abs(v) > 1e-3 for v in self.command(now)) else MODE_STAND

    def body_height(self) -> float:
        """The Go2's standing height when standing. The CHAMP sim robot
        stands at ~0.21 m, below the 0.25 m preflight reads as 'lying down'
        on the real robot; only standing vs not is meaningful here."""
        return 0.08 if self.damped or not self.standing else 0.32
