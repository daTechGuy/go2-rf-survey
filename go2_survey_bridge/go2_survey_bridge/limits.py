"""Velocity limiting and command watchdog.

Deliberately free of ROS and of wall-clock reads: every function takes the
times and deltas it needs as arguments. This is what makes the safety logic
testable -- a watchdog you cannot test is not a safety feature, it is a hope.

Three guards, each for a different failure:

* **clamp** -- a planner emitting a nonsense setpoint. Nav2 misconfigured with
  a 2 m/s max will happily ask for it.
* **slew** -- a legitimate but abrupt setpoint. A quadruped handed an instant
  0 -> 0.6 m/s step can stumble; wheeled robots tolerate this and legged ones
  do not.
* **watchdog** -- the planner dying, the network dropping, a container being
  killed. Without this the robot keeps executing the last velocity it was
  given, which is how robots walk down stairs unattended.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, limit: float) -> float:
    """Clamp to +/- abs(limit). NaN becomes 0.0.

    NaN is treated as a zero rather than passed through: a NaN reaching the
    robot is unpredictable, and the only safe interpretation of "no number"
    in a velocity command is "do not move".
    """
    if value is None or math.isnan(value):
        return 0.0
    lim = abs(limit)
    if math.isinf(value):
        return math.copysign(lim, value)
    return max(-lim, min(lim, value))


@dataclass(frozen=True)
class Velocity:
    """Body-frame velocity: +x forward, +y left, +z yaw CCW."""

    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.vyaw == 0.0

    def speed(self) -> float:
        """Translational magnitude, ignoring yaw."""
        return math.hypot(self.vx, self.vy)


@dataclass(frozen=True)
class Limits:
    """Absolute ceilings and per-second rates of change.

    Defaults are deliberately well under what the robot can do. A survey run
    wants a slow, steady platform: speed costs data quality (risk R4) and buys
    nothing, because the robot stops to scan anyway.
    """

    max_vx: float = 0.6        # m/s forward/back
    max_vy: float = 0.4        # m/s lateral
    max_vyaw: float = 0.8      # rad/s
    accel_xy: float = 0.8      # m/s^2
    accel_yaw: float = 1.6     # rad/s^2

    def clamp_velocity(self, v: Velocity) -> Velocity:
        return Velocity(
            clamp(v.vx, self.max_vx),
            clamp(v.vy, self.max_vy),
            clamp(v.vyaw, self.max_vyaw),
        )


def _approach(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


class VelocityLimiter:
    """Clamps to limits, then slews toward the target at a bounded rate.

    Stateful across calls (it remembers what it last emitted), but
    deterministic: same previous output, same target, same dt, same result.
    """

    def __init__(self, limits: Limits | None = None) -> None:
        self.limits = limits or Limits()
        self._last = Velocity()

    @property
    def current(self) -> Velocity:
        return self._last

    def reset(self) -> None:
        """Forget history. Call when the robot stops, so the next command
        ramps from rest rather than from a stale value."""
        self._last = Velocity()

    def step(self, target: Velocity, dt: float) -> Velocity:
        if dt <= 0.0:
            return self._last
        t = self.limits.clamp_velocity(target)
        d_xy = self.limits.accel_xy * dt
        d_yaw = self.limits.accel_yaw * dt
        self._last = Velocity(
            _approach(self._last.vx, t.vx, d_xy),
            _approach(self._last.vy, t.vy, d_xy),
            _approach(self._last.vyaw, t.vyaw, d_yaw),
        )
        return self._last

    def stop_immediately(self) -> Velocity:
        """Jump to zero, bypassing the slew limit.

        Correct for the watchdog and e-stop paths: ramping down over half a
        second is exactly wrong when the reason for stopping is that something
        has gone wrong.
        """
        self._last = Velocity()
        return self._last


class Watchdog:
    """Tracks command freshness. Never reads a clock itself.

    The caller supplies monotonic timestamps, which is what makes this
    testable -- and also what makes it correct under simulated time, where
    wall-clock reads silently do the wrong thing.
    """

    def __init__(self, timeout_s: float = 0.3) -> None:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = timeout_s
        self._last_cmd_t: float | None = None
        self._tripped = False
        self.trip_count = 0

    @property
    def tripped(self) -> bool:
        return self._tripped

    def feed(self, now: float) -> bool:
        """Record a command arriving. Returns True if this cleared a trip."""
        self._last_cmd_t = now
        if self._tripped:
            self._tripped = False
            return True
        return False

    def age(self, now: float) -> float | None:
        """Seconds since the last command, or None if none has ever arrived."""
        if self._last_cmd_t is None:
            return None
        return now - self._last_cmd_t

    def check(self, now: float) -> bool:
        """Evaluate freshness. Returns True on the *transition* into tripped.

        Edge-triggered on purpose: the caller sends one StopMove when the trip
        happens rather than flooding the robot with them every tick while the
        planner stays dead.
        """
        age = self.age(now)
        if age is None:
            return False               # nothing sent yet; nothing to guard
        if age > self.timeout_s and not self._tripped:
            self._tripped = True
            self.trip_count += 1
            return True
        return False
