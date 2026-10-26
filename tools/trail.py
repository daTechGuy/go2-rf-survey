"""The robot's trail for the live view: keep a pose only when the robot has
moved enough to matter. Pure, stdlib. At ~150 Hz odometry a stationary robot
would otherwise add 150 identical points a second to the path."""

from __future__ import annotations

import math


class Trail:
    def __init__(self, step_m: float = 0.05, turn_rad: float = 0.15, max_points: int = 20000):
        self.step_m = step_m
        self.turn_rad = turn_rad
        self.max_points = max_points
        self.points: list = []          # (x, y, yaw)

    def add(self, x: float, y: float, yaw: float) -> bool:
        """Append if moved >= step_m or turned >= turn_rad since the last kept
        point. Returns True when kept. Non-finite poses are never kept."""
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return False
        if self.points:
            px, py, pyaw = self.points[-1]
            d_yaw = abs((yaw - pyaw + math.pi) % (2 * math.pi) - math.pi)
            # 1e-9 tolerance: a gap of exactly step_m can compute as 4.9999 cm
            # and would otherwise stretch the spacing to the next sample
            if math.hypot(x - px, y - py) < self.step_m - 1e-9 \
                    and d_yaw < self.turn_rad - 1e-9:
                return False
        self.points.append((x, y, yaw))
        if len(self.points) > self.max_points:      # keep the start, thin the middle
            self.points = [self.points[0]] + self.points[2::2]
        return True

    def length(self) -> float:
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(self.points, self.points[1:]))

    def reset(self) -> None:
        self.points = []
