"""Loop geometry: poses, the rectangle, frame transforms. Pure, no ROS.

Conventions follow REP-103: x forward, y left, yaw counter-clockwise. The loop
is defined in the robot's *start* frame -- "4 m ahead, then 3 m to the left"
-- and transformed into the odometry frame once, at the moment the mission is
started. Defining it relative to where the robot actually is, rather than in
absolute odom coordinates, is what lets the same parameters work wherever you
put the robot down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def wrap_angle(a: float) -> float:
    """Angle into [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading from a ROS orientation quaternion (roll/pitch ignored)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rectangle_local(length: float, width: float, turn: str = "left", laps: int = 1):
    """Corner waypoints of a rectangle in the start frame, ending at the start.

    ``length`` is along the robot's initial heading, ``width`` to the side it
    turns towards. Zero or negative sizes are refused rather than defaulted:
    an autonomous loop must never run on a size nobody chose.
    """
    if not (length > 0 and width > 0):
        raise ValueError(f"loop size must be positive, got {length} x {width}")
    if turn not in ("left", "right"):
        raise ValueError(f"turn must be 'left' or 'right', got {turn!r}")
    if laps < 1:
        raise ValueError(f"laps must be >= 1, got {laps}")
    side = 1.0 if turn == "left" else -1.0
    lap = [(length, 0.0), (length, side * width), (0.0, side * width), (0.0, 0.0)]
    return lap * laps


def to_world(start: Pose2D, local_points):
    """Start-frame points -> odometry-frame points."""
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    return [(start.x + c * x - s * y, start.y + s * x + c * y) for x, y in local_points]


def to_robot(pose: Pose2D, px: float, py: float):
    """An odometry-frame point -> the robot's frame (x ahead, y left)."""
    dx, dy = px - pose.x, py - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy
