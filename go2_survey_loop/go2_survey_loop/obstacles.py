"""Obstacle detection from the LiDAR cloud. Pure, no ROS, no numpy.

The cloud arrives in the odometry frame. Each point is moved into the robot's
own frame using the current pose and sorted into two regions:

* **front lane** -- a box from the robot's nose out to ``stop_distance``
  ahead, as wide as the robot plus margin. Anything here means "do not drive
  forward into this".
* **clearance ring** -- anywhere within ``clearance_radius`` of the robot's
  centre, in any direction. Catches someone stepping in beside the robot, and
  covers turning in place, where the front lane is the wrong question.

Points inside the robot's own footprint are discarded first: a head-mounted
LiDAR can see its own legs, and counting those would make the robot refuse
to move forever. Points outside a height band above the floor are discarded
too -- the floor is not an obstacle and neither is the ceiling.

This is a *stop* check, not a planner. It never steers around anything.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from go2_survey_loop.geometry import Pose2D


# The standing Go2, measured by dancher00/go2-nav2-wifi (MIT) from the URDF
# visual meshes over 33 live joint samples (docs/GO2-FOOTPRINT.md there,
# 2026-09-11), in base_link: x -0.353..+0.332, y -0.170..+0.193 m. Their
# padded Nav2 envelope (shared with go2_ros2_sdk and BotBrain) is
# x -0.45..+0.36, y +-0.20, padding 0.03. test/test_footprint.py holds the
# defaults below to these numbers.


@dataclass(frozen=True)
class ObstacleParams:
    body_front: float = 0.35        # m, robot centre to nose (measured 0.332)
    body_rear: float = 0.40         # m, centre to tail (measured 0.353; +leg swing)
    body_half_width: float = 0.20   # m (measured 0.193)
    stop_distance: float = 0.50     # m beyond the nose
    lane_half_width: float = 0.35   # m, half-width of the forward lane
    clearance_radius: float = 0.55  # m from centre, all directions
    z_min: float = 0.10             # m above floor
    z_max: float = 1.00             # m above floor


def classify(points, pose: Pose2D, floor_z: float, p: ObstacleParams = ObstacleParams()):
    """Count obstacle points as (front_lane, clearance_ring).

    ``points`` is any iterable of (x, y, z) in the odometry frame.
    """
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    r2 = p.clearance_radius * p.clearance_radius
    front = near = 0
    for x, y, z in points:
        h = z - floor_z
        if h < p.z_min or h > p.z_max:
            continue
        dx, dy = x - pose.x, y - pose.y
        xr = c * dx + s * dy
        yr = -s * dx + c * dy
        if -p.body_rear <= xr <= p.body_front and abs(yr) <= p.body_half_width:
            continue                            # the robot itself
        if p.body_front < xr <= p.body_front + p.stop_distance \
                and abs(yr) <= p.lane_half_width:
            front += 1
        if xr * xr + yr * yr <= r2:
            near += 1
    return front, near


def floor_from_scan(points, pose: Pose2D, radius: float = 2.0,
                    percentile: float = 2.0, min_points: int = 50):
    """Floor height from the low tail of z near the robot, or None if the
    scan has too few nearby points to say. The robot's LiDAR sees the floor
    all around it, so the low end of the nearby z distribution is the floor;
    a percentile rather than the minimum ignores stray below-floor returns."""
    r2 = radius * radius
    zs = sorted(z for x, y, z in points
                if (x - pose.x) ** 2 + (y - pose.y) ** 2 <= r2)
    if len(zs) < min_points:
        return None
    k = min(len(zs) - 1, max(0, int(len(zs) * percentile / 100.0)))
    return zs[k]


def median(values):
    v = sorted(values)
    if not v:
        return None
    m = len(v) // 2
    return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])
