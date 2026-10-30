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


def ring_count(points_xy, pose: Pose2D, radius: float, p: ObstacleParams = ObstacleParams()) -> int:
    """Remembered obstacle points (odom-frame x, y) within ``radius`` of the
    robot's centre, outside its own footprint. For the turn check: the LiDAR
    on the head sees poorly behind the body, so points seen in the last few
    seconds (now behind it) must count too (2026-10-31: a brisk turn swung the
    rear into a desk)."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    r2 = radius * radius
    n = 0
    for x, y in points_xy:
        dx, dy = x - pose.x, y - pose.y
        if dx * dx + dy * dy > r2:
            continue
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        if -p.body_rear <= xr <= p.body_front and abs(yr) <= p.body_half_width:
            continue
        n += 1
    return n


class ObstacleMemory:
    """Obstacle points (odom-frame x, y) seen in the last ``keep_s`` seconds,
    one per ``cell`` square, each stamped with when it was last seen. What the
    turn check counts (ring_count): the head LiDAR loses sight of things once
    they're behind the body, but a turn on the spot still sweeps them."""

    def __init__(self, keep_s: float = 10.0, cell: float = 0.05) -> None:
        self.keep_s, self.cell = keep_s, cell
        self._cells: dict = {}

    def add(self, points_xy, now: float) -> None:
        c = self.cell
        for x, y in points_xy:
            self._cells[(round(x / c), round(y / c))] = (x, y, now)

    def prune(self, now: float) -> None:
        old = now - self.keep_s
        self._cells = {k: v for k, v in self._cells.items() if v[2] >= old}

    def points(self):
        return [(x, y) for x, y, _ in self._cells.values()]

    def __len__(self) -> int:
        return len(self._cells)


def voxel_xy(points_xy, cell: float = 0.05):
    """One point per ``cell`` square: keeps a few seconds of scans small."""
    seen = {}
    for x, y in points_xy:
        seen.setdefault((round(x / cell), round(y / cell)), (x, y))
    return list(seen.values())


def obstacle_xy(points, pose: Pose2D, floor_z: float, p: ObstacleParams = ObstacleParams(),
                max_range: float = 8.0, body_pad: float = 0.10):
    """(x, y) in the odometry frame of every point in the obstacle height band
    within ``max_range`` of the robot, minus the robot itself -- what is
    remembered for the turn check and planned around by exploration.

    The LiDAR sees the robot's own legs. Kept, they become obstacles left
    behind wherever it walked: in the sim (2026-10-31) its own old leg points
    sat in its turn sweep and it refused to turn anywhere. So the footprint,
    padded by ``body_pad`` for swinging legs, is dropped against the pose of
    *this* scan, before anything is remembered."""
    r2 = max_range * max_range
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    front, rear = p.body_front + body_pad, p.body_rear + body_pad
    half = p.body_half_width + body_pad
    out = []
    for x, y, z in points:
        h = z - floor_z
        if not p.z_min <= h <= p.z_max:
            continue
        dx, dy = x - pose.x, y - pose.y
        if dx * dx + dy * dy > r2:
            continue
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        if -rear <= xr <= front and abs(yr) <= half:
            continue                            # the robot itself
        out.append((x, y))
    return out


def floor_xy(points, pose: Pose2D, floor_z: float, band: float = 0.06,
             max_range: float = 8.0):
    """(x, y) of floor returns -- within ``band`` of the floor height, inside
    ``max_range`` -- for exploration's coverage grid: floor the LiDAR has seen
    is floor known to be there (coverage.py)."""
    r2 = max_range * max_range
    return [(x, y) for x, y, z in points
            if abs(z - floor_z) <= band and (x - pose.x) ** 2 + (y - pose.y) ** 2 <= r2]


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
