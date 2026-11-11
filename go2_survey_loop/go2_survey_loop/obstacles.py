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
    fz = _floor_fn(floor_z)
    front = near = 0
    for x, y, z in points:
        h = z - fz(x, y)
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
    fz = _floor_fn(floor_z)
    out = []
    for x, y, z in points:
        h = z - fz(x, y)
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


def ray_points(points, pose: Pose2D, floor_z: float, p: ObstacleParams = ObstacleParams(),
               max_range: float = 12.0, body_pad: float = 0.10, cell: float = 0.25):
    """(x, y, height above floor) of returns to trace rays to (coverage.
    mark_rays): within ``max_range``, not the robot's own legs, thinned to
    the lowest return per ``cell`` square (one ray per cell is enough)."""
    r2 = max_range * max_range
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    front, rear = p.body_front + body_pad, p.body_rear + body_pad
    half = p.body_half_width + body_pad
    fz = _floor_fn(floor_z)
    low = {}
    for x, y, z in points:
        dx, dy = x - pose.x, y - pose.y
        if dx * dx + dy * dy > r2:
            continue
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        if -rear <= xr <= front and abs(yr) <= half:
            continue
        h = z - fz(x, y)
        k = (round(x / cell), round(y / cell))
        if k not in low or h < low[k][2]:
            low[k] = (x, y, h)
    return list(low.values())


def floor_xy(points, pose: Pose2D, floor_z: float, band: float = 0.06,
             max_range: float = 8.0):
    """(x, y) of floor returns -- within ``band`` of the floor height, inside
    ``max_range`` -- for exploration's coverage grid: floor the LiDAR has seen
    is floor known to be there (coverage.py)."""
    r2 = max_range * max_range
    fz = _floor_fn(floor_z)
    return [(x, y) for x, y, z in points
            if abs(z - fz(x, y)) <= band and (x - pose.x) ** 2 + (y - pose.y) ** 2 <= r2]


def floor_from_scan(points, pose: Pose2D, radius: float = 2.0,
                    percentile: float = 2.0, min_points: int = 50, layer: float = 0.06):
    """Floor height near the robot, or None if the scan has too few nearby
    points to say. The robot's LiDAR sees the floor all around it, so the low
    end of the nearby z distribution is the floor layer (a percentile rather
    than the minimum ignores stray below-floor returns); the floor is the
    middle of that layer -- the median of points within ``layer`` above it.

    Not the low tail itself: on carpet (2026-11-11) the returns spread ~7 cm,
    the 2nd percentile sat 3.6 cm under the median, so the 10 cm obstacle
    band began ~6 cm above the carpet. Ten seconds of turn memory collected
    a few such carpet points and a route refused to turn in open floor."""
    r2 = radius * radius
    zs = sorted(z for x, y, z in points
                if (x - pose.x) ** 2 + (y - pose.y) ** 2 <= r2)
    if len(zs) < min_points:
        return None
    k = min(len(zs) - 1, max(0, int(len(zs) * percentile / 100.0)))
    low = zs[k]
    band = [z for z in zs[k:] if z <= low + layer]
    return band[len(band) // 2]


@dataclass(frozen=True)
class FloorPlane:
    """The floor as a tilted plane, odometry frame: z = z0 + gx (x - x0) +
    gy (y - y0), anchored where the robot stood. Wherever a floor height is
    taken (classify, obstacle_xy, ray_points, floor_xy) a FloorPlane can
    stand in for a plain number."""
    z0: float
    gx: float
    gy: float
    x0: float
    y0: float

    def at(self, x: float, y: float) -> float:
        return self.z0 + self.gx * (x - self.x0) + self.gy * (y - self.y0)


def _floor_fn(floor):
    """floor height as a function of (x, y), from a number or a FloorPlane."""
    if isinstance(floor, FloorPlane):
        return floor.at
    return lambda _x, _y: floor


def _fit_plane(pts):
    """Least squares z = a + b dx + c dy over (dx, dy, z); None if the points
    don't spread in both directions."""
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts); sz = sum(p[2] for p in pts)
    sxx = sum(p[0] * p[0] for p in pts); syy = sum(p[1] * p[1] for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    sxz = sum(p[0] * p[2] for p in pts); syz = sum(p[1] * p[2] for p in pts)
    m = ((n, sx, sy), (sx, sxx, sxy), (sy, sxy, syy))
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    if abs(det) < 1e-9 * max(1.0, n) ** 3:
        return None
    r = (sz, sxz, syz)

    def col(k):
        return tuple(tuple(r[i] if j == k else m[i][j] for j in range(3)) for i in range(3))

    def d3(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))

    return d3(col(0)) / det, d3(col(1)) / det, d3(col(2)) / det


def floor_plane_from_scan(points, pose: Pose2D, radius: float = 3.0, near: float = 1.5,
                          band: float = 0.04, max_slope: float = 0.10,
                          min_points: int = 100, iterations: int = 3):
    """The floor round the robot as a FloorPlane, or None.

    On the robot (2026-11-11) the built-in cloud's floor tilted while it
    walked -- -0.07 m underfoot, +0.03 m two metres ahead, ~3 deg -- so one
    floor height put carpet ahead at the obstacle threshold. Fit: start from
    the floor layer close by (floor_from_scan within ``near``), then fit a
    plane to the returns within ``band`` of it out to ``radius``, a few
    rounds. Slopes over ``max_slope`` (5.7 deg) aren't a classroom floor:
    flat instead."""
    f0 = floor_from_scan(points, pose, radius=near)
    if f0 is None:
        return None
    r2 = radius * radius
    rel = [(x - pose.x, y - pose.y, z) for x, y, z in points
           if (x - pose.x) ** 2 + (y - pose.y) ** 2 <= r2]
    a, b, c = f0, 0.0, 0.0
    for _ in range(iterations):
        sel = [p for p in rel if abs(p[2] - (a + b * p[0] + c * p[1])) <= band]
        if len(sel) < min_points:
            break
        fit = _fit_plane(sel)
        if fit is None:
            break
        if abs(fit[1]) > max_slope or abs(fit[2]) > max_slope:
            return FloorPlane(f0, 0.0, 0.0, pose.x, pose.y)
        a, b, c = fit
    return FloorPlane(a, b, c, pose.x, pose.y)


def median(values):
    v = sorted(values)
    if not v:
        return None
    m = len(v) // 2
    return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])
