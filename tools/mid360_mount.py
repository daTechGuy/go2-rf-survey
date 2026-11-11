"""Where the Mid-360 sits on the robot, from the floor it sees. Pure, stdlib.

config/mount.yaml holds placeholder numbers (22 deg nose-down is the design,
not a measurement). Standing still on a flat floor, the floor is the biggest
plane in the Mid-360's view; its normal gives the sensor's pitch and roll,
its distance gives the sensor's height -- to well under a degree and a
centimetre, with no tape measure. Yaw (which way it faces) and the forward /
sideways offset can't come from the floor: yaw is checked against the
built-in LiDAR's walls (wall_yaw_offset), offsets stay as in mount.yaml.

Conventions, REP-103: x forward, y left, z up; rpy applied as
R = Rz(yaw) Ry(pitch) Rx(roll); a POSITIVE pitch tilts +x towards -z
(nose-down), matching go2_survey_description's URDF.
"""

from __future__ import annotations

import math
import random


# ----------------------------------------------------------------- vectors
def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _unit(v):
    n = math.sqrt(_dot(v, v))
    return None if n < 1e-9 else (v[0] / n, v[1] / n, v[2] / n)


# ------------------------------------------------------------------ rotation
def rpy_matrix(roll: float, pitch: float, yaw: float):
    """3x3 (rows) for R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def apply(R, t, p):
    return (R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + t[0],
            R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + t[1],
            R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + t[2])


def matrix_rpy(R):
    """(roll, pitch, yaw) of R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = -math.asin(max(-1.0, min(1.0, R[2][0])))
    return math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0])


def mount_in_body(roll, pitch, yaw, t, body_roll, body_pitch):
    """The mount measured against level (floor/ceiling) -> the mount on the
    body, given the body's own tilt while measuring (from odometry).
    mid360_to_odom applies the body's full orientation to every scan, so a
    level-frame mount would count the body's tilt twice -- on the robot
    (2026-11-11, body rolled -2.2 deg) that lifted the floor 9 cm.
    Returns ((roll, pitch, yaw), (x, y, z))."""
    B = rpy_matrix(body_roll, body_pitch, 0.0)
    Bt = tuple(zip(*B))
    L = rpy_matrix(roll, pitch, yaw)
    R = tuple(tuple(sum(Bt[i][k] * L[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    return matrix_rpy(R), apply(Bt, (0.0, 0.0, 0.0), t)


def icp_2d(ref_xy, pts_xy, iterations: int = 15, max_d: float = 0.15):
    """Rigid 2D fit of ``pts_xy`` onto ``ref_xy`` (both seen from the robot,
    level frame): only pairs within ``max_d`` count, so what one LiDAR sees
    and the other doesn't is ignored. Returns (theta, sx, sy, pairs,
    median gap) with p' = R(theta) p + s, or None with under 50 pairs."""
    grid = {}
    for x, y in ref_xy:
        grid.setdefault((math.floor(x / max_d), math.floor(y / max_d)), []).append((x, y))

    def pairs(th, sx, sy):
        c, s = math.cos(th), math.sin(th)
        out = []
        for x, y in pts_xy:
            px, py = c * x - s * y + sx, s * x + c * y + sy
            i, j = math.floor(px / max_d), math.floor(py / max_d)
            best, bq = max_d * max_d, None
            for a in (-1, 0, 1):
                for b in (-1, 0, 1):
                    for q in grid.get((i + a, j + b), ()):
                        d2 = (q[0] - px) ** 2 + (q[1] - py) ** 2
                        if d2 < best:
                            best, bq = d2, q
            if bq is not None:
                out.append(((px, py), bq, math.sqrt(best)))
        return out

    th = sx = sy = 0.0
    for _ in range(iterations):
        pr = pairs(th, sx, sy)
        if len(pr) < 50:
            return None
        n = len(pr)
        max_ = sum(p[0][0] for p in pr) / n; may = sum(p[0][1] for p in pr) / n
        mbx = sum(p[1][0] for p in pr) / n; mby = sum(p[1][1] for p in pr) / n
        num = sum((a[0] - max_) * (b[1] - mby) - (a[1] - may) * (b[0] - mbx) for a, b, _ in pr)
        den = sum((a[0] - max_) * (b[0] - mbx) + (a[1] - may) * (b[1] - mby) for a, b, _ in pr)
        d = math.atan2(num, den)
        c, s = math.cos(d), math.sin(d)
        # p'' = R(d) p' + (mb - R(d) ma), with p' = R(th) p + s
        tx, ty = mbx - (c * max_ - s * may), mby - (s * max_ + c * may)
        th, sx, sy = th + d, c * sx - s * sy + tx, s * sx + c * sy + ty
    pr = pairs(th, sx, sy)
    if len(pr) < 50:
        return None
    gaps = sorted(p[2] for p in pr)
    return th, sx, sy, len(pr), gaps[len(gaps) // 2]


# ------------------------------------------------------------------- floor
def fit_floor(points, iterations: int = 300, tol: float = 0.02, max_tilt_deg: float = 40.0,
              min_height: float = 0.15, seed: int = 1, ceiling: bool = False):
    """RANSAC: the plane with the most points within ``tol`` that faces the
    sensor from below (normal within ``max_tilt_deg`` of the sensor's own +z,
    at least ``min_height`` under it). Walls and desks fail that test.
    ``ceiling``: the same, but the plane above the sensor -- on the robot
    (2026-11-11) the Mid-360 sees the floor only from ~1.6 m ahead and the
    ceiling everywhere, so the ceiling is the better level reference.
    Returns (normal pointing up, distance from the sensor to it, inliers) or
    None. The normal is refined from the inliers' spread."""
    pts = list(points)
    if len(pts) < 50:
        return None
    rng = random.Random(seed)
    cos_max = math.cos(math.radians(max_tilt_deg))
    best = None
    for _ in range(iterations):
        a, b, c = rng.sample(pts, 3)
        n = _unit(_cross(_sub(b, a), _sub(c, a)))
        if n is None:
            continue
        if n[2] < 0:
            n = (-n[0], -n[1], -n[2])              # point "up", towards the sensor side
        if n[2] < cos_max:
            continue
        d = -_dot(n, a)                            # plane: n.p + d = 0; sensor at origin
        if (-d if ceiling else d) < min_height:    # sensor must be below / above it
            continue
        inl = sum(1 for p in pts if abs(_dot(n, p) + d) <= tol)
        if best is None or inl > best[2]:
            best = (n, d, inl)
    if best is None:
        return None
    # refine: plane through the inliers' centroid, normal from their spread
    n0, d0, _ = best
    inl = [p for p in pts if abs(_dot(n0, p) + d0) <= tol]
    cx = sum(p[0] for p in inl) / len(inl)
    cy = sum(p[1] for p in inl) / len(inl)
    cz = sum(p[2] for p in inl) / len(inl)
    # covariance, then the smallest-eigenvalue direction by power iteration on (tr*I - C)
    sxx = sum((p[0] - cx) ** 2 for p in inl); syy = sum((p[1] - cy) ** 2 for p in inl)
    szz = sum((p[2] - cz) ** 2 for p in inl); sxy = sum((p[0] - cx) * (p[1] - cy) for p in inl)
    sxz = sum((p[0] - cx) * (p[2] - cz) for p in inl); syz = sum((p[1] - cy) * (p[2] - cz) for p in inl)
    tr = sxx + syy + szz
    M = ((tr - sxx, -sxy, -sxz), (-sxy, tr - syy, -syz), (-sxz, -syz, tr - szz))
    v = n0
    for _ in range(50):
        v = _unit((_dot(M[0], v), _dot(M[1], v), _dot(M[2], v))) or n0
    if v[2] < 0:
        v = (-v[0], -v[1], -v[2])
    d = -_dot(v, (cx, cy, cz))
    return v, (-d if ceiling else d), len(inl)


def level_peak(zs, bin_m: float = 0.02, min_points: int = 30):
    """The height where most of ``zs`` pile up -- a floor or ceiling seen by
    the built-in LiDAR (hanging lights are a few points, the ceiling is
    thousands). None if no bin holds ``min_points``."""
    counts = {}
    for z in zs:
        k = math.floor(z / bin_m)
        counts[k] = counts.get(k, 0) + 1
    if not counts:
        return None
    k, n = max(counts.items(), key=lambda kv: kv[1])
    if n < min_points:
        return None
    # centre of mass of the peak bin and its neighbours
    num = sum((j + 0.5) * bin_m * counts.get(j, 0) for j in (k - 1, k, k + 1))
    return num / sum(counts.get(j, 0) for j in (k - 1, k, k + 1))


def floor_below(points, up, min_drop: float = 0.15, band: float = 0.08):
    """Height of the sensor over the floor, measuring along ``up`` (from the
    ceiling fit): the lowest dense level of points below the sensor. The
    floor is the lowest thing there is, so look only within ``band`` of the
    deepest points (2nd percentile) -- chair seats and the like sit higher.
    None if too little is below the sensor."""
    depth = sorted(-_dot(up, p) for p in points if -_dot(up, p) >= min_drop)
    if len(depth) < 50:
        return None
    deep = depth[int(len(depth) * 0.98)]
    return level_peak([d for d in depth if d >= deep - band])


def mount_from_floor(normal, height):
    """(roll, pitch, height) of the sensor over a level floor, from the floor
    (or ceiling) normal seen in the sensor frame, pointing up. World up in
    sensor coordinates is R^T (0,0,1) = (-sin p, cos p sin r, cos p cos r)."""
    ux, uy, uz = normal
    pitch = -math.asin(max(-1.0, min(1.0, ux)))
    roll = math.atan2(uy, uz)
    return roll, pitch, height


# -------------------------------------------------------------- yaw check
def wall_angle_histogram(points_xy, bins: int = 90):
    """Directions of local wall segments, folded into [0, 90) deg -- a room's
    walls are mostly at right angles. From pairs of nearby points."""
    h = [0.0] * bins
    cells = {}
    for x, y in points_xy:                         # pairs only within a 0.25 m cell
        cells.setdefault((math.floor(x / 0.25), math.floor(y / 0.25)), []).append((x, y))
    for pts in cells.values():
        pts = pts[:12]
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                (x0, y0), (x1, y1) = pts[i], pts[j]
                if math.hypot(x1 - x0, y1 - y0) > 0.05:
                    a = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 90.0
                    h[int(a / 90.0 * bins) % bins] += 1.0
    return h


def wall_yaw_offset(ref_xy, other_xy, bins: int = 90):
    """Degrees to rotate ``other`` (about z) to line its walls up with
    ``ref``'s, in (-45, 45]: the circular cross-correlation peak of their
    wall-direction histograms. None if either has too few wall points."""
    a, b = wall_angle_histogram(ref_xy, bins), wall_angle_histogram(other_xy, bins)
    if sum(a) < 20 or sum(b) < 20:
        return None
    best, shift = -1.0, 0
    for s in range(bins):
        c = sum(a[i] * b[(i - s) % bins] for i in range(bins))
        if c > best:
            best, shift = c, s
    deg = shift * 90.0 / bins
    return deg - 90.0 if deg > 45.0 else deg
