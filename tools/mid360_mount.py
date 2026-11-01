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


# ------------------------------------------------------------------- floor
def fit_floor(points, iterations: int = 300, tol: float = 0.02, max_tilt_deg: float = 60.0,
              min_height: float = 0.15, seed: int = 1):
    """RANSAC: the plane with the most points within ``tol`` that faces the
    sensor from below (normal within ``max_tilt_deg`` of the sensor's own +z,
    at least ``min_height`` under it). Walls and desks fail that test.
    Returns (normal pointing up, height of the sensor above it, inliers) or
    None. The normal is refined by a least-squares-free average over inlier
    triples -- plenty at these point counts."""
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
        if d < min_height:                         # sensor must be above it
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
    return v, d, len(inl)


def mount_from_floor(normal, height):
    """(roll, pitch, height) of the sensor over a level floor, from the floor
    normal seen in the sensor frame. World up in sensor coordinates is
    R^T (0,0,1) = (-sin p, cos p sin r, cos p cos r)."""
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
