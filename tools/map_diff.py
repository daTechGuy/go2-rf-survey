#!/usr/bin/env python3
"""What moved between two visits to the same room.

    python tools/map_diff.py BEFORE_map.yaml AFTER_map.yaml --out what-moved.png

Each run's floorplan (bag_to_map: <run>_map.pgm + .yaml) is in that run's own
odom frame -- a different start spot and heading every visit. So:

1. **align** AFTER onto BEFORE: rotation from the walls (they run the same
   way; wall_yaw_offset, tried at +0/90/180/270 deg), shift by a coarse search
   for the most overlap, then point-to-point refinement (2D ICP);
2. **diff**: "new" = occupied now where BEFORE saw open floor; "gone" =
   occupied before where AFTER sees open floor. A spot either map never saw is
   not a change -- an unvisited corner must not read as "furniture removed";
3. small specks are dropped; what's left is listed (where, how big) and drawn.

The alignment and diff are pure Python (tested in tools/tests/test_map_diff.py);
loading and drawing use numpy / matplotlib (tools/requirements.txt).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mid360_mount import wall_yaw_offset  # noqa: E402

CELL = 0.10        # m: the diff's grid
_N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def cell(x, y, c=CELL):
    return (math.floor(x / c), math.floor(y / c))


def transform(pts, theta, tx, ty):
    ct, st = math.cos(theta), math.sin(theta)
    return [(ct * x - st * y + tx, st * x + ct * y + ty) for x, y in pts]


def dilate(cells, r=1):
    out = set(cells)
    for i, j in cells:
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                out.add((i + di, j + dj))
    return out


def _overlap(ref_dil, pts):
    return sum(1 for x, y in pts if cell(x, y) in ref_dil) / max(1, len(pts))


def _icp(ref_pts, mov, theta, tx, ty, iterations=20, max_d=0.5):
    """Refine (theta, tx, ty) by point-to-point ICP; ref looked up by grid."""
    grid = {}
    for p in ref_pts:
        grid.setdefault(cell(*p, 0.5), []).append(p)
    for _ in range(iterations):
        pairs = []
        for (mx, my), (x, y) in zip(mov, transform(mov, theta, tx, ty)):
            ci, cj = cell(x, y, 0.5)
            best, bd = None, max_d
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    for q in grid.get((ci + di, cj + dj), ()):
                        d = math.hypot(q[0] - x, q[1] - y)
                        if d < bd:
                            best, bd = q, d
            if best is not None:
                pairs.append(((mx, my), best))
        if len(pairs) < 10:
            break
        # closed-form 2D rigid fit of mov -> ref over the pairs
        n = len(pairs)
        mxc = sum(a[0] for a, _ in pairs) / n; myc = sum(a[1] for a, _ in pairs) / n
        rxc = sum(b[0] for _, b in pairs) / n; ryc = sum(b[1] for _, b in pairs) / n
        sxx = sum((a[0] - mxc) * (b[0] - rxc) + (a[1] - myc) * (b[1] - ryc) for a, b in pairs)
        sxy = sum((a[0] - mxc) * (b[1] - ryc) - (a[1] - myc) * (b[0] - rxc) for a, b in pairs)
        theta = math.atan2(sxy, sxx)
        ct, st = math.cos(theta), math.sin(theta)
        tx, ty = rxc - (ct * mxc - st * myc), ryc - (st * mxc + ct * myc)
        max_d = max(0.15, max_d * 0.7)
    return theta, tx, ty


def align(ref_pts, mov_pts, search=4.0, step=0.2, sample=400):
    """(theta, tx, ty, overlap) that puts mov_pts onto ref_pts."""
    ref_dil = dilate({cell(x, y) for x, y in ref_pts})
    smp = mov_pts[:: max(1, len(mov_pts) // sample)]
    rcx = sum(x for x, _ in ref_pts) / len(ref_pts); rcy = sum(y for _, y in ref_pts) / len(ref_pts)
    off = wall_yaw_offset(ref_pts, mov_pts) or 0.0
    best = None
    for k in range(4):
        th = math.radians(off + 90.0 * k)
        rot = transform(smp, th, 0.0, 0.0)
        mcx = sum(x for x, _ in rot) / len(rot); mcy = sum(y for _, y in rot) / len(rot)
        n = int(search / step)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                tx, ty = rcx - mcx + i * step, rcy - mcy + j * step
                s = _overlap(ref_dil, [(x + tx, y + ty) for x, y in rot])
                if best is None or s > best[0]:
                    best = (s, th, tx, ty)
    _, th, tx, ty = best
    th, tx, ty = _icp(ref_pts, mov_pts[:: max(1, len(mov_pts) // 1500)], th, tx, ty)
    score = _overlap(dilate({cell(x, y) for x, y in ref_pts}, 1), transform(smp, th, tx, ty))
    return th, tx, ty, score


def blobs(cells, min_cells=6):
    """8-connected groups: [(centre x, centre y, area m^2, cells)], big first."""
    left, out = set(cells), []
    while left:
        todo = [left.pop()]
        grp = []
        while todo:
            c = todo.pop()
            grp.append(c)
            for di, dj in _N8:
                nb = (c[0] + di, c[1] + dj)
                if nb in left:
                    left.remove(nb)
                    todo.append(nb)
        if len(grp) >= min_cells:
            cx = (sum(i for i, _ in grp) / len(grp) + 0.5) * CELL
            cy = (sum(j for _, j in grp) / len(grp) + 0.5) * CELL
            out.append((cx, cy, len(grp) * CELL * CELL, grp))
    return sorted(out, key=lambda b: -b[2])


def diff(ref_occ, ref_free, mov_occ, mov_free, min_cells=6):
    """Changes as (new, gone) blob lists; all inputs are point lists already
    in the same frame. Only where the other visit actually saw open floor."""
    ro, mo = {cell(*p) for p in ref_occ}, {cell(*p) for p in mov_occ}
    rf, mf = {cell(*p) for p in ref_free}, {cell(*p) for p in mov_free}
    ro_d, mo_d = dilate(ro), dilate(mo)            # a wall shifted by a cell isn't a change
    new = {c for c in mo if c in rf and c not in ro_d}
    gone = {c for c in ro if c in mf and c not in mo_d}
    return blobs(new, min_cells), blobs(gone, min_cells)


# ------------------------------------------------------------- files, drawing
def _points(grid, value):
    import numpy as np
    rows, cols = np.nonzero(grid.cells == value)
    r = grid.resolution
    return list(zip((grid.origin_x + (cols + 0.5) * r).tolist(),
                    (grid.origin_y + (rows + 0.5) * r).tolist()))


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("before", type=Path, help="earlier visit's <run>_map.yaml")
    ap.add_argument("after", type=Path, help="later visit's <run>_map.yaml")
    ap.add_argument("--out", type=Path, default=Path("what-moved.png"))
    ap.add_argument("--min-area", type=float, default=0.06, help="m^2: smaller changes ignored")
    a = ap.parse_args(argv)

    import occupancy
    A, B = occupancy.load(a.before), occupancy.load(a.after)
    a_occ, a_free = _points(A, occupancy.OCCUPIED), _points(A, occupancy.FREE)
    b_occ, b_free = _points(B, occupancy.OCCUPIED), _points(B, occupancy.FREE)
    th, tx, ty, score = align(a_occ, b_occ)
    print(f"aligned: rotated {math.degrees(th):+.1f} deg, shifted ({tx:+.2f}, {ty:+.2f}) m; "
          f"{100 * score:.0f}% of the later map's walls land on the earlier's")
    if score < 0.5:
        print("WARN poor alignment -- the same room? enough walls seen in both?")
    b_occ_t, b_free_t = transform(b_occ, th, tx, ty), transform(b_free, th, tx, ty)
    new, gone = diff(a_occ, a_free, b_occ_t, b_free_t, max(1, int(a.min_area / CELL ** 2)))
    for name, bl in (("new", new), ("gone", gone)):
        print(f"{name}: {len(bl)}" + "".join(f"\n   {area:.2f} m^2 at ({x:.1f}, {y:.1f})"
                                             for x, y, area, _ in bl[:12]))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 7), dpi=110)
    for pts, colr, size in ((a_free, "#eeeeee", 2), (a_occ, "#555555", 2)):
        if pts:
            ax.scatter(*zip(*pts), s=size, c=colr, marker="s", linewidths=0)
    for bl, colr, label in ((gone, "#1f77b4", "gone since the first visit"),
                            (new, "#ff7f0e", "new in the second visit")):
        pts = [((i + 0.5) * CELL, (j + 0.5) * CELL) for _, _, _, g in bl for i, j in g]
        if pts:
            ax.scatter(*zip(*pts), s=8, c=colr, marker="s", linewidths=0, label=label)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"What moved: {a.before.stem.replace('_map', '')} -> "
                 f"{a.after.stem.replace('_map', '')}")
    if new or gone:
        ax.legend(loc="upper right")
    fig.savefig(a.out, bbox_inches="tight")
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
