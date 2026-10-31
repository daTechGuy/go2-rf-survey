"""What has been seen, and how to get somewhere through it. Pure, no ROS.

For exploring (explore.py, ``strategy="frontier"``). In a 25 x 20 m hall
(sim, 2026-10-30) greedy legs stayed within ~8 m of the start: they avoided
where the robot had *walked*, not what the LiDAR had already *seen*, and the
walk home retraced every zig-zag. Here instead:

* **Grid** -- 0.25 m cells around the start. Floor returns mark a cell FREE,
  obstacle returns (the height band above the floor) mark it OCC; everything
  else stays UNKNOWN. A cell needs two obstacle hits to count, so one stray
  return doesn't wall off an aisle; an obstacle beats floor in the same cell.
* **Clearance** -- distance from each cell to the nearest OCC cell (octile,
  multi-source Dijkstra): where the robot's body fits.
* **Frontiers** -- FREE cells next to UNKNOWN ones: walking there shows new
  floor. Grouped into connected patches.
* **Paths** -- Dijkstra from the robot over FREE cells with at least
  ``min_clearance``, a small extra cost near obstacles (it keeps to the middle
  of an aisle), then simplified to the few waypoints whose straight lines keep
  that clearance.

Cells the robot can't see (under a desk, behind a cupboard) stay UNKNOWN and
are never planned through.
"""

from __future__ import annotations

import heapq
import math

UNKNOWN, FREE, OCC = 0, 1, 2
_N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
_SQ2 = math.sqrt(2.0)


class Grid:
    def __init__(self, cx: float, cy: float, half_size: float, cell: float = 0.25) -> None:
        self.cell = cell
        self.n = int(2 * half_size / cell) + 1
        self.x0, self.y0 = cx - half_size, cy - half_size
        self._free = bytearray(self.n * self.n)       # hit counts, capped at 255
        self._occ = bytearray(self.n * self.n)

    # ------------------------------------------------------------- geometry
    def cell_of(self, x: float, y: float):
        i, j = int((x - self.x0) / self.cell), int((y - self.y0) / self.cell)
        return (i, j) if 0 <= i < self.n and 0 <= j < self.n else None

    def centre(self, c):
        return self.x0 + (c[0] + 0.5) * self.cell, self.y0 + (c[1] + 0.5) * self.cell

    def state(self, c) -> int:
        k = c[0] * self.n + c[1]
        if self._occ[k] >= 2:
            return OCC
        return FREE if self._free[k] else UNKNOWN

    # ------------------------------------------------------------- updates
    def mark(self, floor_xy=(), obstacle_xy=()) -> None:
        n, x0, y0, s = self.n, self.x0, self.y0, self.cell
        for xs, counts in ((floor_xy, self._free), (obstacle_xy, self._occ)):
            for x, y in xs:
                i, j = int((x - x0) / s), int((y - y0) / s)
                if 0 <= i < n and 0 <= j < n:
                    k = i * n + j
                    if counts[k] < 255:
                        counts[k] += 1

    def mark_rays(self, origin, hits, sensor_h: float = 0.35, band=(0.10, 1.00)) -> None:
        """Every return proves its ray was clear: mark FREE the cells the ray
        crossed while it was inside the obstacle ``band`` (heights above the
        floor). ``origin`` is the robot (x, y), ``sensor_h`` the LiDAR's
        height, ``hits`` (x, y, height above floor). A ray to a wall 2 m up
        and 20 m away is still under 1 m for its first metres: the floor
        there is clear of anything desk-high. The hit cell itself excluded.

        Sim hall (2026-10-31): floor returns only reach 3-5 m, so with floor
        points alone the aisle beyond stayed 'unseen', though the desks and
        walls 6-14 m away were in plain view."""
        o = self.cell_of(*origin)
        if o is None:
            return
        n, free = self.n, self._free
        oi, oj = o
        lo, hi_band = band
        seen = {}
        for x, y, h in hits:                    # one ray per hit cell (lowest hit)
            c = self.cell_of(x, y)
            if c is not None and c != o and (c not in seen or h < seen[c]):
                seen[c] = h
        for (ci, cj), h in seen.items():
            # fraction of the ray, [t0, t1], spent inside the band
            dh = h - sensor_h
            if abs(dh) < 1e-6:
                t0, t1 = (0.0, 1.0) if lo <= sensor_h <= hi_band else (1.0, 0.0)
            else:
                ta, tb = (lo - sensor_h) / dh, (hi_band - sensor_h) / dh
                t0, t1 = max(0.0, min(ta, tb)), min(1.0, max(ta, tb))
            if t0 >= t1:
                continue
            steps = max(abs(ci - oi), abs(cj - oj))
            for k in range(max(1, math.ceil(t0 * steps)), min(steps, int(t1 * steps) + 1)):
                i = oi + round((ci - oi) * k / steps)
                j = oj + round((cj - oj) * k / steps)
                key = i * n + j
                if free[key] < 255:
                    free[key] += 1

    def counts(self):
        """(free, occ, unknown) cell counts."""
        f = o = 0
        for i in range(self.n):
            for j in range(self.n):
                st = self.state((i, j))
                f += st == FREE
                o += st == OCC
        return f, o, self.n * self.n - f - o

    # ------------------------------------------------------------- analysis
    def clearance(self, cap: float = 3.0):
        """{cell: metres to the nearest OCC cell}, capped (cells further than
        ``cap`` from anything are simply absent: read them as ``cap``)."""
        n, s = self.n, self.cell
        dist = {}
        heap = []
        for i in range(n):
            for j in range(n):
                if self.state((i, j)) == OCC:
                    dist[(i, j)] = 0.0
                    heap.append((0.0, i, j))
        heapq.heapify(heap)
        while heap:
            d, i, j = heapq.heappop(heap)
            if d > dist.get((i, j), math.inf) or d > cap:
                continue
            for di, dj in _N8:
                a, b = i + di, j + dj
                if 0 <= a < n and 0 <= b < n:
                    nd = d + s * (_SQ2 if di and dj else 1.0)
                    if nd < dist.get((a, b), math.inf) and nd <= cap:
                        dist[(a, b)] = nd
                        heapq.heappush(heap, (nd, a, b))
        return dist

    def frontiers(self, min_cells: int = 4):
        """Patches of FREE cells bordering UNKNOWN ones: [[cell, ...], ...]."""
        n = self.n
        edge = set()
        for i in range(n):
            for j in range(n):
                if self.state((i, j)) != FREE:
                    continue
                for di, dj in _N8:
                    a, b = i + di, j + dj
                    if 0 <= a < n and 0 <= b < n and self.state((a, b)) == UNKNOWN:
                        edge.add((i, j))
                        break
        patches, seen = [], set()
        for c in edge:
            if c in seen:
                continue
            patch, todo = [], [c]
            seen.add(c)
            while todo:
                i, j = todo.pop()
                patch.append((i, j))
                for di, dj in _N8:
                    nb = (i + di, j + dj)
                    if nb in edge and nb not in seen:
                        seen.add(nb)
                        todo.append(nb)
            if len(patch) >= min_cells:
                patches.append(patch)
        return patches


def costs_from(grid: Grid, start, clear: dict, min_clearance: float, cap: float = 3.0,
               near_cost: float = 0.5):
    """Dijkstra from ``start`` over FREE cells with clearance >= min_clearance
    (the start cell itself is always allowed: the robot is already there).
    Returns ({cell: cost}, {cell: parent})."""
    n, s = grid.n, grid.cell
    cost, parent = {start: 0.0}, {start: None}
    heap = [(0.0, start)]
    while heap:
        d, c = heapq.heappop(heap)
        if d > cost.get(c, math.inf):
            continue
        for di, dj in _N8:
            nb = (c[0] + di, c[1] + dj)
            if not (0 <= nb[0] < n and 0 <= nb[1] < n) or grid.state(nb) != FREE:
                continue
            cl = clear.get(nb, cap)
            if cl < min_clearance:
                continue
            step = s * (_SQ2 if di and dj else 1.0)
            nd = d + step * (1.0 + near_cost * max(0.0, 1.5 - cl))   # prefer mid-aisle
            if nd < cost.get(nb, math.inf):
                cost[nb], parent[nb] = nd, c
                heapq.heappush(heap, (nd, nb))
    return cost, parent


def path_to(parent: dict, goal):
    out, c = [], goal
    while c is not None:
        out.append(c)
        c = parent.get(c)
    return out[::-1]


def line_clear(grid: Grid, clear: dict, a, b, min_clearance: float, cap: float = 3.0) -> bool:
    """Every point of the straight segment a -> b (metres) sits in a FREE cell
    with at least ``min_clearance`` (the first cell, where the robot stands,
    excepted)."""
    (ax, ay), (bx, by) = a, b
    steps = max(1, int(math.hypot(bx - ax, by - ay) / (grid.cell * 0.5)))
    first = grid.cell_of(ax, ay)
    for k in range(steps + 1):
        x, y = ax + (bx - ax) * k / steps, ay + (by - ay) * k / steps
        c = grid.cell_of(x, y)
        if c is None:
            return False
        if c == first:
            continue
        if grid.state(c) != FREE or clear.get(c, cap) < min_clearance:
            return False
    return True


def simplify(grid: Grid, clear: dict, cells, min_clearance: float):
    """Path cells -> the fewest waypoints (metres) whose straight lines keep
    the clearance. The robot's own cell is not a waypoint."""
    pts = [grid.centre(c) for c in cells]
    if len(pts) < 2:
        return pts[1:]
    out, i = [], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not line_clear(grid, clear, pts[i], pts[j], min_clearance):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def sees(grid: Grid, a, b) -> bool:
    """No OCC cell on the straight line between cells ``a`` and ``b``: from
    ``a`` the LiDAR could see ``b`` (unknown cells don't block -- they may be
    floor nobody has looked at yet)."""
    (ai, aj), (bi, bj) = a, b
    steps = max(abs(bi - ai), abs(bj - aj), 1) * 2
    for k in range(1, steps):
        c = (round(ai + (bi - ai) * k / steps), round(aj + (bj - aj) * k / steps))
        if c != b and grid.state(c) == OCC:
            return False
    return True


def truncate(waypoints, start, max_len: float):
    """Cut a waypoint list after ``max_len`` metres of path from ``start``."""
    out, px, py, left = [], start[0], start[1], max_len
    for x, y in waypoints:
        d = math.hypot(x - px, y - py)
        if d >= left:
            f = left / d if d else 0.0
            out.append((px + (x - px) * f, py + (y - py) * f))
            return out
        out.append((x, y))
        left -= d
        px, py = x, y
    return out
