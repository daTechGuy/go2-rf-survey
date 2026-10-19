#!/usr/bin/env python3
"""Generate a synthetic floorplan and WiFi survey.

The point of this script is to make the post-processing chain verifiable
*before* the robot exists. It produces exactly the artefacts a real run
produces -- a map_server map and a survey CSV in the schema survey_node.py
writes -- so heatmap.py can be developed and checked against known ground
truth: we know where the APs actually are, so we can see whether the heatmap
puts the hot spots in the right places.

    python tools/make_synthetic_survey.py --out docs/sample

Writes <out>.pgm, <out>.yaml, <out>.csv and <out>-truth.csv.

The propagation model is log-distance with a wall-attenuation factor. It is
good enough to sanity-check rendering and masking; it is not a substitute for
measuring the real building, which is the entire reason the robot exists.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

import occupancy
from occupancy import FREE, OCCUPIED, OccupancyGrid

# --- propagation model ------------------------------------------------------
# RSSI(d) = TX_EIRP - PL0 - 10*n*log10(d) - walls*WAF
TX_EIRP_DBM = 17.0      # typical indoor AP, EIRP
PL0_DB = 47.0           # path loss at 1 m, ~2.4 GHz
PATH_LOSS_N = 3.2       # indoor exponent (2 = free space, 3-4 = cluttered)
WALL_ATTEN_DB = 6.5     # per interior wall crossed
NOISE_FLOOR_DBM = -95.0 # below this an AP is not heard at all
RSSI_JITTER_DB = 2.0    # measurement noise, 1 sigma


def build_floorplan(res: float = 0.05) -> OccupancyGrid:
    """A 30 x 18 m floor: central corridor, rooms either side, one glass wall.

    Deliberately includes a long thin wall between two sampled rooms, which is
    where euclidean interpolation misbehaves -- see the note in heatmap.py.
    """
    w_m, h_m = 30.0, 18.0
    cols, rows = int(w_m / res), int(h_m / res)
    cells = np.full((rows, cols), OCCUPIED, dtype=np.uint8)

    def carve(x0: float, y0: float, x1: float, y1: float) -> None:
        c0, c1 = int(x0 / res), int(x1 / res)
        r0, r1 = int(y0 / res), int(y1 / res)
        cells[r0:r1, c0:c1] = FREE

    # central corridor, full length
    carve(1.0, 8.0, 29.0, 10.5)

    # rooms along the north side
    for i, x0 in enumerate((1.5, 8.0, 14.5, 21.0)):
        carve(x0, 11.0, x0 + 5.5, 16.5)
        carve(x0 + 2.3, 10.5, x0 + 3.2, 11.0)        # doorway

    # rooms along the south side
    for i, x0 in enumerate((1.5, 9.5, 17.5)):
        carve(x0, 2.0, x0 + 7.0, 7.5)
        carve(x0 + 3.0, 7.5, x0 + 3.9, 8.0)          # doorway

    # a large south-east space, opened to the corridor by a wide gap
    carve(25.0, 3.0, 29.0, 7.5)
    carve(26.0, 7.5, 28.0, 8.0)

    return OccupancyGrid(cells, res, 0.0, 0.0)


def count_walls(grid: OccupancyGrid, ax: float, ay: float, bx: float, by: float) -> int:
    """Number of wall *crossings* on the segment a->b.

    Counts transitions into occupied space rather than occupied cells, so a
    30 cm wall counts once, not six times.
    """
    dist = math.hypot(bx - ax, by - ay)
    steps = max(2, int(dist / (grid.resolution * 0.5)))
    crossings = 0
    was_occupied = False
    for i in range(steps + 1):
        t = i / steps
        x, y = ax + (bx - ax) * t, ay + (by - ay) * t
        row, col = grid.world_to_cell(x, y)
        if 0 <= row < grid.height and 0 <= col < grid.width:
            now_occupied = grid.cells[row, col] == OCCUPIED
        else:
            now_occupied = True
        if now_occupied and not was_occupied:
            crossings += 1
        was_occupied = now_occupied
    return crossings


def rssi_at(
    grid: OccupancyGrid,
    ap: dict,
    x: float,
    y: float,
    rng: np.random.Generator,
) -> float | None:
    """Modelled RSSI at (x, y), or None if below the noise floor."""
    d = max(0.5, math.hypot(x - ap["x"], y - ap["y"]))
    walls = count_walls(grid, ap["x"], ap["y"], x, y)
    rssi = (
        TX_EIRP_DBM
        - PL0_DB
        - 10.0 * PATH_LOSS_N * math.log10(d)
        - walls * WALL_ATTEN_DB
        + rng.normal(0.0, RSSI_JITTER_DB)
    )
    # 5 GHz radios are attenuated more and reach less far.
    if ap["freq"] > 5000:
        rssi -= 4.0 + walls * 1.5
    return None if rssi < NOISE_FLOOR_DBM else rssi


def plan_route(grid: OccupancyGrid, spacing: float = 1.5) -> list[tuple[float, float]]:
    """A survey route: down the corridor, then into each room.

    Mirrors what a waypoint route actually looks like -- a corridor spine with
    branches -- so the resulting sample distribution has the same uneven
    density a real run produces, including rooms the robot never entered.
    """
    pts: list[tuple[float, float]] = []

    def run(x0, y0, x1, y1):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / spacing))
        for i in range(n + 1):
            t = i / n
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            if grid.is_free(x, y):
                pts.append((x, y))

    run(2.0, 9.2, 28.0, 9.2)                      # corridor spine

    for x0 in (1.5, 8.0, 14.5, 21.0):             # north rooms
        cx = x0 + 2.75
        run(cx, 10.0, cx, 15.5)
        run(x0 + 0.8, 13.0, x0 + 4.7, 13.0)

    for x0 in (1.5, 9.5):                         # south rooms -- note: the
        cx = x0 + 3.45                            # third is left unvisited on
        run(cx, 8.5, cx, 3.0)                     # purpose, to exercise the
        run(x0 + 0.8, 4.5, x0 + 6.2, 4.5)         # unsampled-area mask

    run(27.0, 8.0, 27.0, 4.0)                     # south-east space
    return pts


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="docs/sample", help="output path prefix")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--spacing", type=float, default=1.5, help="metres between samples")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    grid = build_floorplan()

    # Ground truth: four APs, two dual-band. Deliberately leaves the far south
    # of the floor poorly covered so the heatmap has a real dead zone to show.
    aps = [
        {"bssid": "3c:37:86:00:00:01", "ssid": "CorpNet", "freq": 2437, "x": 5.0, "y": 13.0},
        {"bssid": "3c:37:86:00:00:02", "ssid": "CorpNet-5G", "freq": 5180, "x": 5.0, "y": 13.0},
        {"bssid": "3c:37:86:00:00:11", "ssid": "CorpNet", "freq": 2462, "x": 17.0, "y": 9.5},
        {"bssid": "3c:37:86:00:00:12", "ssid": "CorpNet-5G", "freq": 5745, "x": 17.0, "y": 9.5},
        {"bssid": "3c:37:86:00:00:21", "ssid": "CorpNet", "freq": 2412, "x": 26.5, "y": 12.0},
        {"bssid": "00:11:22:33:44:55", "ssid": "Guest", "freq": 2437, "x": 3.0, "y": 4.0},
    ]

    occupancy.save(grid, out.with_suffix(".yaml"))

    with out.with_suffix("").with_name(out.name + "-truth.csv").open(
        "w", newline=""
    ) as fh:
        w = csv.DictWriter(fh, fieldnames=["bssid", "ssid", "freq", "x", "y"])
        w.writeheader()
        w.writerows(aps)

    route = plan_route(grid, args.spacing)
    rows = 0
    with out.with_suffix(".csv").open("w", newline="") as fh:
        w = csv.DictWriter(
            fh,
            fieldnames=[
                "stamp", "sample_id", "x", "y", "z", "yaw",
                "bssid", "ssid", "freq", "band", "rssi",
            ],
        )
        w.writeheader()
        t = 1760932400.0
        for sid, (x, y) in enumerate(route):
            # Stop-and-scan: ~4 s per waypoint plus travel.
            t += 4.0 + args.spacing / 0.35
            yaw = rng.uniform(-math.pi, math.pi)
            for a in aps:
                r = rssi_at(grid, a, x, y, rng)
                if r is None:
                    continue       # genuinely not heard -- absence is data
                band = "2.4" if a["freq"] < 2500 else "5"
                w.writerow({
                    "stamp": f"{t:.3f}", "sample_id": sid,
                    "x": f"{x:.4f}", "y": f"{y:.4f}", "z": "0.4000",
                    "yaw": f"{yaw:.5f}",
                    "bssid": a["bssid"], "ssid": a["ssid"],
                    "freq": a["freq"], "band": band, "rssi": f"{r:.1f}",
                })
                rows += 1

    print(f"floorplan  {out.with_suffix('.pgm')}  "
          f"({grid.width}x{grid.height} cells @ {grid.resolution} m)")
    print(f"map meta   {out.with_suffix('.yaml')}")
    print(f"survey     {out.with_suffix('.csv')}  "
          f"({len(route)} samples, {rows} rows)")
    print(f"truth      {out.name}-truth.csv  ({len(aps)} APs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
