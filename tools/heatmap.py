#!/usr/bin/env python3
"""Render WiFi survey CSVs as coverage heatmaps over the SLAM floorplan.

    python tools/heatmap.py docs/sample.csv --map docs/sample.yaml --out docs/

Runs off-robot. numpy + scipy + matplotlib, no ROS.

Produces one panel per SSID plus a combined best-signal layer, which is the
one that answers "can a client associate anywhere on this floor".

Two masks are applied, and both matter:

* **occupancy** -- cells that are walls or unknown are not coverage, and
  colouring them invents data.
* **sample distance** -- a cell further than ``--max-reach`` from any actual
  sample is left blank. Without this, ``griddata`` cheerfully fills a room the
  robot never entered, which is the single most misleading thing a survey
  tool can do.

Known limitation: the distance mask is euclidean, not geodesic. Two rooms
sampled either side of a thin wall can bleed into each other through it. Doing
this properly means a geodesic distance transform over the free space; it is
worth adding if the floorplans get complicated, and the synthetic fixture
includes exactly such a wall so the effect is visible.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")                     # no display on the robot or in CI
import matplotlib.pyplot as plt           # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, Normalize  # noqa: E402
from scipy.interpolate import griddata    # noqa: E402
from scipy.ndimage import distance_transform_edt  # noqa: E402

import occupancy                          # noqa: E402
from occupancy import FREE                # noqa: E402

# RSSI bands used by every site-survey tool, so the output is readable by
# anyone who has seen one before.
#   >= -50  excellent     -50..-60  good       -60..-67  fair
#   -67..-75 weak         < -75     unusable
RSSI_MIN, RSSI_MAX = -90.0, -40.0

# Sequential ramp, dark-red (weak) through amber to deep teal (strong).
# Deliberately not red-green: ~8% of men have some form of red-green colour
# deficiency, and this chart's whole job is to be read at a glance.
SIGNAL_CMAP = LinearSegmentedColormap.from_list(
    "signal",
    ["#5B1A12", "#A93B2B", "#D98324", "#E8C547", "#5FAE8C", "#0C7C72", "#06403C"],
)

GRID_RES_M = 0.10        # heatmap cell size; finer than this just interpolates noise


def load_survey(path: Path) -> list[dict]:
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"{path}: no rows")
    required = {"x", "y", "rssi", "bssid", "ssid"}
    missing = required - set(rows[0])
    if missing:
        raise SystemExit(f"{path}: missing columns {sorted(missing)}")
    out = []
    for r in rows:
        try:
            out.append({
                "x": float(r["x"]), "y": float(r["y"]),
                "rssi": float(r["rssi"]),
                "bssid": r["bssid"], "ssid": r["ssid"],
                "band": r.get("band", "?"),
                "sample_id": r.get("sample_id", ""),
            })
        except (ValueError, TypeError):
            continue          # skip unparseable rows rather than abort a survey
    return out


def collapse_per_position(rows: list[dict]) -> list[dict]:
    """Take the strongest reading per (position, bssid).

    Rotate-and-average produces several scans at one waypoint (risk R5). For
    a coverage map the strongest reading is the right summary: it answers
    whether a client *could* associate there, which does not depend on which
    way the robot happened to be facing.
    """
    best: dict[tuple, dict] = {}
    for r in rows:
        key = (round(r["x"], 2), round(r["y"], 2), r["bssid"])
        prev = best.get(key)
        if prev is None or r["rssi"] > prev["rssi"]:
            best[key] = r
    return list(best.values())


def interpolate(
    rows: list[dict],
    grid: occupancy.OccupancyGrid | None,
    max_reach_m: float,
    bounds: tuple[float, float, float, float],
) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """Interpolate RSSI onto a regular grid, masking what we cannot claim."""
    xmin, xmax, ymin, ymax = bounds
    nx = max(2, int((xmax - xmin) / GRID_RES_M))
    ny = max(2, int((ymax - ymin) / GRID_RES_M))
    gx = np.linspace(xmin, xmax, nx)
    gy = np.linspace(ymin, ymax, ny)
    mesh_x, mesh_y = np.meshgrid(gx, gy)

    pts = np.array([[r["x"], r["y"]] for r in rows])
    vals = np.array([r["rssi"] for r in rows])

    if len(pts) < 4:
        # Too few points to triangulate; fall back to nearest so a short run
        # still renders something honest rather than crashing.
        field = griddata(pts, vals, (mesh_x, mesh_y), method="nearest")
    else:
        field = griddata(pts, vals, (mesh_x, mesh_y), method="linear")
        near = griddata(pts, vals, (mesh_x, mesh_y), method="nearest")
        # Linear leaves NaN outside the convex hull; backfill with nearest and
        # let the reach mask below decide what is actually defensible.
        field = np.where(np.isnan(field), near, field)

    masked = np.ma.masked_invalid(field)

    # --- mask 1: distance from a real sample -------------------------------
    sampled = np.zeros((ny, nx), dtype=bool)
    for r in rows:
        col = int((r["x"] - xmin) / (xmax - xmin) * (nx - 1))
        row = int((r["y"] - ymin) / (ymax - ymin) * (ny - 1))
        if 0 <= row < ny and 0 <= col < nx:
            sampled[row, col] = True
    cell_m = max((xmax - xmin) / nx, (ymax - ymin) / ny)
    reach_cells = max_reach_m / cell_m
    dist = distance_transform_edt(~sampled)
    masked = np.ma.masked_where(dist > reach_cells, masked)

    # --- mask 2: occupancy --------------------------------------------------
    # Vectorised: a per-cell Python loop here cost ~4 s per panel on a 30x18 m
    # floor, and scales with building size.
    if grid is not None:
        cols = np.floor((gx - grid.origin_x) / grid.resolution).astype(int)
        grows = np.floor((gy - grid.origin_y) / grid.resolution).astype(int)
        in_x = (cols >= 0) & (cols < grid.width)
        in_y = (grows >= 0) & (grows < grid.height)
        cc, rr = np.meshgrid(np.clip(cols, 0, grid.width - 1),
                             np.clip(grows, 0, grid.height - 1))
        inside = np.outer(in_y, in_x)
        free = (grid.cells[rr, cc] == FREE) & inside
        masked = np.ma.masked_where(~free, masked)

    return masked, (xmin, xmax, ymin, ymax)


def draw_panel(
    ax,
    field: np.ma.MaskedArray,
    extent: tuple[float, float, float, float],
    grid: occupancy.OccupancyGrid | None,
    rows: list[dict],
    title: str,
    show_samples: bool,
) -> "matplotlib.image.AxesImage":
    if grid is not None:
        walls = np.ma.masked_where(grid.cells == FREE, grid.cells)
        ax.imshow(
            walls, extent=grid.extent, origin="lower",
            cmap="Greys", vmin=0, vmax=3, alpha=0.5, interpolation="nearest",
        )

    im = ax.imshow(
        field, extent=extent, origin="lower",
        cmap=SIGNAL_CMAP, norm=Normalize(RSSI_MIN, RSSI_MAX),
        interpolation="bilinear", zorder=2,
    )

    if show_samples:
        xs = [r["x"] for r in rows]
        ys = [r["y"] for r in rows]
        ax.plot(
            xs, ys, ".", color="#11191C", markersize=1.6,
            alpha=0.55, zorder=3, label=f"{len(set(zip(xs, ys)))} sample positions",
        )
        ax.legend(loc="upper right", fontsize=7, framealpha=0.85)

    ax.set_title(title, fontsize=11, fontweight="bold", loc="left")
    ax.set_xlabel("x (m)", fontsize=8)
    ax.set_ylabel("y (m)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_aspect("equal")
    return im


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="survey CSV from survey_node")
    ap.add_argument("--map", type=Path, default=None,
                    help="map_server .yaml; without it, no wall masking")
    ap.add_argument("--out", type=Path, default=Path("."), help="output directory")
    ap.add_argument("--max-reach", type=float, default=3.0, metavar="M",
                    help="blank cells further than this from a real sample")
    ap.add_argument("--band", choices=["2.4", "5", "6"], default=None,
                    help="restrict to one band")
    ap.add_argument("--min-samples", type=int, default=8,
                    help="skip SSIDs with fewer readings than this")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    rows = load_survey(args.csv)
    if args.band:
        rows = [r for r in rows if r["band"] == args.band]
        if not rows:
            raise SystemExit(f"no rows in band {args.band}")
    rows = collapse_per_position(rows)

    grid = occupancy.load(args.map) if args.map else None
    args.out.mkdir(parents=True, exist_ok=True)

    if grid is not None:
        bounds = grid.extent
    else:
        xs = [r["x"] for r in rows]
        ys = [r["y"] for r in rows]
        pad = 2.0
        bounds = (min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad)

    by_ssid: dict[str, list[dict]] = defaultdict(list)
    for r in rows:
        by_ssid[r["ssid"] or "<hidden>"].append(r)

    # Best-signal-anywhere: the strongest AP heard at each position. This is
    # the panel that answers the actual question a site survey is asked.
    best_any: dict[tuple, dict] = {}
    for r in rows:
        key = (round(r["x"], 2), round(r["y"], 2))
        prev = best_any.get(key)
        if prev is None or r["rssi"] > prev["rssi"]:
            best_any[key] = r

    panels: list[tuple[str, list[dict], bool]] = [
        ("Best signal, any network", list(best_any.values()), True)
    ]
    for ssid, srows in sorted(by_ssid.items(), key=lambda kv: -len(kv[1])):
        if len(srows) < args.min_samples:
            print(f"  skip {ssid!r}: only {len(srows)} readings")
            continue
        panels.append((ssid, srows, False))

    ncols = min(2, len(panels))
    nrows = math.ceil(len(panels) / ncols)
    fig, axes = plt.subplots(
        nrows, ncols, figsize=(7.4 * ncols, 4.9 * nrows), squeeze=False
    )
    fig.patch.set_facecolor("#FAFBFB")

    im = None
    for idx, (title, srows, show_samples) in enumerate(panels):
        ax = axes[idx // ncols][idx % ncols]
        field, extent = interpolate(srows, grid, args.max_reach, bounds)
        suffix = f"  ({len(srows)} readings)"
        im = draw_panel(ax, field, extent, grid, srows, title + suffix, show_samples)

    for idx in range(len(panels), nrows * ncols):
        axes[idx // ncols][idx % ncols].axis("off")

    cbar = fig.colorbar(
        im, ax=axes.ravel().tolist(), shrink=0.55, pad=0.02, aspect=28
    )
    cbar.set_label("RSSI (dBm)", fontsize=9)
    cbar.ax.tick_params(labelsize=8)
    # Band names sit *inside* the bar. Placing them outside collides with the
    # numeric ticks, which is unreadable at any figure size.
    for lo, hi, label, ink in [
        (-50, -40, "excellent", "#FFFFFF"),
        (-60, -50, "good", "#11191C"),
        (-67, -60, "fair", "#11191C"),
        (-75, -67, "weak", "#11191C"),
        (-90, -75, "unusable", "#FFFFFF"),
    ]:
        cbar.ax.axhline(hi, color="#11191C", linewidth=0.7, alpha=0.35)
        # Rotated to run along the bar: horizontal text is clipped by the
        # bar's width at any readable font size.
        cbar.ax.text(
            0.5, (lo + hi) / 2.0, label, fontsize=6.5, va="center", ha="center",
            rotation=90, transform=cbar.ax.get_yaxis_transform(),
            color=ink, fontweight="bold",
        )

    fig.suptitle(
        f"WiFi coverage — {args.csv.name}"
        + (f"  ·  {args.band} GHz" if args.band else ""),
        fontsize=13, fontweight="bold", x=0.065, ha="left",
    )
    note = (
        f"Blank = no wall-free sample within {args.max_reach:.1f} m. "
        "Grey = wall or unmapped. Measured at ~0.40 m, not user height."
    )
    fig.text(0.065, 0.015, note, fontsize=7.5, color="#65757A")

    stem = args.csv.stem + (f"-{args.band}ghz" if args.band else "")
    out_png = args.out / f"{stem}-heatmap.png"
    fig.savefig(out_png, dpi=args.dpi, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)

    print(f"{len(rows)} readings, {len(by_ssid)} SSID(s), {len(panels)} panel(s)")
    print(f"wrote {out_png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
