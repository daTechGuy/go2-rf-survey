#!/usr/bin/env python3
"""Turn a recorded drive into a map: PCD point cloud, top-down PNG, occupancy grid.

    # inside the container, after recording with
    #   ros2 bag record -o /surveys/classroom1 /utlidar/cloud_deskewed /utlidar/robot_odom
    python3 /workspace/tools/bag_to_map.py /surveys/classroom1 --out /surveys/classroom1_map

Writes <out>.pcd, <out>-topdown.png, and <out>.pgm + <out>.yaml (map_server
format, the same one heatmap.py reads).

Why this works without running SLAM: the Go2's own LiDAR stack publishes
/utlidar/cloud_deskewed already in the ``odom`` frame -- every scan is placed
in a world-fixed frame by the robot's own odometry. Stacking the scans IS the
map. Its quality is exactly the quality of that odometry, so drift shows up as
doubled walls. The loop-closure gap printed at the end (start pose vs end
pose, if you returned to where you started) is the honest measure of it.

Runs in the container: rosbag2_py and rclpy from ROS Humble, numpy and
matplotlib from the image. PointCloud2 is parsed with numpy straight from its
field table rather than via sensor_msgs_py, whose read_points return type
changed between distros. Everything except read_bag() is ROS-free, so the map
building can be exercised on synthetic points off-robot.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")                      # headless on the robot
import matplotlib.pyplot as plt            # noqa: E402
from matplotlib.colors import LogNorm      # noqa: E402

import occupancy                           # noqa: E402  (tools/, same dir)
from occupancy import FREE, OCCUPIED, UNKNOWN, OccupancyGrid  # noqa: E402

from pointcloud import (  # noqa: E402  (tools/, same dir; numpy-only)
    VoxelAccumulator, cloud_to_xyz, voxel_downsample,
)


# --- map products -------------------------------------------------------------

def estimate_floor(z: np.ndarray, percentile: float = 1.0) -> float:
    """Floor height in the odom frame. The LiDAR sees the floor around the
    robot everywhere it walked, so the low tail of z is the floor; the 1st
    percentile rather than the minimum ignores stray below-floor returns."""
    return float(np.percentile(z, percentile))


def occupancy_from_points(pts, floor_z, traj=(), res=0.05, obstacle_min=0.15,
                          obstacle_max=1.8, floor_tol=0.08, min_hits=2,
                          fill_cells=2, pad=1.0) -> OccupancyGrid:
    """2-D occupancy grid from a 3-D cloud.

    * occupied -- at least ``min_hits`` points between ``obstacle_min`` and
      ``obstacle_max`` above the floor (walls, desk tops, chair backs). The
      upper limit drops the ceiling and light fittings.
    * free -- floor returns seen there, or the robot physically walked there.
    * unknown -- everything else. Never guessed: an unseen corner stays grey.

    Floor returns are sparse -- far from the robot a 5 cm cell often gets
    none -- so the free mask is morphologically *closed* by ``fill_cells``.
    Closing fills holes up to ~2*fill_cells cells wide without growing the
    region outward, so free space never leaks past a wall it did not see
    through. Set 0 to disable.
    """
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    tx = [p[0] for p in traj]
    ty = [p[1] for p in traj]
    xmin = min(x.min(), *tx) - pad if tx else x.min() - pad
    ymin = min(y.min(), *ty) - pad if ty else y.min() - pad
    xmax = max(x.max(), *tx) + pad if tx else x.max() + pad
    ymax = max(y.max(), *ty) + pad if ty else y.max() + pad
    w = int(math.ceil((xmax - xmin) / res))
    h = int(math.ceil((ymax - ymin) / res))

    def cells(xs, ys):
        c = np.clip(((np.asarray(xs) - xmin) / res).astype(int), 0, w - 1)
        r = np.clip(((np.asarray(ys) - ymin) / res).astype(int), 0, h - 1)
        return r, c

    rel = z - floor_z
    obstacle = (rel > obstacle_min) & (rel < obstacle_max)
    floor = np.abs(rel) < floor_tol

    hits = np.zeros((h, w), dtype=np.int32)
    np.add.at(hits, cells(x[obstacle], y[obstacle]), 1)

    seen = np.zeros((h, w), dtype=bool)
    fr, fc = cells(x[floor], y[floor])
    seen[fr, fc] = True
    if tx:
        tr, tc = cells(tx, ty)
        seen[tr, tc] = True
    if fill_cells:
        from scipy.ndimage import binary_closing
        seen |= binary_closing(seen, structure=np.ones((3, 3), dtype=bool),
                               iterations=fill_cells)

    cells_grid = np.full((h, w), UNKNOWN, dtype=np.uint8)
    cells_grid[seen] = FREE
    cells_grid[hits >= min_hits] = OCCUPIED
    return OccupancyGrid(cells_grid, res, xmin, ymin)


def write_pcd(path: Path, pts: np.ndarray) -> None:
    """Binary PCD v0.7, float32 x y z. Opens in CloudCompare, PCL, Open3D."""
    n = len(pts)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with path.open("wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(pts.astype("<f4").tobytes())


def render_topdown(path: Path, pts, floor_z, traj, title, res=0.05,
                   obstacle_min=0.15, obstacle_max=1.8) -> None:
    """The picture to show people: walls and furniture from above, the floor
    the robot saw as a faint wash, and the path it walked."""
    rel = pts[:, 2] - floor_z
    obst = pts[(rel > obstacle_min) & (rel < obstacle_max)]
    flo = pts[np.abs(rel) < 0.08]

    xs = np.concatenate([pts[:, 0], [p[0] for p in traj]]) if traj else pts[:, 0]
    ys = np.concatenate([pts[:, 1], [p[1] for p in traj]]) if traj else pts[:, 1]
    xe = np.arange(xs.min() - 0.5, xs.max() + 0.5 + res, res)
    ye = np.arange(ys.min() - 0.5, ys.max() + 0.5 + res, res)

    span_x, span_y = xe[-1] - xe[0], ye[-1] - ye[0]
    scale = 9.0 / max(span_x, span_y)
    fig, ax = plt.subplots(figsize=(max(4.0, span_x * scale) + 1.2,
                                    max(4.0, span_y * scale) + 1.0))
    fig.patch.set_facecolor("#FAFBFB")
    ax.set_facecolor("#FFFFFF")

    if len(flo):
        fh, _, _ = np.histogram2d(flo[:, 0], flo[:, 1], bins=[xe, ye])
        ax.imshow(np.where(fh.T > 0, 1.0, np.nan), extent=(xe[0], xe[-1], ye[0], ye[-1]),
                  origin="lower", cmap="Greys", vmin=0, vmax=8, interpolation="nearest")
    if len(obst):
        oh, _, _ = np.histogram2d(obst[:, 0], obst[:, 1], bins=[xe, ye])
        oh = np.where(oh > 0, oh, np.nan)
        ax.imshow(oh.T, extent=(xe[0], xe[-1], ye[0], ye[-1]), origin="lower",
                  cmap="viridis_r", norm=LogNorm(vmin=1, vmax=max(2, np.nanmax(oh))),
                  interpolation="nearest")
    if traj:
        tx = [p[0] for p in traj]
        ty = [p[1] for p in traj]
        ax.plot(tx, ty, "-", color="#D1495B", lw=1.4, alpha=0.9, label="robot path")
        ax.plot(tx[0], ty[0], "o", color="#D1495B", ms=7, label="start")
        ax.plot(tx[-1], ty[-1], "s", color="#2E4057", ms=7, label="end")
        ax.legend(loc="upper right", fontsize=8, framealpha=0.9)

    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", fontsize=8)
    ax.set_ylabel("y (m)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_title(title, fontsize=11, fontweight="bold", loc="left")
    fig.savefig(path, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def finish_map(pts, traj, out: Path, res=0.05, title="map") -> dict:
    """Everything after accumulation. ROS-free; takes points and (x, y) path."""
    if len(pts) == 0:
        raise SystemExit("no points accumulated -- nothing to map")
    floor_z = estimate_floor(pts[:, 2])
    out.parent.mkdir(parents=True, exist_ok=True)

    write_pcd(out.with_suffix(".pcd"), pts)
    grid = occupancy_from_points(pts, floor_z, traj=traj, res=res)
    occupancy.save(grid, out.with_suffix(".yaml"))

    gap = None
    path_len = 0.0
    if len(traj) >= 2:
        gap = math.hypot(traj[-1][0] - traj[0][0], traj[-1][1] - traj[0][1])
        path_len = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(traj, traj[1:]))

    sub = title
    if gap is not None:
        sub += f"  ·  path {path_len:.1f} m, start-end gap {gap:.2f} m"
    render_topdown(out.with_name(out.name + "-topdown.png"), pts, floor_z, traj, sub, res=res)

    return {
        "points": len(pts), "floor_z": floor_z, "grid": (grid.width, grid.height),
        "extent": grid.extent, "path_len": path_len, "gap": gap,
    }


# --- bag reading (the only ROS-dependent part) --------------------------------

def _storage_id(uri: Path) -> str:
    meta = uri / "metadata.yaml"
    if meta.exists():
        m = re.search(r"storage_identifier:\s*(\S+)", meta.read_text())
        if m:
            return m.group(1)
    return "sqlite3"


# The robot's own outline in base_link (x forward, y left), for dropping the
# LiDAR's view of its own legs and body from the map. Larger than the obstacle
# check's box (go2_survey_loop/obstacles.py): legs mid-stride reach past the
# standing footprint -- 0.41 m behind centre in the Gazebo walk test -- and a
# map has no reason to keep anything inside the robot.
BODY_FRONT, BODY_REAR, BODY_HALF_WIDTH = 0.40, 0.45, 0.25


def drop_body(xyz: np.ndarray, pose, front=BODY_FRONT, rear=BODY_REAR,
              half_width=BODY_HALF_WIDTH, max_range=None) -> np.ndarray:
    """Points of one scan outside the robot's outline at ``pose`` (x, y, yaw)
    and, if given, within ``max_range`` of it (horizontally)."""
    if pose is None or len(xyz) == 0:
        return xyz
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = xyz[:, 0] - x, xyz[:, 1] - y
    xr, yr = c * dx + s * dy, -s * dx + c * dy
    drop = (xr >= -rear) & (xr <= front) & (np.abs(yr) <= half_width)
    if max_range:
        drop |= dx * dx + dy * dy > max_range * max_range
    return xyz[~drop]


def read_bag(uri: Path, cloud_topic: str, odom_topic: str):
    """Yield ('cloud', xyz, frame_id) and ('odom', (x, y, yaw)) in recorded order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=_storage_id(uri)),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if cloud_topic not in types:
        raise SystemExit(f"{cloud_topic} not in bag; topics: {sorted(types)}")
    cloud_cls = get_message(types[cloud_topic])
    odom_cls = get_message(types[odom_topic]) if odom_topic in types else None

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == cloud_topic:
            m = deserialize_message(data, cloud_cls)
            fields = [(f.name, f.offset, f.datatype) for f in m.fields]
            xyz = cloud_to_xyz(fields, m.point_step, m.data, m.is_bigendian,
                               width=m.width, height=m.height, row_step=m.row_step)
            yield "cloud", xyz, m.header.frame_id
        elif odom_cls is not None and topic == odom_topic:
            m = deserialize_message(data, odom_cls)
            pose = m.pose.pose if hasattr(m.pose, "pose") else m.pose
            q = pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            yield "odom", (pose.position.x, pose.position.y, yaw)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path, help="bag directory from ros2 bag record")
    ap.add_argument("--out", type=Path, default=None,
                    help="output prefix (default: <bag>_map next to the bag)")
    ap.add_argument("--cloud-topic", default="/utlidar/cloud_deskewed")
    ap.add_argument("--odom-topic", default="/utlidar/robot_odom")
    ap.add_argument("--voxel", type=float, default=0.05, help="map resolution, m")
    ap.add_argument("--every", type=int, default=1,
                    help="use every Nth scan (raise to go faster on long drives)")
    ap.add_argument("--force", action="store_true",
                    help="build even if clouds are not in a world-fixed frame")
    ap.add_argument("--keep-body", action="store_true",
                    help="keep the LiDAR's view of the robot itself (legs) in the map")
    ap.add_argument("--max-range", type=float, default=15.0,
                    help="ignore returns further than this from the robot, m (0 = keep all). "
                         "A low ray through an open door can land tens of metres away "
                         "and stretch the floorplan to fit it.")
    args = ap.parse_args()

    out = args.out or args.bag.with_name(args.bag.name + "_map")
    acc = VoxelAccumulator(voxel=args.voxel)
    traj: list[tuple[float, float]] = []
    frames: set[str] = set()
    n = 0
    pose = None          # latest odometry (x, y, yaw), to drop the robot from each scan
    dropped = 0

    for item in read_bag(args.bag, args.cloud_topic, args.odom_topic):
        if item[0] == "cloud":
            _, xyz, frame = item
            frames.add(frame)
            n += 1
            if n % args.every == 0:
                if not args.keep_body or args.max_range:
                    kept = drop_body(xyz, pose, max_range=args.max_range or None,
                                     **({"front": -1e9} if args.keep_body else {}))
                    dropped += len(xyz) - len(kept)
                    xyz = kept
                acc.add(xyz)
            if n % 200 == 0:
                print(f"  {n} scans read, {acc.raw_points:,} raw points", flush=True)
        else:
            pose = item[1]
            traj.append(pose[:2])

    if not frames:
        raise SystemExit("no clouds read from the bag")
    world = {"odom", "map", "world"}
    if not frames <= world and not args.force:
        raise SystemExit(
            f"clouds are in frame(s) {sorted(frames)}, not a world-fixed frame. "
            "Stacking sensor-frame scans makes a smear, not a map. "
            "Re-run with --force only if you know better.")

    pts = acc.result()
    stats = finish_map(pts, traj, out, res=args.voxel, title=f"Go2 map — {args.bag.name}")

    print(f"scans      {acc.scans} used of {n}, {acc.raw_points:,} raw -> {stats['points']:,} voxels")
    if not args.keep_body:
        print(f"filtered   {dropped:,} points left out: the robot itself and returns "
              f"beyond {args.max_range:g} m")
    print(f"floor      z = {stats['floor_z']:.2f} m in {sorted(frames)[0]}")
    x0, x1, y0, y1 = stats["extent"]
    print(f"extent     {x1 - x0:.1f} x {y1 - y0:.1f} m ({stats['grid'][0]}x{stats['grid'][1]} cells)")
    if stats["gap"] is not None:
        pct = 100 * stats["gap"] / stats["path_len"] if stats["path_len"] else float("nan")
        print(f"loop       path {stats['path_len']:.1f} m, start-end gap {stats['gap']:.2f} m "
              f"({pct:.1f}% -- only meaningful if you finished where you started)")
    print(f"wrote      {out.with_suffix('.pcd')}")
    print(f"           {out.name}-topdown.png, {out.with_suffix('.pgm').name}, "
          f"{out.with_suffix('.yaml').name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
