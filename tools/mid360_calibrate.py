#!/usr/bin/env python3
"""Measure where the Mid-360 sits, from the floor it sees. Nothing moves.

    (run by: bash tools/go2loop.sh mid360-calibrate -- robot STANDING STILL
     on a flat floor, ideally a few metres from walls on more than one side)

Collects a few seconds of /livox/lidar, /utlidar/cloud_deskewed and
/utlidar/robot_odom, then:
  * pitch, roll, height of the Mid-360 over the floor -- the floor plane in
    its own view (mid360_mount.fit_floor);
  * its height relative to the robot body, using the floor height the
    built-in LiDAR sees and the odometry's z;
  * yaw -- lining the walls it sees up with the built-in LiDAR's
    (mid360_mount.wall_yaw_offset; assumes it is mounted facing forward
    within +-45 deg);
  * forward / sideways offset: from config/mount.yaml (lidar_x, lidar_y) --
    the floor can't tell, and a few cm don't matter to the stops.
and writes config/mid360_mount.yaml for tools/mid360_to_odom.py.
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "go2_survey_loop"))
from mid360_mount import apply, fit_floor, mount_from_floor, rpy_matrix, wall_yaw_offset  # noqa: E402
from go2_survey_loop.geometry import yaw_from_quaternion  # noqa: E402
from go2_survey_loop.loop_node import cloud_points  # noqa: E402
from go2_survey_loop.obstacles import floor_from_scan  # noqa: E402
from go2_survey_loop.geometry import Pose2D  # noqa: E402

CONFIG = Path("/workspace/config")
DESIGN_PITCH_DEG = 22.0


def mount_yaml_xy(path: Path):
    """lidar_x / lidar_y from config/mount.yaml (placeholders until measured)."""
    vals = {"lidar_x": 0.16, "lidar_y": 0.0}
    if path.is_file():
        for line in path.read_text().splitlines():
            k, _, v = line.split("#", 1)[0].partition(":")
            if k.strip() in vals:
                try:
                    vals[k.strip()] = float(v)
                except ValueError:
                    pass
    return vals["lidar_x"], vals["lidar_y"]


class Collect(Node):
    def __init__(self) -> None:
        super().__init__("mid360_calibrate")
        self.livox, self.ut, self.odom = [], [], []
        self.create_subscription(PointCloud2, os.environ.get("MID360_TOPIC", "/livox/lidar"), self._on_livox,
                                 qos_profile_sensor_data)   # MID360_TOPIC: the sim stands its LiDAR in
        self.create_subscription(PointCloud2, "/utlidar/cloud_deskewed", self._on_ut,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/utlidar/robot_odom", self._on_odom,
                                 qos_profile_sensor_data)

    def _on_livox(self, m):
        self.livox.extend(cloud_points(m, 4))

    def _on_ut(self, m):
        self.ut.append(cloud_points(m, 2))

    def _on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom.append((p.x, p.y, p.z, yaw_from_quaternion(q.x, q.y, q.z, q.w)))


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    n = Collect()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    livox, ut, odom = n.livox, n.ut, n.odom
    n.destroy_node()
    rclpy.try_shutdown()

    print(f"collected: {len(livox)} Mid-360 points, {len(ut)} built-in scans, {len(odom)} poses")
    if len(livox) < 500:
        print("FAIL too few Mid-360 points -- is the driver up? (go2loop.sh mid360-check)")
        return 1
    if len(odom) < 10 or not ut:
        print("FAIL no odometry / built-in LiDAR -- is the robot on and connected?")
        return 1
    xs, ys = [o[0] for o in odom], [o[1] for o in odom]
    if max(xs) - min(xs) > 0.03 or max(ys) - min(ys) > 0.03:
        print("FAIL the robot moved while measuring -- keep it standing still and try again")
        return 1

    pts = livox[:: max(1, len(livox) // 20000)]
    fit = fit_floor(pts)
    if fit is None:
        print("FAIL no floor found in the Mid-360's view -- flat open floor around it?")
        return 1
    roll, pitch, h_sensor = mount_from_floor(fit[0], fit[1])
    print(f"floor: {fit[2]} of {len(pts)} points; sensor {h_sensor:.3f} m above it, "
          f"pitch {math.degrees(pitch):.1f} deg (design {DESIGN_PITCH_DEG:.0f}), "
          f"roll {math.degrees(roll):.1f} deg")

    x0, y0, z0, yaw0 = (sum(v) / len(v) for v in zip(*odom))
    pose = Pose2D(x0, y0, yaw0)
    floors = [f for f in (floor_from_scan(s, pose) for s in ut) if f is not None]
    if not floors:
        print("FAIL the built-in LiDAR found no floor")
        return 1
    floor_z = sorted(floors)[len(floors) // 2]
    base_h = z0 - floor_z                                   # odometry body height over the floor
    lx, ly = mount_yaml_xy(CONFIG / "mount.yaml")
    z_ext = h_sensor - base_h

    # yaw: walls 0.3-1.5 m above the floor, both in the body frame
    R0 = rpy_matrix(roll, pitch, 0.0)
    mid_xy = []
    for p in pts:
        b = apply(R0, (lx, ly, z_ext), p)
        if 0.3 <= b[2] + base_h <= 1.5 and math.hypot(b[0], b[1]) > 0.6:
            mid_xy.append((b[0], b[1]))
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    ut_xy = []
    for scan in ut:
        for x, y, z in scan[::2]:
            dx, dy = x - x0, y - y0
            bx, by = c * dx - s * dy, s * dx + c * dy
            if 0.3 <= z - floor_z <= 1.5 and math.hypot(bx, by) > 0.6:
                ut_xy.append((bx, by))
    yaw_off = wall_yaw_offset(ut_xy, mid_xy)
    if yaw_off is None:
        print("WARN not enough wall in view to check yaw -- assuming 0 (facing forward)")
        yaw_off = 0.0
    else:
        print(f"yaw: walls line up after rotating the Mid-360's view {yaw_off:+.1f} deg")

    warn = []
    if abs(math.degrees(pitch) - DESIGN_PITCH_DEG) > 8:
        warn.append(f"pitch {math.degrees(pitch):.1f} deg is far from the {DESIGN_PITCH_DEG:.0f} deg design")
    if abs(math.degrees(roll)) > 3:
        warn.append(f"roll {math.degrees(roll):.1f} deg: is the mount straight, the floor flat?")
    out = CONFIG / "mid360_mount.yaml"
    out.write_text(
        "# Mid-360 mount on the robot body, measured by tools/mid360_calibrate.py\n"
        f"# {time.strftime('%Y-%m-%d %H:%M:%S')}: floor fit {fit[2]} points, "
        f"body {base_h:.3f} m over the floor\n"
        "# roll/pitch/z from the floor, yaw from the walls, x/y from config/mount.yaml\n"
        f"roll_deg: {math.degrees(roll):.2f}\n"
        f"pitch_deg: {math.degrees(pitch):.2f}\n"
        f"yaw_deg: {yaw_off:.2f}\n"
        f"x: {lx:.3f}\n"
        f"y: {ly:.3f}\n"
        f"z: {z_ext:.3f}\n")
    print(f"wrote {out}")
    for w in warn:
        print(f"WARN {w}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
