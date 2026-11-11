#!/usr/bin/env python3
"""Measure where the Mid-360 sits, from the floor it sees. Nothing moves.

    (run by: bash tools/go2loop.sh mid360-calibrate -- robot STANDING STILL
     on a flat floor, ideally a few metres from walls on more than one side)

Collects a few seconds of /livox/lidar, /utlidar/cloud_deskewed and
/utlidar/robot_odom, then:
  * pitch, roll, height of the Mid-360 over the floor -- the floor plane in
    its own view, or the ceiling's when it sees more ceiling than floor
    (as on the robot), with the height from the floor sliver it sees ahead
    (mid360_mount.fit_floor, floor_below);
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
from mid360_mount import (apply, fit_floor, floor_below, icp_2d, mount_from_floor,  # noqa: E402
                          mount_in_body, rpy_matrix, wall_yaw_offset)
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
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.odom.append((p.x, p.y, p.z, yaw_from_quaternion(q.x, q.y, q.z, q.w), roll, pitch))


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

    x0, y0, z0, yaw0, body_roll, body_pitch = (sum(v) / len(v) for v in zip(*odom))
    pose = Pose2D(x0, y0, yaw0)
    floors = [f for f in (floor_from_scan(s, pose) for s in ut) if f is not None]
    if not floors:
        print("FAIL the built-in LiDAR found no floor")
        return 1
    floor_z = sorted(floors)[len(floors) // 2]
    base_h = z0 - floor_z                                   # odometry body height over the floor

    # The level reference: the floor if the Mid-360 sees enough of it, else
    # the ceiling (it looks -7..+52 deg, so mounted ~13 deg nose-down it sees
    # floor only from ~1.6 m ahead, but ceiling all round).
    pts = livox[:: max(1, len(livox) // 20000)]
    floor_fit, ceil_fit = fit_floor(pts), fit_floor(pts, ceiling=True)
    if floor_fit is None and ceil_fit is None:
        print("FAIL no floor or ceiling found in the Mid-360's view")
        return 1
    if ceil_fit is not None and (floor_fit is None or ceil_fit[2] > floor_fit[2]):
        # tilt from the ceiling; height from the floor it does see, measured
        # along the ceiling's up (the built-in cloud is cut off ~1.3 m up,
        # so it can't give the room height)
        roll, pitch, d = mount_from_floor(ceil_fit[0], ceil_fit[1])
        h_sensor = floor_below(pts, ceil_fit[0])
        if h_sensor is None:
            print(f"FAIL the Mid-360 sees the ceiling ({ceil_fit[2]} points) but no floor "
                  f"-- give it 2 m of open floor ahead")
            return 1
        print(f"ceiling: {ceil_fit[2]} of {len(pts)} points (floor plane only "
              f"{floor_fit[2] if floor_fit else 0}), {d:.2f} m above the sensor; "
              f"floor {h_sensor:.3f} m below it along the ceiling's up")
    else:
        roll, pitch, h_sensor = mount_from_floor(floor_fit[0], floor_fit[1])
        print(f"floor: {floor_fit[2]} of {len(pts)} points")
    print(f"sensor {h_sensor:.3f} m above the floor, pitch {math.degrees(pitch):.1f} deg "
          f"(design {DESIGN_PITCH_DEG:.0f}), roll {math.degrees(roll):.1f} deg")
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

    # refine yaw and x/y on what both LiDARs see (the built-in cloud covers
    # only part of the room; icp_2d ignores the rest)
    warn = []
    yaw = math.radians(yaw_off)
    c, s = math.cos(yaw), math.sin(yaw)
    ref = list({(round(x / 0.02), round(y / 0.02)): (x, y) for x, y in ut_xy}.values())
    fit = icp_2d(ref, [(c * x - s * y, s * x + c * y) for x, y in mid_xy[:: max(1, len(mid_xy) // 4000)]])
    if fit is not None and fit[3] >= 300 and abs(math.degrees(fit[0])) <= 10 and math.hypot(fit[1], fit[2]) <= 0.25:
        th, sx, sy, n_pairs, gap = fit
        # p' = R(th) (R(yaw) (R0 p + t)) + s  ->  yaw + th, t_xy rotated by both, plus s
        yaw += th
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = c * lx - s * ly + sx, s * lx + c * ly + sy
        print(f"refined on {n_pairs} shared wall points: yaw {math.degrees(yaw):+.1f} deg, "
              f"at x {lx:+.3f} y {ly:+.3f} m; median gap {100 * gap:.1f} cm")
        if gap > 0.04:
            warn.append(f"median gap to the built-in LiDAR {100 * gap:.1f} cm -- expected ~1.5")
    else:
        warn.append("could not refine on shared walls -- yaw from wall directions only, x/y from mount.yaml")

    # the floor/ceiling measured the mount against level; mid360_to_odom
    # applies the body's own tilt too, so take the body's tilt out
    level = (roll, pitch, yaw)
    (roll, pitch, yaw), (lx, ly, z_ext) = mount_in_body(roll, pitch, yaw, (lx, ly, z_ext),
                                                        body_roll, body_pitch)
    print(f"body tilt while measuring: roll {math.degrees(body_roll):+.1f} pitch "
          f"{math.degrees(body_pitch):+.1f} deg -> mount on the body: roll {math.degrees(roll):+.2f} "
          f"pitch {math.degrees(pitch):+.2f} yaw {math.degrees(yaw):+.2f} deg")

    if abs(math.degrees(pitch) - DESIGN_PITCH_DEG) > 8:
        warn.append(f"pitch {math.degrees(pitch):.1f} deg is far from the {DESIGN_PITCH_DEG:.0f} deg design")
    if not 0.0 <= z_ext <= 0.35:
        warn.append(f"sensor {z_ext:+.2f} m above the body: expected 0-0.35 on the back")
    if abs(math.degrees(roll)) > 3:
        warn.append(f"roll {math.degrees(roll):.1f} deg: is the mount straight, the floor flat?")
    out = CONFIG / "mid360_mount.yaml"
    out.write_text(
        "# Mid-360 mount on the robot body, measured by tools/mid360_calibrate.py\n"
        f"# {time.strftime('%Y-%m-%d %H:%M:%S')}: body {base_h:.3f} m over the floor, tilted "
        f"roll {math.degrees(body_roll):+.2f} pitch {math.degrees(body_pitch):+.2f} deg\n"
        f"# against level: roll {math.degrees(level[0]):+.2f} pitch {math.degrees(level[1]):+.2f} "
        f"yaw {math.degrees(level[2]):+.2f} deg\n"
        "# roll/pitch/z from the floor or ceiling, yaw and x/y from the walls both LiDARs see\n"
        f"roll_deg: {math.degrees(roll):.2f}\n"
        f"pitch_deg: {math.degrees(pitch):.2f}\n"
        f"yaw_deg: {math.degrees(yaw):.2f}\n"
        f"x: {lx:.3f}\n"
        f"y: {ly:.3f}\n"
        f"z: {z_ext:.3f}\n")
    print(f"wrote {out}")
    for w in warn:
        print(f"WARN {w}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
