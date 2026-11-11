#!/usr/bin/env python3
"""Mid-360 points into the odom frame, beside the built-in LiDAR's.

    (started by: bash tools/go2loop.sh mid360-up)

/livox/lidar (PointCloud2 in the Mid-360's own frame, livox_ros_driver2
xfer_format 0) is placed with the mount from config/mid360_mount.yaml
(tools/mid360_calibrate.py) and the robot's pose from /utlidar/robot_odom,
and republished as /mid360/cloud_odom -- the same frame as
/utlidar/cloud_deskewed, so go2_survey_loop takes it as its second LiDAR
(extra_cloud_topic). Each scan uses the latest pose: at 10 Hz and 0.3 m/s
that is under 3 cm of smear, inside the stops' margins.

Refuses to start without a calibrated mount: placing points with guessed
numbers puts obstacles where they aren't, quietly.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "go2_survey_loop"))
from mid360_mount import rpy_matrix  # noqa: E402
from go2_survey_loop.loop_node import cloud_points  # noqa: E402

MOUNT = Path("/workspace/config/mid360_mount.yaml")

# The robot itself, in its body frame (x forward, z up from the body centre):
# the Mid-360 on the back sees the top of the head 0.30-0.45 m ahead in every
# scan (2026-11-11) -- just past the loop's 0.35 m nose, so its stop check
# counted the robot's own head as an obstacle in front. Padded for leg swing.
SELF_X = (-0.50, 0.50)
SELF_HALF_Y = 0.25
SELF_TOP_Z = 0.30


def read_mount(path: Path) -> dict:
    """The flat key: value file mid360_calibrate.py writes."""
    out = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" in line:
            k, v = line.split(":", 1)
            try:
                out[k.strip()] = float(v)
            except ValueError:
                pass
    return out


def quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Mid360ToOdom(Node):
    def __init__(self, mount: dict) -> None:
        super().__init__("mid360_to_odom")
        r, p, y = (math.radians(mount[k]) for k in ("roll_deg", "pitch_deg", "yaw_deg"))
        self.R_ext = np.array(rpy_matrix(r, p, y))
        self.t_ext = np.array([mount["x"], mount["y"], mount["z"]])
        self.declare_parameter("stride", 2)
        self.stride = int(self.get_parameter("stride").value)
        self.odom = None
        self.create_subscription(Odometry, "/utlidar/robot_odom", self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, os.environ.get("MID360_TOPIC", "/livox/lidar"), self._on_cloud,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, "/mid360/cloud_odom", qos_profile_sensor_data)
        self.n = 0
        self.create_timer(10.0, self._report)
        print(f"mid360_to_odom: mount roll {mount['roll_deg']:.1f} pitch {mount['pitch_deg']:.1f} "
              f"yaw {mount['yaw_deg']:.1f} deg, at ({mount['x']:.2f}, {mount['y']:.2f}, "
              f"{mount['z']:.2f}) m -> /mid360/cloud_odom", flush=True)

    def _on_odom(self, m: Odometry) -> None:
        self.odom = m

    def _on_cloud(self, msg: PointCloud2) -> None:
        o = self.odom
        if o is None:
            return
        pts = np.asarray(cloud_points(msg, self.stride), dtype=np.float64)
        if pts.size == 0:
            return
        q, t = o.pose.pose.orientation, o.pose.pose.position
        R = quat_matrix(q.x, q.y, q.z, q.w)
        base = pts @ self.R_ext.T + self.t_ext                 # Mid-360 -> robot body
        on_robot = ((base[:, 0] >= SELF_X[0]) & (base[:, 0] <= SELF_X[1])
                    & (np.abs(base[:, 1]) <= SELF_HALF_Y) & (base[:, 2] <= SELF_TOP_Z))
        base = base[~on_robot]
        if base.size == 0:
            return
        odom = base @ R.T + np.array([t.x, t.y, t.z])          # robot body -> odom
        out = PointCloud2(header=Header(stamp=msg.header.stamp, frame_id=o.header.frame_id))
        out.height, out.width = 1, len(odom)
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        out.is_bigendian, out.point_step, out.is_dense = False, 12, True
        out.row_step = 12 * len(odom)
        out.data = odom.astype(np.float32).tobytes()
        self.pub.publish(out)
        self.n += 1

    def _report(self) -> None:
        print(f"   {self.n / 10.0:.1f} scans/s republished", flush=True)
        self.n = 0


def main() -> int:
    if not MOUNT.is_file():
        print(f"REFUSED: no {MOUNT} -- calibrate first, robot standing still on a flat floor:\n"
              "   bash tools/go2loop.sh mid360-calibrate", flush=True)
        return 2
    mount = read_mount(MOUNT)
    missing = [k for k in ("roll_deg", "pitch_deg", "yaw_deg", "x", "y", "z") if k not in mount]
    if missing:
        print(f"REFUSED: {MOUNT} lacks {missing}", flush=True)
        return 2
    rclpy.init()
    node = Mid360ToOdom(mount)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
