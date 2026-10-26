#!/usr/bin/env python3
"""Live map for the laptop view (Foxglove / RViz).

Turns the robot's own LiDAR into a room map that grows as it walks, plus its
trail and current pose. Runs in the container next to foxglove_bridge
(`bash tools/go2loop.sh viz-up`). Read-only with respect to the robot: it
subscribes and publishes visualisation topics, and commands nothing.

Subscribes  /utlidar/cloud_deskewed  (already in the odom frame)
            /utlidar/robot_odom
Publishes   /go2/map_cloud   PointCloud2  accumulated, voxel-downsampled map
            /go2/scan_lite   PointCloud2  decimated live scan (light on the link)
            /go2/path        nav_msgs/Path  the robot's trail
            /go2/pose        PoseStamped    where it is now
Service     ~/reset          clear map and trail (std_srvs/Trigger)

Everything is in the "odom" frame, so a viewer needs no TF: set the 3D
panel's display frame to "odom".

Bandwidth matters on the robot's 100 Mbit link: the raw cloud is several
MB/s, so the map is republished every few seconds at a coarser voxel, and
the live scan is thinned.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pointcloud import VoxelAccumulator, cloud_to_xyz, voxel_downsample, xyz_to_payload  # noqa: E402
from trail import Trail  # noqa: E402

FRAME = "odom"


def make_cloud(pts: np.ndarray, stamp, frame: str = FRAME) -> PointCloud2:
    fields, step, data = xyz_to_payload(pts)
    m = PointCloud2()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in fields]
    m.is_bigendian = False
    m.point_step = step
    m.row_step = step * len(pts)
    m.is_dense = True
    # Bypass the slow per-byte setter, as sensor_msgs_py's create_cloud does
    # (ros2/common_interfaces#176).
    m._data = data
    return m


class LiveMap(Node):
    def __init__(self) -> None:
        super().__init__("go2_live_map")
        d = self.declare_parameter
        d("voxel", 0.05)               # accumulation resolution, m
        d("publish_voxel", 0.08)       # coarser for the link
        d("map_period_s", 2.0)
        d("max_publish_points", 300000)
        d("scan_stride", 4)
        d("scan_rate_hz", 5.0)
        d("path_step_m", 0.05)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.acc = VoxelAccumulator(voxel=g("voxel"), flush_at=300_000)
        self.publish_voxel = g("publish_voxel")
        self.max_pts = int(g("max_publish_points"))
        self.stride = int(g("scan_stride"))
        self.scan_period = 1.0 / max(0.1, g("scan_rate_hz"))
        self.trail = Trail(step_m=g("path_step_m"))
        self._last_scan_pub = 0.0
        self._frame_warned = False
        self._pose = None

        self.map_pub = self.create_publisher(PointCloud2, "/go2/map_cloud", 1)
        self.scan_pub = self.create_publisher(PointCloud2, "/go2/scan_lite", 1)
        self.path_pub = self.create_publisher(PathMsg, "/go2/path", 1)
        self.pose_pub = self.create_publisher(PoseStamped, "/go2/pose", 5)
        self.create_subscription(PointCloud2, "/utlidar/cloud_deskewed", self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/utlidar/robot_odom", self._on_odom,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, "~/reset", self._srv_reset)
        self.create_timer(g("map_period_s"), self._publish_map)
        self.create_timer(0.5, self._publish_path)
        self.get_logger().info(
            f"live map up: /go2/map_cloud every {g('map_period_s')} s at "
            f"{self.publish_voxel} m, /go2/scan_lite {g('scan_rate_hz')} Hz, "
            f"/go2/path, /go2/pose -- display frame '{FRAME}'")

    def _on_cloud(self, m: PointCloud2) -> None:
        if m.header.frame_id != FRAME:
            if not self._frame_warned:
                self.get_logger().error(
                    f"cloud frame '{m.header.frame_id}' is not '{FRAME}' -- not accumulating")
                self._frame_warned = True
            return
        pts = cloud_to_xyz([(f.name, f.offset, f.datatype) for f in m.fields],
                           m.point_step, m.data, m.is_bigendian,
                           width=m.width, height=m.height, row_step=m.row_step)
        self.acc.add(pts)
        now = time.monotonic()
        if now - self._last_scan_pub >= self.scan_period and len(pts):
            self.scan_pub.publish(make_cloud(pts[::self.stride], m.header.stamp))
            self._last_scan_pub = now

    def _on_odom(self, m: Odometry) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.trail.add(p.x, p.y, yaw)
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.header.stamp = m.header.stamp
        ps.pose = m.pose.pose
        self._pose = ps
        self.pose_pub.publish(ps)

    def _publish_map(self) -> None:
        pts = self.acc.result()
        if not len(pts):
            return
        pub = voxel_downsample(pts, self.publish_voxel)
        if len(pub) > self.max_pts:                   # still too big: thin evenly
            pub = pub[:: int(math.ceil(len(pub) / self.max_pts))]
        self.map_pub.publish(make_cloud(pub, self.get_clock().now().to_msg()))

    def _publish_path(self) -> None:
        msg = PathMsg()
        msg.header.frame_id = FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self.trail.points:
            ps = PoseStamped()
            ps.header.frame_id = FRAME
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _srv_reset(self, _req, res):
        self.acc.reset()
        self.trail.reset()
        res.success, res.message = True, "map and trail cleared"
        self.get_logger().info(res.message)
        return res


def main() -> None:
    rclpy.init()
    node = LiveMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
