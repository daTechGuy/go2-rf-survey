#!/usr/bin/env python3
"""Make the Gazebo Go2 look like the real one to our code.

    source /opt/ros/humble/setup.bash && source ~/go2_sim_ws/install/setup.bash
    python3 sim/go2_sim_adapter/adapter.py --ros-args -p sim_odom_topic:=/odom

Real robot interface (what our code uses)      <- from the sim
  /api/sport/request -> /api/sport/response     robot_model.py; Move -> sim_cmd_topic
  /api/obstacles_avoid/request|response         robot_model.py
  /api/motion_switcher/request|response         CheckMode -> "normal"
  /utlidar/robot_odom   (frame odom)            sim_odom_topic, re-framed
  /utlidar/cloud_deskewed (frame odom, 15 Hz)   sim_cloud_topic, moved into odom via TF
  /sportmodestate                               robot_model + odometry
  /wirelesscontroller   (read: L2+B damps)  published by fake_remote.py

The sim's controller (CHAMP) must listen on ``sim_cmd_topic`` (default
/sim/cmd_vel), NOT /cmd_vel: /cmd_vel is our loop -> bridge link, and the
sim must only move through the bridge's sport requests, as the robot does.
"""

from __future__ import annotations

import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState, WirelessController

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_model import RobotModel  # noqa: E402

ODOM_FRAME = "odom"


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SimAdapter(Node):
    def __init__(self) -> None:
        super().__init__("go2_sim_adapter")
        d = self.declare_parameter
        d("sim_cmd_topic", "/sim/cmd_vel")
        # "ground_truth" (default): Gazebo's exact base_link pose places both
        # the odometry and the cloud. The robot's /utlidar/* come from
        # Unitree's LiDAR-inertial odometry, which is good; CHAMP's leg-EKF
        # ("ekf", /odom) misses the trot's pitch and bounce, so a walking
        # robot sees its own floor as obstacles and loses its position.
        d("pose_source", "ground_truth")
        d("sim_cloud_topic", "/velodyne_points")
        d("cloud_rate_hz", 15.0)
        d("move_timeout_s", 0.0)
        # Scale the turn rate actually driven. 1.0 = obey; ~0.33 mimics the
        # real Go2 on firmware 'mcf', which turned at ~1/3 of the commanded
        # rate on 2026-10-28 (sim/run_sim.sh: YAW_GAIN=0.33).
        d("yaw_gain", 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.yaw_gain = float(g("yaw_gain"))

        self.model = RobotModel(move_timeout_s=float(g("move_timeout_s")))
        self.pose_source = str(g("pose_source"))
        if self.pose_source not in ("ground_truth", "ekf"):
            raise ValueError("pose_source must be ground_truth or ekf")
        odom_topic = "/odom/ground_truth" if self.pose_source == "ground_truth" else "/odom"
        self._base_to_sensor = None      # static base_link <- LiDAR transform, looked up once
        self._pose_hist = deque(maxlen=400)  # (sim stamp ns, pose): ~4 s of ground truth
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.odom: Odometry | None = None
        self.odom_t = 0.0
        self._cloud_period = 1.0 / max(1.0, g("cloud_rate_hz"))
        self._last_cloud = 0.0
        self._warned_tf = False

        self.cmd_pub = self.create_publisher(Twist, g("sim_cmd_topic"), 10)
        self.odom_pub = self.create_publisher(Odometry, "/utlidar/robot_odom", qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/utlidar/cloud_deskewed", qos_profile_sensor_data)
        self.state_pub = self.create_publisher(SportModeState, "/sportmodestate", qos_profile_sensor_data)

        for name, handler in (("sport", self.model.sport),
                              ("obstacles_avoid", self.model.avoid),
                              ("motion_switcher", self.model.motion_switcher)):
            pub = self.create_publisher(Response, f"/api/{name}/response", 10)
            self.create_subscription(Request, f"/api/{name}/request",
                                     lambda m, h=handler, p=pub, n=name: self._on_request(m, h, p, n), 10)

        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, g("sim_cloud_topic"), self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(WirelessController, "/wirelesscontroller", self._on_remote,
                                 qos_profile_sensor_data)
        self.create_timer(0.02, self._tick)          # 50 Hz command + sport state
        self.create_timer(0.01, self._publish_odom)  # 100 Hz /utlidar/robot_odom
        self.get_logger().info(
            f"sim adapter up: sport/avoid/motion_switcher -> {g('sim_cmd_topic')}; "
            f"{odom_topic} -> /utlidar/robot_odom (pose_source={self.pose_source}); "
            f"{g('sim_cloud_topic')} -> /utlidar/cloud_deskewed (odom frame)")

    # ------------------------------------------------------------ requests
    def _on_request(self, msg: Request, handler, pub, name: str) -> None:
        api = int(msg.header.identity.api_id)
        code, data = handler(api, msg.parameter, time.monotonic())
        for line in self.model.log:
            self.get_logger().info(line)
        self.model.log.clear()
        if msg.header.policy.noreply:
            return
        r = Response()
        r.header.identity.id = msg.header.identity.id
        r.header.identity.api_id = api
        r.header.status.code = int(code)
        r.data = data
        pub.publish(r)

    def _on_remote(self, m: WirelessController) -> None:
        self.model.remote(int(m.keys), m.lx, m.ly, m.rx, m.ry)
        for line in self.model.log:
            self.get_logger().warning(line)
        self.model.log.clear()

    # -------------------------------------------------------------- sensors
    def _on_odom(self, m: Odometry) -> None:
        self.odom, self.odom_t = m, time.monotonic()
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        self._pose_hist.append((stamp, m.pose.pose))     # sim-time stamped

    def _publish_odom(self) -> None:
        """The sim's EKF odom runs ~40 Hz; the robot's ~150 Hz and preflight
        wants >= 50. Repeat the latest pose at 100 Hz -- but only while it is
        fresh, so a sim whose odometry stops still looks blind to our loop."""
        m = self.odom
        if m is None or time.monotonic() - self.odom_t > 0.1:
            return
        out = Odometry()
        out.header.stamp = m.header.stamp
        out.header.frame_id = ODOM_FRAME
        out.child_frame_id = "base_link"
        out.pose = m.pose
        out.twist = m.twist
        self.odom_pub.publish(out)

    def _on_cloud(self, m: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_cloud < self._cloud_period:
            return
        try:
            if self.pose_source == "ground_truth":
                stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
                rot, trans = self._ground_truth_transform(m.header.frame_id, stamp)
            else:
                t = self.tf.lookup_transform(ODOM_FRAME, m.header.frame_id, Time())
                rot = quat_to_matrix(t.transform.rotation)
                tr = t.transform.translation
                trans = np.array([tr.x, tr.y, tr.z])
        except (TransformException, LookupError) as exc:
            if not self._warned_tf:
                self.get_logger().warning(f"cannot place cloud yet: {exc}")
                self._warned_tf = True
            return
        try:
            # read_points, not read_points_numpy: the Velodyne cloud mixes
            # float32 x/y/z with uint16 ring, which read_points_numpy refuses.
            a = point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
            pts = np.column_stack([a["x"], a["y"], a["z"]])
        except Exception as exc:  # noqa: BLE001 -- a bad cloud must not kill the adapter
            self.get_logger().error(f"cloud conversion failed: {exc}",
                                    throttle_duration_sec=10.0)
            return
        if pts.size == 0:
            return
        world = pts.astype(np.float64) @ rot.T + trans
        m.header.frame_id = ODOM_FRAME
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(m.header, world.astype(np.float32)))
        self._last_cloud = now

    def _ground_truth_transform(self, sensor_frame: str, stamp_ns: int):
        """odom <- sensor as (R, t): Gazebo's exact base_link pose *at the
        scan's own sim time*, composed with the URDF's fixed base_link <-
        sensor mount. The latest pose is not good enough: a trotting body
        pitches, and a degree of pitch lifts the floor ~5 cm per 3 m."""
        if self._base_to_sensor is None:
            t = self.tf.lookup_transform("base_link", sensor_frame, Time())
            tr = t.transform.translation
            self._base_to_sensor = (quat_to_matrix(t.transform.rotation),
                                    np.array([tr.x, tr.y, tr.z]))
        if self.odom is None or time.monotonic() - self.odom_t > 0.2 or not self._pose_hist:
            raise LookupError("no fresh ground-truth pose")
        t_near, pose = min(self._pose_hist, key=lambda sp: abs(sp[0] - stamp_ns))
        if abs(t_near - stamp_ns) > 50_000_000:             # 50 ms of sim time
            raise LookupError("no ground-truth pose near the scan's time")
        p, q = pose.position, pose.orientation
        r_wb, t_wb = quat_to_matrix(q), np.array([p.x, p.y, p.z])
        r_bs, t_bs = self._base_to_sensor
        return r_wb @ r_bs, r_wb @ t_bs + t_wb

    # ---------------------------------------------------------------- tick
    def _tick(self) -> None:
        now = time.monotonic()
        vx, vy, wz = self.model.command(now)
        tw = Twist()
        tw.linear.x, tw.linear.y = float(vx), float(vy)
        tw.angular.z = float(wz) * self.yaw_gain
        self.cmd_pub.publish(tw)

        s = SportModeState()
        s.mode = self.model.mode(now)
        s.gait_type = 1 if s.mode == 3 else 0
        s.error_code = 0
        if self.odom is not None:
            p, q, v = self.odom.pose.pose.position, self.odom.pose.pose.orientation, self.odom.twist.twist
            s.position = [float(p.x), float(p.y), float(p.z)]
            s.velocity = [float(v.linear.x), float(v.linear.y), float(v.linear.z)]
            s.yaw_speed = float(v.angular.z)
            s.imu_state.rpy = [0.0, 0.0, float(yaw_of(q))]
            s.imu_state.quaternion = [float(q.w), float(q.x), float(q.y), float(q.z)]
        s.body_height = float(self.model.body_height())
        self.state_pub.publish(s)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
