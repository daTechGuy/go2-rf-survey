#!/usr/bin/env python3
"""Inspect, command a single goal to, or stop the CMU Go2 autonomy stack (D5).

Runs inside the go2-survey:humble container (normally via go2loop.sh):

    bash tools/go2loop.sh cmu-check          # S1: read-only topic/frame check
    bash tools/go2loop.sh cmu-goal 2.0 0.0   # S1/S3: send one /way_point (m, map frame)
    bash tools/go2loop.sh cmu-stop           # S3/S4: publish /stop 15 + sport StopMove
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "go2_survey_loop"))
from go2_survey_loop.cmu import (  # noqa: E402
    CMU_FRAME, SPORT_STOP_MOVE_ID, STOP_ALL, STOP_FREE, assess_cmu,
)

try:
    from unitree_api.msg import Request as SportRequest
except ImportError:  # pragma: no cover - environment
    SportRequest = None

WINDOW = 3.0


def cmd_check(live: bool = False) -> int:
    rclpy.init()
    node = Node("go2_cmu_check")
    counts = {"se": 0, "rs": 0, "ter": 0, "path": 0}
    frames = {"se": None, "rs": None}

    def on_se(m: Odometry) -> None:
        counts["se"] += 1
        frames["se"] = m.header.frame_id

    def on_rs(m: PointCloud2) -> None:
        counts["rs"] += 1
        frames["rs"] = m.header.frame_id

    node.create_subscription(Odometry, "/state_estimation", on_se, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/registered_scan", on_rs, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/terrain_map", lambda _m: counts.__setitem__("ter", counts["ter"] + 1),
                             qos_profile_sensor_data)
    node.create_subscription(PathMsg, "/path", lambda _m: counts.__setitem__("path", counts["path"] + 1), 10)

    def spin(seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        spin(1.0)
        for k in counts:
            counts[k] = 0
        spin(WINDOW)
        sport_pubs = node.count_publishers("/api/sport/request")
        snap = {
            "state_est_hz": counts["se"] / WINDOW,
            "state_est_frame": frames["se"],
            "reg_scan_hz": counts["rs"] / WINDOW,
            "reg_scan_frame": frames["rs"],
            "terrain_hz": counts["ter"] / WINDOW,
            "path_hz": counts["path"] / WINDOW,
            "sport_pub_count": sport_pubs,
        }
        ready, lines = assess_cmu(snap, sensing_only=not live)
        print("\n".join(lines))
        print(f"CMU stack: {'READY' if ready else 'NOT READY'}")
        return 0 if ready else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def cmd_stop() -> int:
    rclpy.init()
    node = Node("go2_cmu_stop")
    stop_pub = node.create_publisher(Int8, "/stop", 10)
    sport_pub = node.create_publisher(SportRequest, "/api/sport/request", 10) if SportRequest else None
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6:
            stop_pub.publish(Int8(data=STOP_ALL))
            if sport_pub is not None:
                req = SportRequest()
                req.header.identity.api_id = SPORT_STOP_MOVE_ID
                sport_pub.publish(req)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        print("sent /stop 15 (block all motion) + sport StopMove")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def cmd_goal(x: float, y: float, z: float = 0.0) -> int:
    rclpy.init()
    node = Node("go2_cmu_goal")
    stop_pub = node.create_publisher(Int8, "/stop", 10)
    wp_pub = node.create_publisher(PointStamped, "/way_point", 10)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6:
            stop_pub.publish(Int8(data=STOP_FREE))
            pt = PointStamped()
            pt.header.frame_id = CMU_FRAME
            pt.header.stamp = node.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
            wp_pub.publish(pt)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        print(f"sent /stop 0 + /way_point ({x:.2f}, {y:.2f}, {z:.2f}) in frame '{CMU_FRAME}'")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if not args or args[0] not in ("check", "stop", "goal"):
        print(__doc__)
        return 2
    cmd = args[0]
    if cmd == "check":
        return cmd_check(live="--live" in args[1:])
    if cmd == "stop":
        return cmd_stop()
    if len(args) < 3:
        print("usage: cmu_ctl.py goal X Y [Z]", file=sys.stderr)
        return 2
    return cmd_goal(float(args[1]), float(args[2]), float(args[3]) if len(args) > 3 else 0.0)


if __name__ == "__main__":
    sys.exit(main())
