#!/usr/bin/env python3
"""Read-only readiness snapshot of the robot, judged by readiness.py.

Runs inside the go2-survey:humble container (normally via preflight.sh):

    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/sensor_check.py

Listens for 3 s and asks two read-only questions (motion_switcher CheckMode,
obstacles_avoid SwitchGet). Commands nothing.

Exit code: 0 motion-ready, 1 sensing-ready only, 2 not ready.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from readiness import assess, summary  # noqa: E402
from unitree_rpc import UnitreeRpc, check_mode  # noqa: E402

try:
    from unitree_go.msg import SportModeState
except ImportError:  # pragma: no cover - environment
    SportModeState = None

WINDOW = 3.0


def main() -> int:
    as_json = "--json" in sys.argv
    rclpy.init()
    n = Node("go2_sensor_check")
    counts = {"odom": 0, "cloud": 0, "sport": 0}
    last = {}

    def bump(key):
        counts[key] += 1

    def on_cloud(m):
        bump("cloud")
        last["frame"] = m.header.frame_id

    def on_sport(m):
        bump("sport")
        last["sport"] = m

    n.create_subscription(Odometry, "/utlidar/robot_odom", lambda m: bump("odom"),
                          qos_profile_sensor_data)
    n.create_subscription(PointCloud2, "/utlidar/cloud_deskewed", on_cloud,
                          qos_profile_sensor_data)
    if SportModeState is not None:
        n.create_subscription(SportModeState, "/sportmodestate", on_sport,
                              qos_profile_sensor_data)

    def spin(seconds):
        import time
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.01)

    try:
        spin(1.5)                                   # discovery
        for k in counts:
            counts[k] = 0
        spin(WINDOW)
        snap = {
            "odom_hz": counts["odom"] / WINDOW,
            "cloud_hz": counts["cloud"] / WINDOW,
            "cloud_frame": last.get("frame"),
            "sport_hz": counts["sport"] / WINDOW,
        }
        s = last.get("sport")
        if s is not None:
            snap.update(sport_mode=int(s.mode), gait_type=int(s.gait_type),
                        error_code=int(s.error_code), body_height=float(s.body_height))

        rpc = UnitreeRpc(n)
        mode = check_mode(rpc)
        snap["mode_name"] = None if mode is None or mode[0].startswith("error") else mode[0]
        r = rpc.call("obstacles_avoid", 1002)        # SwitchGet, read-only
        snap["avoid_enabled"] = (r[1].get("enable") if r and r[0] == 0
                                 and isinstance(r[1], dict) else None)

        sensing_ok, motion_ok, lines = assess(snap)
        print("\n".join(lines))
        print(summary(sensing_ok, motion_ok))
        if as_json:
            print(json.dumps(snap))
        return 0 if motion_ok else (1 if sensing_ok else 2)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
