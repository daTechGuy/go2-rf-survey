#!/usr/bin/env python3
"""Stage 0 against the sim (or the robot): what our obstacle check sees.

Runs go2_survey_loop's real floor finder and classifier on live
/utlidar/cloud_deskewed + /utlidar/robot_odom for a few seconds and prints
floor height and the front/near counts -- the numbers the loop node's start
check uses. Read-only.

    python3 sim/stage0_probe.py [seconds]
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "go2_survey_loop"))
from go2_survey_loop.geometry import Pose2D  # noqa: E402
from go2_survey_loop.obstacles import ObstacleParams, classify, floor_from_scan  # noqa: E402


def main() -> int:
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    n = Node("go2_stage0_probe")
    st = {"pose": None, "rows": []}

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        st["pose"] = Pose2D(p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                  1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_cloud(m):
        if st["pose"] is None:
            return
        a = point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
        pts = list(zip(a["x"].tolist(), a["y"].tolist(), a["z"].tolist()))
        f = floor_from_scan(pts, st["pose"])
        if f is None:
            st["rows"].append((None, None, None, len(pts)))
            return
        prm = ObstacleParams()
        fr, nr = classify(pts, st["pose"], f, prm)
        st["rows"].append((f, fr, nr, len(pts)))
        if fr or nr:
            # where are they, in the robot's own frame? (x fwd, y left, h above floor)
            pz = st["pose"]
            c, s = math.cos(pz.yaw), math.sin(pz.yaw)
            hits = []
            for x, y, z in pts:
                h = z - f
                if h < prm.z_min or h > prm.z_max:
                    continue
                xr = c * (x - pz.x) + s * (y - pz.y)
                yr = -s * (x - pz.x) + c * (y - pz.y)
                if -prm.body_rear <= xr <= prm.body_front and abs(yr) <= prm.body_half_width:
                    continue
                in_front = prm.body_front < xr <= prm.body_front + prm.stop_distance \
                    and abs(yr) <= prm.lane_half_width
                if in_front or xr * xr + yr * yr <= prm.clearance_radius ** 2:
                    hits.append((xr, yr, h))
            if hits:
                a = np.array(hits)
                st.setdefault("hits", []).append(a)
                print(f"  front={fr} near={nr}: x {a[:, 0].min():+.2f}..{a[:, 0].max():+.2f}  "
                      f"y {a[:, 1].min():+.2f}..{a[:, 1].max():+.2f}  "
                      f"h {a[:, 2].min():.2f}..{a[:, 2].max():.2f} m", flush=True)

    n.create_subscription(Odometry, "/utlidar/robot_odom", on_odom, qos_profile_sensor_data)
    n.create_subscription(PointCloud2, "/utlidar/cloud_deskewed", on_cloud, qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.1)
    rows = st["rows"]
    if not rows:
        print("no clouds received")
        return 1
    ok = [r for r in rows if r[0] is not None]
    p = st["pose"]
    print(f"pose x={p.x:.2f} y={p.y:.2f} yaw={math.degrees(p.yaw):.0f} deg; "
          f"{len(rows)} clouds, {rows[-1][3]} points each")
    if not ok:
        print("floor NOT found in any cloud (start would refuse: 'floor not found yet')")
        return 1
    fl = np.median([r[0] for r in ok])
    fr = [r[1] for r in ok]
    nr = [r[2] for r in ok]
    print(f"floor {fl:.2f} m; front min/median/max {min(fr)}/{int(np.median(fr))}/{max(fr)}; "
          f"near {min(nr)}/{int(np.median(nr))}/{max(nr)}  (start refuses at >= 5)")
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
