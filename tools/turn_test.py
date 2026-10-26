#!/usr/bin/env python3
"""Stage 1, instrumented: turn in place, stop sending, prove the watchdog
stops it -- and log enough to say *why* if it does not.

Runs inside the bridge container, with go2_survey_bridge already up:

    python3 /workspace/tools/turn_test.py --i-confirm-breathing
    python3 /workspace/tools/turn_test.py --i-confirm-breathing --vyaw -0.2   # clockwise

Logs at ~50 Hz to /surveys/turn_<time>.csv: what the bridge actually sent
(decoded from /api/sport/request), heading from the LiDAR odometry AND from
the IMU (/sportmodestate), the IMU yaw rate, and the sport mode/gait/error
code. Then prints turn_analysis's report: PASS only for a prompt,
proportionate turn that stops cleanly with both heading sources agreeing.

It REFUSES to command anything unless all of these hold:
  * --i-confirm-breathing is given. The robot must be in walking mode -- on the
    remote, L2+A then a SINGLE START, body visibly balancing ("breathing").
    The first Stage 1 run skipped this check; this flag exists because of it.
  * |vyaw| <= 0.3 rad/s and seconds <= 3.0 (hard caps, not defaults)
  * odometry and /sportmodestate are live, and the bridge is subscribed
  * the robot is not already rotating
It never sends a zero to end the test: stopping is the watchdog's job, which
is the thing under test.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parent))
from turn_analysis import MOVE, STOP_MOVE, analyse, format_report, passed  # noqa: E402
from unitree_rpc import UnitreeRpc, check_mode  # noqa: E402

try:
    from unitree_api.msg import Request
    from unitree_go.msg import SportModeState
except ImportError as exc:  # pragma: no cover - environment
    raise SystemExit("needs unitree_api/unitree_go: run inside the go2-survey:humble "
                     "container and `source /opt/unitree_msgs/setup.bash`") from exc

MAX_VYAW, MAX_SECONDS = 0.3, 3.0
FIELDS = ["t", "phase", "cmd", "api", "odom_yaw", "imu_yaw", "yaw_speed",
          "mode", "gait_type", "error_code"]


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("go2_turn_test")
        self.odom = self.odom_t = None
        self.sport = self.sport_t = None
        self.cmd, self.api = 0.0, None
        self.create_subscription(Odometry, "/utlidar/robot_odom", self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(SportModeState, "/sportmodestate", self._on_sport,
                                 qos_profile_sensor_data)
        self.create_subscription(Request, "/api/sport/request", self._on_request, 50)
        # The bridge's avoid backend drives through obstacles_avoid instead.
        self.create_subscription(Request, "/api/obstacles_avoid/request",
                                 self._on_avoid_request, 50)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_odom(self, m):
        self.odom, self.odom_t = yaw_of(m.pose.pose.orientation), time.monotonic()

    def _on_sport(self, m):
        self.sport, self.sport_t = m, time.monotonic()

    def _on_request(self, m):
        api = int(m.header.identity.api_id)
        self.api = api
        if api == MOVE:
            try:
                self.cmd = float(json.loads(m.parameter).get("z", 0.0))
            except (ValueError, TypeError):
                self.cmd = float("nan")
        elif api == STOP_MOVE:
            self.cmd = 0.0

    def _on_avoid_request(self, m):
        # obstacles_avoid Move is api 1003 -- the SAME number as sport
        # StopMove -- with the turn rate in "yaw", not "z". Normalise to the
        # sport ids turn_analysis works in: non-zero Move -> MOVE, zero -> STOP.
        if int(m.header.identity.api_id) != 1003:
            return                            # SwitchSet / UseRemote: not motion
        try:
            yaw = float(json.loads(m.parameter).get("yaw", 0.0))
        except (ValueError, TypeError):
            yaw = float("nan")
        self.cmd = yaw
        self.api = MOVE if yaw != 0.0 else STOP_MOVE

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def row(self, t: float, phase: str) -> dict:
        s = self.sport
        return {
            "t": round(t, 4), "phase": phase, "cmd": self.cmd, "api": self.api,
            "odom_yaw": self.odom,
            "imu_yaw": None if s is None else float(s.imu_state.rpy[2]),
            "yaw_speed": None if s is None else float(s.yaw_speed),
            "mode": None if s is None else int(s.mode),
            "gait_type": None if s is None else int(s.gait_type),
            "error_code": None if s is None else int(s.error_code),
        }


def preflight(p: Probe, args, mode) -> list:
    problems = []
    if args.allow_mode != "any":
        if mode is None:
            problems.append("motion_switcher did not answer CheckMode "
                            "(override with --allow-mode any)")
        elif mode[0] != args.allow_mode:
            problems.append(f"motion mode is '{mode[0]}', not '{args.allow_mode}': sport "
                            "Move is documented for 'normal' (override with --allow-mode)")
    if not args.i_confirm_breathing:
        problems.append("robot walking mode not confirmed: remote L2+A then a SINGLE "
                        "START, body visibly breathing; then pass --i-confirm-breathing")
    if abs(args.vyaw) > MAX_VYAW or args.vyaw == 0.0:
        problems.append(f"--vyaw must be non-zero and within +/-{MAX_VYAW} rad/s")
    if not 0.0 < args.seconds <= MAX_SECONDS:
        problems.append(f"--seconds must be in (0, {MAX_SECONDS}]")
    now = time.monotonic()
    if p.odom_t is None or now - p.odom_t > 0.5:
        problems.append("no fresh /utlidar/robot_odom")
    if p.sport_t is None or now - p.sport_t > 0.5:
        problems.append("no fresh /sportmodestate")
    if p.pub.get_subscription_count() == 0:
        problems.append("bridge not subscribed to /cmd_vel (is go2_survey_bridge up?)")
    if p.sport is not None and abs(p.sport.yaw_speed) > 0.05:
        problems.append(f"robot already rotating ({p.sport.yaw_speed:.2f} rad/s)")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vyaw", type=float, default=0.2, help="rad/s, + = CCW (left)")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--settle", type=float, default=5.0, help="logging after the stop")
    ap.add_argument("--rate", type=float, default=20.0, help="command publish rate, Hz")
    ap.add_argument("--i-confirm-breathing", action="store_true")
    ap.add_argument("--allow-mode", default="normal",
                    help="required motion_switcher mode name, or 'any' to skip the check")
    ap.add_argument("--csv", type=Path, default=None)
    args = ap.parse_args()

    rclpy.init()
    p = Probe()
    try:
        p.spin_for(2.0)                                  # discovery + first samples
        mode = check_mode(UnitreeRpc(p))
        print(f"motion mode       : {'no reply' if mode is None else f'{mode[0]!r} (form {mode[1]!r})'}")
        problems = preflight(p, args, mode)
        s = p.sport
        if s is not None:
            print(f"sport state before: mode={s.mode} gait_type={s.gait_type} "
                  f"error_code={s.error_code} body_height={s.body_height:.3f} m")
        if problems:
            print("REFUSED -- nothing was commanded:")
            for msg in problems:
                print(f"  - {msg}")
            return 2

        out = args.csv or Path(f"/surveys/turn_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        rows, t0 = [], time.monotonic()
        tw = Twist()
        tw.angular.z = float(args.vyaw)
        period, next_pub, next_log = 1.0 / args.rate, 0.0, 0.0

        def tick(phase: str, publish: bool) -> None:
            nonlocal next_pub, next_log
            now = time.monotonic() - t0
            if publish and now >= next_pub:
                p.pub.publish(tw)
                next_pub = now + period
            if now >= next_log:
                rows.append(p.row(now, phase))
                next_log = now + 0.02
            rclpy.spin_once(p, timeout_sec=0.002)

        while time.monotonic() - t0 < 1.0:               # baseline, robot still
            tick("pre", False)
        end_cmd = time.monotonic() + args.seconds
        print(f"commanding {args.vyaw:+.2f} rad/s for {args.seconds:.1f} s ...")
        while time.monotonic() < end_cmd:
            tick("cmd", True)
        print("commands stopped -- no zero sent; the watchdog must stop it")
        end_all = time.monotonic() + args.settle
        while time.monotonic() < end_all:
            tick("post", False)

        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            w.writerows(rows)
        s = p.sport
        if s is not None:
            print(f"sport state after : mode={s.mode} gait_type={s.gait_type} "
                  f"error_code={s.error_code}")
        print(f"log: {out} ({len(rows)} rows)\n")
        report = analyse(rows)
        print(format_report(report))
        return 0 if passed(report) else 1
    finally:
        p.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
