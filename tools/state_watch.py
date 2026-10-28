#!/usr/bin/env python3
"""Watch what the robot reports about its own gait. READ-ONLY: sends nothing.

    bash tools/go2loop.sh state-watch               # until Ctrl-C
    bash tools/go2loop.sh state-watch --seconds 30  # stops by itself

Why: on 2026-10-28 (firmware 'mcf') our turn commands got a twist on planted
feet, and /sportmodestate said mode=0 gait=0 the whole time. We don't know
what this firmware reports while the robot *really* walks. Run this, then
walk and turn the robot with the remote sticks: it prints a line whenever
mode/gait change, and once a second the speeds, the sticks and the steps
each foot took (gait_analysis: foot force + foot height).

Stand the robot still for the first 2 s -- that is the standing baseline the
steps are judged against. Ctrl-C prints a summary. Everything is logged at
50 Hz to /surveys/state_<time>.csv (~/surveys on the robot).
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gait_analysis import FOOT_FIELDS, foot_row, format_steps, steps_between  # noqa: E402

try:
    from unitree_go.msg import SportModeState, WirelessController
except ImportError as exc:  # pragma: no cover - environment
    raise SystemExit("needs unitree_go: run inside the go2-survey:humble container") from exc

FIELDS = ["t", "mode", "gait_type", "progress", "error_code", "body_height",
          "vx", "vy", "yaw_speed", "keys", "lx", "ly", "rx", "ry"] + FOOT_FIELDS
BASELINE_S = 2.0


class Watch(Node):
    def __init__(self, out: Path) -> None:
        super().__init__("go2_state_watch")
        self.sport = self.remote = None
        self.rows, self.t0 = [], time.monotonic()
        self.seen = Counter()               # (mode, gait) -> samples at 50 Hz
        self.last_mg = None
        self.fh = out.open("w", newline="")
        self.w = csv.DictWriter(self.fh, fieldnames=FIELDS)
        self.w.writeheader()
        self.create_subscription(SportModeState, "/sportmodestate", self._on_sport,
                                 qos_profile_sensor_data)
        self.create_subscription(WirelessController, "/wirelesscontroller",
                                 self._on_remote, qos_profile_sensor_data)
        self.create_timer(0.02, self._log)
        self.create_timer(1.0, self._print)
        print(f"state watch: READ-ONLY. Hold still {BASELINE_S:.0f} s, then drive with "
              f"the sticks. Ctrl-C to quit. Log: {out}", flush=True)

    def _on_sport(self, m):
        self.sport = m

    def _on_remote(self, m):
        self.remote = m

    def _log(self):
        s, r = self.sport, self.remote
        if s is None:
            return
        row = {"t": round(time.monotonic() - self.t0, 3), "mode": int(s.mode),
               "gait_type": int(s.gait_type), "progress": round(float(s.progress), 3),
               "error_code": int(s.error_code), "body_height": round(float(s.body_height), 3),
               "vx": round(float(s.velocity[0]), 3), "vy": round(float(s.velocity[1]), 3),
               "yaw_speed": round(float(s.yaw_speed), 3),
               # the remote goes quiet when nothing is pressed: last value, else zeros
               "keys": 0 if r is None else int(r.keys),
               "lx": 0.0 if r is None else round(float(r.lx), 2),
               "ly": 0.0 if r is None else round(float(r.ly), 2),
               "rx": 0.0 if r is None else round(float(r.rx), 2),
               "ry": 0.0 if r is None else round(float(r.ry), 2),
               **foot_row(s)}
        self.rows.append(row)
        self.w.writerow(row)
        mg = (row["mode"], row["gait_type"])
        self.seen[mg] += 1
        if mg != self.last_mg:
            was = "start" if self.last_mg is None else f"mode={self.last_mg[0]} gait={self.last_mg[1]}"
            print(f"t={row['t']:6.1f}  {was} -> mode={mg[0]} gait={mg[1]}", flush=True)
            self.last_mg = mg

    def _print(self):
        self.fh.flush()
        if not self.rows:
            print("   (no /sportmodestate yet)", flush=True)
            return
        r, now = self.rows[-1], self.rows[-1]["t"]
        base = [x for x in self.rows if x["t"] < BASELINE_S]
        line = (f"t={now:6.1f}  mode={r['mode']} gait={r['gait_type']} "
                f"vx={r['vx']:+.2f} vy={r['vy']:+.2f} yaw={r['yaw_speed']:+.2f}  "
                f"sticks ly={r['ly']:+.1f} rx={r['rx']:+.1f}")
        if now >= BASELINE_S:
            rep = steps_between(self.rows[-60:], now - 1.0, now, baseline_rows=base)
            line += "  | " + format_steps(rep).split(": ", 1)[1]
        print(line, flush=True)

    def summary(self):
        self.fh.flush()
        total = sum(self.seen.values()) or 1
        print("\nmode/gait seen (share of time):")
        for (m, g), n in self.seen.most_common():
            print(f"  mode={m} gait={g}: {100 * n / total:5.1f}%  ({n / 50:.1f} s)")
        print(f"log: {self.fh.name} ({len(self.rows)} rows)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop by itself after this long (0 = until Ctrl-C)")
    args = ap.parse_args()
    out = Path(f"/surveys/state_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Watch(out)
    try:
        end = time.monotonic() + args.seconds if args.seconds > 0 else None
        while end is None or time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.fh.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
