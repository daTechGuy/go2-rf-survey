#!/usr/bin/env python3
"""Start / resume / abort the autonomous loop from the Go2's handheld remote.

So a run needs no laptop: plug in, set it up, unplug, then hold the start
combo on the remote. Decisions live in remote_keys.py (tested); this file
only wires /wirelesscontroller to the loop's Trigger services.

    python3 remote_trigger.py [--start SELECT+UP] [--abort SELECT+DOWN] [--hold 1.5]
    python3 remote_trigger.py --probe      READ-ONLY: print buttons as you press
                                           them, plus the message rate

Commands nothing itself: it only calls /go2_loop/start|resume|stop, and the
loop node keeps every one of its own refusals (blocked path, stale sensors).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from unitree_go.msg import WirelessController

sys.path.insert(0, str(Path(__file__).resolve().parent))
from remote_keys import (DEFAULT_ABORT, DEFAULT_START, HARD_STOP,  # noqa: E402
                         RemoteTrigger, action_for, decode, state_of)

LOOP = "/go2_loop"


class TriggerNode(Node):
    def __init__(self, a) -> None:
        super().__init__("go2_remote_trigger")
        self.trig = RemoteTrigger(a.start, a.abort, a.hold, a.stick)
        self.state = None
        self.clients_ = {s: self.create_client(Trigger, f"{LOOP}/{s}")
                         for s in ("start", "resume", "stop")}
        self.create_subscription(String, f"{LOOP}/status", self._on_status, 10)
        self.create_subscription(WirelessController, "/wirelesscontroller",
                                 self._on_remote, qos_profile_sensor_data)
        self._msgs = 0
        self.create_timer(10.0, self._heartbeat)
        print(f"remote trigger up: hold {a.start} {a.hold:.1f} s = start/resume; "
              f"{a.abort}, {HARD_STOP} or a stick = abort", flush=True)

    def _on_status(self, msg: String) -> None:
        self.state = state_of(msg.data)

    def _on_remote(self, msg: WirelessController) -> None:
        self._msgs += 1
        t = self.get_clock().now().nanoseconds * 1e-9
        event = self.trig.feed(t, msg.keys, (msg.lx, msg.ly, msg.rx, msg.ry))
        if event is None:
            return
        service = action_for(event, self.state)
        print(f"{time.strftime('%H:%M:%S')} remote {event} (loop {self.state}) -> "
              f"{service or 'nothing to do'}", flush=True)
        if service:
            fut = self.clients_[service].call_async(Trigger.Request())
            fut.add_done_callback(lambda f, s=service: self._done(s, f))

    def _done(self, service, fut) -> None:
        try:
            r = fut.result()
            print(f"   {service}: success={r.success} {r.message}", flush=True)
        except Exception as exc:  # noqa: BLE001 -- report, never crash the trigger
            print(f"   {service}: call failed: {exc}", flush=True)

    def _heartbeat(self) -> None:
        if self._msgs == 0:
            print("WARNING: no /wirelesscontroller messages in 10 s -- remote off?",
                  flush=True)
        self._msgs = 0


class ProbeNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_remote_probe")
        self.create_subscription(WirelessController, "/wirelesscontroller",
                                 self._on_remote, qos_profile_sensor_data)
        self._last = None
        self._n = 0
        self._t0 = time.monotonic()
        self.create_timer(5.0, self._rate)
        print("probe: READ-ONLY. Press buttons one at a time; Ctrl-C to quit.", flush=True)

    def _on_remote(self, msg: WirelessController) -> None:
        self._n += 1
        sticks = tuple(round(v, 1) for v in (msg.lx, msg.ly, msg.rx, msg.ry))
        cur = (msg.keys, sticks)
        if cur != self._last:
            names = "+".join(sorted(decode(msg.keys))) or "-"
            print(f"keys=0x{msg.keys:04x} {names:<20} sticks lx,ly,rx,ry={sticks}",
                  flush=True)
            self._last = cur

    def _rate(self) -> None:
        dt = time.monotonic() - self._t0
        print(f"   ({self._n / dt:.0f} msg/s)", flush=True)
        self._n, self._t0 = 0, time.monotonic()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", default=DEFAULT_START)
    ap.add_argument("--abort", default=DEFAULT_ABORT)
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--stick", type=float, default=0.5)
    ap.add_argument("--probe", action="store_true")
    a = ap.parse_args()
    try:
        RemoteTrigger(a.start, a.abort, a.hold, a.stick)   # reject bad combos before ROS
    except ValueError as exc:
        print(f"REFUSED: {exc}", flush=True)
        return 2
    rclpy.init()
    node = ProbeNode() if a.probe else TriggerNode(a)
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
