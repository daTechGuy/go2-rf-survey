"""Just enough of rclpy to run tools/remote_trigger.py on the fake robot.

spin() plays the robot: every 20 ms it publishes /wirelesscontroller from
$FAKE_W/remote_now ("keys lx ly rx ry", or "off" for a silent remote), every
0.2 s /go2_loop/status from fakeloop, and runs due timers.
"""

import os
import time

import fakeloop
from std_msgs.msg import String
from unitree_go.msg import WirelessController


def init(*_a, **_k):
    pass


def try_shutdown():
    pass


def _remote():
    try:
        with open(os.path.join(os.environ["FAKE_W"], "remote_now")) as f:
            parts = f.read().split()
    except OSError:
        parts = []
    if parts and parts[0] == "off":
        return None
    parts = (parts + ["0"] * 5)[:5]
    m = WirelessController()
    m.keys = int(parts[0], 0)
    m.lx, m.ly, m.rx, m.ry = (float(p) for p in parts[1:])
    return m


def spin(node):
    last_status = 0.0
    while True:
        now = time.time()
        msg = _remote()
        if msg is not None:
            for cb in node._subs.get("/wirelesscontroller", []):
                cb(msg)
        if now - last_status >= 0.2:
            last_status = now
            line = fakeloop.status_line()
            for cb in node._subs.get("/go2_loop/status", []):
                cb(String(data=line))
        for t in node._timers:
            if now >= t[0]:
                t[0] = now + t[1]
                t[2]()
        time.sleep(0.02)
