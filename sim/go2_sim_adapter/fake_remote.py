#!/usr/bin/env python3
"""A pretend handheld remote for the sim: /wirelesscontroller at 50 Hz.

    python3 fake_remote.py serve                 # run in the background: streams
                                                 # whatever /tmp/go2_fake_remote says
    python3 fake_remote.py press SELECT+UP 2     # hold a combo for 2 s, then release
    python3 fake_remote.py stick 0.9 1           # left stick forward 0.9 for 1 s
    python3 fake_remote.py interactive           # keys: u SELECT+UP, d SELECT+DOWN,
                                                 # x L2+B (damp), a L2+A, w stick fwd, q quit

The real remote streams continuously (zeros when nothing is pressed); so
does this. press/stick only write the shared state file that `serve` reads.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

STATE = Path(os.environ.get("GO2_FAKE_REMOTE", "/tmp/go2_fake_remote"))
KEY = {"R1": 0, "L1": 1, "START": 2, "SELECT": 3, "R2": 4, "L2": 5,
       "A": 8, "B": 9, "X": 10, "Y": 11, "UP": 12, "RIGHT": 13, "DOWN": 14, "LEFT": 15}


def combo(text: str) -> int:
    return sum(1 << KEY[k.strip().upper()] for k in text.split("+"))


def write(keys=0, lx=0.0, ly=0.0, rx=0.0, ry=0.0) -> None:
    tmp = STATE.with_suffix(".tmp")
    tmp.write_text(f"{keys} {lx} {ly} {rx} {ry}")
    os.replace(tmp, STATE)


def read():
    try:
        k, lx, ly, rx, ry = STATE.read_text().split()
        return int(k), float(lx), float(ly), float(rx), float(ry)
    except (OSError, ValueError):
        return 0, 0.0, 0.0, 0.0, 0.0


def serve() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import WirelessController

    rclpy.init()
    node = Node("go2_fake_remote")
    pub = node.create_publisher(WirelessController, "/wirelesscontroller", qos_profile_sensor_data)
    write()

    def tick():
        k, lx, ly, rx, ry = read()
        m = WirelessController()
        m.keys, m.lx, m.ly, m.rx, m.ry = k, lx, ly, rx, ry
        pub.publish(m)

    node.create_timer(0.02, tick)
    print(f"fake remote streaming /wirelesscontroller from {STATE}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def interactive() -> None:
    print(__doc__.split("\n\n")[1] if "\n\n" in __doc__ else "", flush=True)
    actions = {"u": (combo("SELECT+UP"), 2.0), "d": (combo("SELECT+DOWN"), 0.5),
               "x": (combo("L2+B"), 0.5), "a": (combo("L2+A"), 0.5)}
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            c = sys.stdin.read(1).lower()
            if c == "q":
                break
            if c == "w":
                write(ly=0.9); time.sleep(1.0); write()
                print("stick forward 1 s", flush=True)
            elif c in actions:
                k, secs = actions[c]
                write(k); time.sleep(secs); write()
                print(f"pressed {c} for {secs} s", flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        write()


def main(argv) -> int:
    cmd = argv[1] if len(argv) > 1 else "interactive"
    if cmd == "serve":
        serve()
    elif cmd == "press":
        write(combo(argv[2])); time.sleep(float(argv[3]) if len(argv) > 3 else 2.0); write()
    elif cmd == "stick":
        write(ly=float(argv[2])); time.sleep(float(argv[3]) if len(argv) > 3 else 1.0); write()
    elif cmd == "interactive":
        interactive()
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
