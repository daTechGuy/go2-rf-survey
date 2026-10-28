#!/usr/bin/env python3
"""Switch the handheld remote's sticks off / back on (sport SwitchJoystick).

Runs inside the go2-survey:humble container (needs unitree_api):

    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/go2_joystick.py hold
    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/go2_joystick.py release

hold    = SwitchJoystick {"data": false}: the robot ignores the sticks.
release = SwitchJoystick {"data": true}:  the sticks work again.

Sends NO motion. This is the zero-motion safety gate for the bridge's
``sport_prep`` backend, which switches the sticks off while driving: hold,
then check on the remote that L2+B still damps the robot (and, with
``go2loop.sh remote-keys``, that the buttons are still reported), then
release. Always release afterwards.

Sport api 1027 and its {"data": bool} parameter as used by
dancher00/go2-nav2-wifi (MIT), ws/scripts/robot_sport_bridge.py.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from unitree_rpc import UnitreeRpc  # noqa: E402

SWITCH_JOYSTICK = 1027


def main(argv) -> int:
    cmd = argv[1] if len(argv) > 1 else ""
    if cmd not in ("hold", "release"):
        print(__doc__)
        return 2
    rclpy.init()
    node = Node("go2_joystick_cli")
    try:
        sticks_on = cmd == "release"
        r = UnitreeRpc(node).call("sport", SWITCH_JOYSTICK, json.dumps({"data": sticks_on}))
        if r is None:
            print(f"{cmd}: sent, but the sport service did not reply -- state unknown. "
                  + ("Check the sticks yourself; run 'release' again if unsure."
                     if sticks_on else "Treat the sticks as OFF; run 'release' when done."))
            return 1
        code, data = r
        if code != 0:
            print(f"{cmd}: refused, error code {code} {data}")
            return 1
        print("sticks OFF -- the robot ignores them. Test L2+B, then run 'release'."
              if not sticks_on else "sticks ON -- handed back to the remote")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
