#!/usr/bin/env python3
"""Query or switch the Go2's built-in obstacle avoidance.

Runs inside the go2-survey:humble container (needs unitree_api):

    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/go2_avoid.py get
    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/go2_avoid.py on
    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/go2_avoid.py off
    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/go2_avoid.py hold
    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/go2_avoid.py release

hold / release: UseRemoteCommandFromApi(true/false) (api 1004) -- the API
takes control from the handheld remote, WITHOUT sending any motion. This is
the zero-motion safety gate for the bridge's avoid backend: hold, then check
on the remote that L2+B still damps the robot, then release. Always
release afterwards; until then the remote's sticks may be ignored.

Talks to the robot's ``obstacles_avoid`` service over
/api/obstacles_avoid/{request,response}. API ids and JSON fields are from
Unitree's official SDK, unitreerobotics/unitree_sdk2,
include/unitree/robot/go2/obstacles_avoid/obstacles_avoid_api.hpp:
SwitchSet 1001 {"enable": bool}, SwitchGet 1002 -> {"enable": bool}.

Scope, stated plainly: this switches *Unitree's* avoidance, which governs the
remote and Unitree's own avoidance Move (api 1003). go2_survey_bridge drives
through sport-mode Move (/api/sport/request), which this switch is not known
to filter -- the autonomous loop's protection is its own LiDAR stop.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from unitree_rpc import UnitreeRpc  # noqa: E402

SERVICE = "obstacles_avoid"
SWITCH_SET = 1001
SWITCH_GET = 1002
USE_REMOTE_COMMAND_FROM_API = 1004


def describe(r) -> str:
    if r is None:
        return "no reply (is the obstacles_avoid service running on the robot?)"
    code, data = r
    if code != 0:
        return f"error code {code} {data}"
    if isinstance(data, dict) and "enable" in data:
        return "obstacle avoidance is ON" if data["enable"] else "obstacle avoidance is OFF"
    return f"ok {data}"


def main(argv) -> int:
    cmd = argv[1] if len(argv) > 1 else "get"
    if cmd not in ("get", "on", "off", "hold", "release"):
        print(__doc__)
        return 2
    rclpy.init()
    node = Node("go2_avoid_cli")
    rpc = UnitreeRpc(node)
    rpc.prepare(SERVICE)
    try:
        if cmd in ("hold", "release"):
            hold = cmd == "hold"
            r = rpc.call(SERVICE, USE_REMOTE_COMMAND_FROM_API,
                         json.dumps({"is_remote_commands_from_api": hold}))
            if r is None or r[0] != 0:
                print(f"{cmd}: {describe(r)}")
                return 1
            print("API now HOLDS control -- remote sticks may be ignored. "
                  "Test L2+B, then run 'release'." if hold
                  else "control handed back to the handheld remote")
            return 0
        if cmd != "get":
            r = rpc.call(SERVICE, SWITCH_SET, json.dumps({"enable": cmd == "on"}))
            if r is None or r[0] != 0:
                print(f"switch {cmd}: {describe(r)}")
                return 1
            print(f"switch {cmd}: accepted")
        r = rpc.call(SERVICE, SWITCH_GET)
        print(describe(r))
        return 0 if r is not None and r[0] == 0 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
