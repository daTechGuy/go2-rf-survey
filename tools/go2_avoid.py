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
import time

import rclpy
from rclpy.node import Node

from unitree_api.msg import Request, Response

SWITCH_SET = 1001
SWITCH_GET = 1002
USE_REMOTE_COMMAND_FROM_API = 1004
REQ_TOPIC = "/api/obstacles_avoid/request"
RES_TOPIC = "/api/obstacles_avoid/response"


def call(node: Node, pub, api_id: int, parameter: str = "", timeout: float = 5.0):
    """Send one request and wait for the reply carrying the same id."""
    rid = time.time_ns() % (1 << 62)
    got = {}

    def on_response(msg: Response) -> None:
        if msg.header.identity.id == rid:
            got["r"] = msg

    sub = node.create_subscription(Response, RES_TOPIC, on_response, 10)
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)          # wait for discovery

    req = Request()
    req.header.identity.id = rid
    req.header.identity.api_id = api_id
    req.parameter = parameter
    pub.publish(req)

    t0 = time.monotonic()
    while "r" not in got and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got.get("r")


def describe(r) -> str:
    if r is None:
        return "no reply (is the obstacles_avoid service running on the robot?)"
    code = r.header.status.code
    try:
        data = json.loads(r.data) if r.data else {}
    except ValueError:
        data = {"raw": r.data}
    if code != 0:
        return f"error code {code} {data}"
    if "enable" in data:
        return "obstacle avoidance is ON" if data["enable"] else "obstacle avoidance is OFF"
    return f"ok {data}"


def main(argv) -> int:
    cmd = argv[1] if len(argv) > 1 else "get"
    if cmd not in ("get", "on", "off", "hold", "release"):
        print(__doc__)
        return 2
    rclpy.init()
    node = Node("go2_avoid_cli")
    pub = node.create_publisher(Request, REQ_TOPIC, 10)
    try:
        if cmd in ("hold", "release"):
            hold = cmd == "hold"
            r = call(node, pub, USE_REMOTE_COMMAND_FROM_API,
                     json.dumps({"is_remote_commands_from_api": hold}))
            if r is None or r.header.status.code != 0:
                print(f"{cmd}: {describe(r)}")
                return 1
            print("API now HOLDS control -- remote sticks may be ignored. "
                  "Test L2+B, then run 'release'." if hold
                  else "control handed back to the handheld remote")
            return 0
        if cmd != "get":
            r = call(node, pub, SWITCH_SET, json.dumps({"enable": cmd == "on"}))
            if r is None or r.header.status.code != 0:
                print(f"switch {cmd}: {describe(r)}")
                return 1
            print(f"switch {cmd}: accepted")
        r = call(node, pub, SWITCH_GET)
        print(describe(r))
        return 0 if r is not None and r.header.status.code == 0 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
