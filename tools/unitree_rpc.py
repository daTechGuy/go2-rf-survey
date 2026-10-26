"""Request/response calls to Go2 services over unitree_api, from rclpy.

Unitree's services (sport, obstacles_avoid, motion_switcher, ...) listen on
/api/<service>/request and answer on /api/<service>/response with the same
header.identity.id. This is the one place that pairs them up, so each tool
does not re-implement it.

Needs unitree_api (inside the go2-survey:humble container).
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node

from unitree_api.msg import Request, Response


class UnitreeRpc:
    def __init__(self, node: Node) -> None:
        self.node = node
        self._pubs: dict = {}

    def _pub(self, service: str):
        if service not in self._pubs:
            self._pubs[service] = self.node.create_publisher(
                Request, f"/api/{service}/request", 10)
        return self._pubs[service]

    def call(self, service: str, api_id: int, parameter: str = "", timeout: float = 5.0):
        """Returns (status_code, data) or None on no reply. ``data`` is the
        parsed JSON object when the reply carries JSON, else the raw string."""
        pub = self._pub(service)
        rid = time.time_ns() % (1 << 62)
        got = {}

        def on_response(msg: Response) -> None:
            if msg.header.identity.id == rid:
                got["r"] = msg

        sub = self.node.create_subscription(
            Response, f"/api/{service}/response", on_response, 10)
        try:
            t0 = time.monotonic()
            while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self.node, timeout_sec=0.1)       # discovery
            req = Request()
            req.header.identity.id = rid
            req.header.identity.api_id = api_id
            req.parameter = parameter
            pub.publish(req)
            t0 = time.monotonic()
            while "r" not in got and time.monotonic() - t0 < timeout:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(sub)
        r = got.get("r")
        if r is None:
            return None
        try:
            data = json.loads(r.data) if r.data else {}
        except ValueError:
            data = r.data
        return int(r.header.status.code), data


# motion_switcher -- ids from unitreerobotics/unitree_sdk2,
# include/unitree/robot/b2/motion_switcher/motion_switcher_api.hpp
MOTION_SWITCHER = "motion_switcher"
CHECK_MODE = 1001


def check_mode(rpc: UnitreeRpc):
    """Active motion mode as (name, form), or None if the service did not
    answer. Sport-mode Move is documented to work in mode 'normal'."""
    r = rpc.call(MOTION_SWITCHER, CHECK_MODE)
    if r is None:
        return None
    code, data = r
    if code != 0 or not isinstance(data, dict):
        return (f"error {code}", str(data))
    return str(data.get("name", "")), str(data.get("form", ""))
