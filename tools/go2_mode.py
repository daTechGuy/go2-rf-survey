#!/usr/bin/env python3
"""Report which motion mode owns the Go2's legs. Read-only.

    IFACE=eth0 bash docker/run.sh python3 /workspace/tools/go2_mode.py

Asks the robot's motion_switcher service (CheckMode, api 1001). Sport-mode
commands -- what go2_survey_bridge sends -- are documented to be honoured
only in mode "normal", which firmware 1.1.6+ calls "mcf". An empty name means
no motion service holds the robot.

Deliberately no select/release here: those hand control of the legs between
services and are not something to do casually.
"""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from unitree_rpc import UnitreeRpc, check_mode  # noqa: E402


def main() -> int:
    rclpy.init()
    node = Node("go2_mode_cli")
    try:
        mode = check_mode(UnitreeRpc(node))
        if mode is None:
            print("no reply from motion_switcher")
            return 1
        name, form = mode
        print(f"motion mode: name='{name}' form='{form}'")
        if name not in ("normal", "mcf"):
            print("note: sport-mode Move (go2_survey_bridge) is documented for mode "
                  "'normal' ('mcf' on firmware 1.1.6+)")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
