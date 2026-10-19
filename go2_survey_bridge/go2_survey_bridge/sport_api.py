"""Go2 sport-mode API constants and request construction.

The bridge talks to the robot through ``unitree_ros2``'s ``/api/sport/request``
topic rather than linking ``unitree_sdk2`` directly. That choice buys a lot:
no C++ build against an external SDK, no ChannelFactory/network-interface
handling in our code, the same language as the rest of this repo, and -- most
usefully -- request construction becomes a pure function that can be tested
with no robot attached.

A request is ``unitree_api/msg/Request``::

    RequestHeader header      # .identity.id (int64), .identity.api_id (int64)
    string parameter          # JSON, API-specific
    uint8[] binary

Constants reproduced from unitree_sdk2's
``include/unitree/robot/go2/sport/sport_api.hpp``.
"""

from __future__ import annotations

import json

SPORT_REQUEST_TOPIC = "/api/sport/request"

# --- the ones this bridge uses ---------------------------------------------
DAMP = 1001           # limp. The safest terminal state.
BALANCE_STAND = 1002  # stand and hold balance; required before MOVE is honoured
STOP_MOVE = 1003      # zero velocity, keep standing
STAND_UP = 1004
STAND_DOWN = 1005
RECOVERY_STAND = 1006  # get up after a fall
EULER = 1007
MOVE = 1008
SPEED_LEVEL = 1015

# --- the rest, for completeness --------------------------------------------
SIT = 1009
RISE_SIT = 1010
HELLO = 1016
STRETCH = 1017
POSE = 1028
STATIC_WALK = 1061
TROT_RUN = 1062
ECONOMIC_GAIT = 1063
FREE_WALK = 2045
FREE_AVOID = 2048
CLASSIC_WALK = 2049
CROSS_STEP = 2051
SWITCH_AVOID_MODE = 2058

# Human-readable names, for log lines that would otherwise be bare integers.
API_NAMES = {
    DAMP: "Damp", BALANCE_STAND: "BalanceStand", STOP_MOVE: "StopMove",
    STAND_UP: "StandUp", STAND_DOWN: "StandDown",
    RECOVERY_STAND: "RecoveryStand", EULER: "Euler", MOVE: "Move",
    SPEED_LEVEL: "SpeedLevel",
}


def move_parameter(vx: float, vy: float, vyaw: float) -> str:
    """JSON parameter for a MOVE request.

    Axes are body-frame, right-handed, Z up: +x forward, +y left, +z yaw CCW.
    This matches ROS ``geometry_msgs/Twist`` exactly, so the bridge is a
    straight field copy with no sign flips -- which is worth asserting in a
    test, because a silent sign error here drives the robot sideways into
    things.
    """
    return json.dumps({"x": float(vx), "y": float(vy), "z": float(vyaw)})


def euler_parameter(roll: float, pitch: float, yaw: float) -> str:
    return json.dumps({"x": float(roll), "y": float(pitch), "z": float(yaw)})


def speed_level_parameter(level: int) -> str:
    """-1 slow, 0 normal, 1 fast. Surveying wants slow."""
    return json.dumps({"data": int(level)})


def api_name(api_id: int) -> str:
    return API_NAMES.get(api_id, f"api_id={api_id}")
