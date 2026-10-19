#!/usr/bin/env bash
# Container entrypoint: source every overlay, then check the things that
# silently produce an empty topic list if they are wrong.
set -e

source /opt/ros/humble/setup.bash
[ -f /opt/go2_ws/install/setup.bash ] && source /opt/go2_ws/install/setup.bash

# unitree_ros2 is built on the host and bind-mounted; it provides unitree_api,
# without which the bridge cannot start.
if [ -f "${UNITREE_ROS2_WS:-/unitree_ros2}/install/setup.bash" ]; then
  source "${UNITREE_ROS2_WS:-/unitree_ros2}/install/setup.bash"
else
  echo "WARN: unitree_ros2 not found at ${UNITREE_ROS2_WS:-/unitree_ros2}." >&2
  echo "      go2_survey_bridge will fail to import unitree_api." >&2
fi

# This repo's own packages, if they have been built.
[ -f "${GO2_SURVEY_WS:-/workspace}/install/setup.bash" ] \
  && source "${GO2_SURVEY_WS:-/workspace}/install/setup.bash"

# --- the check that saves an afternoon ---------------------------------------
# A wrong interface name in CYCLONEDDS_URI gives a clean build, no error, and
# zero topics. Name it loudly at startup instead.
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  echo "WARN: CYCLONEDDS_URI is unset. DDS will pick an interface on its own," >&2
  echo "      which on a multi-homed Jetson is usually the wrong one." >&2
  echo "      Set it to the interface facing the robot -- see docker/run.sh." >&2
else
  iface=$(echo "$CYCLONEDDS_URI" | grep -o 'name="[^"]*"' | head -1 | cut -d'"' -f2)
  if [ -n "$iface" ] && ! ip link show "$iface" >/dev/null 2>&1; then
    echo "ERROR: CYCLONEDDS_URI names interface '$iface', which does not exist" >&2
    echo "       in this container. Available:" >&2
    ip -brief link show | awk '{print "         " $1}' >&2
    echo "       With --network host these match the robot's own interfaces." >&2
  fi
fi

exec "$@"
