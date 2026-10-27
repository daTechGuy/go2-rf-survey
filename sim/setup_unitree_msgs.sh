#!/usr/bin/env bash
# Unitree's ROS 2 messages (unitree_api, unitree_go) for the sim, built the
# same way docker/Dockerfile builds them for the robot (same commit).
# Run as root:   sudo bash sim/setup_unitree_msgs.sh <linux-user>
set -euo pipefail
SIM_USER="${1:-${SUDO_USER:-}}"
[ -n "$SIM_USER" ] && [ "$(id -u)" = 0 ] || { echo "usage: sudo bash $0 <linux-user>" >&2; exit 2; }
REF=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
WS="$(getent passwd "$SIM_USER" | cut -d: -f6)/unitree_msgs_ws"

DEBIAN_FRONTEND=noninteractive apt-get install -y -qq ros-humble-rosidl-generator-dds-idl >/dev/null
sudo -u "$SIM_USER" -H bash -s "$WS" "$REF" <<'USER'
set -euo pipefail
WS="$1"; REF="$2"
if [ ! -d "$WS/src/unitree_api" ]; then
  rm -rf /tmp/unitree_ros2 && git init -q /tmp/unitree_ros2
  git -C /tmp/unitree_ros2 fetch -q --depth 1 https://github.com/unitreerobotics/unitree_ros2.git "$REF"
  git -C /tmp/unitree_ros2 checkout -q --detach FETCH_HEAD
  mkdir -p "$WS/src"
  cp -r /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api \
        /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go "$WS/src/"
  rm -rf /tmp/unitree_ros2
fi
set +u; source /opt/ros/humble/setup.bash; set -u   # ROS setup trips over set -u
cd "$WS" && colcon build --cmake-args -DBUILD_TESTING=OFF 2>&1 | tail -3
USER
echo "== done: source $WS/install/setup.bash"
