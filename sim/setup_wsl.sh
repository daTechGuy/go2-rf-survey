#!/usr/bin/env bash
# One-time setup of the Go2 simulation on Ubuntu 22.04 (WSL2 on the laptop).
# Run as root:   sudo bash sim/setup_wsl.sh [linux-user]
#
# Installs ROS 2 Humble (base), Gazebo Classic 11 via gazebo_ros, the
# packages the community Go2 sim needs (CHAMP controller, Velodyne), and
# foxglove_bridge + CycloneDDS to match the robot's container. Then, as the
# normal user, clones the Go2 sim into ~/go2_sim_ws and builds it.
# Safe to re-run: apt and git steps skip what is already there.
set -euo pipefail

SIM_USER="${1:-${SUDO_USER:-}}"
[ -n "$SIM_USER" ] && [ "$SIM_USER" != root ] || { echo "usage: sudo bash $0 <linux-user>" >&2; exit 2; }
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 2; }
export DEBIAN_FRONTEND=noninteractive

echo "== apt sources"
apt-get update -qq
apt-get install -y -qq software-properties-common curl gnupg lsb-release git >/dev/null
add-apt-repository -y universe >/dev/null
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  > /etc/apt/sources.list.d/ros2.list
apt-get update -qq

echo "== ROS 2 Humble + Gazebo Classic + sim dependencies (several minutes)"
apt-get install -y -qq \
  ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-robot-localization \
  ros-humble-velodyne ros-humble-velodyne-gazebo-plugins ros-humble-velodyne-description \
  ros-humble-teleop-twist-keyboard ros-humble-foxglove-bridge \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
  python3-colcon-common-extensions python3-rosdep python3-numpy python3-scipy \
  >/dev/null
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init >/dev/null

echo "== Go2 sim source for $SIM_USER"
WS="$(getent passwd "$SIM_USER" | cut -d: -f6)/go2_sim_ws"
sudo -u "$SIM_USER" -H bash -s "$WS" <<'USER'
set -euo pipefail
mkdir -p "$1/src" && cd "$1/src"
# Go2 description + CHAMP config, ROS 2 Humble branch
[ -d unitree-go2-ros2 ] || git clone -q -b humble --recurse-submodules \
  https://github.com/anujjain-dev/unitree-go2-ros2.git
USER

echo "== its dependencies (rosdep)"
set +u; source /opt/ros/humble/setup.bash; set -u   # ROS setup trips over set -u
rosdep update -q >/dev/null 2>&1 || true
rosdep install --from-paths "$WS/src" --ignore-src -r -y 2>&1 \
  | grep -vE "^#|^$|already installed" | tail -15 || true

echo "== build (as $SIM_USER)"
sudo -u "$SIM_USER" -H bash -c "source /opt/ros/humble/setup.bash && cd '$WS' && \
  colcon build --symlink-install 2>&1 | tail -15"
echo "== done: source $WS/install/setup.bash"
