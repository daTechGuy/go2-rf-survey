#!/usr/bin/env bash
# Start the Humble container on the robot's Jetson.
#
#   bash docker/run.sh                 # interactive shell
#   bash docker/run.sh ros2 topic list # one command
#
# Every flag below exists for a reason; the comments say which.

set -euo pipefail

IMAGE="${IMAGE:-go2-survey:humble}"
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
UNITREE_ROS2_WS="${UNITREE_ROS2_WS:-$HOME/unitree_ros2}"

# The interface facing the robot's internal network. On the Jetson this is
# usually eth0; confirm with `ip -brief addr` and look for 192.168.123.x.
# A WRONG VALUE HERE IS THE #1 TIME SINK ON THIS ROBOT: the build succeeds,
# nothing errors, and `ros2 topic list` is simply empty.
IFACE="${IFACE:-eth0}"

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "ERROR: interface '$IFACE' not found. Available:" >&2
  ip -brief addr >&2
  echo >&2
  echo "Re-run as: IFACE=<name> bash docker/run.sh" >&2
  exit 1
fi

CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>\
<NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/>\
</Interfaces></General></Domain></CycloneDDS>"

ARGS=(
  --rm -it
  # DDS discovery needs multicast on the robot's L2 segment. Bridged
  # networking silently yields zero topics -- this is not optional.
  --network host
  --ipc host
  # USB hotplug for the RealSense. Without this the D435i disappears from
  # the container whenever it re-enumerates.
  -v /dev:/dev
  --device-cgroup-rule='c 81:* rmw'
  --device-cgroup-rule='c 189:* rmw'
  -e "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  # Bind-mounted, not baked in: editing a node should not mean rebuilding.
  -v "${REPO_DIR}:/workspace"
  -e GO2_SURVEY_WS=/workspace
  -w /workspace
)

if [ -d "$UNITREE_ROS2_WS" ]; then
  ARGS+=(-v "${UNITREE_ROS2_WS}:/unitree_ros2:ro" -e UNITREE_ROS2_WS=/unitree_ros2)
else
  echo "WARN: no unitree_ros2 at ${UNITREE_ROS2_WS}; the bridge will not start." >&2
  echo "      Set UNITREE_ROS2_WS=/path/to/unitree_ros2 if it lives elsewhere." >&2
fi

# Survey CSVs are the deliverable. Keep them outside the container.
mkdir -p "${HOME}/surveys"
ARGS+=(-v "${HOME}/surveys:/surveys")

# X11 passthrough for RViz, only if there is a display to pass.
if [ -n "${DISPLAY:-}" ]; then
  ARGS+=(-e "DISPLAY=${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix)
fi

echo "image     ${IMAGE}"
echo "interface ${IFACE}"
echo "repo      ${REPO_DIR} -> /workspace"
echo "surveys   ${HOME}/surveys -> /surveys"
echo

exec docker run "${ARGS[@]}" "$IMAGE" "${@:-bash}"
