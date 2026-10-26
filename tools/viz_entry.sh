#!/usr/bin/env bash
# Runs INSIDE the container (started by `bash tools/go2loop.sh viz-up`):
# foxglove_bridge on :8765 plus the live map. Read-only towards the robot.
# Logs land in ~/surveys on the host.
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 \
  > /surveys/go2viz_bridge.log 2>&1 &
exec python3 -u /workspace/tools/live_map.py > /surveys/go2viz.log 2>&1
