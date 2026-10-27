#!/usr/bin/env bash
# Start / stop the simulated Go2 (WSL Ubuntu). It then looks like the real
# robot to our code: run tools/go2loop.sh, tools/demo.sh etc. as on the robot.
#
#   bash sim/run_sim.sh up [small_room|classroom|box] [--gui]
#   bash sim/run_sim.sh status        what's running, key topic rates
#   bash sim/run_sim.sh press SELECT+UP [secs]   the fake remote
#   bash sim/run_sim.sh stick 0.9 [secs]
#   bash sim/run_sim.sh down
#
# Logs: ~/surveys/sim_gazebo.log, sim_adapter.log, sim_remote.log
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SIM="$REPO/sim"
LOGS="$HOME/surveys"; mkdir -p "$LOGS"
PIDS="$LOGS/.sim_pids"
IFACE="${IFACE:-eth0}"

ros_env() {
  set +u
  source /opt/ros/humble/setup.bash
  source "$HOME/go2_sim_ws/install/setup.bash"
  source "$HOME/unitree_msgs_ws/install/setup.bash"
  set -u
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  # identical to docker/run.sh, so the container and the sim discover each other
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
}

case "${1:-}" in
  up)
    world="${2:-small_room}"; gui=false
    [ "${3:-}" = "--gui" ] || [ "${2:-}" = "--gui" ] && gui=true
    [ "$world" = "--gui" ] && world=small_room
    if [ -f "$PIDS" ]; then echo "sim already up (bash sim/run_sim.sh down first)"; exit 1; fi
    case "$world" in
      small_room) python3 "$SIM/worlds/make_worlds.py" >/dev/null; wfile="$SIM/worlds/small_room.world" ;;
      box)        python3 "$SIM/worlds/make_worlds.py" --box >/dev/null; wfile="$SIM/worlds/small_room.world" ;;
      classroom)  python3 "$SIM/worlds/make_worlds.py" ${AISLE:+--aisle "$AISLE"} >/dev/null; wfile="$SIM/worlds/classroom.world" ;;
      *) echo "world must be small_room, box or classroom" >&2; exit 2 ;;
    esac
    ros_env
    # CHAMP's own bringup, with its controller moved off /cmd_vel (see the launch file)
    src="$(ros2 pkg prefix champ_bringup)/share/champ_bringup/launch/bringup.launch.py"
    patched="$LOGS/.sim_bringup.launch.py"
    sed 's|("/cmd_vel/smooth", "/cmd_vel")|("/cmd_vel/smooth", "/sim/cmd_vel")|' "$src" > "$patched"
    grep -q '"/sim/cmd_vel"' "$patched" || { echo "could not patch $src (upstream changed?)" >&2; exit 1; }
    : > "$PIDS"
    setsid ros2 launch "$SIM/launch/go2_sim.launch.py" world:="$wfile" gui:="$gui" bringup:="$patched" \
      > "$LOGS/sim_gazebo.log" 2>&1 & echo $! >> "$PIDS"
    setsid python3 -u "$SIM/go2_sim_adapter/adapter.py" > "$LOGS/sim_adapter.log" 2>&1 & echo $! >> "$PIDS"
    setsid python3 -u "$SIM/go2_sim_adapter/fake_remote.py" serve > "$LOGS/sim_remote.log" 2>&1 & echo $! >> "$PIDS"
    echo "sim starting: world $world ($wfile), gui=$gui -- give Gazebo ~20 s"
    echo "then: bash sim/run_sim.sh status"
    ;;
  status)
    [ -f "$PIDS" ] || { echo "sim not up"; exit 1; }
    ros_env
    for t in /utlidar/robot_odom /utlidar/cloud_deskewed /sportmodestate /wirelesscontroller; do
      hz=$(timeout 6 ros2 topic hz "$t" --window 20 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
      printf '  %-26s %s\n' "$t" "${hz:+$hz Hz}${hz:-NOT PUBLISHING}"
    done
    tail -n 3 "$LOGS/sim_adapter.log"
    ;;
  press|stick)
    python3 "$SIM/go2_sim_adapter/fake_remote.py" "$@"
    ;;
  down)
    if [ -f "$PIDS" ]; then
      while read -r p; do kill -INT -- -"$p" 2>/dev/null || kill -INT "$p" 2>/dev/null; done < "$PIDS"
      sleep 3
      while read -r p; do kill -KILL -- -"$p" 2>/dev/null; done < "$PIDS"
      rm -f "$PIDS"
    fi
    pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null
    echo "sim down"
    ;;
  *)
    awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 2 ;;
esac
