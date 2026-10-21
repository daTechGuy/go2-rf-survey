#!/usr/bin/env bash
# Control the autonomous-loop node on the robot with one short command.
# Run on the robot host (unitree@ubuntu), from anywhere:
#
#   bash tools/go2loop.sh up -p shape:=out_and_back -p length:=3.0
#                                   start the node -- DRY RUN unless you add
#                                   -p dry_run:=false (which can move the robot)
#   bash tools/go2loop.sh status    one status line (state, would-send, sensors)
#   bash tools/go2loop.sh start     begin the mission  (~/start)
#   bash tools/go2loop.sh stop      abort the mission  (~/stop)
#   bash tools/go2loop.sh resume    resume after a pause (~/resume)
#   bash tools/go2loop.sh log       last lines of the node's log
#   bash tools/go2loop.sh down      stop the node and remove its container
#
# Why a script: the node runs in a detached container, and every query needs
# the ROS environment sourced inside it. Two traps it avoids:
#  * PYTHONPATH must be PREPENDED -- replacing it hides ROS (no rclpy).
#  * The ros2 CLI's background daemon can die if a CLI call is killed, after
#    which `ros2 topic echo` fails silently. status restarts it and retries.
# The node's log goes to ~/surveys/go2loop.log on the host, so it survives the
# container being removed.
set -uo pipefail

NAME=go2loop
REPO="$(cd "$(dirname "$0")/.." && pwd)"
LOG="${HOME}/surveys/${NAME}.log"

running() { docker ps --filter "name=^${NAME}$" --format '{{.Names}}' | grep -q .; }
ros()     { docker exec "$NAME" bash -c "source /opt/ros/humble/setup.bash; $1"; }
need_up() { running || { echo "node not running -- start it with: bash tools/go2loop.sh up ..." >&2; exit 1; }; }

case "${1:-}" in
  up)
    shift
    docker rm -f "$NAME" >/dev/null 2>&1
    mkdir -p "${HOME}/surveys"
    : > "$LOG"
    DETACH="$NAME" IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c \
      "PYTHONPATH=/workspace/go2_survey_loop:\$PYTHONPATH python3 -u -m go2_survey_loop.loop_node --ros-args $* > /surveys/${NAME}.log 2>&1" \
      >/dev/null || { echo "failed to start container" >&2; exit 1; }
    sleep 6
    if running; then grep -v '^WARN' "$LOG" | tail -n 5; else echo "node exited:"; tail -n 20 "$LOG"; exit 1; fi
    ;;
  status)
    need_up
    out=$(ros "timeout 15 ros2 topic echo /go2_loop/status --once --field data 2>/dev/null | head -1")
    if [ -z "$out" ]; then          # dead CLI daemon: restart it once and retry
      ros "ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1"
      out=$(ros "timeout 15 ros2 topic echo /go2_loop/status --once --field data 2>/dev/null | head -1")
    fi
    echo "${out:-no status received -- check: bash tools/go2loop.sh log}"
    ;;
  start|stop|resume)
    need_up
    ros "timeout 20 ros2 service call /go2_loop/$1 std_srvs/srv/Trigger 2>/dev/null | grep -o 'success=.*' || echo 'no reply from /go2_loop/$1'"
    ;;
  log)
    grep -v '^WARN' "$LOG" | tail -n "${2:-20}"
    ;;
  down)
    if running; then docker stop "$NAME" >/dev/null && echo "node stopped"; else echo "node not running"; fi
    ;;
  *)
    sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
