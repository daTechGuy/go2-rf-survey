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
#   bash tools/go2loop.sh preflight    start-of-session check (updates code;
#                                      read-only otherwise)
#   bash tools/go2loop.sh build        (re)build the image, stamped so preflight
#                                      can tell when it is out of date
#   bash tools/go2loop.sh mode         which motion mode owns the legs (read-only;
#                                      sport Move needs 'normal')
#   bash tools/go2loop.sh avoid get|on|off|hold|release
#                                      Unitree's built-in obstacle avoidance;
#                                      hold/release = API takes/returns control
#                                      from the remote (no motion)
#   bash tools/go2loop.sh bridge-up    start go2_survey_bridge (sends BalanceStand
#                                      ~1 s after start: a posture change)
#   bash tools/go2loop.sh turn-test --i-confirm-breathing
#                                      Stage 1: 0.2 rad/s turn for 2 s, then the
#                                      watchdog must stop it (MOVES THE ROBOT)
#   bash tools/go2loop.sh bridge-log   last lines of the bridge's log
#   bash tools/go2loop.sh bridge-down  stop the bridge (sends StopMove on exit)
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
BRIDGE=go2bridge
REPO="$(cd "$(dirname "$0")/.." && pwd)"
LOG="${HOME}/surveys/${NAME}.log"
BLOG="${HOME}/surveys/${BRIDGE}.log"
SRC='source /opt/ros/humble/setup.bash; source /opt/unitree_msgs/setup.bash'

running() { docker ps --filter "name=^${NAME}$" --format '{{.Names}}' | grep -q .; }
bridge_running() { docker ps --filter "name=^${BRIDGE}$" --format '{{.Names}}' | grep -q .; }
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
  preflight)
    shift
    bash "$REPO/tools/preflight.sh" "$@"
    ;;
  build)
    cd "$REPO" || exit 1
    sha=$(cat docker/Dockerfile docker/entrypoint.sh | sha256sum | cut -c1-16)
    docker build --label "go2.src_sha=$sha" -t go2-survey:humble docker/
    ;;
  mode)
    IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" python3 /workspace/tools/go2_mode.py \
      2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2'
    ;;
  avoid)
    IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" python3 /workspace/tools/go2_avoid.py "${2:-get}" \
      2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2'
    ;;
  bridge-up)
    shift
    docker rm -f "$BRIDGE" >/dev/null 2>&1
    mkdir -p "${HOME}/surveys"
    : > "$BLOG"
    DETACH="$BRIDGE" IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c \
      "PYTHONPATH=/workspace/go2_survey_bridge:\$PYTHONPATH python3 -u -m go2_survey_bridge.bridge_node --ros-args -p max_vx:=0.3 -p max_vyaw:=0.5 $* > /surveys/${BRIDGE}.log 2>&1" \
      >/dev/null || { echo "failed to start bridge container" >&2; exit 1; }
    sleep 4
    if bridge_running; then grep -v '^WARN' "$BLOG" | tail -n 6; else echo "bridge exited:"; tail -n 20 "$BLOG"; exit 1; fi
    ;;
  turn-test)
    shift
    bridge_running || { echo "bridge not running -- start it with: bash tools/go2loop.sh bridge-up" >&2; exit 1; }
    docker exec "$BRIDGE" bash -c "$SRC; python3 /workspace/tools/turn_test.py $*"
    ;;
  bridge-log)
    grep -v '^WARN' "$BLOG" | tail -n "${2:-20}"
    ;;
  bridge-down)
    if bridge_running; then docker stop "$BRIDGE" >/dev/null && echo "bridge stopped (StopMove sent on exit)"; else echo "bridge not running"; fi
    ;;
  *)
    sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
