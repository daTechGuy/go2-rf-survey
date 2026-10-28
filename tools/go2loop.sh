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
#                                      sport Move needs 'normal'/'mcf')
#   bash tools/go2loop.sh avoid get|on|off|hold|release
#                                      Unitree's built-in obstacle avoidance;
#                                      hold/release = API takes/returns control
#                                      from the remote (no motion)
#   bash tools/go2loop.sh joystick hold|release
#                                      remote sticks off / on (SwitchJoystick);
#                                      the gate for bridge backend sport_prep
#   bash tools/go2loop.sh viz-up       live view: foxglove_bridge + live map;
#                                      connect Foxglove to ws://192.168.123.18:8765
#                                      (+ front camera on /go2/camera/compressed)
#   bash tools/go2loop.sh camera-check READ-ONLY: one camera picture to
#                                      ~/surveys/camera_test.jpg
#   bash tools/go2loop.sh viz-reset    clear the live map and trail
#   bash tools/go2loop.sh viz-down     stop the live view
#   bash tools/go2loop.sh bridge-up    start go2_survey_bridge (sends BalanceStand
#                                      ~1 s after start: a posture change)
#   bash tools/go2loop.sh turn-test --i-confirm-breathing
#                                      Stage 1: 0.2 rad/s turn for 2 s, then the
#                                      watchdog must stop it (MOVES THE ROBOT)
#   bash tools/go2loop.sh bridge-log   last lines of the bridge's log
#   bash tools/go2loop.sh bridge-down  stop the bridge (sends StopMove on exit)
#
#   bash tools/go2loop.sh remote-keys  READ-ONLY: show which buttons the robot
#                                      sees as you press them on the remote
#   bash tools/go2loop.sh state-watch  READ-ONLY: mode/gait/speeds/foot steps the
#                                      robot reports while you drive with the sticks
#   bash tools/go2loop.sh remote-up    start/abort the loop from the remote
#                                      (default: hold SELECT+UP 1.5 s = start)
#   bash tools/go2loop.sh remote-log | remote-down
#   bash tools/go2loop.sh demo-log     follow a detached demo (demo.sh --detach)
#   bash tools/go2loop.sh demo-stop    end a detached demo safely
#   bash tools/go2loop.sh results      recent runs + the scp lines to fetch them
#   bash tools/go2loop.sh record-start [NAME]   record only (you drive on the
#                                      remote); survives unplugging
#   bash tools/go2loop.sh record-stop  close the recording + build the floorplan
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
# Show a log without WARN lines. Never fails: with pipefail, grep on a log that
# is still empty returns 1 -- which made a healthy bridge-up/up report failure.
show_log() { grep -v '^WARN' "$1" | tail -n "$2"; return 0; }
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
    if running; then show_log "$LOG" 5; else echo "node exited:"; tail -n 20 "$LOG"; exit 1; fi
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
    show_log "$LOG" "${2:-20}"
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
  joystick)
    IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" python3 /workspace/tools/go2_joystick.py "${2:-}" \
      2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2'
    ;;
  viz-up)
    docker rm -f go2viz >/dev/null 2>&1
    mkdir -p "${HOME}/surveys"
    : > "${HOME}/surveys/go2viz.log"; : > "${HOME}/surveys/go2viz_bridge.log"
    : > "${HOME}/surveys/go2viz_camera.log"
    DETACH=go2viz IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash /workspace/tools/viz_entry.sh \
      >/dev/null || { echo "failed to start viz container" >&2; exit 1; }
    sleep 6
    if docker ps --filter "name=^go2viz$" --format '{{.Names}}' | grep -q .; then
      show_log "${HOME}/surveys/go2viz.log" 3
      if grep -qi "error" "${HOME}/surveys/go2viz_bridge.log"; then
        echo "foxglove_bridge reported:"; tail -n 5 "${HOME}/surveys/go2viz_bridge.log"
      fi
      show_log "${HOME}/surveys/go2viz_camera.log" 2
      echo "Foxglove app -> Open connection -> Foxglove WebSocket -> ws://192.168.123.18:8765"
    else
      echo "viz exited:"; tail -n 20 "${HOME}/surveys/go2viz.log" "${HOME}/surveys/go2viz_bridge.log"; exit 1
    fi
    ;;
  camera-check)
    # read-only: one picture from the front camera to ~/surveys/camera_test.jpg
    IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c       "$SRC; python3 -u /workspace/tools/front_camera.py --once /surveys/camera_test.jpg"       2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2'
    [ -s "${HOME}/surveys/camera_test.jpg" ] && echo "on the laptop:  scp go2:surveys/camera_test.jpg ."
    ;;
  viz-reset)
    docker exec go2viz bash -c "source /opt/ros/humble/setup.bash; timeout 20 ros2 service call /go2_live_map/reset std_srvs/srv/Trigger 2>/dev/null | grep -o 'success=.*'" \
      || echo "viz not running"
    ;;
  viz-down)
    if docker ps --filter "name=^go2viz$" --format '{{.Names}}' | grep -q .; then
      docker stop go2viz >/dev/null && echo "live view stopped"; else echo "live view not running"; fi
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
    if bridge_running; then show_log "$BLOG" 6; else echo "bridge exited:"; tail -n 20 "$BLOG"; exit 1; fi
    ;;
  turn-test)
    shift
    bridge_running || { echo "bridge not running -- start it with: bash tools/go2loop.sh bridge-up" >&2; exit 1; }
    docker exec "$BRIDGE" bash -c "$SRC; python3 /workspace/tools/turn_test.py $*"
    ;;
  bridge-log)
    show_log "$BLOG" "${2:-20}"
    ;;
  bridge-down)
    if bridge_running; then docker stop "$BRIDGE" >/dev/null && echo "bridge stopped (StopMove sent on exit)"; else echo "bridge not running"; fi
    ;;
  state-watch)
    IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c \
      "$SRC; python3 -u /workspace/tools/state_watch.py ${*:2}" \
      2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2'
    ;;
  remote-keys)
    IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c \
      "$SRC; python3 -u /workspace/tools/remote_trigger.py --probe" \
      2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2'
    ;;
  remote-up)
    shift
    docker rm -f go2remote >/dev/null 2>&1
    mkdir -p "${HOME}/surveys"
    : > "${HOME}/surveys/go2remote.log"
    DETACH=go2remote IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c \
      "$SRC; python3 -u /workspace/tools/remote_trigger.py $* > /surveys/go2remote.log 2>&1" \
      >/dev/null || { echo "failed to start remote trigger" >&2; exit 1; }
    sleep 4
    if docker ps --filter "name=^go2remote$" --format '{{.Names}}' | grep -q .; then
      tail -n 3 "${HOME}/surveys/go2remote.log"
    else
      echo "remote trigger exited:"; tail -n 20 "${HOME}/surveys/go2remote.log"; exit 1
    fi
    ;;
  remote-log)
    tail -n "${2:-20}" "${HOME}/surveys/go2remote.log"
    ;;
  remote-down)
    if docker ps --filter "name=^go2remote$" --format '{{.Names}}' | grep -q .; then
      docker stop go2remote >/dev/null && echo "remote trigger stopped"; else echo "remote trigger not running"; fi
    ;;
  demo-log)
    f=$(ls -t "${HOME}"/surveys/*.demolog 2>/dev/null | head -1)
    [ -n "$f" ] || { echo "no demo logs in ~/surveys"; exit 1; }
    echo "== $f   (Ctrl-C stops following; the demo keeps running)"
    tail -n 40 -f "$f"
    ;;
  demo-stop)
    pidf="${HOME}/surveys/.demo.pid"
    pid=$(cat "$pidf" 2>/dev/null)
    if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
      echo "no detached demo running"; rm -f "$pidf"; exit 0
    fi
    kill -TERM "$pid" && echo "asked demo (pid $pid) to shut down safely..."
    for _ in $(seq 60); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
    f=$(ls -t "${HOME}"/surveys/*.demolog 2>/dev/null | head -1)
    [ -n "$f" ] && tail -n 8 "$f"
    ;;
  record-start)
    # record only -- for a map you drive by hand on the remote; commands nothing
    if docker ps --filter "name=^go2rec$" --format '{{.Names}}' | grep -q .; then
      echo "already recording ($(cat "${HOME}/surveys/.rec_current" 2>/dev/null)) -- record-stop first"; exit 1
    fi
    bag="${2:-manual}_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "${HOME}/surveys"
    docker rm -f go2rec >/dev/null 2>&1
    DETACH=go2rec IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c \
      "exec ros2 bag record -o /surveys/$bag /utlidar/cloud_deskewed /utlidar/robot_odom > /surveys/$bag.reclog 2>&1" \
      >/dev/null || { echo "could not start recording" >&2; exit 1; }
    echo "$bag" > "${HOME}/surveys/.rec_current"
    echo "recording -> ~/surveys/$bag  (keeps going if you unplug; finish with: record-stop)"
    ;;
  record-stop)
    bag=$(cat "${HOME}/surveys/.rec_current" 2>/dev/null)
    if docker ps --filter "name=^go2rec$" --format '{{.Names}}' | grep -q .; then
      docker kill -s INT go2rec >/dev/null 2>&1       # SIGINT: the bag closes cleanly
      timeout 30 docker wait go2rec >/dev/null 2>&1
      echo "recording closed: ~/surveys/$bag"
    else
      echo "not recording"
    fi
    rm -f "${HOME}/surveys/.rec_current"
    if [ -n "$bag" ] && [ -d "${HOME}/surveys/$bag" ]; then
      echo "building the floorplan..."
      IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" python3 /workspace/tools/bag_to_map.py "/surveys/$bag" \
        2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2|^\s+[0-9]+ scans read'
      echo "on the laptop:  scp go2:surveys/${bag}_map-topdown.png ."
    fi
    ;;
  results)
    n=0
    for d in $(ls -1dt "${HOME}"/surveys/*_20[0-9][0-9][0-9][0-9][0-9][0-9]_[0-9][0-9][0-9][0-9][0-9][0-9] 2>/dev/null); do
      [ -d "$d" ] || continue
      b=$(basename "$d")
      outcome=$(grep -o 'demo finished: .*' "$d.demolog" 2>/dev/null | tail -1)
      if [ -z "$outcome" ]; then
        if [ ! -f "$d.demolog" ]; then outcome="(recording only, no demo log)"
        elif kill -0 "$(cat "${HOME}/surveys/.demo.pid" 2>/dev/null)" 2>/dev/null; then outcome="(still running)"
        else outcome="(ended without finishing -- see its log)"; fi
      fi
      echo "$b   $outcome   $(du -sh "$d" | cut -f1)"
      [ -f "$d.demolog" ] && echo "    log:      scp go2:surveys/$b.demolog ."
      [ -f "${d}_map-topdown.png" ] && echo "    floorplan: scp go2:surveys/${b}_map-topdown.png ."
      echo "    whole run: scp -r go2:surveys/$b* ."
      n=$((n + 1)); [ "$n" -ge "${2:-5}" ] && break
    done
    [ "$n" = 0 ] && echo "no runs in ~/surveys yet"
    ;;
  *)
    awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"
    exit 2
    ;;
esac
