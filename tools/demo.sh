#!/usr/bin/env bash
# One-command autonomous demo. Run on the robot host (unitree@ubuntu).
#
#   bash tools/demo.sh --length 1.0 --i-confirm-breathing     # Stage 2
#   bash tools/demo.sh --length 3.0 --i-confirm-breathing     # the classroom route
#   bash tools/demo.sh --length 3.0 --rehearse                # NOTHING MOVES
#   bash tools/demo.sh --explore --i-confirm-breathing        # no route: it picks
#                   its own way into open space it hasn't seen, then walks home
#                   (--legs N, default 8; --radius R m from the start, default 4)
#
# Options: --shape out_and_back|rectangle  --width W (rectangle)
#          --backend sport|sport_prep|avoid  --name NAME  --no-viz  --no-map
#
# Untethered (no laptop cable during the run):
#   --remote-start  don't start right away: wait (up to --wait S, default 600)
#                   for SELECT+UP held 1.5 s on the handheld remote. Same combo
#                   resumes after a pause. SELECT+DOWN, L2+B or pushing a
#                   stick aborts the route. Check the buttons first, read-only:
#                   bash tools/go2loop.sh remote-keys
#   --detach        checks + preflight run here, then the demo carries on in
#                   the background and survives unplugging. Log, recording and
#                   floorplan stay in ~/surveys. Later: go2loop.sh demo-log |
#                   demo-stop | results
#   sport_prep / avoid take the sticks / control from the remote while driving:
#   pass their zero-motion gate first (docs/autonomous-loop.md).
#
# ⚠ Without --rehearse this MOVES THE ROBOT. It refuses unless:
#   * --i-confirm-breathing (remote L2+A then a SINGLE START, body balancing)
#   * the stage gate passes: a Stage 1 turn-test PASS today on the same
#     backend, and for routes over 1 m a completed 1 m route today
#     (tools/stage_gate.py; override only with --skip-stage-gate)
#   * preflight says motion-ready
#
# Then: live view reset -> recording -> bridge -> loop -> start -> status
# until done/aborted -> safe shutdown -> floorplan PNG. Ctrl-C at any point
# runs the same safe shutdown: stop mission, bridge down (zero + hand back
# to the remote), loop down, close the recording cleanly.
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO" || exit 1
G="bash $REPO/tools/go2loop.sh"

LENGTH=""; WIDTH=""; SHAPE=out_and_back; BACKEND=sport; NAME=demo
EXPLORE=0; LEGS=8; RADIUS=4.0
BREATHING=0; REHEARSE=0; VIZ=1; MAP=1; SKIP_GATE=0
DETACH_RUN=0; REMOTE=0; WAIT=600
ARGS=("$@")
usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 2; }
while [ $# -gt 0 ]; do
  case "$1" in
    --detach) DETACH_RUN=1; shift ;;
    --remote-start) REMOTE=1; shift ;;
    --wait) WAIT="$2"; shift 2 ;;
    --length) LENGTH="$2"; shift 2 ;;
    --width) WIDTH="$2"; shift 2 ;;
    --shape) SHAPE="$2"; shift 2 ;;
    --explore) EXPLORE=1; shift ;;
    --legs) LEGS="$2"; shift 2 ;;
    --radius) RADIUS="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --i-confirm-breathing) BREATHING=1; shift ;;
    --rehearse) REHEARSE=1; shift ;;
    --no-viz) VIZ=0; shift ;;
    --no-map) MAP=0; shift ;;
    --skip-stage-gate) SKIP_GATE=1; shift ;;
    -h|--help) usage ;;
    *) echo "unknown option: $1" >&2; usage ;;
  esac
done
if [ "$EXPLORE" = 1 ]; then
  # the stage gate judges by distance: exploring reaches up to --radius
  SHAPE=explore; LENGTH="$RADIUS"
fi
[ -n "$LENGTH" ] || { echo "--length is required (or --explore)" >&2; usage; }
case "$BACKEND" in sport|sport_prep|avoid) ;; *) echo "--backend must be sport, sport_prep or avoid" >&2; exit 2 ;; esac

STAMP="${DEMO_STAMP:-$(date +%Y%m%d_%H%M%S)}"   # the detached child keeps the parent's
BAG="${NAME}_${STAMP}"
ts() { date +%H:%M:%S; }
PIDF="$HOME/surveys/.demo.pid"
CHILD="${DEMO_CHILD:-0}"
[ "$CHILD" = 1 ] && trap 'rm -f "$PIDF"' EXIT

if [ "$CHILD" = 0 ] && [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; then
  echo "REFUSED: a detached demo is already running (pid $(cat "$PIDF")) --"
  echo "         bash tools/go2loop.sh demo-log  or  demo-stop"; exit 2
fi

# ---- cheap refusals first: nothing has been started yet --------------------
if [ "$REHEARSE" = 0 ]; then
  if [ "$EXPLORE" = 1 ]; then
    echo "== LIVE run: the robot will explore on its own, up to $LEGS legs within ${RADIUS} m, then walk home (backend ${BACKEND})"
  else
    echo "== LIVE run: the robot will walk a ${LENGTH} m ${SHAPE} on its own (backend ${BACKEND})"
  fi
  if [ "$BREATHING" = 0 ]; then
    echo "REFUSED: confirm walking mode first -- remote L2+A then a SINGLE START, body"
    echo "         visibly balancing -- then add --i-confirm-breathing"; exit 2
  fi
  if [ "$SKIP_GATE" = 1 ]; then
    echo "WARNING: --skip-stage-gate -- the staged rollout is NOT being enforced"
  else
    python3 "$REPO/tools/stage_gate.py" check --length "$LENGTH" --backend "$BACKEND" \
      --dir "$HOME/surveys" || exit 2
  fi
else
  echo "== REHEARSAL: nothing will move (dry-run loop, no bridge)"
fi

# ---- preflight (the detached child relies on the parent's) -------------------
if [ "$CHILD" = 0 ]; then
  echo "== preflight"
  $G preflight --no-update
  rc=$?
  if [ "$REHEARSE" = 0 ] && [ "$rc" != 0 ]; then echo "REFUSED: not motion-ready (above)"; exit 2; fi
  if [ "$REHEARSE" = 1 ] && [ "$rc" -gt 1 ]; then echo "REFUSED: sensors not ready (above)"; exit 2; fi
fi

# ---- detach: the rest runs in its own session, so unplugging can't kill it --
if [ "$DETACH_RUN" = 1 ] && [ "$CHILD" = 0 ]; then
  LOGF="$HOME/surveys/$BAG.demolog"
  mkdir -p "$HOME/surveys"
  # setsid: no controlling terminal, so the SSH drop sends no SIGHUP. Run in the
  # background, bash ignores SIGINT -- demo-stop therefore sends SIGTERM.
  DEMO_CHILD=1 DEMO_STAMP="$STAMP" setsid nohup bash "$0" "${ARGS[@]}" \
    > "$LOGF" 2>&1 < /dev/null &
  echo $! > "$PIDF"
  echo "== detached (pid $!) -- log ~/surveys/$BAG.demolog"
  echo "   following until it is ready; Ctrl-C here only stops following"
  if [ "$REMOTE" = 1 ]; then ready='WAITING FOR THE REMOTE'; else ready='== [0-9:]* start'; fi
  for _ in $(seq 120); do
    grep -qE "$ready|demo finished|REFUSED|could not|refused" "$LOGF" 2>/dev/null && break
    kill -0 "$(cat "$PIDF")" 2>/dev/null || break
    sleep 1
  done
  cat "$LOGF"
  echo
  if grep -q 'WAITING FOR THE REMOTE' "$LOGF"; then
    echo ">> SAFE TO UNPLUG. Walk to the robot with the remote, then hold SELECT+UP 1.5 s."
  elif grep -qE '== [0-9:]* start' "$LOGF"; then
    echo ">> running -- unplug when clear of the robot."
  fi
  echo "   later, plugged in:  bash tools/go2loop.sh demo-log | demo-stop | results"
  exit 0
fi

# ---- from here on, always shut down safely ----------------------------------
STARTED_BRIDGE=0
cleanup() {
  trap - INT TERM
  echo "== $(ts) shutting down (safe order)"
  [ "$REMOTE" = 1 ] && $G remote-down >/dev/null 2>&1   # 0. nothing can restart it
  $G stop >/dev/null 2>&1                       # 1. mission: zero velocity
  if [ "$STARTED_BRIDGE" = 1 ]; then $G bridge-down; fi   # 2. StopMove + hand back
  $G down >/dev/null 2>&1                       # 3. loop node
  if docker ps --filter "name=^go2rec$" --format '{{.Names}}' | grep -q .; then
    docker kill -s INT go2rec >/dev/null 2>&1   # 4. SIGINT: ros2 bag closes cleanly
    timeout 30 docker wait go2rec >/dev/null 2>&1
    echo "   recording closed: ~/surveys/$BAG"
  fi
}
# one page per run (tools/run_report.py): after the "demo finished:" line,
# which it reads; best-effort, never what decides how the run ended
report() {
  if python3 "$REPO/tools/run_report.py" "$HOME/surveys/$BAG" >/dev/null 2>&1; then
    echo "   report:  scp go2:surveys/${BAG}_report.html ."
  fi
  return 0                            # a missing report never changes how a run ended
}
# every way out leaves a "demo finished:" line, which go2loop.sh results reads
bail() { echo "$1"; cleanup; echo "== $(ts) demo finished: ${2:-failed ($1)}"; report; exit 1; }
trap 'cleanup; echo "== $(ts) demo finished: stopped early (Ctrl-C / demo-stop)"; report; exit 130' INT TERM

if [ "$VIZ" = 1 ]; then
  echo "== live view"
  docker ps --filter "name=^go2viz$" --format '{{.Names}}' | grep -q . || $G viz-up
  $G viz-reset
fi

echo "== recording -> ~/surveys/$BAG"
docker rm -f go2rec >/dev/null 2>&1
DETACH=go2rec IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c \
  "exec ros2 bag record -o /surveys/$BAG /utlidar/cloud_deskewed /utlidar/robot_odom > /surveys/$BAG.reclog 2>&1" \
  >/dev/null || bail "could not start recording"
sleep 3
# a still from the front camera for the report (best-effort: no camera, no picture)
$G camera-snap "${BAG}_camera_start.jpg" >/dev/null 2>&1 || true

if [ "$REHEARSE" = 0 ]; then
  echo "== bridge (backend $BACKEND)"
  STARTED_BRIDGE=1                    # before bridge-up: a half-started bridge is still taken down
  $G bridge-up -p "backend:=$BACKEND" || bail "bridge did not start"
  DRY=false
else
  DRY=true
fi

echo "== loop"
if [ "$EXPLORE" = 1 ]; then
  # wider stops than a route's: exploring a classroom brushed a podium and a
  # desk with the route's 0.55 m ring / 0.5 m stop / 0.35 m lane (2026-10-31)
  LOOP_ARGS="-p shape:=explore -p explore_legs:=$LEGS -p explore_radius:=$RADIUS -p dry_run:=$DRY"
  LOOP_ARGS="$LOOP_ARGS -p clearance_radius:=0.70 -p stop_distance:=0.70 -p lane_half_width:=0.45"
  LOOP_ARGS="$LOOP_ARGS -p turn_clearance:=0.70"
else
  LOOP_ARGS="-p shape:=$SHAPE -p length:=$LENGTH -p dry_run:=$DRY"
  [ -n "$WIDTH" ] && LOOP_ARGS="$LOOP_ARGS -p width:=$WIDTH"
fi
$G up $LOOP_ARGS || bail "loop node did not start"

# what the remote trigger did (e.g. a start the loop refused: path blocked)
seen=0
show_remote() {
  [ "$REMOTE" = 1 ] || return 0
  local n; n=$(wc -l < "$HOME/surveys/go2remote.log" 2>/dev/null || echo 0)
  [ "$n" -gt "$seen" ] && tail -n +"$((seen + 1))" "$HOME/surveys/go2remote.log" | head -n "$((n - seen))" | sed 's/^/   /'
  seen=$n
}

if [ "$REMOTE" = 1 ]; then
  echo "== remote trigger"
  $G remote-up || bail "remote trigger did not start"
  seen=$(wc -l < "$HOME/surveys/go2remote.log")   # remote-up already showed these
  echo "== $(ts) WAITING FOR THE REMOTE: hold SELECT+UP 1.5 s to start (up to ${WAIT} s)"
  echo "   abort any time: SELECT+DOWN, L2+B, or push a stick"
  wait_end=$(( $(date +%s) + WAIT ))
  while :; do
    show_remote
    s=$($G status); state="${s%% |*}"
    case "$state" in running|paused|done|aborted) echo "   $(ts) $s"; break ;; esac
    if [ "$(date +%s)" -ge "$wait_end" ]; then
      bail "   no start from the remote in ${WAIT} s" "not started"
    fi
    sleep 1
  done
else
  echo "== $(ts) start"
  out=$($G start)
  echo "   $out"
  case "$out" in *success=True*) ;; *) bail "start refused -- see above" "not started" ;; esac
fi

# ---- watch ------------------------------------------------------------------
if [ "$REHEARSE" = 1 ]; then LIMIT=15; else LIMIT=360; fi
deadline=$(( $(date +%s) + LIMIT ))
prev=""; state=""; paused_hint=0
while [ "$(date +%s)" -lt "$deadline" ]; do
  show_remote
  s=$($G status)
  state="${s%% |*}"
  if [ "$s" != "$prev" ]; then echo "   $(ts) $s"; prev="$s"; fi
  case "$state" in
    done|aborted) break ;;
    paused)
      if [ "$paused_hint" = 0 ]; then
        if [ "$REMOTE" = 1 ]; then
          echo "   PAUSED -- clear the way, then hold SELECT+UP on the remote to resume"
          echo "            (or SELECT+DOWN to end the route)"
        else
          echo "   PAUSED -- clear the way, then in another shell: bash tools/go2loop.sh resume"
          echo "            or Ctrl-C here to end the demo safely"
        fi
        paused_hint=1
      fi ;;
    *) paused_hint=0 ;;
  esac
  sleep 1
done
[ "$(date +%s)" -ge "$deadline" ] && echo "   $(ts) time limit reached (${LIMIT} s)"
show_remote

cleanup
trap - INT TERM
$G camera-snap "${BAG}_camera_end.jpg" >/dev/null 2>&1 || true

# ---- record stage 2 / build the map -------------------------------------------
if [ "$REHEARSE" = 0 ] && [ "$state" = "done" ] && [ "$EXPLORE" = 0 ] && \
   python3 -c "import sys; sys.exit(0 if float('$LENGTH') <= 1.0 else 1)"; then
  python3 "$REPO/tools/stage_gate.py" record-stage2 --backend "$BACKEND" --dir "$HOME/surveys"
fi

if [ "$MAP" = 1 ]; then
  echo "== floorplan"
  IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" python3 /workspace/tools/bag_to_map.py "/surveys/$BAG" \
    2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2|^\s+[0-9]+ scans read'
  echo "   on the laptop:  scp go2:surveys/${BAG}_map-topdown.png ."
fi
if [ "$REHEARSE" = 1 ] && [ "$state" != aborted ]; then
  # a dry run never reaches a waypoint, so it ends on the 15 s limit, still "running"
  echo "== $(ts) demo finished: rehearsal OK (dry run, nothing moved; loop was ${state:-unknown})"
else
  echo "== $(ts) demo finished: ${state:-unknown}"
fi
report
