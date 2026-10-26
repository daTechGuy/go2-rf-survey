#!/usr/bin/env bash
# One-command autonomous demo. Run on the robot host (unitree@ubuntu).
#
#   bash tools/demo.sh --length 1.0 --i-confirm-breathing     # Stage 2
#   bash tools/demo.sh --length 3.0 --i-confirm-breathing     # the classroom route
#   bash tools/demo.sh --length 3.0 --rehearse                # NOTHING MOVES
#
# Options: --shape out_and_back|rectangle  --width W (rectangle)
#          --backend sport|avoid  --name NAME  --no-viz  --no-map
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
BREATHING=0; REHEARSE=0; VIZ=1; MAP=1; SKIP_GATE=0
usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 2; }
while [ $# -gt 0 ]; do
  case "$1" in
    --length) LENGTH="$2"; shift 2 ;;
    --width) WIDTH="$2"; shift 2 ;;
    --shape) SHAPE="$2"; shift 2 ;;
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
[ -n "$LENGTH" ] || { echo "--length is required" >&2; usage; }
case "$BACKEND" in sport|avoid) ;; *) echo "--backend must be sport or avoid" >&2; exit 2 ;; esac

STAMP=$(date +%Y%m%d_%H%M%S)
BAG="${NAME}_${STAMP}"
ts() { date +%H:%M:%S; }

# ---- cheap refusals first: nothing has been started yet --------------------
if [ "$REHEARSE" = 0 ]; then
  echo "== LIVE run: the robot will walk a ${LENGTH} m ${SHAPE} on its own (backend ${BACKEND})"
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

# ---- preflight --------------------------------------------------------------
echo "== preflight"
$G preflight --no-update
rc=$?
if [ "$REHEARSE" = 0 ] && [ "$rc" != 0 ]; then echo "REFUSED: not motion-ready (above)"; exit 2; fi
if [ "$REHEARSE" = 1 ] && [ "$rc" -gt 1 ]; then echo "REFUSED: sensors not ready (above)"; exit 2; fi

# ---- from here on, always shut down safely ----------------------------------
STARTED_BRIDGE=0
cleanup() {
  trap - INT TERM
  echo "== $(ts) shutting down (safe order)"
  $G stop >/dev/null 2>&1                       # 1. mission: zero velocity
  if [ "$STARTED_BRIDGE" = 1 ]; then $G bridge-down; fi   # 2. StopMove + hand back
  $G down >/dev/null 2>&1                       # 3. loop node
  if docker ps --filter "name=^go2rec$" --format '{{.Names}}' | grep -q .; then
    docker kill -s INT go2rec >/dev/null 2>&1   # 4. SIGINT: ros2 bag closes cleanly
    timeout 30 docker wait go2rec >/dev/null 2>&1
    echo "   recording closed: ~/surveys/$BAG"
  fi
}
trap 'cleanup; exit 130' INT TERM

if [ "$VIZ" = 1 ]; then
  echo "== live view"
  docker ps --filter "name=^go2viz$" --format '{{.Names}}' | grep -q . || $G viz-up
  $G viz-reset
fi

echo "== recording -> ~/surveys/$BAG"
docker rm -f go2rec >/dev/null 2>&1
DETACH=go2rec IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c \
  "exec ros2 bag record -o /surveys/$BAG /utlidar/cloud_deskewed /utlidar/robot_odom > /surveys/$BAG.reclog 2>&1" \
  >/dev/null || { echo "could not start recording"; cleanup; exit 1; }
sleep 3

if [ "$REHEARSE" = 0 ]; then
  echo "== bridge (backend $BACKEND)"
  $G bridge-up -p "backend:=$BACKEND" || { cleanup; exit 1; }
  STARTED_BRIDGE=1
  DRY=false
else
  DRY=true
fi

echo "== loop"
LOOP_ARGS="-p shape:=$SHAPE -p length:=$LENGTH -p dry_run:=$DRY"
[ -n "$WIDTH" ] && LOOP_ARGS="$LOOP_ARGS -p width:=$WIDTH"
$G up $LOOP_ARGS || { cleanup; exit 1; }

echo "== $(ts) start"
out=$($G start)
echo "   $out"
case "$out" in *success=True*) ;; *) echo "start refused -- see above"; cleanup; exit 1 ;; esac

# ---- watch ------------------------------------------------------------------
if [ "$REHEARSE" = 1 ]; then LIMIT=15; else LIMIT=360; fi
deadline=$(( $(date +%s) + LIMIT ))
prev=""; state=""; paused_hint=0
while [ "$(date +%s)" -lt "$deadline" ]; do
  s=$($G status)
  state="${s%% |*}"
  if [ "$s" != "$prev" ]; then echo "   $(ts) $s"; prev="$s"; fi
  case "$state" in
    done|aborted) break ;;
    paused)
      if [ "$paused_hint" = 0 ]; then
        echo "   PAUSED -- clear the way, then in another shell: bash tools/go2loop.sh resume"
        echo "            or Ctrl-C here to end the demo safely"
        paused_hint=1
      fi ;;
    *) paused_hint=0 ;;
  esac
  sleep 1
done
[ "$(date +%s)" -ge "$deadline" ] && echo "   $(ts) time limit reached (${LIMIT} s)"

cleanup
trap - INT TERM

# ---- record stage 2 / build the map -------------------------------------------
if [ "$REHEARSE" = 0 ] && [ "$state" = "done" ] && \
   python3 -c "import sys; sys.exit(0 if float('$LENGTH') <= 1.0 else 1)"; then
  python3 "$REPO/tools/stage_gate.py" record-stage2 --backend "$BACKEND" --dir "$HOME/surveys"
fi

if [ "$MAP" = 1 ]; then
  echo "== floorplan"
  IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" python3 /workspace/tools/bag_to_map.py "/surveys/$BAG" \
    2>&1 | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2|^\s+[0-9]+ scans read'
  echo "   on the laptop:  scp go2:surveys/${BAG}_map-topdown.png ."
fi
echo "== $(ts) demo finished: ${state:-unknown}"
