#!/usr/bin/env bash
# Run demo.sh / go2loop.sh / remote_trigger.py against a FAKE robot.
#
#   bash tools/fake_robot/run.sh            # all scenarios (~4 min)
#   bash tools/fake_robot/run.sh 3 10       # just scenarios 3 and 10
#
# Real: demo.sh, go2loop.sh, stage_gate.py, remote_trigger.py, remote_keys.py.
# Fake: docker (containers are files), docker/run.sh, preflight.sh, the loop
# node (lib/fakeloop.py), rclpy and the remote (lib/, $FAKE/remote_now).
# Runs in Git Bash on the laptop or on any Linux box; no robot, ROS or docker.
# Not covered: setsid's new session (Git Bash lacks it; stubbed), real ROS,
# real containers, the robot itself.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WORK="$(mktemp -d)"
SB="$WORK/repo"
mkdir -p "$SB/docker"
cp -r "$REPO/tools" "$SB/"
cp "$HERE/run_sh_fake.sh" "$SB/docker/run.sh"
cp "$HERE/preflight_fake.sh" "$SB/tools/preflight.sh"
chmod +x "$HERE"/bin/*

w() { if command -v cygpath >/dev/null; then cygpath -w "$1"; else echo "$1"; fi; }
export PATH="$HERE/bin:$PATH" REPO_SB="$SB" FAKE_LIB_W="$(w "$HERE/lib")"
REAL_HOME="$HOME"

PASS=0; FAILS=0; FAILED=()
SEL=" $* "

START=0x1008; ABORT=0x4008; HARDSTOP=0x0024

fresh() {
  pkill -f remote_trigger.py 2>/dev/null
  export FAKE="$WORK/state_$1"
  rm -rf "$FAKE"; mkdir -p "$FAKE/containers" "$FAKE/home/surveys"
  export FAKE_W; FAKE_W="$(w "$FAKE")"
  export HOME="$FAKE/home" FAKE_ROUTE_S=4 FAKE_PREFLIGHT_RC=0
  echo "0" > "$FAKE/remote_now"
}
check() {  # check "description" command...
  local d="$1"; shift
  if "$@" >/dev/null 2>&1; then echo "   ok    $d"; PASS=$((PASS + 1))
  else echo "   FAIL  $d"; FAILS=$((FAILS + 1)); FAILED+=("$CUR: $d"); fi
}
has()   { grep -qE -- "$2" "$1"; }
hasnt() { ! grep -qE -- "$2" "$1"; }
running() { [ -f "$FAKE/containers/$1" ]; }
stopped() { [ ! -f "$FAKE/containers/$1" ]; }
demo()  { bash "$SB/tools/demo.sh" "$@" > "$FAKE/parent.out" 2>&1; echo $? > "$FAKE/parent.rc"; }
rc_is() { [ "$(cat "$FAKE/parent.rc")" = "$1" ]; }
g()     { bash "$SB/tools/go2loop.sh" "$@"; }
press() { echo "$*" > "$FAKE/remote_now"; }
hold()  { press "$1"; /usr/bin/sleep "$2"; press 0; }
dlog()  { ls -t "$HOME"/surveys/*.demolog 2>/dev/null | head -1; }
demo_pid() { cat "$HOME/surveys/.demo.pid" 2>/dev/null; }
wait_demo() {  # until the detached demo exits
  local pid; pid=$(demo_pid); [ -n "$pid" ] || return 0
  for _ in $(seq $(( ${1:-60} * 5 ))); do kill -0 "$pid" 2>/dev/null || return 0; /usr/bin/sleep 0.2; done
  return 1
}
wait_log() {  # wait_log PATTERN [secs]
  for _ in $(seq $(( ${2:-20} * 5 ))); do
    f=$(dlog); [ -n "$f" ] && grep -qE -- "$1" "$f" && return 0; /usr/bin/sleep 0.2
  done; return 1
}
before() {  # before A B: event A logged before event B
  local a b; a=$(grep -n -- "$1" "$FAKE/events.log" | head -1 | cut -d: -f1)
  b=$(grep -n -- "$2" "$FAKE/events.log" | head -1 | cut -d: -f1)
  [ -n "$a" ] && [ -n "$b" ] && [ "$a" -lt "$b" ]
}
cleaned() { stopped go2loop && stopped go2remote && stopped go2rec && stopped go2bridge; }
stage1() { python -c "import json,time; print(json.dumps({'epoch': time.time(), 'backend': 'sport'}))" > "$HOME/surveys/.stage1_pass"; }
stage2() { python -c "import json,time; print(json.dumps({'epoch': time.time(), 'backend': 'sport'}))" > "$HOME/surveys/.stage2_pass"; }
results() { g results > "$FAKE/results.out"; cat "$FAKE/results.out"; }
dump() { echo "   --- parent ---"; sed 's/^/   | /' "$FAKE/parent.out"; f=$(dlog)
         [ -n "$f" ] && { echo "   --- demolog ---"; sed 's/^/   | /' "$f"; }
         echo "   --- events ---"; sed 's/^[0-9.]* /   | /' "$FAKE/events.log" 2>/dev/null; }

showif() { if [ "$FAILS" != "$1" ] || [ -n "${FAKE_SHOW:-}" ]; then dump; fi; }
scenario() { CUR="$1 $2"; [ "$SEL" = "  " ] || [[ "$SEL" == *" $1 "* ]]; }

# ---------------------------------------------------------------------------
if scenario 1 "rehearsal: detach, remote start, finish"; then
  echo "== $CUR"; fresh 1
  demo --length 1.0 --rehearse --remote-start --detach
  check "parent exits 0"                  rc_is 0
  check "parent says SAFE TO UNPLUG"      has "$FAKE/parent.out" 'SAFE TO UNPLUG'
  check "demo still running after parent" kill -0 "$(demo_pid)"
  hold $START 2.2
  check "demo finishes"                   wait_demo 40
  check "log: remote start -> start"      has "$(dlog)" 'remote start \(loop idle\) -> start'
  check "log: start success=True"         has "$(dlog)" 'start: success=True'
  results >/dev/null
  check "results: rehearsal OK"           has "$FAKE/results.out" 'rehearsal OK'
  check "results: floorplan scp line"     has "$FAKE/results.out" 'floorplan: scp go2:surveys/demo_.*_map-topdown.png'
  check "all containers down (viz stays)" cleaned
  check "viz left running"                running go2viz
  check "pid file removed"                test ! -f "$HOME/surveys/.demo.pid"
  check "order: remote down before stop"  before 'stop go2remote' 'service stop'
  check "order: stop before loop down"    before 'service stop' 'stop go2loop'
  check "order: loop down before rec INT" before 'stop go2loop' 'kill-INT go2rec'
  check "no bridge in a rehearsal"        hasnt "$FAKE/events.log" 'go2bridge'
  check "no unhandled fake calls"         test ! -s "$FAKE/unhandled"
  showif 0
fi

if scenario 2 "rehearsal: SELECT+DOWN aborts"; then
  echo "== $CUR"; fresh 2; f0=$FAILS
  demo --length 1.0 --rehearse --remote-start --detach
  hold $START 2.2; /usr/bin/sleep 1.5; hold $ABORT 0.5
  check "demo finishes"                   wait_demo 40
  check "log: abort -> stop"              has "$(dlog)" 'remote abort \(loop running\) -> stop'
  check "finished: aborted"               has "$(dlog)" 'demo finished: aborted'
  check "cleaned up"                      cleaned
  showif "$f0"
fi

if scenario 3 "rehearsal: a stick aborts"; then
  echo "== $CUR"; fresh 3; f0=$FAILS
  demo --length 1.0 --rehearse --remote-start --detach
  hold $START 2.2; /usr/bin/sleep 1.5; hold "0 0 0.9 0 0" 0.5
  check "demo finishes"                   wait_demo 40
  check "finished: aborted"               has "$(dlog)" 'demo finished: aborted'
  showif "$f0"
fi

if scenario 4 "rehearsal: L2+START aborts"; then
  echo "== $CUR"; fresh 4; f0=$FAILS
  demo --length 1.0 --rehearse --remote-start --detach
  hold $START 2.2; /usr/bin/sleep 1.5; hold $HARDSTOP 0.5
  check "demo finishes"                   wait_demo 40
  check "finished: aborted"               has "$(dlog)" 'demo finished: aborted'
  showif "$f0"
fi

if scenario 5 "short press does nothing, full hold starts"; then
  echo "== $CUR"; fresh 5; f0=$FAILS
  demo --length 1.0 --rehearse --remote-start --detach --wait 30
  hold $START 0.6; /usr/bin/sleep 2.5
  check "still waiting after a short press" has "$(dlog)" 'WAITING FOR THE REMOTE'
  check "no start logged yet"             hasnt "$(dlog)" 'remote start'
  check "state still idle"                hasnt "$FAKE/events.log" 'service start'
  hold $START 2.2
  check "demo finishes"                   wait_demo 40
  check "exactly one start"               test "$(grep -c 'remote start' "$(dlog)")" = 1
  check "rehearsal OK"                    has "$(dlog)" 'rehearsal OK'
  showif "$f0"
fi

if scenario 6 "no press: --wait runs out, shuts down"; then
  echo "== $CUR"; fresh 6; f0=$FAILS
  demo --length 1.0 --rehearse --remote-start --detach --wait 4
  check "demo finishes"                   wait_demo 30
  check "finished: not started"           has "$(dlog)" 'demo finished: not started'
  check "cleaned up"                      cleaned
  results >/dev/null
  check "results: not started"            has "$FAKE/results.out" 'not started'
  showif "$f0"
fi

if scenario 7 "silent remote: warning shown"; then
  echo "== $CUR"; fresh 7; f0=$FAILS
  press off
  demo --length 1.0 --rehearse --remote-start --detach --wait 14
  check "demo finishes"                   wait_demo 40
  check "log warns: no /wirelesscontroller" has "$(dlog)" 'no /wirelesscontroller messages'
  showif "$f0"
fi

if scenario 8 "start refused while blocked, works once clear"; then
  echo "== $CUR"; fresh 8; f0=$FAILS
  touch "$FAKE/blocked"
  demo --length 1.0 --rehearse --remote-start --detach --wait 40
  hold $START 2.2
  check "log shows the refusal"           wait_log 'success=False refused: path blocked' 10
  check "still waiting"                   kill -0 "$(demo_pid)"
  rm -f "$FAKE/blocked"
  hold $START 2.2
  check "demo finishes"                   wait_demo 40
  check "rehearsal OK"                    has "$(dlog)" 'rehearsal OK'
  showif "$f0"
fi

if scenario 9 "live refused without a Stage 1 pass"; then
  echo "== $CUR"; fresh 9; f0=$FAILS
  demo --length 1.0 --i-confirm-breathing --remote-start --detach
  check "parent exits 2"                  rc_is 2
  check "says Stage 1"                    has "$FAKE/parent.out" 'Stage 1'
  check "nothing detached"                test -z "$(dlog)"
  check "no containers started"           test ! -s "$FAKE/events.log"
  demo --length 1.0 --remote-start --detach
  check "refused without --i-confirm-breathing" has "$FAKE/parent.out" 'REFUSED: confirm walking mode'
  showif "$f0"
fi

if scenario 10 "live 1 m: start, walk, done, Stage 2 recorded"; then
  echo "== $CUR"; fresh 10; f0=$FAILS
  stage1
  demo --length 1.0 --i-confirm-breathing --name smallroom --remote-start --detach
  check "parent says SAFE TO UNPLUG"      has "$FAKE/parent.out" 'SAFE TO UNPLUG'
  check "bridge up before start"          running go2bridge
  check "loop started LIVE"               has "$HOME/surveys/go2loop.log" 'dry_run=false'
  hold $START 2.2
  check "demo finishes"                   wait_demo 40
  check "finished: done"                  has "$(dlog)" 'demo finished: done'
  check "Stage 2 marker written"          test -f "$HOME/surveys/.stage2_pass"
  check "cleaned up"                      cleaned
  check "order: remote down before stop"  before 'stop go2remote' 'service stop'
  check "order: stop before bridge down"  before 'service stop' 'stop go2bridge'
  check "order: bridge before loop down"  before 'stop go2bridge' 'stop go2loop'
  check "order: loop down before rec INT" before 'stop go2loop' 'kill-INT go2rec'
  check "floorplan built"                 has "$FAKE/events.log" 'floorplan smallroom_'
  results >/dev/null
  check "results: done"                   has "$FAKE/results.out" 'smallroom_.*demo finished: done'
  showif "$f0"
fi

if scenario 11 "live 3 m needs Stage 2; allowed with it"; then
  echo "== $CUR"; fresh 11; f0=$FAILS
  stage1
  demo --length 3.0 --i-confirm-breathing --remote-start --detach
  check "refused: Stage 2"                has "$FAKE/parent.out" 'Stage 2'
  check "parent exits 2"                  rc_is 2
  stage2
  demo --length 3.0 --i-confirm-breathing --name classroom --remote-start --detach
  check "allowed with both passes"        has "$FAKE/parent.out" 'SAFE TO UNPLUG'
  hold $START 2.2
  check "demo finishes"                   wait_demo 40
  check "finished: done"                  has "$(dlog)" 'demo finished: done'
  showif "$f0"
fi

if scenario 12 "live: pause, then the remote resumes"; then
  echo "== $CUR"; fresh 12; f0=$FAILS
  export FAKE_ROUTE_S=5
  stage1
  demo --length 1.0 --i-confirm-breathing --remote-start --detach
  hold $START 2.2; /usr/bin/sleep 1; touch "$FAKE/pause_now"
  check "pause hint names the remote"     wait_log 'PAUSED -- clear the way, then hold SELECT\+UP' 10
  hold $START 2.2
  check "log: start -> resume"            wait_log 'remote start \(loop paused\) -> resume' 10
  check "demo finishes"                   wait_demo 40
  check "finished: done"                  has "$(dlog)" 'demo finished: done'
  showif "$f0"
fi

if scenario 13 "demo-stop while waiting for the remote"; then
  echo "== $CUR"; fresh 13; f0=$FAILS
  demo --length 1.0 --rehearse --remote-start --detach
  g demo-stop > "$FAKE/stop.out" 2>&1
  check "demo gone"                       wait_demo 5
  check "finished: stopped early"         has "$(dlog)" 'demo finished: stopped early'
  check "cleaned up"                      cleaned
  check "pid file removed"                test ! -f "$HOME/surveys/.demo.pid"
  showif "$f0"
fi

if scenario 14 "demo-stop during a live walk"; then
  echo "== $CUR"; fresh 14; f0=$FAILS
  export FAKE_ROUTE_S=60
  stage1
  demo --length 1.0 --i-confirm-breathing --remote-start --detach
  hold $START 2.2; /usr/bin/sleep 1.5
  g demo-stop > "$FAKE/stop.out" 2>&1
  check "demo gone"                       wait_demo 5
  check "mission was stopped"             has "$FAKE/events.log" 'service stop'
  check "bridge taken down"               has "$FAKE/events.log" 'stop go2bridge'
  check "finished: stopped early"         has "$(dlog)" 'demo finished: stopped early'
  check "cleaned up"                      cleaned
  check "no Stage 2 marker"               test ! -f "$HOME/surveys/.stage2_pass"
  showif "$f0"
fi

if scenario 15 "a second demo is refused while one runs"; then
  echo "== $CUR"; fresh 15; f0=$FAILS
  demo --length 1.0 --rehearse --remote-start --detach
  cp "$FAKE/parent.out" "$FAKE/first.out"
  demo --length 1.0 --rehearse --remote-start --detach
  check "second refused"                  has "$FAKE/parent.out" 'already running'
  check "second exits 2"                  rc_is 2
  g demo-stop >/dev/null 2>&1; wait_demo 5
  showif "$f0"
fi

if scenario 16 "preflight refusals"; then
  echo "== $CUR"; fresh 16; f0=$FAILS
  stage1
  FAKE_PREFLIGHT_RC=1 demo --length 1.0 --i-confirm-breathing --remote-start --detach
  check "live refused when not motion-ready" has "$FAKE/parent.out" 'REFUSED: not motion-ready'
  FAKE_PREFLIGHT_RC=1 demo --length 1.0 --rehearse --remote-start --detach
  check "rehearsal allowed sensing-only"  has "$FAKE/parent.out" 'SAFE TO UNPLUG'
  g demo-stop >/dev/null 2>&1; wait_demo 5
  FAKE_PREFLIGHT_RC=2 demo --length 1.0 --rehearse --remote-start --detach
  check "rehearsal refused sensors down"  has "$FAKE/parent.out" 'REFUSED: sensors not ready'
  showif "$f0"
fi

if scenario 17 "a part fails to start: shut down, log says why"; then
  echo "== $CUR"; fresh 17; f0=$FAILS
  stage1; touch "$FAKE/fail_go2bridge"
  demo --length 1.0 --i-confirm-breathing --remote-start --detach
  check "no SAFE TO UNPLUG"               hasnt "$FAKE/parent.out" 'SAFE TO UNPLUG'
  check "demo gone"                       wait_demo 20
  check "finished: failed (bridge...)"    has "$(dlog)" 'demo finished: failed \(bridge did not start\)'
  check "cleaned up"                      cleaned
  showif "$f0"
fi

if scenario 18 "the old ways still work (no detach)"; then
  echo "== $CUR"; fresh 18; f0=$FAILS
  demo --length 1.0 --rehearse
  check "plain rehearsal exits 0"         rc_is 0
  check "plain rehearsal OK"              has "$FAKE/parent.out" 'rehearsal OK'
  check "cleaned up"                      cleaned
  fresh 18b
  ( demo --length 1.0 --rehearse --remote-start ) &
  for _ in $(seq 50); do grep -q 'WAITING FOR THE REMOTE' "$FAKE/parent.out" 2>/dev/null && break; /usr/bin/sleep 0.2; done
  hold $START 2.2; wait
  check "tethered remote start OK"        has "$FAKE/parent.out" 'rehearsal OK'
  showif "$f0"
fi

if scenario 19 "record-start / record-stop"; then
  echo "== $CUR"; fresh 19; f0=$FAILS
  g record-start classroom-manual > "$FAKE/r1.out" 2>&1
  check "recording"                       running go2rec
  g record-start again > "$FAKE/r2.out" 2>&1
  check "second start refused"            has "$FAKE/r2.out" 'already recording'
  g record-stop > "$FAKE/r3.out" 2>&1
  check "closed with SIGINT"              has "$FAKE/events.log" 'kill-INT go2rec'
  check "floorplan + scp line"            has "$FAKE/r3.out" 'scp go2:surveys/classroom-manual_.*_map-topdown.png'
  results >/dev/null
  check "results lists it"                has "$FAKE/results.out" 'classroom-manual_.*recording only'
  showif "$f0"
fi

if scenario 20 "backend sport_prep: own stage gate, bridge gets the backend"; then
  echo "== $CUR"; fresh 20; f0=$FAILS
  stage1    # a pass on plain sport says nothing about sport_prep
  demo --length 1.0 --i-confirm-breathing --backend sport_prep --remote-start --detach
  check "refused: Stage 1 was on sport"   has "$FAKE/parent.out" "backend 'sport', not 'sport_prep'"
  python -c "import json,time; print(json.dumps({'epoch': time.time(), 'backend': 'sport_prep'}))" \
    > "$HOME/surveys/.stage1_pass"
  demo --length 1.0 --i-confirm-breathing --backend sport_prep --name prep --remote-start --detach
  check "allowed with a sport_prep pass"  has "$FAKE/parent.out" 'SAFE TO UNPLUG'
  check "bridge told backend sport_prep"  has "$(dlog)" 'bridge \(backend sport_prep\)'
  hold $START 2.2
  check "demo finishes"                   wait_demo 40
  check "finished: done"                  has "$(dlog)" 'demo finished: done'
  check "Stage 2 recorded for sport_prep" has "$HOME/surveys/.stage2_pass" '"sport_prep"'
  showif "$f0"
fi

pkill -f remote_trigger.py 2>/dev/null
export HOME="$REAL_HOME"
rm -rf "$WORK"
echo
if [ "$FAILS" = 0 ]; then echo "fake robot: all $PASS checks passed"; exit 0; fi
echo "fake robot: $FAILS FAILED, $PASS passed"; printf '  - %s\n' "${FAILED[@]}"; exit 1
