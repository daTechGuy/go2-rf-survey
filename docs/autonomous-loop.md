# Autonomous loop — staged rollout

`go2_survey_loop` walks the robot along a route you specify, on its own,
stopping for anything in its way. Two route shapes:

- **`out_and_back`** — straight out `length` m, turn round in place, back to
  the start. For a room with one clear aisle. **The classroom route:**
  `shape:=out_and_back length:=3.0` (10 ft in from the door).
- **`rectangle`** — `length` ahead, then `width` to the side, back to start.
  Needs open floor on both axes.

This page is the order to bring it up in. **Do not skip a stage.** Each one
proves the thing the next one relies on.

## Will it fit? (tight aisles)

The robot stops if anything is within **0.55 m of its centre**, in any
direction. So the route needs **more than ~1.1 m of clear width** along its
whole length, and especially at the far end where it turns round. In a 1 m
aisle between desks it will refuse to start (`path blocked (near=...)`) —
that is the system working, not a fault: turning round in that gap would
brush the desks.

If the aisle is tight, choose a turn-round spot in open floor (the front of
the room, a gap by the board) and set `length` to reach it. Stage 0 measures
the real clearance with the robot standing still — check it before anything
moves. Shrinking `clearance_radius` is possible but cuts the margin while
turning; **don't go below 0.50**.

The numbers, from the Go2's measured footprint (see [related-work.md](related-work.md)):

| | Size | Furthest corner from centre (what a turn on the spot sweeps) |
|---|---|---|
| Standing robot, measured from its 3D model over 33 live poses | 68.5 × 36.3 cm | **0.40 m** |
| Nav2 envelope used by go2_ros2_sdk, BotBrain, go2-nav2-wifi | 81 × 40 cm | 0.49 m |
| …plus their 3 cm padding | 87 × 46 cm | 0.53 m |
| **Our clearance ring** | | **0.55 m** |

So 0.55 m clears the standing robot's sweep by 15 cm and the padded envelope
by 2 cm. Below 0.50 the ring sits inside the envelope's sweep. The body box
(what the robot ignores as itself) is x −0.40…+0.35, y ±0.20 m, covering the
measured robot (whose rear reaches 0.353 m; the box was 0.35 m until
2026-10-26) while staying inside the envelope. `go2_survey_loop/test/test_footprint.py`
holds these to the measurements. They're the standing pose, not every gait's
leg swing.

## How it is kept safe

| Layer | What it does | Where |
|---|---|---|
| **Dry-run default** | computes commands, publishes nothing, unless launched with `dry_run:=false` | loop node |
| **No default loop** | refuses to start without a size you chose | loop node |
| **Explicit start** | never moves on launch; only `~/start` begins a mission | loop node |
| **Start checks** | refuses unless odometry + LiDAR are fresh, the floor is found, and nothing is in the way | loop node |
| **Obstacle stop** | anything within 0.5 m ahead of the nose, or 0.55 m around the body → pause | loop node |
| **Blind stop** | odometry or LiDAR goes stale → pause | loop node |
| **Pauses latch** | a pause holds still until you call `~/resume` (which itself refuses if still blocked) | loop node |
| **Geofence** | further than loop diagonal + 1 m from the start → abort | loop node |
| **Time limit** | 300 s → abort | loop node |
| **Speed cap** | 0.3 m/s, 0.5 rad/s in the node; clamped again in the bridge | both |
| **Watchdog** | `/cmd_vel` silent for 0.3 s → StopMove | bridge |
| **E-stop** | latching; clearing it never moves the robot | bridge |
| **You** | remote in hand. **L2 + START = damping**, overrides everything | remote |

It is a *stop* system, not a planner: it never steers around anything. If
something is in the way it stops and waits for you.

The node uses the robot's own LiDAR stack (`/utlidar/robot_odom`,
`/utlidar/cloud_deskewed` — already in the `odom` frame). Both it and the
bridge run inside the Humble container: the image carries Unitree's
`unitree_api` / `unitree_go` messages, built from the official
`unitree_ros2` repo (see `docker/Dockerfile`).

## Getting it onto the robot

```bash
# robot host shell (unitree@ubuntu)
cd ~/go2-rf-survey && git fetch origin
git checkout origin/main -- go2_survey_loop go2_survey_bridge docker
docker build -t go2-survey:humble docker/        # adds the Unitree message layer
IFACE=eth0 bash docker/run.sh
```

The rebuild only adds the new layer at the end (everything before it is
cached). Inside the container, confirm the messages are there:

```bash
python3 -c "import unitree_api.msg; print('unitree_api ok')"
```

## Stage 0 — dry run, robot does not move

Proves the sensors feed it, its own legs are not "obstacles", it finds the
floor, and it detects a real obstacle. Robot can be standing or lying down.

Put the robot where the route will start — **in the doorway, facing down
the aisle** — then:

```bash
# inside the container (root@ubuntu:/workspace#)
# Keep ':$PYTHONPATH' -- replacing it instead of prepending hides ROS
# ("No module named 'rclpy'").
PYTHONPATH=/workspace/go2_survey_loop:$PYTHONPATH python3 -m go2_survey_loop.loop_node \
    --ros-args -p shape:=out_and_back -p length:=3.0 > /tmp/loop.log 2>&1 &
sleep 3
timeout 3 ros2 topic echo /go2_loop/status
```

Expect `idle | odom 0.0xs old, lidar 0.0xs old, floor -0.0x, front 0, near 0`.

- `front`/`near` must be **0 in open space**. If they're not, the robot is
  seeing itself — tell Claude the numbers; the body box needs enlarging.
- Hold a box ~0.6 m in front of the robot's nose: `front` should jump well
  above 5. Move it beside the robot within half a metre: `near` should rise.
- **In the doorway, `near` is the aisle check.** Non-zero there means desks or
  the door frame are within 0.55 m — see "Will it fit?" above. Carry the robot
  (or walk it on the remote) to the planned turn-round spot and read `near`
  there too: that is where clearance matters most.

Then a dry mission:

```bash
ros2 service call /go2_loop/start std_srvs/srv/Trigger
timeout 5 ros2 topic echo /go2_loop/status     # "running | waypoint 1/4 | would send vx=0.30 ..."
ros2 service call /go2_loop/stop  std_srvs/srv/Trigger
kill %1
```

`would send` means it computed a command and published nothing. Nothing moves.

## Stage 1 — bridge only, robot turns on the spot

Proves `/cmd_vel` reaches the robot through the bridge, that the robot
follows it **promptly and in proportion**, and that the bridge's **watchdog**
stops it when commands stop. The only motion is a slow turn in place of
roughly 20°, then a stop. Open floor, you on the remote.

All on the robot host (`unitree@ubuntu`):

1. **Remote: L2 + A, then a single START.** Watch for the body visibly
   balancing ("breathing"). Joint-locked (L2 + A alone) is *not* enough — see
   the first-run note below.
2. Check which motion mode owns the legs (read-only):
   ```bash
   bash tools/go2loop.sh mode          # want: name='mcf' (firmware 1.1.6+) or 'normal'
   ```
   Sport-mode `Move` — what the bridge sends — is documented to be honoured
   only in mode `normal`. The turn test refuses in any other mode unless you
   pass `--allow-mode <name>` (or `any`) deliberately.
3. Start the bridge. ⚠ About 1 s after it starts it sends **BalanceStand** (a
   posture command, no walking); that is expected.
   ```bash
   bash tools/go2loop.sh bridge-up
   ```
4. Run the instrumented turn — only once the robot is breathing:
   ```bash
   bash tools/go2loop.sh turn-test --i-confirm-breathing
   ```
   It refuses (and sends nothing) unless odometry and `/sportmodestate` are
   live, the bridge is listening, the robot isn't already turning, and speed
   and duration are within hard caps (0.3 rad/s, 3 s). It never sends a zero:
   stopping is the watchdog's job, which is what's under test.
5. `bash tools/go2loop.sh bridge-down`

It logs ~50 Hz to `~/surveys/turn_<time>.csv` — what the bridge sent, heading
from **both** the LiDAR odometry and the IMU, IMU yaw rate, sport mode/gait/
error code — and prints a report. **PASS** needs all of: response within
0.5 s, at least half the commanded rotation while commanded, no swing over 5°
after the stop, both heading sources within 8°, and still in the last 2 s.
**If it keeps turning after the command ends, L2 + START.**

The two heading sources are what make a failure diagnosable: if they agree,
the motion was real (lag, wrong mode); if they disagree, the LiDAR odometry
jumped and the robot didn't move.

### First attempt (2026-10-21) — FAIL, cause not yet known

Commanded +0.2 rad/s for 2 s (≈23°). Measured on LiDAR odometry: **3°** while
commanded, then a swing to **+26° about 1.5 s after the stop**, back to +8°,
settled by ~3 s. The bridge's watchdog fired on time (0.31 s) and sent
StopMove — the software side worked; the robot's motion did not match.

Known procedural miss: the robot had just been put in a **joint-locked stand
(L2 + A, no START)** for a clearance reading, and the test ran without
re-confirming walking mode. `/sportmodestate` read mode 0, gait 0,
error_code 100 (one unofficial source maps 100 to 灵动 / "agile" mode). Only
odometry heading was logged, so a real late rotation and an odometry jump
could not be told apart — which is why the test is now instrumented as
above. Do not proceed to Stage 2 until Stage 1 passes.

## Optional: drive through Unitree's obstacle avoidance (`backend:=avoid`)

By default the bridge sends **sport-mode `Move`** (api 1008). Unitree's
built-in obstacle avoidance — confirmed ON on this robot — is not known to
filter that, so the loop's protection is its own LiDAR stop.

`backend:=avoid` instead drives through Unitree's **obstacle-avoidance
`Move`** (api 1003 on `obstacles_avoid`), so the robot's own avoidance applies
*on top of* our LiDAR stop. Sequence from Unitree's official Python example:
`SwitchSet(true)` → `UseRemoteCommandFromApi(true)` → `Move` … → `Move(0)` →
`UseRemoteCommandFromApi(false)`.

**The catch:** `UseRemoteCommandFromApi(true)` lets the API **override the
handheld remote**. The bridge therefore claims control only when it has
motion to send, keeps it through pauses, and **hands it back to the remote**
on watchdog, e-stop, damp and shutdown. Unitree does not document whether
L2 + START still damps the robot while the API holds control.

### Gate — must pass before the avoid backend moves the robot (no motion)

1. Robot standing, open floor, you on the remote.
2. `bash tools/go2loop.sh avoid hold` — the API takes control. Nothing moves.
3. **Press L2 + START.** Pass: the robot damps (goes limp / lies down) as it
   normally would. Optionally push a stick gently first: with the API holding
   control it should *not* walk.
4. `bash tools/go2loop.sh avoid release` — always, pass or fail.

If L2 + START does **not** work while held, do not use `backend:=avoid`.

If it passes, Stage 1 can be repeated through the avoid path:
```bash
bash tools/go2loop.sh bridge-up -p backend:=avoid
bash tools/go2loop.sh turn-test --i-confirm-breathing
bash tools/go2loop.sh bridge-down          # zeroes and hands back to the remote
```

## Optional: walk prep (`backend:=sport_prep`)

**Try this if Stage 1 fails again on the default `sport` backend** with the
robot confirmed breathing and in mode `normal`.

`sport_prep` drives with the same sport Move, but first sends the walk
preparation used by [dancher00/go2-nav2-wifi](https://github.com/dancher00/go2-nav2-wifi)
(MIT): **`SwitchJoystick(false)` + `FreeWalk`**. Their note says this is
*"required after RC use"*, and their troubleshooting says *"If hz>0 but robot
jerks: … (FreeWalk + SwitchJoystick)"*. That is our first Stage 1
result: commands arriving, motion wrong. See [related-work.md](related-work.md).

**The catch:** `SwitchJoystick(false)` makes the robot **ignore the remote's
sticks**. The bridge sends it ~1.5 s after starting, and hands the sticks back
(`SwitchJoystick(true)`) on watchdog, e-stop, damp and shutdown. Whether
**L2 + START** still damps while the sticks are off isn't documented.

### Gate — must pass before sport_prep moves the robot (no motion)

1. Robot standing, open floor, you on the remote.
2. `bash tools/go2loop.sh joystick hold`. The sticks are now off. Nothing moves.
3. Push a stick gently: the robot should **not** walk.
4. In a second shell, `bash tools/go2loop.sh remote-keys`: buttons and sticks
   must **still be reported** (the remote start/abort depends on it). Ctrl-C.
5. **Press L2 + START.** Pass: the robot damps as it normally would.
6. `bash tools/go2loop.sh joystick release`: always, pass or fail. Check
   that the sticks walk it again.

If L2 + START doesn't work while held, **don't use `sport_prep`**.

If it passes, Stage 1 on this path (the PASS is recorded as `sport_prep`,
separate from `sport`):
```bash
bash tools/go2loop.sh bridge-up -p backend:=sport_prep
bash tools/go2loop.sh turn-test --i-confirm-breathing
bash tools/go2loop.sh bridge-down          # StopMove + sticks back on
```
Later stages then use `bash tools/demo.sh ... --backend sport_prep`.

## Stage 2 — live, 1 m out-and-back, open floor

Smallest possible real route. Clear ~1.5 m around it. You on the remote.

`tools/demo.sh` does the whole run with one command, and **enforces the
staged rollout**: it refuses a live run without a Stage 1 turn-test PASS
from today on the same backend, and refuses anything over 1 m without a
completed 1 m route from today (`tools/stage_gate.py`).

**First, rehearse** — same command, nothing moves (dry-run loop, no bridge),
proves the whole chain incl. recording and the floorplan:
```bash
bash tools/demo.sh --length 1.0 --rehearse
```

**Then live** — remote: L2 + A, then a **single** START (robot breathes):
```bash
bash tools/demo.sh --length 1.0 --i-confirm-breathing
```
It runs preflight, resets the live view, starts recording, the bridge and
the loop, starts the mission and prints each state change. Watch it: forward
1 m, turn round in place (one direction, no back-and-forth), walk back, stop.
A completed 1 m run is recorded as the Stage 2 pass.

Abort any time: **Ctrl-C** in that shell (safe shutdown: stop mission → bridge
down, zero + hand back → loop down → close recording), `bash tools/go2loop.sh
stop` from another shell, or **L2 + START** on the remote.

Then walk in front of it mid-leg: it must pause and stay paused until
`bash tools/go2loop.sh resume` (from another shell).

## Stage 3 — the classroom route, recording the map

```bash
bash tools/demo.sh --length 3.0 --i-confirm-breathing --name classroom
```
3 m (10 ft), starting just inside the doorway facing down the aisle. The
live view shows the room appear on the laptop as it walks; at the end it
builds the floorplan and prints the `scp` line to fetch it. Add
`--backend avoid` only after the avoid-backend gate above has passed.

## Untethered — no cable during the run

Everything that keeps the robot safe runs on the robot: the obstacle stop,
geofence, time limit, the bridge watchdog. The handheld remote is radio, so
L2+START works anywhere. The cable is only needed to *set up* a run and for
the live view. So: set up plugged in, unplug, start from the remote, and fetch
the results afterwards.

**1. Check the remote's buttons (read-only, robot lying down).** The start/abort
combos are read from `/wirelesscontroller`; the bit layout is unitree_sdk2's
but unverified on this remote. Also watch that the robot itself does nothing
on SELECT+UP / SELECT+DOWN — if the firmware uses either, pick other combos
(`remote_trigger.py --start/--abort`).
```bash
bash tools/go2loop.sh remote-keys    # press SELECT+UP, SELECT+DOWN, L2+START; note msg/s
```
The rate should be steady (the hold timer needs a continuous stream).

**2. Rehearse untethered, nothing moves:**
```bash
bash tools/demo.sh --length 1.0 --rehearse --remote-start --detach
```
Unplug, hold SELECT+UP 1.5 s, watch nothing happen, plug back in:
`bash tools/go2loop.sh results` should list the run as `rehearsal OK`.

**3. Live** (after Stages 1–2 as above, sport backend):
```bash
bash tools/demo.sh --length 3.0 --i-confirm-breathing --name classroom --remote-start --detach
```
When it prints `SAFE TO UNPLUG`, unplug, and hold SELECT+UP to start.
- **Abort:** SELECT+DOWN, L2+START (also damps), or *any stick* pushed past
  half way — so keep thumbs off the sticks unless you mean to take over.
- **Paused** (something in the way): clear it, hold SELECT+UP to resume.
- Not started within 10 min (`--wait`): it shuts itself down.

**Afterwards, plugged in:**
```bash
bash tools/go2loop.sh results          # outcome + scp lines
bash tools/go2loop.sh demo-log         # the run's log
bash tools/go2loop.sh demo-stop        # end a detached demo that is still going
```
On the laptop: `scp go2:surveys/classroom_<stamp>_map-topdown.png .`

## Parameters

Typed as their defaults (`length:=4.0`, not `length:=4`).

| Param | Default | |
|---|---|---|
| `shape` | `rectangle` | or `out_and_back` |
| `length` | 0.0 | **required**, metres, along the start heading |
| `width` | 0.0 | rectangle only, metres to the side |
| `turn` | `left` | rectangle only: which way the corners go |
| `laps` | 1 | |
| `dry_run` | **true** | `false` to actually publish |
| `max_vx` / `max_vyaw` | 0.30 / 0.50 | |
| `stop_distance` | 0.50 | m beyond the nose |
| `clearance_radius` | 0.55 | m from centre, all round |
| `lookahead` | 0.6 | m along the route line the robot steers at; pulls it back to the centre after a turn. 0 = aim at each corner (the old behaviour) |
| `auto_resume_s` | 0.0 | >0 resumes after an obstacle has been clear this long; 0 = only `~/resume` |
| `max_duration_s` | 300 | |
