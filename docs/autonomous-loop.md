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
turning (the body is ~0.7 m long); don't go below 0.45 without a reason.

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
   bash tools/go2loop.sh mode          # want: name='normal'
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

## Stage 2 — live, 1 m out-and-back, open floor

Smallest possible real route. Clear ~1.5 m around it. You on the remote.

1. Remote: L2 + A, then a **single** START (robot "breathes").
2. Bridge running (stage 1).
3. Container:
   ```bash
   PYTHONPATH=/workspace/go2_survey_loop:$PYTHONPATH python3 -m go2_survey_loop.loop_node \
       --ros-args -p shape:=out_and_back -p length:=1.0 -p dry_run:=false \
       > /tmp/loop.log 2>&1 &
   sleep 3
   ros2 service call /go2_loop/start std_srvs/srv/Trigger
   ```
4. Watch it: forward 1 m, turn round in place (one direction, no back-and-forth),
   walk back, stop.
5. Abort any time: `ros2 service call /go2_loop/stop std_srvs/srv/Trigger`,
   or **L2 + START** on the remote.

Then walk in front of it mid-leg: it must stop and stay stopped until
`ros2 service call /go2_loop/resume std_srvs/srv/Trigger`.

## Stage 3 — the classroom route, recording the map

Same as stage 2 with `length:=3.0` (10 ft), starting in the doorway facing
down the aisle, and `ros2 bag record` running in a second container shell
(see `bag_to_map.py`). One run gives the autonomous demo and the floorplan.

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
| `auto_resume_s` | 0.0 | >0 resumes after an obstacle has been clear this long; 0 = only `~/resume` |
| `max_duration_s` | 300 | |
