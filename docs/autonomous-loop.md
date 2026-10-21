# Autonomous loop — staged rollout

`go2_survey_loop` walks the robot around a rectangle you specify, on its own,
stopping for anything in its way. This page is the order to bring it up in.
**Do not skip a stage.** Each one proves the thing the next one relies on.

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
`/utlidar/cloud_deskewed` — already in the `odom` frame), so it runs inside
the Humble container with no extra build. The bridge needs Unitree's message
types and runs on the host.

## Getting it onto the robot

```bash
# robot host shell (unitree@ubuntu)
cd ~/go2-rf-survey && git fetch origin && git checkout origin/main -- go2_survey_loop
```

## Stage 0 — dry run, robot does not move

Proves the sensors feed it, its own legs are not "obstacles", it finds the
floor, and it detects a real obstacle. Robot can be standing or lying down.

```bash
# inside the container (root@ubuntu:/workspace#)
PYTHONPATH=/workspace/go2_survey_loop python3 -m go2_survey_loop.loop_node \
    --ros-args -p length:=1.0 -p width:=1.0 > /tmp/loop.log 2>&1 &
sleep 3
timeout 3 ros2 topic echo /go2_loop/status
```

Expect `idle | odom 0.0xs old, lidar 0.0xs old, floor -0.0x, front 0, near 0`.

- `front`/`near` must be **0 in open space**. If they're not, the robot is
  seeing itself — tell Claude the numbers; the body box needs enlarging.
- Hold a box ~0.6 m in front of the robot's nose: `front` should jump well
  above 5. Move it beside the robot within half a metre: `near` should rise.

Then a dry mission:

```bash
ros2 service call /go2_loop/start std_srvs/srv/Trigger
timeout 5 ros2 topic echo /go2_loop/status     # "running | waypoint 1/4 | would send vx=0.30 ..."
ros2 service call /go2_loop/stop  std_srvs/srv/Trigger
kill %1
```

`would send` means it computed a command and published nothing. Nothing moves.

## Stage 1 — bridge only, robot does not walk

Proves `/cmd_vel` reaches the robot through the bridge, and that the bridge's
watchdog stops it. Follow the existing bridge procedure in the README
("Verify the watchdog before trusting this to a planner"), robot on a stand or
held, with a hand-published `/cmd_vel`.

*Pending: confirm Unitree's `unitree_api` messages exist on the host (see
below) — that decides how the bridge is started.*

## Stage 2 — live, 1 m × 1 m, open floor

Smallest possible real loop. Clear 2 m around it. You on the remote.

1. Remote: L2 + A, then a **single** START (robot "breathes").
2. Bridge running (stage 1).
3. Container:
   ```bash
   PYTHONPATH=/workspace/go2_survey_loop python3 -m go2_survey_loop.loop_node \
       --ros-args -p length:=1.0 -p width:=1.0 -p dry_run:=false > /tmp/loop.log 2>&1 &
   sleep 3
   ros2 service call /go2_loop/start std_srvs/srv/Trigger
   ```
4. Watch it: forward 1 m, turn left, 1 m, left, 1 m, left, home, stop.
5. Abort any time: `ros2 service call /go2_loop/stop std_srvs/srv/Trigger`,
   or **L2 + START** on the remote.

Then walk in front of it mid-leg: it must stop and stay stopped until
`ros2 service call /go2_loop/resume std_srvs/srv/Trigger`.

## Stage 3 — the classroom loop, recording the map

Same as stage 2 with the real size, and `ros2 bag record` running in a second
container shell (see `bringup-log` / `bag_to_map.py`). One run gives the
autonomous demo and the floorplan.

## Parameters

Typed as their defaults (`length:=4.0`, not `length:=4`).

| Param | Default | |
|---|---|---|
| `length`, `width` | 0.0 | **required**, metres; `length` is along the start heading |
| `turn` | `left` | which way the corners go |
| `laps` | 1 | |
| `dry_run` | **true** | `false` to actually publish |
| `max_vx` / `max_vyaw` | 0.30 / 0.50 | |
| `stop_distance` | 0.50 | m beyond the nose |
| `clearance_radius` | 0.55 | m from centre, all round |
| `auto_resume_s` | 0.0 | >0 resumes after an obstacle has been clear this long; 0 = only `~/resume` |
| `max_duration_s` | 300 | |
