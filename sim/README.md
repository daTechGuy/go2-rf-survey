# Go2 simulation (Gazebo, on the laptop)

A simulated Go2 that our real code can't tell from the robot: the same
scripts (`tools/go2loop.sh`, `tools/demo.sh`), the same Docker image, the
same topics and Unitree services. Use it to try a change before a robot
session, and to rehearse a route in a model of the room.

```
Gazebo Classic + CHAMP Go2 (anujjain-dev/unitree-go2-ros2, humble)
   │  /sim/cmd_vel  ▲            │ Velodyne, ground-truth pose
   │                │            ▼
   └─ go2_sim_adapter ── /api/sport|obstacles_avoid|motion_switcher, /utlidar/robot_odom,
        (robot_model.py)  /utlidar/cloud_deskewed, /sportmodestate    ◄── fake_remote.py
                                   ▲                                       /wirelesscontroller
        our container (go2-survey:humble): bridge, loop, turn test, demo, live view, bag_to_map
```

## What it is good for, and what not

| Tests for real | Does **not** test |
|---|---|
| The whole chain: preflight, turn test, bridge + watchdog, loop, obstacle stop, pauses, remote start/abort, `demo.sh --detach`, recording, floorplan, live view | Unitree's firmware: walking mode, `SwitchJoystick`, whether L2+B works while the API holds control, the camera service |
| Real ROS 2 / CycloneDDS between our container and "the robot" | Unitree's gait: CHAMP walks and turns differently (it drifts ~10 cm sideways turning in place) |
| Whether a route fits a room (desks, aisles), before the robot tries it | The robot's LiDAR exactly: the sim's is a Velodyne patched to look down (−50…+15°) |

The adapter deliberately makes the sim robot **hold a Move until StopMove**,
so only *our* watchdog stops it (the thing Stage 1 proves).

## Setup (once, WSL Ubuntu 22.04)

```bash
sudo bash sim/setup_wsl.sh <linux-user>          # ROS 2 Humble, Gazebo, Go2 sim (~10 min)
sudo bash sim/setup_unitree_msgs.sh <linux-user> # unitree_api / unitree_go
sudo bash sim/setup_docker.sh <linux-user>       # Docker Engine inside WSL
git clone /mnt/c/Users/<you>/go2-rf-survey ~/go2-rf-survey   # Linux line endings
cd ~/go2-rf-survey && bash tools/go2loop.sh build # our image (~10 min)
```

## Use

```bash
bash sim/run_sim.sh up small_room        # or: box (obstacle test), classroom, furnished, hall
AISLE=1.4 bash sim/run_sim.sh up classroom
LAYOUT=2 bash sim/run_sim.sh up furnished   # 9 x 7 m, random tables/chairs per LAYOUT
LAYOUT=1 bash sim/run_sim.sh up hall        # 25 x 20 m, rows of desks
YAW_GAIN=0.5 bash sim/run_sim.sh up ...     # turn at half the commanded rate, like the Go2
bash tools/demo.sh --explore --i-confirm-breathing --remote-start --detach --skip-stage-gate
bash sim/run_sim.sh status               # topic rates
bash tools/go2loop.sh preflight --no-update
bash tools/go2loop.sh bridge-up && bash tools/go2loop.sh turn-test --i-confirm-breathing
bash tools/demo.sh --length 1.0 --i-confirm-breathing --remote-start --detach
bash sim/run_sim.sh press SELECT+UP 2    # the fake remote: start / resume
bash sim/run_sim.sh press SELECT+DOWN 1  # abort;  stick 0.9 1 = push a stick
python3 sim/stage0_probe.py 6            # what the obstacle check sees, and where
bash sim/run_sim.sh down
```

Worlds come from `sim/worlds/make_worlds.py` (placeholder classroom: 3 m of
desks either side of the aisle from the door; `--aisle` sets the width;
furnished room and hall: furniture placed from `--layout N`).

## Results, 2026-10-26

| Test | Result |
|---|---|
| Preflight | motion-ready (after making the sim match the robot: odometry ≥ 50 Hz, LiDAR 15 Hz, standing height) |
| Stage 1 turn test | **PASS**: 25° commanded, 16° while commanded, response 0.42 s, stopped by our watchdog |
| Untethered rehearsal (remote start, detach) | **rehearsal OK**, floorplan built |
| Stage 2: live 1 m, remote start | **done**: out, turn round, back; 0.17 m from the start; Stage 2 recorded |
| Classroom 3 m, 1.2 m aisle | out, turn, then **paused** on the way back: the turn shifted it 14 cm sideways, desks inside the 0.55 m ring |
| Classroom 3 m, 1.4 m aisle | out, turn, **paused** briefly on the way back (12 cm off-centre); pauses wait for a resume, and the 5 min limit ended it |

**After the fix (2026-10-27): the follower steers along the route line**
(`lookahead` 0.6 m) instead of at the corner, which pulls the robot back to
the centre after a turn.

| Aisle | Before (aiming at the corner) | After (route line) |
|---|---|---|
| 1.4 m (15 cm slack each side) | paused on the return leg | **2 of 2 done** |
| 1.2 m (5 cm slack each side) | paused on the return leg | **3 of 4 done**; the miss: 6 cm off-centre where the desks begin, 0.8 m after the turn |

A 1.2 m aisle stays marginal: the turn shifts the robot sideways and it has
less than a metre to re-centre before the desks. Turning further beyond the
last desk, or a narrower side check on straight legs, would add margin.

Found and fixed in our code: the robot's own legs drawn into every floorplan
(`bag_to_map` now leaves them out); rays through an open door stretching a
floorplan to 136 m (`--max-range`, default 15 m); container log times in
UTC beside host times in local time.

The sim's own artifacts, fixed in the adapter / launch: CHAMP's leg odometry
misses the trot's pitch (floor read as obstacles, position lost) → ground
truth pose; 10 Hz ground truth → 150 Hz, and each scan placed with the pose
at its own time; Velodyne off without `always_on`, level, and blind under
0.9 m → patched; worlds must be named `default` for the Velodyne plugin.

## Results, 2026-10-28 to 10-31: a sim that turns like the Go2

On the robot the Go2 only *sways* for gentle turn or walk requests (firmware
`mcf`; see [autonomous-loop.md](../docs/autonomous-loop.md#stage-1--bridge-only-robot-turns-on-the-spot)).
`YAW_GAIN` makes the sim turn at a fraction of the commanded rate, so every
change was tried on a sluggish robot before the real one.

**Routes** (small room, 1 m out-and-back, Stage 1 turn test first):

| Change | Sim result |
|---|---|
| Brisk turns (1.0 rad/s), `YAW_GAIN=0.33` / `0.5` | turn test PASS; 1 m route done |
| Steady 0.3 m/s walking, stall check, L2+A abort | 1 m route done (the robot had stalled short of the turn point at 0.13 and 0.20 m/s) |
| Turn memory, first version | **stalled**: "no room to turn" at the turn point, every time |
| … robot's own legs dropped before remembering | 1 m route done |

The turn-memory stall was a real bug the robot would have hit too: the LiDAR
sees the robot's own legs, and remembered at every step they became
obstacles left behind it, inside its own turn sweep.

**Exploring the furnished room** (9 × 7 m, tables, chairs, a cupboard; three
layouts; `YAW_GAIN=0.5`):

| Version | Layout 1 | Layout 2 | Layout 3 |
|---|---|---|---|
| Greedy, first | stuck: legs ended beside a chair, could not turn | sim pose dropout, timed out | done, 3 legs |
| + legs end 0.8 m clear of anything | done: 8 legs, 21 m, home 0.14 m | done: 4 legs, 10 m, 0.13 m | done: 8 legs, 23 m, 0.12 m |
| + wider margins (after the robot brushed a podium and a desk) | done: 9 m, 0.11 m | done: 5 m, 0.09 m | done: 8 m, 0.10 m |
| + turn memory (legs fixed) | done: 8 legs, 22 m, 0.11 m | done: 8 legs, 22 m, 0.10 m | done: 8 legs, 23 m, 0.10 m |

**Exploring the hall** (25 × 20 m, seven rows of desks, greedy, 20 legs
within 10 m, 2.5 m legs):

| Layout | Result |
|---|---|
| 1 | done: 20 legs, 80 m walked, home within 0.13 m, no pauses |
| 2 | done: 20 legs, 92 m, home within 0.14 m, no pauses |

Safe at this size, but **inefficient**: it never went deeper than ~8–9 m into
the hall (it avoids where it has *walked*, not what it has already *seen*),
half the distance was the walk home retracing every leg, and the far half of
the hall was only glimpsed between desks. That is what frontier exploring
(`coverage.py`) is for — results below.
