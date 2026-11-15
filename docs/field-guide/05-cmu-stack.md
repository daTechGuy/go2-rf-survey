# 05 — Trying the CMU autonomy stack

Decided 2026-11-11 (D5 in CLAUDE.md): try
[jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2)
(branch `foxy-humble`, read at commit `43d5f54`) for obstacle avoidance,
instead of growing our own. Ours stops, waits and plans a detour; each fix
exposed the next edge case (flickering counts, unseen corners, placement).
CMU's local planner checks 343 precomputed curved paths against the
obstacles ~10 times a second and steers along the clear one closest to the
goal, so the robot goes round things without stopping. It is proven on the
Go2 EDU, by the author of LOAM.

Our code stays: routes and exploration become goals sent to it, and the
mapping, Mid-360 and WiFi survey remain ours.

## What the code does (read, not assumed)

| | |
|---|---|
| Runs on | the robot's own computer, **native** Ubuntu 20.04 + ROS 2 Foxy (not our Humble container: the README reports >1 s delays on Humble) |
| SLAM | Point-LIO on the built-in L1: `/utlidar/cloud` + `/utlidar/imu` (config `utlidar.yaml`). `/state_estimation` and `/registered_scan` are stamped **`camera_init`** (`laserMapping.cpp:691,776`; the launch publishes an identity `map` → `camera_init` TF), and `/state_estimation` is the **L1 LiDAR's** pose (`sensorOffsetX` = 0.3 m ahead of the body centre: `cmu.vehicle_pose` shifts it back, just as `localPlanner` and `pathFollower` do). |
| Avoidance | `localPlanner` (path library) + `terrain_analysis`; obstacles above 0.3 m (`obstacleHeightThre`) — README: lower ones can be missed |
| Goals | `geometry_msgs/PointStamped` on **`/way_point`** ( only listened to when `autonomyMode` is true — the launch default is **false**) |
| Motion | `pathFollower` publishes **`Move` / `StopMove` straight to `/api/sport/request`** at ~50 Hz when `is_real_robot` is true — our bridge (clamp, 0.3 s watchdog, e-stop) is **bypassed** |
| Speed | from a PS3 joystick by default; `autonomyMode:=true` + `autonomySpeed` makes it autonomous. Default `maxSpeed` 1.0 m/s. **Note:** `system_real_robot.launch` starts `joy_node` on `/dev/input/js0`; any `/joy` message with L2 unpressed (`axes[2] > -0.1`) resets `autonomyMode` to `false` in both nodes (`localPlanner.cpp:236`, `pathFollower.cpp:183`). Unplug any USB joystick or comment `joy_node` out. |
| Goal slow-down | `pathFollower` slows linearly inside `slowDwnDisThre` (0.75 m) of its goal and stops at `stopDisThre` (0.3 m). At 0.5 m to the goal it commands 0.2 m/s — the Go2's forward sway zone — and it never reaches within our 0.2 m `reach_tol`. Our CMU adapter (`cmu.py`) therefore sends a **carrot** 0.8 m beyond the waypoint along the leg and counts the waypoint reached as soon as the robot crosses its line (`passed`). |
| Stop input | `/stop` (`std_msgs/Int8`): bit 1 forward, 2 back, 4 and 8 the two turn directions — `15` blocks all motion |
| Launch | `system_real_robot.launch`: Point-LIO, local planner, terrain analysis, a PS3 joy node, **rviz2** (no screen on the robot: expect it to fail harmlessly) |

## The IMU calibration MOVES the robot

`ros2 run calibrate_imu calibrate_imu`, once per robot: 2 s settle, 13 s
still, then **turns on the spot at 80°/s for 20 s (about 4½ turns)**, stops,
writes `~/Desktop/imu_calib_data.yaml`. Needs a clear circle ~1.2 m across
and the user's go.

## Things to change before it drives

- In `src/base_autonomy/local_planner/launch/local_planner.launch` (note:
  `system_real_robot.launch` does not forward these arguments, so edit
  `local_planner.launch` directly): `autonomyMode` default `true`,
  `autonomySpeed` default `0.3`, `maxSpeed` default `0.3` (1.0 is the
  stock default; `go2loop.sh cmu-check --live` and `demo.sh --cmu` read the
  live node parameters and **refuse** if `autonomyMode` is false or speed > 0.5 m/s).
- In `system_real_robot.launch`: comment out `joy_node` (or make sure
  `/dev/input/js0` is absent), and comment out `rviz2`.
- When launching the CMU stack in a Foxy shell on the robot, set
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI` on `eth0`
  (same as our container; `system_real_robot.sh` does not set them, and the
  repo's `unitree_setup.sh` names `enp3s0`).
- The Go2 only **sways** for turns under ~0.7 rad/s and walks under
  ~0.3 m/s (2026-10-28/31). `pathFollower` turns on the spot at
  `stopYawRateGain * dirDiff` (`local_planner.launch` sets `stopYawRateGain`
  to 1.5); watch for swaying on small heading corrections and raise
  `stopYawRateGain` if needed.
- A way to stop it from our side: `bash ~/go2-rf-survey/tools/go2loop.sh cmu-stop`
  publishes `/stop 15` (`std_msgs/Int8`) and `StopMove` (`api_id: 1003`, with
  request id) directly on `/api/sport/request`. `remote_trigger.py` also
  publishes `/stop 15` + `StopMove` directly on any remote abort (SELECT+DOWN,
  L2+B, or stick push), even if `go2_loop` has died. The remote's **L2+B**
  works in firmware regardless.
- Unknown: whether the Go2 keeps walking on the last `Move` if the stack
  dies mid-command. Until that is known, stop it with `go2loop.sh cmu-stop`
  first and only then kill the launch.

## Session plan

Each step says whether the robot moves. Ask before every one that does.

**S0 — install (nothing moves, ~45 min).**
1. Internet on the robot through the laptop hotspot. With the cable
   unplugged, the stale route via the laptop's cable is the blocker:
   `sudo ip route del default via 192.168.123.222 dev eth0` (user, sudo).
2. Clock: with WiFi it syncs itself (README: it resets to 1970 at boot).
3. Dependencies (user, sudo):
   `sudo apt install libusb-dev ros-foxy-perception-pcl ros-foxy-sensor-msgs-py ros-foxy-tf-transformations ros-foxy-joy ros-foxy-rmw-cyclonedds-cpp ros-foxy-rosidl-generator-dds-idl`
   then `pip install transforms3d pyyaml`.
4. Code: `git clone -b foxy-humble https://github.com/jizhang-cmu/autonomy_stack_go2.git ~/autonomy_stack_go2`
   (or copy the laptop's checked copy, `C:\Users\Professor\vendor\autonomy_stack_go2`).
5. Build, in a Foxy shell: `source /opt/ros/foxy/setup.bash; cd ~/autonomy_stack_go2; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.
   Untimed on the Orin; expect tens of minutes.

**S1 — sensing only (nothing moves).** Launch with `is_real_robot:=false`
(pathFollower then only publishes `/cmd_vel`, never `/api/sport/request`):
wait out the startup delay, then run
`bash ~/go2-rf-survey/tools/go2loop.sh cmu-check` to check `/state_estimation`,
`/registered_scan` (frame `camera_init`), `/terrain_map`, `/path` rates and
`/way_point` + `/stop` subscribers; drive the robot by remote and see SLAM
keep up; send a test goal with `bash ~/go2-rf-survey/tools/go2loop.sh cmu-goal 2.0 0.0`
and watch `/path` bend round the chair.

**S2 — IMU calibration (MOVES: 20 s spin).**

**S3 — first autonomous goal (MOVES).** Chair 1.5 m ahead, one waypoint
3 m ahead behind it (`bash ~/go2-rf-survey/tools/go2loop.sh cmu-goal 3.0 0.0`
sets `/stop 0` and publishes `/way_point`), 0.3 m/s,
`bash ~/go2-rf-survey/tools/go2loop.sh cmu-stop` and L2+B ready. It should
curve round the chair without stopping. Record a Stage 1 pass for the `cmu`
backend once verified:
`python3 ~/go2-rf-survey/tools/stage_gate.py record 1 --backend cmu`.

**S4 — wire in ours.** Built and tested offline (`loop_node.py -p planner:=cmu`,
`demo.sh --cmu`): routes → successive `/way_point` carrots (crossed-the-line
advancement); exploration's next goal → `/way_point` (re-planning if CMU
stalls at a front obstacle); body clearance ring (`near < 0.25 m`), geofence,
sensor staleness, and the remote's abort combos → `/stop 15` + `StopMove`;
`go2bridge` is skipped so only CMU's `pathFollower` drives `/api/sport/request`;
recordings and floorplans as now:
```bash
bash ~/go2-rf-survey/tools/demo.sh --cmu --length 1.0 --rehearse
bash ~/go2-rf-survey/tools/demo.sh --cmu --length 1.0 --i-confirm-breathing --remote-start --detach
```
