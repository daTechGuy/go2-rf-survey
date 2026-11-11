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
| SLAM | Point-LIO on the built-in L1: `/utlidar/cloud` + `/utlidar/imu` (config `utlidar.yaml`). The Mid-360 is not used. |
| Avoidance | `localPlanner` (path library) + `terrain_analysis`; obstacles above 0.3 m (`obstacleHeightThre`) — README: lower ones can be missed |
| Goals | `geometry_msgs/PointStamped` on **`/way_point`** (the `waypoint_example` node shows how) |
| Motion | `pathFollower` publishes **`Move` / `StopMove` straight to `/api/sport/request`** at ~50 Hz when `is_real_robot` is true — our bridge (clamp, 0.3 s watchdog, e-stop) is **bypassed** |
| Speed | from a PS3 joystick by default; `autonomyMode:=true` + `autonomySpeed` makes it autonomous. Default `maxSpeed` 1.0 m/s |
| Stop input | `/stop` (`std_msgs/Int8`): bit 1 forward, 2 back, 4 and 8 the two turn directions — `15` blocks all motion |
| Launch | `system_real_robot.launch`: Point-LIO, local planner, terrain analysis, a PS3 joy node, **rviz2** (no screen on the robot: expect it to fail harmlessly) |

## The IMU calibration MOVES the robot

`ros2 run calibrate_imu calibrate_imu`, once per robot: 2 s settle, 13 s
still, then **turns on the spot at 80°/s for 20 s (about 4½ turns)**, stops,
writes `~/Desktop/imu_calib_data.yaml`. Needs a clear circle ~1.2 m across
and the user's go.

## Things to change before it drives

- `autonomyMode:=true`, `autonomySpeed:=0.3`, `maxSpeed:=0.3` (our proven
  walking speed; 1.0 is the default).
- The Go2 only **sways** for turns under ~0.7 rad/s and walks under
  ~0.3 m/s (2026-10-28/31). `pathFollower` turns proportionally
  (`stopYawRateGain` 1.5 x heading error) and slows near goals: it may
  stall turning on the spot or just short of a goal. Watch for it; tune
  `stopYawRateGain` / a minimum turn rate if so.
- A way to stop it from our side: publish `/stop 15`, then `StopMove`
  (`go2loop.sh bridge`'s path, or a one-shot request). The remote's
  **L2+B** works regardless.
- Unknown: whether the Go2 keeps walking on the last `Move` if the stack
  dies mid-command. Until that is known, stop it with `/stop 15` first and
  only then kill the launch.

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
check `/state_estimation`, `/terrain_map`, `/path` rates; drive the robot by
remote and see SLAM keep up; put a goal on `/way_point` and watch `/path`
bend round the chair. Wait out the 10–20 s startup delay the README warns of.

**S2 — IMU calibration (MOVES: 20 s spin).**

**S3 — first autonomous goal (MOVES).** Chair 1.5 m ahead, one waypoint
3 m ahead behind it, 0.3 m/s, `/stop 15` and L2+B ready. It should curve
round the chair without stopping.

**S4 — wire in ours.** Routes → successive `/way_point`s; exploration's
next goal → `/way_point`; the remote's abort combos → `/stop 15` +
`StopMove`; recordings and floorplans as now.
