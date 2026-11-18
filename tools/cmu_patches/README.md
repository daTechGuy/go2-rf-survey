# CMU stack patches for this robot

Patches to `jizhang-cmu/autonomy_stack_go2` (branch `foxy-humble`, commit
`43d5f54`) that were needed on this Go2 EDU (firmware 1.1.6+, Orin NX in 15 W
mode). The stack is cloned on the robot at `~/autonomy_stack_go2`, outside this
repo, so a re-clone or reflash loses them. Both patch files apply cleanly
(`git apply --check`) to an untouched `43d5f54`. Found and verified 2026-11-19.

As cloned, the stack **could not follow a walk** here. Four separate problems,
found in this order, each hid the next:

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 1 | 10 m walked read as 0.08 m | `transform_everything` runs the 250 Hz IMU and the 15 Hz cloud on one Python thread; the cloud callback loops over every point, so the transformed IMU fell to ~13 Hz | callback groups + 2-thread executor, vectorised filter box (IMU ~200-240 Hz) |
| 2 | map floor tilted 24 deg, height drifted 0.3-0.4 m per metre walked | the upstream mount rotation does not match this robot | measured level rotation, applied to cloud AND gyro (`make_level.py`) |
| 3 | pose **95 s behind real time and growing ~0.2 s per second**; looked like "drift at rest" and "late walks" | Point-LIO runs slower than real time on 4 cores at 15 W (3,800 points/scan at 15 Hz, no skipping); its input buffer is unbounded | `mapping_utlidar.launch`: `point_filter_num` 4, `filter_size_surf` 0.3, `filter_size_map` 0.3 -> lag 0.1-0.5 s and flat |
| 4 | pose wandered 5 m at rest, 31 m of path for 5 m walked, height drift back; walks showed motion **-111 deg off the heading** (3 walks) | the lidar cloud is yawed ~113 deg relative to the IMU body frame the pose uses; Point-LIO fuses the two. The L1's best upward view also sat at azimuth -115 | rotate the CLOUD only by `cloud_yaw_deg` (113) after the level rotation, not the gyro |

Result (fresh stack, forward / sideways / left-turn walk, against the Go2's
own leg odometry `/utlidar/robot_odom`): forward direction within 1.5 deg,
pose lag under 1 s, 0.09 m wander over 82 s standing, height flat
(-0.01..+0.20 m), turn +100 deg vs +102 deg. **SLAM reports ~15-20% more
distance than leg odometry** (3.2 m vs 2.8 m forward): not yet checked with a
tape measure. The L1 now sees +7 deg above horizontal straight ahead, and the
planner's path bends round a chair 2 m ahead (0.6 m clearance).

## Files

| File | What |
|---|---|
| `transform_everything.patch` | fixes 1, 2 and 4. Loads `~/Desktop/level_correction.yaml` (`R_level`, `cloud_yaw_deg`) and `~/Desktop/imu_calib_data.yaml`; prints `level_correction.yaml loaded` and `cloud yaw correction NN deg` at startup |
| `launch_files.patch` | fix 3 (`mapping_utlidar.launch`), plus `local_planner.launch`: `maxSpeed` / `autonomySpeed` 0.3, `autonomyMode` true, **`is_real_robot` false**; `system_real_robot.launch`: `joy_node` and `rviz2` commented out |
| `make_level.py` | measures the floor-level rotation (robot standing still and level, stack up) and writes `level_correction.yaml`; keeps an existing `cloud_yaw_deg` |
| `dual_odometry_record.py` + `yaw_from_walk.py` | record SLAM and leg odometry together, then measure the cloud yaw from a straight forward walk |
| `lag_check.py` | prints the timestamp lag of every pipeline stage. Healthy: `state_estimation` / `registered_scan` under ~1 s and **not growing** (the raw `/utlidar/*` stamps are a constant ~49.5 s off the system clock: the transform node corrects that) |
| `ransac_floor.py` | floor tilt on any topic; healthy under ~1.5 deg on `/utlidar/transformed_cloud` and `/registered_scan` |

**`is_real_robot` is false on purpose.** The upstream `system_real_robot.launch`
does not forward it and `local_planner.launch` defaults it to true, so the
guide's `is_real_robot:=false` never reached `pathFollower`. With it false,
`pathFollower` only publishes `/cmd_vel` and cannot send `Move` to
`/api/sport/request`. Set it true only for S3, and only after asking.

Apply on a fresh clone (from `~/autonomy_stack_go2/src`):

```bash
git apply -p1 ~/go2-rf-survey/tools/cmu_patches/transform_everything.patch
git apply -p1 ~/go2-rf-survey/tools/cmu_patches/launch_files.patch
```

(`--symlink-install`: no rebuild needed for these four files.)

## Per-robot calibration (in `~/Desktop`, not in the repo)

- `imu_calib_data.yaml`: `ros2 run calibrate_imu calibrate_imu` (S2; the robot
  turns on the spot at 80 deg/s for 20 s). This robot: `ang_z2x_proj` 0.134,
  `ang_z2y_proj` -0.195.
- `level_correction.yaml`: run `make_level.py` with the robot **standing still
  and level on flat floor** and the stack up. This robot: tilt 24.3 deg (12
  scans, 0.34 deg spread). Then add `cloud_yaw_deg`: walk the robot straight
  forward a few metres with `dual_odometry_record.py` running, run
  `yaw_from_walk.py`, add the printed value. This robot: **113** (measured
  -108..-115 over three walks; good to about +-5 deg). Re-measure after a
  firmware update or if the L1 is re-mounted.

## Operating notes

- The stack is not autostarted. Launch script on the robot: `~/cmu_launch.sh`
  (sources Foxy, `~/cyclonedds_ws`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
  `CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml`; `~/.bashrc` has an
  interactive prompt, so do not source it).
- After moving the robot, restart the stack (Point-LIO takes its frame from
  the first ~40 s standing still).
- Stop nodes with `kill` on the PIDs (SIGTERM). Processes started with `&` from
  a non-interactive shell ignore SIGINT. Do not put `pkill -f` / `pgrep -f`
  inside the same `ssh "..."` string as its own pattern: it matches its own
  shell and kills it (put it in a script file).
- Foxy's `ros2 topic echo` has no `--once`.
- The Jetson is in 15 W mode with 4 of 8 cores online. A higher `nvpmodel`
  mode (25 W / MAXN) would add headroom but is a system setting and a
  power/heat trade-off: the user's call, not changed here.
