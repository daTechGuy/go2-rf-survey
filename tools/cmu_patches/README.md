# CMU stack patches for this robot

Patches to `jizhang-cmu/autonomy_stack_go2` (branch `foxy-humble`, commit
`43d5f54`) that were needed on this Go2 EDU (firmware 1.1.6+). The stack is
cloned on the robot at `~/autonomy_stack_go2`, outside this repo, so a
re-clone or reflash loses them. Both patch files apply cleanly
(`git apply --check`) to an untouched `43d5f54`. Found and verified 2026-11-19.

## What and why

| File | Fixes |
|---|---|
| `transform_everything.patch` | (1) **IMU starved:** upstream runs the IMU (250 Hz) and cloud (15 Hz) callbacks on one thread, and the cloud callback loops over every point in Python, so the transformed IMU dropped to ~13 Hz and Point-LIO could not follow a walk (10 m walked read as 0.08 m). Now separate callback groups on a 2-thread executor, and the filter box is vectorised: IMU ~200-240 Hz. (2) **Tilted map:** the upstream mount rotation does not match this robot, so the map floor was tilted 24 degrees (height drifted 0.26-0.4 m per metre walked, to -1.3 m over a room). Now loads `~/Desktop/level_correction.yaml` and applies that rotation to the cloud (after the body filter box) and to the gyro. Result: floor tilt 0.3 deg in the body cloud, 0.9 deg in the map, walked height flat (-0.01..+0.07 m over 5 m). |
| `launch_files.patch` | `local_planner.launch`: `maxSpeed` and `autonomySpeed` 0.3, `autonomyMode` true, **`is_real_robot` false**. `system_real_robot.launch`: `joy_node` and `rviz2` commented out. **The upstream `system_real_robot.launch` does not forward `is_real_robot`, and `local_planner.launch` defaults it to true**, so the guide's `is_real_robot:=false` never reached `pathFollower`. With it false, `pathFollower` only publishes `/cmd_vel` and cannot send `Move` to `/api/sport/request`. **Set it back to true only for S3, and only after asking.** |

Apply on a fresh clone (from `~/autonomy_stack_go2/src`):

```bash
git apply -p1 ~/go2-rf-survey/tools/cmu_patches/transform_everything.patch
git apply -p1 ~/go2-rf-survey/tools/cmu_patches/launch_files.patch
```

(`--symlink-install` means no rebuild is needed for these two files.)

## The two calibration files (per robot, in `~/Desktop`)

- `imu_calib_data.yaml`: from `ros2 run calibrate_imu calibrate_imu` (S2: 13 s
  still, then a 20 s on-the-spot spin at 80 deg/s; the robot moves). This
  robot: `ang_z2x_proj` 0.134, `ang_z2y_proj` -0.195, `ang_bias` 0.031 / -0.0005
  / 0.009.
- `level_correction.yaml`: from `make_level.py` (**the robot must stand still
  and level on flat floor**, with the stack up so
  `/utlidar/transformed_cloud` is publishing). It RANSAC-fits the floor in the
  body-frame cloud, averages 12 scans and writes the smallest rotation that
  makes the floor normal vertical. This robot: tilt 24.3 deg, 0.34 deg spread.
  **Re-measure after a firmware update, or if the L1 is re-mounted.** The
  node prints `level_correction.yaml loaded, tilt NN deg` at startup (or says
  `NOT loaded`).

## Check it

`ransac_floor.py <topic>` prints the floor tilt on a topic (run it in a Foxy
shell with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and the CycloneDDS config).
Healthy: `/utlidar/transformed_cloud` and `/registered_scan` under about 1.5
deg, `/utlidar/transformed_imu` above 150 Hz.

## Operating notes

- Stop the launch with `kill` on the node PIDs (SIGTERM). Processes started
  from a non-interactive shell with `&` ignore SIGINT.
- Do not use `pkill -f` / `pgrep -f` with a pattern inside the same `ssh "..."`
  command string: it matches its own shell.
- Foxy's `ros2 topic echo` has no `--once`.
