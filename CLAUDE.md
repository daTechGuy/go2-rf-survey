# go2-rf-survey

Context for resuming work on this project. Read this first.

## What this is

A Unitree Go2 EDU that maps a building with LiDAR, then walks it again
recording WiFi signal strength at every waypoint, producing per-SSID heatmaps
registered to the SLAM floorplan.

Hardware: Go2 EDU (Jetson Orin, `192.168.123.18`), Livox Mid-360, RealSense
D435i, plus a dedicated USB WiFi adapter for scanning.

## Where things stand

**No hardware access yet.** Everything buildable without the robot is built
and tested; everything remaining needs real sensor data.

| Phase | State |
|---|---|
| 00 Access and inventory | 🟨 SSH + inventory done 2026-10-20 (eth0, NVIDIA runtime present); password left at default (demo), /sportmodestate not yet verified |
| 01 ROS runtime | 🟨 container build fixed (ROS2/SDK-driver pairing, 2026-10-20), rebuilding |
| 02 Mid-360 bring-up | 🟨 LiDAR found at `192.168.123.20` (same subnet, no alias); config set; driver not yet launched |
| 03 Mount and TF | 🟨 mounted nose-down; exact pitch still to measure for URDF |
| 04 SLAM | 🟨 config written, needs the container + hardware run |
| 05 **WiFi survey** | ✅ written + tested — the deliverable (deferred; current goal is map-only) |
| 06 Velocity bridge | ✅ written + tested (not used yet; first motion was via vendor SDK) |
| 07 Depth and costmaps | 🟨 RealSense D435i present on USB (`8086:0b3a`); not yet brought up |
| 08 Nav2 | ⬜ |

**Next action:** finish the container build (fixes in `docker/Dockerfile`,
2026-10-20), then start it (`IFACE=eth0 bash docker/run.sh`), launch the Livox
driver, confirm `/livox/lidar`, and run FAST-LIO2 to map the room (teleop via
remote, collision avoidance on). See `docs/bringup-log-2026-10-20.md` for the
full session state and the internet-sharing + robot-local config details.

## Decisions already made — don't relitigate

Full reasoning in [`docs/plan.md`](docs/plan.md).

- **D1** — Humble in a Docker container, not the stock Foxy. Foxy is EOL since
  June 2023. *Revisit only if phase 0 shows Docker absent or without the
  NVIDIA runtime.*
- **D2** — Alias a second IP on the Jetson rather than reflashing the Mid-360.
  Non-destructive and reversible. *(Moot on this unit: the Mid-360 is already at
  `192.168.123.20`, same subnet as the Jetson — no alias needed. 2026-10-20.)*
- **D3** — Pre-mapped waypoints, not live frontier exploration. Mapping and
  surveying in one autonomous pass is where projects of this shape stall.
- **D4** — Stop-and-scan, timestamped at scan *completion*. A moving scan
  smears 1–2.5 m of travel into one sample.
- The bridge talks to `unitree_ros2`'s `/api/sport/request` topic rather than
  linking `unitree_sdk2`. No C++ build against an external SDK, and the safety
  logic stays unit-testable.

## Conventions

- **Pure functions for anything that can be wrong quietly.** `iw` parsing,
  velocity limiting, the watchdog, and the FOV maths are all ROS-free and
  clock-free so they can be tested without hardware. 85 tests run on a bare
  Python. Keep it that way — if new logic can be made testable, make it.
- **Tests before hardware.** `python tools/run_tests.py` must stay green and
  must not acquire dependencies.
- **Placeholders are marked loudly.** `config/mount.yaml` and
  `config/MID360_config.json` contain values that are guesses. Never quietly
  treat them as measured.
- **The mount pitch lives in the URDF only** — never also in
  `MID360_config.json`'s extrinsics. Setting both double-applies the rotation,
  which looks almost right and is completely wrong.
- Commits: no co-author trailers. The user is sole contributor.

## Gotchas that have already cost time

- XML comments cannot contain `--`. Broke the xacro files once.
- A wrong interface in `CYCLONEDDS_URI` gives a clean build, no error, and an
  empty topic list. The container entrypoint checks this at startup.
- Docker shares the **host kernel**, so a container cannot rescue a WiFi
  driver that needs a newer kernel than the Jetson's 5.10.
- `git filter-branch` leaves backup refs under `refs/original/`; `git log
  --all` still shows the old messages until they are deleted.

## Hardware notes

- WiFi adapter must be **MT7612U** (mainline since 4.19). MT7921AU needs 5.18
  and RTL8814AU was never mainlined. See [`docs/HARDWARE.md`](docs/HARDWARE.md).
- The user owns an Edimax EW-7833UAC (`7392:a833`, RTL8814AU) — tested, works
  as hardware, wrong chipset. Recommendation is to replace it.
- Mid-360 vertical FOV is −7° to +52°, biased **upward**. Mounted level it
  cannot see the floor within 3.3 m. Pitch it ~22° nose-down.

## Layout

```
go2_survey_wifi/         scan + pose join → CSV          ✅ 18 tests
go2_survey_bridge/       /cmd_vel → sport mode           ✅ 46 tests
go2_survey_description/  TF frames, 22° LiDAR mount      ✅ 21 geometry tests
tools/                   post-processing, runs off-robot
docker/                  Humble container for the Jetson
config/                  mount, MID360, FAST-LIO2 params
docs/                    RUNBOOK, HARDWARE, BUILD_PLAN, plan
```

## Docs

| | |
|---|---|
| [docs/RUNBOOK.md](docs/RUNBOOK.md) | what to type, in order |
| [docs/HARDWARE.md](docs/HARDWARE.md) | what to buy, what to avoid |
| [docs/BUILD_PLAN.md](docs/BUILD_PLAN.md) | nine phases, exit criteria, risks R1–R8 |
| [docs/plan.md](docs/plan.md) | derivations and decision records |
