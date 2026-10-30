# go2-rf-survey

Context for resuming work on this project. Read this first.

## What this is

A Unitree Go2 EDU that maps a building with LiDAR, then walks it again
recording WiFi signal strength at every waypoint, producing per-SSID heatmaps
registered to the SLAM floorplan.

Hardware: Go2 EDU (Jetson Orin, `192.168.123.18`), Livox Mid-360, RealSense
D435i, plus a dedicated USB WiFi adapter for scanning.

## Where things stand

**Hardware bring-up under way (sessions 2026-10-20, -21).** Current goal is an
autonomous classroom *mapping* demo on the Go2's **built-in** LiDAR; the WiFi
survey waits for it.

| Phase | State |
|---|---|
| 00 Access and inventory | ✅ 2026-10-20 (eth0, NVIDIA runtime); password left at default (demo); key-based `ssh go2` |
| 01 ROS runtime | ✅ Humble container builds and runs on the robot, Unitree msgs baked in |
| 02 Mid-360 bring-up | 🟨 at `192.168.123.20`; `livox_ros_driver2` initialises but receives no data (unresolved) |
| 03 Mount and TF | 🟨 mounted nose-down; exact pitch still to measure for URDF |
| 04 SLAM | 🟨 config written; mapping meanwhile = built-in `/utlidar/*` + `tools/bag_to_map.py` |
| 05 **WiFi survey** | ✅ written + tested — the deliverable (deferred) |
| 06 Velocity bridge | ✅ watchdog stops it cleanly; turn test passes at 1.0 rad/s (gentler turns only sway the Go2); untethered runs done |
| 07 Depth and costmaps | 🟨 D435i present on USB (`8086:0b3a`); not brought up |
| 08 Nav2 | ⬜ (later: go2_ros2_sdk) |
| — Autonomous loop | ✅ routes and exploration live on the robot, untethered; stages gated (`tools/stage_gate.py`); a report per run |

Run on the robot 2026-10-28: preflight, image build, live view + camera,
remote-keys. **Not yet run on the robot:** `tools/demo.sh`, untethered runs,
`state-watch`. All pass the fake robot / sim.

**Session 2026-10-28 (guide 01, reached step J):** remote keys verified;
image rebuilt; camera works; motion mode is `mcf` (firmware 1.1.6+ name for
`normal`, accepted). **The stop is L2+B, not L2+START** (fixed everywhere).
Turn test failed 3× by under-response: prompt start, clean stop, but a twist
on planted feet with `/sportmodestate` mode=0 gait=0 — the gait never ran.

**Solved later that day:** the Go2 only *sways* for gentle turn requests —
from the remote too (stick 10% → 0.03 rad/s, 30% → 0.5, 40% → 0.8). The
0.2–0.5 rad/s tests sat in that zone. At 1.0 rad/s × 0.5 s the turn test
delivered 58% and **passed Stage 1**. Follower now turns in place at
0.7–1.0 rad/s (bridge limit 1.0); the turn-test rule needs 15% of the
commanded turn and fails a wrong-way turn. The sim (`YAW_GAIN=0.5`) completes
the 1 m out-and-back. `/sportmodestate` mode/gait and foot fields are
meaningless on this firmware. A hand-driven map of the room worked.

**First autonomous walk, 2026-10-29:** `demo.sh --length 1.0 --remote-start
--detach`, untethered, started with SELECT+UP: out 1 m, turned round, back,
`demo finished: done` in 22 s, 0.19 m from the start, floorplan built.
Stage 2 passed.

**2026-10-31:** 1 m runs stalled in the forward sway zone (≤0.2 m/s), then
passed at a steady 0.3 m/s; stall check + L2+A abort added after the robot
was laid down mid-run while the route kept commanding. **Exploration built**
(`demo.sh --explore`, guide 04): 3 of 3 sim rooms done, not yet on the robot.

**Next action:** guide 04 on the robot (turn test, 1 m walk, then explore with
the wider margins + turn memory; watch the podium/desk). Then
`python tools\get_results.py` on the laptop.
Then the 3 m route / classroom (guide 02). Each session:
route (user sudo) → preflight → `bridge-up` + `turn-test --i-confirm-breathing`
(defaults to the 1.0 rad/s × 0.5 s turn that passes) → [ASK] demo. Robot-side
state that resets each boot: `sudo ip route replace default via
192.168.123.222 dev eth0` (user runs sudo).

**Working with the robot:** ask before anything that can move it; the user
runs sudo commands; diagnostics and builds are fine without asking.

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
  clock-free so they can be tested without hardware. 375 tests run on a bare
  Python. Keep it that way — if new logic can be made testable, make it.
- **Tests before hardware.** `python tools/run_tests.py` must stay green and
  must not acquire dependencies.
- **The Gazebo sim** (`sim/`, WSL Ubuntu on the laptop, user `rofessor`) runs our
  real image and scripts against a simulated Go2; see `sim/README.md`. Drive WSL
  from Git Bash (`MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 -- bash <script>`);
  PowerShell mangles nested quotes. WSL clone: `~/go2-rf-survey` (pull from the
  Windows repo).
- **Shell flows get the fake robot.** After changing `demo.sh`, `go2loop.sh`
  or `remote_trigger.py`, run `bash tools/fake_robot/run.sh` (Git Bash on the
  laptop, ~5 min, 20 scenarios): real scripts against fake docker/ROS/remote.
  It found a pipefail bug that made a healthy `bridge-up` report failure.
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
go2_survey_bridge/       /cmd_vel → sport mode           ✅ tested
go2_survey_loop/         autonomous route + obstacle stop ✅ tested
go2_survey_description/  TF frames, 22° LiDAR mount      ✅ 21 geometry tests
tools/                   post-processing + on-robot scripts (go2loop.sh, demo.sh); fake_robot/
docker/                  Humble container for the Jetson
config/                  mount, MID360, FAST-LIO2 params
docs/                    field-guide/, autonomous-loop, live-view, RUNBOOK, HARDWARE, BUILD_PLAN, plan
```

## Docs

| | |
|---|---|
| [docs/field-guide/](docs/field-guide/README.md) | hands-on session steps (user-facing) |
| [docs/autonomous-loop.md](docs/autonomous-loop.md) | the route, safety layers, staged rollout |
| [docs/RUNBOOK.md](docs/RUNBOOK.md) | what to type, in order |
| [docs/HARDWARE.md](docs/HARDWARE.md) | what to buy, what to avoid |
| [docs/BUILD_PLAN.md](docs/BUILD_PLAN.md) | nine phases, exit criteria, risks R1–R8 |
| [docs/plan.md](docs/plan.md) | derivations and decision records |
