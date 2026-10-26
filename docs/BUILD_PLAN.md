# Full build plan

Complete execution plan for autonomous WiFi site surveying with a Unitree Go2
EDU, Livox Mid-360 and RealSense D435i.

Nine phases, each with commands, verification, and a stated exit criterion.
Phases are ordered by dependency — phase *n* assumes phase *n−1* passed. The
survey capability lands at **phase 5**, deliberately ahead of autonomy, so
there is a working deliverable even if Nav2 tuning stalls.

- Derivations and decision records: [`plan.md`](plan.md)
- Diagrams: https://claude.ai/artifact/SR2QY8zsuDdAiaqEbs2SBy

| | Phase | Effort | Status |
|---|---|---|---|
| 00 | [Access and inventory](#phase-00--access-and-inventory) | ~2 h | ✅ done 2026-10-20 |
| 01 | [ROS runtime](#phase-01--ros-runtime) | ~half day | ✅ Humble container on the robot |
| 02 | [Mid-360 bring-up](#phase-02--mid-360-bring-up) | ~1 day | 🟨 sensor at .20, driver gets no data |
| 03 | [Mount geometry and TF](#phase-03--mount-geometry-and-tf) | ~half day | 🟨 code written |
| 04 | [LiDAR-inertial SLAM](#phase-04--lidar-inertial-slam) | ~1 day | 🟨 config written |
| 05 | **[WiFi survey node](#phase-05--wifi-survey-node)** | ~half day | ✅ **written + tested** |
| 06 | [Velocity bridge](#phase-06--velocity-bridge) | ~half day | 🟨 watchdog OK on robot; first turn test failed |
| 07 | [Depth camera and costmaps](#phase-07--depth-camera-and-costmaps) | ~1 day | ⬜ |
| 08 | [Nav2 and waypoint route](#phase-08--nav2-and-waypoint-route) | several days | ⬜ |

**242 tests pass today** with no robot attached: `python tools/run_tests.py`.
Interim work not in the phase list: an autonomous out-and-back route on the
built-in LiDAR (`go2_survey_loop`), see [autonomous-loop.md](autonomous-loop.md)
and the hands-on [field guide](field-guide/README.md).

---

## Before anything: three purchases

Each removes a specific failure documented below. Ordering them late blocks a
phase.

| Item | Removes | Needed by |
|---|---|---|
| **USB WiFi adapter — MT7612U** | R2 — scanning tears down your own link | phase 5 |
| **USB-Ethernet adapter** for your laptop | — | phase 0 |
| **Mount with ~22° forward pitch** | R3 — near-field blindness | phase 3 |
| Second Go2 battery | R6 — 1–1.5 h of walking | phase 8 |

### Choosing the scanning adapter

Throughput is irrelevant — it only ever scans. What matters is whether the
chipset has a driver on **the Jetson's kernel**, which JetPack 5.1.1 puts at
**5.10**. Docker shares the host kernel, so the container does not help here.

| Chipset | Mainline since | On kernel 5.10 |
|---|---|---|
| **MT7612U** (`mt76x2u`) | 4.19 | ✅ plug and play |
| MT7921AU (`mt7921u`) | 5.18 | ❌ too new |
| RTL8812BU | 6.12 | ❌ far too new |
| RTL8812AU / 8814AU | never mainline | ❌ DKMS against a vendor kernel |

This inverts the usual advice: MT7921AU would normally be the better buy, but
on a 5.10 kernel it is useless without backporting. Known-good MT7612U parts:
**Netgear A6210**, **Alfa AWUS036ACM**.

#### Adapter on hand: Edimax EW-7833UAC — not recommended

Tested 2026-10-19. USB ID `7392:a833`, chipset **RTL8814AU**. Hardware is
healthy (enumerates, radio on), but it is the worst of the four cases:

- never merged to mainline
- Edimax's own driver supports kernel 2.6–4.4 and will not compile past 4.10
- the only path on 5.10 is out-of-tree DKMS
  ([`morrownr/8814au`](https://github.com/morrownr/8814au), covers 5.4–6.18)
  against a Jetson **vendor** kernel — needs L4T headers, rebuilds on every
  kernel bump, and RTL8814AU is not known for stability under sustained
  repeated scanning
- it is USB 3.0, which buys nothing here (scanning only) and works against R8
- its antenna is 180° adjustable — the articulating-antenna problem in R5

Worth a time-boxed attempt if you already own it:

```bash
sudo apt install -y dkms git linux-headers-$(uname -r)
git clone https://github.com/morrownr/8814au && cd 8814au && sudo ./install-driver.sh
iw dev && sudo iw dev wlan1 scan | grep -c '^BSS'
```

Stop after an hour if it is not scanning. If it does work, soak-test it —
scan in a loop for 30 minutes and confirm the interface does not wedge —
before trusting it to a 90-minute survey.

Test any adapter you already own before buying — it costs nothing:

```bash
lsusb                                   # 0e8d: = MediaTek (good), 0bda: = Realtek
dmesg | tail -20                        # did a driver bind?
ip -brief link                          # did an interface appear?
sudo iw dev wlan1 scan | grep -c '^BSS' # does it actually scan?
```

Prefer a **fixed, non-articulating antenna**. R5 already has RSSI varying
5–10 dB with heading; an antenna that flops as the robot walks adds a second
uncontrolled variable. Zip-tie hinged antennas in one position for the whole
survey.

---

## Phase 00 — Access and inventory

**Goal:** a shell on the robot, and a written record of what is installed.

Set a static IP on your wired adapter:

```powershell
# Windows, PowerShell as admin
Get-NetAdapter
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.123.222 -PrefixLength 24
```

```bash
ping 192.168.123.18
ssh unitree@192.168.123.18        # stock password: 123
```

**Change the password immediately.** It is `123` on every Go2 on earth, and
this is a 15 kg machine with network-reachable motor control.

| Address | What | Access |
|---|---|---|
| `192.168.123.18` | Jetson Orin — dev target | `unitree` / `123` |
| `192.168.123.161` | MCU / motion controller | no SSH |
| `192.168.123.222` | your laptop | — |

If ping fails, it is almost always Windows routing over WiFi rather than the
wired link. Turn WiFi off for the first connection.

Then capture the inventory:

```bash
bash tools/go2_survey.sh 2>&1 | tee docs/survey-$(date +%F).txt
```

Read-only; changes nothing. Commit the output — it makes robot-side changes
traceable over the life of the project.

**Set up editing from your machine** (`~/.ssh/config`):

```
Host go2
    HostName 192.168.123.18
    User unitree
```

```bash
code --remote ssh-remote+go2 /home/unitree
```

> **Exit criteria** — `ros2 topic echo /sportmodestate` streams live data, and
> `docs/survey-YYYY-MM-DD.txt` is committed.

**What the inventory decides**

| Finding | Consequence |
|---|---|
| Ubuntu 20.04 / Foxy (expected) | phase 1 goes the container route |
| Docker absent, or no NVIDIA runtime | **revisit D1** — native Foxy may be less work |
| Jetson interface name | `CYCLONEDDS_URI` — wrong value gives silent zero topics |
| Free disk < ~20 GB | cannot record raw clouds; map-only runs |

---

## Phase 01 — ROS runtime

**Goal:** a supported ROS 2 with working DDS to the robot.

The EDU ships JetPack 5.1.1 → Ubuntu 20.04 → **ROS 2 Foxy**, EOL since June
2023. Its Nav2 predates the current collision monitor and velocity smoother.
Decision **D1** is to run Humble in a container instead.

```bash
docker build -t go2-survey:humble docker/
IFACE=eth0 bash docker/run.sh
```

Inside:

```bash
ros2 topic list | head -20
ros2 topic echo /sportmodestate --once
```

**If the topic list is empty** — and this is the single most common failure on
this robot — work through these in order:

1. `CYCLONEDDS_URI` names the wrong interface. `ip -brief addr`, find the one
   holding `192.168.123.x`, re-run with `IFACE=<name>`.
2. Container not on `--network host`. DDS discovery needs multicast on the
   robot's L2 segment; bridged networking yields zero topics with no error.
3. `RMW_IMPLEMENTATION` not `rmw_cyclonedds_cpp`.
4. `ROS_DOMAIN_ID` mismatch with the robot's own DDS traffic.

The entrypoint checks 1 and 3 at startup and says so loudly.

> **Exit criteria** — `unitree_ros2` topics visible from inside the container,
> and `/sportmodestate` echoes.

---

## Phase 02 — Mid-360 bring-up

**Goal:** point cloud and IMU publishing.

### The subnet collision (D2)

The Mid-360 ships on `192.168.1.x`; the Go2 lives on `192.168.123.x`. They
cannot see each other. Alias a second address onto the Jetson rather than
reflashing the sensor — non-destructive and reversible:

```bash
sudo ip addr add 192.168.1.5/24 dev eth0
```

### Find the LiDAR's real address

Its address derives from its **serial number**, typically `192.168.1.1XX`
where `XX` is the last two digits — *not* the `192.168.1.12` placeholder in
the upstream sample config.

```bash
for i in $(seq 100 130); do ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 && echo "alive: 192.168.1.$i"; done
# nothing? catch its boot broadcast:
sudo tcpdump -i eth0 -n udp and not port 22
```

### Configure and launch

Edit two fields in [`config/MID360_config.json`](../config/MID360_config.json):
`host_net_info.*_ip` (the alias above) and `lidar_configs[0].ip` (what you just
found). **Leave the extrinsics at zero** — the mount pitch belongs in the URDF,
and setting it in both places double-applies the rotation, which looks almost
right and is completely wrong.

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 topic hz /livox/lidar     # expect ~10 Hz
ros2 topic hz /livox/imu       # expect ~200 Hz
```

Use `msg_MID360_launch.py` (Livox custom format) — FAST-LIO2 wants it. The
`rviz_` variant publishes `PointCloud2`, which is for looking at.

> **Exit criteria** — `/livox/lidar` renders a recognisable room in RViz, and
> `/livox/imu` is publishing.

---

## Phase 03 — Mount geometry and TF

**Goal:** a TF tree that matches physical reality.

🟨 *Code written: [`go2_survey_description`](../go2_survey_description/), 21
geometry tests passing.*

### Decide the angle

The Mid-360's vertical FOV is **−7° to +52°** — tilted *upward*. Mounted level
at ~0.40 m its lowest ray does not reach the floor until 3.3 m ahead.
Everything nearer is invisible. **This fails silently:** the map looks
excellent and the robot walks into things.

```bash
python tools/fov_geometry.py --height 0.42   # use YOUR measured height
```

| Pitch | Lowest ray | Blind radius | Upper FOV kept |
|---|---|---|---|
| 0° (level) | −7° | 3.26 m | 52° |
| 15° | −22° | 0.99 m | 37° |
| **22°** | **−29°** | **0.72 m** | **30°** |
| 30° | −37° | 0.53 m | 22° |

Too shallow and the blind radius exceeds what the D435i can cover; too steep
and the upper FOV stops catching the ceilings and doorframes that make indoor
LiDAR odometry stable. ~22° balances both.

### Measure, then declare

Measure every offset from the trunk origin and record it in
[`config/mount.yaml`](../config/mount.yaml). **A wrong translation is the
quietest bug in this project** — SLAM still converges, the map still looks
clean, and the costmaps sit silently offset from where the robot really is.

```bash
ros2 launch go2_survey_description description.launch.py rviz:=true \
    mount_pitch_deg:=22.0 lidar_z:=0.10
```

### Verify

Fixed frame `base_footprint`, LiDAR cloud displayed, robot in an open flat
area:

| Symptom | Cause |
|---|---|
| Floor slopes | `mount_pitch_deg` wrong |
| Floor flat but not at z=0 | `lidar_z` or `stand_height` wrong |
| Walls lean | roll/yaw in the mount, not modelled |
| Cloud rotated 90° | optical-frame rotation removed |

> **Exit criteria** — floor reads flat and at z = 0 in `base_footprint`; walls
> vertical; `ros2 run tf2_tools view_frames` shows one connected tree.

---

## Phase 04 — LiDAR-inertial SLAM

**Goal:** a metric pose and a saved map.

🟨 *Config written: [`config/fastlio_mid360.yaml`](../config/fastlio_mid360.yaml)*

```bash
ros2 launch fast_lio mapping.launch.py config_file:=fastlio_mid360.yaml
```

Three parameters matter more than the rest:

- **`blind: 0.5`** — the Mid-360's own blind sphere is 0.1 m, but on a
  quadruped the legs intrude much further. Too small and leg returns enter the
  map as moving obstacles that odometry fights. Raise it if a blob follows the
  robot.
- **`acc_cov` / `gyr_cov`** — a walking quadruped produces far more IMU noise
  than a wheeled robot. Raise these to trust the LiDAR more. First knob to
  reach for when odometry jitters with the gait cycle.
- **`extrinsic_T`** — internal to the sensor, *not* your mount. It does not
  change when you pitch the bracket. A wrong value causes slow rotational
  drift that looks like bad tuning.

### The mapping run (D3)

Teleop-drive with a human present. Do **not** attempt mapping and surveying in
one autonomous pass — that is where projects of this shape stall.

Walk a closed loop and return to the start.

```bash
ros2 run nav2_map_server map_saver_cli -f maps/floor1
```

Then **annotate glass as keep-out** in the saved PGM. Glass doors and
partitions return nothing to the LiDAR, will not appear in the map, and Nav2
will plan paths straight through them (R1). Pre-mapping gives you the one
natural opportunity to fix this by hand.

> **Exit criteria** — closed-loop walk returns with drift under ~1% of path
> length; `maps/floor1.pgm` saved and glass annotated.

---

## Phase 05 — WiFi survey node ← **the deliverable**

**Goal:** a heatmap. No autonomy required.

✅ *Written and tested: [`go2_survey_wifi`](../go2_survey_wifi/), 18 parser
tests passing.*

Confirm the scanning adapter before involving ROS:

```bash
python -m go2_survey_wifi.scanner wlan1
```

Use the **dedicated** adapter, never the one carrying your SSH session — a
full sweep channel-hops and drops association for 2–5 s (R2).

```bash
# avoid sudo per-scan:
sudo setcap cap_net_admin+ep $(which iw)

ros2 launch go2_survey_wifi survey.launch.py \
    interface:=wlan1 output_csv:=/surveys/floor1.csv min_move_m:=1.5
```

Teleop-drive the route. Rows flush as produced, so a flat battery costs the
last sample rather than the run.

```bash
python tools/heatmap.py /surveys/floor1.csv --map maps/floor1.yaml --out docs/
```

**Stop-and-scan, not continuous** (D4): a 2–5 s sweep at 0.5 m/s smears 1–2.5 m
of travel into one sample, blurring exactly the gradients being measured. In
continuous mode keep `min_move_m` generous and drive slowly; in phase 8 the
waypoint follower enforces true stops.

**Log heading** (R5): the chassis and battery attenuate signal; the same
position 180° apart routinely differs by 5–10 dB. `yaw` is in the schema for
this reason. Rotate-and-average — four scans at 90° — if the survey needs to
be defensible.

> **Exit criteria** — a per-SSID heatmap of one floor, registered to the SLAM
> map, produced entirely by teleop.

---

## Phase 06 — Velocity bridge

**Goal:** `/cmd_vel` moves the robot, and losing `/cmd_vel` stops it.

✅ *Written and tested: [`go2_survey_bridge`](../go2_survey_bridge/), 46 tests
passing.*

Goes through `unitree_ros2`'s `/api/sport/request` rather than linking
`unitree_sdk2` — no C++ build against an external SDK, and the safety logic
stays unit-testable.

```bash
ros2 launch go2_survey_bridge bridge.launch.py
```

### Verify the watchdog before trusting this to a planner

**Robot on a stand or hoist.**

```bash
ros2 service call /go2_cmd_bridge/balance_stand std_srvs/srv/Trigger
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
# Ctrl-C the publisher.
```

The robot must stop within `cmd_timeout_s` (0.3 s default). **If it does not,
do not proceed to Nav2.** Without a working watchdog, a crashed planner leaves
a 15 kg machine walking.

Also exercise `~/estop` mid-motion, and confirm `~/clear_estop` does **not**
restart it — motion resumes only on the next `/cmd_vel`, ramping from rest.

> **Exit criteria** — robot tracks `/cmd_vel`; halts within 300 ms of silence;
> e-stop latches and clears without self-starting.

---

## Phase 07 — Depth camera and costmaps

**Goal:** obstacles inside the LiDAR blind radius are visible.

```bash
ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true depth_module.profile:=640x480x15
```

**Decimate hard.** Full-resolution D435i cloud will eat an Orin that is also
running FAST-LIO2 and Nav2. 640×480×15 with voxel downsampling is plenty for
obstacle detection — this is not a reconstruction task.

Two costmap sources:

| Source | Role | How |
|---|---|---|
| Mid-360 | global structure | `pointcloud_to_laserscan` for a 2D layer |
| D435i | near-field, inside 0.72 m | `voxel_layer` on the depth cloud |

If overhangs matter, `spatio_temporal_voxel_layer` on the Mid-360 instead of
flattening — more correct, more tuning.

> **Exit criteria** — a box placed 0.5 m in front of the stationary robot
> appears in the local costmap; it clears when removed.

---

## Phase 08 — Nav2 and waypoint route

**Goal:** an unattended survey run. **The long pole — expect days of tuning,
not writing.**

### Footprint

A walking quadruped is not a circle. Set the footprint to the standing stance
plus margin, and inflate generously — legs swing outside the trunk.

### Route

Pre-mapped waypoints (D3), not live exploration:

```bash
ros2 launch nav2_bringup bringup_launch.py map:=maps/floor1.yaml
ros2 launch go2_survey_bridge bridge.launch.py
ros2 launch go2_survey_wifi survey.launch.py mode:=triggered
```

At each waypoint: arrive → settle → `~/scan_now` → resume.

```bash
ros2 service call /wifi_survey/scan_now std_srvs/srv/Trigger
```

### Make the route resumable (R6)

1–1.5 h of battery will not cover a floor. Design the route as **segments with
a persisted index**, not one continuous mission, so a battery swap resumes
rather than restarts.

### Expect to tune

| Symptom | Where to look |
|---|---|
| Oscillates at waypoints | goal tolerance, controller gains |
| Clips doorframes | footprint, inflation radius |
| Plans through glass | keep-out annotations (R1) |
| Stalls on thresholds | local costmap decay, recovery behaviours |

> **Exit criteria** — a 20-waypoint route completes unattended with zero
> interventions, **twice**, producing a CSV that renders to a heatmap.

---

## Risk register

| | Risk | Mitigation | Phase |
|---|---|---|---|
| **R1** | LiDAR sees through glass; Nav2 plans through it | annotate keep-out in saved map | 04, 08 |
| **R2** | Scanning drops your own WiFi link | second USB adapter | 05 |
| **R3** | Mount angle blinds the near field | 22° pitch, declared in URDF | 03 |
| **R4** | Scan duration smears samples | stop-and-scan at waypoints | 05, 08 |
| **R5** | RSSI depends on heading | log yaw; rotate-and-average | 05 |
| **R6** | Battery will not cover a floor | resumable route segments | 08 |
| **R7** | 0.40 m is not user height | mast, only if absolute dBm matters | — |
| **R8** | **USB 3.0 raises the 2.4 GHz noise floor ~20 dB** — the robot biases its own measurement | USB 2.0 port, physical separation, characterise before trusting data | 05, 07 |

### R8 in detail — the robot interferes with its own instrument

This one deserves expanding because it is invisible in the output. Intel's
[whitepaper](https://www.usb.org/sites/default/files/327216.pdf) measured USB 3.0
raising the 2.4 GHz noise floor by nearly 20 dB. The D435i is USB 3.0 and runs
throughout a survey, a few centimetres from the scanning antenna.

The failure mode: every 2.4 GHz reading is depressed while 5 GHz looks fine, and
the heatmap shows a dead zone that is actually the robot.

1. Put the WiFi adapter on a **USB 2.0** port.
2. Separate it physically from the RealSense and its cable — a short extension
   to the opposite end of the chassis.
3. **Characterise it** (ten minutes, do this once in phase 5):

```bash
# park the robot, do not move it between the two scans
python -m go2_survey_wifi.scanner wlan1 > /tmp/with_depth.txt
# stop the RealSense, wait ~5 s
python -m go2_survey_wifi.scanner wlan1 > /tmp/without_depth.txt
diff <(sort /tmp/with_depth.txt) <(sort /tmp/without_depth.txt)
```

Compare 2.4 GHz RSSI for the same BSSIDs. More than a couple of dB means fix
the layout before surveying — the instrument is lying.

---

## Open questions

All answered by `tools/go2_survey.sh` in phase 0.

| Question | Command | Decides |
|---|---|---|
| JetPack / Ubuntu / ROS version | `lsb_release -a` | D1 — container vs native |
| Docker + NVIDIA runtime? | `docker info` | D1 — phase 1 effort |
| Jetson interface name | `ip -brief addr` | `CYCLONEDDS_URI` |
| Mid-360's real IP | `tcpdump -n udp` | D2 — config and alias |
| librealsense installed? | `rs-enumerate-devices` | phase 7 effort |
| Free disk | `df -h /` | whether raw clouds can be recorded |
