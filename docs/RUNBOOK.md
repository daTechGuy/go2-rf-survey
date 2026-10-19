# Runbook

What to type, in order, to go from a fresh clone to a WiFi heatmap.

This is the operational guide. For *why* each choice was made, see
[`BUILD_PLAN.md`](BUILD_PLAN.md) and [`plan.md`](plan.md).

| Part | Needs | Time |
|---|---|---|
| [0 — Right now, no robot](#part-0--right-now-no-robot) | any machine with Python | 5 min |
| [1 — First contact](#part-1--first-contact-with-the-robot) | robot + ethernet | 30 min |
| [2 — Container](#part-2--build-the-container) | part 1 | 1 h (mostly build) |
| [3 — LiDAR](#part-3--bring-up-the-lidar) | Mid-360 wired + powered | 1 h |
| [4 — Mount and TF](#part-4--mount-geometry-and-tf) | LiDAR publishing | 30 min |
| [5 — Map a floor](#part-5--map-a-floor) | part 4 | 1 h |
| [6 — Run a survey](#part-6--run-a-survey--the-payoff) | map + WiFi adapter | 1 h |
| [7 — Autonomous](#part-7--autonomous-run) | everything above | days |

Jump to [Routine survey](#routine-survey) once set up, or
[Troubleshooting](#troubleshooting) when something is wrong.

---

## Part 0 — Right now, no robot

Verified on Windows 11 with Python 3.12 from a clean clone. Works on Linux and
macOS too.

### Clone and run the tests

```powershell
git clone https://github.com/daTechGuy/go2-rf-survey.git
cd go2-rf-survey
python tools/run_tests.py
```

```
85 tests passed across 4 suites
```

No dependencies needed — stdlib only, deliberately, so this works on a bare
Python before anything is installed.

### Produce a heatmap from synthetic data

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools/requirements.txt
```

<details>
<summary>macOS / Linux</summary>

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r tools/requirements.txt
```
Substitute `.venv/bin/python` for `.venv\Scripts\python.exe` below.
</details>

```powershell
.\.venv\Scripts\python.exe tools/make_synthetic_survey.py --out docs/demo
.\.venv\Scripts\python.exe tools/heatmap.py docs/demo.csv --map docs/demo.yaml --out docs/
```

Open `docs/demo-heatmap.png`. That is the shape of the final deliverable —
generated from a modelled building with known AP positions, so the rendering
is checkable against ground truth before any real data exists.

### Work out your mount angle

```powershell
python tools/fov_geometry.py --height 0.42
```

Use your **measured** sensor height. It prints the blind radius for each pitch
and recommends one. Needed in part 4, before you drill a bracket.

---

## Part 1 — First contact with the robot

### Wire up

Ethernet from the robot's rear port to your laptop. You need a USB-Ethernet
adapter if your machine has no wired port.

**Turn WiFi off.** Windows routing over wireless instead of the wired link is
the most common reason the next step fails.

### Static IP

```powershell
# PowerShell as administrator
Get-NetAdapter                       # note the InterfaceAlias
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.123.222 -PrefixLength 24
```

<details>
<summary>Undo this later</summary>

```powershell
Remove-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.123.222 -Confirm:$false
Set-NetIPInterface -InterfaceAlias "Ethernet" -Dhcp Enabled
```
</details>

### Connect

```powershell
ping 192.168.123.18
ssh unitree@192.168.123.18           # stock password: 123
```

**Change the password now.** It is `123` on every Go2 on earth, and this is a
15 kg machine with network-reachable motor control.

```bash
passwd
```

### Inventory the robot

```bash
git clone https://github.com/daTechGuy/go2-rf-survey.git ~/go2-rf-survey
cd ~/go2-rf-survey
bash tools/go2_survey.sh 2>&1 | tee docs/survey-$(date +%F).txt
```

Read-only; changes nothing. Keep the output — it records what the robot had
before you started changing it. Three things from it are needed below:

| From the output | Used in |
|---|---|
| ROS distro and Ubuntu version | part 2 |
| Interface holding `192.168.123.x` (likely `eth0`) | part 2 — `IFACE` |
| Whether Docker is installed | part 2 |

### Edit from your laptop

Add to `~/.ssh/config` on your machine:

```
Host go2
    HostName 192.168.123.18
    User unitree
```

```powershell
code --remote ssh-remote+go2 /home/unitree
```

---

## Part 2 — Build the container

The EDU ships ROS 2 Foxy, EOL since June 2023. We run Humble in a container
instead ([D1](plan.md)).

```bash
cd ~/go2-rf-survey
docker build -t go2-survey:humble docker/
```

Twenty to forty minutes on an Orin — it compiles Livox-SDK2 and FAST-LIO2.

### Start it

```bash
IFACE=eth0 bash docker/run.sh
```

Use the interface name from part 1. **A wrong value here is the single biggest
time sink on this robot**: the build succeeds, nothing errors, and
`ros2 topic list` is simply empty. The entrypoint checks it and complains.

### Verify DDS

```bash
ros2 topic list | head
ros2 topic echo /sportmodestate --once
```

Both must produce output before continuing. If the list is empty, see
[Troubleshooting](#empty-topic-list).

### Build this repo's packages

Inside the container:

```bash
cd /workspace
source /unitree_ros2/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Part 3 — Bring up the LiDAR

### Resolve the subnet collision

The Mid-360 is on `192.168.1.x`; the robot on `192.168.123.x`. Alias a second
address rather than reflashing the sensor ([D2](plan.md)):

```bash
sudo ip addr add 192.168.1.5/24 dev eth0
```

### Find the LiDAR's real address

It derives from the **serial number**, usually `192.168.1.1XX` where `XX` is
the last two digits — *not* the `192.168.1.12` in the sample config.

```bash
for i in $(seq 100 130); do
  ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 && echo "alive: 192.168.1.$i"
done
```

Nothing? Catch its boot broadcast, then power-cycle the LiDAR:

```bash
sudo tcpdump -i eth0 -n udp and not port 22
```

### Configure

Edit two fields in [`config/MID360_config.json`](../config/MID360_config.json):

| Field | Value |
|---|---|
| `MID360.host_net_info.*_ip` | `192.168.1.5` (the alias above) |
| `lidar_configs[0].ip` | what you just found |

**Leave the extrinsics at zero.** The mount pitch goes in the URDF — setting
it in both places double-applies the rotation, which looks almost right and is
completely wrong.

### Launch

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

```bash
ros2 topic hz /livox/lidar     # ~10 Hz
ros2 topic hz /livox/imu       # ~200 Hz
```

Use `msg_MID360_launch.py`, not the `rviz_` variant — FAST-LIO2 wants the
Livox custom format.

---

## Part 4 — Mount geometry and TF

### Set the angle

```bash
python3 tools/fov_geometry.py --height 0.42
```

Pitch the LiDAR **nose-down** by the recommended angle (~22° at 0.40 m).
Mounted level it cannot see the floor within 3.3 m, and **this fails
silently** — the map looks excellent and the robot walks into things.

### Measure and record

Measure every offset from the trunk origin into
[`config/mount.yaml`](../config/mount.yaml). A wrong translation is the
quietest bug in this project: SLAM still converges, the map still looks clean,
and the costmaps sit silently offset from reality.

### Publish and check

```bash
ros2 launch go2_survey_description description.launch.py rviz:=true \
    mount_pitch_deg:=22.0 lidar_z:=0.10
```

Stand the robot in an open flat area. Fixed frame `base_footprint`, LiDAR
cloud shown:

| What you see | What is wrong |
|---|---|
| Floor slopes | `mount_pitch_deg` |
| Floor flat, not at z=0 | `lidar_z` or `stand_height` |
| Walls lean | unmodelled roll/yaw in the bracket |
| Cloud rotated 90° | optical-frame rotation removed |

```bash
ros2 run tf2_tools view_frames      # must be ONE connected tree
```

---

## Part 5 — Map a floor

```bash
ros2 launch fast_lio mapping.launch.py config_file:=fastlio_mid360.yaml
```

Teleop-drive it. **Do not try to map and survey in one autonomous pass** —
that is where projects of this shape stall ([D3](plan.md)).

Walk a closed loop and return to the start. Drift should be under ~1% of path
length. If worse, see [Troubleshooting](#slam-drifts).

```bash
ros2 run nav2_map_server map_saver_cli -f maps/floor1
```

### Annotate glass — do not skip this

Glass doors and partitions return nothing to the LiDAR. They will not be in
the map, and Nav2 will plan straight through them (**R1**). Open
`maps/floor1.pgm` in any image editor and paint glass as black (occupied).

This is the only natural opportunity to fix it, and it is why we pre-map.

---

## Part 6 — Run a survey — the payoff

**No autonomy needed.** Teleop produces a real heatmap.

### Check the adapter

Use the **dedicated** scanning adapter, never the one carrying your SSH
session — a full sweep drops association for 2–5 s (**R2**).

```bash
sudo setcap cap_net_admin+ep $(which iw)    # avoids sudo per scan
python3 -m go2_survey_wifi.scanner wlan1
```

You should get a table of access points. If not, see
[Troubleshooting](#wifi-adapter-will-not-scan).

### Characterise USB 3.0 interference — once

The D435i is USB 3.0 and raises the 2.4 GHz noise floor ~20 dB (**R8**). Park
the robot and scan twice from the identical spot:

```bash
python3 -m go2_survey_wifi.scanner wlan1 > /tmp/with_depth.txt
# stop the RealSense, wait 5 s
python3 -m go2_survey_wifi.scanner wlan1 > /tmp/without_depth.txt
diff <(sort /tmp/with_depth.txt) <(sort /tmp/without_depth.txt)
```

More than a couple of dB on 2.4 GHz and the layout needs fixing before you
collect anything. Put the adapter on USB 2.0 and move it away from the camera.

### Survey

Three terminals, or `tmux`:

```bash
# 1 — localisation
ros2 launch fast_lio mapping.launch.py config_file:=fastlio_mid360.yaml

# 2 — survey node
ros2 launch go2_survey_wifi survey.launch.py \
    interface:=wlan1 output_csv:=/surveys/floor1.csv min_move_m:=1.5

# 3 — watch it work
ros2 topic echo /wifi_survey/status
```

Teleop-drive the route. Drive **slowly** — a 2–5 s sweep at 0.5 m/s smears
1–2.5 m of travel into one sample (**R4**). Pause at each sample point.

Rows flush as produced, so a flat battery costs the last sample, not the run.

### Render

```bash
python3 tools/heatmap.py /surveys/floor1.csv --map maps/floor1.yaml --out out/
python3 tools/heatmap.py /surveys/floor1.csv --map maps/floor1.yaml --band 5 --out out/
```

**You now have the deliverable.** Everything beyond this point is about doing
it unattended.

---

## Part 7 — Autonomous run

### Prove the watchdog first

**Robot on a stand or hoist.** Non-negotiable.

```bash
ros2 launch go2_survey_bridge bridge.launch.py
ros2 service call /go2_cmd_bridge/balance_stand std_srvs/srv/Trigger
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
# Ctrl-C the publisher
```

The robot must stop within 0.3 s. **If it does not, stop here.** Without a
working watchdog a crashed planner leaves a 15 kg machine walking.

Also check the e-stop latches and that clearing it does *not* restart motion:

```bash
ros2 service call /go2_cmd_bridge/estop std_srvs/srv/Trigger
ros2 service call /go2_cmd_bridge/clear_estop std_srvs/srv/Trigger
```

### Depth camera

```bash
ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true depth_module.profile:=640x480x15
```

Decimate hard — full resolution will eat an Orin already running FAST-LIO2
and Nav2.

### Nav2

```bash
ros2 launch nav2_bringup bringup_launch.py map:=maps/floor1.yaml
ros2 launch go2_survey_bridge bridge.launch.py
ros2 launch go2_survey_wifi survey.launch.py mode:=triggered
```

At each waypoint: arrive → settle → scan → resume.

```bash
ros2 service call /wifi_survey/scan_now std_srvs/srv/Trigger
```

Make the route **resumable segments with a persisted index** — 1–1.5 h of
battery will not cover a floor (**R6**).

---

## Routine survey

Once everything above is done, a survey is:

```bash
# on the robot
IFACE=eth0 bash ~/go2-rf-survey/docker/run.sh
```

```bash
# inside the container
source /workspace/install/setup.bash
sudo ip addr add 192.168.1.5/24 dev eth0        # after every reboot

ros2 launch livox_ros_driver2 msg_MID360_launch.py &
ros2 launch go2_survey_description description.launch.py &
ros2 launch fast_lio mapping.launch.py config_file:=fastlio_mid360.yaml &
ros2 launch go2_survey_wifi survey.launch.py \
    interface:=wlan1 output_csv:=/surveys/$(date +%F)-floor1.csv
```

```bash
# off-robot, afterwards
scp go2:surveys/*.csv .
python tools/heatmap.py 2026-10-19-floor1.csv --map maps/floor1.yaml --out out/
```

---

## Troubleshooting

### Empty topic list

In order of likelihood:

1. `CYCLONEDDS_URI` names the wrong interface. `ip -brief addr`, find the one
   with `192.168.123.x`, re-run with `IFACE=<name>`.
2. Container not on `--network host`. DDS discovery needs multicast on the
   robot's L2 segment; bridged networking gives zero topics with no error.
3. `RMW_IMPLEMENTATION` is not `rmw_cyclonedds_cpp`.
4. `ROS_DOMAIN_ID` mismatch.

### Cannot ping 192.168.123.18

Windows is routing over WiFi. Turn WiFi off. Then confirm the static IP
applied with `Get-NetIPAddress -InterfaceAlias "Ethernet"`.

### LiDAR publishes nothing

- Did you alias `192.168.1.5`? It is lost on every reboot.
- Is `lidar_configs[0].ip` the real address, not the `.12` placeholder?
- `sudo tcpdump -i eth0 -n udp` — is the sensor transmitting at all?
- Check power. The Mid-360 needs 9–27 V, not USB.

### SLAM drifts

1. Raise `acc_cov` / `gyr_cov` in `fastlio_mid360.yaml`. A walking quadruped
   is far noisier than a wheeled robot; this is the first knob.
2. Raise `blind` to 0.6–0.7 if a blob follows the robot — legs entering the
   map as moving obstacles.
3. Verify `extrinsic_T`. It is internal to the sensor and does **not** change
   when you pitch the bracket.
4. Featureless corridors defeat LiDAR odometry. Check it is not purely a
   long blank hallway.

### WiFi adapter will not scan

```bash
lsusb                  # 0e8d: MediaTek (good), 0bda: Realtek (likely trouble)
dmesg | tail -20       # did a driver bind?
ip -brief link         # did an interface appear?
```

`iw` exists but errors with *Operation not permitted*:

```bash
sudo setcap cap_net_admin+ep $(which iw)
```

No interface at all means no driver. See
[adapter selection](BUILD_PLAN.md#choosing-the-scanning-adapter) — on the
Jetson's 5.10 kernel you want **MT7612U**, and Realtek RTL88xx needs DKMS.

### Heatmap has large blank areas

Working as designed: cells further than `--max-reach` from a real sample are
left blank rather than invented. Either drive more of the floor, or raise it:

```bash
python tools/heatmap.py survey.csv --map map.yaml --max-reach 5.0 --out out/
```

Be careful — raising it means claiming coverage you did not measure.

### Robot does not respond to /cmd_vel

1. `BalanceStand` must be sent first; Move is ignored without it.
2. Check `/wifi_survey/status` and the bridge log for a latched e-stop.
3. Confirm the bridge is publishing: `ros2 topic hz /api/sport/request`.
4. Is sport mode still running on the robot? Low-level control disables it.
