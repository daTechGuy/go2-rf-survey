# Related work: what exists and what we took from it

Searched 2026-10-26. Robots that make WiFi heatmaps exist, but every one
found is wheeled and measures **one** connected network. None was found on a
legged robot, or surveying **every** access point in range, as this repo does.

## WiFi survey robots

| Project | What it is | What we take / might take |
|---|---|---|
| [wimblerobotics/wifi_logger_visualizer](https://github.com/wimblerobotics/wifi_logger_visualizer) | ROS 2, MIT, active (rewrite 2026-10-24). `iw` logging, RViz + browser views, SQLite. | **The best source of survey ideas** (not yet adopted): record only while localisation is fresh; active tests only after the robot has been still (4 cm / 3 s); per-cell min/max/last with **no interpolation**, the same rule as our `--max-reach` mask; failed tests stored as failures, not zeros; BSSID → room names from a config file; key the dataset by a hash of the map. |
| [husarion/rosbot-demo-wifi-heatmap](https://github.com/husarion/rosbot-demo-wifi-heatmap) | ROS 2 + Nav2 on a wheeled ROSbot, Apache-2.0. Map, then waypoints with 10 s of RSSI each. | Rejects waypoints too close to obstacles: the same problem as our clearance ring. Reads `/proc/net/wireless`, so only the connected AP; we scan all. |
| [ROS 1 `heatmap`](http://wiki.ros.org/heatmap) | Coverage path over a saved map | Generating survey waypoints automatically (phase 8). |
| [herolab-uga/indoor-rssi-mobile-robot](https://github.com/topics/wifi-signal-strength) | Indoor RSSI recorded by mobile robots | Real data to test `tools/heatmap.py` on before ours exists. |

## Go2 navigation

| Project | Notes |
|---|---|
| **[dancher00/go2-nav2-wifi](https://github.com/dancher00/go2-nav2-wifi)** | Go2 Edu, stock LiDAR, Humble in Docker, SLAM Toolbox + Nav2, MIT. **Pulled from**: see below. |
| [abizovnuralem/go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) | CycloneDDS/EDU, SLAM Toolbox + Nav2, Foxglove. The planned route to real navigation. |
| [Unitree-Go2-Physical-AI-Integration/go2-slam-navigation](https://github.com/Unitree-Go2-Physical-AI-Integration/go2-slam-navigation) | SLAM + navigation reference. |
| BotBrain | Web UI and patrol missions; needs JetPack 6.2 (this robot has 5.1.1). |

### What we took from go2-nav2-wifi (commit `4a8a8b9`, 2026-09-27)

1. **Walk preparation before sport Move**, now the bridge's opt-in `sport_prep`
   backend (`go2_survey_bridge/motion_backend.py`). Their bridge sends
   `BalanceStand → RecoveryStand → SwitchJoystick(false) → FreeWalk` with the
   note *"required after RC use"*, and their troubleshooting script says *"If
   hz>0 but robot jerks: … (FreeWalk + SwitchJoystick)"*. That is the
   symptom of our first Stage 1 run. We leave out RecoveryStand (it can
   stand the robot up), and unlike theirs we **give the sticks back**
   (`SwitchJoystick(true)`) on every release. Gate before use:
   `go2loop.sh joystick hold|release` (no motion). See
   [autonomous-loop.md](autonomous-loop.md#optional-walk-prep-backendsport_prep).
2. **Wireless, later.** Their robot is on WiFi (a phone hotspot); a relay
   copies the built-in LiDAR, odometry, IMU and camera from the robot's
   internal DDS (domain 0, never reconfigured) to a separate WiFi DDS domain
   (64), because their RViz and Nav2 run on the laptop. **We don't need the
   relay for the live view**: Foxglove talks to `foxglove_bridge` over a
   websocket, which already listens on every interface. Once the Jetson has
   WiFi (a USB MT7612U), Foxglove connects to `ws://<robot-wifi-ip>:8765`. See
   [live-view.md](live-view.md#without-the-cable-wifi). Their relay is the
   model if we ever run ROS tools on the laptop.
3. **Lessons we already follow, now confirmed:** compute on the Jetson, send
   only visualisation over WiFi (their `lidar3d-map` profile; our
   `live_map.py`). Never compare laptop and robot timestamps: their
   [navigation incident](https://github.com/dancher00/go2-nav2-wifi/blob/main/docs/NAVIGATION-INCIDENT-2026-09-12.md)
   traced a bad turn to a 5 s clock skew between laptop and Jetson. Our loop
   node uses receipt time, on the robot, for exactly this reason.
4. **Not yet taken, worth it:** the front camera into Foxglove
   (`robot_front_camera_bridge.py`, via unitree_sdk2's VideoClient); their
   measured footprint (`docs/GO2-FOOTPRINT.md`) to check our 0.55 m clearance.

## Go2 security (cyber-education material)

| | |
|---|---|
| **[UniPwn](https://github.com/Bin4ry/UniPwn)** | The BLE WiFi-setup interface on Go2, B2, G1 and H1 gives root (CVE-2025-35027). Hard-coded keys, since published. Described as wormable over BLE. Unitree acknowledged it 2025-09-29. Coverage: [IEEE Spectrum](https://spectrum.ieee.org/unitree-robot-exploit). |
| **CloudSail** ([CVE-2025-2894](https://tenable.com/cve/CVE-2025-2894)) | A remote-access tunnel shipped on the Go1; ~1,900 units seen connected, including universities. |
| [39C3 "Skynet Starter Kit"](https://api.media.ccc.de/v/39c3-skynet-starter-kit-from-embodied-ai-jailbreak-to-remote-takeover-of-humanoid-robots) | Hijacking Unitree robots over BLE, LoRa and local WiFi. |

These tie the projects together: the BLE census (go2-bl-survey) would see the
Go2 advertising the very interface UniPwn attacks. A read-only firmware
version check on this robot is a sensible first step.
