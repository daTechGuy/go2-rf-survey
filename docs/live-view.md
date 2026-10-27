# Live view on the laptop (Foxglove)

See what the robot sees, live: its LiDAR, the room map growing as it walks,
its trail, and the autonomous loop's status. Works on Windows over the
direct wired link — no NoMachine, no X11.

```
robot container                                laptop
foxglove_bridge  ── ws://192.168.123.18:8765 ──▶  Foxglove app
live_map.py  (map, trail, pose)
front_camera.py (what the robot sees)
```

The robot-side pieces only **listen** to the robot, or ask its camera for
pictures, and publish visualisation topics. They command nothing.

## Robot side

```bash
bash tools/go2loop.sh viz-up      # start
bash tools/go2loop.sh viz-reset   # clear the map/trail (e.g. before a demo run)
bash tools/go2loop.sh viz-down    # stop
```

Needs an image built after the live-view change (`bash tools/go2loop.sh
preflight` says if a rebuild is due; `bash tools/go2loop.sh build`).
Logs: `~/surveys/go2viz.log`, `~/surveys/go2viz_bridge.log`.

## Laptop side (once)

1. Install **Foxglove** for Windows from <https://foxglove.dev/download>.
   (The desktop app is the reliable choice: browsers may block the web app
   from opening a plain `ws://` connection to the robot.)
2. **Open connection → Foxglove WebSocket →** `ws://192.168.123.18:8765`.
3. Add a **3D** panel and set its **display frame to `odom`** — every topic
   below is already in that frame, so no TF is needed.
4. In the 3D panel, turn on:

   | Topic | What it is | Suggested setting |
   |---|---|---|
   | `/go2/map_cloud` | the room, accumulated (refreshes every 2 s) | colour by `z` |
   | `/go2/scan_lite` | the live scan, thinned (5 Hz) | small points |
   | `/go2/path` | where it has walked | line |
   | `/go2/pose` | where it is now | arrow |

5. Add an **Image** panel on `/go2/camera/compressed`: the robot's front
   camera, JPEG straight from the robot (5 fps; `CAMERA_FPS=2 bash tools/go2loop.sh viz-up`
   for less). Check it once, read-only, with `bash tools/go2loop.sh camera-check`,
   which saves one picture to `~/surveys/camera_test.jpg`. If the camera
   service isn't found, the map still works; see `~/surveys/go2viz_camera.log`.
6. Optional: a **Raw Messages** panel on `/go2_loop/status` shows the
   autonomous loop's state (`running | waypoint 1/2 | ...`, pauses and why).

Save the layout (Layouts → Save) so it reopens next time.

## Bandwidth

The robot's internal link is 100 Mbit. The raw `/utlidar/cloud_deskewed` is
several MB/s, so the live map republishes at a coarser 8 cm voxel every 2 s
and the live scan is thinned to every 4th point at 5 Hz. Subscribing the
raw cloud in Foxglove works but is heavy — prefer `/go2/scan_lite`.

## Without the cable (WiFi)

Not set up yet. The robot's Jetson has no WiFi of its own (only `eth0` was
found). With a USB WiFi adapter on the Jetson (MT7612U: see
[HARDWARE.md](HARDWARE.md); the 5.10 kernel rules most others out) joined to the
same network as the laptop, **nothing else changes**: `foxglove_bridge`
already listens on every interface, so Foxglove connects to
`ws://<robot-wifi-ip>:8765`.

No DDS relay is needed for this. [dancher00/go2-nav2-wifi](https://github.com/dancher00/go2-nav2-wifi)
does relay the robot's topics onto a separate WiFi DDS domain (64), never
touching the onboard domain 0, but only because their RViz and Nav2 run on the
laptop. Keep the heavy work on the robot and send only the thinned map, as
`live_map.py` does: one of their logs shows ~0.9 Mbit/s of Jetson WiFi
traffic. Notes: [related-work.md](related-work.md).

## For the classroom demo

`viz-reset` just before the robot starts, then the audience watches the room
appear on the screen as it walks. The same run can be recorded and turned
into a floorplan PNG afterwards (`tools/bag_to_map.py`).
