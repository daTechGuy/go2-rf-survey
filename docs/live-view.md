# Live view on the laptop (Foxglove)

See what the robot sees, live: its LiDAR, the room map growing as it walks,
its trail, and the autonomous loop's status. Works on Windows over the
direct wired link — no NoMachine, no X11.

```
robot container                                laptop
foxglove_bridge  ── ws://192.168.123.18:8765 ──▶  Foxglove app
live_map.py  (map, trail, pose)
```

Both robot-side pieces only **listen** to the robot and publish
visualisation topics. They command nothing.

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

5. Optional: a **Raw Messages** panel on `/go2_loop/status` shows the
   autonomous loop's state (`running | waypoint 1/2 | ...`, pauses and why).

Save the layout (Layouts → Save) so it reopens next time.

## Bandwidth

The robot's internal link is 100 Mbit. The raw `/utlidar/cloud_deskewed` is
several MB/s, so the live map republishes at a coarser 8 cm voxel every 2 s
and the live scan is thinned to every 4th point at 5 Hz. Subscribing the
raw cloud in Foxglove works but is heavy — prefer `/go2/scan_lite`.

## For the classroom demo

`viz-reset` just before the robot starts, then the audience watches the room
appear on the screen as it walks. The same run can be recorded and turned
into a floorplan PNG afterwards (`tools/bag_to_map.py`).
