# Build plan

Full version with scale diagrams (LiDAR FOV geometry, data flow):
**https://claude.ai/artifact/SR2QY8zsuDdAiaqEbs2SBy**

This file holds the derivations and decision records that don't belong in the
README.

---

## Mount angle derivation

The Mid-360's vertical FOV is −7° to +52°, asymmetric and biased upward. For a
sensor at height `h` with the housing pitched `p` degrees nose-down, the lowest
ray leaves at `−(p + 7)` degrees from horizontal and meets the floor at:

```
d = h / tan(p + 7°)
```

Everything inside `d` is unobserved by the LiDAR. At `h = 0.40 m`:

| `p` | lowest ray | `d` | upper FOV kept |
|---|---|---|---|
| 0° | −7° | 3.26 m | 52° |
| 10° | −17° | 1.31 m | 42° |
| 15° | −22° | 0.99 m | 37° |
| **22°** | **−29°** | **0.72 m** | **30°** |
| 25° | −32° | 0.64 m | 27° |
| 30° | −37° | 0.53 m | 22° |

Measure `h` on the actual mount before trusting any row of this.

**Why ~22°:** below ~15° the blind radius exceeds one body length, which is too
far to cover with the D435i alone. Above ~30° the upper FOV drops under ~22°,
which starts to cost SLAM the ceiling and doorframe features that make indoor
LiDAR odometry stable. 22° keeps the blind radius inside the depth camera's
useful range while retaining 30° of upward reach.

**Where the angle is declared:** the URDF, as a fixed joint between `base_link`
and the LiDAR frame. *Not* in `MID360_config.json`'s extrinsic fields. Two
reasons: the TF tree is what FAST-LIO2, both costmaps, and RViz all read, so
there is exactly one source of truth; and gravity alignment in the IMU
preintegration depends on it being right in the same frame SLAM uses.

---

## Decision record

### D1 — Humble in a container, not native Foxy

The EDU ships JetPack 5.1.1 → Ubuntu 20.04 → ROS 2 Foxy. `unitree_ros2` supports
Foxy on 20.04 and Humble on 22.04, and marks Humble recommended.

Foxy has been end-of-life since June 2023. Its Nav2 predates the current
collision monitor, the velocity smoother, and much of the 3D costmap tooling.
Foxy also requires hand-building CycloneDDS 0.10.2 in a shell where ROS has
*not* been sourced; Humble ships it.

Rejected alternatives:
- **Native Foxy** — building the navigation half of this project on an EOL branch.
- **JetPack 6.x upgrade** → native Humble on 22.04. Technically possible but
  risks the stock Unitree stack and the warranty position. Not for a first build.
- **Nav2 off-robot on a laptop** — fine for bring-up and RViz, but WiFi-dependent
  autonomy isn't autonomy. Keep as a development aid only.

Chosen: Humble in `--network host` Docker on the Jetson. Modern Nav2, everything
onboard, stock software untouched.

*Revisit if:* phase 0 shows Docker absent or without the NVIDIA runtime, which
would make this materially more work than native Foxy.

### D2 — Alias a second subnet, don't reflash the LiDAR

The Mid-360 ships on `192.168.1.x`; the Go2's internal network is
`192.168.123.x`. They cannot see each other.

```bash
sudo ip addr add 192.168.1.5/24 dev <iface>   # must match host_ip in MID360_config.json
```

Chosen over reconfiguring the sensor's IP into the 123 subnet because it is
non-destructive and reversible, and keeps the LiDAR at its factory address if it
ever needs to be tested on another machine.

Note the Mid-360's actual address derives from its **serial number** — typically
`192.168.1.1XX` where `XX` is the last two digits — not the `192.168.1.12`
placeholder in the sample config. Find it with `tcpdump -n udp` during sensor
boot if a ping sweep doesn't locate it.

### D3 — Pre-mapped waypoints, not live exploration

Two-phase: teleop-drive the map with a human present, save it, then survey
against known geometry with `nav2_waypoint_follower`.

Rejected: frontier exploration (`m-explore-ros2`). Mapping and surveying in one
autonomous pass compounds two hard problems, and it is the specific failure mode
that kills projects of this shape. Pre-mapping also gives a natural place to
annotate glass keep-out zones (R1), which has no equivalent in live exploration.

### D4 — Stop-and-scan, not continuous

A full dual-band `iw scan` takes 2–5 s. At 0.5 m/s that smears 1–2.5 m of travel
into a single sample, blurring exactly the gradients being measured.

Chosen: halt at each waypoint, scan, resume. Slower in wall-clock; the data is
what the project is for. This is also what commercial site-survey tools do.

Corollary: timestamp samples at scan **completion**, not request. While
stationary the two agree — but they diverge the moment anyone tries scanning on
the move, and a silently wrong timestamp is worse than a slow survey.

---

## Open questions

Blocking phase 1 onward. Answer with `tools/go2_survey.sh`.

| Question | Command | Decides |
|---|---|---|
| JetPack / Ubuntu / ROS version | `lsb_release -a` | D1 — confirms or overturns the container choice |
| Docker + NVIDIA runtime present? | `docker info` | D1 — effort estimate for phase 1 |
| Jetson interface name | `ip -brief addr` | `CYCLONEDDS_URI`; wrong value → silent zero topics |
| Mid-360's real IP | `tcpdump -n udp` | D2 — `MID360_config.json` and the alias |
| librealsense installed? | `rs-enumerate-devices` | Phase 7 effort |
| Free disk | `df -h /` | Whether raw clouds can be recorded at all |

## Phase 0 survey output

Record each run as `docs/survey-YYYY-MM-DD.txt` so changes to the robot's
software are traceable over the life of the project.
