# Bring-up log — 2026-10-20

First hardware session on the Go2 EDU. Summary of what was done, verified, and
what's still open. Demo robot; stock password left in place deliberately.

## Verified working

| Area | Result |
|---|---|
| **Direct wired link** | Laptop (TP-LINK USB NIC "Ethernet 3") at `192.168.123.222/24` → robot `eth0` `192.168.123.18`. 1 ms ping. |
| **SSH** | `ssh unitree@192.168.123.18`, port 22 open. |
| **Phase-0 inventory** | Ubuntu 20.04, kernel 5.10.104-tegra, JetPack 5.1.1 (R35.3.1), Docker 24.0.5 **+ NVIDIA runtime**, Foxy stock → Humble container (D1), `eth0`, 429 GB free. Full capture in `survey-2026-10-20.txt`. |
| **First motion** | Manual `step_forward` over SSH walked it ~0.9 m and stopped. Procedure + the vendor-SDK facts in `motion-test.md`. |
| **Mid-360 LiDAR** | Present at **`192.168.123.20`** on `eth0` (same subnet as the Jetson — **D2 aliasing NOT needed**). Ping TTL 255; powered OK. Idle until the driver starts it (expected). |
| **RealSense D435i** | Enumerates on USB (`8086:0b3a`). |
| **Internet sharing** | Robot has no WAN on the direct link; shared the laptop's Wi-Fi to it. |

## Internet sharing (laptop → robot), reproducible

The robot needs internet to build the container. Used Windows NAT (not ICS, so
the static `.222` link is preserved):

```powershell
# laptop, admin PowerShell
Set-NetIPInterface -InterfaceAlias "Ethernet 3" -Forwarding Enabled
Set-NetIPInterface -InterfaceAlias "Wi-Fi"       -Forwarding Enabled
New-NetNat -Name Go2Nat -InternalIPInterfaceAddressPrefix 192.168.123.0/24
```
```bash
# robot
sudo ip route replace default via 192.168.123.222 dev eth0
# DNS already worked; if not: echo "nameserver 8.8.8.8" | sudo tee -a /etc/resolv.conf
```
Teardown: `Remove-NetNat -Name Go2Nat -Confirm:$false` + set both Forwarding
Disabled. The robot's route clears on reboot. Internal `123.x` comms (SSH,
LiDAR, MCU) are untouched throughout — only an outbound default route is added.

## Container build — four fixes (this commit)

The pre-hardware Dockerfile failed on first real build. Root causes and fixes,
all in `docker/Dockerfile`:

1. **FAST-LIO default branch is ROS 1 only** (`find_package(catkin)`, fails on
   Humble) → clone `--branch ROS2` (ament-based).
2. **fast_lio needs the livox driver's install sourced** before configure
   (`find_package(livox_ros_driver2 REQUIRED)`) → add `source install/setup.bash`.
3. **SDK/driver version mismatch**: SDK2 pinned `v1.2.5` but the driver's `main`
   used newer/unreleased symbols → pin the driver to the **1.2.6 release** and
   bump **SDK2 to v1.3.1** (matched pair; verified `kLivoxLidarTypeMid360s`
   exists in v1.3.1 and 1.2.6 does not reference the unreleased double-echo
   symbols).

Build was re-running at end of session; these fixes are committed as the
known-correct state.

## Robot-local config (NOT committed — robot-specific)

On the robot's clone, `config/` was edited for this unit (the repo keeps loud
placeholders):

- `MID360_config.json`: host IPs → `192.168.123.18`, `lidar_configs[0].ip` →
  `192.168.123.20`. No subnet alias (same subnet).
- `fastlio_mid360.yaml`: `pcd_save_en: true` for the mapping run.

## Mount

Mid-360 mounted **nose-down** (correct direction — avoids the silent near-field
blind zone). Exact pitch still to be measured and declared in the URDF (phase
3); not required for raw SLAM, since FAST-LIO2 gravity-aligns via the Mid-360
IMU.

## Next

1. Finish the container build (verify `go2-survey:humble` image).
2. Start container (`IFACE=eth0 bash docker/run.sh`), launch the Livox driver,
   confirm `/livox/lidar` ~10 Hz and `/livox/imu` ~200 Hz.
3. FAST-LIO2 mapping (`ros2 launch fast_lio mapping.launch.py
   config_file:=fastlio_mid360.yaml`), teleop-drive the room with the remote
   (**collision avoidance on**), save the PCD. Survey (phases 5+) skipped — this
   is a map-only goal for now.
