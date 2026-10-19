#!/usr/bin/env bash
# Go2 EDU onboard survey — read-only, changes nothing.
# Run on the robot's Jetson: bash go2_survey.sh 2>&1 | tee go2_survey.txt

echo "===== OS / KERNEL / ARCH ====="
lsb_release -a 2>/dev/null
uname -a
cat /etc/nv_tegra_release 2>/dev/null || echo "(no nv_tegra_release -- not a Jetson?)"

echo; echo "===== COMPUTE ====="
nproc
free -h
df -h /
nvidia-smi 2>/dev/null || tegrastats --interval 1000 2>/dev/null & sleep 2; kill %1 2>/dev/null

echo; echo "===== ROS 2 ====="
ls /opt/ros/ 2>/dev/null || echo "(no /opt/ros -- ROS 2 not installed system-wide)"
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"

echo; echo "--- nav/slam/realsense packages already present ---"
for d in /opt/ros/*/; do
  [ -d "$d/share" ] || continue
  ls "$d/share" 2>/dev/null | grep -iE 'nav2|slam|cyclonedds|realsense|pointcloud_to_laserscan|robot_localization|voxel' \
    | sed "s|^|  $(basename $d): |"
done

echo; echo "===== DOCKER ====="
docker --version 2>/dev/null || echo "(docker not installed)"
docker info 2>/dev/null | grep -iE 'runtime|server version' || true

echo; echo "===== NETWORK ====="
ip -brief addr
echo "--- default route ---"
ip route | head -5

echo; echo "===== WHAT'S ON THE 123 SUBNET ====="
command -v nmap >/dev/null && nmap -sn 192.168.123.0/24 2>/dev/null | grep -E 'Nmap scan|MAC' \
  || for i in 10 15 18 161 220 222; do ping -c1 -W1 192.168.123.$i >/dev/null 2>&1 && echo "  alive: 192.168.123.$i"; done

echo; echo "===== USB DEVICES (identify the D435i) ====="
lsusb
echo "--- video nodes ---"
ls -l /dev/video* 2>/dev/null || echo "(none)"
command -v v4l2-ctl >/dev/null && for v in /dev/video*; do echo "  $v: $(v4l2-ctl -d $v --info 2>/dev/null | grep -i 'card type')"; done

echo; echo "===== LIVOX MID-360 (Ethernet device -- NOT on USB) ====="
echo "--- is a 192.168.1.x address configured anywhere? ---"
ip -brief addr | grep -E '192\.168\.1\.' || echo "  NO 192.168.1.x address -- host cannot reach a default-config Mid-360"
echo "--- probe the Livox default subnet ---"
for i in 5 12 100 101 102 103 104 105 106 107 108 109 110 111 112 120 150; do
  ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 && echo "  alive: 192.168.1.$i"
done
echo "--- any Livox already on the robot subnet? ---"
for i in 120 121 122 123 124 125; do
  ping -c1 -W1 192.168.123.$i >/dev/null 2>&1 && echo "  alive: 192.168.123.$i"
done
echo "--- ARP cache (Livox MACs start 47:4f: or are vendor 'Livox') ---"
ip neigh 2>/dev/null
echo
echo ">>> IF NOTHING FOUND ABOVE: the Mid-360 broadcasts on boot. Run this for 15s"
echo ">>> on each candidate interface to catch it, then Ctrl-C:"
echo ">>>   sudo tcpdump -i <iface> -n udp and not port 22"
echo ">>> The source IP in those packets is your LiDAR's real address."

echo; echo "===== REALSENSE ====="
command -v rs-enumerate-devices >/dev/null && rs-enumerate-devices -s || echo "(librealsense not installed)"

echo; echo "===== UNITREE SOFTWARE ON DISK ====="
ls ~ 2>/dev/null
find / -maxdepth 4 -type d \( -name 'unitree*' -o -name 'unilidar*' \) 2>/dev/null | head -30
ls /unitree 2>/dev/null

echo; echo "===== RUNNING UNITREE SERVICES ====="
ps aux | grep -iE 'sport|unitree|lidar|advertise|point_cloud' | grep -v grep

echo; echo "===== LIVE DDS TOPICS (needs ROS sourced; may be empty) ====="
if [ -n "$ROS_DISTRO" ]; then ros2 topic list 2>/dev/null | head -60; else echo "(ROS not sourced -- rerun after sourcing)"; fi

echo; echo "===== DONE ====="
