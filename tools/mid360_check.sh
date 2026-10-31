#!/usr/bin/env bash
# Why does the Mid-360 send us nothing? Run on the robot host (unitree@ubuntu):
#
#   bash tools/go2loop.sh mid360-check
#
# Nothing here moves the robot. Step 7 runs our Livox driver for ~15 s, which
# tells the Mid-360 to start sending points (it spins up); that is all.
#
# Since 2026-10-20 livox_ros_driver2 starts but receives no data. Suspects,
# from our setup and from Go2 + Mid-360 projects on GitHub (ACRE-Lab NCSU,
# I3T_CPSL, SeEEEun/GO2 -- all use lidar 192.168.123.20, host .18 or .99,
# ports 56101-56501):
#   A. `ros2 launch livox_ros_driver2 msg_MID360_launch.py` reads the
#      driver's OWN config inside the image -- upstream placeholders
#      192.168.1.x -- not this repo's config/MID360_config.json.
#   B. Unitree's own Mid-360 driver (unitree_slam / mid360_driver) already
#      running on the robot, holding the ports.
#   C. Small UDP receive buffers (everyone sets net.core.rmem_max=52428800).
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
LIDAR_IP="${MID360_IP:-192.168.123.20}"
HOST_IP="${MID360_HOST_IP:-192.168.123.18}"
OURS="$REPO/config/MID360_config.json"
IMAGE_CFG=/opt/go2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
SRC='source /opt/ros/humble/setup.bash; source /opt/go2_ws/install/setup.bash'
PORTS='56101|56201|56301|56401|56501'
crun() { IFACE="${IFACE:-eth0}" bash "$REPO/docker/run.sh" bash -c "$SRC; $1" 2>&1 \
           | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN|unitree_ros2'; }
ips() {   # host + lidar IPs of a MID360 config, one line
  python3 -c '
import json, sys
c = json.load(open(sys.argv[1]))
h = c["MID360"]["host_net_info"]
print("host", sorted({h[k] for k in h if k.endswith("_ip") and h[k]}),
      "lidar", [l["ip"] for l in c["lidar_configs"]])' "$1"
}
verdict=()

echo "== 1. the sensor on the network"
if ping -c2 -W1 "$LIDAR_IP" >/dev/null 2>&1; then echo "OK   $LIDAR_IP answers"
else echo "FAIL $LIDAR_IP does not answer -- power / cable / address"; verdict+=("sensor not reachable"); fi
ip -br addr show | grep -q "$HOST_IP" && echo "OK   this computer is $HOST_IP" \
  || { echo "FAIL this computer is not $HOST_IP"; verdict+=("host address"); }

echo "== 2. programs already using a Mid-360 (suspect B)"
others=$(ps -eo pid,user,etime,args | grep -i -E "mid360|livox|unitree_slam|lio_|slam" \
         | grep -v -E "grep|mid360_check" || true)
if [ -n "$others" ]; then echo "$others" | sed 's/^/     /' | cut -c1-150
  echo "WARN something else may hold the sensor (above)"; verdict+=("another Mid-360 program is running")
else echo "OK   none running"; fi

echo "== 3. the driver's UDP ports on this computer ($HOST_IP:56101-56501)"
used=$(ss -uanp 2>/dev/null | grep -E ":($PORTS)\b" || true)
if [ -n "$used" ]; then echo "$used" | sed 's/^/     /' | cut -c1-150
  echo "WARN in use already: our driver can't receive on them"; verdict+=("ports already in use")
else echo "OK   free"; fi

echo "== 4. UDP receive buffer (suspect C)"
rmem=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
if [ "$rmem" -ge 52428800 ]; then echo "OK   net.core.rmem_max=$rmem"
else echo "WARN net.core.rmem_max=$rmem (Go2 + Mid-360 setups use 52428800; dropped packets)"
  verdict+=("small receive buffer"); fi

echo "== 5. which config the driver reads (suspect A)"
[ -f "$OURS" ] && echo "     ours  ($OURS): $(ips "$OURS")"
img=$(crun "cat $IMAGE_CFG" 2>/dev/null)
if echo "$img" | python3 -c 'import json,sys; json.load(sys.stdin)' 2>/dev/null; then
  echo "     image (msg_MID360_launch.py's own): $(echo "$img" | ips /dev/stdin)"
  echo "$img" | grep -q "$LIDAR_IP" || { echo "FAIL the launch file's config does not name $LIDAR_IP: the old launch talked to the wrong address"
    verdict+=("launch file used the image's placeholder config"); }
else echo "     (could not read the image's config)"; fi
grep -q "\"$LIDAR_IP\"" "$OURS" 2>/dev/null && grep -q "\"$HOST_IP\"" "$OURS" \
  && echo "OK   ours names lidar $LIDAR_IP and host $HOST_IP" \
  || { echo "FAIL ours does not name lidar $LIDAR_IP and host $HOST_IP"; verdict+=("our config's addresses"); }

echo "== 6. Mid-360 data already on the robot's network? (topics)"
topics=$(crun "timeout 10 ros2 topic list" | grep -i -E "livox|mid360|slam|lidar|point" || true)
echo "${topics:-     (none)}" | sed 's/^/     /'

echo "== 7. our driver for 15 s, reading OUR config"
if [ -n "$used" ]; then
  echo "SKIP ports in use (step 3): it could not receive anyway"
else
  out=$(crun "(timeout 20 ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
          -p user_config_path:=/workspace/config/MID360_config.json -p xfer_format:=0 \
          -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 \
          -p frame_id:=livox_frame > /tmp/livox.log 2>&1 &) ; sleep 6; \
          timeout 8 ros2 topic hz /livox/lidar 2>&1 | grep -m2 -E 'average rate|no new'; \
          echo '--- driver log:'; tail -n 6 /tmp/livox.log")
  echo "$out" | sed 's/^/     /' | cut -c1-150
  if echo "$out" | grep -q "average rate"; then
    echo "OK   /livox/lidar is publishing with our config"; verdict+=("WORKS with our config")
  else echo "FAIL no /livox/lidar with our config either"; verdict+=("no data even with our config"); fi
fi

echo "== your step (needs sudo): is the sensor sending, and to which address?"
echo "     sudo timeout 10 tcpdump -i eth0 -n -c 20 udp and host $LIDAR_IP"
echo "     (lines '$LIDAR_IP.56301 > <ADDRESS>.56301' = points sent to <ADDRESS>)"
echo "== summary"
if [ "${#verdict[@]}" = 0 ]; then echo "     nothing found"; else printf '     - %s\n' "${verdict[@]}"; fi
