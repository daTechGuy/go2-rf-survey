#!/usr/bin/env bash
# Start-of-session check, run on the robot host (unitree@ubuntu):
#
#   bash tools/preflight.sh              # check + update code to origin/main
#   bash tools/preflight.sh --no-update  # check only
#
# Read-only except one thing: it updates the repo's CODE folders to
# origin/main. It never touches config/ (robot-specific IPs live there) and
# never commands the robot. Things it cannot do itself (sudo, the remote) it
# prints as instructions.
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
UPDATE=1
[ "${1:-}" = "--no-update" ] && UPDATE=0
BLOCK=0
CODE_DIRS="go2_survey_bridge go2_survey_loop go2_survey_wifi go2_survey_description tools docker docs"

src_sha() { cat docker/Dockerfile docker/entrypoint.sh | sha256sum | cut -c1-16; }

echo "== network"
NET=0
if timeout 6 ping -c1 -W4 github.com >/dev/null 2>&1; then
  gw=$(ip route show default 2>/dev/null | head -1 | awk '{print $3}')
  echo "OK   internet via the laptop (${gw:-default})"; NET=1
else
  echo "WARN no internet (needed only for code updates and image builds)"
  if ip route | grep -q '^default via 192.168.123.222' && ip -4 addr | grep -q '192\.168\.137\.'; then
    echo "     WiFi hotspot blocked by stale cable route -- run: sudo ip route del default via 192.168.123.222 dev eth0"
  else
    echo "     wired cable: sudo ip route replace default via 192.168.123.222 dev eth0"
    echo "     WiFi hotspot (unplugged): sudo ip route del default via 192.168.123.222 dev eth0"
  fi
fi

echo "== code"
if [ "$NET" = 1 ] && [ "$UPDATE" = 1 ]; then
  if git fetch -q origin && git checkout origin/main -- $CODE_DIRS 2>/dev/null; then
    echo "OK   code folders at origin/main $(git rev-parse --short origin/main) (config/ untouched)"
  else
    echo "WARN could not update code"
  fi
else
  echo "NOTE code not updated ($( [ "$UPDATE" = 0 ] && echo '--no-update' || echo 'no internet'))"
fi

echo "== image"
if ! docker image inspect go2-survey:humble >/dev/null 2>&1; then
  echo "FAIL no go2-survey:humble image -- run: bash tools/go2loop.sh build"; BLOCK=1
else
  have=$(docker inspect -f '{{index .Config.Labels "go2.src_sha"}}' go2-survey:humble 2>/dev/null)
  want=$(src_sha)
  if [ -z "$have" ] || [ "$have" = "<no value>" ]; then
    echo "WARN image has no source stamp -- run once: bash tools/go2loop.sh build"
  elif [ "$have" != "$want" ]; then
    echo "FAIL image is older than docker/ -- run: bash tools/go2loop.sh build"; BLOCK=1
  else
    echo "OK   image matches docker/ ($want)"
  fi
fi

echo "== containers"
running=$(docker ps --format '{{.Names}}' | grep -E '^go2(loop|bridge|viz|rec|mid360|remote)$' | tr '\n' ' ')
if [ -n "$running" ]; then
  echo "NOTE already running: $running(bash tools/go2loop.sh down / bridge-down / viz-down / mid360-down)"
else
  echo "OK   no leftover go2 containers"
fi

echo "== disk"
free_gb=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
if [ "${free_gb:-0}" -ge 5 ]; then echo "OK   ${free_gb} GB free for recordings"
else echo "WARN only ${free_gb} GB free -- a 4 min recording is ~1.2 GB"; fi

echo "== robot"
if [ "$BLOCK" = 1 ]; then
  echo "skipped -- fix the image first"
  exit 2
fi
IFACE="${IFACE:-eth0}" bash docker/run.sh python3 /workspace/tools/sensor_check.py "$@" 2>&1 \
  | grep -v -E '^(image|interface|repo|surveys) |^$|^WARN: unitree|unitree_ros2'
exit "${PIPESTATUS[0]}"
