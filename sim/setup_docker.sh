#!/usr/bin/env bash
# Docker Engine inside WSL Ubuntu (no Docker Desktop), so the sim runs our
# real go2-survey:humble container exactly as the robot does.
# Run as root:   sudo bash sim/setup_docker.sh <linux-user>
set -euo pipefail
SIM_USER="${1:-${SUDO_USER:-}}"
[ -n "$SIM_USER" ] && [ "$(id -u)" = 0 ] || { echo "usage: sudo bash $0 <linux-user>" >&2; exit 2; }
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq docker.io >/dev/null
usermod -aG docker "$SIM_USER"
# WSL may run without systemd; start the daemon either way.
if pidof systemd >/dev/null 2>&1 && systemctl is-system-running >/dev/null 2>&1; then
  systemctl enable --now docker >/dev/null 2>&1 || true
fi
pgrep -x dockerd >/dev/null || (nohup dockerd >/var/log/dockerd.log 2>&1 &)
for _ in $(seq 20); do docker info >/dev/null 2>&1 && break; sleep 1; done
docker info --format 'docker {{.ServerVersion}} ({{.OperatingSystem}})'
echo "== $SIM_USER is in the docker group (new shells pick it up)"
