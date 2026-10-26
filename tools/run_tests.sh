#!/usr/bin/env bash
# Run every test suite that does not need a robot, a radio, or ROS.
#
#   bash tools/run_tests.sh
#
# Thin wrapper: the suite list lives in one place, tools/run_tests.py.
# (This script used to keep its own list and fell behind it.)
cd "$(dirname "$0")/.." || exit 1
PY="${PYTHON:-python3}"
command -v "$PY" >/dev/null 2>&1 || PY=python
exec "$PY" tools/run_tests.py "$@"
