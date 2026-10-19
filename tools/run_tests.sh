#!/usr/bin/env bash
# Run every test suite that does not need a robot, a radio, or ROS.
#
#   bash tools/run_tests.sh
#
# These are plain stdlib unittest so they run anywhere python3 does --
# including on a laptop before the robot is reachable, which is the whole
# reason they are written this way. `colcon test` runs them too, once there
# is a ROS workspace to run it in.

set -u
cd "$(dirname "$0")/.."

suites=(
  go2_survey_wifi/test/test_scanner.py
  go2_survey_bridge/test/test_limits.py
  go2_survey_bridge/test/test_sport_api.py
)

PY="${PYTHON:-python3}"
command -v "$PY" >/dev/null 2>&1 || PY=python

failed=0
total=0

for suite in "${suites[@]}"; do
  printf '\n\033[1m── %s\033[0m\n' "$suite"
  if out=$("$PY" "$suite" 2>&1); then
    n=$(printf '%s\n' "$out" | sed -n 's/^Ran \([0-9]*\) test.*/\1/p')
    total=$(( total + ${n:-0} ))
    printf '   \033[32mok\033[0m  %s tests\n' "${n:-?}"
  else
    printf '%s\n' "$out"
    failed=$(( failed + 1 ))
  fi
done

printf '\n'
if [ "$failed" -eq 0 ]; then
  printf '\033[32m%s tests passed across %s suites\033[0m\n' "$total" "${#suites[@]}"
else
  printf '\033[31m%s suite(s) failed\033[0m\n' "$failed"
fi
exit "$failed"
