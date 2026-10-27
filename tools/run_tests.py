#!/usr/bin/env python3
"""Run every test suite that needs no robot, radio or ROS.

    python tools/run_tests.py

Cross-platform twin of run_tests.sh, so the runbook has one command that
works the same in PowerShell, cmd and bash. Stdlib only -- it must run on a
bare Python before anyone has installed anything.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

SUITES = [
    "go2_survey_wifi/test/test_scanner.py",
    "tools/tests/test_fov_geometry.py",
    "tools/tests/test_turn_analysis.py",
    "tools/tests/test_readiness.py",
    "tools/tests/test_trail.py",
    "tools/tests/test_stage_gate.py",
    "tools/tests/test_remote_keys.py",
    "tools/tests/test_camera_frames.py",
    "go2_survey_bridge/test/test_limits.py",
    "go2_survey_bridge/test/test_sport_api.py",
    "go2_survey_bridge/test/test_motion_backend.py",
    "go2_survey_loop/test/test_geometry.py",
    "go2_survey_loop/test/test_obstacles.py",
    "go2_survey_loop/test/test_follower.py",
    "go2_survey_loop/test/test_footprint.py",
]

# ANSI is fine on modern Windows terminals, but not when piped to a file.
COLOR = sys.stdout.isatty()
BOLD = "\033[1m" if COLOR else ""
GREEN = "\033[32m" if COLOR else ""
RED = "\033[31m" if COLOR else ""
OFF = "\033[0m" if COLOR else ""


def main() -> int:
    failed: list[str] = []
    total = 0

    for suite in SUITES:
        path = ROOT / suite
        print(f"\n{BOLD}-- {suite}{OFF}")
        if not path.exists():
            print(f"   {RED}missing{OFF}")
            failed.append(suite)
            continue

        proc = subprocess.run(
            [sys.executable, str(path)],
            capture_output=True,
            text=True,
            cwd=ROOT,
        )
        # unittest writes its summary to stderr.
        output = (proc.stderr or "") + (proc.stdout or "")
        match = re.search(r"^Ran (\d+) test", output, re.MULTILINE)
        count = int(match.group(1)) if match else 0

        if proc.returncode == 0:
            total += count
            print(f"   {GREEN}ok{OFF}  {count} tests")
        else:
            print(output.rstrip())
            failed.append(suite)

    print()
    if failed:
        print(f"{RED}{len(failed)} suite(s) failed:{OFF} " + ", ".join(failed))
        return 1
    print(f"{GREEN}{total} tests passed across {len(SUITES)} suites{OFF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
