#!/usr/bin/env python3
"""The staged rollout, enforced. Pure decision + a tiny CLI for demo.sh.

docs/autonomous-loop.md says: Stage 1 (turn in place) must pass before any
route; Stage 2 (1 m out-and-back) must pass before anything longer. Writing
that in a doc is easy to skip on demo day, so demo.sh asks this module, and
it refuses a live run unless a recent pass exists *on the same drive path*
(sport vs avoid backend -- a pass on one says nothing about the other).

    python3 tools/stage_gate.py check --length 3.0 --backend sport [--dir ~/surveys]
    python3 tools/stage_gate.py record-stage2 --backend sport [--dir ~/surveys]

Markers: <dir>/.stage1_pass (written by turn_test.py on PASS) and
<dir>/.stage2_pass (written by demo.sh when a <=1 m live route completes).
Stdlib only; runs on the robot host's own Python 3.8.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

STAGE2_MAX_LENGTH = 1.0     # a route this short IS stage 2
MAX_AGE_H = 12.0            # a pass from another day doesn't count


def _fresh(marker, backend, now, max_age_h):
    """None if the marker is a fresh pass on this backend, else the reason."""
    if marker is None:
        return "no pass recorded"
    age_h = (now - float(marker.get("epoch", 0))) / 3600.0
    if age_h > max_age_h or age_h < -0.1:
        return f"last pass is {age_h:.1f} h old (limit {max_age_h:.0f} h)"
    if marker.get("backend") != backend:
        return f"last pass was on backend {marker.get('backend')!r}, not {backend!r}"
    return None


def gate(length: float, backend: str, stage1, stage2, now: float,
         max_age_h: float = MAX_AGE_H):
    """(ok, reasons) for a LIVE route of ``length`` m on ``backend``."""
    reasons = []
    r1 = _fresh(stage1, backend, now, max_age_h)
    if r1:
        reasons.append(f"Stage 1 (turn-test): {r1} -- run "
                       f"'go2loop.sh turn-test --i-confirm-breathing' first")
    if length > STAGE2_MAX_LENGTH:
        r2 = _fresh(stage2, backend, now, max_age_h)
        if r2:
            reasons.append(f"Stage 2 (a {STAGE2_MAX_LENGTH:.0f} m route): {r2} -- run "
                           f"'demo.sh --length {STAGE2_MAX_LENGTH:.1f} ...' first")
    return not reasons, reasons


def _load(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["check", "record-stage2"])
    ap.add_argument("--length", type=float, default=0.0)
    ap.add_argument("--backend", default="sport")
    ap.add_argument("--dir", type=Path, default=Path.home() / "surveys")
    a = ap.parse_args(argv)

    if a.cmd == "record-stage2":
        a.dir.mkdir(parents=True, exist_ok=True)
        (a.dir / ".stage2_pass").write_text(json.dumps(
            {"epoch": time.time(), "time": time.strftime("%Y-%m-%d %H:%M:%S"),
             "backend": a.backend}))
        print(f"Stage 2 PASS recorded (backend={a.backend})")
        return 0

    ok, reasons = gate(a.length, a.backend, _load(a.dir / ".stage1_pass"),
                       _load(a.dir / ".stage2_pass"), time.time())
    if ok:
        print(f"stage gate: OK for a live {a.length:.1f} m route on backend {a.backend!r}")
        return 0
    print("stage gate: REFUSED")
    for r in reasons:
        print(f"  - {r}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
