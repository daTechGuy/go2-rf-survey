#!/usr/bin/env python3
"""Copy run results off the robot onto the laptop -- and carry on where it
stopped. Run on the LAPTOP (PowerShell or any shell), cable plugged in.

    python tools/get_results.py                      # last 7 days of runs -> ~/go2-surveys
    python tools/get_results.py --days 30
    python tools/get_results.py --no-recordings      # floorplans + logs only (small, quick)
    python tools/get_results.py --run room_explore_20261031_023814
    python tools/get_results.py --list               # show what would be copied, copy nothing

Interrupted (cable pulled, robot switched off)? Plug back in and run it
again: files already here at the right size are skipped, and a file is only
given its real name once it has arrived whole. Floorplans and logs go first,
the big raw LiDAR recordings last, so a cut-short copy still has the maps.

Why: on 2026-10-31 a plain `scp -r` of the week's runs died when the cable
came out, leaving no record of what had arrived and what hadn't.

The selection is by the date stamp every run carries in its name
(``<name>_YYYYMMDD_HHMMSS``). Runs made while the robot had no internet are
stamped 1970 (its clock resets) and are always included. Undated files: only
the small session logs (*.log under 5 MB). Stdlib only.
"""

from __future__ import annotations

import argparse
import datetime as dt
import re
import subprocess
import sys
from pathlib import Path

HOST = "go2"
REMOTE_DIR = "surveys"
STAMP = re.compile(r"_(\d{8})_\d{6}")
RECORDING_EXT = (".db3", ".mcap")
SSH_OPTS = ["-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
            "-o", "ServerAliveCountMax=3"]
LIST_CMD = (f"cd ~/{REMOTE_DIR} && find . -mindepth 1 -type f ! -name '.*' "
            r"-printf '%P\t%s\n'")


def parse_listing(text: str):
    """`find -printf '%P\\t%s\\n'` output -> [(relative path, size)]."""
    out = []
    for line in text.splitlines():
        parts = line.rstrip("\r").split("\t")
        if len(parts) == 2 and parts[1].isdigit():
            out.append((parts[0], int(parts[1])))
    return out


def is_recording(path: str) -> bool:
    return path.endswith(RECORDING_EXT)


def select(entries, today: dt.date, days: int, recordings: bool = True, run: str | None = None):
    """Which remote files to fetch, small and important ones first."""
    since = (today - dt.timedelta(days=days)).strftime("%Y%m%d")
    chosen = []
    for path, size in entries:
        head = path.split("/", 1)[0]
        if is_recording(path) and not recordings:
            continue
        if run is not None:
            if head.startswith(run):
                chosen.append((path, size))
            continue
        m = STAMP.search(head)
        if m:
            stamp = m.group(1)
            if stamp >= since or stamp.startswith("1970"):
                chosen.append((path, size))
        elif path.endswith(".log") and size < 5_000_000:
            chosen.append((path, size))
    # floorplans and logs first, then the rest, raw recordings last
    rank = lambda e: (2 if is_recording(e[0]) else 0 if "topdown" in e[0] else 1, e[1])  # noqa: E731
    return sorted(chosen, key=rank)


def plan(chosen, local_size):
    """Split into (to_copy, already_here). ``local_size(path)`` -> size or None."""
    todo, done = [], []
    for path, size in chosen:
        (done if local_size(path) == size else todo).append((path, size))
    return todo, done


def human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit in ("B", "KB") else f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} GB"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dest", type=Path, default=Path.home() / "go2-surveys")
    ap.add_argument("--days", type=int, default=7)
    ap.add_argument("--no-recordings", action="store_true")
    ap.add_argument("--run", help="only this run (name prefix, e.g. room_explore_20261031)")
    ap.add_argument("--list", action="store_true", help="show the plan, copy nothing")
    ap.add_argument("--host", default=HOST)
    a = ap.parse_args(argv)

    try:
        res = subprocess.run(["ssh", *SSH_OPTS, a.host, LIST_CMD],
                             capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        print(f"no answer from {a.host} -- is the cable in and the robot on?")
        return 1
    if res.returncode != 0:
        print(f"could not list the robot's results ({a.host}): {res.stderr.strip()[-200:]}")
        print("  is the cable in, the robot on, and this a normal (not administrator) window?")
        return 1

    chosen = select(parse_listing(res.stdout), dt.date.today(), a.days,
                    recordings=not a.no_recordings, run=a.run)

    def local_size(path):
        p = a.dest / path
        return p.stat().st_size if p.is_file() else None

    todo, done = plan(chosen, local_size)
    total = sum(s for _, s in todo)
    print(f"{len(chosen)} files selected: {len(done)} already here, "
          f"{len(todo)} to copy ({human(total)}) -> {a.dest}")
    if a.list:
        for path, size in todo:
            print(f"   {human(size):>9}  {path}")
        return 0

    copied, failed, moved = 0, [], 0
    for path, size in todo:
        target = a.dest / path
        target.parent.mkdir(parents=True, exist_ok=True)
        part = target.with_name(target.name + ".part")
        print(f"   {human(moved):>9} / {human(total)}  {path}", flush=True)
        r = subprocess.run(["scp", "-q", *SSH_OPTS, f"{a.host}:{REMOTE_DIR}/{path}", str(part)],
                           capture_output=True, text=True)
        if r.returncode == 0 and part.is_file() and part.stat().st_size == size:
            part.replace(target)
            copied += 1
            moved += size
        else:
            part.unlink(missing_ok=True)
            failed.append(path)
            if r.returncode == 255:          # connection lost: the rest will fail too
                failed += [p for p, _ in todo[todo.index((path, size)) + 1:]]
                break

    print(f"copied {copied}, already here {len(done)}, not copied {len(failed)}")
    if failed:
        print("  plug back in and run this again: it carries on from here")
    maps = sorted(a.dest.glob("*_map-topdown.png"), key=lambda p: p.stat().st_mtime)
    if maps:
        print(f"newest floorplan: {maps[-1]}")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
