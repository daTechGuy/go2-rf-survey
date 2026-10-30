#!/usr/bin/env python3
"""One page per run, to look at or show people: outcome, numbers, floorplan,
camera, and what happened when. Stdlib only.

    python3 tools/run_report.py ~/surveys/room_explore_20261031_023814
    python tools\\run_report.py C:\\Users\\me\\go2-surveys\\room_explore_20261031_023814

The argument is the run's name with its folder, no extension. It reads the
files the run left beside it -- <run>.demolog, <run>_map-topdown.png,
<run>_camera_start.jpg / _camera_end.jpg if present -- and writes
<run>_report.html: one self-contained file (images inside), so it can be
emailed or opened on a phone. demo.sh makes one at the end of every run.
"""

from __future__ import annotations

import base64
import html
import re
import sys
from pathlib import Path

STATE_LINE = re.compile(
    r"^\s*(\d\d:\d\d:\d\d) (running|paused|done|aborted) \| ([^|]*?) \|.*?"
    r"(?:front (\d+), near (\d+))?\s*$")
FINISHED = re.compile(r"== (\d\d:\d\d:\d\d) demo finished: (.*)$")
REMOTE_START = re.compile(r"(\d\d:\d\d:\d\d) remote start")
NUM_IN_REASON = re.compile(r"\(front=\d+, near=\d+\)|\d+ points|near=\d+")


def _secs(hms: str) -> int:
    h, m, s = (int(v) for v in hms.split(":"))
    return h * 3600 + m * 60 + s


def parse_demolog(text: str) -> dict:
    """What a demo.sh log says about its run. Pure; missing bits are None."""
    r = {"what": None, "outcome": None, "finished_at": None, "started_at": None,
         "ended_at": None, "timeline": [], "pauses": [], "path_m": None, "gap_m": None,
         "extent": None, "legs": None, "mode": "route", "settings": []}
    last = None
    for line in text.splitlines():
        if line.startswith("== LIVE run:") or line.startswith("== REHEARSAL"):
            r["what"] = r["what"] or line[3:].strip()
            if "explore" in line:
                r["mode"] = "explore"
        if line.strip().startswith(("route ", "explore:", "speed <=")):
            r["settings"].append(line.strip())
        m = REMOTE_START.search(line)
        if m and r["started_at"] is None:
            r["started_at"] = m.group(1)
        m = STATE_LINE.match(line)
        if m:
            t, state, reason = m.group(1), m.group(2), m.group(3).strip()
            if r["started_at"] is None and state == "running":
                r["started_at"] = t
            key = (state, NUM_IN_REASON.sub("", reason))
            if key != last:
                r["timeline"].append((t, state, reason))
                if state == "paused":
                    r["pauses"].append((t, reason))
                last = key
            if state in ("done", "aborted"):
                r["ended_at"] = r["ended_at"] or t
            m2 = re.search(r"explored (\d+) legs", reason)
            if m2:
                r["legs"] = int(m2.group(1))
        m = FINISHED.search(line)
        if m:
            r["finished_at"], r["outcome"] = m.group(1), m.group(2).strip()
        m = re.match(r"^loop\s+path ([\d.]+) m, start-end gap ([\d.]+) m", line)
        if m:
            r["path_m"], r["gap_m"] = float(m.group(1)), float(m.group(2))
        m = re.match(r"^extent\s+([\d.]+ x [\d.]+ m)", line)
        if m:
            r["extent"] = m.group(1)
    end = r["ended_at"] or r["finished_at"]          # stopped early: until it was stopped
    if r["started_at"] and end:
        d = _secs(end) - _secs(r["started_at"])
        r["duration_s"] = d + 86400 if d < 0 else d        # past midnight
    else:
        r["duration_s"] = None
    return r


def verdict(outcome: str | None) -> tuple:
    """(short label, css class) for the headline."""
    o = (outcome or "").lower()
    if o.startswith("done"):
        return "Completed", "ok"
    if o.startswith("rehearsal ok"):
        return "Rehearsal OK (nothing moved)", "ok"
    if o.startswith("aborted"):
        return "Stopped (aborted)", "warn"
    if o.startswith("stopped early"):
        return "Stopped early", "warn"
    if not o:
        return "No result recorded", "warn"
    return o[:60], "bad"


def _img(path: Path, mime: str):
    if not path.is_file():
        return None
    return f"data:{mime};base64," + base64.b64encode(path.read_bytes()).decode()


def render(name: str, r: dict, floorplan=None, cam_start=None, cam_end=None) -> str:
    e = html.escape
    label, cls = verdict(r["outcome"])
    nums = []
    if r["duration_s"] is not None:
        m, s = divmod(r["duration_s"], 60)
        nums.append(("Time walking", f"{m} min {s} s" if m else f"{s} s"))
    if r["path_m"] is not None:
        nums.append(("Distance walked", f"{r['path_m']:.1f} m"))
    if r["gap_m"] is not None:
        nums.append(("Finished from start", f"{r['gap_m']:.2f} m"))
    if r["legs"] is not None:
        nums.append(("Legs explored", str(r["legs"])))
    if r["extent"]:
        nums.append(("Map size", r["extent"]))
    nums.append(("Pauses", str(len(r["pauses"]))))

    tiles = "".join(f'<div class="tile"><div class="v">{e(v)}</div><div class="k">{e(k)}</div></div>'
                    for k, v in nums)
    rows = "".join(
        f'<tr class="{e(st)}"><td>{e(t)}</td><td>{e(st)}</td><td>{e(reason)}</td></tr>'
        for t, st, reason in r["timeline"])
    pauses = ("".join(f"<li><b>{e(t)}</b> {e(reason)}</li>" for t, reason in r["pauses"])
              or "<li>none</li>")
    pics = ""
    if cam_start or cam_end:
        pics = '<h2>Robot camera</h2><div class="cams">' + "".join(
            f'<figure><img src="{src}" alt="{e(cap)}"><figcaption>{e(cap)}</figcaption></figure>'
            for src, cap in ((cam_start, "at the start"), (cam_end, "at the end")) if src) + "</div>"
    plan = (f'<h2>Floorplan</h2><img class="plan" src="{floorplan}" alt="floorplan with the robot\'s path">'
            if floorplan else "<h2>Floorplan</h2><p>No floorplan was built for this run.</p>")
    settings = "".join(f"<li>{e(s)}</li>" for s in r["settings"])

    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Go2 run report</title>
<style>
:root {{ --bg:#fbfbf9; --fg:#1d1d1b; --muted:#6b6b66; --card:#fff; --line:#e3e2dc;
        --ok:#1f7a4a; --warn:#a15c00; --bad:#b3261e; }}
@media (prefers-color-scheme: dark) {{
  :root {{ --bg:#161615; --fg:#ecebe6; --muted:#a3a29c; --card:#21211f; --line:#34332f;
          --ok:#5cc28d; --warn:#e3a14b; --bad:#f0857d; }} }}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--fg);
       font:16px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif; }}
main {{ max-width:960px; margin:0 auto; padding:24px 16px 48px; }}
h1 {{ font-size:1.5rem; margin:0 0 4px; }} h2 {{ font-size:1.1rem; margin:32px 0 8px; }}
.sub {{ color:var(--muted); margin:0 0 16px; }}
.badge {{ display:inline-block; padding:4px 10px; border-radius:999px; font-weight:600;
         border:1px solid currentColor; }}
.ok {{ color:var(--ok); }} .warn {{ color:var(--warn); }} .bad {{ color:var(--bad); }}
.tiles {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(140px,1fr)); gap:8px; margin-top:16px; }}
.tile {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:12px; }}
.tile .v {{ font-size:1.3rem; font-weight:650; }} .tile .k {{ color:var(--muted); font-size:.85rem; }}
img.plan {{ width:100%; height:auto; border:1px solid var(--line); border-radius:10px; background:#fff; }}
.cams {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(260px,1fr)); gap:8px; }}
.cams img {{ width:100%; border-radius:10px; }} figure {{ margin:0; }}
figcaption {{ color:var(--muted); font-size:.85rem; }}
table {{ width:100%; border-collapse:collapse; font-size:.9rem; }}
td {{ padding:4px 8px; border-bottom:1px solid var(--line); vertical-align:top; }}
td:first-child {{ white-space:nowrap; color:var(--muted); font-variant-numeric:tabular-nums; }}
tr.paused td:nth-child(2) {{ color:var(--warn); }} tr.aborted td:nth-child(2) {{ color:var(--bad); }}
tr.done td:nth-child(2) {{ color:var(--ok); }}
.wrap {{ overflow-x:auto; }} ul {{ padding-left:20px; }} .muted {{ color:var(--muted); }}
</style></head>
<body><main>
<h1>{e(name)}</h1>
<p class="sub">{e(r["what"] or "Go2 run")}</p>
<span class="badge {cls}">{e(label)}</span>
<span class="muted">&nbsp;{e(r["outcome"] or "")}</span>
<div class="tiles">{tiles}</div>
{plan}
{pics}
<h2>Pauses</h2><ul>{pauses}</ul>
<h2>What happened when</h2><div class="wrap"><table>{rows}</table></div>
<h2>Settings</h2><ul class="muted">{settings or "<li>not recorded</li>"}</ul>
</main></body></html>
"""


def build(run: Path) -> Path:
    log = run.with_name(run.name + ".demolog")
    if not log.is_file():
        raise FileNotFoundError(f"no run log {log}")
    r = parse_demolog(log.read_text(errors="replace"))
    out = run.with_name(run.name + "_report.html")
    out.write_text(render(
        run.name, r,
        floorplan=_img(run.with_name(run.name + "_map-topdown.png"), "image/png"),
        cam_start=_img(run.with_name(run.name + "_camera_start.jpg"), "image/jpeg"),
        cam_end=_img(run.with_name(run.name + "_camera_end.jpg"), "image/jpeg")),
        encoding="utf-8")
    return out


def main(argv=None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if not args:
        print(__doc__)
        return 2
    run = Path(args[0]).expanduser()
    for suffix in (".demolog", "_report.html"):                  # accept a file name too
        if run.name.endswith(suffix):
            run = run.with_name(run.name[: -len(suffix)])
    try:
        out = build(run)
    except FileNotFoundError as exc:
        print(exc)
        return 1
    print(f"report: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
