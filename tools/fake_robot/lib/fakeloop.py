"""The fake robot's autonomous-loop node: state in $FAKE_W/loop.json.

Mirrors go2_survey_loop's service rules closely enough to drive demo.sh:
start refuses while running/paused or when $FAKE_W/blocked exists; stop
with no mission is 'no mission'; resume only from paused. A live (non-dry)
mission finishes FAKE_ROUTE_S seconds after (re)starting; a dry one never
does. $FAKE_W/pause_now turns the next running status into paused.

    python fakeloop.py reset true|false     dry_run
    python fakeloop.py status
    python fakeloop.py service start|stop|resume
"""

from __future__ import annotations

import json
import os
import sys
import time

FAKE = os.environ["FAKE_W"]
STATE = os.path.join(FAKE, "loop.json")
ROUTE_S = float(os.environ.get("FAKE_ROUTE_S", "4"))


def _load():
    try:
        with open(STATE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"state": "idle", "dry": True, "t": None}


def _save(s):
    tmp = STATE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(s, f)
    os.replace(tmp, STATE)


def _flag(name):
    p = os.path.join(FAKE, name)
    if os.path.exists(p):
        try:
            os.remove(p)
        except OSError:
            pass
        return True
    return False


def reset(dry: bool):
    _save({"state": "idle", "dry": dry, "t": None})


def status_line() -> str:
    s = _load()
    if s["state"] == "running":
        if _flag("pause_now"):
            s["state"] = "paused"
            _save(s)
        elif not s["dry"] and time.time() - s["t"] >= ROUTE_S:
            s["state"] = "done"
            _save(s)
    st = s["state"]
    if st == "idle":
        return "idle | odom 0.0s old, lidar 0.0s old, floor -0.30, front 0, near 0"
    tag = "would send" if s["dry"] else "send"
    return f"{st} | fake | {tag} vx=0.00 vyaw=0.00 | odom 0.0s old"


def service(name: str):
    s = _load()
    if name == "start":
        if s["state"] in ("running", "paused"):
            return False, f"mission already {s['state']}"
        if os.path.exists(os.path.join(FAKE, "blocked")):
            return False, "refused: path blocked (front=0, near=12)"
        s.update(state="running", t=time.time())
        _save(s)
        return True, "started " + ("DRY RUN (nothing published)" if s["dry"] else "LIVE")
    if name == "stop":
        if s["state"] == "idle":
            return True, "no mission"
        s["state"] = "aborted"
        _save(s)
        return True, "mission aborted"
    if name == "resume":
        if s["state"] != "paused":
            return False, f"not paused ({s['state']})"
        s.update(state="running", t=time.time())
        _save(s)
        return True, "resumed"
    return False, f"unknown service {name}"


def main(argv):
    if argv[0] == "reset":
        reset(argv[1] == "true")
    elif argv[0] == "status":
        print(status_line())
    elif argv[0] == "service":
        ok, msg = service(argv[1])
        print(f"response:\nstd_srvs.srv.Trigger_Response(success={ok}, message='{msg}')")


if __name__ == "__main__":
    main(sys.argv[1:])
