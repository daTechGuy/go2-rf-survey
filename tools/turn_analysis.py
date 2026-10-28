"""Analyse a logged turn-in-place test. Pure, stdlib, no ROS.

Stage 1's first run (2026-10-21) commanded 0.2 rad/s for 2 s and measured
only 3 deg during the command, then a ~20 deg swing *after* commands stopped,
then settling -- while the bridge watchdog fired on time. That pattern fits
two very different stories, and this module exists to tell them apart:

* **real but late rotation** -- the robot responds with a long lag (e.g. it
  was not in walking mode and BalanceStand took hold mid-test, or the gait
  takes time to start stepping). The IMU yaw *and* the LiDAR odometry yaw both
  show it, and the IMU yaw rate rises well after the command started.
* **odometry artifact** -- the LiDAR odometry jumps when the legs start
  moving, without real rotation. The IMU disagrees with the odometry.

Each row is a dict with (at least):
    t          seconds since the test started
    cmd        yaw rate the bridge actually sent (Move z), 0 after StopMove
    api        api_id of the bridge's last request (1008 Move, 1003 StopMove)
    odom_yaw   heading from /utlidar/robot_odom, rad
    imu_yaw    heading from /sportmodestate imu_state.rpy[2], rad
    yaw_speed  /sportmodestate yaw_speed, rad/s
Missing values may be None.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

MOVE, STOP_MOVE = 1008, 1003


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_from(rows, key, t_from):
    """Cumulative heading change (rad) from the first valid sample at or
    after ``t_from``, summing wrapped successive differences so a crossing of
    +/-pi does not read as a 360 deg turn. Returns list of (t, delta)."""
    out, prev, acc = [], None, 0.0
    for r in rows:
        v = r.get(key)
        if r["t"] < t_from or v is None:
            continue
        if prev is not None:
            acc += _wrap(v - prev)
        prev = v
        out.append((r["t"], acc))
    return out


def _delta_between(series, t0, t1):
    """Net heading change between the samples nearest t0 and t1."""
    pts = [p for p in series if t0 <= p[0] <= t1]
    if len(pts) < 2:
        return None
    return pts[-1][1] - pts[0][1]


def _peak_excursion(series, t0, t1):
    """Largest |heading - heading(t0)| over [t0, t1]. Unlike the net change,
    this catches an out-and-back swing (+20 then -18 nets to +2)."""
    pts = [p for p in series if t0 <= p[0] <= t1]
    if len(pts) < 2:
        return None
    base = pts[0][1]
    return max((p[1] - base for p in pts), key=abs)


@dataclass
class TurnReport:
    commanded_deg: float
    cmd_start: float | None
    cmd_end: float | None
    latency_s: float | None          # command start -> IMU yaw rate responds
    imu_during_deg: float | None
    imu_after_deg: float | None
    imu_after_peak_deg: float | None  # largest swing after the stop
    odom_during_deg: float | None
    odom_after_deg: float | None
    max_disagreement_deg: float | None
    final_rate: float | None         # max |yaw_speed| over the last window
    verdicts: list


def analyse(rows, rate_threshold: float = 0.05, settle_window: float = 2.0,
            lag_limit: float = 0.5, artifact_deg: float = 8.0,
            min_ratio: float = 0.5) -> TurnReport:
    """``min_ratio``: the share of the commanded turn the robot must deliver
    while commanded. 0.5 suits a robot that follows its commands; turn_test
    uses 0.15 for firmware 'mcf', which on 2026-10-28 delivered a steady
    ~25-40% in ten runs (prompt, right direction, clean stop) -- sluggish,
    not erratic. Turning the WRONG way always fails."""
    rows = sorted(rows, key=lambda r: r["t"])
    if not rows:
        raise ValueError("no rows")
    t_end = rows[-1]["t"]

    starts = [r["t"] for r in rows if r.get("api") == MOVE and (r.get("cmd") or 0.0) != 0.0]
    cmd_start = starts[0] if starts else None
    cmd_end = None
    if cmd_start is not None:
        stops = [r["t"] for r in rows if r["t"] > cmd_start
                 and (r.get("api") == STOP_MOVE or (r.get("cmd") or 0.0) == 0.0)]
        cmd_end = stops[0] if stops else None

    commanded = 0.0
    if cmd_start is not None:
        moving = [r for r in rows if r.get("api") == MOVE and r["t"] >= cmd_start
                  and (cmd_end is None or r["t"] < cmd_end)]
        for a, b in zip(moving, moving[1:]):
            commanded += (a.get("cmd") or 0.0) * (b["t"] - a["t"])

    verdicts = []
    if cmd_start is None:
        verdicts.append("NO MOVE SENT: the bridge never published a non-zero Move")
        return TurnReport(0.0, None, None, None, None, None, None, None, None, None, None,
                          verdicts)

    latency = None
    sign = math.copysign(1.0, next(r["cmd"] for r in rows if r["t"] == cmd_start))
    for r in rows:
        ys = r.get("yaw_speed")
        if r["t"] >= cmd_start and ys is not None and ys * sign > rate_threshold:
            latency = r["t"] - cmd_start
            break

    t0 = rows[0]["t"]
    imu = unwrap_from(rows, "imu_yaw", t0)
    odo = unwrap_from(rows, "odom_yaw", t0)
    end_cmd = cmd_end if cmd_end is not None else t_end
    imu_during = _delta_between(imu, cmd_start, end_cmd)
    imu_after = _delta_between(imu, end_cmd, t_end)
    imu_after_peak = _peak_excursion(imu, end_cmd, t_end)
    odo_during = _delta_between(odo, cmd_start, end_cmd)
    odo_after = _delta_between(odo, end_cmd, t_end)

    disagreement = None
    if imu and odo:
        odo_at = dict(odo)
        diffs = []
        for t, d in imu:
            near = min(odo, key=lambda p: abs(p[0] - t))
            if abs(near[0] - t) <= 0.1:
                diffs.append(abs(d - near[1]))
        disagreement = max(diffs) if diffs else None
        del odo_at

    final = [abs(r["yaw_speed"]) for r in rows
             if r["t"] >= t_end - settle_window and r.get("yaw_speed") is not None]
    final_rate = max(final) if final else None

    deg = math.degrees
    if latency is None:
        verdicts.append("NO RESPONSE: IMU yaw rate never followed the command")
    elif latency > lag_limit:
        verdicts.append(f"LAG: robot responded {latency:.2f}s after the command started")
    if imu_during is not None and commanded:
        if imu_during * commanded < 0 and abs(deg(imu_during)) > 2.0:
            verdicts.append(f"WRONG WAY: turned {deg(imu_during):+.1f} deg for a "
                            f"{deg(commanded):+.1f} deg command")
        elif abs(imu_during) < min_ratio * abs(commanded):
            verdicts.append(f"UNDER-RESPONSE: {deg(abs(imu_during)):.1f} of "
                            f"{deg(abs(commanded)):.1f} deg turned while commanded "
                            f"(needs {100 * min_ratio:.0f}%)")
    if imu_after_peak is not None and abs(deg(imu_after_peak)) > 5.0:
        verdicts.append(f"MOVED AFTER STOP: swung up to {deg(imu_after_peak):.1f} deg "
                        f"after commands ended (net {deg(imu_after):.1f})")
    if disagreement is not None and deg(disagreement) > artifact_deg:
        verdicts.append(f"IMU/ODOM DISAGREE by up to {deg(disagreement):.1f} deg "
                        "-- odometry heading not trustworthy here")
    if final_rate is not None:
        if final_rate < rate_threshold:
            verdicts.append("STOPPED: no rotation in the settle window")
        else:
            verdicts.append(f"STILL ROTATING: {final_rate:.2f} rad/s in the settle window")

    return TurnReport(deg(commanded), cmd_start, cmd_end, latency,
                      None if imu_during is None else deg(imu_during),
                      None if imu_after is None else deg(imu_after),
                      None if imu_after_peak is None else deg(imu_after_peak),
                      None if odo_during is None else deg(odo_during),
                      None if odo_after is None else deg(odo_after),
                      None if disagreement is None else deg(disagreement),
                      final_rate, verdicts)


def passed(report: TurnReport) -> bool:
    """Stage 1 passes only on a prompt, proportionate, cleanly-stopped turn
    with the two heading sources agreeing."""
    bad = ("NO MOVE", "NO RESPONSE", "LAG", "UNDER-RESPONSE", "WRONG WAY",
           "MOVED AFTER STOP", "DISAGREE", "STILL ROTATING")
    return any(v.startswith("STOPPED") for v in report.verdicts) and \
        not any(b in v for v in report.verdicts for b in bad)


def summarize_replies(sent: dict, replies: list) -> list:
    """What the robot answered to each request the bridge sent.

    sent:    {request id: (t, api_id, noreply)}   from /api/sport/request
    replies: [(t, request id, api_id, code)]      from /api/sport/response
    Returns report lines, one per api: how many sent, how many answered, the
    status codes (0 = accepted) and the median reply delay. A request sent
    with noreply is expected to get none; a code other than 0 is the robot
    saying why it did not do what was asked.
    """
    lines = []
    for api in sorted({a for _, a, _ in sent.values()}):
        ids = {i for i, (_, a, _) in sent.items() if a == api}
        quiet = sum(1 for i in ids if sent[i][2])
        mine = [r for r in replies if r[1] in ids]
        codes = {}
        for r in mine:
            codes[r[3]] = codes.get(r[3], 0) + 1
        delays = sorted(r[0] - sent[r[1]][0] for r in mine)
        name = {MOVE: "Move", STOP_MOVE: "StopMove"}.get(api, f"api {api}")
        line = f"robot replies, {name:<8}: sent {len(ids)}"
        if quiet:
            line += f" ({quiet} with noreply)"
        line += f", answered {len(mine)}"
        if codes:
            line += ", codes " + ", ".join(f"{c}x{n}" for c, n in sorted(codes.items()))
        if delays:
            line += f", median delay {1000 * delays[len(delays) // 2]:.0f} ms"
        lines.append(line)
    return lines or ["robot replies: no requests seen"]


def format_report(r: TurnReport) -> str:
    f = lambda v, u="deg": "n/a" if v is None else f"{v:.1f} {u}"  # noqa: E731
    ratio = ("n/a" if r.imu_during_deg is None or not r.commanded_deg
             else f"{100 * r.imu_during_deg / r.commanded_deg:.0f}%")
    lines = [
        f"commanded rotation   : {r.commanded_deg:.1f} deg",
        f"delivered            : {ratio} of the commanded turn",
        f"response latency     : {'n/a' if r.latency_s is None else f'{r.latency_s:.2f} s'}",
        f"IMU   during / after : {f(r.imu_during_deg)} / {f(r.imu_after_deg)} "
        f"(peak after stop {f(r.imu_after_peak_deg)})",
        f"odom  during / after : {f(r.odom_during_deg)} / {f(r.odom_after_deg)}",
        f"IMU vs odom, max gap : {f(r.max_disagreement_deg)}",
        f"settle-window rate   : {'n/a' if r.final_rate is None else f'{r.final_rate:.3f} rad/s'}",
    ]
    lines += [f"  - {v}" for v in r.verdicts]
    lines.append(f"STAGE 1: {'PASS' if passed(r) else 'FAIL'}")
    return "\n".join(lines)
