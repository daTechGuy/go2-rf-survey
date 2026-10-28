"""Is the robot ready? Turns a snapshot of its state into two verdicts.
Pure, stdlib, no ROS -- tools/sensor_check.py gathers the snapshot on the
robot, this decides.

Two separate verdicts, because they need different things:

* **sensing** -- enough for dry runs, recording and mapping. Nothing moves.
* **motion** -- additionally needed before anything is allowed to move it.

The snapshot is a dict; any missing key counts as "not seen":
    odom_hz, cloud_hz, cloud_frame, sport_hz,
    mode_name (motion_switcher), avoid_enabled (obstacles_avoid),
    sport_mode, gait_type, error_code, body_height
"""

from __future__ import annotations

ODOM_MIN_HZ = 50.0          # robot publishes ~150 Hz
CLOUD_MIN_HZ = 8.0          # robot publishes ~15 Hz
STANDING_MIN_HEIGHT = 0.25  # m; ~0.32 standing, much lower lying down
# motion_switcher names for the mode that honours sport Move: 'normal' on
# older firmware, 'mcf' (the main motion service) from firmware 1.1.6. Our
# robot reported 'mcf' on 2026-10-28; same api ids per unitree_sdk2 sport_api.hpp.
MOTION_MODES = ("normal", "mcf")


def assess(s: dict):
    """Returns (sensing_ok, motion_ok, lines). Lines are 'OK  ...',
    'FAIL ...', 'WARN ...' or 'NOTE ...' -- FAIL lines are blockers."""
    lines = []
    sensing_ok = True
    motion_ok = True

    def ok(msg):
        lines.append(f"OK   {msg}")

    def fail(msg, motion_only=False):
        nonlocal sensing_ok, motion_ok
        lines.append(f"FAIL {msg}" + ("  [blocks motion]" if motion_only else ""))
        motion_ok = False
        if not motion_only:
            sensing_ok = False

    def warn(msg):
        lines.append(f"WARN {msg}")

    odom = s.get("odom_hz") or 0.0
    (ok if odom >= ODOM_MIN_HZ else fail)(f"odometry /utlidar/robot_odom {odom:.0f} Hz"
                                          + ("" if odom >= ODOM_MIN_HZ else f" (want >= {ODOM_MIN_HZ:.0f})"))
    cloud = s.get("cloud_hz") or 0.0
    (ok if cloud >= CLOUD_MIN_HZ else fail)(f"LiDAR /utlidar/cloud_deskewed {cloud:.1f} Hz"
                                            + ("" if cloud >= CLOUD_MIN_HZ else f" (want >= {CLOUD_MIN_HZ:.0f})"))
    frame = s.get("cloud_frame")
    if cloud >= CLOUD_MIN_HZ:
        if frame == "odom":
            ok("LiDAR cloud is in the odom frame (map building + obstacle stop valid)")
        else:
            fail(f"LiDAR cloud frame is {frame!r}, expected 'odom'")
    sport = s.get("sport_hz") or 0.0
    if sport > 0:
        ok(f"/sportmodestate {sport:.0f} Hz")
    else:
        fail("/sportmodestate not received (robot state unknown)", motion_only=True)

    mode = s.get("mode_name")
    if mode is None:
        fail("motion_switcher did not answer CheckMode", motion_only=True)
    elif mode in MOTION_MODES:
        ok(f"motion mode {mode!r} (sport Move honoured)")
    else:
        fail(f"motion mode {mode!r}, not 'normal'/'mcf' -- sport Move may be ignored",
             motion_only=True)

    avoid = s.get("avoid_enabled")
    if avoid is True:
        ok("Unitree obstacle avoidance ON")
    elif avoid is False:
        warn("Unitree obstacle avoidance OFF -- turn on: go2loop.sh avoid on")
    else:
        warn("obstacle avoidance state unknown (no reply)")

    h = s.get("body_height")
    if h is None:
        warn("body height unknown")
    elif h >= STANDING_MIN_HEIGHT:
        ok(f"robot standing (body height {h:.2f} m)")
    else:
        fail(f"robot not standing (body height {h:.2f} m)", motion_only=True)

    if s.get("sport_mode") is not None:
        lines.append(f"NOTE sport mode={s.get('sport_mode')} gait={s.get('gait_type')} "
                     f"error_code={s.get('error_code')}")
    lines.append("NOTE before ANY motion: remote L2+A then a SINGLE START -- "
                 "confirm the body is visibly breathing")
    return sensing_ok, motion_ok, lines


def summary(sensing_ok: bool, motion_ok: bool) -> str:
    return (f"sensing/dry-run/mapping: {'READY' if sensing_ok else 'NOT READY'}    "
            f"motion: {'READY (after you confirm breathing)' if motion_ok else 'NOT READY'}")
