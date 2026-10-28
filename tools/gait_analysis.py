"""Did the feet step? Pure, stdlib, no ROS.

Stage 1 on 2026-10-28 (firmware 'mcf') turned only 5-14 deg of 25-38
commanded, and /sportmodestate said mode=0 gait=0 throughout. The operator saw
"a rear foot moved slightly": the body twisting on planted feet, not walking.
Eyes are a poor instrument for that, so this reads it off the robot's own
state, two independent ways:

* **foot force** (SportModeState.foot_force[4]) -- a foot in the air carries
  almost nothing. A lift-off is the force falling below ``low`` x its standing
  baseline after being above ``high`` x baseline (hysteresis against noise).
* **foot height** (SportModeState.foot_position_body z, 3rd of each triple) --
  how far each foot rose above its standing height, in metres. A trot step
  lifts a foot several centimetres; a shuffle barely does.

Foot order is Unitree's: FR, FL, RR, RL. Rows are dicts with ``t`` and
``ff0..ff3`` (force) and ``fz0..fz3`` (foot z in the body frame, m); missing
values may be None. The baseline is the median over rows before ``t0`` -- the
robot standing still -- or over all rows if there are none.
"""

from __future__ import annotations

from dataclasses import dataclass

FEET = ("FR", "FL", "RR", "RL")
STEP_LIFT_M = 0.03      # a real step raises the foot at least this much


FOOT_FIELDS = [f"ff{i}" for i in range(4)] + [f"fz{i}" for i in range(4)]


def foot_row(s) -> dict:
    """ff0..3 / fz0..3 from a SportModeState (duck-typed; None -> all None)."""
    if s is None:
        return dict.fromkeys(FOOT_FIELDS)
    row = {f"ff{i}": int(s.foot_force[i]) for i in range(4)}
    row.update({f"fz{i}": round(float(s.foot_position_body[3 * i + 2]), 4) for i in range(4)})
    return row


def _median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def liftoffs(forces, baseline, low: float = 0.35, high: float = 0.65) -> int:
    """Count loaded -> unloaded transitions in a force series."""
    if not baseline or baseline <= 0:
        return 0
    loaded, count = True, 0
    for f in forces:
        if f is None:
            continue
        if loaded and f < low * baseline:
            loaded, count = False, count + 1
        elif not loaded and f > high * baseline:
            loaded = True
    return count


@dataclass
class StepReport:
    liftoffs: dict          # foot -> count during the window
    lift_m: dict            # foot -> max rise above standing height, m (or None)

    @property
    def stepped(self) -> bool:
        """Two or more feet left the ground: the gait actually ran."""
        by_force = sum(1 for n in self.liftoffs.values() if n > 0)
        by_height = sum(1 for z in self.lift_m.values()
                        if z is not None and z >= STEP_LIFT_M)
        return max(by_force, by_height) >= 2


def steps_between(rows, t0: float, t1: float, baseline_rows=None) -> StepReport:
    """Steps taken in [t0, t1], judged against the standing baseline
    (``baseline_rows`` if given, else the rows before t0, else all rows)."""
    before = [r for r in rows if r["t"] < t0]
    window = [r for r in rows if t0 <= r["t"] <= t1]
    base_rows = baseline_rows or before or rows
    counts, lifts = {}, {}
    for i, foot in enumerate(FEET):
        fk, zk = f"ff{i}", f"fz{i}"
        base_f = _median([r[fk] for r in base_rows if r.get(fk) is not None])
        counts[foot] = liftoffs([r.get(fk) for r in window], base_f)
        base_z = _median([r[zk] for r in base_rows if r.get(zk) is not None])
        zs = [r[zk] for r in window if r.get(zk) is not None]
        lifts[foot] = None if base_z is None or not zs else max(zs) - base_z
    return StepReport(counts, lifts)


def format_steps(rep: StepReport) -> str:
    feet = ", ".join(
        f"{foot} {rep.liftoffs[foot]}x"
        + ("" if rep.lift_m[foot] is None else f"/{100 * rep.lift_m[foot]:.0f}cm")
        for foot in FEET)
    verdict = "STEPPED (gait ran)" if rep.stepped else "NO STEPS (feet planted: a twist, not a walk)"
    return f"feet lift-offs/max rise: {feet}  -> {verdict}"
