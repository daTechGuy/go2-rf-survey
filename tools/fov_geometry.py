#!/usr/bin/env python3
"""Mid-360 mount geometry: how much of the floor the LiDAR cannot see.

The Mid-360's vertical FOV is -7 deg to +52 deg -- asymmetric and biased
upward. Mounted level, its lowest ray does not reach the floor until
h/tan(7 deg) ahead, which at a 0.40 m mount height is 3.3 m. Everything
nearer is invisible: the floor, curbs, chair legs, descending steps.

This fails quietly. The map looks excellent and the robot walks into things.

Run it for your own mount height:

    python tools/fov_geometry.py --height 0.42

Stdlib only, so it runs anywhere -- and the table it prints is the same one
in docs/plan.md and the README, generated rather than transcribed, so the
docs cannot drift away from the maths.
"""

from __future__ import annotations

import argparse
import math

# Sensor constants, from the Livox Mid-360 datasheet.
FOV_LOWER_DEG = -7.0     # most negative ray, relative to the sensor's horizon
FOV_UPPER_DEG = 52.0
MIN_RANGE_M = 0.1        # blind sphere around the sensor itself

DEFAULT_MOUNT_HEIGHT_M = 0.40


def lowest_ray_deg(mount_pitch_deg: float) -> float:
    """Angle of the lowest ray below horizontal, for a nose-down pitch.

    Positive pitch tilts the sensor nose-down, which steepens the lowest ray.
    Returned as a signed angle in the sensor's own convention: negative means
    below horizontal.
    """
    return FOV_LOWER_DEG - mount_pitch_deg


def upper_fov_deg(mount_pitch_deg: float) -> float:
    """Remaining coverage above horizontal after pitching down."""
    return FOV_UPPER_DEG - mount_pitch_deg


def ground_contact_m(
    mount_pitch_deg: float, height_m: float = DEFAULT_MOUNT_HEIGHT_M
) -> float:
    """Distance ahead at which the lowest ray meets the floor.

    ``d = h / tan(pitch + 7 deg)``

    Returns ``inf`` when the lowest ray points at or above horizontal, which
    is the honest answer: the ray never meets the floor at all.
    """
    depression_deg = mount_pitch_deg + abs(FOV_LOWER_DEG)
    if depression_deg <= 0.0:
        return math.inf
    if depression_deg >= 90.0:
        return 0.0
    return height_m / math.tan(math.radians(depression_deg))


def blind_radius_m(
    mount_pitch_deg: float, height_m: float = DEFAULT_MOUNT_HEIGHT_M
) -> float:
    """Radius of the unobserved floor disc around the robot.

    Never smaller than the sensor's own 0.1 m blind sphere, however steeply
    it is pitched.
    """
    return max(MIN_RANGE_M, ground_contact_m(mount_pitch_deg, height_m))


def recommend_pitch(
    height_m: float = DEFAULT_MOUNT_HEIGHT_M,
    max_blind_m: float = 0.8,
    min_upper_fov_deg: float = 25.0,
) -> float | None:
    """Shallowest pitch meeting both constraints, or None if impossible.

    Two competing pressures. Too shallow and the blind radius exceeds what
    the depth camera can cover. Too steep and the upper FOV stops catching
    the ceilings and doorframes that make indoor LiDAR odometry stable.

    Shallowest-that-works is the right pick: it keeps the most upper FOV
    while satisfying the floor-coverage requirement.
    """
    for tenth in range(0, 601):
        pitch = tenth / 10.0
        if upper_fov_deg(pitch) < min_upper_fov_deg:
            break
        if blind_radius_m(pitch, height_m) <= max_blind_m:
            return pitch
    return None


def table(height_m: float, pitches: list[float]) -> list[tuple]:
    return [
        (p, lowest_ray_deg(p), blind_radius_m(p, height_m), upper_fov_deg(p))
        for p in pitches
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, default=DEFAULT_MOUNT_HEIGHT_M,
                    metavar="M", help="sensor height above the floor, metres")
    ap.add_argument("--max-blind", type=float, default=0.8, metavar="M",
                    help="largest acceptable unobserved radius")
    ap.add_argument("--min-upper-fov", type=float, default=25.0, metavar="DEG",
                    help="upward coverage to retain for SLAM features")
    args = ap.parse_args()

    h = args.height
    print(f"Livox Mid-360 vertical FOV: {FOV_LOWER_DEG:+.0f} to "
          f"{FOV_UPPER_DEG:+.0f} deg   (sensor height {h:.2f} m)\n")

    print(f"{'pitch':>7}  {'lowest ray':>11}  {'blind radius':>13}  {'upper FOV':>10}")
    print(f"{'-'*7}  {'-'*11}  {'-'*13}  {'-'*10}")
    for pitch, low, blind, upper in table(h, [0, 5, 10, 15, 20, 22, 25, 30, 40]):
        blind_s = "never" if math.isinf(blind) else f"{blind:.2f} m"
        note = "  <- level" if pitch == 0 else ""
        print(f"{pitch:>6.0f}d  {low:>10.0f}d  {blind_s:>13}  {upper:>9.0f}d{note}")

    rec = recommend_pitch(h, args.max_blind, args.min_upper_fov)
    print()
    if rec is None:
        print(f"No pitch satisfies blind <= {args.max_blind} m AND upper FOV "
              f">= {args.min_upper_fov} deg at {h:.2f} m.")
        print("Lower the sensor, or accept a larger blind radius and cover it "
              "with the depth camera.")
    else:
        print(f"Recommended: {rec:.1f} deg nose-down")
        print(f"  blind radius {blind_radius_m(rec, h):.2f} m "
              f"(limit {args.max_blind:.2f} m)")
        print(f"  upper FOV    {upper_fov_deg(rec):.0f} deg "
              f"(floor {args.min_upper_fov:.0f} deg)")
        print(f"\nPut this in config/mount.yaml as mount_pitch_deg, and verify "
              f"in RViz:\n  the floor must read flat and at z = 0 in "
              f"base_footprint.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
