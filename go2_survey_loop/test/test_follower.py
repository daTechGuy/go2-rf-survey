"""Tests for the steering law and the mission state machine. No ROS, no clock.

The centrepiece is a closed-loop simulation: a unicycle robot is driven only
by the mission's own commands and must complete the rectangle and finish at
the start. If that test passes, the follower genuinely closes the loop -- it
is not just emitting plausible-looking numbers.

    python go2_survey_loop/test/test_follower.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_loop.follower import (  # noqa: E402
    Command, FollowerParams, Mission, SafetyParams, State, compute_command,
)
from go2_survey_loop.geometry import (  # noqa: E402
    Pose2D, out_and_back_local, rectangle_local, to_world,
)
import random  # noqa: E402

FP = FollowerParams()
FRESH = 0.05        # input age that counts as fresh
DT = 0.1            # node tick


def simulate(mission, pose, steps, obstacle=lambda t, p: (0, 0),
             odom_age=lambda t: FRESH, cloud_age=lambda t: FRESH):
    """Drive a unicycle with the mission's output. Returns (pose, log)."""
    log = []
    t = 0.0
    for _ in range(steps):
        front, near = obstacle(t, pose)
        cmd = mission.step(t, pose, odom_age(t), cloud_age(t), front, near)
        log.append((t, pose, cmd))
        if cmd.publish and cmd.state == State.RUNNING:
            yaw = pose.yaw + cmd.vyaw * DT
            pose = Pose2D(pose.x + cmd.vx * DT * math.cos(yaw),
                          pose.y + cmd.vx * DT * math.sin(yaw), yaw)
        t += DT
        if cmd.state in (State.DONE, State.ABORTED) and not cmd.publish:
            break
    return pose, log


def new_mission(length=4.0, width=3.0, turn="left", start=Pose2D(0, 0, 0), **safety):
    wps = to_world(start, rectangle_local(length, width, turn))
    return Mission(wps, start, 0.0, FP, SafetyParams(**safety))


class TestSteering(unittest.TestCase):
    def test_straight_ahead_drives(self):
        vx, vyaw, turning, _ = compute_command(Pose2D(0, 0, 0), (2, 0), FP, False)
        self.assertGreater(vx, 0)
        self.assertAlmostEqual(vyaw, 0.0)
        self.assertFalse(turning)

    def test_target_behind_turns_in_place(self):
        vx, vyaw, turning, _ = compute_command(Pose2D(0, 0, 0), (-2, 0.01), FP, False)
        self.assertEqual(vx, 0.0)
        self.assertTrue(turning)
        self.assertNotEqual(vyaw, 0.0)

    def test_target_left_turns_counter_clockwise(self):
        _, vyaw, _, _ = compute_command(Pose2D(0, 0, 0), (0, 2), FP, False)
        self.assertGreater(vyaw, 0)

    def test_target_right_turns_clockwise(self):
        _, vyaw, _, _ = compute_command(Pose2D(0, 0, 0), (0, -2), FP, False)
        self.assertLess(vyaw, 0)

    def test_speed_never_exceeds_limits(self):
        for tx, ty in [(100, 0), (0, 100), (-100, 1), (3, 3)]:
            vx, vyaw, _, _ = compute_command(Pose2D(0, 0, 0), (tx, ty), FP, False)
            self.assertLessEqual(abs(vx), FP.max_vx + 1e-9)
            self.assertLessEqual(abs(vyaw), FP.max_vyaw + 1e-9)

    def test_never_reverses(self):
        vx, _, _, _ = compute_command(Pose2D(0, 0, 0), (-1, 0.5), FP, False)
        self.assertGreaterEqual(vx, 0.0)

    def test_walks_steadily_up_to_the_target(self):
        # no slow-down into the Go2's sway zone (min_vx = max_vx since 2026-10-31)
        far, _, _, _ = compute_command(Pose2D(0, 0, 0), (3.0, 0), FP, False)
        close, _, _, _ = compute_command(Pose2D(0, 0, 0), (0.25, 0), FP, False)
        self.assertAlmostEqual(close, far)
        self.assertAlmostEqual(close, FP.max_vx)

    def test_never_slower_than_min_vx_short_of_target(self):
        # 2026-10-31 on the robot: 0.13 m/s at 22 cm out, and it stood still
        vx, _, turning, dist = compute_command(Pose2D(0, 0, 0), (0.22, 0), FP, False)
        self.assertFalse(turning)
        self.assertGreater(dist, FP.reach_tol)
        self.assertGreaterEqual(vx, FP.min_vx - 1e-9)

    def test_min_turn_rate_when_turning(self):
        # small error while already turning: still turns at least min_vyaw
        target = (math.cos(0.13), math.sin(0.13))
        _, vyaw, turning, _ = compute_command(Pose2D(0, 0, 0), target, FP, True)
        self.assertTrue(turning)
        self.assertGreaterEqual(abs(vyaw), FP.min_vyaw)

    def test_hysteresis(self):
        # 0.2 rad off: below turn_enter (drive) but above turn_exit (keep turning)
        target = (math.cos(0.2), math.sin(0.2))
        self.assertFalse(compute_command(Pose2D(0, 0, 0), target, FP, False)[2])
        self.assertTrue(compute_command(Pose2D(0, 0, 0), target, FP, True)[2])

    def test_nan_pose_does_not_command_motion(self):
        for pose in (Pose2D(math.nan, 0, 0), Pose2D(0, 0, math.inf)):
            vx, vyaw, _, dist = compute_command(pose, (1, 0), FP, False)
            self.assertEqual((vx, vyaw), (0.0, 0.0))
            self.assertEqual(dist, math.inf)   # explicit refusal, not NaN luck

    def test_nan_pose_pauses_mission(self):
        m = new_mission()
        cmd = m.step(0.0, Pose2D(math.nan, math.nan, 0), FRESH, FRESH, 0, 0)
        self.assertEqual(cmd.state, State.PAUSED)
        self.assertEqual(m.index, 0)
        self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))


class TestClosedLoop(unittest.TestCase):
    def assert_completes(self, mission, start):
        end, log = simulate(mission, start, steps=4000)
        self.assertEqual(mission.state, State.DONE, mission.reason)
        self.assertLess(math.hypot(end.x - start.x, end.y - start.y), FP.reach_tol + 0.05)
        for _, _, cmd in log:
            self.assertLessEqual(abs(cmd.vx), FP.max_vx + 1e-9)
            self.assertLessEqual(abs(cmd.vyaw), FP.max_vyaw + 1e-9)
        return log

    def test_left_loop_completes(self):
        start = Pose2D(0, 0, 0)
        self.assert_completes(new_mission(start=start), start)

    def test_right_loop_completes(self):
        start = Pose2D(0, 0, 0)
        self.assert_completes(new_mission(turn="right", start=start), start)

    def test_loop_from_arbitrary_start_pose(self):
        start = Pose2D(12.3, -4.5, 2.1)
        self.assert_completes(new_mission(start=start), start)

    def test_visits_every_corner(self):
        start = Pose2D(0, 0, 0)
        log = self.assert_completes(new_mission(start=start), start)
        corners = [(4, 0), (4, 3), (0, 3)]
        for cx, cy in corners:
            closest = min(math.hypot(p.x - cx, p.y - cy) for _, p, _ in log)
            self.assertLess(closest, FP.reach_tol + 0.05, f"never reached {(cx, cy)}")

    def test_tiny_test_loop_completes(self):
        start = Pose2D(0, 0, 0)
        self.assert_completes(new_mission(length=1.0, width=1.0, start=start), start)

    def test_goes_silent_after_done(self):
        start = Pose2D(0, 0, 0)
        _, log = simulate(new_mission(start=start), start, steps=4000)
        self.assertFalse(log[-1][2].publish)


class TestOutAndBackLoop(unittest.TestCase):
    """The classroom route: 3 m (10 ft) out from the door, turn round, home."""

    def mission(self, start=Pose2D(0, 0, 0), length=3.0):
        return Mission(to_world(start, out_and_back_local(length)), start, 0.0, FP,
                       SafetyParams(geofence_m=length + 1.0))

    def test_completes_and_returns_to_door(self):
        start = Pose2D(0, 0, 0)
        m = self.mission(start)
        end, log = simulate(m, start, steps=4000)
        self.assertEqual(m.state, State.DONE, m.reason)
        self.assertLess(math.hypot(end.x, end.y), FP.reach_tol + 0.05)
        far = max(p.x for _, p, _ in log)
        self.assertGreater(far, 3.0 - FP.reach_tol - 0.05)

    def test_never_leaves_the_aisle(self):
        # with no obstacles to dodge it should stay on the line out and back:
        # sideways drift is what would put a leg into a desk
        start = Pose2D(0, 0, 0)
        _, log = simulate(self.mission(start), start, steps=4000)
        self.assertLess(max(abs(p.y) for _, p, _ in log), 0.10)

    def test_geofence_is_length_plus_margin(self):
        m = self.mission()
        self.assertEqual(m.step(0.0, Pose2D(4.2, 0, 0), FRESH, FRESH, 0, 0).state,
                         State.ABORTED)

    # The bridge ramps yaw rate (accel_yaw 1.6 rad/s^2), so at the start of the
    # turn-round the real robot lingers near the +/-pi seam for several ticks.
    # That -- not instantaneous rotation -- is when odometry noise can flip the
    # direction, so the model must include the ramp or the test proves nothing.
    YAW_ACCEL = 1.6      # rad/s^2, go2_survey_bridge default
    YAW_NOISE = 0.03     # rad, 1 sigma, gait-induced odometry jitter

    def _turnaround_reversals(self, seed, latch=True):
        """Direction flips while turning in place at the far end, for one
        noise seed, with the bridge's yaw ramp modelled. Returns (flips, m)."""
        rnd = random.Random(seed)
        start = Pose2D(0, 0, 0)
        m = self.mission(start)
        if not latch:
            m._latch_turn = lambda pose, vyaw: vyaw          # disable the fix
        pose, t, w, signs = start, 0.0, 0.0, []
        for _ in range(6000):
            seen = Pose2D(pose.x, pose.y, pose.yaw + rnd.gauss(0.0, self.YAW_NOISE))
            cmd = m.step(t, seen, FRESH, FRESH, 0, 0)
            if cmd.state != State.RUNNING:
                break
            if m.index == 1 and cmd.vx == 0.0 and cmd.vyaw != 0.0:
                signs.append(math.copysign(1.0, cmd.vyaw))
            step = self.YAW_ACCEL * DT                       # slew-limited yaw rate
            w += max(-step, min(step, cmd.vyaw - w))
            yaw = pose.yaw + w * DT
            pose = Pose2D(pose.x + cmd.vx * DT * math.cos(yaw),
                          pose.y + cmd.vx * DT * math.sin(yaw), yaw)
            t += DT
        return sum(1 for a, b in zip(signs, signs[1:]) if a != b), m

    def test_turnaround_commits_to_one_direction_under_noise(self):
        for seed in range(40):
            flips, m = self._turnaround_reversals(seed, latch=True)
            self.assertEqual(flips, 0, f"seed {seed}: turn reversed {flips}x")
            self.assertEqual(m.state, State.DONE, f"seed {seed}: {m.reason}")

    def test_model_is_able_to_show_the_problem(self):
        # Guards the test above: without the latch, the same model and seeds
        # must exhibit reversals. If this ever fails, the noise/ramp model has
        # stopped exercising the seam and the latch test is no longer evidence.
        total = sum(self._turnaround_reversals(s, latch=False)[0] for s in range(40))
        self.assertGreater(total, 0)


class TestRouteLine(unittest.TestCase):
    """Steering along the route line, not at the corner (FollowerParams.lookahead)."""

    def test_aim_point_on_the_line_ahead(self):
        from go2_survey_loop.follower import aim_point
        ax, ay = aim_point((0, 0), (3, 0), Pose2D(1.0, 0.3, 0), 0.6)
        self.assertAlmostEqual(ax, 1.6)
        self.assertAlmostEqual(ay, 0.0)                 # on the line, not at the robot's y

    def test_aim_point_never_past_the_corner(self):
        from go2_survey_loop.follower import aim_point
        self.assertEqual(aim_point((0, 0), (3, 0), Pose2D(2.8, 0.1, 0), 0.6), (3, 0))
        self.assertEqual(aim_point((0, 0), (3, 0), Pose2D(4.0, 0.0, 0), 0.6), (3, 0))

    def test_aim_point_degenerate_cases(self):
        from go2_survey_loop.follower import aim_point
        self.assertEqual(aim_point(None, (3, 0), Pose2D(1, 1, 0), 0.6), (3, 0))
        self.assertEqual(aim_point((0, 0), (3, 0), Pose2D(1, 1, 0), 0.0), (3, 0))
        self.assertEqual(aim_point((3, 0), (3, 0), Pose2D(1, 1, 0), 0.6), (3, 0))

    def test_offset_robot_steers_back_harder_than_corner_aiming(self):
        pose = Pose2D(1.0, 0.15, 0.0)                    # 15 cm left of the line
        _, line_yaw, _, _ = compute_command(pose, (3, 0), FP, False, origin=(0, 0))
        _, corner_yaw, _, _ = compute_command(pose, (3, 0), FP, False)
        self.assertLess(line_yaw, 0)                     # turns right, back to the line
        self.assertLess(line_yaw, corner_yaw)            # and harder than aiming at the corner

    def test_on_the_line_drives_straight(self):
        vx, vyaw, _, _ = compute_command(Pose2D(1.0, 0.0, 0.0), (3, 0), FP, False, origin=(0, 0))
        self.assertGreater(vx, 0)
        self.assertAlmostEqual(vyaw, 0.0)

    def _return_leg(self, lookahead, shift=0.13, creep=0.0):
        """Out-and-back 3 m with what the Gazebo Go2 did: a sideways shift
        while turning on the spot (and optionally a slow sideways creep while
        walking). Returns (completed, {x: |y|} along the return leg)."""
        # the turn speeds of the Gazebo runs this reproduces (before 2026-10-28)
        fp = FollowerParams(lookahead=lookahead, max_vyaw=0.5, min_vyaw=0.15)
        wps = to_world(Pose2D(0, 0, 0), out_and_back_local(3.0))
        m = Mission(wps, Pose2D(0, 0, 0), 0.0, fp, SafetyParams())
        pose, t, turned, prof = Pose2D(0, 0, 0), 0.0, 0.0, {}
        for _ in range(3000):
            cmd = m.step(t, pose, FRESH, FRESH, 0, 0)
            if cmd.state in (State.DONE, State.ABORTED):
                return cmd.state == State.DONE, prof
            if cmd.publish and cmd.state == State.RUNNING:
                yaw = pose.yaw + cmd.vyaw * DT
                lat = creep * DT if cmd.vx > 0 else 0.0
                if cmd.vx == 0 and abs(cmd.vyaw) > 0 and turned < shift:
                    lat, turned = shift / 20, turned + shift / 20   # shoved sideways mid-turn
                pose = Pose2D(pose.x + cmd.vx * DT * math.cos(yaw) - lat * math.sin(yaw),
                              pose.y + cmd.vx * DT * math.sin(yaw) + lat * math.cos(yaw), yaw)
            if m.index >= 1 and turned >= shift:
                for xs in (2.0, 1.5, 1.0):
                    if xs not in prof and pose.x <= xs:
                        prof[xs] = abs(pose.y)
            t += DT
        return False, prof

    def test_turn_shift_is_pulled_back_to_the_centre(self):
        done, prof = self._return_leg(lookahead=0.6)
        self.assertTrue(done)
        # a metre into the return leg it is back near the centre line
        self.assertLess(prof[2.0], 0.04)
        self.assertLess(prof[1.5], 0.02)

    def test_corner_aiming_keeps_the_shift(self):
        # the model reproduces the Gazebo failure without the fix: aiming at
        # the corner only shrinks the offset in proportion to distance left
        _, old = self._return_leg(lookahead=0.0)
        _, new = self._return_leg(lookahead=0.6)
        self.assertGreater(old[2.0], 0.04)        # still ~5 cm off a metre in
        self.assertLess(new[2.0], old[2.0] / 4)   # line-following: well under 1 cm

    def test_steady_creep_stays_bounded(self):
        # a constant sideways creep leaves a steady offset ~ creep * lookahead / speed
        done, prof = self._return_leg(lookahead=0.6, creep=0.03)
        self.assertTrue(done)
        self.assertLess(max(prof.values()), 0.10)


class TestPauses(unittest.TestCase):
    def test_obstacle_pauses_and_holds_zero(self):
        m = new_mission()
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, front=10, near=0)
        self.assertEqual(cmd.state, State.PAUSED)
        self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))
        self.assertTrue(cmd.publish)

    def test_front_lane_does_not_stop_a_turn_on_the_spot(self):
        # facing a wall with the next waypoint behind: it must be able to turn
        m = Mission([(-2.0, 0.0)], Pose2D(0, 0, 0), 0.0, FP, SafetyParams())
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, front=50, near=0)
        self.assertEqual(cmd.state, State.RUNNING)
        self.assertEqual(cmd.vx, 0.0)
        self.assertNotEqual(cmd.vyaw, 0.0)

    def test_remembered_points_stop_a_turn_on_the_spot(self):
        # something behind, out of the LiDAR's sight but remembered
        m = Mission([(-2.0, 0.0)], Pose2D(0, 0, 0), 0.0, FP, SafetyParams())
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, turn_near=20)
        self.assertEqual(cmd.state, State.PAUSED)
        self.assertIn("no room to turn", cmd.reason)

    def test_remembered_points_dont_stop_walking(self):
        m = Mission([(2.0, 0.0)], Pose2D(0, 0, 0), 0.0, FP, SafetyParams())
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, turn_near=20)
        self.assertEqual(cmd.state, State.RUNNING)
        self.assertGreater(cmd.vx, 0)

    def test_ring_stops_a_turn_on_the_spot(self):
        m = Mission([(-2.0, 0.0)], Pose2D(0, 0, 0), 0.0, FP, SafetyParams())
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, front=0, near=50)
        self.assertEqual(cmd.state, State.PAUSED)

    def test_skip_waypoint_only_from_a_pause_with_the_ring_clear(self):
        m = new_mission()
        self.assertFalse(m.skip_waypoint(0.0, 0))           # not paused
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)   # walking into something
        self.assertEqual(m.state, State.PAUSED)
        self.assertFalse(m.skip_waypoint(0.1, 50))          # someone beside it
        self.assertTrue(m.skip_waypoint(0.1, 0))
        self.assertEqual((m.state, m.index), (State.RUNNING, 1))

    def test_skip_last_waypoint_is_done(self):
        m = Mission([(1.0, 0.0)], Pose2D(0, 0, 0), 0.0, FP, SafetyParams())
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)
        self.assertTrue(m.skip_waypoint(0.1, 0))
        self.assertEqual(m.state, State.DONE)

    def test_obstacle_below_threshold_ignored(self):
        m = new_mission(min_obstacle_points=5)
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, front=4, near=0)
        self.assertEqual(cmd.state, State.RUNNING)

    def test_stays_paused_after_obstacle_clears(self):
        m = new_mission()
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)
        for i in range(1, 50):
            cmd = m.step(i * DT, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0)
            self.assertEqual(cmd.state, State.PAUSED)

    def test_resume_after_clear(self):
        m = new_mission()
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)
        ok, _ = m.resume(FRESH, FRESH, 0, 0)
        self.assertTrue(ok)
        self.assertEqual(m.step(0.1, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0).state,
                         State.RUNNING)

    def test_resume_refused_while_blocked(self):
        m = new_mission()
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)
        ok, msg = m.resume(FRESH, FRESH, 10, 0)
        self.assertFalse(ok)
        self.assertIn("blocked", msg)

    def test_resume_refused_when_blind(self):
        m = new_mission()
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)
        self.assertFalse(m.resume(FRESH, 5.0, 0, 0)[0])
        self.assertFalse(m.resume(5.0, FRESH, 0, 0)[0])

    def test_resume_when_not_paused_refused(self):
        self.assertFalse(new_mission().resume(FRESH, FRESH, 0, 0)[0])

    def test_auto_resume_only_after_clear_window(self):
        m = new_mission(auto_resume_s=2.0)
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)
        self.assertEqual(m.step(1.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0).state, State.PAUSED)
        self.assertEqual(m.step(2.5, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0).state, State.PAUSED)
        self.assertEqual(m.step(3.1, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0).state, State.RUNNING)

    def test_auto_resume_window_restarts_if_blocked_again(self):
        m = new_mission(auto_resume_s=2.0)
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)
        m.step(1.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0)
        m.step(1.5, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)     # blocked again
        self.assertEqual(m.step(3.2, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0).state, State.PAUSED)

    def test_stale_odometry_pauses(self):
        cmd = new_mission().step(0.0, Pose2D(0, 0, 0), 2.0, FRESH, 0, 0)
        self.assertEqual(cmd.state, State.PAUSED)
        self.assertIn("odometry", cmd.reason)

    def test_no_pose_pauses(self):
        self.assertEqual(new_mission().step(0.0, None, FRESH, FRESH, 0, 0).state, State.PAUSED)

    def test_stale_lidar_pauses(self):
        cmd = new_mission().step(0.0, Pose2D(0, 0, 0), FRESH, 3.0, 0, 0)
        self.assertEqual(cmd.state, State.PAUSED)
        self.assertIn("blind", cmd.reason)

    def test_stale_lidar_never_auto_resumes(self):
        m = new_mission(auto_resume_s=0.5)
        m.step(0.0, Pose2D(0, 0, 0), FRESH, 3.0, 0, 0)
        for i in range(1, 30):
            self.assertEqual(m.step(i * DT, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0).state,
                             State.PAUSED)

    def test_obstacle_mid_loop_stops_robot(self):
        start = Pose2D(0, 0, 0)
        m = new_mission(start=start)
        wall = lambda t, p: (20, 0) if p.x > 2.0 else (0, 0)
        end, log = simulate(m, start, steps=400, obstacle=wall)
        self.assertEqual(m.state, State.PAUSED)
        self.assertLess(end.x, 2.1)
        paused = [c for _, _, c in log if c.state == State.PAUSED]
        self.assertTrue(all(c.vx == 0 and c.vyaw == 0 for c in paused))


class TestStall(unittest.TestCase):
    """2026-10-31: laid down mid-run, the route kept commanding a walk."""

    def test_commanded_but_not_moving_pauses(self):
        m = new_mission(stall_s=5.0)
        pose = Pose2D(0, 0, 0)                       # never moves
        t, cmd = 0.0, None
        while t < 6.0:
            cmd = m.step(t, pose, FRESH, FRESH, 0, 0)
            if cmd.state != State.RUNNING:
                break
            t += 0.1
        self.assertEqual(cmd.state, State.PAUSED)
        self.assertIn("not moving", cmd.reason)
        self.assertGreaterEqual(t, 5.0)
        self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))
        # and it stays paused: standing it up again doesn't restart the walk
        later = m.step(t + 10.0, pose, FRESH, FRESH, 0, 0)
        self.assertEqual(later.state, State.PAUSED)

    def test_progress_keeps_it_running(self):
        m = new_mission(stall_s=5.0)
        pose, t = Pose2D(0, 0, 0), 0.0
        for _ in range(100):                         # 10 s, creeping 1 cm per 0.1 s
            cmd = m.step(t, pose, FRESH, FRESH, 0, 0)
            self.assertNotEqual(cmd.state, State.PAUSED, cmd.reason)
            pose = Pose2D(pose.x + 0.01 * math.cos(pose.yaw),
                          pose.y + 0.01 * math.sin(pose.yaw), pose.yaw + 0.02 * cmd.vyaw)
            t += 0.1

    def test_resume_after_stall(self):
        m = new_mission(stall_s=1.0)
        pose = Pose2D(0, 0, 0)
        for i in range(15):
            m.step(i * 0.1, pose, FRESH, FRESH, 0, 0)
        self.assertEqual(m.state, State.PAUSED)
        ok, _ = m.resume(FRESH, FRESH, 0, 0)
        self.assertTrue(ok)
        self.assertEqual(m.step(1.6, pose, FRESH, FRESH, 0, 0).state, State.RUNNING)

    def test_off_when_zero(self):
        m = new_mission(stall_s=0.0)
        for i in range(100):
            cmd = m.step(i * 0.1, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0)
        self.assertEqual(cmd.state, State.RUNNING)


class TestAborts(unittest.TestCase):
    def test_max_duration_aborts(self):
        m = new_mission(max_duration_s=10.0)
        cmd = m.step(11.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0)
        self.assertEqual(cmd.state, State.ABORTED)

    def test_max_duration_aborts_even_when_paused(self):
        m = new_mission(max_duration_s=10.0)
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0)
        self.assertEqual(m.step(11.0, Pose2D(0, 0, 0), FRESH, FRESH, 10, 0).state,
                         State.ABORTED)

    def test_geofence_aborts(self):
        m = new_mission(geofence_m=6.0)
        cmd = m.step(0.0, Pose2D(7.0, 0, 0), FRESH, FRESH, 0, 0)
        self.assertEqual(cmd.state, State.ABORTED)
        self.assertIn("geofence", cmd.reason)

    def test_operator_abort(self):
        m = new_mission()
        self.assertEqual(m.abort(0.0).state, State.ABORTED)

    def test_abort_is_terminal(self):
        m = new_mission()
        m.abort(0.0)
        self.assertFalse(m.resume(FRESH, FRESH, 0, 0)[0])
        cmd = m.step(0.5, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0)
        self.assertEqual(cmd.state, State.ABORTED)
        self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))

    def test_holds_zero_then_goes_silent(self):
        m = new_mission()
        m.abort(0.0)
        self.assertTrue(m.step(0.5, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0).publish)
        self.assertFalse(m.step(1.5, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0).publish)

    def test_empty_waypoints_refused(self):
        with self.assertRaises(ValueError):
            Mission([], Pose2D(0, 0, 0), 0.0)


class TestCommandType(unittest.TestCase):
    def test_command_is_immutable(self):
        c = Command(0.1, 0.0, True, State.RUNNING)
        with self.assertRaises(Exception):
            c.vx = 5.0  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main(verbosity=2)
