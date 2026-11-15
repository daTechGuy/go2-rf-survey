"""Tests for the CMU autonomy stack adapter and S1 readiness checks (cmu.py).
Pure, stdlib-only, no ROS.

    python go2_survey_loop/test/test_cmu.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_loop.cmu import (  # noqa: E402
    STOP_ALL, STOP_FREE, active_target, assess_cmu, cmu_command, step_cmu,
)
from go2_survey_loop.explore import ExploreMission, ExploreParams  # noqa: E402
from go2_survey_loop.follower import FollowerParams, Mission, SafetyParams, State  # noqa: E402
from go2_survey_loop.geometry import Pose2D  # noqa: E402
from go2_survey_loop.route import RouteMission  # noqa: E402

FP = FollowerParams()
SAFETY = SafetyParams(stall_s=5.0)
FRESH = 0.02


class TestCmuCommand(unittest.TestCase):
    def test_running_emits_waypoint_and_zero_stop_mask(self):
        out = cmu_command(State.RUNNING, (2.5, -1.0), pose_z=0.32, was_running=True)
        self.assertEqual(out.waypoint, (2.5, -1.0, 0.32))
        self.assertEqual(out.stop_mask, STOP_FREE)
        self.assertFalse(out.send_sport_stop)

    def test_transition_from_running_to_paused_sends_stop_all_and_sport_stop(self):
        out = cmu_command(State.PAUSED, (2.5, -1.0), pose_z=0.32, was_running=True)
        self.assertIsNone(out.waypoint)
        self.assertEqual(out.stop_mask, STOP_ALL)
        self.assertTrue(out.send_sport_stop)

    def test_staying_paused_keeps_stop_all_without_repeating_sport_stop(self):
        out = cmu_command(State.PAUSED, (2.5, -1.0), pose_z=0.32, was_running=False)
        self.assertIsNone(out.waypoint)
        self.assertEqual(out.stop_mask, STOP_ALL)
        self.assertFalse(out.send_sport_stop)

    def test_running_without_target_between_explore_legs_holds_stop(self):
        out = cmu_command(State.RUNNING, None, pose_z=0.0, was_running=False)
        self.assertIsNone(out.waypoint)
        self.assertEqual(out.stop_mask, STOP_ALL)

    def test_non_finite_target_or_z_handled_safely(self):
        out_bad = cmu_command(State.RUNNING, (math.nan, 1.0), pose_z=0.3, was_running=True)
        self.assertIsNone(out_bad.waypoint)
        self.assertEqual(out_bad.stop_mask, STOP_ALL)
        self.assertTrue(out_bad.send_sport_stop)

        out_nan_z = cmu_command(State.RUNNING, (1.0, 2.0), pose_z=math.nan)
        self.assertEqual(out_nan_z.waypoint, (1.0, 2.0, 0.0))


class TestStepCmu(unittest.TestCase):
    def test_front_lane_delegated_to_cmu_while_clearance_ring_stops(self):
        m = RouteMission([(2.0, 0.0), (2.0, 2.0)], Pose2D(0, 0, 0), 0.0, FP, SAFETY)
        # Front lane blocked (front=12), but clearance ring clear (near=0):
        # CMU localPlanner steers round it, so mission stays RUNNING.
        cmd, out = step_cmu(m, 1.0, Pose2D(0.5, 0.0, 0.0), FRESH, FRESH,
                            front=12, near=0, pose_z=0.35, was_running=True)
        self.assertEqual(cmd.state, State.RUNNING)
        self.assertEqual(out.waypoint, (2.0, 0.0, 0.35))
        self.assertEqual(out.stop_mask, STOP_FREE)

        # Something enters the body clearance ring (near=6): pauses at once!
        cmd2, out2 = step_cmu(m, 1.1, Pose2D(0.6, 0.0, 0.0), FRESH, FRESH,
                              front=12, near=6, pose_z=0.35, was_running=True)
        self.assertEqual(cmd2.state, State.PAUSED)
        self.assertIsNone(out2.waypoint)
        self.assertEqual(out2.stop_mask, STOP_ALL)
        self.assertTrue(out2.send_sport_stop)

    def test_advances_waypoints_and_stops_on_done(self):
        m = Mission([(1.0, 0.0), (2.0, 0.0)], Pose2D(0, 0, 0), 0.0, FP, SAFETY)
        cmd1, out1 = step_cmu(m, 1.0, Pose2D(0.92, 0.0, 0.0), FRESH, FRESH, 0, 0, was_running=True)
        self.assertEqual(cmd1.state, State.RUNNING)
        self.assertEqual(out1.waypoint, (2.0, 0.0, 0.0))

        cmd2, out2 = step_cmu(m, 2.0, Pose2D(1.95, 0.0, 0.0), FRESH, FRESH, 0, 0, was_running=True)
        self.assertEqual(cmd2.state, State.DONE)
        self.assertIsNone(out2.waypoint)
        self.assertEqual(out2.stop_mask, STOP_ALL)
        self.assertTrue(out2.send_sport_stop)

    def test_explore_replans_when_cmu_stalls_at_front_obstacle_but_latches_in_open_floor(self):
        ep = ExploreParams(legs=3, strategy="greedy")
        m = ExploreMission(Pose2D(0, 0, 0), 0.0, FP, SAFETY, ep)
        obst = [(4.0, y / 5.0) for y in range(-15, 16)]
        # Plan first leg
        cmd, out = step_cmu(m, 0.1, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, obst)
        self.assertEqual(cmd.state, State.RUNNING)
        self.assertIsNotNone(out.waypoint)
        first_goal = m.leg.waypoints[-1]

        # Robot stalls for stall_s (5.0s) with an obstacle in front (front=8):
        # CMU localPlanner couldn't get past it -> mark goal spent and re-plan!
        step_cmu(m, 1.0, Pose2D(0.5, 0.0, 0.0), FRESH, FRESH, front=8, near=0, obstacles_xy=obst)
        cmd_replan, _ = step_cmu(m, 6.5, Pose2D(0.5, 0.0, 0.0), FRESH, FRESH,
                                 front=8, near=0, obstacles_xy=obst)
        self.assertEqual(cmd_replan.state, State.RUNNING)
        self.assertIn(first_goal, m._spent_goals)
        self.assertEqual(m.legs_done, 1)

        # Next leg starts, then robot stalls in open floor (front=0: e.g. laid down):
        # must latch PAUSED!
        step_cmu(m, 7.0, Pose2D(0.5, 0.0, 0.0), FRESH, FRESH, front=0, near=0, obstacles_xy=obst)
        cmd_stall, out_stall = step_cmu(m, 12.5, Pose2D(0.5, 0.0, 0.0), FRESH, FRESH,
                                        front=0, near=0, obstacles_xy=obst, was_running=True)
        self.assertEqual(cmd_stall.state, State.PAUSED)
        self.assertIn("stalled", cmd_stall.reason)
        self.assertEqual(out_stall.stop_mask, STOP_ALL)
        self.assertTrue(out_stall.send_sport_stop)

    def test_active_target_helper(self):
        self.assertIsNone(active_target(None))
        m = Mission([(1.5, -0.5)], Pose2D(0, 0, 0), 0.0, FP, SAFETY)
        self.assertEqual(active_target(m), (1.5, -0.5))


class TestAssessCmu(unittest.TestCase):
    def test_healthy_s1_snapshot(self):
        ok, lines = assess_cmu({
            "state_est_hz": 100.0,
            "state_est_frame": "map",
            "reg_scan_hz": 5.0,
            "reg_scan_frame": "map",
            "terrain_hz": 5.0,
            "path_hz": 10.0,
            "sport_pub_count": 0,
        }, sensing_only=True)
        self.assertTrue(ok)
        self.assertFalse(any(line.startswith(("FAIL", "WARN")) for line in lines))

    def test_warns_when_sport_request_active_in_s1(self):
        ok, lines = assess_cmu({
            "state_est_hz": 50.0,
            "state_est_frame": "map",
            "reg_scan_hz": 5.0,
            "reg_scan_frame": "map",
            "terrain_hz": 5.0,
            "path_hz": 10.0,
            "sport_pub_count": 1,
        }, sensing_only=True)
        self.assertTrue(ok)
        self.assertTrue(any("is_real_robot:=false" in line for line in lines))

    def test_fails_when_topics_missing_or_wrong_frame(self):
        ok, lines = assess_cmu({
            "state_est_hz": 0.0,
            "state_est_frame": "odom",
            "reg_scan_hz": 1.0,
            "terrain_hz": 0.0,
            "path_hz": 0.0,
        })
        self.assertFalse(ok)
        self.assertGreaterEqual(sum(1 for line in lines if line.startswith("FAIL")), 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
