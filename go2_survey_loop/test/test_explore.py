"""Tests for greedy exploration. No ROS, no clock.

The centrepiece is a closed-loop run in a synthetic room with desks: a
unicycle robot driven only by ExploreMission's commands, its obstacle counts
computed by the real obstacles.classify from the room's points, must explore,
never come near a wall or desk, and finish back at the start. Also with a
robot that turns at half the commanded rate, like the real Go2.

    python go2_survey_loop/test/test_explore.py
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_loop.explore import (  # noqa: E402
    ExploreMission, ExploreParams, choose_leg, free_distance, robot_frame,
)
from go2_survey_loop.follower import FollowerParams, SafetyParams, State  # noqa: E402
from go2_survey_loop.geometry import Pose2D  # noqa: E402
from go2_survey_loop.obstacles import ObstacleParams, classify, ring_count  # noqa: E402

FP = FollowerParams()
EP = ExploreParams()
# the loop's stops as demo.sh --explore sets them
BODY = ObstacleParams(clearance_radius=0.70, stop_distance=0.70, lane_half_width=0.45)
FRESH = 0.05
DT = 0.1


def segment(x0, y0, x1, y1, step=0.05):
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def box(x0, y0, x1, y1):
    return (segment(x0, y0, x1, y0) + segment(x1, y0, x1, y1)
            + segment(x1, y1, x0, y1) + segment(x0, y1, x0, y0))


# A 7 m x 9 m room, robot starting 1 m in from one end, two desks and a
# cupboard placed so the straight line ahead is blocked.
ROOM = (box(-1.0, -3.5, 8.0, 3.5)
        + box(2.0, -0.6, 3.0, 0.6)          # desk dead ahead, 2 m out
        + box(4.5, 1.2, 5.7, 2.4)           # desk front-left
        + box(5.0, -3.5, 6.0, -2.4))        # cupboard against the right wall


def counts(points_xy, pose):
    return classify([(x, y, 0.5) for x, y in points_xy], pose, 0.0, BODY)


TURN_CLEARANCE = 0.70     # demo.sh --explore


def run(mission, pose, room, steps=6000, yaw_gain=1.0, extra=lambda t: []):
    """Closed loop. Returns (pose, log of (t, pose, cmd), closest approach).
    The turn check sees the whole room: a perfect memory."""
    log, t, closest = [], 0.0, math.inf
    for _ in range(steps):
        pts = room + extra(t)
        front, near = counts(pts, pose)
        turn_near = ring_count(pts, pose, TURN_CLEARANCE, BODY)
        cmd = mission.step(t, pose, FRESH, FRESH, front, near, pts, turn_near)
        log.append((t, pose, cmd))
        if cmd.publish and cmd.state == State.RUNNING:
            yaw = pose.yaw + yaw_gain * cmd.vyaw * DT
            pose = Pose2D(pose.x + cmd.vx * DT * math.cos(yaw),
                          pose.y + cmd.vx * DT * math.sin(yaw), yaw)
        closest = min(closest, min(math.hypot(x - pose.x, y - pose.y) for x, y in room))
        t += DT
        if cmd.state in (State.DONE, State.ABORTED) and not cmd.publish:
            break
    return pose, log, closest


def new_mission(start=Pose2D(0, 0, 0), legs=6, **safety):
    s = dict(max_duration_s=600.0, geofence_m=EP.radius + 1.0)
    s.update(safety)
    return ExploreMission(start, 0.0, FP, SafetyParams(**s),
                          ExploreParams(legs=legs), BODY)


class TestFreeDistance(unittest.TestCase):
    def test_wall_ahead(self):
        self.assertAlmostEqual(free_distance([(2.0, 0.1), (3.0, 0.0)], 0.0, 0.35, 8.0), 2.0)

    def test_open(self):
        self.assertEqual(free_distance([], 1.0, 0.35, 8.0), 8.0)

    def test_behind_and_beside_ignored(self):
        pts = [(-1.0, 0.0), (1.0, 0.5)]               # behind; outside the corridor
        self.assertEqual(free_distance(pts, 0.0, 0.35, 8.0), 8.0)

    def test_other_heading(self):
        self.assertAlmostEqual(free_distance([(0.0, 1.5)], math.pi / 2, 0.35, 8.0), 1.5)

    def test_robot_frame_drops_own_body(self):
        pose = Pose2D(1.0, 1.0, math.pi / 2)
        pts = robot_frame([(1.0, 1.1), (1.0, 3.0)], pose, BODY, 8.0)
        self.assertEqual(len(pts), 1)
        self.assertAlmostEqual(pts[0][0], 2.0)        # 2 m straight ahead
        self.assertAlmostEqual(pts[0][1], 0.0, places=9)


class TestChooseLeg(unittest.TestCase):
    def test_avoids_the_blocked_way(self):
        # wall 1.2 m ahead: no leg straight on (it couldn't keep its margin)
        wall = segment(1.2, -3, 1.2, 3)
        leg = choose_leg(Pose2D(0, 0, 0), wall, [(0, 0)], Pose2D(0, 0, 0))
        self.assertIsNotNone(leg)
        self.assertGreater(abs(leg.heading_rel), math.radians(30))

    def test_leg_keeps_its_margin(self):
        wall = segment(2.0, -3, 2.0, 3)
        leg = choose_leg(Pose2D(0, 0, 0), wall, [(0, 0)], Pose2D(0, 0, 0),
                         ExploreParams(turn_cost=5.0))      # make it want straight on
        self.assertAlmostEqual(leg.heading_rel, 0.0)
        self.assertLessEqual(leg.length, 2.0 - EP.margin + 1e-9)

    def test_boxed_in_returns_none(self):
        leg = choose_leg(Pose2D(0, 0, 0), box(-0.9, -0.9, 0.9, 0.9), [(0, 0)], Pose2D(0, 0, 0))
        self.assertIsNone(leg)

    def test_prefers_where_it_hasnt_been(self):
        visited = [(x / 10, 0.0) for x in range(0, 30)]     # walked 3 m along +x already
        leg = choose_leg(Pose2D(0, 0, 0), [], visited, Pose2D(0, 0, 0))
        tx, ty = leg.target
        self.assertGreater(min(math.hypot(tx - vx, ty - vy) for vx, vy in visited), 1.0)

    def test_stays_inside_the_radius(self):
        start = Pose2D(0, 0, 0)
        for x in (2.0, 3.0, 3.6):
            leg = choose_leg(Pose2D(x, 0, 0), [], [(0, 0)], start, ExploreParams(radius=4.0))
            if leg is not None:
                self.assertLessEqual(math.hypot(*leg.target), 4.0 + 1e-9)

    def test_leg_never_ends_beside_something(self):
        # a chair just off the corridor at 1.5 m: the full leg would end beside
        # it (sim, 2026-10-31: stuck, unable to turn). It must stop short.
        chair = box(1.3, 0.65, 1.7, 1.05)
        leg = choose_leg(Pose2D(0, 0, 0), chair, [(0, 0)], Pose2D(0, 0, 0),
                         ExploreParams(turn_cost=5.0))          # wants straight on
        self.assertIsNotNone(leg)
        tx, ty = leg.target
        self.assertGreater(min(math.hypot(tx - x, ty - y) for x, y in chair),
                           EP.end_clearance)

    def test_every_leg_end_can_turn(self):
        for x, y in [(0.0, 0.0), (3.5, -1.5), (6.0, 1.0)]:
            leg = choose_leg(Pose2D(x, y, 0.3), ROOM, [(x, y)], Pose2D(x, y, 0))
            if leg is None:
                continue
            tx, ty = leg.target
            self.assertGreater(min(math.hypot(tx - px, ty - py) for px, py in ROOM),
                               EP.end_clearance)

    def test_corridor_clears_the_ring(self):
        # planning corridor at least as wide as the clearance ring, so walking a
        # leg along a wall never trips the ring and stalls the exploration
        self.assertGreaterEqual(EP.corridor_half, BODY.clearance_radius)


class TestClosedLoop(unittest.TestCase):
    def check_run(self, yaw_gain):
        m = new_mission()
        end, log, closest = run(m, Pose2D(0, 0, 0), ROOM, yaw_gain=yaw_gain)
        self.assertEqual(m.state, State.DONE, m.reason)
        self.assertEqual(m.legs_done, 6)
        self.assertLess(math.hypot(end.x, end.y), FP.reach_tol + 0.1)     # home
        self.assertGreater(closest, BODY.clearance_radius - 0.05)         # never close
        xs = [p.x for _, p, _ in log]
        ys = [p.y for _, p, _ in log]
        self.assertGreater((max(xs) - min(xs)) * (max(ys) - min(ys)), 4.0)  # got about
        self.assertFalse(any(c.state == State.PAUSED for _, _, c in log))
        return m, log

    def test_explores_and_comes_home(self):
        self.check_run(1.0)

    def test_sluggish_turning_robot(self):
        self.check_run(0.5)          # turns at half the commanded rate, like the Go2

    def test_never_drives_outside_the_radius(self):
        m, log = self.check_run(1.0)
        far = max(math.hypot(p.x, p.y) for _, p, _ in log)
        self.assertLess(far, EP.radius + 0.3)


class TestInterruptions(unittest.TestCase):
    def test_blocked_mid_leg_replans(self):
        # someone steps in front of the robot during the first leg, then leaves
        m = new_mission(legs=3)
        person = []
        pose, t, replanned = Pose2D(0, 0, 0), 0.0, False
        for _ in range(3000):
            if 4.0 <= t < 4.1 and not person:
                c, s = math.cos(pose.yaw), math.sin(pose.yaw)
                person.extend(segment(pose.x + 0.8 * c - 0.2 * s, pose.y + 0.8 * s + 0.2 * c,
                                      pose.x + 0.8 * c + 0.2 * s, pose.y + 0.8 * s - 0.2 * c))
            if t >= 9.0:
                person.clear()
            pts = ROOM + person
            front, near = counts(pts, pose)
            cmd = m.step(t, pose, FRESH, FRESH, front, near, pts)
            if cmd.reason.startswith("blocked"):
                replanned = True
            if cmd.publish and cmd.state == State.RUNNING:
                yaw = pose.yaw + cmd.vyaw * DT
                pose = Pose2D(pose.x + cmd.vx * DT * math.cos(yaw),
                              pose.y + cmd.vx * DT * math.sin(yaw), yaw)
            t += DT
            if m.state in (State.DONE, State.ABORTED):
                break
        self.assertTrue(replanned)
        self.assertEqual(m.state, State.DONE, m.reason)

    def test_waits_while_something_is_beside_it(self):
        m = new_mission()
        beside = segment(0.0, 0.4, 0.3, 0.4)            # inside the 0.55 m ring
        pts = ROOM + beside
        front, near = counts(pts, Pose2D(0, 0, 0))
        self.assertGreaterEqual(near, 5)
        for i in range(30):
            cmd = m.step(i * DT, Pose2D(0, 0, 0), FRESH, FRESH, front, near, pts)
            self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))
            self.assertEqual(cmd.state, State.PAUSED)
        # gone: it carries on by itself (a person, not a reason to stop for good)
        front, near = counts(ROOM, Pose2D(0, 0, 0))
        cmd = m.step(3.1, Pose2D(0, 0, 0), FRESH, FRESH, front, near, ROOM)
        self.assertEqual(cmd.state, State.RUNNING)

    def test_blocked_on_the_way_home_waits_for_the_operator(self):
        # walking home, mid-way (not just short of a turn point, which it may
        # skip): someone stands in front. That waits for the remote.
        m = new_mission(legs=1)
        pose, t, cmd = Pose2D(0, 0, 0), 0.0, None
        while t < 120:
            front, near = counts(ROOM, pose)
            cmd = m.step(t, pose, FRESH, FRESH, front, near, ROOM)
            if m.phase == "home" and cmd.vx > 0 and \
                    math.hypot(m.leg.target[0] - pose.x, m.leg.target[1] - pose.y) > 0.8:
                break
            if cmd.state == State.RUNNING:
                yaw = pose.yaw + cmd.vyaw * DT
                pose = Pose2D(pose.x + cmd.vx * DT * math.cos(yaw),
                              pose.y + cmd.vx * DT * math.sin(yaw), yaw)
            t += DT
        self.assertEqual(m.phase, "home")
        self.assertGreater(cmd.vx, 0)
        t += DT
        cmd = m.step(t, pose, FRESH, FRESH, 20, 0, ROOM)          # blocked
        self.assertEqual(cmd.state, State.PAUSED)
        cmd = m.step(t + 30, pose, FRESH, FRESH, 0, 0, ROOM)      # clear, but latched
        self.assertEqual(cmd.state, State.PAUSED)
        ok, _ = m.resume(FRESH, FRESH, 0, 0)
        self.assertTrue(ok)
        self.assertEqual(m.step(t + 31, pose, FRESH, FRESH, 0, 0, ROOM).state, State.RUNNING)

    def test_stale_odometry_latches(self):
        m = new_mission()
        cmd = m.step(0.0, Pose2D(0, 0, 0), 5.0, FRESH, 0, 0, ROOM)
        self.assertEqual(cmd.state, State.PAUSED)
        cmd = m.step(1.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, ROOM)
        self.assertEqual(cmd.state, State.PAUSED)                 # stays until resumed
        ok, _ = m.resume(FRESH, FRESH, 0, 0)
        self.assertTrue(ok)
        self.assertEqual(m.step(1.1, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, ROOM).state,
                         State.RUNNING)

    def test_no_lidar_picture_no_plan(self):
        m = new_mission()
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, [])
        self.assertEqual(cmd.state, State.PAUSED)
        self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))
        self.assertIsNone(m.leg)
        self.assertEqual(m.step(0.1, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, ROOM).state,
                         State.RUNNING)

    def test_boxed_in_at_start_aborts(self):
        m = new_mission()
        tight = box(-0.9, -0.9, 0.9, 0.9)
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, tight)
        self.assertEqual(cmd.state, State.ABORTED)
        self.assertIn("nowhere", cmd.reason)


class TestNoRoomToTurn(unittest.TestCase):
    """Something close behind (out of the head LiDAR's sight, but remembered):
    no turning on the spot here -- only roughly straight on, or wait."""

    def test_only_straight_ahead_when_it_cannot_turn(self):
        # open floor ahead (+x along y = -2); the best leg with room to turn
        # would be elsewhere, but here it may only go roughly straight on
        pose = Pose2D(1.0, -2.0, 0.0)
        m = new_mission()
        cmd = m.step(0.0, pose, FRESH, FRESH, 0, 0, ROOM, turn_near=20)
        self.assertEqual(cmd.state, State.RUNNING, cmd.reason)
        self.assertIsNotNone(m.leg)
        tx, ty = m.leg.target
        self.assertLess(abs(math.atan2(ty - pose.y, tx - pose.x)), EP.straight_turn + 1e-6)
        self.assertFalse(cmd.vx == 0.0 and cmd.vyaw != 0.0)      # no turn on the spot

    def test_waits_for_the_operator_if_nothing_is_straight_ahead(self):
        m = new_mission()
        wall_ahead = ROOM + segment(1.2, -2, 1.2, 2)
        cmd = m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, wall_ahead, turn_near=20)
        self.assertEqual(cmd.state, State.PAUSED)
        self.assertIn("no room to turn", cmd.reason)
        # latched: room to turn later doesn't restart it by itself
        cmd = m.step(1.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, wall_ahead, turn_near=0)
        self.assertEqual(cmd.state, State.PAUSED)
        ok, _ = m.resume(FRESH, FRESH, 0, 0)
        self.assertTrue(ok)

    def test_leg_needing_a_turn_is_dropped_for_a_straight_one(self):
        m = new_mission()
        # plan with room to turn: a leg that starts with a turn on the spot
        pose = Pose2D(0, 0, math.pi)                   # facing the near wall
        m.step(0.0, pose, FRESH, FRESH, 0, 0, ROOM, turn_near=0)
        self.assertIsNotNone(m.leg)
        # then the memory fills behind it: the turn must not happen
        cmd = m.step(0.1, pose, FRESH, FRESH, 0, 0, ROOM, turn_near=20)
        self.assertEqual((cmd.vx, cmd.vyaw), (0.0, 0.0))
        self.assertIn("no room to turn", cmd.reason)


class TestLimits(unittest.TestCase):
    def test_time_limit(self):
        m = new_mission(max_duration_s=10.0)
        _, log, _ = run(m, Pose2D(0, 0, 0), ROOM, steps=400)
        self.assertEqual(m.state, State.ABORTED)
        self.assertIn("limit", m.reason)

    def test_geofence(self):
        m = new_mission(geofence_m=1.0)
        _, log, _ = run(m, Pose2D(0, 0, 0), ROOM, steps=2000)
        self.assertEqual(m.state, State.ABORTED)
        self.assertIn("geofence", m.reason)

    def test_terminal_goes_silent(self):
        m = new_mission(max_duration_s=1.0)
        m.step(0.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, ROOM)
        m.step(2.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, ROOM)
        self.assertFalse(m.step(10.0, Pose2D(0, 0, 0), FRESH, FRESH, 0, 0, ROOM).publish)


if __name__ == "__main__":
    unittest.main(verbosity=2)
