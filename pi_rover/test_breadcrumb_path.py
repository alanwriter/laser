"""Breadcrumb (recorded Leader track) path and its follower integration."""

import json
import math
import unittest

from breadcrumb_path import BreadcrumbPath, LeaderTarget
from follower_formation import (
    FOLLOWER_1,
    BreadcrumbFormationTracker,
    FormationPlan,
    LeaderFrame,
    PathFormationTracker,
    frame_has_reference,
    make_tracker,
    parse_leader_frame,
)
from leader_formation import leader_state_payload, next_target_payload
from simulate_formation import status
from turn_path import TurnPath, TurnPathReference


def feed(path: BreadcrumbPath, route: TurnPath, end_s: float, step: float = 4.0) -> None:
    s = 0.0
    while s <= end_s:
        x, y, heading, _ = route.pose_at_s(s, 0.0)
        path.observe(x, y, heading)
        s += step


ROUTE_D = TurnPath(TurnPathReference(600.0, 800.0, 400.0, 0.0, 0.0, 0.0))


class BreadcrumbPathTests(unittest.TestCase):
    def test_straight_lane_and_backward_extension(self) -> None:
        path = BreadcrumbPath()
        feed(path, ROUTE_D, 500.0)
        x, y, heading, curvature = path.pose_at_s(250.0, 400.0)
        self.assertAlmostEqual(x, 250.0, places=6)
        self.assertAlmostEqual(y, 400.0, places=6)
        self.assertAlmostEqual(heading, 0.0, places=6)
        self.assertAlmostEqual(curvature, 0.0, places=9)
        # Follower starts 400 mm behind the first Leader pose.
        x, y, _, _ = path.pose_at_s(-400.0, -400.0)
        self.assertAlmostEqual(x, -400.0, places=6)
        self.assertAlmostEqual(y, -400.0, places=6)

    def test_arc_curvature_and_lane_match_analytic_route(self) -> None:
        path = BreadcrumbPath()
        feed(path, ROUTE_D, 1800.0)
        s = 600.0 + 800.0 * math.pi / 4
        self.assertAlmostEqual(path.centre_curvature_at_s(s), 1 / 800.0, delta=0.02 / 800.0)
        for offset in (400.0, -400.0):
            bx, by, bh, bk = path.pose_at_s(s, offset)
            ax, ay, ah, ak = ROUTE_D.pose_at_s(s, offset)
            self.assertLess(math.hypot(bx - ax, by - ay), 2.0)
            self.assertAlmostEqual(bh, ah, delta=0.5)
            self.assertAlmostEqual(bk, ak, delta=0.03 * ak)
            self.assertAlmostEqual(path.speed_ratio_at_s(s, offset),
                                   ROUTE_D.speed_ratio_at_s(s, offset), delta=0.02)

    def test_position_tangent_source_matches_arc(self) -> None:
        path = BreadcrumbPath(tangent_source="position", smoothing_window_mm=300.0)
        feed(path, ROUTE_D, 1800.0)
        s = 600.0 + 800.0 * math.pi / 4
        self.assertAlmostEqual(path.centre_curvature_at_s(s), 1 / 800.0, delta=0.03 / 800.0)
        self.assertAlmostEqual(path.pose_at_s(s, 0.0)[2], 45.0, delta=0.5)
        self.assertAlmostEqual(path.centre_curvature_at_s(300.0), 0.0, delta=1e-6)
        with self.assertRaises(ValueError):
            BreadcrumbPath(tangent_source="spline")

    def test_projection_matches_analytic_route(self) -> None:
        path = BreadcrumbPath()
        feed(path, ROUTE_D, 1800.0)
        for s, offset, lateral in ((300.0, 400.0, 25.0), (1200.0, -400.0, -30.0), (1200.0, 400.0, 10.0)):
            px, py, heading, _ = ROUTE_D.pose_at_s(s, offset)
            h = math.radians(heading)
            world = (px - lateral * math.sin(h), py + lateral * math.cos(h))
            bs, blat = path.project(*world, offset)
            self.assertAlmostEqual(bs, s, delta=3.0)
            self.assertAlmostEqual(blat, lateral, delta=2.0)

    def test_spacing_head_progress_and_jump_rejection(self) -> None:
        path = BreadcrumbPath(spacing_mm=20.0)
        path.observe(0.0, 0.0, 0.0)
        path.observe(15.0, 0.0, 0.0)
        self.assertEqual(path.point_count, 1)
        self.assertAlmostEqual(path.leader_s_mm, 15.0)
        path.observe(25.0, 0.0, 0.0)
        self.assertEqual(path.point_count, 2)
        self.assertAlmostEqual(path.leader_s_mm, 25.0)
        with self.assertRaises(ValueError):
            path.observe(900.0, 0.0, 0.0)

    def test_heading_unwraps_across_180(self) -> None:
        path = BreadcrumbPath()
        for i in range(40):
            heading = 170.0 + i * 0.5
            x, y = i * 20.0, 0.0
            path.observe(x, y, (heading + 180) % 360 - 180)
        _, _, heading, _ = path.pose_at_s(path.end_s_mm - 100, 0.0)
        self.assertGreater(heading, 170.0)

    def test_prune_keeps_recent_points(self) -> None:
        path = BreadcrumbPath(keep_behind_mm=500.0)
        feed(path, ROUTE_D, 1500.0)
        before = path.point_count
        path.prune(1200.0)
        self.assertLess(path.point_count, before)
        x, y, _, _ = path.pose_at_s(1000.0, 0.0)
        ax, ay, _, _ = ROUTE_D.pose_at_s(1000.0, 0.0)
        self.assertLess(math.hypot(x - ax, y - ay), 1.0)

    def test_extension_bends_toward_next_target(self) -> None:
        path = BreadcrumbPath()
        feed(path, ROUTE_D, 700.0)
        tx, ty, th, _ = ROUTE_D.pose_at_s(1000.0, 0.0)
        path.observe(*ROUTE_D.pose_at_s(700.0, 0.0)[:3], LeaderTarget(tx, ty, th, 300.0))
        _, _, _, curvature = path.pose_at_s(path.end_s_mm + 200.0, 0.0)
        self.assertGreater(curvature, 0.5 / 800.0)


class BreadcrumbProtocolTests(unittest.TestCase):
    def test_next_target_round_trip_and_old_packets(self) -> None:
        target = next_target_payload(ROUTE_D, 600.0)
        packet = json.dumps(leader_state_payload(status(), 38.0, 0.0, next_target=target)).encode()
        frame = parse_leader_frame(packet, "192.168.1.166", 1.0)
        self.assertIsNotNone(frame.next_target)
        self.assertEqual(frame.next_target.ahead_mm, 300.0)
        old = parse_leader_frame(json.dumps(leader_state_payload(status(), 38.0)).encode(), "h", 1.0)
        self.assertIsNone(old.next_target)

    def test_rejects_invalid_next_target(self) -> None:
        bad = leader_state_payload(status(), 38.0, 0.0,
                                   next_target={"x_mm": 1, "y_mm": 2, "heading_deg": 0, "ahead_mm": -5})
        with self.assertRaises(ValueError):
            parse_leader_frame(json.dumps(bad).encode(), "h", 1.0)


class BreadcrumbTrackerTests(unittest.TestCase):
    def frame(self, t: float, x: float, with_path: bool) -> LeaderFrame:
        reference = TurnPathReference(600.0, 800.0, 400.0, 0, 0, 0, progress_s_mm=x)
        return LeaderFrame(status=status(x, 0.0, 0.0), received_monotonic_s=t, source_host="h",
                           forward_mm_per_second=38.0, has_motion_reference=True,
                           path=reference if with_path else None)

    def test_breadcrumb_tracker_needs_no_analytic_path(self) -> None:
        plan = FormationPlan()
        tracker = BreadcrumbFormationTracker(plan)
        for k in range(20):
            control = tracker.control(self.frame(k * 0.1, k * 3.8, False), status())
        self.assertTrue(control.tracking_safe)
        self.assertEqual(control.path_source, "breadcrumb")
        self.assertGreater(control.left_mm_per_second, 0)
        self.assertGreater(control.breadcrumb_points, 1)
        with self.assertRaises(ValueError):
            PathFormationTracker(plan).control(self.frame(0.0, 0.0, False), status())

    def test_reference_gate_and_factory(self) -> None:
        plan = FormationPlan()
        breadcrumb = make_tracker(plan, FOLLOWER_1, "breadcrumb")
        analytic = make_tracker(plan, FOLLOWER_1, "analytic")
        no_path = self.frame(0.0, 0.0, False)
        self.assertTrue(frame_has_reference(no_path, breadcrumb, FOLLOWER_1))
        self.assertFalse(frame_has_reference(no_path, analytic, FOLLOWER_1))
        with self.assertRaises(ValueError):
            make_tracker(plan, FOLLOWER_1, "spline")


if __name__ == "__main__":
    unittest.main()
