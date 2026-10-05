"""Pure protocol/control tests; they never open a serial port or UDP socket."""

import json
import csv
import tempfile
import unittest
from pathlib import Path

from follower_formation import (
    FormationPlan,
    FormationTracker,
    FollowerCsvLogger,
    LeaderFrame,
    leader_is_safe_for_settle,
    leader_is_safe_and_moving,
    parse_leader_frame,
)
from leader_formation import RoverStatus


def status(**changes: object) -> RoverStatus:
    values: dict[str, object] = dict(
        mode="velocity", x_mm=0.0, y_mm=0.0, heading_deg=0.0,
        left_count=0, right_count=0, left_tps=0.0, right_tps=0.0,
        left_target_tps=0.0, right_target_tps=0.0, left_pwm=0, right_pwm=0,
        fault_code=0, encoder_preflight=1, imu_present=1, imu_calibrated=1,
        path_number=0, path_step=0,
    )
    values.update(changes)
    return RoverStatus(**values)  # type: ignore[arg-type]


class FollowerFormationTests(unittest.TestCase):
    def test_csv_logger_records_leader_follower_and_control_fields(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(x_mm=10.0), 1.0, "192.168.1.166", 50.0, 0.1, True)
        follower = status(mode="idle")
        control = tracker.control(leader, follower)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "f1.csv"
            logger = FollowerCsvLogger(path)
            logger.record("control_moving", 0.0, leader, follower, control)
            logger.close()
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["event"], "control_moving")
        self.assertEqual(rows[0]["leader_source_host"], "192.168.1.166")
        self.assertEqual(rows[0]["leader_x_mm"], "10.0")
        self.assertEqual(rows[0]["follower_encoder_preflight"], "1")
        self.assertEqual(rows[0]["control_tracking_safe"], "True")

    def test_packet_requires_current_protocol_shape(self) -> None:
        payload = json.dumps({"type": "leader_state", "version": 1, "status": status().__dict__}).encode()
        frame = parse_leader_frame(payload, "192.168.1.166", 12.5)
        self.assertEqual(frame.status.mode, "velocity")
        self.assertEqual(frame.source_host, "192.168.1.166")
        self.assertFalse(frame.has_motion_reference)

    def test_packet_accepts_leader_motion_reference(self) -> None:
        payload = json.dumps({
            "type": "leader_state",
            "version": 1,
            "status": status().__dict__,
            "motion": {
                "forward_mm_per_second": 50.0,
                "yaw_rate_radians_per_second": 0.12,
            },
        }).encode()
        frame = parse_leader_frame(payload, "192.168.1.166", 12.5)
        self.assertTrue(frame.has_motion_reference)
        self.assertEqual(frame.forward_mm_per_second, 50.0)
        self.assertEqual(frame.yaw_rate_radians_per_second, 0.12)

    def test_follower_stays_at_the_left_rear_target(self) -> None:
        plan = FormationPlan()
        tracker = FormationTracker(plan)
        leader = LeaderFrame(status(), 1.0, "192.168.1.166")
        follower = status(mode="idle", x_mm=0.0, y_mm=0.0, heading_deg=0.0)
        control = tracker.control(leader, follower)
        self.assertEqual((control.target_x_mm, control.target_y_mm), (-400.0, 400.0))
        self.assertAlmostEqual(control.leader_distance_mm, 400.0 * 2 ** 0.5)
        self.assertEqual((control.rho_mm, control.alpha_deg, control.beta_deg), (0.0, 0.0, 0.0))
        self.assertEqual((control.left_mm_per_second, control.right_mm_per_second), (0, 0))
        self.assertTrue(control.tracking_safe)

    def test_target_ahead_commands_forward_without_turn(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(), 1.0, "192.168.1.166")
        # F1 is 100 mm behind its left-rear virtual target.
        follower = status(mode="idle", x_mm=-100.0, y_mm=0.0, heading_deg=0.0)
        control = tracker.control(leader, follower)
        self.assertEqual((control.rho_mm, control.alpha_deg, control.beta_deg), (100.0, 0.0, 0.0))
        self.assertEqual((control.left_mm_per_second, control.right_mm_per_second), (15, 15))

    def test_lateral_error_is_steered_smoothly_without_reverse(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(), 1.0, "192.168.1.166", 50.0, 0.0, True)
        # F1 is 100 mm left of target.  A Leader-frame controller uses a
        # bounded preview angle instead of pivoting towards a sideways point.
        follower = status(mode="idle", x_mm=0.0, y_mm=100.0, heading_deg=0.0)
        for _ in range(8):
            control = tracker.control(leader, follower)
        self.assertLess(control.alpha_deg, -15.0)
        self.assertTrue(control.tracking_safe)
        self.assertEqual(control.stop_reason, "")
        self.assertGreaterEqual(control.left_mm_per_second, 0)
        self.assertGreaterEqual(control.right_mm_per_second, 0)
        self.assertGreater(control.left_mm_per_second, control.right_mm_per_second)

    def test_only_150_mm_collision_distance_hard_stops_tracking(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(), 1.0, "192.168.1.166")
        # F1 is 140 mm directly behind Leader in the shared experiment frame.
        follower = status(mode="idle", x_mm=260.0, y_mm=-400.0, heading_deg=0.0)
        control = tracker.control(leader, follower)
        self.assertLess(control.leader_distance_mm, 150.0)
        self.assertFalse(control.tracking_safe)
        self.assertEqual(control.stop_reason, "leader is too close")

    def test_target_motion_provides_feedforward_speed(self) -> None:
        tracker = FormationTracker(FormationPlan())
        follower = status(mode="idle", x_mm=0.0, y_mm=0.0, heading_deg=0.0)
        tracker.control(LeaderFrame(status(x_mm=0.0), 1.0, "192.168.1.166"), follower)
        control = tracker.control(LeaderFrame(status(x_mm=5.0), 1.1, "192.168.1.166"), follower)
        self.assertAlmostEqual(control.target_speed_mm_per_second, 50.0)
        self.assertEqual((control.left_mm_per_second, control.right_mm_per_second), (10, 10))

    def test_noisy_target_heading_does_not_feed_yaw_or_flip_wheels(self) -> None:
        tracker = FormationTracker(FormationPlan())
        follower = status(mode="idle", x_mm=-100.0, y_mm=0.0, heading_deg=0.0)
        first = tracker.control(LeaderFrame(status(x_mm=0.0, y_mm=0.0), 1.0, "192.168.1.166"), follower)
        second = tracker.control(LeaderFrame(status(x_mm=5.0, y_mm=20.0), 1.1, "192.168.1.166"), follower)
        third = tracker.control(LeaderFrame(status(x_mm=10.0, y_mm=-20.0), 1.2, "192.168.1.166"), follower)
        self.assertEqual(second.target_yaw_rate_radians_per_second, 0.0)
        self.assertEqual(third.target_yaw_rate_radians_per_second, 0.0)
        self.assertLessEqual(abs(second.left_mm_per_second - first.left_mm_per_second), 10)
        self.assertLessEqual(abs(second.right_mm_per_second - first.right_mm_per_second), 10)
        self.assertLessEqual(abs(third.left_mm_per_second - second.left_mm_per_second), 10)
        self.assertLessEqual(abs(third.right_mm_per_second - second.right_mm_per_second), 10)

    def test_passed_target_reduces_speed_instead_of_turning_back(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(), 1.0, "192.168.1.166", 50.0, 0.0, True)
        # F1 is 200 mm ahead of its virtual target but remains laterally and
        # directionally aligned.  It must slow rather than make a U-turn.
        follower = status(mode="idle", x_mm=200.0, y_mm=0.0, heading_deg=0.0)
        for _ in range(8):
            control = tracker.control(leader, follower)
        self.assertLess(control.longitudinal_error_mm, -190.0)
        self.assertEqual(control.left_mm_per_second, control.right_mm_per_second)
        self.assertGreaterEqual(control.left_mm_per_second, 0)
        self.assertLess(control.left_mm_per_second, 50)

    def test_large_error_has_less_positive_catchup_than_near_error(self) -> None:
        leader = LeaderFrame(status(), 1.0, "192.168.1.166", 50.0, 0.0, True)
        near_tracker = FormationTracker(FormationPlan())
        far_tracker = FormationTracker(FormationPlan())
        # Both F1 poses are behind their target.  The far pose must not receive
        # a larger catch-up speed than the near pose.
        near = status(mode="idle", x_mm=-100.0, y_mm=0.0, heading_deg=0.0)
        far = status(mode="idle", x_mm=-400.0, y_mm=0.0, heading_deg=0.0)
        for _ in range(10):
            near_control = near_tracker.control(leader, near)
            far_control = far_tracker.control(leader, far)
        self.assertGreater(near_control.left_mm_per_second, far_control.left_mm_per_second)

    def test_leader_yaw_reference_uses_filtered_measured_heading(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(heading_deg=0.0), 1.0, "192.168.1.166", 50.0, 0.10, True)
        follower = status(mode="idle", x_mm=0.0, y_mm=0.0, heading_deg=0.0)
        tracker.control(leader, follower)
        control = tracker.control(
            LeaderFrame(status(heading_deg=1.0), 1.1, "192.168.1.166", 50.0, 0.10, True),
            follower,
        )
        self.assertGreater(control.target_yaw_rate_radians_per_second, 0.0)
        self.assertLess(control.target_yaw_rate_radians_per_second, 0.10)
        self.assertEqual(control.target_speed_mm_per_second, 50.0)
        self.assertGreater(control.right_mm_per_second, control.left_mm_per_second)

    def test_settle_phase_crawls_to_fix_lateral_error_before_heading(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(
            status(mode="idle"), 1.0, "192.168.1.166",
            0.0, 0.0, True, "settle",
        )
        # F1 is to the right of the target and points 45 degrees too far left.
        # Position priority must still produce a left-turning crawl, not zero.
        follower = status(mode="idle", x_mm=0.0, y_mm=-100.0, heading_deg=45.0)
        for _ in range(8):
            control = tracker.control(leader, follower)
        self.assertGreater(control.rho_mm, 55.0)
        self.assertGreater(control.right_mm_per_second, control.left_mm_per_second)
        self.assertGreater(control.right_mm_per_second, 0)

    def test_10_mm_position_deadband_leaves_only_heading_control(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(), 1.0, "192.168.1.166", 50.0, 0.0, True)
        # F1 is 5 mm from the virtual target but has a 20 degree heading error.
        follower = status(mode="idle", x_mm=5.0, y_mm=0.0, heading_deg=20.0)
        for _ in range(8):
            control = tracker.control(leader, follower)
        self.assertLess(control.rho_mm, 10.0)
        self.assertEqual(control.alpha_deg, 0.0)
        self.assertLess(control.right_mm_per_second, control.left_mm_per_second)

    def test_heading_correction_keeps_both_wheels_forward(self) -> None:
        tracker = FormationTracker(FormationPlan())
        follower = status(mode="idle", heading_deg=20.0)
        for index in range(6):
            tracker.control(LeaderFrame(status(x_mm=index * 5.0), 1.0 + index * 0.1, "192.168.1.166"), follower)
        control = tracker.control(LeaderFrame(status(x_mm=30.0), 1.6, "192.168.1.166"), follower)
        self.assertGreaterEqual(control.left_mm_per_second, 0)
        self.assertGreaterEqual(control.right_mm_per_second, 0)
        self.assertGreater(control.left_mm_per_second, control.right_mm_per_second)

    def test_only_safe_active_leader_can_move_follower(self) -> None:
        self.assertTrue(leader_is_safe_and_moving(LeaderFrame(status(), 1.0, "192.168.1.166")))
        self.assertFalse(leader_is_safe_and_moving(LeaderFrame(status(mode="idle"), 1.0, "192.168.1.166")))
        self.assertFalse(leader_is_safe_and_moving(LeaderFrame(status(fault_code=4), 1.0, "192.168.1.166")))

    def test_only_safe_idle_leader_can_offer_settle_phase(self) -> None:
        frame = LeaderFrame(status(mode="idle"), 1.0, "192.168.1.166", formation_phase="settle")
        self.assertTrue(leader_is_safe_for_settle(frame))
        self.assertFalse(leader_is_safe_for_settle(LeaderFrame(status(), 1.0, "192.168.1.166")))


if __name__ == "__main__":
    unittest.main()
