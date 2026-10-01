"""Pure protocol/control tests; they never open a serial port or UDP socket."""

import json
import unittest

from follower_formation import (
    FormationPlan,
    FormationTracker,
    LeaderFrame,
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
    def test_packet_requires_current_protocol_shape(self) -> None:
        payload = json.dumps({"type": "leader_state", "version": 1, "status": status().__dict__}).encode()
        frame = parse_leader_frame(payload, "192.168.1.166", 12.5)
        self.assertEqual(frame.status.mode, "velocity")
        self.assertEqual(frame.source_host, "192.168.1.166")

    def test_follower_stays_at_the_left_rear_target(self) -> None:
        plan = FormationPlan()
        tracker = FormationTracker(plan)
        leader = LeaderFrame(status(), 1.0, "192.168.1.166")
        follower = status(mode="idle", x_mm=0.0, y_mm=0.0, heading_deg=0.0)
        control = tracker.control(leader, follower)
        self.assertEqual((control.target_x_mm, control.target_y_mm), (-200.0, 200.0))
        self.assertAlmostEqual(control.leader_distance_mm, 200.0 * 2 ** 0.5)
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
        self.assertEqual((control.left_mm_per_second, control.right_mm_per_second), (30, 30))

    def test_large_bearing_error_blocks_tracking(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(), 1.0, "192.168.1.166")
        # F1 is 100 mm left of target, so the target is directly right (alpha=-90).
        follower = status(mode="idle", x_mm=0.0, y_mm=100.0, heading_deg=0.0)
        control = tracker.control(leader, follower)
        self.assertLess(control.alpha_deg, -60.0)
        self.assertFalse(control.tracking_safe)
        self.assertEqual(control.stop_reason, "target bearing error is too large")

    def test_target_motion_provides_feedforward_speed(self) -> None:
        tracker = FormationTracker(FormationPlan())
        follower = status(mode="idle", x_mm=0.0, y_mm=0.0, heading_deg=0.0)
        tracker.control(LeaderFrame(status(x_mm=0.0), 1.0, "192.168.1.166"), follower)
        control = tracker.control(LeaderFrame(status(x_mm=5.0), 1.1, "192.168.1.166"), follower)
        self.assertAlmostEqual(control.target_speed_mm_per_second, 50.0)
        self.assertEqual((control.left_mm_per_second, control.right_mm_per_second), (45, 45))

    def test_heading_correction_keeps_both_wheels_forward(self) -> None:
        tracker = FormationTracker(FormationPlan())
        follower = status(mode="idle", heading_deg=20.0)
        tracker.control(LeaderFrame(status(x_mm=0.0), 1.0, "192.168.1.166"), follower)
        control = tracker.control(LeaderFrame(status(x_mm=5.0), 1.1, "192.168.1.166"), follower)
        self.assertGreaterEqual(control.left_mm_per_second, 0)
        self.assertGreaterEqual(control.right_mm_per_second, 0)
        self.assertGreater(control.left_mm_per_second, control.right_mm_per_second)

    def test_only_safe_active_leader_can_move_follower(self) -> None:
        self.assertTrue(leader_is_safe_and_moving(LeaderFrame(status(), 1.0, "192.168.1.166")))
        self.assertFalse(leader_is_safe_and_moving(LeaderFrame(status(mode="idle"), 1.0, "192.168.1.166")))
        self.assertFalse(leader_is_safe_and_moving(LeaderFrame(status(fault_code=4), 1.0, "192.168.1.166")))


if __name__ == "__main__":
    unittest.main()
