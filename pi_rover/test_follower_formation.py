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

    def test_follower_stays_on_directly_behind_target(self) -> None:
        plan = FormationPlan()
        tracker = FormationTracker(plan)
        leader = LeaderFrame(status(), 1.0, "192.168.1.166")
        follower = status(mode="idle", x_mm=0.0, y_mm=0.0, heading_deg=0.0)
        control = tracker.control(leader, follower)
        self.assertEqual((control.target_x_mm, control.target_y_mm), (-400.0, 0.0))
        self.assertEqual((control.forward_error_mm, control.lateral_error_mm), (0.0, 0.0))
        self.assertEqual((control.left_mm_per_second, control.right_mm_per_second), (0, 0))

    def test_lateral_error_commands_a_corrective_turn(self) -> None:
        tracker = FormationTracker(FormationPlan())
        leader = LeaderFrame(status(), 1.0, "192.168.1.166")
        # F1 is 100 mm to the right of its desired behind-Leader location.
        follower = status(mode="idle", x_mm=0.0, y_mm=100.0, heading_deg=0.0)
        control = tracker.control(leader, follower)
        self.assertLess(control.lateral_error_mm, 0.0)
        self.assertGreater(control.left_mm_per_second, control.right_mm_per_second)

    def test_only_safe_active_leader_can_move_follower(self) -> None:
        self.assertTrue(leader_is_safe_and_moving(LeaderFrame(status(), 1.0, "192.168.1.166")))
        self.assertFalse(leader_is_safe_and_moving(LeaderFrame(status(mode="idle"), 1.0, "192.168.1.166")))
        self.assertFalse(leader_is_safe_and_moving(LeaderFrame(status(fault_code=4), 1.0, "192.168.1.166")))


if __name__ == "__main__":
    unittest.main()
