"""Straight controller geometry tests; no serial port or motors are opened."""

import unittest

from leader_formation import parse_status_fields
from leader_straight_calibration import straight_control


BASE = (
    "idle", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0",
    "0", "1", "1", "1", "0", "0",
)


def status(x: float, y: float, heading: float = 0.0):
    fields = list(BASE)
    fields[1:4] = [str(x), str(y), str(heading)]
    return parse_status_fields(fields)


class StraightCalibrationTests(unittest.TestCase):
    def test_origin_commands_equal_forward_wheel_speeds(self) -> None:
        origin = status(0, 0)
        command = straight_control(origin, origin, 2000, 25)
        self.assertEqual((command.left_mm_per_second, command.right_mm_per_second), (25, 25))

    def test_positive_lateral_error_steers_back_toward_center(self) -> None:
        origin = status(0, 0)
        command = straight_control(status(500, 50), origin, 2000, 25)
        self.assertLess(command.desired_heading_deg, 0)
        self.assertGreater(command.left_mm_per_second, command.right_mm_per_second)

    def test_rotated_origin_uses_original_forward_axis(self) -> None:
        origin = status(100, 200, 90)
        command = straight_control(status(100, 700, 90), origin, 2000, 25)
        self.assertAlmostEqual(command.forward_mm, 500)
        self.assertAlmostEqual(command.lateral_mm, 0)

    def test_fixed_equal_wheels_never_adds_heading_correction(self) -> None:
        origin = status(0, 0)
        command = straight_control(status(120, 30, 7), origin, 400, 18,
                                   fixed_equal_wheels=True)
        self.assertEqual((command.left_mm_per_second, command.right_mm_per_second), (18, 18))
        self.assertEqual(command.desired_heading_deg, 0)
        self.assertEqual(command.heading_error_deg, -7)


if __name__ == "__main__":
    unittest.main()
