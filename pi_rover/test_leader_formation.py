"""Protocol-only tests; they do not open a serial port or command motors."""

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from leader_formation import (
    NanoLink,
    WaveCsvLogger,
    WavePlan,
    parse_status_fields,
    wave_control,
)


STATUS = (
    "idle", "12.5", "-4.0", "90.0", "100", "101", "10.0", "10.1",
    "11.0", "11.0", "80", "81", "0", "1", "1", "1", "0", "0",
)


class LeaderProtocolTests(unittest.TestCase):
    def test_parses_status_with_all_preflight_flags(self) -> None:
        status = parse_status_fields(STATUS)
        self.assertTrue(status.ready_for_path)
        self.assertEqual((status.x_mm, status.y_mm, status.heading_deg), (12.5, -4.0, 90.0))

    def test_fault_blocks_a_path(self) -> None:
        fields = list(STATUS)
        fields[12] = "7"
        self.assertFalse(parse_status_fields(fields).ready_for_path)

    def test_parses_matching_hello_and_async_telemetry_shapes(self) -> None:
        hello = NanoLink.parse(b"IO,HELLO,1,1,115200,FIRMWARE_PROFILE=L1\n")
        telemetry = NanoLink.parse(b"IO,TELEMETRY,0," + ",".join(STATUS).encode() + b"\n")
        self.assertEqual((hello.operation, hello.sequence), ("HELLO", 1))
        self.assertEqual((telemetry.operation, telemetry.sequence), ("TELEMETRY", 0))
        self.assertTrue(parse_status_fields(telemetry.fields).ready_for_path)

    def test_ignores_non_io_boot_text(self) -> None:
        self.assertIsNone(NanoLink.parse(b"Nano booting...\n"))

    def test_wave_starts_with_zero_tangent_for_safe_follower_takeoff(self) -> None:
        origin = parse_status_fields(STATUS)
        control = wave_control(origin, origin, WavePlan())
        self.assertAlmostEqual(control.forward_mm, 0.0)
        self.assertAlmostEqual(control.desired_lateral_mm, 0.0)
        self.assertAlmostEqual(control.desired_heading_deg, origin.heading_deg, places=3)
        self.assertEqual(control.left_mm_per_second, control.right_mm_per_second)

    def test_wave_reference_is_smooth_and_positive_after_250_mm(self) -> None:
        origin_fields = list(STATUS)
        origin_fields[1] = "0.0"
        origin_fields[2] = "0.0"
        origin_fields[3] = "0.0"
        origin = parse_status_fields(origin_fields)
        fields = list(origin_fields)
        fields[1] = "250.0"
        fields[2] = "200.0"
        point = parse_status_fields(fields)
        control = wave_control(point, origin, WavePlan())
        self.assertAlmostEqual(control.forward_mm, 250.0)
        self.assertAlmostEqual(control.desired_lateral_mm, 153.960, places=3)
        self.assertGreater(control.desired_heading_deg, origin.heading_deg)
        self.assertLess(control.left_mm_per_second, control.right_mm_per_second)

    def test_wave_csv_log_records_status_and_control(self) -> None:
        status = parse_status_fields(STATUS)
        control = wave_control(status, status, WavePlan())
        with TemporaryDirectory() as directory:
            path = Path(directory) / "wave.csv"
            logger = WaveCsvLogger(path, WavePlan())
            logger.record("control", status, 1.25, control)
            logger.close()
            contents = path.read_text(encoding="utf-8")
        self.assertIn("control_heading_error_deg", contents)
        self.assertIn("control", contents)
        self.assertIn(",1.25,", contents)


if __name__ == "__main__":
    unittest.main()
