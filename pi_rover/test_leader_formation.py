"""Protocol-only tests; they do not open a serial port or command motors."""

import math
import csv
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from leader_formation import (
    NanoLink,
    NanoGeometry,
    WaveCsvLogger,
    WavePlan,
    build_parser,
    parse_status_fields,
    wave_control,
    turn_control,
)
from wave_path import WavePath, WavePathReference
from turn_path import TurnPath, TurnPathReference


STATUS = (
    "idle", "12.5", "-4.0", "90.0", "100", "101", "10.0", "10.1",
    "11.0", "11.0", "80", "81", "0", "1", "1", "1", "0", "0",
)


class LeaderProtocolTests(unittest.TestCase):
    def test_turn_geometry_and_leader_control(self) -> None:
        reference = TurnPathReference(1500.0, 1200.0, 1000.0, 0.0, 0.0, 0.0)
        route = TurnPath(reference)
        halfway = 1500 + 1200 * math.pi / 4
        x, y, heading, curvature = route.pose_at_s(halfway)
        self.assertAlmostEqual(heading, 45.0)
        self.assertAlmostEqual(curvature, 1 / 1200)
        self.assertAlmostEqual(route.project(x, y, 0.0)[0], halfway, delta=0.01)
        self.assertAlmostEqual(route.speed_ratio_at_s(halfway, 600), 0.5)
        self.assertAlmostEqual(route.speed_ratio_at_s(halfway, -600), 1.5)
        origin = parse_status_fields(("velocity", "0", "0", "0", *STATUS[4:]))
        point = parse_status_fields(("velocity", str(x), str(y), str(heading), *STATUS[4:]))
        control = turn_control(point, origin, WavePlan(route_kind="left_turn_v1", cruise_mm_per_second=30))
        self.assertLess(abs(control.heading_error_deg), 0.01)
        self.assertGreater(control.right_mm_per_second, control.left_mm_per_second)

    def test_compact_turn_ends_one_meter_forward_and_1_2_meter_left(self) -> None:
        reference = TurnPathReference(200.0, 800.0, 400.0, 0.0, 0.0, 0.0)
        route = TurnPath(reference)
        x, y, heading, _ = route.pose_at_s(reference.end_s_mm)
        self.assertAlmostEqual(x, 1000.0)
        self.assertAlmostEqual(y, 1200.0)
        self.assertAlmostEqual(heading, 90.0)
        self.assertAlmostEqual(reference.end_s_mm, 200 + 400 * math.pi + 400)

    def test_mpu_heading_gate_and_encoder_distance_use_reported_config(self) -> None:
        geometry = NanoGeometry.from_config_fields(
            ("1200", "1000", "65", "130", "0", "165", "1200", "2", "1.00")
        )
        geometry.require_mpu_heading()
        status = parse_status_fields(STATUS)
        left, right, average = geometry.wheel_distances_mm(status)
        self.assertAlmostEqual(left, 100 * math.pi * 65 / 1200)
        self.assertAlmostEqual(right, 101 * math.pi * 65 / 1000)
        self.assertAlmostEqual(average, (left + right) / 2)
        with self.assertRaisesRegex(RuntimeError, "does not confirm"):
            NanoGeometry.from_config_fields(("1200", "1000", "65", "130")).require_mpu_heading()
        with self.assertRaisesRegex(RuntimeError, "does not confirm"):
            NanoGeometry.from_config_fields(
                ("1200", "1000", "65", "130", "0", "165", "1200", "2", "0.70")
            ).require_mpu_heading()

    def test_transmitted_wave_geometry_matches_existing_leader_planner(self) -> None:
        plan = WavePlan()
        path = WavePath(WavePathReference(1500.0, 0.0, 50.0, 3, 0.0, 0.0, 0.0))
        origin_fields = list(STATUS)
        origin_fields[1:4] = ["0.0", "0.0", "0.0"]
        origin = parse_status_fields(origin_fields)
        for x in (0.0, 100.0, 250.0, 500.0, 750.0, 1000.0, 1250.0, 1500.0):
            y, slope, _ = path.shape(x)
            fields = list(origin_fields)
            fields[1:4] = [str(x), str(y), str(math.degrees(math.atan(slope)))]
            control = wave_control(parse_status_fields(fields), origin, plan)
            self.assertAlmostEqual(control.desired_lateral_mm, y, places=5)
            self.assertAlmostEqual(control.heading_error_deg, 0.0, places=5)

    def test_wave_cli_defaults_to_short_route_and_no_endpoint_chase(self) -> None:
        args = build_parser().parse_args(["wave", "--unlock"])
        self.assertEqual(args.length_mm, 1500.0)
        self.assertEqual(args.formation_settle_s, 0.0)

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
        # This point lies close to the default gentle (+/-50 mm) reference.
        fields[2] = "40.0"
        point = parse_status_fields(fields)
        control = wave_control(point, origin, WavePlan(length_mm=3000.0))
        self.assertAlmostEqual(control.forward_mm, 250.0)
        self.assertAlmostEqual(control.desired_lateral_mm, 38.490, places=3)
        self.assertGreater(control.desired_heading_deg, origin.heading_deg)
        self.assertLess(control.left_mm_per_second, control.right_mm_per_second)

    def test_wave_lead_in_is_straight_then_enters_smooth_sine(self) -> None:
        origin_fields = list(STATUS)
        origin_fields[1:4] = ["0.0", "0.0", "0.0"]
        origin = parse_status_fields(origin_fields)
        plan = WavePlan(length_mm=2000.0, lead_in_mm=400.0, amplitude_mm=150.0, cycles=1)
        straight_fields = list(origin_fields)
        straight_fields[1] = "400.0"
        straight = wave_control(parse_status_fields(straight_fields), origin, plan)
        self.assertAlmostEqual(straight.desired_lateral_mm, 0.0, places=4)
        self.assertAlmostEqual(straight.desired_heading_deg, 0.0, places=3)
        wave_fields = list(origin_fields)
        wave_fields[1] = "800.0"
        wave = wave_control(parse_status_fields(wave_fields), origin, plan)
        self.assertGreater(wave.desired_lateral_mm, 100.0)

    def test_300_mm_amplitude_is_allowed(self) -> None:
        plan = WavePlan(length_mm=2000.0, lead_in_mm=400.0, amplitude_mm=300.0, cycles=1)
        from leader_formation import validate_wave_plan
        validate_wave_plan(plan)

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

    def test_wave_csv_contains_raw_counts_and_calibrated_distances(self) -> None:
        geometry = NanoGeometry(1200.0, 1000.0, 65.0, 130.0, 1.0)
        with TemporaryDirectory() as directory:
            path = Path(directory) / "wave.csv"
            logger = WaveCsvLogger(path, WavePlan(), geometry)
            logger.record("control", parse_status_fields(STATUS), 1.0)
            logger.close()
            with path.open(newline="") as handle:
                row = next(csv.DictReader(handle))
        self.assertEqual(row["status_left_count"], "100")
        self.assertEqual(row["status_right_count"], "101")
        self.assertEqual(row["config_heading_gyro_blend"], "1.0")
        self.assertAlmostEqual(float(row["encoder_left_distance_mm"]), 100 * math.pi * 65 / 1200)


if __name__ == "__main__":
    unittest.main()
