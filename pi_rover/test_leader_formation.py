"""Protocol-only tests; they do not open a serial port or command motors."""

import unittest

from leader_formation import NanoLink, parse_status_fields


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


if __name__ == "__main__":
    unittest.main()
