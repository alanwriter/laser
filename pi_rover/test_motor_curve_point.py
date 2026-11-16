"""Offline checks for the grounded one-point motor characterization command."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import motor_curve_point as curve


class FakeLink:
    def __init__(self) -> None:
        self.commands: list[str] = []

    def request(self, operation: str, *arguments, **_kwargs):
        self.commands.append(operation)
        return SimpleNamespace(raw=f"ACK {operation}", fields=())

    def stop_safely(self) -> None:
        self.commands.append("STOP")


def status(**overrides):
    values = dict(
        mode="manual", x_mm=0.0, y_mm=0.0, heading_deg=0.0,
        left_count=0, right_count=0, left_tps=0.0, right_tps=0.0,
        left_pwm=0, right_pwm=0, fault_code=0, encoder_preflight=1,
        imu_present=1, imu_calibrated=1, ready_for_path=True,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


class MotorCurveSafetyTest(unittest.TestCase):
    def test_heading_guard_rejects_unsafe_reading(self) -> None:
        with patch.object(curve, "parse_status_fields", return_value=status(heading_deg=20.1)):
            with self.assertRaisesRegex(RuntimeError, "heading guard"):
                curve.checked_status(FakeLink())

    def test_run_stops_when_sampling_fails(self) -> None:
        link = FakeLink()
        with tempfile.TemporaryDirectory() as temporary:
            with patch.object(curve, "commission"), patch.object(curve, "require_confirmation"), \
                 patch.object(curve, "checked_status", return_value=status()), \
                 patch.object(curve, "sample_phase", side_effect=RuntimeError("guard")):
                with self.assertRaisesRegex(RuntimeError, "guard"):
                    curve.run(link, "Leader", 60, 55, 1.2, 300, Path(temporary) / "point.csv")
        self.assertIn("STOP", link.commands)


if __name__ == "__main__":
    unittest.main()
