#!/usr/bin/env python3
"""Safety-first Raspberry Pi controller for the Nano rover firmware.

The Nano owns motor PI control, encoders, odometry, and all real-time safety.
This program only issues high-level commands over USB serial.  It never starts
motion on launch, and every movement requires an explicit --unlock flag plus a
typed confirmation.
"""

from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial


DEFAULT_PORT = (
    "/dev/serial/by-path/"
    "platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0"
)
BAUD_RATE = 115200

TRAJECTORIES = {
    "G1": ("500 mm straight line", 20.0),
    "G2": ("400 mm × 400 mm square", 60.0),
    "G3": ("350 mm L-shaped path", 35.0),
}


@dataclass
class Preflight:
    lines: list[str]
    errors: list[str]
    warnings: list[str]

    @property
    def passed(self) -> bool:
        return not self.errors


class NanoLink:
    def __init__(self, port: str, baud: int = BAUD_RATE) -> None:
        self.port = port
        self.baud = baud
        self.device: serial.Serial | None = None

    def open(self) -> None:
        if not Path(self.port).exists():
            raise RuntimeError(f"Nano serial device not found: {self.port}")
        self.device = serial.Serial(
            self.port, self.baud, timeout=0.15, write_timeout=1.0
        )
        # Opening the CH340/Nano USB serial port may reset the Nano.  Do not
        # send commands until its boot sequence has finished.
        time.sleep(2.0)
        boot_lines = self.read_for(0.4)
        if boot_lines:
            print_lines("Nano boot", boot_lines)

    def close(self) -> None:
        if self.device and self.device.is_open:
            self.device.close()

    def read_for(self, seconds: float) -> list[str]:
        if not self.device:
            raise RuntimeError("Serial port is not open")
        deadline = time.monotonic() + seconds
        lines: list[str] = []
        while time.monotonic() < deadline:
            raw = self.device.readline()
            if raw:
                lines.append(raw.decode("utf-8", errors="replace").strip())
        return lines

    def command(self, command: str, response_seconds: float = 0.8) -> list[str]:
        self.send(command)
        lines = self.read_for(response_seconds)
        print_lines("Nano", lines)
        return lines

    def send(self, command: str) -> None:
        """Transmit one complete Nano command without waiting for a reply."""
        if not self.device:
            raise RuntimeError("Serial port is not open")
        print(f">>> {command}")
        self.device.write((command + "\n").encode("ascii"))
        self.device.flush()

    def stop(self) -> None:
        """Best-effort motor stop; safe to call during exceptions."""
        try:
            self.command("S", response_seconds=0.35)
        except (RuntimeError, serial.SerialException, OSError):
            pass


def print_lines(label: str, lines: list[str]) -> None:
    if not lines:
        print(f"{label}: (no response)")
        return
    for line in lines:
        print(f"{label}: {line}")


def contains(lines: list[str], text: str) -> bool:
    return text.lower() in "\n".join(lines).lower()


def run_preflight(link: NanoLink) -> Preflight:
    """Stop first, then check the exact conditions required by the firmware."""
    link.command("S", response_seconds=0.5)
    info = link.command("I")
    diagnostics = link.command("D")
    pose = link.command("P")
    lines = info + diagnostics + pose
    errors: list[str] = []
    warnings: list[str] = []

    if contains(lines, "FAULT:") or contains(lines, "fault=") and not contains(lines, "fault=none"):
        errors.append("Nano reports an active fault.")
    if not contains(info, "present=yes"):
        errors.append("MPU6050 is not reported as present.")
    if not contains(info, "calibrated=yes"):
        errors.append("MPU6050 gyro Z bias has not been calibrated; keep the car still and run calibrate.")
    if not contains(lines, "encoder_preflight=passed"):
        errors.append("Encoder preflight has not passed.")
    if not diagnostics:
        warnings.append("D produced no response; inspect the Nano serial output before moving.")
    return Preflight(lines=lines, errors=errors, warnings=warnings)


def print_preflight(preflight: Preflight) -> None:
    for warning in preflight.warnings:
        print(f"WARNING: {warning}")
    if preflight.errors:
        print("PRE-FLIGHT BLOCKED:")
        for error in preflight.errors:
            print(f"  - {error}")
    else:
        print("PRE-FLIGHT PASSED")


def require_typed_confirmation(expected: str, prompt: str) -> None:
    entered = input(f"{prompt}\nType {expected} to continue: ").strip()
    if entered != expected:
        raise RuntimeError("Confirmation did not match; no movement command was sent.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safe high-level controller for the Nano rover.")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Nano serial device (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("inspect", help="Stop, then query I/D/P. Never moves the car.")
    subparsers.add_parser("stop", help="Immediately send S. Never moves the car.")

    calibrate = subparsers.add_parser("calibrate", help="Calibrate gyro bias while the car is stationary.")
    calibrate.add_argument("--confirm-still", action="store_true", help="Required acknowledgement that the car will not move.")

    pulse = subparsers.add_parser("pulse", help="Brief, manual equal-PWM wheel test. Not position control.")
    pulse.add_argument("--seconds", type=float, required=True, help="Motor-on duration; 0 < seconds <= 1.0.")
    pulse.add_argument("--pwm", type=int, default=80, help="Equal left/right PWM, 1 through 165 (default: 80).")
    pulse.add_argument("--unlock", action="store_true", help="Required explicit acknowledgement before motion is enabled.")

    run = subparsers.add_parser("run", help="Run one Nano firmware trajectory after safety checks.")
    run.add_argument("trajectory", choices=sorted(TRAJECTORIES))
    run.add_argument("--unlock", action="store_true", help="Required explicit acknowledgement before motion is enabled.")
    run.add_argument(
        "--watch-seconds", type=float, default=None,
        help="How long to read Nano output before sending a final S (default depends on trajectory).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    link = NanoLink(args.port, args.baud)

    def emergency_stop(*_unused: object) -> None:
        print("\nEmergency stop requested.", file=sys.stderr)
        link.stop()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)
    atexit.register(link.stop)

    try:
        link.open()
        if args.action == "stop":
            link.stop()
            return 0

        if args.action == "inspect":
            print_preflight(run_preflight(link))
            return 0

        if args.action == "calibrate":
            if not args.confirm_still:
                raise RuntimeError("Refusing calibration without --confirm-still.")
            link.command("S", response_seconds=0.5)
            require_typed_confirmation("CALIBRATE", "Place the car on a stable surface; it must remain still for about 1.5 seconds.")
            link.command("C", response_seconds=2.5)
            print_preflight(run_preflight(link))
            return 0

        if args.action == "pulse":
            if not args.unlock:
                raise RuntimeError("Refusing motor test without --unlock.")
            if not 0 < args.seconds <= 1.0:
                raise RuntimeError("--seconds must be greater than 0 and no more than 1.0.")
            if not 1 <= abs(args.pwm) <= 165:
                raise RuntimeError("--pwm magnitude must be between 1 and 165.")
            link.command("S", response_seconds=0.5)
            require_typed_confirmation(
                "PULSE",
                f"Wheel test: M{args.pwm},{args.pwm} for about {args.seconds:g} s. "
                "Keep the car suspended and clear of people.",
            )
            # Do not wait for a reply here: the requested pulse duration starts
            # immediately after the motor command is transmitted.
            link.send(f"M{args.pwm},{args.pwm}")
            time.sleep(args.seconds)
            link.command("S", response_seconds=0.5)
            return 0

        if not args.unlock:
            raise RuntimeError("Refusing motion without --unlock.")
        preflight = run_preflight(link)
        print_preflight(preflight)
        if not preflight.passed:
            return 2

        description, default_watch = TRAJECTORIES[args.trajectory]
        require_typed_confirmation(
            args.trajectory,
            f"Ready to run {args.trajectory}: {description}. Keep the route clear and be ready to cut motor power.",
        )
        link.command(args.trajectory, response_seconds=1.0)
        watch_seconds = args.watch_seconds if args.watch_seconds is not None else default_watch
        if watch_seconds <= 0:
            raise RuntimeError("--watch-seconds must be positive.")
        print(f"Monitoring Nano output for {watch_seconds:g} seconds; Ctrl-C sends S.")
        print_lines("Nano", link.read_for(watch_seconds))
        link.stop()
        return 0
    except (RuntimeError, serial.SerialException, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        link.stop()
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
