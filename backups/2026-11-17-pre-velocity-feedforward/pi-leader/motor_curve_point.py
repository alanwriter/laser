#!/usr/bin/env python3
"""One supervised, grounded PWM/TPS point for one rover; never starts automatically."""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import serial

from leader_formation import (
    BAUD, NanoLink, ProtocolError, parse_status_fields, print_status,
    require_confirmation, resolve_port, validate_motor_test,
)


FIELDS = (
    "vehicle", "phase", "elapsed_s", "left_command_pwm", "right_command_pwm",
    "x_mm", "y_mm", "heading_deg", "left_count", "right_count",
    "left_tps", "right_tps", "left_pwm", "right_pwm", "fault_code",
    "encoder_preflight", "imu_present", "imu_calibrated",
)
MAX_FORWARD_MM = 450.0
MAX_LATERAL_MM = 120.0
MAX_HEADING_DEG = 20.0
EXPIRY_MS = 900


def checked_status(link: NanoLink):
    status = parse_status_fields(link.request("STATUS", wait=0.45).fields)
    if status.fault_code or not status.imu_present or not status.imu_calibrated:
        raise RuntimeError("Nano fault or IMU state lost")
    if not -30.0 <= status.x_mm <= MAX_FORWARD_MM or abs(status.y_mm) > MAX_LATERAL_MM:
        raise RuntimeError("Onboard position guard exceeded")
    if abs(status.heading_deg) > MAX_HEADING_DEG:
        raise RuntimeError("Onboard heading guard exceeded")
    return status


def record(writer, vehicle: str, phase: str, elapsed: float, left: int, right: int, status) -> None:
    writer.writerow({
        "vehicle": vehicle, "phase": phase, "elapsed_s": round(elapsed, 4),
        "left_command_pwm": left, "right_command_pwm": right,
        "x_mm": status.x_mm, "y_mm": status.y_mm, "heading_deg": status.heading_deg,
        "left_count": status.left_count, "right_count": status.right_count,
        "left_tps": status.left_tps, "right_tps": status.right_tps,
        "left_pwm": status.left_pwm, "right_pwm": status.right_pwm,
        "fault_code": status.fault_code, "encoder_preflight": status.encoder_preflight,
        "imu_present": status.imu_present, "imu_calibrated": status.imu_calibrated,
    })


def commission(link: NanoLink, vehicle: str) -> None:
    initial = link.startup_check()
    print_status(initial)
    if initial.imu_present != 1:
        raise RuntimeError("MPU6050 absent")
    require_confirmation("CALIBRATE", f"Keep {vehicle} still for gyro calibration.")
    print(link.request("CALIBRATE", wait=3.0).raw)
    if not checked_status(link).imu_calibrated:
        raise RuntimeError("Gyro calibration failed")
    print(link.request("RESET").raw)
    require_confirmation(
        "MOTOR",
        f"Grounded {vehicle} preflight: PWM 80 for 800 ms will briefly move it. "
        "Clear the floor and keep motor cutoff reachable.",
    )
    try:
        print(link.request("MOTOR", 80, 80, 800).raw)
        time.sleep(0.95)
    finally:
        link.stop_safely()
    print(link.request("ENCODER").raw)
    status = checked_status(link)
    print_status(status)
    if not status.ready_for_path:
        raise RuntimeError("Grounded motor/encoder preflight failed")


def sample_phase(link: NanoLink, writer, vehicle: str, phase: str,
                 left: int, right: int, duration_s: float) -> None:
    started = time.monotonic()
    last_command = -1.0
    while True:
        elapsed = time.monotonic() - started
        if elapsed >= duration_s:
            break
        if last_command < 0 or elapsed - last_command >= 0.40:
            link.request("MOTOR", left, right, EXPIRY_MS, wait=0.45)
            last_command = time.monotonic() - started
        status = checked_status(link)
        if status.mode.lower() != "manual":
            raise RuntimeError("MOTOR command expired or Nano left manual mode")
        record(writer, vehicle, phase, time.monotonic() - started, left, right, status)
        time.sleep(0.05)


def run(link: NanoLink, vehicle: str, left: int, right: int,
        hold_s: float, startup_ms: int, log_path: Path) -> None:
    commission(link, vehicle)
    require_confirmation(
        "CURVE-POINT",
        f"Put {vehicle} MPU back at 0 cm, nose along the tape. Confirm 60 cm "
        "clear ahead and on both sides, camera recording, and motor cutoff reachable. "
        f"This point uses PWM {left}/{right} for {hold_s:.2f} s "
        f"after an optional PWM 80 start of {startup_ms} ms.",
    )
    print(link.request("RESET").raw)
    origin = checked_status(link)
    if not origin.ready_for_path:
        raise RuntimeError("RESET did not preserve preflight flags")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, FIELDS)
        writer.writeheader()
        record(writer, vehicle, "origin", 0, 0, 0, origin)
        try:
            if startup_ms:
                sample_phase(link, writer, vehicle, "startup", 80, 80, startup_ms / 1000)
            sample_phase(link, writer, vehicle, "hold", left, right, hold_s)
        finally:
            link.stop_safely()
        final = checked_status(link)
        record(writer, vehicle, "stopped", 0, 0, 0, final)
        stream.flush()
    print(link.request("ENCODER").raw)
    print_status(final)
    print(f"Curve point CSV: {log_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", required=True, choices=("Leader", "F1", "F2"))
    parser.add_argument("--port", help="Nano /dev/serial/by-id path")
    parser.add_argument("--left-pwm", type=int, required=True)
    parser.add_argument("--right-pwm", type=int, required=True)
    parser.add_argument("--hold-s", type=float, default=1.2)
    parser.add_argument("--startup-ms", type=int, default=300)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--unlock", action="store_true")
    args = parser.parse_args()
    if not args.unlock:
        parser.error("--unlock is required")
    if not 25 <= args.left_pwm <= 100 or not 25 <= args.right_pwm <= 100:
        parser.error("PWM must be 25..100")
    if not 0.5 <= args.hold_s <= 1.8:
        parser.error("hold duration must be 0.5..1.8 s")
    if args.startup_ms and not 200 <= args.startup_ms <= 500:
        parser.error("startup duration must be 0 or 200..500 ms")
    validate_motor_test(80, 800)
    link = NanoLink(resolve_port(args.port), BAUD)

    def on_signal(*_unused: object) -> None:
        link.stop_safely()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        link.open()
        run(link, args.vehicle, args.left_pwm, args.right_pwm,
            args.hold_s, args.startup_ms, args.log)
        return 0
    except (RuntimeError, ProtocolError, serial.SerialException, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        link.stop_safely()
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
