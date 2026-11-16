#!/usr/bin/env python3
"""Supervised Leader-only straight-line odometry measurement.

The Nano owns wheel PID and its expiring VELOCITY command.  This script keeps
one USB session from gyro calibration through the final STOP so RESET, counts,
and the CSV refer to the same run.  No follower traffic or LiDAR is used.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path

import serial

from leader_formation import (
    BAUD,
    NanoGeometry,
    NanoLink,
    ProtocolError,
    RoverStatus,
    WaveControl,
    WaveCsvLogger,
    WavePlan,
    clamp,
    parse_status_fields,
    print_status,
    require_confirmation,
    resolve_port,
    validate_motor_test,
    wrap_degrees,
)


def straight_control(status: RoverStatus, origin: RoverStatus,
                     distance_mm: float, speed_mm_s: float,
                     fixed_equal_wheels: bool = False) -> WaveControl:
    heading = math.radians(origin.heading_deg)
    dx, dy = status.x_mm - origin.x_mm, status.y_mm - origin.y_mm
    forward = math.cos(heading) * dx + math.sin(heading) * dy
    lateral = -math.sin(heading) * dx + math.cos(heading) * dy
    if fixed_equal_wheels:
        # Characterize the existing per-wheel VELOCITY loop without Pi yaw correction.
        equal = round(speed_mm_s)
        return WaveControl(forward, lateral, 0.0, origin.heading_deg,
                           wrap_degrees(origin.heading_deg - status.heading_deg),
                           equal, equal)
    desired_heading = origin.heading_deg + clamp(
        math.degrees(math.atan2(-lateral, 260.0)), -15.0, 15.0,
    )
    heading_error = wrap_degrees(desired_heading - status.heading_deg)
    speed = min(speed_mm_s, max(18.0, (distance_mm - forward) * 0.20))
    speed *= clamp(1.0 - abs(heading_error) / 120.0, 0.55, 1.0)
    yaw_rate = clamp(1.5 * math.radians(heading_error),
                     -0.92 * 2.0 * speed / 130.0,
                     +0.92 * 2.0 * speed / 130.0)
    left = round(speed - 65.0 * yaw_rate)
    right = round(speed + 65.0 * yaw_rate)
    return WaveControl(forward, lateral, 0.0, desired_heading,
                       heading_error, left, right)


def commission_grounded(link: NanoLink, test_pwm: int, test_ms: int) -> RoverStatus:
    validate_motor_test(test_pwm, test_ms)
    initial = link.startup_check()
    print_status(initial)
    if initial.imu_present != 1:
        raise RuntimeError("MPU6050 absent; refusing calibration and movement")
    require_confirmation("CALIBRATE", "Keep Leader completely still for gyro calibration.")
    print(link.request("CALIBRATE", wait=3.0).raw)
    calibrated = parse_status_fields(link.request("STATUS").fields)
    if calibrated.imu_calibrated != 1:
        raise RuntimeError("Gyro calibration failed")
    print(link.request("RESET").raw)
    require_confirmation(
        "MOTOR",
        f"Grounded preflight: clear space around Leader; PWM {test_pwm} for {test_ms} ms "
        "will briefly move it. Keep motor power cutoff within reach.",
    )
    print(link.request("MOTOR", test_pwm, test_pwm, test_ms).raw)
    time.sleep(test_ms / 1000.0 + 0.15)
    link.stop_safely()
    print(link.request("ENCODER").raw)
    result = parse_status_fields(link.request("STATUS").fields)
    print_status(result)
    if not result.ready_for_path:
        raise RuntimeError("Grounded motor/encoder preflight failed")
    return result


def run(link: NanoLink, distance_mm: float, speed_mm_s: float,
        log_path: Path, test_pwm: int, test_ms: int,
        fixed_equal_wheels: bool = False) -> None:
    preflight = commission_grounded(link, test_pwm, test_ms)
    if fixed_equal_wheels and abs(preflight.heading_deg) > 20.0:
        raise RuntimeError("Grounded preflight yaw exceeded 20 degrees")
    geometry = NanoGeometry.from_config_fields(link.request("CONFIG").fields)
    geometry.require_mpu_heading()
    prompt = (
        "Place Leader MPU at 0 mm with its nose along the tape. Start camera "
        f"recording, verify the {distance_mm / 1000.0:g} m lane and stopping "
        "margin are clear, and keep motor power cutoff within reach. "
    )
    if fixed_equal_wheels:
        prompt += "Equal left/right VELOCITY commands, with no Pi heading correction. "
    require_confirmation(f"STRAIGHT-{distance_mm / 1000.0:g}M",
                         prompt + "RESET and motion follow immediately.")
    print(link.request("RESET").raw)
    origin = parse_status_fields(link.request("STATUS").fields)
    if not origin.ready_for_path:
        raise RuntimeError("RESET did not preserve preflight state")
    plan = WavePlan(length_mm=distance_mm, amplitude_mm=0.0, cycles=1,
                    cruise_mm_per_second=speed_mm_s,
                    route_kind="straight_calibration_v1")
    logger = WaveCsvLogger(log_path, plan, geometry)
    logger.record("preflight", preflight, 0.0)
    logger.record("origin", origin, 0.0)
    started = time.monotonic()
    current = origin
    completed = False
    try:
        while True:
            elapsed = time.monotonic() - started
            control = straight_control(current, origin, distance_mm, speed_mm_s,
                                       fixed_equal_wheels=fixed_equal_wheels)
            if not current.ready_for_path:
                raise RuntimeError("Nano safety state lost")
            if elapsed > max(120.0, distance_mm / speed_mm_s * 1.5):
                raise RuntimeError("Straight run timed out")
            if abs(control.lateral_mm) > 120.0 or abs(control.heading_error_deg) > 20.0:
                raise RuntimeError("Leader left the straight corridor")
            if control.forward_mm < -50.0 or control.forward_mm > distance_mm + 100.0:
                raise RuntimeError("Odometry outside allowed travel corridor")
            logger.record("control", current, elapsed, control)
            if control.forward_mm >= distance_mm:
                completed = True
                break
            cycle_started = time.monotonic()
            link.request("VELOCITY", control.left_mm_per_second,
                         control.right_mm_per_second, 180, wait=0.7)
            time.sleep(max(0.0, 0.10 - (time.monotonic() - cycle_started)))
            current = parse_status_fields(link.request("STATUS", wait=0.7).fields)
    finally:
        link.stop_safely()
        try:
            encoder = link.request("ENCODER", wait=0.7)
            encoder_path = logger.path.with_suffix(".encoder.txt")
            encoder_path.write_text(encoder.raw + "\n", encoding="utf-8")
            print(f"Final encoder A/B diagnostics: {encoder.raw}")
            print(f"Encoder diagnostics: {encoder_path}")
        except (RuntimeError, ProtocolError, serial.SerialException, OSError) as error:
            print(f"Final encoder diagnostics unavailable: {error}", file=sys.stderr)
        try:
            final = parse_status_fields(link.request("STATUS", wait=0.7).fields)
            logger.record("completed" if completed else "aborted", final,
                          time.monotonic() - started)
            print_status(final)
        except (RuntimeError, ProtocolError, serial.SerialException, OSError, ValueError):
            logger.record("stop_status_unavailable", current,
                          time.monotonic() - started)
        logger.close()
        print(f"Straight CSV log: {logger.path}")
    print("Straight run completed; Nano STOP sent.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Nano /dev/serial/by-id path")
    parser.add_argument("--distance-mm", type=float, default=2000.0)
    parser.add_argument("--speed-mm-s", type=float, default=25.0)
    parser.add_argument("--test-pwm", type=int, default=80)
    parser.add_argument("--test-duration-ms", type=int, default=800)
    parser.add_argument("--fixed-equal-wheels", action="store_true",
                        help="Send identical fixed left/right VELOCITY setpoints; no Pi yaw correction")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--unlock", action="store_true")
    args = parser.parse_args()
    if not args.unlock:
        parser.error("--unlock is required")
    if not 200.0 <= args.distance_mm <= 2000.0:
        parser.error("--distance-mm must be 200..2000")
    if not 18.0 <= args.speed_mm_s <= 30.0:
        parser.error("--speed-mm-s must be 18..30")
    timestamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    log_path = args.log or Path(__file__).resolve().parent / "logs" / f"straight_{timestamp}.csv"
    link = NanoLink(resolve_port(args.port), BAUD)

    def on_signal(*_unused: object) -> None:
        link.stop_safely()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        link.open()
        run(link, args.distance_mm, args.speed_mm_s, log_path,
            args.test_pwm, args.test_duration_ms, args.fixed_equal_wheels)
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
