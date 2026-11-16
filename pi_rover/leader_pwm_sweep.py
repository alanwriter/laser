#!/usr/bin/env python3
"""Supervised, grounded Leader PWM versus encoder-speed characterization.

Only Leader's two motors receive equal, short open-loop MOTOR pulses.  A user
repositions it after the existing grounded preflight and confirms once before
the sweep.  Every pulse expires on the Nano and is followed by an explicit STOP.
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import serial

from leader_formation import (
    BAUD,
    NanoLink,
    ProtocolError,
    parse_status_fields,
    print_status,
    require_confirmation,
    resolve_port,
)
from leader_straight_calibration import commission_grounded


PULSES = (25, 30, 35, 40, 45, 50)
PULSE_MS = 800
MAX_FORWARD_MM = 450.0
MAX_LATERAL_MM = 120.0
MAX_HEADING_DEG = 20.0
FIELDS = (
    "pulse", "pwm", "event", "elapsed_s", "x_mm", "y_mm", "heading_deg",
    "left_count", "right_count", "left_tps", "right_tps", "left_pwm",
    "right_pwm", "fault_code", "encoder_preflight", "imu_present",
    "imu_calibrated",
)


def safe_status(link: NanoLink):
    status = parse_status_fields(link.request("STATUS", wait=0.45).fields)
    if status.fault_code or not status.imu_present or not status.imu_calibrated:
        raise RuntimeError("Nano fault or IMU state lost during PWM sweep")
    if status.x_mm > MAX_FORWARD_MM or abs(status.y_mm) > MAX_LATERAL_MM:
        raise RuntimeError("Grounded sweep exceeded onboard travel guard")
    if abs(status.heading_deg) > MAX_HEADING_DEG:
        raise RuntimeError("Grounded sweep exceeded onboard heading guard")
    return status


def log_status(writer, pulse: int, pwm: int, event: str, elapsed: float, status) -> None:
    writer.writerow({
        "pulse": pulse, "pwm": pwm, "event": event,
        "elapsed_s": round(elapsed, 4), "x_mm": status.x_mm,
        "y_mm": status.y_mm, "heading_deg": status.heading_deg,
        "left_count": status.left_count, "right_count": status.right_count,
        "left_tps": status.left_tps, "right_tps": status.right_tps,
        "left_pwm": status.left_pwm, "right_pwm": status.right_pwm,
        "fault_code": status.fault_code,
        "encoder_preflight": status.encoder_preflight,
        "imu_present": status.imu_present,
        "imu_calibrated": status.imu_calibrated,
    })


def run(link: NanoLink, log_path: Path) -> None:
    commission_grounded(link, test_pwm=80, test_ms=800)
    require_confirmation(
        "PWM-SWEEP",
        "Put Leader back at the tape's 0 cm MPU mark. Keep at least 60 cm "
        "clear ahead and on both sides, camera recording, motor cutoff "
        "reachable. Six equal-wheel 800 ms pulses, PWM 25..50, will move it.",
    )
    print(link.request("RESET").raw)
    origin = safe_status(link)
    if not origin.ready_for_path:
        raise RuntimeError("RESET did not preserve preflight flags")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, FIELDS)
        writer.writeheader()
        for pulse, pwm in enumerate(PULSES, 1):
            before = safe_status(link)
            log_status(writer, pulse, pwm, "before", 0.0, before)
            print(f"Pulse {pulse}/{len(PULSES)}: PWM {pwm} for {PULSE_MS} ms", flush=True)
            started = time.monotonic()
            try:
                link.request("MOTOR", pwm, pwm, PULSE_MS, wait=0.45)
                while time.monotonic() - started < PULSE_MS / 1000.0:
                    status = safe_status(link)
                    log_status(writer, pulse, pwm, "moving", time.monotonic() - started, status)
                    time.sleep(0.06)
            finally:
                link.stop_safely()
            time.sleep(0.12)
            after = safe_status(link)
            log_status(writer, pulse, pwm, "after", time.monotonic() - started, after)
            output.flush()
            print(f"  count delta L/R {after.left_count-before.left_count}/"
                  f"{after.right_count-before.right_count}; "
                  f"pose x/y/heading {after.x_mm:.1f}/{after.y_mm:.1f}/"
                  f"{after.heading_deg:.1f}", flush=True)
            if not after.encoder_preflight:
                raise RuntimeError("Encoder preflight flag lost")
        print(link.request("ENCODER").raw)
        final = safe_status(link)
        print_status(final)
    print(f"PWM sweep CSV: {log_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Leader Nano /dev/serial/by-id path")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--unlock", action="store_true")
    args = parser.parse_args()
    if not args.unlock:
        parser.error("--unlock is required")
    timestamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    log_path = args.log or Path(__file__).resolve().parent / "logs" / f"pwm_sweep_{timestamp}.csv"
    link = NanoLink(resolve_port(args.port), BAUD)

    def on_signal(*_unused: object) -> None:
        link.stop_safely()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        link.open()
        run(link, log_path)
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
