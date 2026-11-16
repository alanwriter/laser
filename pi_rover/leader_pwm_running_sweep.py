#!/usr/bin/env python3
"""Measure Leader wheel speed while PWM descends without stopping between steps."""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import serial

from leader_formation import BAUD, NanoLink, ProtocolError, print_status, require_confirmation, resolve_port
from leader_pwm_sweep import FIELDS, log_status, safe_status
from leader_straight_calibration import commission_grounded


STEPS = ((80, 0.55), (60, 0.75), (55, 0.75), (50, 0.75), (45, 0.75), (40, 0.75), (35, 0.75))
COMMAND_EXPIRY_MS = 1200


def run(link: NanoLink, log_path: Path) -> None:
    commission_grounded(link, test_pwm=80, test_ms=800)
    require_confirmation(
        "RUNNING-SWEEP",
        "Return Leader MPU to 0 cm, nose along the tape. Keep at least 60 cm "
        "clear ahead and on both sides and motor cutoff reachable. After a "
        "brief PWM 80 start, PWM descends 60,55,50,45,40,35 without stopping.",
    )
    print(link.request("RESET").raw)
    origin = safe_status(link)
    if not origin.ready_for_path:
        raise RuntimeError("RESET did not preserve preflight flags")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, FIELDS)
        writer.writeheader()
        try:
            for step, (pwm, dwell_s) in enumerate(STEPS, 1):
                before = safe_status(link)
                log_status(writer, step, pwm, "before", 0.0, before)
                print(f"Step {step}/{len(STEPS)} PWM {pwm}; switching after {dwell_s:.2f} s", flush=True)
                started = time.monotonic()
                link.request("MOTOR", pwm, pwm, COMMAND_EXPIRY_MS, wait=0.45)
                while time.monotonic() - started < dwell_s:
                    current = safe_status(link)
                    log_status(writer, step, pwm, "moving", time.monotonic() - started, current)
                    if time.monotonic() - started > 0.20 and current.mode != "manual":
                        raise RuntimeError("Manual motor command expired before next step")
                    time.sleep(0.05)
                after = safe_status(link)
                log_status(writer, step, pwm, "segment_end", time.monotonic() - started, after)
                output.flush()
                print(f"  count delta L/R {after.left_count-before.left_count}/"
                      f"{after.right_count-before.right_count}; "
                      f"pose x/y/heading {after.x_mm:.1f}/{after.y_mm:.1f}/"
                      f"{after.heading_deg:.1f}", flush=True)
        finally:
            link.stop_safely()
        print(link.request("ENCODER").raw)
        final = safe_status(link)
        print_status(final)
    print(f"Running PWM sweep CSV: {log_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Leader Nano /dev/serial/by-id path")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--unlock", action="store_true")
    args = parser.parse_args()
    if not args.unlock:
        parser.error("--unlock is required")
    timestamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    log_path = args.log or Path(__file__).resolve().parent / "logs" / f"pwm_running_{timestamp}.csv"
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
