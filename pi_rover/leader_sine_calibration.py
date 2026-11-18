#!/usr/bin/env python3
"""Supervised Leader-only 2 m, 1.5-wave right-first path experiment.

Nano retains wheel PID and 180 ms VELOCITY expiry. The optional odometry scale
changes only the Pi pose used by this experiment; Nano CONFIG is untouched.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import serial

from leader_formation import (BAUD, NanoGeometry, NanoLink, ProtocolError,
                              RoverStatus, clamp, parse_status_fields,
                              print_status, require_confirmation, resolve_port,
                              wrap_degrees)
from leader_straight_calibration import commission_grounded

LENGTH_MM = 2000.0
AMPLITUDE_MM = 70.0
TRACK_MM = 130.0
# 2026-11-18: firmware WHEEL_DIAMETER_MM now folds in each rover's own 2 m
# ground/odometry ratio (Leader 74.64 mm), so Nano odometry reports ground mm.  Pass --odom-scale 1.1483 only when
# replaying a run made on the old 65 mm firmware.
SCALE_DEFAULT = 1.0


def reference(x_mm: float) -> tuple[float, float, float]:
    """Return right-first path y, dy/dx and signed curvature."""
    phase = 3.0 * math.pi * clamp(x_mm, 0.0, LENGTH_MM) / LENGTH_MM
    rate = 3.0 * math.pi / LENGTH_MM
    sine, cosine = math.sin(phase), math.cos(phase)
    y = -AMPLITUDE_MM * sine ** 3
    slope = -3.0 * AMPLITUDE_MM * rate * sine ** 2 * cosine
    second = -3.0 * AMPLITUDE_MM * rate ** 2 * (2.0 * sine * cosine ** 2 - sine ** 3)
    return y, slope, second / (1.0 + slope * slope) ** 1.5


@dataclass(frozen=True)
class Control:
    x_mm: float
    y_mm: float
    target_y_mm: float
    lateral_error_mm: float
    target_heading_deg: float
    heading_error_deg: float
    left_mm_s: int
    right_mm_s: int


def pose(status: RoverStatus, origin: RoverStatus, scale: float) -> tuple[float, float]:
    heading = math.radians(origin.heading_deg)
    dx, dy = status.x_mm - origin.x_mm, status.y_mm - origin.y_mm
    return (scale * (math.cos(heading) * dx + math.sin(heading) * dy),
            scale * (-math.sin(heading) * dx + math.cos(heading) * dy))


def control(status: RoverStatus, origin: RoverStatus, scale: float,
            speed_mm_s: float) -> Control:
    x, y = pose(status, origin, scale)
    target_y, slope, curvature = reference(x)
    lateral_error = target_y - y
    target_heading = origin.heading_deg + math.degrees(math.atan(slope)) + clamp(
        math.degrees(math.atan2(lateral_error, 260.0)), -18.0, 18.0)
    heading_error = wrap_degrees(target_heading - status.heading_deg)
    remaining = max(0.0, LENGTH_MM - x)
    base_speed = clamp(remaining * 0.20, 18.0, speed_mm_s)
    base_speed *= clamp(1.0 - abs(heading_error) / 120.0, 0.4, 1.0)
    yaw_rate = base_speed * curvature + 1.5 * math.radians(heading_error)
    yaw_limit = min(0.65, 0.92 * 2.0 * base_speed / TRACK_MM)
    yaw_rate = clamp(yaw_rate, -yaw_limit, yaw_limit)
    left = round(base_speed - TRACK_MM * yaw_rate / 2.0)
    right = round(base_speed + TRACK_MM * yaw_rate / 2.0)
    return Control(x, y, target_y, lateral_error, target_heading, heading_error,
                   left, right)


def guard(status: RoverStatus, origin: RoverStatus, command: Control,
          elapsed_s: float) -> None:
    if not status.ready_for_path:
        raise RuntimeError("Nano safety state lost")
    if elapsed_s > 130.0:
        raise RuntimeError("Sine experiment exceeded 130 s")
    if command.x_mm < -50.0 or command.x_mm > LENGTH_MM + 150.0:
        raise RuntimeError("Forward pose outside corridor")
    if abs(command.y_mm) > 220.0 or abs(command.lateral_error_mm) > 110.0:
        raise RuntimeError("Lateral pose outside corridor")
    if abs(wrap_degrees(status.heading_deg - origin.heading_deg)) > 35.0:
        raise RuntimeError("Absolute heading outside corridor")
    if abs(command.heading_error_deg) > 25.0:
        raise RuntimeError("Heading tracking error outside corridor")


def run(link: NanoLink, log_path: Path, scale: float, speed_mm_s: float) -> None:
    preflight = commission_grounded(link, 80, 800)
    if abs(preflight.heading_deg) > 20.0:
        raise RuntimeError("Grounded preflight yaw exceeded 20 degrees")
    geometry = NanoGeometry.from_config_fields(link.request("CONFIG").fields)
    geometry.require_mpu_heading()
    require_confirmation(
        "SINE-RIGHT-2M",
        "Return Leader MPU projection to 0 cm, point nose along center tape, "
        "confirm left/right 40 cm and 50 cm beyond endpoint are clear, "
        "start fixed camera if available, and keep motor cutoff in reach. "
        "RESET and motion follow immediately.",
    )
    print(link.request("RESET").raw)
    origin = parse_status_fields(link.request("STATUS").fields)
    if not origin.ready_for_path:
        raise RuntimeError("RESET did not preserve Nano preflight state")

    log_path = log_path.expanduser().resolve()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["event", "wall_time_utc", "elapsed_s", "odom_scale", "route_length_mm",
              "amplitude_mm", "cycles", "speed_mm_s", "raw_x_mm", "raw_y_mm",
              "corrected_x_mm", "corrected_y_mm", "target_y_mm", "lateral_error_mm",
              "target_heading_deg", "heading_error_deg", "left_command_mm_s",
              "right_command_mm_s", *(f"status_{name}" for name in
                                       RoverStatus.__dataclass_fields__)]
    started = time.monotonic()
    current = origin
    completed = False
    with log_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()

        def record(event: str, state: RoverStatus, elapsed: float,
                   cmd: Control | None = None) -> None:
            raw_x, raw_y = pose(state, origin, 1.0)
            corrected_x, corrected_y = pose(state, origin, scale)
            row = dict(event=event, wall_time_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                       elapsed_s=round(elapsed, 6), odom_scale=scale,
                       route_length_mm=LENGTH_MM, amplitude_mm=AMPLITUDE_MM,
                       cycles=1.5, speed_mm_s=speed_mm_s,
                       raw_x_mm=raw_x, raw_y_mm=raw_y,
                       corrected_x_mm=corrected_x, corrected_y_mm=corrected_y)
            if cmd is not None:
                row.update(target_y_mm=cmd.target_y_mm,
                           lateral_error_mm=cmd.lateral_error_mm,
                           target_heading_deg=cmd.target_heading_deg,
                           heading_error_deg=cmd.heading_error_deg,
                           left_command_mm_s=cmd.left_mm_s,
                           right_command_mm_s=cmd.right_mm_s)
            row.update({f"status_{name}": value for name, value in asdict(state).items()})
            writer.writerow(row)
            handle.flush()

        record("preflight", preflight, 0.0)
        record("origin", origin, 0.0)
        try:
            while True:
                elapsed = time.monotonic() - started
                cmd = control(current, origin, scale, speed_mm_s)
                guard(current, origin, cmd, elapsed)
                record("control", current, elapsed, cmd)
                if cmd.x_mm >= LENGTH_MM:
                    if abs(cmd.y_mm) > 50.0 or abs(cmd.heading_error_deg) > 12.0:
                        raise RuntimeError("Endpoint reached with excessive lateral/heading error")
                    completed = True
                    break
                cycle_start = time.monotonic()
                link.request("VELOCITY", cmd.left_mm_s, cmd.right_mm_s, 180, wait=0.7)
                time.sleep(max(0.0, 0.10 - (time.monotonic() - cycle_start)))
                current = parse_status_fields(link.request("STATUS", wait=0.7).fields)
        finally:
            link.stop_safely()
            try:
                encoder = link.request("ENCODER", wait=0.7)
                log_path.with_suffix(".encoder.txt").write_text(encoder.raw + "\n", encoding="utf-8")
                print(f"Final encoder: {encoder.raw}")
            except (RuntimeError, ProtocolError, serial.SerialException, OSError) as error:
                print(f"Encoder diagnostics unavailable: {error}", file=sys.stderr)
            try:
                final = parse_status_fields(link.request("STATUS", wait=0.7).fields)
                record("completed" if completed else "aborted", final,
                       time.monotonic() - started)
                print_status(final)
            except (RuntimeError, ProtocolError, serial.SerialException, OSError, ValueError):
                record("stop_status_unavailable", current, time.monotonic() - started)
    print(f"Sine CSV log: {log_path}")
    print("Sine run completed; Nano STOP sent.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--odom-scale", type=float, default=SCALE_DEFAULT)
    parser.add_argument("--speed-mm-s", type=float, default=25.0)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--unlock", action="store_true")
    args = parser.parse_args()
    if not args.unlock:
        parser.error("--unlock is required")
    if not math.isfinite(args.odom_scale) or not 0.8 <= args.odom_scale <= 1.3:
        parser.error("--odom-scale must be finite and within 0.8..1.3")
    if not math.isfinite(args.speed_mm_s) or not 18.0 <= args.speed_mm_s <= 30.0:
        parser.error("--speed-mm-s must be finite and within 18..30")
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    log_path = args.log or Path(__file__).resolve().parent / "logs" / f"sine_right_{stamp}.csv"
    link = NanoLink(resolve_port(args.port), BAUD)

    def on_signal(*_unused: object) -> None:
        link.stop_safely()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        link.open()
        run(link, log_path, args.odom_scale, args.speed_mm_s)
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
