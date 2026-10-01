#!/usr/bin/env python3
"""Leader-1 controller and formation-reference publisher for the Nano L1 USB protocol.

The Nano remains the only process that drives motors and owns its safety timeout.
This Pi program never moves the rover on launch.  In ``leader`` mode it starts a
firmware PATH only after a manual unlock and publishes the *measured* leader
pose, rather than assuming the requested path was followed perfectly.
"""

from __future__ import annotations

import argparse
import atexit
import csv
import json
import signal
import socket
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import serial


BAUD = 115200
MAX_PWM = 165
MIN_MOTOR_MS = 50
MAX_MOTOR_MS = 1200
MIN_TELEMETRY_MS = 100
MAX_TELEMETRY_MS = 2000


@dataclass(frozen=True)
class IoLine:
    operation: str
    sequence: int
    fields: tuple[str, ...]
    raw: str


@dataclass(frozen=True)
class RoverStatus:
    mode: str
    x_mm: float
    y_mm: float
    heading_deg: float
    left_count: int
    right_count: int
    left_tps: float
    right_tps: float
    left_target_tps: float
    right_target_tps: float
    left_pwm: int
    right_pwm: int
    fault_code: int
    encoder_preflight: int
    imu_present: int
    imu_calibrated: int
    path_number: int
    path_step: int

    @property
    def ready_for_path(self) -> bool:
        return (
            self.imu_present == 1
            and self.imu_calibrated == 1
            and self.encoder_preflight == 1
            and self.fault_code == 0
        )


class ProtocolError(RuntimeError):
    """The Nano returned a well-formed IO,ERROR response."""


class NanoLink:
    """A single-owner, request/response client for Leader-1 Nano firmware."""

    def __init__(self, port: str, baud: int = BAUD, timeout: float = 0.12) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.device: serial.Serial | None = None
        self._sequence = 0
        self.telemetry: list[RoverStatus] = []

    def open(self) -> None:
        path = Path(self.port)
        if not path.exists():
            raise RuntimeError(f"Nano USB serial device not found: {path}")
        self.device = serial.Serial(str(path), self.baud, timeout=self.timeout, write_timeout=1.0)
        # A USB open resets many Nano boards.  Never issue a command during boot.
        time.sleep(2.0)
        self._drain_boot_messages()

    def close(self) -> None:
        if self.device and self.device.is_open:
            self.device.close()

    def _drain_boot_messages(self) -> None:
        for line in self.read_for(0.35):
            print(f"Nano boot: {line.raw}")

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFF
        return self._sequence

    @staticmethod
    def parse(raw: bytes) -> IoLine | None:
        text = raw.decode("ascii", errors="replace").strip()
        if not text.startswith("IO,"):
            return None
        try:
            fields = next(csv.reader([text]))
            if len(fields) < 3:
                return None
            return IoLine(fields[1].upper(), int(fields[2]), tuple(fields[3:]), text)
        except (ValueError, csv.Error):
            return None

    def _read_one(self) -> IoLine | None:
        if not self.device:
            raise RuntimeError("Serial port is not open")
        raw = self.device.readline()
        if not raw:
            return None
        line = self.parse(raw)
        if line and line.operation == "TELEMETRY":
            try:
                self.telemetry.append(parse_status_fields(line.fields))
            except ValueError as error:
                print(f"WARNING: ignored malformed telemetry: {error}", file=sys.stderr)
        return line

    def read_for(self, seconds: float) -> list[IoLine]:
        deadline = time.monotonic() + seconds
        result: list[IoLine] = []
        while time.monotonic() < deadline:
            line = self._read_one()
            if line:
                result.append(line)
        return result

    def request(self, operation: str, *arguments: object, wait: float = 1.0) -> IoLine:
        """Send one IO request and wait only for its matching sequence response."""
        if not self.device:
            raise RuntimeError("Serial port is not open")
        sequence = self._next_sequence()
        command = ",".join(("IO", str(sequence), operation.upper(), *(str(x) for x in arguments)))
        print(f">>> {command}")
        try:
            self.device.write((command + "\n").encode("ascii"))
            self.device.flush()
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline:
                line = self._read_one()
                if line is None:
                    continue
                if line.operation == "ERROR" and line.sequence == sequence:
                    detail = ",".join(line.fields) or "unspecified Nano error"
                    raise ProtocolError(f"Nano rejected {operation}: {detail}")
                if line.sequence != sequence:
                    continue
                # Replies are normally IO,<OP>,<seq>,...; STOP/MOTOR can be ACK.
                if line.operation in {operation.upper(), "ACK"}:
                    print(f"<<< {line.raw}")
                    return line
            raise RuntimeError(f"Timed out waiting for {operation} sequence {sequence}")
        except (serial.SerialException, OSError) as error:
            raise RuntimeError(f"Nano serial disconnect: {error}") from error

    def stop_safely(self) -> None:
        """Best-effort STOP for Ctrl-C, faults, disconnects, and ordinary exit."""
        if not self.device or not self.device.is_open:
            return
        try:
            self.request("STOP", wait=0.45)
        except (RuntimeError, ProtocolError):
            pass

    def startup_check(self) -> RoverStatus:
        hello = self.request("HELLO", 1)
        if not hello.fields or hello.fields[0] != "1":
            raise ProtocolError(f"Unexpected HELLO reply: {hello.raw}")
        self.stop_safely()
        self.request("IMU")
        self.request("ENCODER")
        reply = self.request("STATUS")
        return parse_status_fields(reply.fields)


def parse_status_fields(fields: Iterable[str]) -> RoverStatus:
    """Parse the payload shared by IO,STATUS and IO,TELEMETRY."""
    values = tuple(fields)
    if len(values) < 18:
        raise ValueError(f"STATUS needs 18 fields after sequence; received {len(values)}")
    try:
        return RoverStatus(
            mode=values[0], x_mm=float(values[1]), y_mm=float(values[2]), heading_deg=float(values[3]),
            left_count=int(values[4]), right_count=int(values[5]), left_tps=float(values[6]), right_tps=float(values[7]),
            left_target_tps=float(values[8]), right_target_tps=float(values[9]), left_pwm=int(values[10]), right_pwm=int(values[11]),
            fault_code=int(values[12]), encoder_preflight=int(values[13]), imu_present=int(values[14]),
            imu_calibrated=int(values[15]), path_number=int(values[16]), path_step=int(values[17]),
        )
    except ValueError as error:
        raise ValueError(f"invalid STATUS values: {values!r}") from error


def resolve_port(requested: str | None) -> str:
    if requested:
        return requested
    candidates = sorted(Path("/dev/serial/by-id").glob("*"))
    if len(candidates) == 1:
        return str(candidates[0])
    if not candidates:
        raise RuntimeError("No Nano found in /dev/serial/by-id; connect its USB cable or pass --port.")
    found = "\n  ".join(str(path) for path in candidates)
    raise RuntimeError(f"More than one serial device found; choose with --port:\n  {found}")


def print_status(status: RoverStatus) -> None:
    print(json.dumps(asdict(status), ensure_ascii=False, sort_keys=True))


def require_confirmation(expected: str, message: str) -> None:
    entered = input(f"{message}\nType {expected} to continue: ").strip()
    if entered != expected:
        raise RuntimeError("Confirmation did not match; no movement command was sent.")


class LeaderPublisher:
    """Broadcast the actual leader pose; follower controllers own their own motors."""

    def __init__(self, destination: str | None) -> None:
        self.sock: socket.socket | None = None
        self.destination: tuple[str, int] | None = None
        if destination:
            host, separator, port = destination.rpartition(":")
            if not separator or not host or not port.isdecimal():
                raise RuntimeError("--broadcast must use HOST:PORT, for example 239.42.0.1:5005")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.destination = (host, int(port))

    def publish(self, status: RoverStatus) -> None:
        frame = {
            "type": "leader_state",
            "version": 1,
            "monotonic_s": round(time.monotonic(), 3),
            "status": asdict(status),
        }
        encoded = json.dumps(frame, separators=(",", ":")).encode("utf-8")
        if self.sock and self.destination:
            self.sock.sendto(encoded, self.destination)
        print(encoded.decode("utf-8"))

    def close(self) -> None:
        if self.sock:
            self.sock.close()


def preflight_or_raise(link: NanoLink) -> RoverStatus:
    status = link.startup_check()
    print_status(status)
    if not status.ready_for_path:
        raise RuntimeError(
            "PATH blocked: require imu_present=1, imu_calibrated=1, "
            "encoder_preflight=1, and fault_code=0."
        )
    return status


def validate_motor_test(pwm: int, duration_ms: int) -> None:
    if not 1 <= abs(pwm) <= MAX_PWM:
        raise RuntimeError(f"Test PWM magnitude must be 1..{MAX_PWM}.")
    if not MIN_MOTOR_MS <= duration_ms <= MAX_MOTOR_MS:
        raise RuntimeError(f"Test duration must be {MIN_MOTOR_MS}..{MAX_MOTOR_MS} ms.")


def commission(link: NanoLink, pwm: int, duration_ms: int) -> RoverStatus:
    """Calibrate and prove both encoder phases in one USB-open session.

    Opening the Nano USB serial device resets its volatile calibration and
    encoder state.  This deliberately keeps CALIBRATE, RESET, MOTOR, and the
    final STATUS in one session so a successful result is meaningful.
    """
    validate_motor_test(pwm, duration_ms)
    initial = link.startup_check()
    print_status(initial)
    if not initial.imu_present:
        raise RuntimeError("MPU6050 is not detected; check I2C wiring before calibration.")

    require_confirmation("CALIBRATE", "Keep the rover completely still for the 1.5 s gyro calibration.")
    print(link.request("CALIBRATE", wait=3.0).raw)
    calibrated = parse_status_fields(link.request("STATUS").fields)
    print_status(calibrated)
    if not calibrated.imu_calibrated:
        raise RuntimeError("Gyro calibration did not complete.")

    # Start the A/B phase test with fresh counts while retaining gyro bias.
    print(link.request("RESET").raw)
    require_confirmation(
        "MOTOR",
        f"Lift both wheels clear. Test both motors at PWM {pwm} for {duration_ms} ms.",
    )
    print(link.request("MOTOR", pwm, pwm, duration_ms).raw)
    time.sleep(duration_ms / 1000.0 + 0.15)
    link.stop_safely()
    print(link.request("ENCODER").raw)
    result = parse_status_fields(link.request("STATUS").fields)
    print_status(result)
    if not result.encoder_preflight:
        raise RuntimeError(
            "Encoder preflight did not pass. Check both wheel motors and A/B encoder phases."
        )
    if not result.ready_for_path:
        raise RuntimeError("Commissioning did not produce a PATH-ready status.")
    print("COMMISSION PASSED: IMU and both encoder phases are ready for PATH.")
    return result


def run_leader(link: NanoLink, status: RoverStatus, path: int, telemetry_ms: int, broadcast: str | None) -> None:
    if path not in {1, 2}:
        raise RuntimeError("Leader firmware permits PATH 1 or 2 only.")
    if not status.ready_for_path:
        raise RuntimeError("Leader PATH requires a successful commissioning session.")
    require_confirmation(
        f"LEADER-PATH-{path}",
        f"Leader will run PATH {path}. Clear the route and retain physical motor-power cutoff.",
    )
    publisher = LeaderPublisher(broadcast)
    try:
        link.request("TELEMETRY", telemetry_ms)
        link.request("PATH", path)
        print("Leader path started. Ctrl-C sends STOP; path completion is detected from STATUS mode.")
        last_status = status
        while True:
            # Telemetry is asynchronous, so keep serial ownership and evaluate every frame.
            for _line in link.read_for(max(telemetry_ms / 1000 * 1.6, 0.8)):
                while link.telemetry:
                    last_status = link.telemetry.pop(0)
                    publisher.publish(last_status)
                    if last_status.fault_code != 0:
                        raise RuntimeError(f"Nano reported fault_code={last_status.fault_code}")
            latest = link.request("STATUS", wait=0.8)
            last_status = parse_status_fields(latest.fields)
            publisher.publish(last_status)
            if last_status.fault_code != 0:
                raise RuntimeError(f"Nano reported fault_code={last_status.fault_code}")
            if last_status.mode.lower() == "idle" and last_status.path_number == 0:
                print("Leader path completed.")
                return
    finally:
        try:
            link.request("TELEMETRY", 0, wait=0.5)
        except RuntimeError:
            pass
        publisher.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe Leader-1 Nano USB and formation-reference controller.")
    parser.add_argument("--port", help="Nano /dev/serial/by-id path; auto-detects only when exactly one exists.")
    parser.add_argument("--baud", type=int, default=BAUD)
    actions = parser.add_subparsers(dest="action", required=True)
    actions.add_parser("status", help="HELLO → STOP → IMU/ENCODER/STATUS; never moves.")
    actions.add_parser("config", help="Read Nano mechanical configuration; never moves.")
    actions.add_parser("imu", help="Read IMU status; never moves.")
    actions.add_parser("encoder", help="Read encoder status; never moves.")
    actions.add_parser("stop", help="Immediately send STOP.")
    actions.add_parser("reset", help="Reset relative pose/encoder state; never drives motors.")
    calibrate = actions.add_parser("calibrate", help="Calibrate gyro Z while motionless.")
    calibrate.add_argument("--confirm-still", action="store_true", help="Required acknowledgement that the rover is still.")
    motor = actions.add_parser("motor", help="Timed manual PWM test; Nano stops automatically at expiry.")
    motor.add_argument("left_pwm", type=int)
    motor.add_argument("right_pwm", type=int)
    motor.add_argument("duration_ms", type=int)
    motor.add_argument("--unlock", action="store_true")
    commission_parser = actions.add_parser(
        "commission",
        help="One USB session: calibrate gyro, test both motors/encoder phases, then report PATH readiness.",
    )
    commission_parser.add_argument("--pwm", type=int, default=80)
    commission_parser.add_argument("--duration-ms", type=int, default=800)
    commission_parser.add_argument("--unlock", action="store_true")
    leader = actions.add_parser("leader", help="Run PATH 1/2 and publish measured leader poses for followers.")
    leader.add_argument("path", type=int, choices=(1, 2))
    leader.add_argument("--telemetry-ms", type=int, default=500, help="Nano status interval, 100..2000 ms (default 500).")
    leader.add_argument("--broadcast", help="Optional follower network destination, HOST:PORT (UDP).")
    leader.add_argument("--test-pwm", type=int, default=80, help="Pre-PATH encoder-test PWM, 1..165 (default 80).")
    leader.add_argument("--test-duration-ms", type=int, default=800, help="Pre-PATH encoder-test duration, 50..1200 ms (default 800).")
    leader.add_argument("--unlock", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    link = NanoLink(resolve_port(args.port), args.baud)

    def on_signal(*_unused: object) -> None:
        print("\nEmergency STOP requested.", file=sys.stderr)
        link.stop_safely()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    atexit.register(link.stop_safely)
    try:
        link.open()
        if args.action == "stop":
            link.stop_safely()
        elif args.action == "status":
            # Inspection must be useful even when calibration/preflight is not
            # ready yet; only movement paths are blocked by that condition.
            print_status(link.startup_check())
        elif args.action == "config":
            link.startup_check()
            print(link.request("CONFIG").raw)
        elif args.action == "imu":
            link.startup_check()
            print(link.request("IMU").raw)
        elif args.action == "encoder":
            link.startup_check()
            print(link.request("ENCODER").raw)
        elif args.action == "reset":
            link.startup_check()
            print(link.request("RESET").raw)
        elif args.action == "calibrate":
            if not args.confirm_still:
                raise RuntimeError("Refusing CALIBRATE without --confirm-still.")
            link.stop_safely()
            require_confirmation("CALIBRATE", "Keep the rover still for roughly 1.5 seconds.")
            print(link.request("CALIBRATE", wait=3.0).raw)
            status = link.startup_check()
            print_status(status)
            if not status.imu_present or not status.imu_calibrated:
                raise RuntimeError("Gyro calibration did not complete; PATH remains blocked.")
            if not status.ready_for_path:
                print(
                    "Gyro calibration succeeded. PATH remains blocked until encoder preflight passes.",
                    file=sys.stderr,
                )
        elif args.action == "motor":
            if not args.unlock:
                raise RuntimeError("Refusing MOTOR without --unlock.")
            if not all(-MAX_PWM <= value <= MAX_PWM for value in (args.left_pwm, args.right_pwm)):
                raise RuntimeError(f"PWM must be within {-MAX_PWM}..{MAX_PWM}.")
            if not MIN_MOTOR_MS <= args.duration_ms <= MAX_MOTOR_MS:
                raise RuntimeError(f"duration_ms must be {MIN_MOTOR_MS}..{MAX_MOTOR_MS}.")
            link.stop_safely()
            require_confirmation("MOTOR", "Clear the rover and keep physical motor cutoff ready.")
            print(link.request("MOTOR", args.left_pwm, args.right_pwm, args.duration_ms).raw)
            time.sleep(args.duration_ms / 1000.0 + 0.15)
            link.stop_safely()
            print(link.request("ENCODER").raw)
            print_status(parse_status_fields(link.request("STATUS").fields))
        elif args.action == "commission":
            if not args.unlock:
                raise RuntimeError("Refusing commissioning motor test without --unlock.")
            commission(link, args.pwm, args.duration_ms)
        elif args.action == "leader":
            if not args.unlock:
                raise RuntimeError("Refusing leader PATH without --unlock.")
            if not MIN_TELEMETRY_MS <= args.telemetry_ms <= MAX_TELEMETRY_MS:
                raise RuntimeError(
                    f"--telemetry-ms must be {MIN_TELEMETRY_MS}..{MAX_TELEMETRY_MS}."
                )
            status = commission(link, args.test_pwm, args.test_duration_ms)
            run_leader(link, status, args.path, args.telemetry_ms, args.broadcast)
        return 0
    except (RuntimeError, ProtocolError, serial.SerialException, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        link.stop_safely()
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
