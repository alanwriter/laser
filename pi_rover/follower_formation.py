#!/usr/bin/env python3
"""Safe Follower-1 formation controller for the shared Nano USB protocol.

The Leader broadcasts its measured pose as UDP JSON.  This process owns only
Follower 1's Nano USB port: it validates each leader packet, computes the F1
target at a fixed Leader-body offset, and sends short ``VELOCITY`` setpoints.
No packet can carry a raw PWM command.  A stale/unsafe packet, serial failure,
fault, signal, or ordinary program exit sends F1 ``STOP``.
"""

from __future__ import annotations

import argparse
import atexit
import ipaddress
import json
import math
import select
import signal
import socket
import sys
import time
from dataclasses import asdict, dataclass

import serial

from leader_formation import (
    MAX_MOTOR_MS,
    MIN_MOTOR_MS,
    NanoLink,
    ProtocolError,
    RoverStatus,
    clamp,
    commission,
    parse_status_fields,
    require_confirmation,
    resolve_port,
    wrap_degrees,
)


ACTIVE_LEADER_MODES = frozenset({"velocity", "path"})


@dataclass(frozen=True)
class FormationPlan:
    """Rigid target pose and conservative local F1 tracking limits.

    ``offset_*`` is expressed in the Leader body frame.  ``follower_start_*``
    maps F1's freshly RESET Nano frame into the same experiment frame as the
    Leader.  For a follower placed 400 mm directly behind a Leader starting at
    (0, 0, 0), use the defaults for both x values.
    """

    offset_x_mm: float = -400.0
    offset_y_mm: float = 0.0
    follower_start_x_mm: float = -400.0
    follower_start_y_mm: float = 0.0
    follower_start_heading_deg: float = 0.0
    position_gain_per_second: float = 0.8
    lateral_gain_radians_per_second_per_radian: float = 2.0
    heading_gain_radians_per_second_per_radian: float = 1.4
    lateral_lookahead_mm: float = 220.0
    max_wheel_mm_per_second: float = 75.0
    max_yaw_rate_radians_per_second: float = 0.7
    wheel_track_mm: float = 130.0
    command_ms: int = 180
    control_period_s: float = 0.10
    frame_timeout_s: float = 0.35


@dataclass(frozen=True)
class LeaderFrame:
    status: RoverStatus
    received_monotonic_s: float
    source_host: str


@dataclass(frozen=True)
class FormationControl:
    target_x_mm: float
    target_y_mm: float
    target_heading_deg: float
    forward_error_mm: float
    lateral_error_mm: float
    heading_error_deg: float
    left_mm_per_second: int
    right_mm_per_second: int


def parse_endpoint(text: str) -> tuple[str, int]:
    host, separator, port_text = text.rpartition(":")
    if not separator or not host or not port_text.isdecimal():
        raise RuntimeError("endpoint must use HOST:PORT, for example 0.0.0.0:5005")
    port = int(port_text)
    if not 1 <= port <= 65535:
        raise RuntimeError("UDP port must be 1..65535")
    return host, port


def parse_leader_frame(payload: bytes, source_host: str, received_at: float) -> LeaderFrame:
    """Decode the versioned Leader packet without trusting arbitrary fields."""
    try:
        frame = json.loads(payload.decode("utf-8"))
        if frame.get("type") != "leader_state" or frame.get("version") != 1:
            raise ValueError("unexpected packet type or version")
        raw_status = frame["status"]
        if not isinstance(raw_status, dict):
            raise ValueError("status is not an object")
        status = RoverStatus(**{
            name: raw_status[name] for name in RoverStatus.__dataclass_fields__
        })
    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid leader packet: {error}") from error
    return LeaderFrame(status=status, received_monotonic_s=received_at, source_host=source_host)


class LeaderUdpReceiver:
    """Receive Leader packets from an explicit source and optional multicast group."""

    def __init__(
        self,
        listen: tuple[str, int],
        leader_host: str,
        multicast_group: str | None,
    ) -> None:
        self.leader_host = leader_host
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(listen)
        if multicast_group:
            group = ipaddress.ip_address(multicast_group)
            if not group.is_multicast:
                raise RuntimeError("--multicast-group must be an IPv4 multicast address")
            membership = socket.inet_aton(str(group)) + socket.inet_aton("0.0.0.0")
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)

    def receive_latest(self, timeout_s: float) -> LeaderFrame | None:
        """Wait up to ``timeout_s`` and coalesce queued packets to the newest one."""
        ready, _, _ = select.select([self.sock], [], [], timeout_s)
        if not ready:
            return None
        latest: LeaderFrame | None = None
        while True:
            payload, (source_host, _source_port) = self.sock.recvfrom(4096)
            if source_host == self.leader_host:
                try:
                    latest = parse_leader_frame(payload, source_host, time.monotonic())
                except ValueError as error:
                    print(f"WARNING: ignored Leader packet: {error}", file=sys.stderr)
            ready, _, _ = select.select([self.sock], [], [], 0)
            if not ready:
                return latest

    def close(self) -> None:
        self.sock.close()


class FormationTracker:
    """Pure pose controller; it neither opens USB nor communicates over UDP."""

    def __init__(self, plan: FormationPlan) -> None:
        self.plan = plan
        self._previous_target: tuple[float, float, float, float] | None = None

    def control(self, leader: LeaderFrame, follower: RoverStatus) -> FormationControl:
        leader_heading_rad = math.radians(leader.status.heading_deg)
        target_x = (
            leader.status.x_mm
            + math.cos(leader_heading_rad) * self.plan.offset_x_mm
            - math.sin(leader_heading_rad) * self.plan.offset_y_mm
        )
        target_y = (
            leader.status.y_mm
            + math.sin(leader_heading_rad) * self.plan.offset_x_mm
            + math.cos(leader_heading_rad) * self.plan.offset_y_mm
        )
        target_heading = leader.status.heading_deg

        # Feed forward the measured movement of the target pose.  This makes a
        # correctly positioned F1 roll with the Leader rather than waiting for
        # a large longitudinal error to accumulate.
        forward_feedforward = 0.0
        yaw_feedforward = 0.0
        if self._previous_target:
            old_x, old_y, old_heading, old_time = self._previous_target
            elapsed = leader.received_monotonic_s - old_time
            if 0.02 <= elapsed <= 1.0:
                vx = (target_x - old_x) / elapsed
                vy = (target_y - old_y) / elapsed
                follower_heading_rad = math.radians(
                    self.plan.follower_start_heading_deg + follower.heading_deg
                )
                forward_feedforward = (
                    math.cos(follower_heading_rad) * vx
                    + math.sin(follower_heading_rad) * vy
                )
                yaw_feedforward = math.radians(wrap_degrees(target_heading - old_heading)) / elapsed
        self._previous_target = (target_x, target_y, target_heading, leader.received_monotonic_s)

        follower_x = self.plan.follower_start_x_mm + follower.x_mm
        follower_y = self.plan.follower_start_y_mm + follower.y_mm
        follower_heading = self.plan.follower_start_heading_deg + follower.heading_deg
        follower_heading_rad = math.radians(follower_heading)
        world_error_x = target_x - follower_x
        world_error_y = target_y - follower_y
        forward_error = (
            math.cos(follower_heading_rad) * world_error_x
            + math.sin(follower_heading_rad) * world_error_y
        )
        lateral_error = (
            -math.sin(follower_heading_rad) * world_error_x
            + math.cos(follower_heading_rad) * world_error_y
        )
        heading_error = wrap_degrees(target_heading - follower_heading)

        forward_speed = clamp(
            forward_feedforward + self.plan.position_gain_per_second * forward_error,
            -self.plan.max_wheel_mm_per_second,
            self.plan.max_wheel_mm_per_second,
        )
        lateral_angle = math.atan2(lateral_error, self.plan.lateral_lookahead_mm)
        yaw_rate = clamp(
            yaw_feedforward
            + self.plan.lateral_gain_radians_per_second_per_radian * lateral_angle
            + self.plan.heading_gain_radians_per_second_per_radian * math.radians(heading_error),
            -self.plan.max_yaw_rate_radians_per_second,
            self.plan.max_yaw_rate_radians_per_second,
        )
        left = round(clamp(
            forward_speed - self.plan.wheel_track_mm * yaw_rate / 2.0,
            -self.plan.max_wheel_mm_per_second,
            self.plan.max_wheel_mm_per_second,
        ))
        right = round(clamp(
            forward_speed + self.plan.wheel_track_mm * yaw_rate / 2.0,
            -self.plan.max_wheel_mm_per_second,
            self.plan.max_wheel_mm_per_second,
        ))
        return FormationControl(
            target_x_mm=target_x,
            target_y_mm=target_y,
            target_heading_deg=target_heading,
            forward_error_mm=forward_error,
            lateral_error_mm=lateral_error,
            heading_error_deg=heading_error,
            left_mm_per_second=left,
            right_mm_per_second=right,
        )


def validate_plan(plan: FormationPlan) -> None:
    if not all(abs(value) <= 2000.0 for value in (
        plan.offset_x_mm, plan.offset_y_mm, plan.follower_start_x_mm, plan.follower_start_y_mm,
    )):
        raise RuntimeError("formation offsets/start coordinates must be within +/-2000 mm")
    if not 0.0 < plan.lateral_lookahead_mm <= 1000.0:
        raise RuntimeError("lateral lookahead must be 0..1000 mm")
    if not 0.0 < plan.max_wheel_mm_per_second <= 100.0:
        raise RuntimeError("max wheel speed must be 0..100 mm/s")
    if plan.wheel_track_mm <= 0.0:
        raise RuntimeError("wheel track must be positive")
    if not MIN_MOTOR_MS <= plan.command_ms <= MAX_MOTOR_MS:
        raise RuntimeError(f"command timeout must be {MIN_MOTOR_MS}..{MAX_MOTOR_MS} ms")
    if not 0.0 < plan.control_period_s < plan.command_ms / 1000.0:
        raise RuntimeError("control period must be positive and shorter than the Nano timeout")
    if not plan.control_period_s <= plan.frame_timeout_s <= 2.0:
        raise RuntimeError("frame timeout must be at least one control period and at most 2 s")


def leader_is_safe_and_moving(frame: LeaderFrame) -> bool:
    return frame.status.ready_for_path and frame.status.mode.lower() in ACTIVE_LEADER_MODES


def run_follow(
    link: NanoLink,
    receiver: LeaderUdpReceiver,
    plan: FormationPlan,
    test_pwm: int,
    test_duration_ms: int,
) -> None:
    """Commission F1, then follow only fresh and safe Leader packets."""
    validate_plan(plan)
    commission(link, test_pwm, test_duration_ms)
    print(link.request("RESET").raw)
    initial = parse_status_fields(link.request("STATUS").fields)
    if not initial.ready_for_path:
        raise RuntimeError("F1 RESET did not preserve ready state; refusing formation control.")
    require_confirmation(
        "FOLLOWER-1",
        "Place F1 at its declared start pose, clear the route, start Leader broadcast, "
        "and retain physical motor-power cutoff.",
    )
    tracker = FormationTracker(plan)
    latest: LeaderFrame | None = None
    stopped_for_wait = True
    print("F1 armed and listening. It remains stopped until a fresh, safe moving Leader packet arrives.")
    try:
        while True:
            frame = receiver.receive_latest(plan.control_period_s)
            if frame:
                latest = frame
            stale = not latest or time.monotonic() - latest.received_monotonic_s > plan.frame_timeout_s
            if stale or not latest or not leader_is_safe_and_moving(latest):
                if not stopped_for_wait:
                    link.stop_safely()
                    stopped_for_wait = True
                    print("F1 STOP: waiting for a fresh, safe moving Leader frame.")
                continue

            follower = parse_status_fields(link.request("STATUS", wait=0.7).fields)
            if not follower.ready_for_path:
                raise RuntimeError("F1 Nano preflight became unsafe; stopping.")
            control = tracker.control(latest, follower)
            link.request(
                "VELOCITY",
                control.left_mm_per_second,
                control.right_mm_per_second,
                plan.command_ms,
                wait=0.7,
            )
            stopped_for_wait = False
            print(json.dumps({
                "leader": asdict(latest.status),
                "follower": asdict(follower),
                "control": asdict(control),
            }, separators=(",", ":")))
    finally:
        link.stop_safely()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe Follower-1 UDP formation controller.")
    parser.add_argument("--port", help="F1 Nano /dev/serial/by-id path; auto-detect only when exactly one exists.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--listen", default="0.0.0.0:5005", help="local UDP bind endpoint (default 0.0.0.0:5005)")
    parser.add_argument("--leader-host", required=True, help="required source IPv4 address of the Leader Pi")
    parser.add_argument("--multicast-group", help="join this IPv4 multicast group, e.g. 239.42.0.1")
    parser.add_argument("--offset-x-mm", type=float, default=-400.0, help="F1 target x in Leader body frame (default -400)")
    parser.add_argument("--offset-y-mm", type=float, default=0.0, help="F1 target y in Leader body frame (default 0)")
    parser.add_argument("--follower-start-x-mm", type=float, default=-400.0, help="F1 Nano RESET origin in experiment frame")
    parser.add_argument("--follower-start-y-mm", type=float, default=0.0, help="F1 Nano RESET origin in experiment frame")
    parser.add_argument("--follower-start-heading-deg", type=float, default=0.0, help="F1 initial heading relative to Leader frame")
    parser.add_argument("--max-speed-mm-s", type=float, default=75.0)
    parser.add_argument("--command-ms", type=int, default=180)
    parser.add_argument("--frame-timeout-s", type=float, default=0.35)
    parser.add_argument("--test-pwm", type=int, default=80)
    parser.add_argument("--test-duration-ms", type=int, default=800)
    parser.add_argument("--unlock", action="store_true", help="required before the explicit commissioning/follow confirmations")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.unlock:
        print("ERROR: refusing formation control without --unlock.", file=sys.stderr)
        return 1
    try:
        listen = parse_endpoint(args.listen)
        ipaddress.ip_address(args.leader_host)
        plan = FormationPlan(
            offset_x_mm=args.offset_x_mm,
            offset_y_mm=args.offset_y_mm,
            follower_start_x_mm=args.follower_start_x_mm,
            follower_start_y_mm=args.follower_start_y_mm,
            follower_start_heading_deg=args.follower_start_heading_deg,
            max_wheel_mm_per_second=args.max_speed_mm_s,
            command_ms=args.command_ms,
            frame_timeout_s=args.frame_timeout_s,
        )
        validate_plan(plan)
    except (RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    link = NanoLink(resolve_port(args.port), args.baud)
    receiver = LeaderUdpReceiver(listen, args.leader_host, args.multicast_group)

    def on_signal(*_unused: object) -> None:
        print("\nEmergency STOP requested.", file=sys.stderr)
        link.stop_safely()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    atexit.register(link.stop_safely)
    try:
        link.open()
        run_follow(link, receiver, plan, args.test_pwm, args.test_duration_ms)
        return 0
    except (RuntimeError, ProtocolError, serial.SerialException, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        link.stop_safely()
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        receiver.close()
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
