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
    """Left-rear target pose and conservative polar-tracking limits.

    ``offset_*`` is expressed in the Leader body frame.  ``follower_start_*``
    maps F1's freshly RESET Nano frame into the same experiment frame as the
    Leader.  The defaults mean 200 mm rearward (-x) and 200 mm leftward (+y):
    F1 starts at Leader's left-rear 45-degree position, 282.8 mm away.

    The outer loop is intentionally polar: rho is distance to the virtual
    target, alpha is its bearing from F1's forward direction, and beta closes
    the target heading.  The Nano remains the owner of wheel-speed PID/PWM.
    """

    offset_x_mm: float = -200.0
    offset_y_mm: float = 200.0
    follower_start_x_mm: float = -200.0
    follower_start_y_mm: float = 200.0
    follower_start_heading_deg: float = 0.0
    distance_gain_per_second: float = 0.30
    bearing_gain_radians_per_second_per_radian: float = 1.20
    terminal_gain_radians_per_second_per_radian: float = -0.45
    max_wheel_mm_per_second: float = 45.0
    max_yaw_rate_radians_per_second: float = 0.35
    wheel_track_mm: float = 130.0
    max_target_error_mm: float = 250.0
    max_bearing_error_deg: float = 60.0
    min_leader_distance_mm: float = 180.0
    max_leader_distance_mm: float = 500.0
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
    target_speed_mm_per_second: float
    target_yaw_rate_radians_per_second: float
    leader_distance_mm: float
    rho_mm: float
    alpha_deg: float
    beta_deg: float
    left_mm_per_second: int
    right_mm_per_second: int
    tracking_safe: bool
    stop_reason: str


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
    """Pure polar controller; it neither opens USB nor communicates over UDP."""

    def __init__(self, plan: FormationPlan) -> None:
        self.plan = plan
        self._previous_target: tuple[float, float, float, float] | None = None

    def _target_kinematics(
        self,
        target_x_mm: float,
        target_y_mm: float,
        fallback_heading_deg: float,
        received_monotonic_s: float,
    ) -> tuple[float, float, float]:
        """Estimate the virtual target's forward speed, heading and yaw rate.

        The target is offset from the Leader body, so its travel direction can
        differ from Leader heading during a turn.  Differentiating the target
        itself preserves the non-holonomic geometry better than assuming both
        headings are always identical.
        """
        target_speed_mm_per_second = 0.0
        target_heading_deg = fallback_heading_deg
        target_yaw_rate_radians_per_second = 0.0
        if self._previous_target:
            old_x, old_y, old_heading_deg, old_time_s = self._previous_target
            elapsed_s = received_monotonic_s - old_time_s
            if 0.02 <= elapsed_s <= 1.0:
                vx = (target_x_mm - old_x) / elapsed_s
                vy = (target_y_mm - old_y) / elapsed_s
                target_speed_mm_per_second = math.hypot(vx, vy)
                if target_speed_mm_per_second >= 1.0:
                    target_heading_deg = math.degrees(math.atan2(vy, vx))
                    target_yaw_rate_radians_per_second = math.radians(
                        wrap_degrees(target_heading_deg - old_heading_deg)
                    ) / elapsed_s
        self._previous_target = (
            target_x_mm,
            target_y_mm,
            target_heading_deg,
            received_monotonic_s,
        )
        return (
            target_speed_mm_per_second,
            target_heading_deg,
            target_yaw_rate_radians_per_second,
        )

    def _follower_world_pose(self, follower: RoverStatus) -> tuple[float, float, float]:
        """Map F1's RESET-local odometry into the Leader experiment frame."""
        start_heading_rad = math.radians(self.plan.follower_start_heading_deg)
        follower_x = (
            self.plan.follower_start_x_mm
            + math.cos(start_heading_rad) * follower.x_mm
            - math.sin(start_heading_rad) * follower.y_mm
        )
        follower_y = (
            self.plan.follower_start_y_mm
            + math.sin(start_heading_rad) * follower.x_mm
            + math.cos(start_heading_rad) * follower.y_mm
        )
        follower_heading_deg = self.plan.follower_start_heading_deg + follower.heading_deg
        return follower_x, follower_y, follower_heading_deg

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
        target_speed, target_heading, target_yaw_rate = self._target_kinematics(
            target_x,
            target_y,
            leader.status.heading_deg,
            leader.received_monotonic_s,
        )

        follower_x, follower_y, follower_heading = self._follower_world_pose(follower)
        follower_heading_rad = math.radians(follower_heading)
        world_error_x = target_x - follower_x
        world_error_y = target_y - follower_y
        rho_mm = math.hypot(world_error_x, world_error_y)
        if rho_mm < 1.0:
            bearing_deg = target_heading
            alpha_deg = wrap_degrees(target_heading - follower_heading)
            beta_deg = 0.0
        else:
            bearing_deg = math.degrees(math.atan2(world_error_y, world_error_x))
            alpha_deg = wrap_degrees(bearing_deg - follower_heading)
            beta_deg = wrap_degrees(target_heading - bearing_deg)

        # The Leader's real separation is a collision/lost-formation monitor.
        # rho instead is distance to the left-rear virtual target used by the
        # polar controller.
        leader_distance_mm = math.hypot(
            leader.status.x_mm - follower_x,
            leader.status.y_mm - follower_y,
        )
        tracking_safe = True
        stop_reason = ""
        if leader_distance_mm < self.plan.min_leader_distance_mm:
            tracking_safe, stop_reason = False, "leader is too close"
        elif leader_distance_mm > self.plan.max_leader_distance_mm:
            tracking_safe, stop_reason = False, "leader separation is too large"
        elif rho_mm > self.plan.max_target_error_mm:
            tracking_safe, stop_reason = False, "virtual target error is too large"
        elif abs(alpha_deg) > self.plan.max_bearing_error_deg:
            tracking_safe, stop_reason = False, "target bearing error is too large"

        # v* and omega* are target feed-forward; rho/alpha/beta close the
        # local formation loop.  Translation never reverses in this initial
        # controller.  Large bearing errors are blocked above rather than
        # allowing a surprise pivot or blind reverse.
        forward_speed = clamp(
            (target_speed + self.plan.distance_gain_per_second * rho_mm)
            * math.cos(math.radians(alpha_deg)),
            0.0,
            self.plan.max_wheel_mm_per_second,
        )
        yaw_rate = clamp(
            target_yaw_rate
            + self.plan.bearing_gain_radians_per_second_per_radian * math.radians(alpha_deg)
            + self.plan.terminal_gain_radians_per_second_per_radian * math.radians(beta_deg),
            -self.plan.max_yaw_rate_radians_per_second,
            self.plan.max_yaw_rate_radians_per_second,
        )
        # First-stage formation tracking keeps both wheels non-negative. It
        # therefore cannot pivot or reverse unexpectedly when a pose estimate
        # is noisy; it steers progressively as forward target speed arrives.
        forward_only_yaw_limit = 0.85 * 2.0 * forward_speed / self.plan.wheel_track_mm
        yaw_rate = clamp(yaw_rate, -forward_only_yaw_limit, forward_only_yaw_limit)
        raw_left = forward_speed - self.plan.wheel_track_mm * yaw_rate / 2.0
        raw_right = forward_speed + self.plan.wheel_track_mm * yaw_rate / 2.0
        # Scale both wheels together if a limit is reached. This preserves the
        # requested curvature instead of clipping only one wheel.
        scale = max(1.0, abs(raw_left) / self.plan.max_wheel_mm_per_second,
                    abs(raw_right) / self.plan.max_wheel_mm_per_second)
        left = round(raw_left / scale)
        right = round(raw_right / scale)
        return FormationControl(
            target_x_mm=target_x,
            target_y_mm=target_y,
            target_heading_deg=target_heading,
            target_speed_mm_per_second=target_speed,
            target_yaw_rate_radians_per_second=target_yaw_rate,
            leader_distance_mm=leader_distance_mm,
            rho_mm=rho_mm,
            alpha_deg=alpha_deg,
            beta_deg=beta_deg,
            left_mm_per_second=left,
            right_mm_per_second=right,
            tracking_safe=tracking_safe,
            stop_reason=stop_reason,
        )


def validate_plan(plan: FormationPlan) -> None:
    if not all(abs(value) <= 2000.0 for value in (
        plan.offset_x_mm, plan.offset_y_mm, plan.follower_start_x_mm, plan.follower_start_y_mm,
    )):
        raise RuntimeError("formation offsets/start coordinates must be within +/-2000 mm")
    if not 0.0 < plan.distance_gain_per_second <= 2.0:
        raise RuntimeError("distance gain must be 0..2 per second")
    if not 0.0 < plan.max_wheel_mm_per_second <= 100.0:
        raise RuntimeError("max wheel speed must be 0..100 mm/s")
    if plan.wheel_track_mm <= 0.0 or plan.max_yaw_rate_radians_per_second <= 0.0:
        raise RuntimeError("wheel track must be positive")
    if not 0.0 < plan.max_target_error_mm <= 1000.0:
        raise RuntimeError("maximum target error must be 0..1000 mm")
    if not 0.0 < plan.max_bearing_error_deg <= 90.0:
        raise RuntimeError("maximum bearing error must be 0..90 degrees")
    if not 0.0 < plan.min_leader_distance_mm < plan.max_leader_distance_mm:
        raise RuntimeError("leader distance limits must be positive and ordered")
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
    consecutive_active_frames = 0
    stopped_for_wait = True
    print("F1 armed and listening. It remains stopped until three fresh, safe moving Leader packets arrive.")
    try:
        while True:
            frame = receiver.receive_latest(plan.control_period_s)
            if frame:
                latest = frame
                consecutive_active_frames = (
                    consecutive_active_frames + 1 if leader_is_safe_and_moving(frame) else 0
                )
            stale = not latest or time.monotonic() - latest.received_monotonic_s > plan.frame_timeout_s
            if (
                stale
                or not latest
                or not leader_is_safe_and_moving(latest)
                or consecutive_active_frames < 3
            ):
                if not stopped_for_wait:
                    link.stop_safely()
                    stopped_for_wait = True
                    print("F1 STOP: waiting for a fresh, safe moving Leader frame.")
                continue

            follower = parse_status_fields(link.request("STATUS", wait=0.7).fields)
            if not follower.ready_for_path:
                raise RuntimeError("F1 Nano preflight became unsafe; stopping.")
            control = tracker.control(latest, follower)
            if not control.tracking_safe:
                raise RuntimeError(f"F1 formation safety gate: {control.stop_reason}.")
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
    parser.add_argument("--offset-x-mm", type=float, default=-200.0, help="F1 target x in Leader body frame (default -200, rear)")
    parser.add_argument("--offset-y-mm", type=float, default=200.0, help="F1 target y in Leader body frame (default +200, left)")
    parser.add_argument("--follower-start-x-mm", type=float, default=-200.0, help="F1 Nano RESET origin in experiment frame")
    parser.add_argument("--follower-start-y-mm", type=float, default=200.0, help="F1 Nano RESET origin in experiment frame")
    parser.add_argument("--follower-start-heading-deg", type=float, default=0.0, help="F1 initial heading relative to Leader frame")
    parser.add_argument("--max-speed-mm-s", type=float, default=45.0)
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
