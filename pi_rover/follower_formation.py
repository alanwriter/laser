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
import csv
import ipaddress
import json
import math
import select
import signal
import socket
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

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
    """Left-rear target pose and conservative Leader-frame tracking limits.

    ``offset_*`` is expressed in the Leader body frame.  ``follower_start_*``
    maps F1's freshly RESET Nano frame into the same experiment frame as the
    Leader.  The defaults mean 400 mm rearward (-x) and 400 mm leftward (+y):
    F1 starts at Leader's left-rear 45-degree position, 565.7 mm away.

    The outer loop separates longitudinal, lateral and heading errors in the
    Leader body frame.  F1 therefore does not turn directly towards a moving
    point that it has just passed.  The Nano remains the owner of wheel-speed
    PID/PWM.
    """

    offset_x_mm: float = -400.0
    offset_y_mm: float = 400.0
    follower_start_x_mm: float = -400.0
    follower_start_y_mm: float = 400.0
    follower_start_heading_deg: float = 0.0
    near_distance_gain_per_second: float = 0.26
    far_distance_gain_per_second: float = 0.05
    distance_gain_transition_mm: float = 120.0
    near_catchup_limit_mm_per_second: float = 18.0
    far_catchup_limit_mm_per_second: float = 12.0
    lateral_gain_radians_per_second_per_radian: float = 0.60
    heading_gain_radians_per_second_per_radian: float = 0.50
    preview_distance_mm: float = 300.0
    max_wheel_mm_per_second: float = 75.0
    max_yaw_rate_radians_per_second: float = 0.25
    wheel_track_mm: float = 130.0
    # Collision is the hard physical-stop condition for this supervised trial.
    # Pose/bearing errors use the low-speed reacquisition behaviour below.
    collision_stop_distance_mm: float = 150.0
    max_target_error_mm: float = 800.0
    max_leader_distance_mm: float = 1200.0
    reacquire_bearing_deg: float = 65.0
    reacquire_speed_mm_per_second: float = 15.0
    max_wheel_step_mm_per_second: float = 10.0
    max_yaw_step_radians_per_second: float = 0.05
    settle_max_speed_mm_per_second: float = 20.0
    settle_target_tolerance_mm: float = 55.0
    settle_heading_tolerance_deg: float = 15.0
    position_deadband_mm: float = 10.0
    command_ms: int = 180
    control_period_s: float = 0.10
    frame_timeout_s: float = 0.35


@dataclass(frozen=True)
class LeaderFrame:
    status: RoverStatus
    received_monotonic_s: float
    source_host: str
    forward_mm_per_second: float = 0.0
    yaw_rate_radians_per_second: float = 0.0
    has_motion_reference: bool = False
    formation_phase: str = "moving"


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
    longitudinal_error_mm: float
    lateral_error_mm: float
    heading_error_deg: float
    left_mm_per_second: int
    right_mm_per_second: int
    tracking_safe: bool
    stop_reason: str


FOLLOWER_LOG_STATUS_COLUMNS = tuple(RoverStatus.__dataclass_fields__)
FOLLOWER_LOG_CONTROL_COLUMNS = tuple(FormationControl.__dataclass_fields__)
FOLLOWER_LOG_FIELDS = (
    "event",
    "wall_time_utc",
    "monotonic_s",
    "elapsed_s",
    "reason",
    "leader_source_host",
    "leader_received_monotonic_s",
    "leader_frame_age_s",
    "leader_motion_forward_mm_per_second",
    "leader_motion_yaw_rate_radians_per_second",
    "leader_has_motion_reference",
    "leader_formation_phase",
    *(f"leader_{name}" for name in FOLLOWER_LOG_STATUS_COLUMNS),
    *(f"follower_{name}" for name in FOLLOWER_LOG_STATUS_COLUMNS),
    *(f"control_{name}" for name in FOLLOWER_LOG_CONTROL_COLUMNS),
)


class FollowerCsvLogger:
    """Persist every follower-loop decision without affecting control policy."""

    def __init__(self, path: Path) -> None:
        self.path = path.expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=FOLLOWER_LOG_FIELDS)
        self.writer.writeheader()
        self.file.flush()

    def record(
        self,
        event: str,
        started_at: float,
        leader: LeaderFrame | None = None,
        follower: RoverStatus | None = None,
        control: FormationControl | None = None,
        reason: str = "",
    ) -> None:
        now = time.monotonic()
        row: dict[str, object] = {
            "event": event,
            "wall_time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "monotonic_s": now,
            "elapsed_s": now - started_at,
            "reason": reason,
        }
        if leader:
            row.update({
                "leader_source_host": leader.source_host,
                "leader_received_monotonic_s": leader.received_monotonic_s,
                "leader_frame_age_s": now - leader.received_monotonic_s,
                "leader_motion_forward_mm_per_second": leader.forward_mm_per_second,
                "leader_motion_yaw_rate_radians_per_second": leader.yaw_rate_radians_per_second,
                "leader_has_motion_reference": int(leader.has_motion_reference),
                "leader_formation_phase": leader.formation_phase,
            })
            row.update({f"leader_{name}": value for name, value in asdict(leader.status).items()})
        if follower:
            row.update({f"follower_{name}": value for name, value in asdict(follower).items()})
        if control:
            row.update({f"control_{name}": value for name, value in asdict(control).items()})
        self.writer.writerow(row)
        # A controller crash or Ctrl-C must still leave the last complete
        # control observation readable for post-run analysis.
        self.file.flush()

    def close(self) -> None:
        self.file.close()


def default_follower_log_path() -> Path:
    timestamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    return Path(__file__).resolve().parent / "logs" / f"follower_{timestamp}.csv"


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
        raw_motion = frame.get("motion")
        if raw_motion is None:
            forward_mm_per_second = 0.0
            yaw_rate_radians_per_second = 0.0
            has_motion_reference = False
        elif isinstance(raw_motion, dict):
            forward_mm_per_second = float(raw_motion["forward_mm_per_second"])
            yaw_rate_radians_per_second = float(raw_motion["yaw_rate_radians_per_second"])
            if not all(math.isfinite(value) for value in (
                forward_mm_per_second, yaw_rate_radians_per_second,
            )):
                raise ValueError("motion reference is not finite")
            has_motion_reference = True
        else:
            raise ValueError("motion is not an object")
        formation_phase = frame.get("formation_phase", "moving")
        if formation_phase not in {"moving", "settle", "stopped"}:
            raise ValueError("unrecognized formation phase")
    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid leader packet: {error}") from error
    return LeaderFrame(
        status=status,
        received_monotonic_s=received_at,
        source_host=source_host,
        forward_mm_per_second=forward_mm_per_second,
        yaw_rate_radians_per_second=yaw_rate_radians_per_second,
        has_motion_reference=has_motion_reference,
        formation_phase=formation_phase,
    )


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
    """Leader-frame controller; it neither opens USB nor communicates over UDP."""

    def __init__(self, plan: FormationPlan) -> None:
        self.plan = plan
        self._previous_target: tuple[float, float, float] | None = None
        self._previous_leader_heading: tuple[float, float] | None = None
        self._filtered_leader_yaw_rate = 0.0
        self._previous_wheels: tuple[int, int] | None = None
        self._previous_yaw_rate = 0.0

    def _measured_leader_yaw_rate(self, leader: LeaderFrame) -> float:
        """Low-pass Leader heading change; never use its planned wheel split.

        Cheap drivetrains can depart materially from their requested wheel
        speeds.  The measured heading is the only trustworthy yaw reference;
        filtering avoids the noisy target-position second derivative that
        caused the earlier alternating steering commands.
        """
        if self._previous_leader_heading:
            old_heading_deg, old_time_s = self._previous_leader_heading
            elapsed_s = leader.received_monotonic_s - old_time_s
            if 0.02 <= elapsed_s <= 1.0:
                raw_rate = math.radians(
                    wrap_degrees(leader.status.heading_deg - old_heading_deg)
                ) / elapsed_s
                raw_rate = clamp(raw_rate, -0.35, 0.35)
                self._filtered_leader_yaw_rate += 0.20 * (
                    raw_rate - self._filtered_leader_yaw_rate
                )
        self._previous_leader_heading = (
            leader.status.heading_deg,
            leader.received_monotonic_s,
        )
        return self._filtered_leader_yaw_rate

    def _target_kinematics(
        self,
        leader: LeaderFrame,
        target_x_mm: float,
        target_y_mm: float,
    ) -> tuple[float, float, float, float]:
        """Return target body velocity and yaw rate, preferring Leader's plan.

        The Leader's planned *forward* speed is a useful feed-forward term.
        Its planned wheel split is not: low-cost wheel dynamics can make the
        actual turn quite different.  Yaw feed-forward therefore comes from a
        filtered measured Leader heading change, while lateral offset motion
        remains in the preview steering loop.
        """
        if leader.has_motion_reference:
            forward = leader.forward_mm_per_second
            lateral = 0.0
            return (
                forward,
                lateral,
                abs(forward),
                self._measured_leader_yaw_rate(leader),
            )

        # Backwards-safe fallback for an older Leader that publishes pose only.
        forward = 0.0
        lateral = 0.0
        if self._previous_target:
            old_x, old_y, old_time_s = self._previous_target
            elapsed_s = leader.received_monotonic_s - old_time_s
            if 0.02 <= elapsed_s <= 1.0:
                vx = (target_x_mm - old_x) / elapsed_s
                vy = (target_y_mm - old_y) / elapsed_s
                heading = math.radians(leader.status.heading_deg)
                forward = math.cos(heading) * vx + math.sin(heading) * vy
                lateral = -math.sin(heading) * vx + math.cos(heading) * vy
        self._previous_target = (target_x_mm, target_y_mm, leader.received_monotonic_s)
        return forward, lateral, math.hypot(forward, lateral), 0.0

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
        target_forward, _target_lateral, target_speed, target_yaw_rate = self._target_kinematics(
            leader,
            target_x,
            target_y,
        )
        target_heading = leader.status.heading_deg

        follower_x, follower_y, follower_heading = self._follower_world_pose(follower)
        world_error_x = target_x - follower_x
        world_error_y = target_y - follower_y
        rho_mm = math.hypot(world_error_x, world_error_y)
        # Resolve position error in the Leader's body frame.  ``x`` is the
        # longitudinal catch-up error; ``y`` is the lateral formation error.
        longitudinal_error_mm = (
            math.cos(leader_heading_rad) * world_error_x
            + math.sin(leader_heading_rad) * world_error_y
        )
        lateral_error_mm = (
            -math.sin(leader_heading_rad) * world_error_x
            + math.cos(leader_heading_rad) * world_error_y
        )
        heading_error_deg = wrap_degrees(target_heading - follower_heading)
        # Inside the formation-position deadband, keep only heading control.
        # As soon as Leader movement makes the virtual target leave this 10 mm
        # disk, normal distance and bearing correction resumes automatically.
        position_correction_active = rho_mm > self.plan.position_deadband_mm
        control_longitudinal_error_mm = longitudinal_error_mm if position_correction_active else 0.0
        control_lateral_error_mm = lateral_error_mm if position_correction_active else 0.0
        # Preview turns the lateral error into a bounded curvature request;
        # it does not move the actual (-400,+400) formation target.
        preview_x_mm = self.plan.preview_distance_mm + max(control_longitudinal_error_mm, 0.0)
        alpha_deg = math.degrees(math.atan2(control_lateral_error_mm, preview_x_mm))
        beta_deg = heading_error_deg

        # The Leader's real separation is a collision/lost-formation monitor.
        # rho instead is distance to the left-rear virtual target used by the
        # polar controller.
        leader_distance_mm = math.hypot(
            leader.status.x_mm - follower_x,
            leader.status.y_mm - follower_y,
        )
        tracking_safe = True
        stop_reason = ""
        if leader_distance_mm < self.plan.collision_stop_distance_mm:
            tracking_safe, stop_reason = False, "leader is too close"
        elif leader_distance_mm > self.plan.max_leader_distance_mm:
            tracking_safe, stop_reason = False, "leader separation is too large"
        elif rho_mm > self.plan.max_target_error_mm:
            tracking_safe, stop_reason = False, "virtual target error is too large"

        # Longitudinal correction changes speed only; lateral and heading
        # corrections change curvature only.  This is what prevents a passed
        # target from being chased with a sharp U-turn.
        # Large formation error must not turn into a high-speed chase.  Blend
        # from a responsive near controller to a deliberately softer far
        # controller, while also reducing the maximum positive catch-up term.
        near_weight = math.exp(-(rho_mm / self.plan.distance_gain_transition_mm) ** 2)
        distance_gain = (
            self.plan.far_distance_gain_per_second
            + (self.plan.near_distance_gain_per_second - self.plan.far_distance_gain_per_second)
            * near_weight
        )
        catchup_limit = (
            self.plan.far_catchup_limit_mm_per_second
            + (self.plan.near_catchup_limit_mm_per_second - self.plan.far_catchup_limit_mm_per_second)
            * near_weight
        )
        speed_correction = distance_gain * control_longitudinal_error_mm
        if speed_correction > 0.0:
            speed_correction = min(speed_correction, catchup_limit)
        desired_forward_speed = target_forward + speed_correction
        forward_speed = clamp(
            desired_forward_speed * max(0.20, math.cos(math.radians(heading_error_deg))),
            0.0,
            self.plan.max_wheel_mm_per_second,
        )
        position_not_settled = rho_mm > self.plan.settle_target_tolerance_mm
        heading_not_settled = abs(heading_error_deg) > self.plan.settle_heading_tolerance_deg
        if leader.formation_phase == "settle" and (position_not_settled or heading_not_settled):
            # At the endpoint there is no Leader forward reference.  Keep a
            # deliberate crawl so lateral error can still be converted into a
            # smooth arc; a pure longitudinal controller would otherwise
            # command zero forever when e_x is already near zero.
            forward_speed = clamp(
                max(forward_speed, self.plan.reacquire_speed_mm_per_second),
                0.0,
                self.plan.settle_max_speed_mm_per_second,
            )
        if abs(heading_error_deg) > self.plan.reacquire_bearing_deg:
            # Preserve the no-pivot/no-reverse rule while reacquiring a large
            # initial-heading mismatch.
            forward_speed = min(
                self.plan.reacquire_speed_mm_per_second,
                desired_forward_speed,
            )
        lateral_gain = self.plan.lateral_gain_radians_per_second_per_radian
        heading_gain = self.plan.heading_gain_radians_per_second_per_radian
        if leader.formation_phase == "settle" and position_not_settled:
            # First get back to the relative position.  A large heading term
            # can oppose the lateral preview term and make the rover freeze.
            lateral_gain = 0.85
            heading_gain = 0.12
        yaw_rate = clamp(
            target_yaw_rate
            + lateral_gain * math.radians(alpha_deg)
            + heading_gain * math.radians(heading_error_deg),
            -self.plan.max_yaw_rate_radians_per_second,
            self.plan.max_yaw_rate_radians_per_second,
        )
        yaw_rate = clamp(
            yaw_rate,
            self._previous_yaw_rate - self.plan.max_yaw_step_radians_per_second,
            self._previous_yaw_rate + self.plan.max_yaw_step_radians_per_second,
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
        if self._previous_wheels:
            previous_left, previous_right = self._previous_wheels
            max_step = self.plan.max_wheel_step_mm_per_second
            left = round(clamp(left, previous_left - max_step, previous_left + max_step))
            right = round(clamp(right, previous_right - max_step, previous_right + max_step))
        self._previous_wheels = (left, right)
        self._previous_yaw_rate = yaw_rate
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
            longitudinal_error_mm=longitudinal_error_mm,
            lateral_error_mm=lateral_error_mm,
            heading_error_deg=heading_error_deg,
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
    if not 0.0 < plan.far_distance_gain_per_second <= plan.near_distance_gain_per_second <= 2.0:
        raise RuntimeError("distance gains must satisfy 0 < far <= near <= 2 per second")
    if not 10.0 <= plan.distance_gain_transition_mm <= 1000.0:
        raise RuntimeError("distance-gain transition must be 10..1000 mm")
    if not 0.0 < plan.far_catchup_limit_mm_per_second <= plan.near_catchup_limit_mm_per_second:
        raise RuntimeError("catch-up limits must satisfy 0 < far <= near")
    if not 0.0 < plan.preview_distance_mm <= 1000.0:
        raise RuntimeError("preview distance must be 0..1000 mm")
    if not 0.0 < plan.max_wheel_mm_per_second <= 100.0:
        raise RuntimeError("max wheel speed must be 0..100 mm/s")
    if plan.wheel_track_mm <= 0.0 or plan.max_yaw_rate_radians_per_second <= 0.0:
        raise RuntimeError("wheel track must be positive")
    if not 0.0 < plan.max_target_error_mm <= 2000.0:
        raise RuntimeError("maximum target error must be 0..2000 mm")
    if not 0.0 < plan.collision_stop_distance_mm < plan.max_leader_distance_mm:
        raise RuntimeError("leader distance limits must be positive and ordered")
    if not 0.0 < plan.reacquire_bearing_deg < 180.0:
        raise RuntimeError("reacquire bearing must be between 0 and 180 degrees")
    if not 0.0 < plan.reacquire_speed_mm_per_second <= plan.max_wheel_mm_per_second:
        raise RuntimeError("reacquire speed must be positive and within the wheel-speed limit")
    if not 0.0 < plan.max_wheel_step_mm_per_second <= plan.max_wheel_mm_per_second:
        raise RuntimeError("wheel command step must be positive and within the wheel-speed limit")
    if not 0.0 < plan.max_yaw_step_radians_per_second <= plan.max_yaw_rate_radians_per_second:
        raise RuntimeError("yaw command step must be positive and within the yaw-rate limit")
    if not 0.0 < plan.settle_max_speed_mm_per_second <= plan.max_wheel_mm_per_second:
        raise RuntimeError("settle speed must be positive and within the wheel-speed limit")
    if not 0.0 <= plan.position_deadband_mm <= 100.0:
        raise RuntimeError("position deadband must be 0..100 mm")
    if not MIN_MOTOR_MS <= plan.command_ms <= MAX_MOTOR_MS:
        raise RuntimeError(f"command timeout must be {MIN_MOTOR_MS}..{MAX_MOTOR_MS} ms")
    if not 0.0 < plan.control_period_s < plan.command_ms / 1000.0:
        raise RuntimeError("control period must be positive and shorter than the Nano timeout")
    if not plan.control_period_s <= plan.frame_timeout_s <= 2.0:
        raise RuntimeError("frame timeout must be at least one control period and at most 2 s")


def leader_is_safe_and_moving(frame: LeaderFrame) -> bool:
    return frame.status.ready_for_path and frame.status.mode.lower() in ACTIVE_LEADER_MODES


def leader_is_safe_for_settle(frame: LeaderFrame) -> bool:
    return (
        frame.formation_phase == "settle"
        and frame.status.ready_for_path
        and frame.status.mode.lower() == "idle"
    )


def run_follow(
    link: NanoLink,
    receiver: LeaderUdpReceiver,
    plan: FormationPlan,
    test_pwm: int,
    test_duration_ms: int,
    log_path: Path | None,
) -> None:
    """Commission F1, then follow only fresh and safe Leader packets."""
    validate_plan(plan)
    commission(link, test_pwm, test_duration_ms)
    require_confirmation(
        "FOLLOWER-1",
        "Place F1 at its declared start pose and keep it still. After confirmation Pi "
        "will RESET its odometry at that pose, then wait for Leader broadcast. Retain "
        "physical motor-power cutoff.",
    )
    # RESET must happen *after* the vehicle is physically placed. Moving a
    # rover after RESET makes its encoders/IMU interpret placement as travel,
    # which corrupts the shared formation-frame origin before the first frame.
    print(link.request("RESET").raw)
    initial = parse_status_fields(link.request("STATUS").fields)
    if not initial.ready_for_path:
        raise RuntimeError("F1 RESET did not preserve ready state; refusing formation control.")
    tracker = FormationTracker(plan)
    latest: LeaderFrame | None = None
    follower: RoverStatus | None = None
    control: FormationControl | None = None
    consecutive_active_frames = 0
    stopped_for_wait = True
    logger = FollowerCsvLogger(log_path or default_follower_log_path())
    started_at = time.monotonic()
    print(f"F1 CSV log: {logger.path}")
    print("F1 armed and listening. It remains stopped until three fresh, safe moving Leader packets arrive.")
    try:
        while True:
            frame = receiver.receive_latest(plan.control_period_s)
            if frame:
                latest = frame
                if leader_is_safe_and_moving(frame):
                    consecutive_active_frames += 1
                elif not leader_is_safe_for_settle(frame):
                    consecutive_active_frames = 0
            stale = not latest or time.monotonic() - latest.received_monotonic_s > plan.frame_timeout_s
            leader_allowed = bool(latest) and (
                leader_is_safe_and_moving(latest)
                or (leader_is_safe_for_settle(latest) and consecutive_active_frames >= 3)
            )
            if (
                stale
                or not latest
                or not leader_allowed
                or consecutive_active_frames < 3
            ):
                if not stopped_for_wait:
                    link.stop_safely()
                    stopped_for_wait = True
                    print("F1 STOP: waiting for a fresh, safe moving Leader frame.")
                    logger.record("wait_stop", started_at, latest, follower, control, "leader frame unavailable or unsafe")
                else:
                    logger.record("waiting", started_at, latest, follower, control, "leader frame unavailable or unsafe")
                continue

            follower = parse_status_fields(link.request("STATUS", wait=0.7).fields)
            if not follower.ready_for_path:
                logger.record("unsafe_follower", started_at, latest, follower, control, "F1 Nano preflight became unsafe")
                raise RuntimeError("F1 Nano preflight became unsafe; stopping.")
            control = tracker.control(latest, follower)
            if not control.tracking_safe:
                logger.record("safety_stop", started_at, latest, follower, control, control.stop_reason)
                raise RuntimeError(f"F1 formation safety gate: {control.stop_reason}.")
            if (
                latest.formation_phase == "settle"
                and control.rho_mm <= plan.settle_target_tolerance_mm
                and abs(control.heading_error_deg) <= plan.settle_heading_tolerance_deg
            ):
                link.stop_safely()
                logger.record("settled", started_at, latest, follower, control)
                print("F1 settled at the Leader-relative target.")
                return
            link.request(
                "VELOCITY",
                control.left_mm_per_second,
                control.right_mm_per_second,
                plan.command_ms,
                wait=0.7,
            )
            stopped_for_wait = False
            logger.record(f"control_{latest.formation_phase}", started_at, latest, follower, control)
            print(json.dumps({
                "leader": asdict(latest.status),
                "follower": asdict(follower),
                "control": asdict(control),
            }, separators=(",", ":")))
    finally:
        link.stop_safely()
        logger.record("final_stop", started_at, latest, follower, control)
        logger.close()
        print(f"F1 CSV log: {logger.path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe Follower-1 UDP formation controller.")
    parser.add_argument("--port", help="F1 Nano /dev/serial/by-id path; auto-detect only when exactly one exists.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--listen", default="0.0.0.0:5005", help="local UDP bind endpoint (default 0.0.0.0:5005)")
    parser.add_argument("--leader-host", required=True, help="required source IPv4 address of the Leader Pi")
    parser.add_argument("--multicast-group", help="join this IPv4 multicast group, e.g. 239.42.0.1")
    parser.add_argument("--offset-x-mm", type=float, default=-400.0, help="F1 target x in Leader body frame (default -400, rear)")
    parser.add_argument("--offset-y-mm", type=float, default=400.0, help="F1 target y in Leader body frame (default +400, left)")
    parser.add_argument("--follower-start-x-mm", type=float, default=-400.0, help="F1 Nano RESET origin in experiment frame")
    parser.add_argument("--follower-start-y-mm", type=float, default=400.0, help="F1 Nano RESET origin in experiment frame")
    parser.add_argument("--follower-start-heading-deg", type=float, default=0.0, help="F1 initial heading relative to Leader frame")
    parser.add_argument("--max-speed-mm-s", type=float, default=75.0)
    parser.add_argument("--command-ms", type=int, default=180)
    parser.add_argument("--frame-timeout-s", type=float, default=0.35)
    parser.add_argument("--test-pwm", type=int, default=80)
    parser.add_argument("--test-duration-ms", type=int, default=800)
    parser.add_argument(
        "--log",
        type=Path,
        help="CSV destination; default is ~/pi_rover/logs/follower_<UTC timestamp>.csv",
    )
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
        run_follow(link, receiver, plan, args.test_pwm, args.test_duration_ms, args.log)
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
