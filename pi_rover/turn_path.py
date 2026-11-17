"""Straight, quarter-circle left turn, and straight exit route geometry.

Progress is Leader arc length. A follower lane is offset along the local left
normal, so its inner/outer arc has a different radius and speed ratio.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TurnPathReference:
    straight_mm: float
    radius_mm: float
    exit_mm: float
    origin_x_mm: float
    origin_y_mm: float
    origin_heading_deg: float
    progress_s_mm: float = 0.0

    @classmethod
    def from_payload(cls, value: object) -> "TurnPathReference":
        if not isinstance(value, dict) or value.get("kind") != "left_turn_v1":
            raise ValueError("unsupported turn path")
        try:
            reference = cls(**{name: value[name] for name in cls.__dataclass_fields__})
        except (KeyError, TypeError) as error:
            raise ValueError("incomplete turn path") from error
        reference.validate()
        return reference

    def validate(self) -> None:
        values = (self.straight_mm, self.radius_mm, self.exit_mm,
                  self.origin_x_mm, self.origin_y_mm, self.origin_heading_deg,
                  self.progress_s_mm)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                   for v in values):
            raise ValueError("turn path values must be finite numbers")
        if not 150 <= self.straight_mm <= 5000 or not 800 <= self.radius_mm <= 5000:
            raise ValueError("turn straight or radius out of range")
        if not 200 <= self.exit_mm <= 5000:
            raise ValueError("turn exit out of range")
        if any(abs(v) > 100000 for v in (self.origin_x_mm, self.origin_y_mm)):
            raise ValueError("turn origin out of range")
        if abs(self.origin_heading_deg) > 360:
            raise ValueError("turn origin heading out of range")
        if not 0 <= self.progress_s_mm <= self.end_s_mm + 500:
            raise ValueError("turn progress out of range")

    @property
    def end_s_mm(self) -> float:
        return self.straight_mm + self.radius_mm * math.pi / 2 + self.exit_mm

    @property
    def length_mm(self) -> float:
        """Compatibility with the CSV route-length column."""
        return self.end_s_mm

    @property
    def geometry_key(self) -> tuple[float, ...]:
        return (self.straight_mm, self.radius_mm, self.exit_mm,
                self.origin_x_mm, self.origin_y_mm, self.origin_heading_deg)

    def to_payload(self) -> dict[str, float | str]:
        self.validate()
        return {"kind": "left_turn_v1", **self.__dict__}


class TurnPath:
    def __init__(self, reference: TurnPathReference) -> None:
        reference.validate()
        self.reference = reference

    @property
    def end_s_mm(self) -> float:
        return self.reference.end_s_mm

    def speed_ratio_at_s(self, s_mm: float, lateral_offset_mm: float) -> float:
        r = self.reference
        if r.straight_mm < s_mm < r.straight_mm + math.pi * r.radius_mm / 2:
            return 1.0 - lateral_offset_mm / r.radius_mm
        return 1.0

    def pose_at_s(self, s_mm: float, lateral_offset_mm: float = 0.0) -> tuple[float, float, float, float]:
        r = self.reference
        arc_end = r.straight_mm + math.pi * r.radius_mm / 2
        if s_mm <= r.straight_mm:
            x, y, heading, curvature = s_mm, lateral_offset_mm, 0.0, 0.0
        elif s_mm <= arc_end:
            theta = (s_mm - r.straight_mm) / r.radius_mm
            lane_radius = r.radius_mm - lateral_offset_mm
            x = r.straight_mm + lane_radius * math.sin(theta)
            y = r.radius_mm - lane_radius * math.cos(theta)
            heading, curvature = theta, 1.0 / lane_radius
        else:
            x = r.straight_mm + r.radius_mm - lateral_offset_mm
            y = r.radius_mm + s_mm - arc_end
            heading, curvature = math.pi / 2, 0.0
        origin_heading = math.radians(r.origin_heading_deg)
        return (r.origin_x_mm + x * math.cos(origin_heading) - y * math.sin(origin_heading),
                r.origin_y_mm + x * math.sin(origin_heading) + y * math.cos(origin_heading),
                r.origin_heading_deg + math.degrees(heading), curvature)

    def project(self, world_x_mm: float, world_y_mm: float,
                lateral_offset_mm: float) -> tuple[float, float]:
        """Nearest point on the start line, quarter-circle, or exit line."""
        r = self.reference
        heading = math.radians(r.origin_heading_deg)
        dx, dy = world_x_mm - r.origin_x_mm, world_y_mm - r.origin_y_mm
        x = dx * math.cos(heading) + dy * math.sin(heading)
        y = -dx * math.sin(heading) + dy * math.cos(heading)
        arc_end = r.straight_mm + math.pi * r.radius_mm / 2
        theta = max(0.0, min(math.pi / 2, math.atan2(x - r.straight_mm, r.radius_mm - y)))
        candidates = (min(x, r.straight_mm),
                      r.straight_mm + r.radius_mm * theta,
                      arc_end + max(0.0, y - r.radius_mm))
        def distance_squared(s: float) -> float:
            px, py, _, _ = self.pose_at_s(s, lateral_offset_mm)
            return (world_x_mm - px) ** 2 + (world_y_mm - py) ** 2
        s = min(candidates, key=distance_squared)
        px, py, path_heading_deg, _ = self.pose_at_s(s, lateral_offset_mm)
        tangent = math.radians(path_heading_deg)
        signed_lateral = -(world_x_mm - px) * math.sin(tangent) + (world_y_mm - py) * math.cos(tangent)
        return s, signed_lateral
