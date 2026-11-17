"""Shared, read-only geometry for the Pi-planned eased-sine route.

The leader sends the route parameters and measured progress.  Followers use
this module to reconstruct a path; no network packet contains motor commands.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class WavePathReference:
    length_mm: float
    lead_in_mm: float
    amplitude_mm: float
    cycles: int
    origin_x_mm: float
    origin_y_mm: float
    origin_heading_deg: float
    progress_s_mm: float = 0.0

    @classmethod
    def from_payload(cls, value: object) -> "WavePathReference":
        if not isinstance(value, dict) or value.get("kind") != "eased_sine_v1":
            raise ValueError("unsupported wave path")
        try:
            reference = cls(**{
                name: value[name] for name in cls.__dataclass_fields__
            })
        except (KeyError, TypeError) as error:
            raise ValueError("incomplete wave path") from error
        reference.validate()
        return reference

    def validate(self) -> None:
        values = (
            self.length_mm, self.lead_in_mm, self.amplitude_mm,
            self.origin_x_mm, self.origin_y_mm, self.origin_heading_deg,
            self.progress_s_mm,
        )
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError("wave path values must be finite numbers")
        if not 1000 <= self.length_mm <= 5000:
            raise ValueError("wave path length out of range")
        if not 0 <= self.lead_in_mm <= self.length_mm - 600:
            raise ValueError("wave path lead-in out of range")
        if not 0 < self.amplitude_mm <= 300:
            raise ValueError("wave path amplitude out of range")
        if isinstance(self.cycles, bool) or not isinstance(self.cycles, int) or not 1 <= self.cycles <= 3:
            raise ValueError("wave path cycles out of range")
        if any(abs(v) > 100000 for v in (self.origin_x_mm, self.origin_y_mm)):
            raise ValueError("wave path origin out of range")
        if abs(self.origin_heading_deg) > 360:
            raise ValueError("wave path heading out of range")
        if not 0 <= self.progress_s_mm <= 25000:
            raise ValueError("wave path progress out of range")

    def to_payload(self) -> dict[str, float | int | str]:
        self.validate()
        return {"kind": "eased_sine_v1", **self.__dict__}

    @property
    def geometry_key(self) -> tuple[float, ...]:
        return (
            self.length_mm, self.lead_in_mm, self.amplitude_mm,
            float(self.cycles), self.origin_x_mm, self.origin_y_mm,
            self.origin_heading_deg,
        )


class WavePath:
    """Eased-sine geometry parameterized by arc length, including straight ends."""

    def __init__(self, reference: WavePathReference, sample_mm: float = 5.0) -> None:
        reference.validate()
        self.reference = reference
        count = math.ceil(reference.length_mm / sample_mm)
        self.x_samples = [reference.length_mm * i / count for i in range(count + 1)]
        self.s_samples = [0.0]
        previous_slope = self.shape(0.0)[1]
        for i in range(1, len(self.x_samples)):
            x = self.x_samples[i]
            slope = self.shape(x)[1]
            dx = x - self.x_samples[i - 1]
            self.s_samples.append(
                self.s_samples[-1] + dx * (
                    math.hypot(1.0, previous_slope) + math.hypot(1.0, slope)
                ) / 2.0
            )
            previous_slope = slope

    @property
    def end_s_mm(self) -> float:
        return self.s_samples[-1]

    def shape(self, x_mm: float) -> tuple[float, float, float]:
        r = self.reference
        if x_mm <= r.lead_in_mm or x_mm >= r.length_mm:
            return 0.0, 0.0, 0.0
        span = r.length_mm - r.lead_in_mm
        rate = 2.0 * math.pi * r.cycles / span
        phase = rate * (x_mm - r.lead_in_mm)
        scale = 8.0 / (3.0 * math.sqrt(3.0))
        envelope = 0.5 * (1.0 - math.cos(phase))
        y = r.amplitude_mm * scale * math.sin(phase) * envelope
        slope = r.amplitude_mm * scale * rate * (
            math.cos(phase) * envelope + 0.5 * math.sin(phase) ** 2
        )
        second = r.amplitude_mm * scale * rate ** 2 * (
            -math.sin(phase) * envelope + 1.5 * math.sin(phase) * math.cos(phase)
        )
        curvature = second / (1.0 + slope * slope) ** 1.5
        return y, slope, curvature

    def s_at_x(self, x_mm: float) -> float:
        if x_mm <= 0:
            return x_mm
        if x_mm >= self.reference.length_mm:
            return self.end_s_mm + x_mm - self.reference.length_mm
        i = bisect.bisect_right(self.x_samples, x_mm) - 1
        ratio = (x_mm - self.x_samples[i]) / (self.x_samples[i + 1] - self.x_samples[i])
        return self.s_samples[i] + ratio * (self.s_samples[i + 1] - self.s_samples[i])

    def x_at_s(self, s_mm: float) -> float:
        if s_mm <= 0:
            return s_mm
        if s_mm >= self.end_s_mm:
            return self.reference.length_mm + s_mm - self.end_s_mm
        i = bisect.bisect_right(self.s_samples, s_mm) - 1
        ratio = (s_mm - self.s_samples[i]) / (self.s_samples[i + 1] - self.s_samples[i])
        return self.x_samples[i] + ratio * (self.x_samples[i + 1] - self.x_samples[i])

    def pose_at_s(self, s_mm: float, lateral_offset_mm: float = 0.0) -> tuple[float, float, float, float]:
        x = self.x_at_s(s_mm)
        y, slope, curvature = self.shape(x)
        r = self.reference
        angle = math.radians(r.origin_heading_deg)
        local_y = y + lateral_offset_mm
        return (
            r.origin_x_mm + math.cos(angle) * x - math.sin(angle) * local_y,
            r.origin_y_mm + math.sin(angle) * x + math.cos(angle) * local_y,
            r.origin_heading_deg + math.degrees(math.atan(slope)),
            curvature,
        )

    def project(self, world_x_mm: float, world_y_mm: float, lateral_offset_mm: float) -> tuple[float, float]:
        """Return nearest route arc length and signed lateral error."""
        r = self.reference
        angle = math.radians(r.origin_heading_deg)
        dx = world_x_mm - r.origin_x_mm
        dy = world_y_mm - r.origin_y_mm
        local_x = math.cos(angle) * dx + math.sin(angle) * dy
        local_y = -math.sin(angle) * dx + math.cos(angle) * dy - lateral_offset_mm
        # Search a local x window; the route is a single-valued y(x).
        lo = max(0.0, local_x - 400.0)
        hi = min(r.length_mm, local_x + 400.0)
        candidates = [lo, hi]
        first = bisect.bisect_left(self.x_samples, lo)
        last = bisect.bisect_right(self.x_samples, hi)
        candidates.extend(self.x_samples[first:last])
        if local_x < 0:
            candidates.append(local_x)
        if local_x > r.length_mm:
            candidates.append(local_x)
        nearest_x = min(candidates, key=lambda x: (local_x - x) ** 2 + (local_y - self.shape(x)[0]) ** 2)
        # Refine the nearest sampled point to sub-millimetre path resolution.
        left = max(0.0, nearest_x - 5.0)
        right = min(r.length_mm, nearest_x + 5.0)
        if 0 < nearest_x < r.length_mm:
            for _ in range(16):
                a = left + (right - left) / 3.0
                b = right - (right - left) / 3.0
                da = (local_x - a) ** 2 + (local_y - self.shape(a)[0]) ** 2
                db = (local_x - b) ** 2 + (local_y - self.shape(b)[0]) ** 2
                if da <= db:
                    right = b
                else:
                    left = a
            nearest_x = (left + right) / 2.0
        path_y, slope, _ = self.shape(nearest_x)
        lateral_error = (local_y - path_y - slope * (local_x - nearest_x)) / math.hypot(1.0, slope)
        return self.s_at_x(nearest_x), lateral_error
