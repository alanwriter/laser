"""Follower lane rebuilt from the Leader's broadcast trajectory ("breadcrumbs").

The analytic ``TurnPath``/``WavePath`` classes know the planned route in closed
form.  ``BreadcrumbPath`` offers the same three methods the path follower uses
(``pose_at_s``, ``project``, ``speed_ratio_at_s``) but builds the route from the
Leader poses received over UDP, so the follower control law is unchanged and
the Leader may drive any path.

Path parameter ``s`` is the Leader's travelled arc length, measured along the
recorded points from the first received pose (``s = 0``).  The lane of a
follower is the Leader track offset ``d`` mm along its local left normal.

Tangent and curvature come from a local linear regression of the broadcast
MPU heading against ``s`` over a short window.  Differencing neighbouring
positions would also give a tangent, but 20 mm point spacing turns millimetre
odometry noise into degrees of heading noise and makes curvature unusable.
Beyond the newest point the lane is extended by a constant-curvature arc that
passes through the Leader's announced next target point (if any); behind the
first point it is extended straight along the first tangent, which covers
followers that start behind the Leader start line.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass


def _wrap_radians(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class LeaderTarget:
    """Leader's planned pose ``ahead_mm`` further along its own route."""

    x_mm: float
    y_mm: float
    heading_deg: float
    ahead_mm: float

    @classmethod
    def from_payload(cls, value: object) -> "LeaderTarget":
        if not isinstance(value, dict):
            raise ValueError("next_target is not an object")
        try:
            target = cls(**{name: float(value[name]) for name in cls.__dataclass_fields__})
        except (KeyError, TypeError) as error:
            raise ValueError("incomplete next_target") from error
        if not all(math.isfinite(v) for v in (target.x_mm, target.y_mm, target.heading_deg, target.ahead_mm)):
            raise ValueError("next_target values must be finite")
        if not 0.0 < target.ahead_mm <= 2000.0 or max(abs(target.x_mm), abs(target.y_mm)) > 100000.0:
            raise ValueError("next_target out of range")
        return target

    def to_payload(self) -> dict[str, float]:
        return {name: round(getattr(self, name), 2) for name in self.__dataclass_fields__}


TANGENT_SOURCES = ("heading", "position")


def _quadratic_derivatives(u: list[float], v: list[float]) -> tuple[float, float] | None:
    """Least-squares v = a + b u + c u^2; return (v'(0), v''(0)) or None."""
    sums = [sum(x ** k for x in u) for k in range(5)]
    rhs = [sum(vi * ui ** k for ui, vi in zip(u, v)) for k in range(3)]
    m = [[sums[0], sums[1], sums[2]], [sums[1], sums[2], sums[3]], [sums[2], sums[3], sums[4]]]
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    if abs(det) < 1e-6:
        return None
    def solve(col: int) -> float:
        mm = [row[:] for row in m]
        for r in range(3):
            mm[r][col] = rhs[r]
        return (mm[0][0] * (mm[1][1] * mm[2][2] - mm[1][2] * mm[2][1])
                - mm[0][1] * (mm[1][0] * mm[2][2] - mm[1][2] * mm[2][0])
                + mm[0][2] * (mm[1][0] * mm[2][1] - mm[1][1] * mm[2][0])) / det
    return solve(1), 2.0 * solve(2)


class BreadcrumbPath:
    """Leader trajectory with the TurnPath interface used by the path follower."""

    def __init__(
        self,
        spacing_mm: float = 20.0,
        smoothing_window_mm: float = 160.0,
        keep_behind_mm: float = 1000.0,
        max_extension_mm: float = 400.0,
        max_jump_mm: float = 300.0,
        max_curvature_per_mm: float = 1.0 / 250.0,
        projection_window_mm: float = 500.0,
        tangent_source: str = "heading",
    ) -> None:
        if tangent_source not in TANGENT_SOURCES:
            raise ValueError(f"tangent source must be one of {TANGENT_SOURCES}")
        if not 5.0 <= spacing_mm <= 100.0:
            raise ValueError("breadcrumb spacing must be 5..100 mm")
        if smoothing_window_mm < 2.0 * spacing_mm:
            raise ValueError("smoothing window must span at least two breadcrumb spacings")
        self.spacing_mm = spacing_mm
        self.smoothing_window_mm = smoothing_window_mm
        self.keep_behind_mm = keep_behind_mm
        self.max_extension_mm = max_extension_mm
        self.max_jump_mm = max_jump_mm
        self.max_curvature_per_mm = max_curvature_per_mm
        self.projection_window_mm = projection_window_mm
        self.tangent_source = tangent_source
        self._s: list[float] = []
        self._x: list[float] = []
        self._y: list[float] = []
        self._psi: list[float] = []          # unwrapped broadcast heading, rad
        self._tangent: list[float] = []      # smoothed heading, rad
        self._curvature: list[float] = []    # centre-line curvature, 1/mm
        self._dirty = False
        self._head: tuple[float, float, float, float] | None = None  # x, y, psi, s
        self.target: LeaderTarget | None = None
        self._projection_hint: float | None = None
        self.pruned_count = 0

    # ------------------------------------------------------------------ input
    @property
    def point_count(self) -> int:
        return len(self._s)

    @property
    def leader_s_mm(self) -> float:
        """Arc length of the most recent Leader pose (0 before any pose)."""
        return self._head[3] if self._head else 0.0

    @property
    def end_s_mm(self) -> float:
        return self._s[-1] if self._s else 0.0

    def observe(self, x_mm: float, y_mm: float, heading_deg: float,
                target: LeaderTarget | None = None) -> None:
        """Add one broadcast Leader pose; commit a point every ``spacing_mm``."""
        if not all(math.isfinite(v) for v in (x_mm, y_mm, heading_deg)):
            raise ValueError("leader pose must be finite")
        heading = math.radians(heading_deg)
        if target is not None:
            self.target = target
        if self._head is None:
            self._append(0.0, x_mm, y_mm, heading)
            self._head = (x_mm, y_mm, heading, 0.0)
            return
        head_x, head_y, head_psi, _ = self._head
        if math.hypot(x_mm - head_x, y_mm - head_y) > self.max_jump_mm:
            raise ValueError("leader pose jumped; breadcrumb trajectory is no longer continuous")
        psi = head_psi + _wrap_radians(heading - head_psi)
        step = math.hypot(x_mm - self._x[-1], y_mm - self._y[-1])
        if step >= self.spacing_mm:
            self._append(self._s[-1] + step, x_mm, y_mm, psi)
            step = 0.0
        self._head = (x_mm, y_mm, psi, self._s[-1] + step)

    def _append(self, s: float, x: float, y: float, psi: float) -> None:
        self._s.append(s)
        self._x.append(x)
        self._y.append(y)
        self._psi.append(psi)
        self._dirty = True

    def prune(self, follower_s_mm: float) -> None:
        """Drop points more than ``keep_behind_mm`` behind the follower."""
        cutoff = follower_s_mm - self.keep_behind_mm
        drop = bisect.bisect_left(self._s, cutoff)
        drop = min(drop, max(0, len(self._s) - 2))
        if drop > 0:
            for values in (self._s, self._x, self._y, self._psi):
                del values[:drop]
            self.pruned_count += drop
            self._dirty = True

    # ------------------------------------------------------------- smoothing
    def _refresh(self) -> None:
        if not self._dirty:
            return
        n = len(self._s)
        half = self.smoothing_window_mm / 2.0
        tangent: list[float] = []
        curvature: list[float] = []
        lo = hi = 0
        for i in range(n):
            while self._s[lo] < self._s[i] - half:
                lo += 1
            while hi + 1 < n and self._s[hi + 1] <= self._s[i] + half:
                hi += 1
            count = hi - lo + 1
            if count < 3:
                tangent.append(self._psi[i])
                curvature.append(0.0)
                continue
            if self.tangent_source == "position":
                u = [s - self._s[i] for s in self._s[lo:hi + 1]]
                dx = _quadratic_derivatives(u, self._x[lo:hi + 1])
                dy = _quadratic_derivatives(u, self._y[lo:hi + 1])
                if dx is not None and dy is not None and math.hypot(dx[0], dy[0]) > 0.5:
                    (x1, x2), (y1, y2) = dx, dy
                    direction = math.atan2(y1, x1)
                    reference = tangent[-1] if tangent else self._psi[i]
                    kappa = (x1 * y2 - y1 * x2) / math.hypot(x1, y1) ** 3
                    tangent.append(reference + _wrap_radians(direction - reference))
                    curvature.append(max(-self.max_curvature_per_mm, min(self.max_curvature_per_mm, kappa)))
                    continue
            mean_s = sum(self._s[lo:hi + 1]) / count
            mean_psi = sum(self._psi[lo:hi + 1]) / count
            sxx = sum((s - mean_s) ** 2 for s in self._s[lo:hi + 1])
            sxy = sum((s - mean_s) * (p - mean_psi)
                      for s, p in zip(self._s[lo:hi + 1], self._psi[lo:hi + 1]))
            slope = sxy / sxx if sxx > 1e-9 else 0.0
            slope = max(-self.max_curvature_per_mm, min(self.max_curvature_per_mm, slope))
            tangent.append(mean_psi + slope * (self._s[i] - mean_s))
            curvature.append(slope)
        self._tangent, self._curvature = tangent, curvature
        self._dirty = False

    def _centre(self, s_mm: float) -> tuple[float, float, float, float]:
        """Centre-line x, y, tangent (rad) and curvature at ``s_mm``."""
        if not self._s:
            raise ValueError("breadcrumb path has no Leader pose yet")
        self._refresh()
        if s_mm <= self._s[0]:
            t = self._tangent[0]
            ds = s_mm - self._s[0]
            return self._x[0] + ds * math.cos(t), self._y[0] + ds * math.sin(t), t, 0.0
        if s_mm <= self._s[-1]:
            i = bisect.bisect_right(self._s, s_mm) - 1
            i = min(i, len(self._s) - 2)
            f = (s_mm - self._s[i]) / (self._s[i + 1] - self._s[i])
            lerp = lambda a: a[i] + f * (a[i + 1] - a[i])  # noqa: E731
            return lerp(self._x), lerp(self._y), lerp(self._tangent), lerp(self._curvature)
        return self._extension(min(s_mm, self._s[-1] + self.max_extension_mm))

    def _extension_curvature(self) -> float:
        if self.target is None:
            return self._curvature[-1]
        t = self._tangent[-1]
        dx, dy = self.target.x_mm - self._x[-1], self.target.y_mm - self._y[-1]
        chord = math.hypot(dx, dy)
        theta = _wrap_radians(math.atan2(dy, dx) - t)
        if chord < self.spacing_mm or math.cos(theta) <= 0.0:
            return self._curvature[-1]
        kappa = 2.0 * math.sin(theta) / chord
        return max(-self.max_curvature_per_mm, min(self.max_curvature_per_mm, kappa))

    def _extension(self, s_mm: float) -> tuple[float, float, float, float]:
        t0, x0, y0 = self._tangent[-1], self._x[-1], self._y[-1]
        kappa = self._extension_curvature()
        ds = s_mm - self._s[-1]
        if abs(kappa) < 1e-9:
            return x0 + ds * math.cos(t0), y0 + ds * math.sin(t0), t0, 0.0
        t = t0 + kappa * ds
        return (x0 + (math.sin(t) - math.sin(t0)) / kappa,
                y0 - (math.cos(t) - math.cos(t0)) / kappa, t, kappa)

    # ------------------------------------------------- TurnPath-like interface
    def speed_ratio_at_s(self, s_mm: float, lateral_offset_mm: float) -> float:
        _, _, _, kappa = self._centre(s_mm)
        return max(0.2, 1.0 - lateral_offset_mm * kappa)

    def pose_at_s(self, s_mm: float, lateral_offset_mm: float = 0.0) -> tuple[float, float, float, float]:
        x, y, t, kappa = self._centre(s_mm)
        denominator = max(0.2, 1.0 - lateral_offset_mm * kappa)
        return (x - lateral_offset_mm * math.sin(t), y + lateral_offset_mm * math.cos(t),
                math.degrees(t), kappa / denominator)

    def centre_curvature_at_s(self, s_mm: float) -> float:
        return self._centre(s_mm)[3]

    def project(self, world_x_mm: float, world_y_mm: float,
                lateral_offset_mm: float) -> tuple[float, float]:
        """Nearest point of the offset lane, searched near the previous result."""
        if not self._s:
            raise ValueError("breadcrumb path has no Leader pose yet")
        self._refresh()
        knots = [self._s[0] - 5000.0, *self._s]
        extension_end = self._s[-1] + self.max_extension_mm
        step = 50.0
        s = self._s[-1] + step
        while s < extension_end:
            knots.append(s)
            s += step
        knots.append(extension_end)
        best_s, best_distance = math.inf, math.inf
        if self._projection_hint is not None:
            # Search near the previous answer so a lane that curves back near
            # itself cannot capture the follower; fall back if it moved far.
            lo = self._projection_hint - self.projection_window_mm
            hi = self._projection_hint + self.projection_window_mm
            first = max(0, bisect.bisect_left(knots, lo) - 1)
            last = min(len(knots), bisect.bisect_right(knots, hi) + 1)
            best_s, best_distance = self._nearest(knots[first:last], world_x_mm, world_y_mm, lateral_offset_mm)
        if best_distance > self.projection_window_mm / 2.0:
            best_s, best_distance = self._nearest(knots, world_x_mm, world_y_mm, lateral_offset_mm)
        self._projection_hint = best_s
        px, py, heading_deg, _ = self.pose_at_s(best_s, lateral_offset_mm)
        tangent = math.radians(heading_deg)
        signed_lateral = -(world_x_mm - px) * math.sin(tangent) + (world_y_mm - py) * math.cos(tangent)
        return best_s, signed_lateral

    def _nearest(self, knots: list[float], world_x_mm: float, world_y_mm: float,
                 lateral_offset_mm: float) -> tuple[float, float]:
        lane = [self.pose_at_s(k, lateral_offset_mm)[:2] for k in knots]
        best_s, best_distance = knots[0], math.inf
        for (s0, (x0, y0)), (s1, (x1, y1)) in zip(zip(knots, lane), zip(knots[1:], lane[1:])):
            ex, ey = x1 - x0, y1 - y0
            length_sq = ex * ex + ey * ey
            f = 0.0 if length_sq < 1e-9 else max(0.0, min(1.0, (
                (world_x_mm - x0) * ex + (world_y_mm - y0) * ey) / length_sq))
            distance = math.hypot(world_x_mm - (x0 + f * ex), world_y_mm - (y0 + f * ey))
            if distance < best_distance:
                best_s, best_distance = s0 + f * (s1 - s0), distance
        return best_s, best_distance
