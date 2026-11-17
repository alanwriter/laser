#!/usr/bin/env python3
"""Render the Leader-only 1.5-cycle smooth sine candidate in MPU coordinates."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


HERE = Path(__file__).resolve().parent
LENGTH_MM = 2000.0
AMPLITUDE_MM = 70.0


def shape(x_mm: float) -> tuple[float, float, float]:
    phase = 3.0 * math.pi * max(0.0, min(LENGTH_MM, x_mm)) / LENGTH_MM
    rate = 3.0 * math.pi / LENGTH_MM
    sine, cosine = math.sin(phase), math.cos(phase)
    y = -AMPLITUDE_MM * sine ** 3
    slope = -3.0 * AMPLITUDE_MM * rate * sine ** 2 * cosine
    second = -3.0 * AMPLITUDE_MM * rate ** 2 * (2.0 * sine * cosine ** 2 - sine ** 3)
    return y, slope, second / (1.0 + slope ** 2) ** 1.5


def footprint(x_mm: float) -> Polygon:
    y_mm, slope, _ = shape(x_mm)
    angle = math.atan(slope)
    corners = ((100, 75), (100, -75), (-150, -75), (-150, 75))
    points = [(x_mm + u * math.cos(angle) - v * math.sin(angle),
               y_mm + u * math.sin(angle) + v * math.cos(angle))
              for u, v in corners]
    return Polygon(points, closed=True, facecolor="#1769aa", edgecolor="#1769aa",
                   alpha=0.11, linestyle="--", linewidth=1.0)


def main() -> None:
    xs = [LENGTH_MM * i / 500 for i in range(501)]
    ys = [shape(x)[0] for x in xs]
    headings = [math.degrees(math.atan(shape(x)[1])) for x in xs]
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 6.0), sharex=True,
                             gridspec_kw={"height_ratios": [2.4, 1]}, constrained_layout=True)
    axes[0].axhline(0, color="#777777", linestyle=":", linewidth=1, label="Center tape")
    axes[0].plot(xs, ys, color="#1769aa", linewidth=2.2, label="Desired MPU path")
    for x in (0, LENGTH_MM / 6, LENGTH_MM / 2, LENGTH_MM * 5 / 6, LENGTH_MM):
        axes[0].add_patch(footprint(x))
        axes[0].scatter([x], [shape(x)[0]], s=25, color="#1769aa", zorder=3)
    axes[0].set(ylabel="Lateral y (mm)", ylim=(-170, 170),
                title="Leader-only candidate: three visible lobes, net forward 2 m")
    axes[0].legend(loc="upper right")
    axes[1].plot(xs, headings, color="#614197", linewidth=1.8)
    axes[1].axhline(0, color="#777777", linestyle=":", linewidth=1)
    axes[1].set(xlabel="Forward x from MPU origin (mm)", ylabel="Tangent (deg)")
    for ax in axes:
        ax.grid(alpha=0.18)
        ax.spines[["top", "right"]].set_visible(False)
    fig.savefig(HERE / "path.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
