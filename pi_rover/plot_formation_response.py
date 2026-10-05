#!/usr/bin/env python3
"""Export distance, bearing and heading formation-response SVG charts."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path


W, H = 900, 900
LEFT, RIGHT, TOP, BOTTOM = 110, 50, 120, 190
PLOT_W, PLOT_H = W - LEFT - RIGHT, H - TOP - BOTTOM
BLUE, ORANGE, GRID, TEXT = "#1677b8", "#d96c0f", "#a8b5c2", "#20252b"


def wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def moving_median(values: list[float], radius: int = 4) -> list[float]:
    return [statistics.median(values[max(0, index - radius):index + radius + 1]) for index in range(len(values))]


def padded_bounds(values: list[float], reference: float = 0.0) -> tuple[float, float]:
    low, high = min(values + [reference]), max(values + [reference])
    span = max(high - low, 1.0)
    pad = max(span * 0.12, 5.0)
    return low - pad, high + pad


def svg_chart(
    title: str,
    y_label: str,
    x_values: list[float],
    raw_values: list[float],
    output: Path,
) -> None:
    smooth_values = moving_median(raw_values)
    x_low, x_high = padded_bounds(x_values)
    y_low, y_high = padded_bounds(raw_values)

    def sx(value: float) -> float:
        return LEFT + (value - x_low) / (x_high - x_low) * PLOT_W

    def sy(value: float) -> float:
        return TOP + PLOT_H - (value - y_low) / (y_high - y_low) * PLOT_H

    def polyline(values: list[float]) -> str:
        return " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in zip(x_values, values))

    grid = []
    for index in range(5):
        x = x_low + (x_high - x_low) * index / 4
        px = sx(x)
        grid.append(f'<line x1="{px:.1f}" y1="{TOP}" x2="{px:.1f}" y2="{TOP + PLOT_H}" class="grid"/>')
        grid.append(f'<text x="{px:.1f}" y="{TOP + PLOT_H + 32}" text-anchor="middle" class="tick">{x:.0f}</text>')
        y = y_low + (y_high - y_low) * index / 4
        py = sy(y)
        grid.append(f'<line x1="{LEFT}" y1="{py:.1f}" x2="{LEFT + PLOT_W}" y2="{py:.1f}" class="grid"/>')
        grid.append(f'<text x="{LEFT - 15}" y="{py + 5:.1f}" text-anchor="end" class="tick">{y:.0f}</text>')

    zero_y = sy(0.0)
    late = raw_values[-min(30, len(raw_values)):]
    late_median_abs = statistics.median(abs(value) for value in late)
    final_value = raw_values[-1]
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<style>
.title{{font:500 25px -apple-system,BlinkMacSystemFont,"Noto Sans TC",sans-serif;fill:{TEXT}}}
.sub{{font:15px -apple-system,BlinkMacSystemFont,"Noto Sans TC",sans-serif;fill:{TEXT}}}
.tick{{font:13px -apple-system,BlinkMacSystemFont,"Noto Sans TC",sans-serif;fill:{TEXT}}}
.axis{{font:16px -apple-system,BlinkMacSystemFont,"Noto Sans TC",sans-serif;fill:{TEXT}}}
.grid{{stroke:{GRID};stroke-width:1;opacity:.7}}
</style>
<rect width="100%" height="100%" fill="white"/>
<text x="{LEFT}" y="40" class="title">{title}</text>
<text x="{LEFT}" y="70" class="sub">半透明：原始 telemetry　　實線：9 點中位數　　虛線：理想隊形（0）</text>
<rect x="{LEFT}" y="{TOP}" width="{PLOT_W}" height="{PLOT_H}" fill="white" stroke="{GRID}" stroke-width="2"/>
{''.join(grid)}
<line x1="{LEFT}" y1="{zero_y:.1f}" x2="{LEFT + PLOT_W}" y2="{zero_y:.1f}" stroke="{TEXT}" stroke-width="2" stroke-dasharray="8 7" opacity=".75"/>
<polyline points="{polyline(raw_values)}" fill="none" stroke="{ORANGE}" stroke-width="2" stroke-opacity=".28" stroke-linejoin="round" stroke-linecap="round"/>
<polyline points="{polyline(smooth_values)}" fill="none" stroke="{BLUE}" stroke-width="4" stroke-linejoin="round" stroke-linecap="round"/>
<circle cx="{sx(x_values[-1]):.1f}" cy="{sy(final_value):.1f}" r="7" fill="{BLUE}"/>
<text x="{LEFT + PLOT_W - 5}" y="{TOP + 28}" text-anchor="end" class="sub">最後值 {final_value:.1f}　後段 |誤差| 中位數 {late_median_abs:.1f}</text>
<text x="{LEFT + PLOT_W / 2}" y="{H - 28}" text-anchor="middle" class="axis">Leader 前進進度（mm）</text>
<text x="30" y="{TOP + PLOT_H / 2}" text-anchor="middle" class="axis" transform="rotate(-90 30 {TOP + PLOT_H / 2})">{y_label}</text>
</svg>'''
    output.write_text(svg, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("follower_log", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    progress, distance_error, bearing_error, heading_error = [], [], [], []
    desired_distance = math.hypot(400.0, 400.0)
    desired_bearing = 135.0
    for raw in args.follower_log.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(raw)
            leader, follower, control = row["leader"], row["follower"], row["control"]
        except (json.JSONDecodeError, KeyError, TypeError):
            continue
        leader_heading = math.radians(float(leader["heading_deg"]))
        # F1 RESET origin is the declared (-400,+400) Leader-frame start pose.
        follower_x = float(follower["x_mm"]) - 400.0
        follower_y = float(follower["y_mm"]) + 400.0
        dx, dy = follower_x - float(leader["x_mm"]), follower_y - float(leader["y_mm"])
        relative_x = math.cos(leader_heading) * dx + math.sin(leader_heading) * dy
        relative_y = -math.sin(leader_heading) * dx + math.cos(leader_heading) * dy
        progress.append(float(leader["x_mm"]))
        distance_error.append(math.hypot(relative_x, relative_y) - desired_distance)
        bearing_error.append(wrap_degrees(math.degrees(math.atan2(relative_y, relative_x)) - desired_bearing))
        heading_error.append(wrap_degrees(float(follower["heading_deg"]) - float(leader["heading_deg"])))

    if len(progress) < 2:
        raise RuntimeError("not enough F1 formation telemetry rows")
    svg_chart("編隊距離誤差響應", "實際距離 − 565.7（mm）", progress, distance_error, args.output_dir / "formation-distance-response.svg")
    svg_chart("編隊相對方位角誤差響應", "實際方位 − 135°（deg）", progress, bearing_error, args.output_dir / "formation-bearing-response.svg")
    svg_chart("編隊航向角誤差響應", "F1 航向 − Leader 航向（deg）", progress, heading_error, args.output_dir / "formation-heading-response.svg")


if __name__ == "__main__":
    main()
