#!/usr/bin/env python3
"""Export JPEG-ready SVG plots from a complete Follower CSV telemetry log."""

from __future__ import annotations

import csv
import math
import statistics
import sys
from pathlib import Path


W, H = 1100, 900
LEFT, RIGHT, TOP, BOTTOM = 120, 55, 115, 120
PW, PH = W - LEFT - RIGHT, H - TOP - BOTTOM
BLUE, ORANGE, GREY, GRID, TEXT = "#1677b8", "#d96c0f", "#687584", "#cbd4dd", "#20252b"


def wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def bounds(values: list[float], extra: list[float] | None = None, pad: float = 0.10) -> tuple[float, float]:
    source = values + (extra or [])
    low, high = min(source), max(source)
    span = max(high - low, 1.0)
    return low - span * pad, high + span * pad


def median(values: list[float], radius: int = 4) -> list[float]:
    return [statistics.median(values[max(0, index - radius): index + radius + 1]) for index in range(len(values))]


def header(title: str, subtitle: str) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
        '<style>.title{font:500 30px -apple-system,BlinkMacSystemFont,"Noto Sans TC",sans-serif;fill:#20252b}.sub{font:17px -apple-system,BlinkMacSystemFont,"Noto Sans TC",sans-serif;fill:#20252b}.tick{font:15px -apple-system,BlinkMacSystemFont,"Noto Sans TC",sans-serif;fill:#20252b}.axis{font:18px -apple-system,BlinkMacSystemFont,"Noto Sans TC",sans-serif;fill:#20252b}.grid{stroke:#cbd4dd;stroke-width:1}</style>',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{LEFT}" y="44" class="title">{title}</text>',
        f'<text x="{LEFT}" y="77" class="sub">{subtitle}</text>',
    ]


def axes(lines: list[str], x_lo: float, x_hi: float, y_lo: float, y_hi: float, x_label: str, y_label: str):
    def sx(value: float) -> float:
        return LEFT + (value - x_lo) / (x_hi - x_lo) * PW
    def sy(value: float) -> float:
        return TOP + PH - (value - y_lo) / (y_hi - y_lo) * PH
    lines.append(f'<rect x="{LEFT}" y="{TOP}" width="{PW}" height="{PH}" fill="white" stroke="{GRID}" stroke-width="2"/>')
    for index in range(6):
        x = x_lo + (x_hi - x_lo) * index / 5
        y = y_lo + (y_hi - y_lo) * index / 5
        lines.append(f'<line x1="{sx(x):.1f}" y1="{TOP}" x2="{sx(x):.1f}" y2="{TOP+PH}" class="grid"/>')
        lines.append(f'<line x1="{LEFT}" y1="{sy(y):.1f}" x2="{LEFT+PW}" y2="{sy(y):.1f}" class="grid"/>')
        lines.append(f'<text x="{sx(x):.1f}" y="{TOP+PH+35}" text-anchor="middle" class="tick">{x:.0f}</text>')
        lines.append(f'<text x="{LEFT-16}" y="{sy(y)+5:.1f}" text-anchor="end" class="tick">{y:.0f}</text>')
    lines.append(f'<text x="{LEFT+PW/2}" y="{H-28}" text-anchor="middle" class="axis">{x_label}</text>')
    lines.append(f'<text x="34" y="{TOP+PH/2}" text-anchor="middle" class="axis" transform="rotate(-90 34 {TOP+PH/2})">{y_label}</text>')
    return sx, sy


def polyline(points: list[tuple[float, float]], sx, sy) -> str:
    return " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)


def parse_rows(path: Path) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        for raw in csv.DictReader(handle):
            if not raw["follower_x_mm"] or not raw["leader_x_mm"]:
                continue
            row: dict[str, float | str] = {"event": raw["event"]}
            for key, value in raw.items():
                if key == "event" or value in (None, ""):
                    continue
                try:
                    row[key] = float(value)
                except ValueError:
                    row[key] = value
            rows.append(row)
    if len(rows) < 3:
        raise RuntimeError("CSV does not contain enough active F1 telemetry rows")
    return rows


def data(rows: list[dict[str, float | str]]) -> dict[str, list[float]]:
    result = {key: [] for key in ("time", "leader_x", "leader_y", "target_x", "target_y", "follower_x", "follower_y", "relative_x", "relative_y", "distance", "bearing", "heading", "rho")}
    for row in rows:
        leader_x, leader_y = float(row["leader_x_mm"]), float(row["leader_y_mm"])
        leader_heading = math.radians(float(row["leader_heading_deg"]))
        follower_x = float(row["follower_x_mm"]) - 400.0
        follower_y = float(row["follower_y_mm"]) + 400.0
        dx, dy = follower_x - leader_x, follower_y - leader_y
        relative_x = math.cos(leader_heading) * dx + math.sin(leader_heading) * dy
        relative_y = -math.sin(leader_heading) * dx + math.cos(leader_heading) * dy
        target_x = float(row["control_target_x_mm"])
        target_y = float(row["control_target_y_mm"])
        result["time"].append(float(row["elapsed_s"]))
        result["leader_x"].append(leader_x); result["leader_y"].append(leader_y)
        result["target_x"].append(target_x); result["target_y"].append(target_y)
        result["follower_x"].append(follower_x); result["follower_y"].append(follower_y)
        result["relative_x"].append(relative_x); result["relative_y"].append(relative_y)
        result["distance"].append(math.hypot(relative_x, relative_y) - math.hypot(400.0, 400.0))
        result["bearing"].append(wrap_degrees(math.degrees(math.atan2(relative_y, relative_x)) - 135.0))
        result["heading"].append(wrap_degrees(float(row["follower_heading_deg"]) - float(row["leader_heading_deg"])))
        result["rho"].append(float(row["control_rho_mm"]))
    return result


def trajectory(d: dict[str, list[float]], destination: Path) -> None:
    lines = header("Leader 與 F1：姿態相依隊形軌跡", "藍：Leader　灰虛線：姿態旋轉後的 F1 目標　橘：F1 實際；下圖已旋轉到每筆 Leader body frame")
    # upper world-frame panel
    top, height = 120, 320
    xs = d["leader_x"] + d["target_x"] + d["follower_x"]
    ys = d["leader_y"] + d["target_y"] + d["follower_y"]
    x_lo, x_hi = bounds(xs); y_lo, y_hi = bounds(ys)
    def sx(v: float): return LEFT + (v-x_lo)/(x_hi-x_lo)*PW
    def sy(v: float): return top + height - (v-y_lo)/(y_hi-y_lo)*height
    lines.append(f'<rect x="{LEFT}" y="{top}" width="{PW}" height="{height}" fill="white" stroke="{GRID}" stroke-width="2"/>')
    for values_x, values_y, color, width, dash in ((d["leader_x"],d["leader_y"],BLUE,4,""),(d["target_x"],d["target_y"],GREY,3,' stroke-dasharray="10 8"'),(d["follower_x"],d["follower_y"],ORANGE,4,"")):
        lines.append(f'<polyline points="{polyline(list(zip(values_x,values_y)),sx,sy)}" fill="none" stroke="{color}" stroke-width="{width}"{dash} stroke-linejoin="round"/>')
    lines.append(f'<text x="{LEFT}" y="{top-12}" class="sub">世界座標軌跡</text>')
    # lower Leader-body panel
    bottom, height = 535, 245
    rx, ry = d["relative_x"], d["relative_y"]
    x_lo, x_hi = bounds(rx, [-400.0]); y_lo, y_hi = bounds(ry, [400.0])
    def sx2(v: float): return LEFT + (v-x_lo)/(x_hi-x_lo)*PW
    def sy2(v: float): return bottom + height - (v-y_lo)/(y_hi-y_lo)*height
    lines.append(f'<rect x="{LEFT}" y="{bottom}" width="{PW}" height="{height}" fill="white" stroke="{GRID}" stroke-width="2"/>')
    lines.append(f'<polyline points="{polyline(list(zip(rx,ry)),sx2,sy2)}" fill="none" stroke="{ORANGE}" stroke-width="4" stroke-linejoin="round"/>')
    lines.append(f'<circle cx="{sx2(-400):.1f}" cy="{sy2(400):.1f}" r="8" fill="{GREY}"/>')
    lines.append(f'<text x="{sx2(-400)+12:.1f}" y="{sy2(400)-12:.1f}" class="sub">目標 (-400,+400)</text>')
    lines.append(f'<text x="{LEFT}" y="{bottom-12}" class="sub">Leader body frame：F1 相對位置（理想點固定）</text>')
    lines.append(f'<text x="{LEFT+PW/2}" y="{H-26}" text-anchor="middle" class="axis">相對前後 x（mm）</text>')
    lines.append(f'<text x="36" y="{bottom+height/2}" text-anchor="middle" class="axis" transform="rotate(-90 36 {bottom+height/2})">相對左右 y（mm）</text>')
    lines.append('</svg>')
    destination.write_text("\n".join(lines), encoding="utf-8")


def response(d: dict[str, list[float]], key: str, title: str, label: str, destination: Path) -> None:
    values, times = d[key], d["time"]
    x_lo, x_hi = bounds(times, pad=0.04); y_lo, y_hi = bounds(values, [0.0], 0.12)
    lines = header(title, "半透明：原始 telemetry　實線：9 點中位數　虛線：理想隊形（0）")
    sx, sy = axes(lines, x_lo, x_hi, y_lo, y_hi, "經過時間（s；包含 Leader 停車後 settle）", label)
    lines.append(f'<line x1="{LEFT}" y1="{sy(0):.1f}" x2="{LEFT+PW}" y2="{sy(0):.1f}" stroke="{TEXT}" stroke-width="2" stroke-dasharray="8 7"/>')
    raw = polyline(list(zip(times,values)),sx,sy)
    smooth = polyline(list(zip(times,median(values))),sx,sy)
    lines.append(f'<polyline points="{raw}" fill="none" stroke="{ORANGE}" stroke-width="2" stroke-opacity=".24" stroke-linejoin="round"/>')
    lines.append(f'<polyline points="{smooth}" fill="none" stroke="{BLUE}" stroke-width="4" stroke-linejoin="round"/>')
    lines.append(f'<circle cx="{sx(times[-1]):.1f}" cy="{sy(values[-1]):.1f}" r="7" fill="{BLUE}"/>')
    lines.append(f'<text x="{LEFT+PW-5}" y="{TOP+28}" text-anchor="end" class="sub">最後值 {values[-1]:.1f}</text>')
    lines.append('</svg>')
    destination.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    csv_path, output = Path(sys.argv[1]), Path(sys.argv[2])
    output.mkdir(parents=True, exist_ok=True)
    d = data(parse_rows(csv_path))
    trajectory(d, output / "wave300-repeat-relative-trajectory.svg")
    response(d, "distance", "編隊距離誤差：姿態相依相對座標", "實際距離 − 565.7（mm）", output / "wave300-repeat-distance-response.svg")
    response(d, "bearing", "編隊相對方位誤差：姿態相依相對座標", "實際方位 − 135°（deg）", output / "wave300-repeat-bearing-response.svg")
    response(d, "heading", "編隊航向角誤差", "F1 航向 − Leader 航向（deg）", output / "wave300-repeat-heading-response.svg")


if __name__ == "__main__":
    main()
