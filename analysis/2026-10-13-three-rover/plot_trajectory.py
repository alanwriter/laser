"""Render the completed three-rover direct-unicast run as a single JPG."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

# This report is generated headlessly; never invoke a macOS GUI backend.
matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).parent


def number(row: dict[str, str], name: str) -> float | None:
    value = row.get(name, "")
    try:
        return float(value) if value else None
    except ValueError:
        return None


def leader_trace() -> list[tuple[float, float, float]]:
    trace: list[tuple[float, float, float]] = []
    with (HERE / "leader_wave.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            x = number(row, "status_x_mm")
            y = number(row, "status_y_mm")
            heading = number(row, "status_heading_deg")
            if x is not None and y is not None and heading is not None:
                trace.append((x, y, heading))
    return trace


def follower_trace(filename: str, start_x: float, start_y: float) -> list[tuple[float, float]]:
    trace: list[tuple[float, float]] = []
    with (HERE / filename).open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            # Only control records contain a measured follower pose.  Map its
            # Nano RESET-local odometry into the common Leader experiment frame.
            if not row["event"].startswith("control_"):
                continue
            local_x = number(row, "follower_x_mm")
            local_y = number(row, "follower_y_mm")
            if local_x is not None and local_y is not None:
                trace.append((start_x + local_x, start_y + local_y))
    return trace


def virtual_target(trace: list[tuple[float, float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for x, y, heading_deg in trace:
        heading = math.radians(heading_deg)
        result.append((x + math.cos(heading) * dx - math.sin(heading) * dy,
                       y + math.sin(heading) * dx + math.cos(heading) * dy))
    return result


def plot_line(axis: plt.Axes, points: list[tuple[float, float]], label: str, color: str, style: str = "-") -> None:
    axis.plot([point[0] for point in points], [point[1] for point in points], style,
              color=color, linewidth=2.3 if style == "-" else 1.5, label=label)
    axis.scatter(points[0][0], points[0][1], color=color, s=36, zorder=4)
    axis.scatter(points[-1][0], points[-1][1], color=color, s=42, marker="X", zorder=4)


def main() -> None:
    leader = leader_trace()
    f1 = follower_trace("follower1.csv", -400.0, 400.0)
    f2 = follower_trace("follower2.csv", -400.0, -400.0)
    f1_target = virtual_target(leader, -400.0, 400.0)
    f2_target = virtual_target(leader, -400.0, -400.0)

    figure, axis = plt.subplots(figsize=(16, 7.2), dpi=180)
    plot_line(axis, [(x, y) for x, y, _ in leader], "Leader measured", "#1769aa")
    plot_line(axis, f1, "Follower 1 measured", "#df7700")
    plot_line(axis, f2, "Follower 2 measured", "#008a72")
    plot_line(axis, f1_target, "F1 virtual target (-400, +400)", "#df7700", "--")
    plot_line(axis, f2_target, "F2 virtual target (-400, -400)", "#008a72", "--")
    axis.axhline(0, color="#aeb6bf", linewidth=0.7)
    axis.axvline(0, color="#aeb6bf", linewidth=0.7)
    axis.set_xlabel("Experiment-frame x (mm)")
    axis.set_ylabel("Experiment-frame y (mm)")
    axis.grid(alpha=0.25)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(-600, 3200)
    axis.set_ylim(-750, 750)
    axis.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=3,
                frameon=False, fontsize=10)
    figure.suptitle("Leader, F1 and F2 trajectories — 2026-10-13 direct-unicast run", fontsize=16, weight="bold")
    figure.text(0.5, 0.01,
                "Solid = measured Nano odometry; dashed = Leader-body virtual formation target. "
                "Circle = start, X = final logged point.",
                ha="center", fontsize=9, color="#3f4a55")
    figure.subplots_adjust(top=0.88, bottom=0.23, left=0.08, right=0.99)
    figure.savefig(HERE / "three_rover_trajectory.jpg", format="jpg", dpi=180,
                   pil_kwargs={"quality": 95})


if __name__ == "__main__":
    main()
