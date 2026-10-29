"""Overlay logged targets, MPU odometry, and scaled rover bodies."""

from __future__ import annotations

import csv
import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pi_rover"))
from wave_path import WavePath, WavePathReference  # noqa: E402
from vehicle_footprint import draw_body  # noqa: E402

HERE = Path(__file__).resolve().parent
def rows(logs: Path, name: str, event: str) -> list[dict[str, str]]:
    with (logs / f"{name}.csv").open(newline="") as stream:
        return [row for row in csv.DictReader(stream) if row["event"] == event]


def xy(data: list[dict[str, str]], x: str, y: str,
       dx: float = 0.0, dy: float = 0.0) -> tuple[list[float], list[float]]:
    return (
        [float(row[x]) + dx for row in data],
        [float(row[y]) + dy for row in data],
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=HERE / "real_run_084014Z")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--note", default=None)
    args = parser.parse_args()
    leader = rows(args.run_dir, "leader", "control")
    f1 = rows(args.run_dir, "f1", "control_moving")
    f2 = rows(args.run_dir, "f2", "control_moving")
    route = WavePath(WavePathReference(1500, 0, 50, 3, 0, 0, 0))
    plan = [route.pose_at_s(route.end_s_mm * i / 600) for i in range(601)]

    fig, ax = plt.subplots(figsize=(15, 8.5))
    colors = {"leader": "#252a30", "f1": "#2478b3", "f2": "#c66b21"}
    ax.plot([p[0] for p in plan], [p[1] for p in plan],
            color=colors["leader"], linestyle="--", linewidth=2.1,
            label="Leader planned path")
    ax.plot(*xy(leader, "status_x_mm", "status_y_mm"),
            color=colors["leader"], linewidth=2.4,
            label="Leader odometry")

    for name, data, dx, dy, color, label in (
        ("f1", f1, -400, 400, colors["f1"], "F1 old controller"),
        ("f2", f2, -400, -400, colors["f2"], "F2 path controller"),
    ):
        target = xy(data, "control_target_x_mm", "control_target_y_mm")
        actual = xy(data, "follower_x_mm", "follower_y_mm", dx, dy)
        ax.plot(*target, color=color, linestyle="--", linewidth=1.35,
                alpha=0.55,
                label=f"{label} target")
        ax.plot(*actual, color=color, linewidth=2.6,
                label=f"{label} odometry")
        ax.scatter(target[0][-1], target[1][-1], marker="x", s=75,
                   linewidths=2.1, color=color, zorder=5)
        ax.plot([target[0][-1], actual[0][-1]],
                [target[1][-1], actual[1][-1]], color=color,
                linestyle=":", linewidth=1.2)
        ax.annotate(f"{label} end · {float(data[-1]['control_rho_mm']):.0f} mm target gap",
                    (actual[0][-1], actual[1][-1]), xytext=(10, 15),
                    textcoords="offset points", color=color, fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.78,
                          "edgecolor": "none", "pad": 1.5}, zorder=11)

    for name, data, dx, dy, xfield, yfield, heading_field in (
        ("leader", leader, 0, 0, "status_x_mm", "status_y_mm", "status_heading_deg"),
        ("f1", f1, -400, 400, "follower_x_mm", "follower_y_mm", "follower_heading_deg"),
        ("f2", f2, -400, -400, "follower_x_mm", "follower_y_mm", "follower_heading_deg"),
    ):
        for index in (0, len(data) // 2, len(data) - 1):
            sample = data[index]
            draw_body(ax, float(sample[xfield]) + dx, float(sample[yfield]) + dy,
                      float(sample[heading_field]), colors[name],
                      prominent=index == len(data) - 1)

    ax.set(xlim=(-580, 1730), ylim=(-610, 710),
           xlabel="x in shared formation frame (mm)",
           ylabel="y in shared formation frame (mm)",
           title=f"2026-10-29 {args.run_dir.name} | planned route, estimated MPU-anchor poses, and rover footprints")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.16)
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Line2D([], [], color="#52606d", ls=(0, (3, 2)), lw=1.4,
                          marker="o", markerfacecolor="white", markeredgecolor="#52606d",
                          label="Rover outline + MPU coordinate"))
    labels.append("Rover outline + MPU coordinate")
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.075),
               ncol=4, fontsize=9, frameon=False)
    fig.text(0.5, 0.045,
             "Body 250 × 150 mm: MPU to nose +100 mm, tail −150 mm, left/right ±75 mm. "
             "Map positions are onboard odometry, not camera ground truth.",
             ha="center", fontsize=8.8)
    note = args.note
    if note is None and args.run_dir.name == "real_run_084014Z":
        note = "Final video: F1 and Leader nearly touch, while CSV body outlines remain about 203 mm apart. See final_spacing_check.png."
    if note:
        fig.text(0.5, 0.019, note, ha="center", fontsize=9,
                 color="#a33455" if args.run_dir.name == "real_run_084014Z" else "#3f4a55")
    fig.subplots_adjust(bottom=0.21, top=0.92)
    output = args.output or args.run_dir / "target_vs_odometry.png"
    fig.savefig(output, dpi=180)
    print(output)


if __name__ == "__main__":
    main()
