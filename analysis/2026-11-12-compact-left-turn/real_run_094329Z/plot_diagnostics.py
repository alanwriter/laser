"""Plot this run's onboard estimates and controller targets (not ground truth)."""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "pi_rover"))
from turn_path import TurnPath, TurnPathReference  # noqa: E402

COLORS = {"Leader": "#374151", "F1": "#2563eb", "F2": "#ea580c"}


def read(name: str, event: str) -> list[dict[str, str]]:
    with (HERE / f"{name}.csv").open(newline="", encoding="utf-8") as handle:
        return [row for row in csv.DictReader(handle) if row["event"] == event]


def v(rows: list[dict[str, str]], key: str) -> np.ndarray:
    return np.array([float(row[key]) for row in rows])


def body(ax: plt.Axes, x: float, y: float, heading_deg: float, color: str) -> None:
    theta = math.radians(heading_deg)
    c, s = math.cos(theta), math.sin(theta)
    points = [(x + u*c - w*s, y + u*s + w*c)
              for u, w in ((100, -75), (100, 75), (-150, 75), (-150, -75))]
    ax.add_patch(Polygon(points, fill=True, facecolor=color, edgecolor=color,
                         alpha=0.16, linestyle="--", linewidth=1.4))
    ax.plot(x, y, "o", color=color, markersize=4)


def main() -> None:
    leader = read("leader", "control")
    followers = {name: read(name.lower(), "control_moving") for name in ("F1", "F2")}
    route = TurnPath(TurnPathReference(200, 800, 400, 0, 0, 0))
    fig, axes = plt.subplots(2, 2, figsize=(13, 11), constrained_layout=True)
    ax = axes[0, 0]
    for name, lateral in (("Leader", 0), ("F1", 400), ("F2", -400)):
        ss = np.linspace(-400 if name != "Leader" else 0, route.end_s_mm, 400)
        xy = np.array([route.pose_at_s(float(s), lateral)[:2] for s in ss])
        ax.plot(xy[:, 0], xy[:, 1], "--", color=COLORS[name], alpha=0.55,
                label=f"{name} planned")
    ax.plot(v(leader, "status_x_mm"), v(leader, "status_y_mm"),
            color=COLORS["Leader"], label="Leader odometry")
    for name, rows in followers.items():
        offset_y = 400 if name == "F1" else -400
        ax.plot(v(rows, "follower_x_mm") - 400,
                v(rows, "follower_y_mm") + offset_y,
                color=COLORS[name], label=f"{name} odometry")
    end = leader[-1]
    body(ax, float(end["status_x_mm"]), float(end["status_y_mm"]),
         float(end["status_heading_deg"]), COLORS["Leader"])
    for name, rows in followers.items():
        end = rows[-1]
        body(ax, float(end["follower_x_mm"]) - 400,
             float(end["follower_y_mm"]) + (400 if name == "F1" else -400),
             float(end["follower_heading_deg"]), COLORS[name])
    ax.set(title="Onboard odometry and planned lanes; video shows closer spacing",
           xlabel="forward x (mm)", ylabel="left y (mm)")
    ax.axis("equal")
    ax.grid(alpha=.2)
    ax.legend(fontsize=8, ncol=2)

    ax = axes[0, 1]
    t = v(leader, "elapsed_s")
    left_mm = v(leader, "encoder_left_distance_mm")
    right_mm = v(leader, "encoder_right_distance_mm")
    wheel_yaw = np.degrees((right_mm - left_mm) / 130.0)
    ax.plot(t, v(leader, "status_heading_deg"), color="#7c3aed", label="MPU yaw")
    ax.plot(t, wheel_yaw, color="#059669", label="wheel-difference yaw")
    ax.set(title="Leader heading sensors separate after ~20 s",
           xlabel="seconds after Leader start", ylabel="relative angle (degrees)")
    ax.grid(alpha=.2)
    ax.legend()

    ax = axes[1, 0]
    for name, rows in followers.items():
        ax.plot(v(rows, "leader_path_progress_s_mm"), v(rows, "control_rho_mm"),
                color=COLORS[name], label=f"{name} target error")
    ax.axhline(800, color="#991b1b", linestyle=":", label="current target-error stop")
    ax.set(title="Follower error in the onboard coordinate frames",
           xlabel="Leader projected progress (mm)", ylabel="target distance (mm)")
    ax.grid(alpha=.2)
    ax.legend()

    ax = axes[1, 1]
    for name, rows in followers.items():
        ax.plot(v(rows, "leader_path_progress_s_mm"), v(rows, "control_heading_error_deg"),
                color=COLORS[name], label=f"{name} heading error")
    ax.axhline(65, color="#991b1b", linestyle=":", label="reacquire threshold (slows only)")
    ax.set(title="F2 turns the wrong way while its heading error grows",
           xlabel="Leader projected progress (mm)", ylabel="target minus measured heading (degrees)")
    ax.grid(alpha=.2)
    ax.legend()

    fig.suptitle("2026-11-12 left-turn run: internal estimates, not floor truth", fontsize=15)
    fig.savefig(HERE / "internal_diagnostics.png", dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    main()
