"""Plot an ideal three-rover straight, 90-degree left turn, and exit route."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon


HERE = Path(__file__).resolve().parent
STRAIGHT_MM = 1500.0
RADIUS_MM = 1200.0
EXIT_MM = 1000.0
LAG_MM = 400.0
LANE_MM = 600.0
ARC_MM = RADIUS_MM * np.pi / 2
END_MM = STRAIGHT_MM + ARC_MM + EXIT_MM
COLORS = {"Leader": "#374151", "F1": "#2563eb", "F2": "#ea580c"}
OFFSETS = {"Leader": 0.0, "F1": LANE_MM, "F2": -LANE_MM}


def pose(s_mm: float, lateral_mm: float = 0.0) -> tuple[float, float, float]:
    """MPU position and tangent heading; negative s extends the starting line."""
    if s_mm <= STRAIGHT_MM:
        x, y, heading = s_mm, 0.0, 0.0
    elif s_mm <= STRAIGHT_MM + ARC_MM:
        angle = (s_mm - STRAIGHT_MM) / RADIUS_MM
        x = STRAIGHT_MM + RADIUS_MM * np.sin(angle)
        y = RADIUS_MM * (1 - np.cos(angle))
        heading = angle
    else:
        x = STRAIGHT_MM + RADIUS_MM
        y = RADIUS_MM + s_mm - STRAIGHT_MM - ARC_MM
        heading = np.pi / 2
    return x - lateral_mm * np.sin(heading), y + lateral_mm * np.cos(heading), heading


def corners(x: float, y: float, heading: float) -> np.ndarray:
    # MPU is 100 mm behind the nose, 150 mm ahead of the tail.
    local = np.array([[100, -75], [100, 75], [-150, 75], [-150, -75]])
    rotation = np.array([[np.cos(heading), -np.sin(heading)],
                         [np.sin(heading), np.cos(heading)]])
    return local @ rotation.T + (x, y)


def at_leader_progress(name: str, s_leader: float) -> tuple[float, float, float]:
    s = s_leader if name == "Leader" else s_leader - LAG_MM
    return pose(s, OFFSETS[name])


def main() -> None:
    steps = np.linspace(0, END_MM, 1200)
    samples = {name: np.array([at_leader_progress(name, s) for s in steps])
               for name in COLORS}
    fig, ax = plt.subplots(figsize=(10.5, 9))

    for name, color in COLORS.items():
        path = samples[name]
        ax.plot(path[:, 0], path[:, 1], color=color, lw=2.2, label=f"{name} MPU path")

    # At each snapshot, all three footprints are drawn at the same Leader progress.
    snapshots = [0, STRAIGHT_MM, STRAIGHT_MM + ARC_MM / 2, END_MM]
    for index, s in enumerate(snapshots):
        for name, color in COLORS.items():
            x, y, heading = at_leader_progress(name, s)
            ax.add_patch(Polygon(corners(x, y, heading), closed=True,
                                 facecolor=color, edgecolor=color, alpha=0.10,
                                 linestyle="--", linewidth=1.2))
            ax.scatter(x, y, s=45 if index == 3 else 23, facecolor="white",
                       edgecolor=color, linewidth=1.6, zorder=4)
            ax.scatter(x, y, s=5, color=color, zorder=5)
            if index in (0, 3):
                ax.annotate(f"{name} {'start' if index == 0 else 'finish'}",
                            (x, y), xytext=(6, 5), textcoords="offset points",
                            fontsize=8, color=color)

    for x, y, label in [(STRAIGHT_MM, 0, "turn starts"),
                        (STRAIGHT_MM + RADIUS_MM, RADIUS_MM, "turn ends")]:
        ax.scatter(x, y, marker="x", color=COLORS["Leader"], zorder=6)
        ax.annotate(label, (x, y), xytext=(8, -17), textcoords="offset points", fontsize=9)

    all_corners = np.concatenate([corners(*row) for path in samples.values() for row in path])
    xmin, ymin = all_corners.min(axis=0) - 300
    xmax, ymax = all_corners.max(axis=0) + 300
    ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel="x from start (mm)",
           ylabel="y to the left (mm)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.20)
    ax.legend(loc="upper left", frameon=False)
    ax.set_title("1.5 m straight  →  90° left (R = 1.2 m)  →  1.0 m straight")
    ax.text(0.02, 0.76,
            "F1/F2: 400 mm path lag, ±600 mm rotating lane offset\n"
            "Dashed bodies: 250 × 150 mm, anchored at MPU\n"
            "Ideal geometry only; real clearance must be checked",
            transform=ax.transAxes, fontsize=9, va="top",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.88})
    fig.tight_layout()
    output = HERE / "route_candidate.png"
    fig.savefig(output, dpi=180)
    plt.close(fig)

    print(f"Leader distance: {END_MM:.0f} mm")
    print(f"Leader at end: {pose(END_MM)[:2]}")
    print(f"Nominal plot area with 300 mm margin: {xmax-xmin:.0f} x {ymax-ymin:.0f} mm")
    for a, b in [("Leader", "F1"), ("Leader", "F2"), ("F1", "F2")]:
        distance = np.linalg.norm(samples[a][:, :2] - samples[b][:, :2], axis=1).min()
        print(f"Minimum ideal MPU spacing {a}/{b}: {distance:.0f} mm")
    print(output)


if __name__ == "__main__":
    main()
