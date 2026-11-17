"""Draw the planned geometry for the next F1-old / F2-new discussion."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pi_rover"))
from wave_path import WavePath, WavePathReference  # noqa: E402


def f1_target(x: float, y: float, heading_deg: float) -> tuple[float, float]:
    heading = math.radians(heading_deg)
    return (
        x - 400 * math.cos(heading) - 400 * math.sin(heading),
        y - 400 * math.sin(heading) + 400 * math.cos(heading),
    )


def main() -> None:
    route = WavePath(WavePathReference(1500, 0, 50, 3, 0, 0, 0))
    s_end = route.end_s_mm
    s_values = [s_end * i / 700 for i in range(701)]
    leader = [route.pose_at_s(s) for s in s_values]
    f1 = [f1_target(x, y, heading) for x, y, heading, _ in leader]
    f2 = [route.pose_at_s(s - 400, -400) for s in s_values]

    fig, (start, path) = plt.subplots(
        1, 2, figsize=(14, 5.6), gridspec_kw={"width_ratios": [1, 2.2]}
    )
    colors = {"leader": "#20252b", "f1": "#2773aa", "f2": "#ca6f24"}

    for label, xy, color in (
        ("Leader (0, 0)", (0, 0), colors["leader"]),
        ("F1 (-400, +400)", (-400, 400), colors["f1"]),
        ("F2 (-400, -400)", (-400, -400), colors["f2"]),
    ):
        start.scatter(*xy, s=115, color=color, zorder=3)
        start.annotate(label, xy, xytext=(7, 8), textcoords="offset points", fontsize=10)
        start.arrow(xy[0], xy[1], 125, 0, width=7, head_width=40,
                    head_length=45, color=color, length_includes_head=True)
    start.axhline(0, color="#d6d9dc", lw=0.8)
    start.axvline(0, color="#d6d9dc", lw=0.8)
    start.set(xlim=(-640, 290), ylim=(-560, 560),
              xlabel="x (mm), forward →", ylabel="y (mm), left ↑",
              title="A. Final start pose before RESET")
    start.set_aspect("equal", adjustable="box")
    start.grid(alpha=0.15)

    path.plot([v[0] for v in leader], [v[1] for v in leader],
              color=colors["leader"], lw=2.5)
    path.plot([v[0] for v in f1], [v[1] for v in f1],
              color=colors["f1"], lw=2.2)
    path.plot([v[0] for v in f2], [v[1] for v in f2],
              color=colors["f2"], lw=2.2)

    s = s_end * 0.5
    l = route.pose_at_s(s)
    a = f1_target(*l[:3])
    b = route.pose_at_s(s - 400, -400)
    for xy, color, text in ((l[:2], colors["leader"], "Leader"),
                            (a, colors["f1"], "F1 target"),
                            (b[:2], colors["f2"], "F2 target")):
        path.scatter(*xy, s=85, color=color, edgecolor="white", linewidth=0.8, zorder=4)
        path.annotate(text, xy, xytext=(6, 8), textcoords="offset points", fontsize=9)
    path.plot([l[0], a[0]], [l[1], a[1]], color=colors["f1"],
              lw=1, ls=":", alpha=0.8)
    path.plot([l[0], b[0]], [l[1], b[1]], color=colors["f2"],
              lw=1, ls=":", alpha=0.8)
    path.set(xlim=(-520, 1600), ylim=(-550, 680),
             xlabel="x (mm), forward →", ylabel="y (mm), left ↑",
             title="B. Planned routes and moving targets")
    path.grid(alpha=0.15)
    path.annotate("Leader route", leader[-1][:2], xytext=(-95, 10),
                  textcoords="offset points", color=colors["leader"], fontsize=9)
    path.annotate("F1 body-offset target trace", f1[-1], xytext=(8, -3),
                  textcoords="offset points", color=colors["f1"], fontsize=9)
    path.annotate("F2 path target trace", f2[-1][:2], xytext=(8, -3),
                  textcoords="offset points", color=colors["f2"], fontsize=9)
    path.set_aspect("equal", adjustable="box")

    fig.tight_layout()
    out = Path(__file__).with_name("setup_and_targets.png")
    fig.savefig(out, dpi=180)
    print(out)


if __name__ == "__main__":
    main()
