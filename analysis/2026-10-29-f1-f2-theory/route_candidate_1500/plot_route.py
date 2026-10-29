"""Compare the current and proposed 1.5 m leader routes, including F2's 400 mm delay."""

from __future__ import annotations

from pathlib import Path
import sys

import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "pi_rover"))
sys.path.insert(0, str(HERE.parent))

from vehicle_footprint import draw_body  # noqa: E402
from wave_path import WavePath, WavePathReference  # noqa: E402


def main() -> None:
    cases = (
        ("Current: 3 tight cycles", 0, 50, 3, "#8a929c"),
        ("Candidate: 250 mm straight + 1 broad S", 250, 50, 1, "#2478b3"),
    )
    fig, axes = plt.subplots(
        2, 2, figsize=(14, 7.5),
        gridspec_kw={"height_ratios": [0.36, 1.0]},
    )
    for column, (title, lead_in, amplitude, cycles, color) in enumerate(cases):
        route = WavePath(WavePathReference(1500, lead_in, amplitude, cycles, 0, 0, 0))
        top = axes[0, column]
        xs = [1500 * i / 600 for i in range(601)]
        top.plot(xs, [route.shape(x)[0] for x in xs], color=color, lw=2.5)
        for x in (0, 700, 1500):
            px, py, heading, _ = route.pose_at_s(route.s_at_x(x))
            draw_body(top, px, py, heading, color, prominent=x == 1500)
        top.axvline(lead_in, color="#8d99a5", lw=1, ls=":", alpha=0.7)
        top.set(xlim=(-180, 1660), ylim=(-180, 180), title=title,
                xlabel="forward x (mm)", ylabel="lateral y (mm)")
        top.set_aspect("equal")
        top.grid(alpha=0.18)
        top.text(0.02, 0.95,
                 f"1,500 mm forward span\n{route.end_s_mm:,.0f} mm arc length",
                 transform=top.transAxes, va="top", fontsize=9,
                 bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "none"})

        bottom = axes[1, column]
        arc = [route.end_s_mm * i / 600 for i in range(601)]
        bottom.plot(arc, [route.pose_at_s(s)[2] for s in arc],
                    color="#252a30", lw=2, label="Leader path heading")
        bottom.plot(arc, [route.pose_at_s(s - 400)[2] for s in arc],
                    color="#c66b21", lw=2, label="F2 target heading (400 mm behind)")
        follower_turn = lead_in + 400
        bottom.axvline(follower_turn, color="#ad4565", ls=":", lw=1.3)
        bottom.text(follower_turn + 20, 38,
                    f"F2 begins bend\nat Leader s ≈ {follower_turn} mm",
                    color="#a33455", fontsize=9, va="top")
        bottom.set(xlim=(0, 1700), ylim=(-50, 50),
                   xlabel="Leader traveled path s (mm)",
                   ylabel="target heading (deg)")
        bottom.grid(alpha=0.18)
        if column == 1:
            bottom.legend(loc="lower right", fontsize=9, frameon=False)

    fig.suptitle("F2 follows the same path 400 mm after Leader", fontsize=15)
    fig.text(0.5, 0.018,
             "Rectangles show 250 × 150 mm body at the MPU anchor. Ideal path geometry only; no slip, odometry error, or camera ground truth.",
             ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 0.95), h_pad=1.6, w_pad=2.0)
    output = HERE / "route_comparison.png"
    fig.savefig(output, dpi=180)
    print(output)


if __name__ == "__main__":
    main()
