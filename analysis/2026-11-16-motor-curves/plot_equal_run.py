#!/usr/bin/env python3
"""Plot the stopped Leader fixed-equal-VELOCITY trial from its raw CSV."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent


def main() -> None:
    with (HERE / "leader_equal_400mm_aborted.csv").open(newline="", encoding="utf-8-sig") as stream:
        rows = [row for row in csv.DictReader(stream) if row["event"] in ("origin", "control", "aborted")]
    t = [float(r["elapsed_s"]) for r in rows]
    left = [int(r["status_left_count"]) for r in rows]
    right = [int(r["status_right_count"]) for r in rows]
    heading = [float(r["status_heading_deg"]) for r in rows]
    x = [float(r["status_x_mm"]) for r in rows]
    y = [float(r["status_y_mm"]) for r in rows]

    fig, axes = plt.subplots(1, 3, figsize=(13.5, 3.8), constrained_layout=True)
    axes[0].plot(t, left, color="#1769aa", label="Left")
    axes[0].plot(t, right, color="#e3712b", label="Right")
    axes[0].set(title="Encoder progress", xlabel="Time (s)", ylabel="Cumulative count")
    axes[0].legend()

    axes[1].plot(t, heading, color="#614197")
    axes[1].axhline(-20, color="#b83635", linestyle="--", linewidth=1,
                    label="Stop guard: -20 deg")
    axes[1].set(title="MPU heading estimate", xlabel="Time (s)", ylabel="Heading (deg)")
    axes[1].legend(fontsize=8)

    axes[2].plot(x, y, color="#316a57")
    axes[2].scatter([x[-1]], [y[-1]], color="#b83635", zorder=3, label="Stopped")
    axes[2].axhline(0, color="#666666", linestyle=":", linewidth=1)
    axes[2].set(title="Onboard odometry only", xlabel="Forward x (mm)",
                ylabel="Lateral y (mm)")
    axes[2].legend(fontsize=8)
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Leader: equal 18 mm/s wheel commands, stopped at heading guard")
    fig.savefig(HERE / "leader_equal_400mm_aborted.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
