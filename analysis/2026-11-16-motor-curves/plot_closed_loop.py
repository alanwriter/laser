#!/usr/bin/env python3
"""Plot historical commanded versus measured encoder TPS for each rover."""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCES = (
    ("Leader", "status_", ROOT / "analysis/2026-11-16-straight-1m-precise/leader.csv"),
    ("Leader", "status_", ROOT / "analysis/2026-11-16-straight-2m/leader.csv"),
    ("Leader", "status_", ROOT / "analysis/2026-10-29-f1-f2-theory/real_run_074722Z/leader.csv"),
    ("F1", "follower_", ROOT / "analysis/2026-10-29-f1-f2-theory/real_run_074722Z/f1.csv"),
    ("F2", "follower_", ROOT / "analysis/2026-10-29-f1-f2-theory/real_run_074722Z/f2.csv"),
)
COLORS = {"left": "#1769aa", "right": "#e3712b"}


def observations() -> dict[tuple[str, str], list[tuple[float, float]]]:
    found: dict[tuple[str, str], list[tuple[float, float]]] = defaultdict(list)
    for vehicle, prefix, path in SOURCES:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                # Follower waiting rows can repeat stale telemetry after motion.
                if not row["event"].startswith("control") or row[prefix + "mode"] != "velocity":
                    continue
                if int(row[prefix + "fault_code"]) != 0:
                    continue
                for wheel in ("left", "right"):
                    target = float(row[prefix + wheel + "_target_tps"])
                    measured = float(row[prefix + wheel + "_tps"])
                    pwm = float(row[prefix + wheel + "_pwm"])
                    if target > 20 and measured >= 0 and pwm > 0:
                        found[(vehicle, wheel)].append((target, measured))
    return found


def binned_medians(points: list[tuple[float, float]]) -> list[tuple[float, float, int]]:
    groups: dict[int, list[tuple[float, float]]] = defaultdict(list)
    for target, measured in points:
        groups[int(target // 50)].append((target, measured))
    return [
        (median(p[0] for p in rows), median(p[1] for p in rows), len(rows))
        for _, rows in sorted(groups.items()) if len(rows) >= 6
    ]


def main() -> None:
    data = observations()
    fig, axes = plt.subplots(1, 3, figsize=(14.4, 4.7), sharex=True, sharey=True,
                             constrained_layout=True)
    for axis, vehicle in zip(axes, ("Leader", "F1", "F2")):
        axis.plot([0, 500], [0, 500], color="#555555", linestyle=":", linewidth=1,
                  label="Ideal: measured = target")
        for wheel in ("left", "right"):
            points = data[(vehicle, wheel)]
            axis.scatter([p[0] for p in points], [p[1] for p in points],
                         color=COLORS[wheel], alpha=0.11, s=13, linewidths=0)
            medians = binned_medians(points)
            axis.plot([p[0] for p in medians], [p[1] for p in medians],
                      marker="o", markersize=5, linewidth=2,
                      color=COLORS[wheel], label=f"{wheel.capitalize()} wheel")
        axis.set(title=f"{vehicle} (feedback controlled)", xlim=(0, 500), ylim=(0, 1000),
                 xlabel="Commanded encoder speed (ticks/s)")
        axis.grid(alpha=0.2)
        axis.spines[["top", "right"]].set_visible(False)
        axis.legend(fontsize=8, loc="upper left")
    axes[0].set_ylabel("Measured encoder speed (ticks/s)")
    fig.suptitle("Historical closed-loop response — 50 TPS bin medians; faint dots are samples",
                 fontsize=12)
    fig.savefig(HERE / "three_rover_closed_loop.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
