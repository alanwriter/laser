#!/usr/bin/env python3
"""Plot the existing Leader open-loop PWM observations without fitting a motor model."""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "2026-11-16-leader-pwm-sweep"


def read_rows(name: str) -> dict[int, list[dict[str, str]]]:
    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    with (SOURCE / name).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            grouped[int(row["pwm"])].append(row)
    return grouped


def cold_counts(rows: list[dict[str, str]], wheel: str) -> int:
    return int(rows[-1][f"{wheel}_count"]) - int(rows[0][f"{wheel}_count"])


def running_median(rows: list[dict[str, str]], wheel: str) -> float | None:
    samples = [
        float(row[f"{wheel}_tps"])
        for row in rows
        if row["event"] == "moving" and float(row["elapsed_s"]) >= 0.20
    ]
    return median(samples) if len(samples) >= 3 else None


def main() -> None:
    cold = read_rows("leader.csv")
    running = read_rows("running.csv")
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
    colors = {"left": "#1769aa", "right": "#e3712b"}

    for wheel in ("left", "right"):
        pwms = sorted(cold)
        axes[0].plot(
            pwms,
            [cold_counts(cold[pwm], wheel) for pwm in pwms],
            "o-",
            color=colors[wheel],
            label=wheel.capitalize(),
        )
    axes[0].set(title="Leader: 0.8 s from rest", xlabel="PWM command", ylabel="Encoder count change")
    axes[0].set_xticks(sorted(cold))
    axes[0].legend()

    for wheel in ("left", "right"):
        points = [(pwm, running_median(rows, wheel)) for pwm, rows in sorted(running.items())]
        usable = [(pwm, tps) for pwm, tps in points if tps is not None]
        axes[1].plot(
            [pwm for pwm, _ in usable],
            [tps for _, tps in usable],
            "o--",
            color=colors[wheel],
            label=wheel.capitalize(),
        )
    axes[1].axvspan(39.5, 40.5, color="#b83635", alpha=0.12)
    axes[1].annotate("PWM 40: stopped by yaw guard", (41.5, 180), fontsize=8, color="#9f2424")
    axes[1].set(
        title="Leader: descending PWM after start",
        xlabel="PWM command",
        ylabel="Median encoder speed (ticks/s)",
        xlim=(38, 83),
    )
    axes[1].set_xticks(sorted(running))
    axes[1].legend()

    for axis in axes:
        axis.grid(alpha=0.22)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Exploratory grounded test — transient data, not a steady-state motor map", fontsize=12)
    fig.savefig(HERE / "leader_exploratory.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
