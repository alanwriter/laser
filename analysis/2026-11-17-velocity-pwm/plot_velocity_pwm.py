#!/usr/bin/env python3
"""Compare Leader velocity commands, measured wheel rates, and applied PWM.

This is a diagnostic of the existing feedback controller.  In particular,
closed-loop command-to-PWM samples are not an open-loop motor calibration.
"""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
RUNS = {
    "1 m precise": ROOT / "analysis/2026-11-16-straight-1m-precise/leader.csv",
    "2 m repeat": ROOT / "analysis/2026-11-16-straight-2m-repeat/leader.csv",
    "1.5 sine": ROOT / "analysis/2026-11-17-sine-right-run/leader.csv",
}
RUNNING_SWEEP = ROOT / "analysis/2026-11-16-leader-pwm-sweep/running.csv"
TICKS_PER_REV = {"left": 1216.4, "right": 1222.4}
WHEEL_DIAMETER_MM = 65.0  # Current Nano CONFIG, not calibrated ground diameter.
MM_PER_TICK = {
    side: math.pi * WHEEL_DIAMETER_MM / ticks
    for side, ticks in TICKS_PER_REV.items()
}


def usable_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    latest = max(float(r["elapsed_s"]) for r in rows)
    return [
        r for r in rows
        if r["event"] == "control"
        and r["status_mode"] == "velocity"
        and r["status_fault_code"] == "0"
        and 3.0 <= float(r["elapsed_s"]) <= latest - 2.0
    ]


def wheel_samples(rows: list[dict[str, str]], side: str) -> list[tuple[float, float, float]]:
    scale = MM_PER_TICK[side]
    result = []
    for r in rows:
        target_tps = float(r[f"status_{side}_target_tps"])
        measured_tps = float(r[f"status_{side}_tps"])
        pwm = float(r[f"status_{side}_pwm"])
        if target_tps >= 20.0 and pwm > 0:
            result.append((target_tps * scale, measured_tps * scale, pwm))
    return result


def binned(samples: list[tuple[float, float, float]], width: float = 3.0):
    groups: dict[int, list[tuple[float, float, float]]] = defaultdict(list)
    for row in samples:
        groups[int(row[0] // width)].append(row)
    return [
        (median(v[0] for v in values), median(v[1] for v in values),
         median(v[2] for v in values), len(values))
        for _, values in sorted(groups.items()) if len(values) >= 5
    ]


def transient_inverse(side: str) -> list[tuple[float, float]]:
    with RUNNING_SWEEP.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    values = []
    for pwm in (45, 50, 55, 60):
        segment = [
            float(r[f"{side}_tps"])
            for r in rows if r["event"] == "moving"
            and int(r["pwm"]) == pwm and float(r["elapsed_s"]) >= 0.20
        ]
        if segment:
            values.append((median(segment) * MM_PER_TICK[side], float(pwm)))
    return values


def main() -> None:
    colors = {"1 m precise": "#3579b6", "2 m repeat": "#e88d29", "1.5 sine": "#3a9a69"}
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.5), sharex="col")
    summary: dict[str, object] = {
        "unit_note": "Wheel mm/s uses current 65 mm Nano diameter, not ground truth.",
        "runs": {}, "transient_inverse": {},
    }
    for col, side in enumerate(("left", "right")):
        ax_speed, ax_pwm = axes[0, col], axes[1, col]
        for name, path in RUNS.items():
            samples = wheel_samples(usable_rows(path), side)
            bins = binned(samples)
            if not samples:
                continue
            color = colors[name]
            ax_speed.scatter([v[0] for v in samples], [v[1] for v in samples],
                             s=7, alpha=0.08, color=color)
            ax_pwm.scatter([v[0] for v in samples], [v[2] for v in samples],
                           s=7, alpha=0.08, color=color)
            if bins:
                ax_speed.plot([v[0] for v in bins], [v[1] for v in bins],
                              marker="o", ms=3.5, lw=1.5, label=name, color=color)
                ax_pwm.plot([v[0] for v in bins], [v[2] for v in bins],
                            marker="o", ms=3.5, lw=1.5, label=name, color=color)
            summary["runs"].setdefault(name, {})[side] = {
                "n_time_correlated_samples": len(samples),
                "median_command_mm_s": round(median(v[0] for v in samples), 2),
                "median_measured_mm_s": round(median(v[1] for v in samples), 2),
                "median_pwm": round(median(v[2] for v in samples), 1),
            }
        inverse = transient_inverse(side)
        summary["transient_inverse"][side] = [
            {"measured_nominal_mm_s": round(speed, 2), "pwm": int(pwm)}
            for speed, pwm in inverse
        ]
        ax_pwm.scatter([v[0] for v in inverse], [v[1] for v in inverse],
                       color="#242424", marker="x", s=60, linewidth=1.5,
                       label="open-loop moving, transient")
        ax_speed.plot([0, 125], [0, 125], ls="--", lw=1, color="#666666", label="ideal")
        ax_speed.set_title(f"Leader {side} wheel")
        ax_speed.set_ylabel("Measured wheel speed (Nano nominal mm/s)")
        ax_pwm.set_ylabel("Applied PWM")
        ax_pwm.set_xlabel("Requested wheel speed (Nano nominal mm/s)")
        ax_speed.set_xlim(0, 125)
        ax_speed.set_ylim(0, 125)
        ax_pwm.set_ylim(0, 85)
        for ax in (ax_speed, ax_pwm):
            ax.grid(alpha=0.25)
        ax_speed.legend(fontsize=8)
        ax_pwm.legend(fontsize=8)
    fig.suptitle("Velocity request to applied PWM: existing Leader controller", fontsize=14)
    fig.text(0.5, 0.005,
             "3 s after start to 2 s before stop; bins are medians of time-correlated samples. "
             "Open-loop crosses are short moving transients, not a certified feedforward map.",
             ha="center", fontsize=8)
    fig.tight_layout(rect=(0, 0.03, 1, 0.96))
    fig.savefig(HERE / "velocity_to_pwm_diagnostic.png", dpi=180)
    (HERE / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
