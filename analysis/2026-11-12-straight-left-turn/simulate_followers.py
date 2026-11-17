"""Offline trial: ideal Leader and the existing path-tracker control law.

Only the route geometry is adapted to a rotating, normal-offset left-turn lane.
No USB, UDP, or live rover process is used.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import asdict, replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "pi_rover"))
sys.path.insert(0, str(ROOT / "analysis" / "2026-10-29-f1-f2-theory"))

from follower_formation import (  # noqa: E402
    FormationPlan, LeaderFrame, PathFormationTracker,
)
from simulate_formation import status, step_rover  # noqa: E402
from vehicle_footprint import draw_body  # noqa: E402
from turn_path import TurnPathReference  # noqa: E402
from plot_candidate import (  # noqa: E402
    ARC_MM, COLORS, END_MM, EXIT_MM, LAG_MM, LANE_MM, RADIUS_MM,
    STRAIGHT_MM, pose,
)

DT = 0.1
LEADER_SPEED_MM_S = 30.0
SETTLE_S = 20.0
END_TIME_S = END_MM / LEADER_SPEED_MM_S


def simulate() -> tuple[list[dict[str, object]], dict[str, object]]:
    key = TurnPathReference(STRAIGHT_MM, RADIUS_MM, EXIT_MM, 0, 0, 0)
    followers = {"F1": status(), "F2": status()}
    trackers = {}
    stopped: dict[str, str] = {}
    for name, lateral in (("F1", LANE_MM), ("F2", -LANE_MM)):
        plan = FormationPlan(
            offset_x_mm=-LAG_MM, offset_y_mm=lateral,
            follower_start_x_mm=-LAG_MM, follower_start_y_mm=lateral,
        )
        tracker = PathFormationTracker(plan)
        trackers[name] = tracker

    rows: list[dict[str, object]] = []
    total_ticks = math.ceil((END_TIME_S + SETTLE_S) / DT)
    for tick in range(total_ticks + 1):
        t = tick * DT
        leader_s = min(LEADER_SPEED_MM_S * t, END_MM)
        lx, ly, lh = pose(leader_s)
        leader_speed = LEADER_SPEED_MM_S if t < END_TIME_S else 0.0
        leader_yaw = (LEADER_SPEED_MM_S / RADIUS_MM
                      if STRAIGHT_MM < leader_s < STRAIGHT_MM + ARC_MM else 0.0)
        leader = LeaderFrame(
            status=status(float(lx), float(ly), math.degrees(lh)),
            received_monotonic_s=t,
            source_host="offline-ideal-leader",
            forward_mm_per_second=leader_speed,
            yaw_rate_radians_per_second=leader_yaw,
            has_motion_reference=True,
            formation_phase="moving" if leader_speed else "settle",
            path=replace(key, progress_s_mm=leader_s),
        )
        row: dict[str, object] = {
            "time_s": round(t, 2), "leader_s_mm": round(leader_s, 3),
            "leader_x_mm": round(float(lx), 3), "leader_y_mm": round(float(ly), 3),
            "leader_heading_deg": round(math.degrees(lh), 3),
        }
        controls = {}
        for name, lateral in (("F1", LANE_MM), ("F2", -LANE_MM)):
            follower = followers[name]
            x = follower.x_mm - LAG_MM
            y = follower.y_mm + lateral
            row.update({f"{name}_x_mm": round(x, 3),
                        f"{name}_y_mm": round(y, 3),
                        f"{name}_heading_deg": round(follower.heading_deg, 3)})
            target_x, target_y, _ = pose(leader_s - LAG_MM, lateral)
            row.update({f"{name}_target_x_mm": round(float(target_x), 3),
                        f"{name}_target_y_mm": round(float(target_y), 3),
                        f"{name}_target_error_mm": round(math.hypot(x - target_x, y - target_y), 3)})
            if name not in stopped:
                control = trackers[name].control(leader, follower)
                controls[name] = control
                if not control.tracking_safe:
                    stopped[name] = f"t={t:.1f}s: {control.stop_reason}"
            else:
                control = None
            row.update({f"{name}_left_mm_s": control.left_mm_per_second if control else 0,
                        f"{name}_right_mm_s": control.right_mm_per_second if control else 0,
                        f"{name}_stop_reason": stopped.get(name, ""),
                        f"{name}_progress_error_mm": round(control.longitudinal_error_mm, 3) if control else None,
                        f"{name}_cross_track_error_mm": round(control.lateral_error_mm, 3) if control else None})
        rows.append(row)
        for name, control in controls.items():
            if name not in stopped:
                followers[name] = step_rover(
                    followers[name], control.left_mm_per_second,
                    control.right_mm_per_second, DT, trackers[name].plan.wheel_track_mm,
                )

    moving = [r for r in rows if r["time_s"] <= END_TIME_S]
    summary: dict[str, object] = {
        "model": "Ideal Leader at 30 mm/s; live TurnPath and PathFormationTracker control law; 0.1 s ideal differential-drive integration",
        "route": {"straight_mm": STRAIGHT_MM, "turn_deg": 90,
                  "radius_mm": RADIUS_MM, "exit_mm": EXIT_MM,
                  "leader_distance_mm": round(END_MM, 3)},
        "duration_moving_s": round(END_TIME_S, 2),
        "settle_s": SETTLE_S,
        "controller_safety_stops": stopped,
    }
    for name in ("F1", "F2"):
        summary[name] = {
            "moving_median_target_error_mm": round(float(np.median([r[f"{name}_target_error_mm"] for r in moving])), 1),
            "moving_max_target_error_mm": round(max(r[f"{name}_target_error_mm"] for r in moving), 1),
            "final_target_error_mm": round(float(rows[-1][f"{name}_target_error_mm"]), 1),
            "final_mpu_xy_mm": [rows[-1][f"{name}_x_mm"], rows[-1][f"{name}_y_mm"]],
        }
    return rows, summary


def plot(rows: list[dict[str, object]]) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 12),
                             gridspec_kw={"height_ratios": [1.3, 0.55]},
                             constrained_layout=True)
    ax = axes[0]
    ax.plot([r["leader_x_mm"] for r in rows], [r["leader_y_mm"] for r in rows],
            color=COLORS["Leader"], lw=2.2, label="Leader ideal")
    for name in ("F1", "F2"):
        color = COLORS[name]
        ax.plot([r[f"{name}_target_x_mm"] for r in rows],
                [r[f"{name}_target_y_mm"] for r in rows],
                color=color, ls="--", lw=1.5, alpha=0.7, label=f"{name} target")
        ax.plot([r[f"{name}_x_mm"] for r in rows],
                [r[f"{name}_y_mm"] for r in rows],
                color=color, lw=2.3, label=f"{name} simulated")
    for t in (0, 50, 90, END_TIME_S):
        row = min(rows, key=lambda r: abs(r["time_s"] - t))
        for name in ("F1", "F2"):
            draw_body(ax, row[f"{name}_x_mm"], row[f"{name}_y_mm"],
                      row[f"{name}_heading_deg"], COLORS[name],
                      prominent=t == END_TIME_S)
        draw_body(ax, row["leader_x_mm"], row["leader_y_mm"],
                  row["leader_heading_deg"], COLORS["Leader"],
                  prominent=t == END_TIME_S)
    ax.set(xlabel="x (mm)", ylabel="y (mm)",
           title="Ideal Leader; F1/F2 controlled ideal differential-drive motion")
    ax.axis("equal")
    ax.grid(alpha=0.2)
    ax.legend(loc="upper left", ncol=2, frameon=False)
    errors = axes[1]
    for name in ("F1", "F2"):
        errors.plot([r["time_s"] for r in rows],
                    [r[f"{name}_target_error_mm"] for r in rows],
                    color=COLORS[name], label=f"{name} target distance")
    errors.axvline(END_TIME_S, color=COLORS["Leader"], ls=":", label="Leader stops")
    errors.set(xlabel="time (s)", ylabel="MPU distance to target (mm)",
               title="Position error against each follower's moving target")
    errors.grid(alpha=0.2)
    errors.legend(frameon=False)
    fig.savefig(HERE / "simulated_followers.png", dpi=180)
    plt.close(fig)


def main() -> None:
    rows, summary = simulate()
    with (HERE / "simulated_followers.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    (HERE / "simulation_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    plot(rows)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
