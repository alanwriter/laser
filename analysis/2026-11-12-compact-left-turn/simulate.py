"""Offline three-rover trial for the 1.0 m forward / 1.2 m left Leader endpoint."""

from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "pi_rover"))
sys.path.insert(0, str(ROOT / "analysis" / "2026-10-29-f1-f2-theory"))

from follower_formation import FormationPlan, LeaderFrame, PathFormationTracker  # noqa: E402
from simulate_formation import status, step_rover  # noqa: E402
from turn_path import TurnPath, TurnPathReference  # noqa: E402
from vehicle_footprint import draw_body  # noqa: E402

STRAIGHT_MM = 200.0
RADIUS_MM = 800.0
EXIT_MM = 400.0
LAG_MM = 400.0
LANES = {"F1": 400.0, "F2": -400.0}
SPEED_MM_S = 30.0
DT = 0.1
COLORS = {"Leader": "#374151", "F1": "#2563eb", "F2": "#ea580c"}


def body_corners(x: float, y: float, heading_deg: float) -> list[tuple[float, float]]:
    angle = math.radians(heading_deg)
    c, s = math.cos(angle), math.sin(angle)
    return [(x + u*c - v*s, y + u*s + v*c)
            for u, v in ((100, -75), (100, 75), (-150, 75), (-150, -75))]


def body_gap(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> float:
    """Gap between two convex 250 x 150 mm footprints; zero when overlapping."""
    axes = []
    for polygon in (a, b):
        for i in range(4):
            dx = polygon[(i+1) % 4][0] - polygon[i][0]
            dy = polygon[(i+1) % 4][1] - polygon[i][1]
            length = math.hypot(dx, dy)
            axes.append((-dy/length, dx/length))
    if all(not (max(x*ax+y*ay for x,y in a) < min(x*ax+y*ay for x,y in b)
                or max(x*ax+y*ay for x,y in b) < min(x*ax+y*ay for x,y in a))
           for ax, ay in axes):
        return 0.0

    def point_segment(point, start, end):
        dx, dy = end[0]-start[0], end[1]-start[1]
        fraction = max(0.0, min(1.0,
            ((point[0]-start[0])*dx + (point[1]-start[1])*dy) / (dx*dx+dy*dy)))
        return math.dist(point, (start[0]+fraction*dx, start[1]+fraction*dy))

    return min(point_segment(point, polygon[i], polygon[(i+1) % 4])
               for source, polygon in ((a,b),(b,a)) for point in source for i in range(4))


def simulate() -> tuple[list[dict[str, float | int | str]], dict[str, object]]:
    reference = TurnPathReference(STRAIGHT_MM, RADIUS_MM, EXIT_MM, 0, 0, 0)
    route = TurnPath(reference)
    followers = {name: status() for name in LANES}
    trackers = {name: PathFormationTracker(FormationPlan(
        offset_x_mm=-LAG_MM, offset_y_mm=lateral,
        follower_start_x_mm=-LAG_MM, follower_start_y_mm=lateral,
    )) for name, lateral in LANES.items()}
    rows = []
    stops: dict[str, str] = {}
    minimum_mpu = {"Leader-F1": math.inf, "Leader-F2": math.inf, "F1-F2": math.inf}
    minimum_body = {key: math.inf for key in minimum_mpu}
    bounds = [math.inf, math.inf, -math.inf, -math.inf]
    end_time = reference.end_s_mm / SPEED_MM_S
    for tick in range(math.ceil(end_time / DT) + 1):
        t = tick * DT
        progress = min(SPEED_MM_S * t, reference.end_s_mm)
        lx, ly, lh, _ = route.pose_at_s(progress)
        leader = LeaderFrame(
            status(lx, ly, lh), t, "offline-ideal-leader",
            SPEED_MM_S if progress < reference.end_s_mm else 0.0, 0.0, True,
            "moving" if progress < reference.end_s_mm else "stopped",
            replace(reference, progress_s_mm=progress),
        )
        row: dict[str, float | int | str] = {
            "time_s": round(t, 2), "leader_s_mm": round(progress, 3),
            "leader_x_mm": round(lx, 3), "leader_y_mm": round(ly, 3),
            "leader_heading_deg": round(lh, 3),
        }
        world = {"Leader": (lx, ly, lh)}
        for name, lateral in LANES.items():
            own = followers[name]
            x, y = own.x_mm-LAG_MM, own.y_mm+lateral
            world[name] = (x, y, own.heading_deg)
            tx, ty, _, _ = route.pose_at_s(progress-LAG_MM, lateral)
            error = math.hypot(x-tx, y-ty)
            row.update({f"{name}_x_mm": round(x, 3), f"{name}_y_mm": round(y, 3),
                        f"{name}_heading_deg": round(own.heading_deg, 3),
                        f"{name}_target_x_mm": round(tx, 3),
                        f"{name}_target_y_mm": round(ty, 3),
                        f"{name}_target_error_mm": round(error, 3)})
            if progress < reference.end_s_mm and name not in stops:
                control = trackers[name].control(leader, own)
                if not control.tracking_safe:
                    stops[name] = f"t={t:.1f}s: {control.stop_reason}"
                else:
                    followers[name] = step_rover(own, control.left_mm_per_second,
                                                 control.right_mm_per_second, DT, 130.0)
            row[f"{name}_stop_reason"] = stops.get(name, "")
        for key, a, b in (("Leader-F1", "Leader", "F1"),
                          ("Leader-F2", "Leader", "F2"), ("F1-F2", "F1", "F2")):
            minimum_mpu[key] = min(minimum_mpu[key], math.dist(world[a][:2], world[b][:2]))
            minimum_body[key] = min(minimum_body[key],
                body_gap(body_corners(*world[a]), body_corners(*world[b])))
        for pose in world.values():
            for x, y in body_corners(*pose):
                bounds[0] = min(bounds[0], x); bounds[1] = min(bounds[1], y)
                bounds[2] = max(bounds[2], x); bounds[3] = max(bounds[3], y)
        rows.append(row)
    summary: dict[str, object] = {
        "leader_endpoint_mm": [1000, 1200], "leader_route_length_mm": round(reference.end_s_mm, 1),
        "leader_ideal_duration_s": round(end_time, 1), "turn_radius_mm": RADIUS_MM,
        "straight_mm": STRAIGHT_MM, "exit_mm": EXIT_MM, "lanes_mm": LANES,
        "controller_safety_stops": stops,
        "minimum_simulated_mpu_distance_mm": {k: round(v, 1) for k,v in minimum_mpu.items()},
        "minimum_simulated_body_gap_mm": {k: round(v, 1) for k,v in minimum_body.items()},
        "simulated_body_bounds_mm": [round(value, 1) for value in bounds],
        "nominal_plot_area_with_300_mm_margin_each_side_mm": [
            round(bounds[2]-bounds[0]+600, 1), round(bounds[3]-bounds[1]+600, 1)],
    }
    for name in LANES:
        errors = [r[f"{name}_target_error_mm"] for r in rows]
        summary[name] = {"median_target_error_mm": round(float(np.median(errors)), 1),
                         "max_target_error_mm": round(max(errors), 1),
                         "end_target_error_mm": round(errors[-1], 1)}
    return rows, summary


def draw(rows: list[dict[str, float | int | str]]) -> None:
    fig, ax = plt.subplots(figsize=(9, 8), constrained_layout=True)
    ax.plot([r["leader_x_mm"] for r in rows], [r["leader_y_mm"] for r in rows],
            color=COLORS["Leader"], lw=2, label="Leader ideal")
    for name in LANES:
        ax.plot([r[f"{name}_target_x_mm"] for r in rows],
                [r[f"{name}_target_y_mm"] for r in rows], ls="--",
                color=COLORS[name], label=f"{name} target")
        ax.plot([r[f"{name}_x_mm"] for r in rows],
                [r[f"{name}_y_mm"] for r in rows],
                color=COLORS[name], lw=2, label=f"{name} simulated")
    for index in (0, len(rows)//2, len(rows)-1):
        r = rows[index]
        for name in ("Leader", "F1", "F2"):
            prefix = "leader" if name == "Leader" else name
            draw_body(ax, float(r[f"{prefix}_x_mm"]), float(r[f"{prefix}_y_mm"]),
                      float(r[f"{prefix}_heading_deg"]), COLORS[name],
                      prominent=index == len(rows)-1)
    ax.set(xlabel="forward x (mm)", ylabel="left y (mm)",
           title="Leader endpoint (1000, 1200) mm; R=800 mm; F1/F2 ±400 mm")
    ax.axis("equal"); ax.grid(alpha=.2); ax.legend(loc="upper left", ncol=2)
    fig.savefig(HERE / "compact_turn.png", dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    rows, summary = simulate()
    with (HERE / "simulated.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    (HERE / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    draw(rows)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
