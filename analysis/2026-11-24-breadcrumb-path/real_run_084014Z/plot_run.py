#!/usr/bin/env python3
"""Route D, 2026-11-24 08:40:14Z: breadcrumb followers vs 2026-11-18 analytic run.

All positions are on-board odometry + MPU estimates (not ground truth).  Both
runs are scored against the same ideal lanes (TurnPath 600/R800/400 offset
+/-400 mm) and against the Leader's own recorded track offset +/-400 mm.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "pi_rover"))
from breadcrumb_path import BreadcrumbPath  # noqa: E402
from turn_path import TurnPath, TurnPathReference  # noqa: E402

RUNS = {
    "2026-11-24 breadcrumb": HERE,
    "2026-11-18 analytic": ROOT / "analysis/2026-11-18-left-turn-site/real_run_123302Z",
}
LANES = {"F1": 400.0, "F2": -400.0}
ROUTE = TurnPath(TurnPathReference(600.0, 800.0, 400.0, 0, 0, 0))
BODY = np.array([[100, 75], [100, -75], [-150, -75], [-150, 75]], float)
COLORS = {"L": "#374151", "F1": "#2563eb", "F2": "#ea580c"}


def num(v: str) -> float | None:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(folder: Path) -> dict:
    with open(folder / "leader.csv") as fh:
        leader = [r for r in csv.DictReader(fh) if r.get("event") in ("control", "completed")]
    lx = [(float(r["status_x_mm"]), float(r["status_y_mm"]), float(r["status_heading_deg"])) for r in leader]
    out = {"L": lx}
    for name, d in LANES.items():
        with open(folder / f"{name.lower()}.csv") as fh:
            rows = [r for r in csv.DictReader(fh) if r["event"].startswith("control_")]
        out[name] = rows
    return out


def corners(x, y, h):
    h = math.radians(h)
    rot = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    return BODY @ rot.T + [x, y]


def leader_track(L) -> BreadcrumbPath:
    path = BreadcrumbPath()
    for x, y, h in L:
        path.observe(x, y, h)
    return path


def metrics(run: dict) -> dict:
    out = {}
    L = run["L"]
    track = leader_track(L)
    out["leader_end"] = [round(v, 1) for v in L[-1]]
    lat_plan = [abs(ROUTE.project(x, y, 0.0)[1]) for x, y, _ in L]
    out["leader_dev_from_plan_mm_median_max"] = [round(float(np.median(lat_plan)), 1), round(max(lat_plan), 1)]
    for name, d in LANES.items():
        rows = run[name]
        xs = [float(r["follower_x_mm"]) - 400.0 for r in rows]
        ys = [float(r["follower_y_mm"]) + d for r in rows]
        hs = [float(r["follower_heading_deg"]) for r in rows]
        plan = [abs(ROUTE.project(x, y, d)[1]) for x, y in zip(xs, ys)]
        # fresh projection each time against the whole leader track
        trk = []
        for x, y in zip(xs, ys):
            track._projection_hint = None
            trk.append(abs(track.project(x, y, d)[1]))
        ctl_lat = [abs(float(r["control_lateral_error_mm"])) for r in rows]
        along = [float(r["control_longitudinal_error_mm"]) for r in rows]
        cmds = [(int(float(r["control_left_mm_per_second"])), int(float(r["control_right_mm_per_second"]))) for r in rows]
        ld = [float(r["control_leader_distance_mm"]) for r in rows]
        out[name] = {
            "end_world": [round(xs[-1]), round(ys[-1]), round(hs[-1], 1)],
            "lat_vs_ideal_lane_median_max": [round(float(np.median(plan)), 1), round(max(plan), 1)],
            "lat_vs_leader_track_lane_median_max": [round(float(np.median(trk)), 1), round(max(trk), 1)],
            "controller_lat_median_max": [round(float(np.median(ctl_lat)), 1), round(max(ctl_lat), 1)],
            "controller_along_min_max": [round(min(along), 1), round(max(along), 1)],
            "min_leader_distance_mm": round(min(ld)),
            "wheel_cmd_min_max": [min(min(c) for c in cmds), max(max(c) for c in cmds)],
            "control_rows": len(rows),
        }
    return out


def main() -> None:
    runs = {k: load(v) for k, v in RUNS.items()}
    summary = {k: metrics(r) for k, r in runs.items()}
    (HERE / "metrics.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    fig = plt.figure(figsize=(14, 9.5))
    g = fig.add_gridspec(2, 2, width_ratios=[1.1, 1])
    ax = fig.add_subplot(g[:, 0])
    for name, d in [("L", 0.0), *LANES.items()]:
        xs, ys, _, _ = zip(*[ROUTE.pose_at_s(s, d) for s in np.linspace(-400, ROUTE.end_s_mm, 300)])
        ax.plot(xs, ys, "--", color=COLORS[name], lw=0.9, alpha=0.6)
    for label, run in runs.items():
        new = "breadcrumb" in label
        for name in ("L", "F1", "F2"):
            if name == "L":
                pts = run["L"]
            else:
                d = LANES[name]
                pts = [(float(r["follower_x_mm"]) - 400, float(r["follower_y_mm"]) + d,
                        float(r["follower_heading_deg"])) for r in run[name]]
            xy = np.array([p[:2] for p in pts])
            ax.plot(xy[:, 0], xy[:, 1], "-" if new else ":", color=COLORS[name], lw=2 if new else 1.3,
                    alpha=1 if new else 0.7, label=f"{'Leader' if name == 'L' else name} {label}")
            if new:
                for i, p in enumerate((pts[0], pts[len(pts) // 2], pts[-1])):
                    c = corners(*p)
                    ax.fill(c[:, 0], c[:, 1], color=COLORS[name], alpha=0.28 if i == 2 else 0.10, lw=0)
                    ax.plot(*np.vstack([c, c[:1]]).T, "--", color=COLORS[name], lw=0.8)
                    ax.plot(c[:2, 0], c[:2, 1], "-", color=COLORS[name], lw=1.8)
                    ax.plot(*p[:2], "o", mfc="white", mec=COLORS[name], ms=5)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (mm, on-board estimate)")
    ax.set_ylabel("y (mm)")
    ax.set_title("Route D on-board paths: solid = 11/24 breadcrumb, dotted = 11/18 analytic,\n"
                 "thin dashed = ideal lanes", fontsize=10)
    ax.legend(fontsize=7, loc="lower right")

    ax = fig.add_subplot(g[0, 1])
    for label, run in runs.items():
        new = "breadcrumb" in label
        for name, d in LANES.items():
            rows = run[name]
            t = [float(r["elapsed_s"]) - float(rows[0]["elapsed_s"]) for r in rows]
            lat = [ROUTE.project(float(r["follower_x_mm"]) - 400, float(r["follower_y_mm"]) + d, d)[1] for r in rows]
            ax.plot(t, lat, "-" if new else ":", color=COLORS[name], lw=1.8 if new else 1.2,
                    label=f"{name} {label}")
    ax.set_ylabel("lateral offset from ideal lane (mm)")
    ax.set_xlabel("time since follower start (s)")
    ax.set_title("Lane error vs ideal R800 lane (on-board)", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)

    ax = fig.add_subplot(g[1, 1])
    run = runs["2026-11-24 breadcrumb"]
    for name, d in LANES.items():
        rows = run[name]
        t = [float(r["elapsed_s"]) - float(rows[0]["elapsed_s"]) for r in rows]
        ax.plot(t, [float(r["control_path_curvature_per_mm"]) * 1000 for r in rows], color=COLORS[name],
                lw=1.6, label=f"{name} breadcrumb estimate")
        s_act = [ROUTE.project(float(r["follower_x_mm"]) - 400, float(r["follower_y_mm"]) + d, d)[0] for r in rows]
        ax.plot(t, [ROUTE.pose_at_s(s, d)[3] * 1000 for s in s_act], "--", color=COLORS[name], lw=1,
                alpha=0.7, label=f"{name} ideal lane")
    ax.set_ylabel("lane curvature (1/m)")
    ax.set_xlabel("time since follower start (s)")
    ax.set_title("Curvature used for feed-forward (11/24)", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(HERE / "onboard_comparison.png", dpi=130)


if __name__ == "__main__":
    main()
