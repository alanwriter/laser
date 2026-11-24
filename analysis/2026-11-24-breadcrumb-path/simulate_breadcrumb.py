#!/usr/bin/env python3
"""Offline comparison: analytic route vs breadcrumb (recorded Leader track).

Route D of 2026-11-18 (straight 600, R800 left 90 deg, exit 400, Leader 38 mm/s,
followers (-400, +/-400)).  Both followers run the live follower code with the
same control law; only the path source differs:

* analytic   -> PathFormationTracker (TurnPath from the packet, planned s)
* breadcrumb -> BreadcrumbFormationTracker (track rebuilt from Leader poses)

Scenarios
  ideal   Leader drives the plan exactly; packets every 0.1 s.
  noisy   Broadcast heading noise 0.5 deg, position noise 2 mm (1 sigma),
          10 % of UDP packets lost (follower reuses the last frame).
  detour  Leader actually drives R900 instead of the broadcast R800 plan
          (stands in for manual driving / obstacle avoidance).  Truth is the
          Leader's real track, not the plan.

Followers are ideal differential-drive rovers (same as simulate_site.py).
No USB, no UDP.
"""
from __future__ import annotations

import json
import math
import random
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

from breadcrumb_path import LeaderTarget  # noqa: E402
from follower_formation import (  # noqa: E402
    BreadcrumbFormationTracker, FormationPlan, LeaderFrame, PathFormationTracker,
)
from simulate_formation import status, step_rover  # noqa: E402
from turn_path import TurnPath, TurnPathReference  # noqa: E402

STRAIGHT, RADIUS, EXIT, SPEED, LAG, LANE = 600.0, 800.0, 400.0, 38.0, 400.0, 400.0
DT = 0.1
BODY = np.array([[100, 75], [100, -75], [-150, -75], [-150, 75]], float)
COLORS = {"L": "#374151", "F1": "#2563eb", "F2": "#ea580c"}
SCENARIOS = {
    "ideal": dict(radius=RADIUS, heading_noise_deg=0.0, position_noise_mm=0.0, drop=0.0),
    "noisy": dict(radius=RADIUS, heading_noise_deg=0.5, position_noise_mm=2.0, drop=0.10),
    "detour": dict(radius=900.0, heading_noise_deg=0.0, position_noise_mm=0.0, drop=0.0),
}


def corners(x, y, h_deg):
    h = math.radians(h_deg)
    rot = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    return BODY @ rot.T + [x, y]


def edge_points(p, n=12):
    c = corners(*p)
    return np.array([a + (b - a) * t for a, b in zip(c, np.roll(c, -1, 0))
                     for t in np.linspace(0, 1, n, endpoint=False)])


def body_gap(p, q):
    a, b = edge_points(p), edge_points(q)
    return float(np.min(np.linalg.norm(a[:, None] - b[None], axis=2)))


def run(scenario: str, source: str, seed: int = 7) -> dict:
    cfg = SCENARIOS[scenario]
    rng = random.Random(seed)
    plan_ref = TurnPathReference(STRAIGHT, RADIUS, EXIT, 0, 0, 0)       # what is broadcast
    plan_path = TurnPath(plan_ref)
    true_ref = TurnPathReference(STRAIGHT, cfg["radius"], EXIT, 0, 0, 0)  # what Leader drives
    true_path = TurnPath(true_ref)
    end = true_ref.end_s_mm
    arc_end = STRAIGHT + cfg["radius"] * math.pi / 2
    lanes = {"F1": LANE, "F2": -LANE}
    Tracker = BreadcrumbFormationTracker if source == "breadcrumb" else PathFormationTracker
    trackers = {n: Tracker(FormationPlan(offset_x_mm=-LAG, offset_y_mm=d, follower_start_x_mm=-LAG,
                                         follower_start_y_mm=d)) for n, d in lanes.items()}
    local = {n: status() for n in lanes}
    stops: dict[str, str] = {}
    rows = []
    frame: LeaderFrame | None = None
    for k in range(int(end / SPEED / DT) + 1):
        t = k * DT
        s = min(SPEED * t, end)
        lx, ly, lh, _ = true_path.pose_at_s(s, 0.0)
        if frame is None or rng.random() >= cfg["drop"]:
            yaw = SPEED / cfg["radius"] if STRAIGHT < s < arc_end else 0.0
            tx, ty, th, _ = plan_path.pose_at_s(min(s, plan_ref.end_s_mm) + 300.0, 0.0)
            frame = LeaderFrame(
                status=status(lx + rng.gauss(0, cfg["position_noise_mm"] or 1e-12),
                              ly + rng.gauss(0, cfg["position_noise_mm"] or 1e-12),
                              lh + rng.gauss(0, cfg["heading_noise_deg"] or 1e-12)),
                received_monotonic_s=t, source_host="offline",
                forward_mm_per_second=SPEED, yaw_rate_radians_per_second=yaw,
                has_motion_reference=True, formation_phase="moving",
                path=replace(plan_ref, progress_s_mm=min(s, plan_ref.end_s_mm)),
                next_target=LeaderTarget(tx, ty, th, 300.0),
            )
        row = {"t": t, "s_L": s, "L": (lx, ly, lh)}
        for n, d in lanes.items():
            st = local[n]
            fx, fy, fh = st.x_mm - LAG, st.y_mm + d, st.heading_deg
            row[n] = (fx, fy, fh)
            # Truth: lane of the Leader's real track.
            s_true, lat = true_path.project(fx, fy, d)
            _, _, h_true, k_true = true_path.pose_at_s(s_true, d)
            row[n + "_lat"] = lat
            row[n + "_lag_err"] = (s - LAG) - s_true
            row[n + "_head_err"] = (fh - h_true + 180) % 360 - 180
            row[n + "_k_true"] = k_true
            if n in stops:
                continue
            c = trackers[n].control(frame, st)
            row[n + "_k_used"] = c.path_curvature_per_mm
            if not c.tracking_safe:
                stops[n] = f"t={t:.1f}s {c.stop_reason}"
                continue
            row[n + "_cmd"] = (c.left_mm_per_second, c.right_mm_per_second)
            local[n] = step_rover(st, c.left_mm_per_second, c.right_mm_per_second, DT,
                                  trackers[n].plan.wheel_track_mm)
        rows.append(row)
    return {"rows": rows, "stops": stops, "trackers": trackers}


def summarize(result: dict) -> dict:
    rows = result["rows"]
    out: dict[str, object] = {"controller_stops": result["stops"]}
    for n in ("F1", "F2"):
        lat = np.abs([r[n + "_lat"] for r in rows])
        lag = np.abs([r[n + "_lag_err"] for r in rows])
        head = np.abs([r[n + "_head_err"] for r in rows])
        cmds = [r[n + "_cmd"] for r in rows if n + "_cmd" in r]
        k_err = [abs(r[n + "_k_used"] - r[n + "_k_true"]) * 1000 for r in rows if n + "_k_used" in r]
        out[n] = {
            "lateral_mm_median_max": [round(float(np.median(lat)), 1), round(float(lat.max()), 1)],
            "along_mm_median_max": [round(float(np.median(lag)), 1), round(float(lag.max()), 1)],
            "heading_deg_median_max": [round(float(np.median(head)), 1), round(float(head.max()), 1)],
            "curvature_err_per_m_median_max": [round(float(np.median(k_err)), 3), round(float(max(k_err)), 3)],
            "wheel_cmd_mm_s": [min(min(c) for c in cmds), max(max(c) for c in cmds)],
            "final_heading_deg": round(rows[-1][n][2], 1),
            "min_mpu_to_leader_mm": round(min(math.dist(r["L"][:2], r[n][:2]) for r in rows)),
            "min_body_gap_to_leader_mm": round(min(body_gap(r["L"], r[n]) for r in rows[::5])),
        }
    out["F1-F2_min_body_gap_mm"] = round(min(body_gap(r["F1"], r["F2"]) for r in rows[::5]))
    return out


def main() -> None:
    results = {(sc, src): run(sc, src) for sc in SCENARIOS for src in ("analytic", "breadcrumb")}
    summary = {
        "route": {"straight_mm": STRAIGHT, "radius_mm": RADIUS, "exit_mm": EXIT,
                  "leader_speed_mm_s": SPEED, "follower_offset_mm": [-LAG, LANE]},
        "scenarios": {sc: {k: v for k, v in cfg.items()} for sc, cfg in SCENARIOS.items()},
        "results": {f"{sc}/{src}": summarize(r) for (sc, src), r in results.items()},
    }
    bc = results[("ideal", "breadcrumb")]["trackers"]["F1"].breadcrumbs
    summary["breadcrumb_buffer_end_of_ideal_run"] = {"points": bc.point_count, "pruned": bc.pruned_count}
    (HERE / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    fig = plt.figure(figsize=(14, 10.5))
    grid = fig.add_gridspec(3, 2, width_ratios=[1.05, 1])
    # (a) trajectories: ideal + detour
    ax = fig.add_subplot(grid[:2, 0])
    for sc, style in (("ideal", "-"), ("detour", "-")):
        rb, ra = results[(sc, "breadcrumb")]["rows"], results[(sc, "analytic")]["rows"]
        alpha = 1.0 if sc == "ideal" else 0.55
        for key in ("L", "F1", "F2"):
            xy = np.array([r[key][:2] for r in rb])
            ax.plot(xy[:, 0], xy[:, 1], style, color=COLORS[key], lw=2, alpha=alpha,
                    label=(f"{key if key != 'L' else 'Leader'} breadcrumb" if sc == "ideal" else None))
            if key != "L":
                xa = np.array([r[key][:2] for r in ra])
                ax.plot(xa[:, 0], xa[:, 1], ":", color=COLORS[key], lw=1.6, alpha=alpha,
                        label=(f"{key} analytic" if sc == "ideal" else None))
        if sc == "ideal":
            snaps = [rb[0], rb[len(rb) // 2], rb[-1]]
            for i, r in enumerate(snaps):
                for key in ("L", "F1", "F2"):
                    c = corners(*r[key])
                    ax.fill(c[:, 0], c[:, 1], color=COLORS[key], alpha=0.28 if i == 2 else 0.10, lw=0)
                    ax.plot(*np.vstack([c, c[:1]]).T, "--", color=COLORS[key], lw=0.8)
                    ax.plot(c[:2, 0], c[:2, 1], "-", color=COLORS[key], lw=1.8)
                    ax.plot(*r[key][:2], "o", mfc="white", mec=COLORS[key], ms=5)
    ax.text(-600, 1250, "faded: detour scenario\n(Leader drives R900,\nbroadcast plan R800)", fontsize=8,
            ha="left", va="top", color="#555")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("(a) Route D: straight 600, R800 left, exit 400; Leader 38 mm/s")
    ax.legend(loc="lower right", fontsize=8)

    # (b) lateral error, all scenarios
    ax = fig.add_subplot(grid[0, 1])
    for sc, ls in (("ideal", "-"), ("noisy", "--"), ("detour", "-.")):
        for src, lw, alpha in (("analytic", 1.0, 0.45), ("breadcrumb", 1.8, 1.0)):
            rows = results[(sc, src)]["rows"]
            t = [r["t"] for r in rows]
            for n in ("F1", "F2"):
                ax.plot(t, [r[n + "_lat"] for r in rows], ls, color=COLORS[n], lw=lw, alpha=alpha)
    ax.set_title("(b) lane error vs Leader's real track (thin = analytic, thick = breadcrumb)\n"
                 "solid ideal, dashed noisy, dash-dot detour", fontsize=9)
    ax.set_ylabel("lateral error (mm)")
    ax.grid(alpha=0.3)

    # (c) along-track error
    ax = fig.add_subplot(grid[1, 1])
    for sc, ls in (("ideal", "-"), ("noisy", "--"), ("detour", "-.")):
        for src, lw, alpha in (("analytic", 1.0, 0.45), ("breadcrumb", 1.8, 1.0)):
            rows = results[(sc, src)]["rows"]
            t = [r["t"] for r in rows]
            for n in ("F1", "F2"):
                ax.plot(t, [r[n + "_lag_err"] for r in rows], ls, color=COLORS[n], lw=lw, alpha=alpha)
    ax.set_title("(c) along-route gap error (target s_L-400 minus actual)", fontsize=9)
    ax.set_ylabel("along error (mm)")
    ax.grid(alpha=0.3)

    # (d) curvature used vs truth
    ax = fig.add_subplot(grid[2, :])
    for sc, ls in (("ideal", "-"), ("noisy", "--")):
        rows = results[(sc, "breadcrumb")]["rows"]
        t = [r["t"] for r in rows if "F1_k_used" in r]
        for n in ("F1", "F2"):
            ax.plot(t, [r[n + "_k_used"] * 1000 for r in rows if n + "_k_used" in r], ls,
                    color=COLORS[n], lw=1.5, label=f"{n} breadcrumb estimate ({sc})")
    rows = results[("ideal", "analytic")]["rows"]
    for n in ("F1", "F2"):
        ax.plot([r["t"] for r in rows], [r[n + "_k_true"] * 1000 for r in rows], color="black",
                lw=0.9, alpha=0.6, label="true lane curvature" if n == "F1" else None)
    ax.set_title("(d) lane curvature used for yaw feed-forward", fontsize=9)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("curvature (1/m)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=3)
    fig.tight_layout()
    fig.savefig(HERE / "breadcrumb_vs_analytic_route_D.png", dpi=130)


if __name__ == "__main__":
    main()
