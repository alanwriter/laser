#!/usr/bin/env python3
"""Replay recorded Leader frames to separate reference error from tracking error.

Part 1 (no vehicle model): feed the Leader poses that each real follower
received (f1.csv / f2.csv, in order) into BreadcrumbPath variants and compare
their lane curvature with the ideal route-D lane at the follower's real
position.

Part 2 (closed loop): drive the live follower controller with the recorded
Leader frames (leader.csv) on a simulated follower, either ideal or with a
first-order wheel-speed lag, for each path source.  The simulated follower
does not share the real follower's odometry or wheel quirks, so only
differences *between path sources* are meaningful.
"""
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
from breadcrumb_path import BreadcrumbPath  # noqa: E402
from follower_formation import (  # noqa: E402
    BreadcrumbFormationTracker, FormationPlan, LeaderFrame, PathFormationTracker,
)
from simulate_formation import status  # noqa: E402
from turn_path import TurnPath, TurnPathReference  # noqa: E402

RUN = HERE / "real_run_084014Z"
OLD_RUN = ROOT / "analysis/2026-11-18-left-turn-site/real_run_123302Z"
REF = TurnPathReference(600.0, 800.0, 400.0, 0, 0, 0)
ROUTE = TurnPath(REF)
LANES = {"F1": 400.0, "F2": -400.0}
COLORS = {"F1": "#2563eb", "F2": "#ea580c"}
VARIANTS = {
    "heading 160 (ran 11/24)": dict(tangent_source="heading", smoothing_window_mm=160.0),
    "heading 300 (default from 11/24 eve)": dict(tangent_source="heading", smoothing_window_mm=300.0),
    "heading 400": dict(tangent_source="heading", smoothing_window_mm=400.0),
    "position 300": dict(tangent_source="position", smoothing_window_mm=300.0),
    "position 400": dict(tangent_source="position", smoothing_window_mm=400.0),
}


def read(path: Path, prefix: str) -> list[dict]:
    with open(path) as fh:
        return [r for r in csv.DictReader(fh) if r["event"].startswith(prefix)]


# ---------------------------------------------------------------- part 1
def curvature_check() -> dict:
    out: dict = {}
    series: dict = {}
    for name, d in LANES.items():
        rows = read(RUN / f"{name.lower()}.csv", "control_")
        out[name] = {}
        for label, kw in VARIANTS.items():
            path = BreadcrumbPath(**kw)
            last = None
            errs, t, ks = [], [], []
            t0 = float(rows[0]["elapsed_s"])
            for r in rows:
                pose = (float(r["leader_x_mm"]), float(r["leader_y_mm"]), float(r["leader_heading_deg"]))
                if pose != last:
                    path.observe(*pose)
                    last = pose
                fx, fy = float(r["follower_x_mm"]) - 400.0, float(r["follower_y_mm"]) + d
                s_ideal = ROUTE.project(fx, fy, d)[0]
                s_bc = path.project(fx, fy, d)[0]
                k_ideal = ROUTE.pose_at_s(s_ideal, d)[3]
                k_bc = path.pose_at_s(s_bc, d)[3]
                if s_ideal > 0:  # behind the start line both are 0 by construction
                    errs.append((k_bc - k_ideal) * 1000)
                t.append(float(r["elapsed_s"]) - t0)
                ks.append(k_bc * 1000)
            e = np.array(errs)
            # exclude +/-150 mm around the two curvature steps: smoothing there is expected
            out[name][label] = {"rms_per_m": round(float(np.sqrt(np.mean(e ** 2))), 3),
                                "max_abs_per_m": round(float(np.abs(e).max()), 3)}
            series[(name, label)] = (t, ks)
    return out, series


# ---------------------------------------------------------------- part 2
def leader_frames(folder: Path) -> list[LeaderFrame]:
    rows = read(folder / "leader.csv", "control")
    frames = []
    for r in rows:
        left, right = float(r["control_left_mm_per_second"]), float(r["control_right_mm_per_second"])
        s = min(max(float(r["control_forward_mm"]), 0.0), REF.end_s_mm)
        frames.append(LeaderFrame(
            status=status(float(r["status_x_mm"]), float(r["status_y_mm"]), float(r["status_heading_deg"])),
            received_monotonic_s=float(r["elapsed_s"]), source_host="replay",
            forward_mm_per_second=(left + right) / 2, yaw_rate_radians_per_second=(right - left) / 130.0,
            has_motion_reference=True, formation_phase="moving",
            path=replace(REF, progress_s_mm=s)))
    return frames


def closed_loop(frames: list[LeaderFrame], source: str, kw: dict | None, lag_s: float) -> dict:
    out = {}
    for name, d in LANES.items():
        plan = FormationPlan(offset_x_mm=-400, offset_y_mm=d, follower_start_x_mm=-400, follower_start_y_mm=d)
        tracker = (BreadcrumbFormationTracker(plan, BreadcrumbPath(**kw)) if source == "breadcrumb"
                   else PathFormationTracker(plan))
        x, y, h = 0.0, 0.0, 0.0
        vl = vr = 0.0
        lat, ts = [], []
        stop = ""
        t_end = frames[-1].received_monotonic_s
        k = 0
        t = frames[0].received_monotonic_s
        cmd = (0, 0)
        while t <= t_end:
            while k + 1 < len(frames) and frames[k + 1].received_monotonic_s <= t:
                k += 1
            frame = replace(frames[k], received_monotonic_s=t)  # fresh each control cycle
            if hasattr(tracker, "observe_leader"):
                tracker.observe_leader(frames[k])
            c = tracker.control(frame, status(x, y, math.degrees(h)))
            if not c.tracking_safe:
                stop = f"t={t:.1f} {c.stop_reason}"
                break
            cmd = (c.left_mm_per_second, c.right_mm_per_second)
            for _ in range(5):  # 0.1 s in 20 ms substeps
                a = 1.0 if lag_s <= 0 else min(1.0, 0.02 / lag_s)
                vl += a * (cmd[0] - vl)
                vr += a * (cmd[1] - vr)
                v, w = (vl + vr) / 2, (vr - vl) / 130.0
                x += v * math.cos(h) * 0.02
                y += v * math.sin(h) * 0.02
                h += w * 0.02
            wx, wy = x - 400.0, y + d
            lat.append(ROUTE.project(wx, wy, d)[1])
            ts.append(t)
            t += 0.1
        a = np.abs(lat)
        out[name] = {"lat_median_max_mm": [round(float(np.median(a)), 1), round(float(a.max()), 1)],
                     "stop": stop, "t": ts, "lat": lat}
    return out


def main() -> None:
    curv, series = curvature_check()
    frames = {"11/24 Leader": leader_frames(RUN), "11/18 Leader": leader_frames(OLD_RUN)}
    sources = {"analytic": ("analytic", None), **{f"breadcrumb {k}": ("breadcrumb", v) for k, v in VARIANTS.items()}}
    loops = {}
    for fl, fr in frames.items():
        for lag in (0.0, 0.4):
            for label, (src, kw) in sources.items():
                loops[(fl, lag, label)] = closed_loop(fr, src, kw, lag)
    summary = {
        "part1_curvature_error_vs_ideal_lane_at_real_follower_s": curv,
        "part2_closed_loop_lateral_error_vs_ideal_lane": {
            f"{fl} | wheel lag {lag:.1f} s | {label}": {n: {"lat_median_max_mm": r[n]["lat_median_max_mm"],
                                                           "stop": r[n]["stop"]} for n in LANES}
            for (fl, lag, label), r in loops.items()},
    }
    (HERE / "replay_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=False)
    ax = axes[0]
    for (name, label), (t, ks) in series.items():
        if label in ("heading 160 (ran 11/24)", "position 400"):
            ax.plot(t, ks, "-" if label.startswith("position") else ":", color=COLORS[name],
                    lw=1.8 if label.startswith("position") else 1.2, label=f"{name} {label}")
    for name, d in LANES.items():
        rows = read(RUN / f"{name.lower()}.csv", "control_")
        t0 = float(rows[0]["elapsed_s"])
        ax.plot([float(r["elapsed_s"]) - t0 for r in rows],
                [ROUTE.pose_at_s(ROUTE.project(float(r["follower_x_mm"]) - 400, float(r["follower_y_mm"]) + d, d)[0], d)[3] * 1000
                 for r in rows], "--", color="black", lw=0.8, alpha=0.6)
    ax.set_title("Part 1: lane curvature rebuilt from the Leader poses F1/F2 actually received (11/24)\n"
                 "dotted = heading fit (what ran), solid = position fit, black dashed = ideal lane", fontsize=10)
    ax.set_ylabel("curvature (1/m)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    for ax, lag in zip(axes[1:], (0.0, 0.4)):
        for label, ls in (("analytic", "--"), ("breadcrumb heading 160 (ran 11/24)", ":"), ("breadcrumb position 400", "-")):
            r = loops[("11/24 Leader", lag, label)]
            for name in LANES:
                ax.plot(r[name]["t"], r[name]["lat"], ls, color=COLORS[name], lw=1.5, label=f"{name} {label}")
        ax.set_title(f"Part 2: closed-loop replay of the 11/24 Leader frames, simulated follower, wheel lag {lag:.1f} s",
                     fontsize=10)
        ax.set_ylabel("lateral error vs ideal lane (mm)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=3)
    axes[-1].set_xlabel("Leader elapsed time (s)")
    fig.tight_layout()
    fig.savefig(HERE / "replay_reference_error.png", dpi=130)


if __name__ == "__main__":
    main()
