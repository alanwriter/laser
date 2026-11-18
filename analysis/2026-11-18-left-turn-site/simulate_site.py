#!/usr/bin/env python3
"""Offline left-turn candidate sized for the lab floor (2026-11-18).

Firmware now reports ground mm, so Pi arguments equal floor dimensions.  The
live PathFormationTracker drives ideal differential-drive followers; the
Leader follows its plan exactly.  Site box: forward x <= 2000 mm and left
y <= 1200 mm, both measured from the Leader start MPU mark (assumption to be
confirmed by the operator).  No USB, no UDP.
"""
from __future__ import annotations

import argparse
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

BODY = np.array([[100, 75], [100, -75], [-150, -75], [-150, 75]], float)
COLORS = {"L": "#374151", "F1": "#2563eb", "F2": "#ea580c"}
DT = 0.1


def corners(x, y, h_deg):
    h = math.radians(h_deg)
    rot = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    return BODY @ rot.T + [x, y]


def edge_points(p, n=12):
    c = corners(*p)
    return np.array([a + (b - a) * t for a, b in zip(c, np.roll(c, -1, 0))
                     for t in np.linspace(0, 1, n, endpoint=False)])


def gap(p, q):
    a, b = edge_points(p), edge_points(q)
    return float(np.min(np.linalg.norm(a[:, None] - b[None], axis=2)))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--straight", type=float, default=600.0)
    ap.add_argument("--radius", type=float, default=800.0)
    ap.add_argument("--exit", type=float, default=300.0)
    ap.add_argument("--speed", type=float, default=30.0)
    ap.add_argument("--lag", type=float, default=400.0)
    ap.add_argument("--lane", type=float, default=400.0)
    ap.add_argument("--start-back", type=float, default=None,
                    help="follower start distance behind Leader (default = --lag)")
    ap.add_argument("--site-x", type=float, default=2000.0)
    ap.add_argument("--site-left", type=float, default=1200.0)
    ap.add_argument("--name", default="candidate")
    a = ap.parse_args()

    start_back = a.lag if a.start_back is None else a.start_back
    key = TurnPathReference(a.straight, a.radius, a.exit, 0, 0, 0)
    path = TurnPath(key)
    arc = a.radius * math.pi / 2
    end = a.straight + arc + a.exit
    end_t = end / a.speed
    lanes = {"F1": a.lane, "F2": -a.lane}
    trackers = {n: PathFormationTracker(FormationPlan(
        offset_x_mm=-a.lag, offset_y_mm=l, follower_start_x_mm=-start_back,
        follower_start_y_mm=l)) for n, l in lanes.items()}
    local = {n: status() for n in lanes}
    stops: dict[str, str] = {}
    rows = []
    for k in range(int(end_t / DT) + 1):
        t = k * DT
        s = min(a.speed * t, end)
        lx, ly, lh, _ = path.pose_at_s(s, 0.0)
        yaw = a.speed / a.radius if a.straight < s < a.straight + arc else 0.0
        frame = LeaderFrame(status=status(lx, ly, lh), received_monotonic_s=t,
                            source_host="offline", forward_mm_per_second=a.speed,
                            yaw_rate_radians_per_second=yaw, has_motion_reference=True,
                            formation_phase="moving", path=replace(key, progress_s_mm=s))
        row = {"t": t, "L": (lx, ly, lh)}
        for n, lane in lanes.items():
            st = local[n]
            row[n] = (st.x_mm - start_back, st.y_mm + lane, st.heading_deg)
            if n in stops:
                continue
            c = trackers[n].control(frame, st)
            if not c.tracking_safe:
                stops[n] = f"t={t:.1f}s {c.stop_reason}"
                continue
            row[n + "_cmd"] = (c.left_mm_per_second, c.right_mm_per_second)
            local[n] = step_rover(st, c.left_mm_per_second, c.right_mm_per_second, DT,
                                  trackers[n].plan.wheel_track_mm)
        rows.append(row)

    # Leader stops -> followers stop on the `stopped` frame: final poses = last row.
    pts = np.vstack([corners(*r[n]) for r in rows for n in ("L", "F1", "F2")])
    pairs = {"Leader-F1": ("L", "F1"), "Leader-F2": ("L", "F2"), "F1-F2": ("F1", "F2")}
    summary = {
        "pi_args": {"straight_mm": a.straight, "radius_mm": a.radius, "exit_mm": a.exit,
                    "leader_speed_mm_s": a.speed, "follower_offset_x_mm": -a.lag, "follower_start_x_mm": -start_back,
                    "follower_lane_mm": a.lane},
        "leader_route_mm": round(end, 1), "moving_s": round(end_t, 1),
        "controller_stops": stops,
        "body_extent_mm": {"x_min": round(pts[:, 0].min()), "x_max": round(pts[:, 0].max()),
                           "y_min": round(pts[:, 1].min()), "y_max": round(pts[:, 1].max())},
        "final_heading_deg": {n: round(rows[-1][n][2], 1) for n in ("L", "F1", "F2")},
    }
    for name, (p, q) in pairs.items():
        summary[name] = {"min_mpu_mm": round(min(math.dist(r[p][:2], r[q][:2]) for r in rows)),
                         "min_body_gap_mm": round(min(gap(r[p], r[q]) for r in rows[::5]))}
    for n in lanes:
        cmds = [r[n + "_cmd"] for r in rows if n + "_cmd" in r]
        summary[n + "_wheel_cmd_mm_s"] = [min(min(c) for c in cmds), max(max(c) for c in cmds)]
    (HERE / f"{a.name}.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    fig, ax = plt.subplots(figsize=(8.5, 7))
    ax.add_patch(plt.Rectangle((-700, -700), a.site_x + 700, a.site_left + 700, fill=False,
                               ec="#b91c1c", lw=1.5, ls="--"))
    ax.text(a.site_x - 20, a.site_left - 40, f"site limit\nx ≤ {a.site_x:.0f}, y ≤ {a.site_left:.0f}",
            color="#b91c1c", ha="right", va="top", fontsize=9)
    labels = {"L": "Leader", "F1": "F1", "F2": "F2"}
    for key_, col in COLORS.items():
        xy = np.array([r[key_][:2] for r in rows])
        ax.plot(xy[:, 0], xy[:, 1], color=col, lw=2, label=labels[key_])
        snaps = [rows[0], rows[len(rows) // 3], rows[2 * len(rows) // 3], rows[-1]]
        for i, r in enumerate(snaps):
            c = corners(*r[key_])
            ax.fill(c[:, 0], c[:, 1], color=col, alpha=0.28 if i == 3 else 0.10, lw=0)
            ax.plot(*np.vstack([c, c[:1]]).T, "--", color=col, lw=0.8)
            ax.plot(c[:2, 0], c[:2, 1], "-", color=col, lw=1.8)
            ax.plot(*r[key_][:2], "o", mfc="white", mec=col, ms=5)
            if i in (1, 2) and key_ == "L":
                ax.annotate(f"t={r['t']:.0f}s", r[key_][:2], xytext=(6, -12),
                            textcoords="offset points", fontsize=8, color=col)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("forward x (mm, floor)")
    ax.set_ylabel("left y (mm, floor)")
    ax.set_title(f"Left turn: straight {a.straight:.0f}, R {a.radius:.0f}, exit {a.exit:.0f} mm; "
                 f"Leader {a.speed:.0f} mm/s")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(HERE / f"{a.name}.png", dpi=130)


if __name__ == "__main__":
    main()
