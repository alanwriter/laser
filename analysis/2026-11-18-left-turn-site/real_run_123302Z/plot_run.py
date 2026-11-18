#!/usr/bin/env python3
"""Plot the 2026-11-18 12:33Z left turn (exit 400 mm, Leader 38 mm/s) from on-board CSVs.

Follower local odometry is shifted by its declared start (-400, +/-400) mm;
start heading 0.  Points are MPU anchors; outlines follow PLOT_CONVENTIONS.md.
These are on-board estimates, not ground truth.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
START = {"F1": (-400.0, 400.0), "F2": (-400.0, -400.0)}
COLORS = {"Leader": "#374151", "F1": "#2563eb", "F2": "#ea580c"}
BODY = np.array([[100, 75], [100, -75], [-150, -75], [-150, 75], [100, 75]], float)


def load(name: str):
    rows = list(csv.DictReader(open(HERE / name)))
    out = []
    for r in rows:
        if name == "leader.csv":
            if r["event"] != "control":
                continue
            out.append((float(r["elapsed_s"]), float(r["status_x_mm"]),
                        float(r["status_y_mm"]), float(r["status_heading_deg"])))
        else:
            if r["event"] not in ("control_moving", "safety_stop") or not r["follower_x_mm"]:
                continue
            key = "F1" if name == "f1.csv" else "F2"
            sx, sy = START[key]
            out.append((float(r["elapsed_s"]), sx + float(r["follower_x_mm"]),
                        sy + float(r["follower_y_mm"]), float(r["follower_heading_deg"])))
    return out


def outline(ax, x, y, h, color, alpha):
    t = math.radians(h)
    rot = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    pts = BODY @ rot.T + [x, y]
    ax.fill(pts[:, 0], pts[:, 1], color=color, alpha=alpha, lw=0)
    ax.plot(pts[:, 0], pts[:, 1], "--", color=color, lw=0.8)
    ax.plot(pts[:2, 0], pts[:2, 1], "-", color=color, lw=1.6)
    ax.plot(x, y, "o", mfc="white", mec=color, ms=6)


def main() -> None:
    data = {"Leader": load("leader.csv"), "F1": load("f1.csv"), "F2": load("f2.csv")}
    fig, ax = plt.subplots(figsize=(8, 7.5))
    for name, rows in data.items():
        a = np.array(rows)
        ax.plot(a[:, 1], a[:, 2], color=COLORS[name], lw=2, label=f"{name} (on-board)")
        for i in (0, len(rows) // 2, len(rows) - 1):
            outline(ax, *rows[i][1:], COLORS[name], 0.25 if i == len(rows) - 1 else 0.1)
    import sys
    sys.path.insert(0, str(HERE.parents[2] / "pi_rover"))
    from turn_path import TurnPath, TurnPathReference
    plan = TurnPath(TurnPathReference(600, 800, 400, 0, 0, 0))
    for name, lane in (("Leader", 0.0), ("F1", 400.0), ("F2", -400.0)):
        s = np.linspace(-400 if lane else 0, plan.end_s_mm, 400)
        xy = np.array([plan.pose_at_s(v, lane)[:2] for v in s])
        ax.plot(xy[:, 0], xy[:, 1], ":", color=COLORS[name], lw=1.2)
    ax.plot([], [], ":", color="gray", label="planned lanes")
    ax.add_patch(plt.Rectangle((-700, -700), 2700, 2100, fill=False, ec="#b91c1c", ls="--", lw=1.2))
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (mm, on-board odometry)")
    ax.set_ylabel("y (mm, on-board odometry)")
    ax.set_title("2026-11-18 12:33Z left turn (exit 400, Leader 38 mm/s): on-board estimates vs planned lanes")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(HERE / "onboard_paths.png", dpi=130)


if __name__ == "__main__":
    main()
