#!/usr/bin/env python3
"""Offline check of the big left turn with the 1.140 ground/odometry scale.

Ground route: 1500 mm straight, R1200 left 90 deg, 1000 mm exit; followers start
on the fixed floor marks (-400, +/-400) mm and keep +/-400 mm lanes.  Every Pi
argument is the ground value divided by SCALE (odometry units).  The live
PathFormationTracker runs on ideal differential-drive motion; outputs are
reported back in ground mm.  No USB, no UDP.
"""
from __future__ import annotations

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

SCALE = 1.140
GROUND = {"straight": 1500.0, "radius": 1200.0, "exit": 1000.0, "lag": 400.0, "lane": 400.0}
ODOM = {k: round(v / SCALE, 1) for k, v in GROUND.items()}
LEADER_SPEED = 60.0  # odometry mm/s command
DT = 0.1
SETTLE_S = 20.0
BODY = np.array([[100, 75], [100, -75], [-150, -75], [-150, 75]], float)


def outline(x, y, h_deg, n=12):
    h = math.radians(h_deg)
    rot = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    pts = []
    for a, b in zip(BODY, np.roll(BODY, -1, axis=0)):
        for t in np.linspace(0, 1, n, endpoint=False):
            pts.append(a + (b - a) * t)
    return (np.array(pts) @ rot.T) + [x, y]


def body_gap(p, q):
    a, b = outline(*p), outline(*q)
    return float(np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)))


def main() -> None:
    key = TurnPathReference(ODOM["straight"], ODOM["radius"], ODOM["exit"], 0, 0, 0)
    path = TurnPath(key)
    arc = ODOM["radius"] * math.pi / 2
    end = ODOM["straight"] + arc + ODOM["exit"]
    end_t = end / LEADER_SPEED
    lanes = {"F1": ODOM["lane"], "F2": -ODOM["lane"]}
    trackers = {n: PathFormationTracker(FormationPlan(
        offset_x_mm=-ODOM["lag"], offset_y_mm=l,
        follower_start_x_mm=-ODOM["lag"], follower_start_y_mm=l)) for n, l in lanes.items()}
    local = {n: status() for n in lanes}
    stopped: dict[str, str] = {}
    rows = []
    for tick in range(int((end_t + SETTLE_S) / DT) + 1):
        t = tick * DT
        s = min(LEADER_SPEED * t, end)
        lx, ly, lh, _ = path.pose_at_s(s, 0.0)
        speed = LEADER_SPEED if t < end_t else 0.0
        yaw = LEADER_SPEED / ODOM["radius"] if ODOM["straight"] < s < ODOM["straight"] + arc else 0.0
        frame = LeaderFrame(status=status(lx, ly, lh), received_monotonic_s=t,
                            source_host="offline", forward_mm_per_second=speed,
                            yaw_rate_radians_per_second=yaw, has_motion_reference=True,
                            formation_phase="moving" if speed else "settle",
                            path=replace(key, progress_s_mm=s))
        row = {"t": t, "L": (lx, ly, lh)}
        for n, lane in lanes.items():
            st = local[n]
            row[n] = (st.x_mm - ODOM["lag"], st.y_mm + lane, st.heading_deg)
            if n in stopped:
                continue
            c = trackers[n].control(frame, st)
            if not c.tracking_safe:
                stopped[n] = f"t={t:.1f}s {c.stop_reason}"
                continue
            row[n + "_cmd"] = (c.left_mm_per_second, c.right_mm_per_second)
            local[n] = step_rover(st, c.left_mm_per_second, c.right_mm_per_second, DT,
                                  trackers[n].plan.wheel_track_mm)
        rows.append(row)

    def g(p):  # odometry pose -> ground pose
        return (p[0] * SCALE, p[1] * SCALE, p[2])

    moving = [r for r in rows if r["t"] <= end_t]
    pairs = {"Leader-F1": ("L", "F1"), "Leader-F2": ("L", "F2"), "F1-F2": ("F1", "F2")}
    summary = {"scale": SCALE, "ground_mm": GROUND, "pi_arguments_odometry_mm": ODOM,
               "leader_speed_odometry_mm_s": LEADER_SPEED,
               "leader_route_odometry_mm": round(end, 1), "moving_s": round(end_t, 1),
               "controller_stops": stopped}
    for name, (a, b) in pairs.items():
        mpu = min(math.dist(g(r[a])[:2], g(r[b])[:2]) for r in rows)
        gap = min(body_gap(g(r[a]), g(r[b])) for r in rows[::5])
        summary[name] = {"min_mpu_ground_mm": round(mpu), "min_body_gap_ground_mm": round(gap),
                         "min_mpu_odometry_mm": round(mpu / SCALE)}
    for n in lanes:
        cmds = [r[n + "_cmd"] for r in moving if n + "_cmd" in r]
        summary[n + "_wheel_cmd_mm_s"] = {"min": min(min(c) for c in cmds), "max": max(max(c) for c in cmds)}
    (HERE / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    fig, ax = plt.subplots(figsize=(8, 7))
    colors = {"L": "#374151", "F1": "#2563eb", "F2": "#ea580c"}
    for key, col in colors.items():
        xy = np.array([g(r[key])[:2] for r in rows])
        ax.plot(xy[:, 0], xy[:, 1], color=col, lw=2, label={"L": "Leader"}.get(key, key))
        for r in rows[:: max(1, len(rows) // 6)] + [rows[-1]]:
            o = outline(*g(r[key]), n=2)
            ax.fill(o[:, 0], o[:, 1], color=col, alpha=0.12, ls="--", ec=col)
            ax.plot(*g(r[key])[:2], "o", mfc="white", mec=col, ms=5)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("ground x (mm)")
    ax.set_ylabel("ground y (mm)")
    ax.set_title("Big left turn on floor marks (-400, ±400); Pi args = ground / 1.140")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(HERE / "scaled_turn.png", dpi=130)


if __name__ == "__main__":
    main()
