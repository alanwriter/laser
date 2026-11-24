#!/usr/bin/env python3
"""Controller error of the three most recent three-rover left turns.

Assumption (requested): on-board odometry + MPU pose is exact.  The errors
below are therefore pure *controller* errors -- how far each rover is from
the reference its own controller was tracking -- not ground-truth errors.

Runs
  B   2026-11-18 12:23:41Z  straight 600 / R800 / exit 200, Leader 30 mm/s, analytic path
  D   2026-11-18 12:33:02Z  straight 600 / R800 / exit 400, Leader 38 mm/s, analytic path
  D'  2026-11-24 08:40:14Z  same route D, Leader 38 mm/s, breadcrumb path (window 160 mm)

Errors are in the path (Serret-Frenet) frame of each rover's reference:
  e_y  cross-track (left +), e_s along-track gap error (target s minus actual s),
  e_psi heading error (reference tangent minus rover heading).
Segments are by the follower's arc length on the planned route.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "pi_rover"))
from turn_path import TurnPath, TurnPathReference  # noqa: E402

RUNS = {
    "B  11/18 30 mm/s 解析": dict(folder=ROOT / "analysis/2026-11-18-left-turn-site/real_run_122341Z", exit=200.0),
    "D  11/18 38 mm/s 解析": dict(folder=ROOT / "analysis/2026-11-18-left-turn-site/real_run_123302Z", exit=400.0),
    "D' 11/24 38 mm/s breadcrumb": dict(folder=ROOT / "analysis/2026-11-24-breadcrumb-path/real_run_084014Z", exit=400.0),
}
LANES = {"F1": 400.0, "F2": -400.0}
STRAIGHT, RADIUS, TRACK = 600.0, 800.0, 130.0
ARC_END = STRAIGHT + RADIUS * math.pi / 2
SEGMENTS = [("直線", -1e9, STRAIGHT), ("進彎 0–300 mm", STRAIGHT, STRAIGHT + 300),
            ("穩定彎道", STRAIGHT + 300, ARC_END), ("出彎", ARC_END, 1e9)]
STEP_LIMIT, YAW_LIMIT = 10, math.floor(0.25 * TRACK)


def read(path: Path, prefix: str) -> list[dict]:
    with open(path) as fh:
        return [r for r in csv.DictReader(fh) if r["event"].startswith(prefix)]


def f(r: dict, k: str) -> float:
    return float(r[k])


def stats(a) -> dict:
    a = np.asarray(a, float)
    if not len(a):
        return {}
    return {"rms": round(float(np.sqrt(np.mean(a ** 2))), 2), "max_abs": round(float(np.abs(a).max()), 2),
            "p95_abs": round(float(np.percentile(np.abs(a), 95)), 2), "mean": round(float(a.mean()), 2), "n": int(len(a))}


def dominant_period(t, x) -> float | None:
    t, x = np.asarray(t), np.asarray(x) - np.mean(x)
    if len(t) < 50:
        return None
    tu = np.arange(t[0], t[-1], 0.1)
    xu = np.interp(tu, t, x)
    p = np.abs(np.fft.rfft(xu * np.hanning(len(xu))))
    fr = np.fft.rfftfreq(len(xu), 0.1)
    m = (fr > 1 / 40) & (fr < 1 / 3)
    return round(float(1 / fr[m][np.argmax(p[m])]), 1)


def follower(run: dict, name: str, route: TurnPath) -> dict:
    d = LANES[name]
    rows = read(run["folder"] / f"{name.lower()}.csv", "control_")
    t = np.array([f(r, "monotonic_s") for r in rows]); t -= t[0]
    x = np.array([f(r, "follower_x_mm") - 400 for r in rows]); y = np.array([f(r, "follower_y_mm") + d for r in rows])
    h = np.unwrap(np.radians([f(r, "follower_heading_deg") for r in rows]))
    s = np.array([route.project(xi, yi, d)[0] for xi, yi in zip(x, y)])
    ey = np.array([f(r, "control_lateral_error_mm") for r in rows])
    es = np.array([f(r, "control_longitudinal_error_mm") for r in rows])
    epsi = np.array([f(r, "control_heading_error_deg") for r in rows])
    L = np.array([f(r, "control_left_mm_per_second") for r in rows])
    R = np.array([f(r, "control_right_mm_per_second") for r in rows])
    v, w = (L + R) / 2, (R - L) / TRACK
    dt = np.diff(t)
    w_meas = np.diff(h) / np.where(dt > 0, dt, np.nan)
    age = np.array([f(r, "leader_frame_age_s") for r in rows])
    lx = np.array([f(r, "leader_x_mm") for r in rows]); ly = np.array([f(r, "leader_y_mm") for r in rows])
    seg = {}
    for label, lo, hi in SEGMENTS:
        m = (s >= lo) & (s < hi)
        if m.sum() >= 5:
            seg[label] = {"e_y": stats(ey[m]), "e_s": stats(es[m]), "e_psi": stats(epsi[m]), "length_mm": round(float(np.ptp(s[m])))}
    step_hit = (np.abs(np.diff(L)) >= STEP_LIMIT) | (np.abs(np.diff(R)) >= STEP_LIMIT)
    yaw_hit = np.abs(R - L) >= YAW_LIMIT
    moving = t > 2.0
    return {
        "series": {"t": t.tolist(), "s": s.tolist(), "e_y": ey.tolist(), "e_s": es.tolist(), "e_psi": epsi.tolist()},
        "all": {"e_y": stats(ey), "e_s": stats(es), "e_psi": stats(epsi)},
        "segments": seg,
        "e_y_period_s": dominant_period(t[moving], ey[moving]),
        "smoothness": {"dv_rms_mm_s_per_cycle": round(float(np.sqrt(np.mean(np.diff(v) ** 2))), 2),
                       "dw_rms_rad_s_per_cycle": round(float(np.sqrt(np.mean(np.diff(w) ** 2))), 4),
                       "step_limit_hit_pct": round(100 * float(step_hit.mean()), 1),
                       "yaw_limit_hit_pct": round(100 * float(yaw_hit.mean()), 1),
                       "wheel_cmd_min_max": [int(min(L.min(), R.min())), int(max(L.max(), R.max()))]},
        "actuation": {"yaw_rate_meas_minus_cmd_rms": round(float(np.sqrt(np.nanmean((w_meas - w[:-1]) ** 2))), 3),
                      "yaw_rate_cmd_rms": round(float(np.sqrt(np.mean(w ** 2))), 3)},
        "timing": {"loop_period_median_s": round(float(np.median(dt)), 3), "loop_period_p95_s": round(float(np.percentile(dt, 95)), 3),
                   "leader_age_median_s": round(float(np.median(age)), 3), "leader_age_p95_s": round(float(np.percentile(age, 95)), 3),
                   "leader_age_max_s": round(float(age.max()), 3)},
        "min_leader_distance_mm": round(float(np.min(np.hypot(lx - x, ly - y)))),
        "final_s_mm": round(float(s[-1])),
        "final_heading_deg": round(math.degrees(h[-1]), 1),
    }


def leader(run: dict, route: TurnPath) -> dict:
    rows = read(run["folder"] / "leader.csv", "control")
    t = np.array([f(r, "elapsed_s") for r in rows]); t -= t[0]
    x = np.array([f(r, "status_x_mm") for r in rows]); y = np.array([f(r, "status_y_mm") for r in rows])
    hd = np.array([f(r, "status_heading_deg") for r in rows])
    proj = [route.project(xi, yi, 0.0) for xi, yi in zip(x, y)]
    ey = np.array([p[1] for p in proj])
    epsi = np.array([((route.pose_at_s(p[0], 0.0)[2] - h + 180) % 360) - 180 for p, h in zip(proj, hd)])
    return {"series": {"t": t.tolist(), "s": [p[0] for p in proj], "e_y": ey.tolist()},
            "all": {"e_y": stats(ey), "e_psi": stats(epsi)}, "e_y_period_s": dominant_period(t, ey),
            "e_psi_period_s": dominant_period(t, epsi), "duration_s": round(float(t[-1]), 1)}


def main() -> None:
    out = {}
    for label, run in RUNS.items():
        route = TurnPath(TurnPathReference(STRAIGHT, RADIUS, run["exit"], 0, 0, 0))
        out[label] = {"L": leader(run, route), **{n: follower(run, n, route) for n in LANES}}
        out[label]["F1-F2_note"] = "n = 1 run per condition; F1 and F2 are two lanes, not repeats"
    slim = {k: {c: {kk: vv for kk, vv in v.items() if kk != "series"} if isinstance(v, dict) else v
                for c, v in r.items()} for k, r in out.items()}
    (HERE / "metrics.json").write_text(json.dumps(slim, indent=2, ensure_ascii=False) + "\n")
    (HERE / "series.json").write_text(json.dumps({k: {c: v["series"] for c, v in r.items() if isinstance(v, dict)}
                                                  for k, r in out.items()}) + "\n")
    for k, r in slim.items():
        print("==", k, "| Leader e_y", r["L"]["all"]["e_y"], "e_psi", r["L"]["all"]["e_psi"], "period", r["L"]["e_y_period_s"], r["L"]["e_psi_period_s"])
        for n in LANES:
            a = r[n]
            print(f"  {n} e_y {a['all']['e_y']}\n     e_s {a['all']['e_s']}\n     e_psi {a['all']['e_psi']}\n"
                  f"     period {a['e_y_period_s']} smooth {a['smoothness']} act {a['actuation']}\n     timing {a['timing']} "
                  f"min_dist {a['min_leader_distance_mm']} final_s {a['final_s_mm']} final_h {a['final_heading_deg']}")
            for sg, v in a["segments"].items():
                print(f"     [{sg}] len {v['length_mm']} e_y rms/max {v['e_y']['rms']}/{v['e_y']['max_abs']}  e_s rms/max {v['e_s']['rms']}/{v['e_s']['max_abs']}  e_psi rms/max {v['e_psi']['rms']}/{v['e_psi']['max_abs']}")


if __name__ == "__main__":
    main()
