#!/usr/bin/env python3
"""Presentation figure: breadcrumb track, smoothing window and tangent vs analytic.

Data: 2026-11-24 08:40:14Z route-D run.  The Leader poses are the ones F1
actually received (f1.csv leader_* columns); F2 is matched to the same Leader
packet.  All positions are on-board odometry + MPU estimates.

Top left : breadcrumb -- Leader points recorded by the followers, the smoothing
           window around each follower's reference point, fitted tangent.
Top right: analytic route given in advance -- same moment, exact tangent.
Bottom   : purity along the turn -- tangent-angle error and F1 lane curvature
           for raw neighbouring points, 160 mm and 300 mm windows, analytic.
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
RUN = HERE.parent / "real_run_084014Z"
sys.path.insert(0, str(ROOT / "pi_rover"))
from breadcrumb_path import BreadcrumbPath  # noqa: E402
from turn_path import TurnPath, TurnPathReference  # noqa: E402

FONT = "/System/Library/Fonts/STHeiti Medium.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
plt.rcParams["axes.unicode_minus"] = False

ROUTE = TurnPath(TurnPathReference(600.0, 800.0, 400.0, 0, 0, 0))
LANES = {"F1": 400.0, "F2": -400.0}
COLORS = {"L": "#374151", "F1": "#2563eb", "F2": "#ea580c"}
WINDOW = 300.0          # current default
MOMENT_S_L = 1490.0     # Leader progress along its track for the snapshot
BODY = np.array([[100, 75], [100, -75], [-150, -75], [-150, 75]], float)


def rows(name: str) -> list[dict]:
    with open(RUN / f"{name}.csv") as fh:
        return [r for r in csv.DictReader(fh) if r["event"].startswith("control_")]


def leader_pose(r: dict) -> tuple[float, float, float]:
    return float(r["leader_x_mm"]), float(r["leader_y_mm"]), float(r["leader_heading_deg"])


def corners(x, y, h_deg):
    h = math.radians(h_deg)
    rot = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    return BODY @ rot.T + [x, y]


def draw_body(ax, pose, color, label=None):
    c = corners(*pose)
    ax.fill(c[:, 0], c[:, 1], color=color, alpha=0.22, lw=0)
    ax.plot(*np.vstack([c, c[:1]]).T, "--", color=color, lw=0.9)
    ax.plot(c[:2, 0], c[:2, 1], "-", color=color, lw=2.2)
    ax.plot(*pose[:2], "o", mfc="white", mec=color, ms=7, mew=1.6, zorder=6)
    ax.plot(*pose[:2], "o", color=color, ms=2.5, zorder=7)
    if label:
        ax.annotate(label, pose[:2], xytext=(10, -16), textcoords="offset points", color=color,
                    fontsize=12, weight="bold")


def tangent_segment(ax, x, y, heading_deg, color, half=230, **kw):
    h = math.radians(heading_deg)
    ax.plot([x - half * math.cos(h), x + half * math.cos(h)],
            [y - half * math.sin(h), y + half * math.sin(h)], color=color, **kw)


def build_path(f1_rows: list[dict], upto: int, window: float) -> BreadcrumbPath:
    path = BreadcrumbPath(smoothing_window_mm=window, keep_behind_mm=1e9)
    last = None
    for r in f1_rows[:upto + 1]:
        pose = leader_pose(r)
        if pose != last:
            path.observe(*pose)
            last = pose
    return path


def main() -> None:
    f1, f2 = rows("f1"), rows("f2")
    k = min(range(len(f1)), key=lambda i: abs(float(f1[i]["control_leader_progress_s_mm"]) - MOMENT_S_L))
    snap = f1[k]
    lpose = leader_pose(snap)
    k2 = min(range(len(f2)), key=lambda i: math.dist(leader_pose(f2[i])[:2], lpose[:2]))
    poses = {"L": lpose,
             "F1": (float(snap["follower_x_mm"]) - 400, float(snap["follower_y_mm"]) + 400, float(snap["follower_heading_deg"])),
             "F2": (float(f2[k2]["follower_x_mm"]) - 400, float(f2[k2]["follower_y_mm"]) - 400, float(f2[k2]["follower_heading_deg"]))}
    path = build_path(f1, k, WINDOW)
    t_snap = float(snap["elapsed_s"]) - float(f1[0]["elapsed_s"])

    fig = plt.figure(figsize=(16, 11.5))
    g = fig.add_gridspec(2, 2, height_ratios=[1.45, 1], hspace=0.28, wspace=0.12)
    xs = [p[0] for p in poses.values()]
    ys = [p[1] for p in poses.values()]
    view = (min(xs) - 300, max(xs) + 300, min(ys) - 470, max(ys) + 220)

    # ---------------- top left: breadcrumb
    ax = fig.add_subplot(g[0, 0])
    s_pts, x_pts, y_pts, psi_pts = path._s, path._x, path._y, path._psi
    ax.plot(x_pts, y_pts, ".", color=COLORS["L"], ms=4, label="L 被記錄的點（每 20 mm）")
    for x, y, p in zip(x_pts, y_pts, psi_pts):     # raw MPU heading tick at each point
        ax.plot([x, x + 55 * math.cos(p)], [y, y + 55 * math.sin(p)], color="#6b7280", lw=0.8)
    s_refs = {name: path.project(*poses[name][:2], d)[0] for name, d in LANES.items()}
    s_mid = sum(s_refs.values()) / 2
    lo, hi = s_mid - WINDOW / 2, s_mid + WINDOW / 2
    m = [(lo <= s <= hi) for s in s_pts]
    ax.plot(np.array(x_pts)[m], np.array(y_pts)[m], "-", color="#059669", lw=18, alpha=0.22,
            solid_capstyle="butt", label=f"平滑視窗 {WINDOW:.0f} mm（F1、F2 參考同一段 L 軌跡）")
    for edge in (lo, hi):
        ex, ey, eh, _ = path.pose_at_s(edge, 0.0)
        n = math.radians(eh) + math.pi / 2
        ax.plot([ex - 55 * math.cos(n), ex + 55 * math.cos(n)], [ey - 55 * math.sin(n), ey + 55 * math.sin(n)],
                color="#059669", lw=2.0)
    ax.annotate(f"視窗 {WINDOW:.0f} mm", path.pose_at_s(hi, 0.0)[:2], xytext=(12, -4), textcoords="offset points",
                color="#059669", fontsize=11)
    for name, d in LANES.items():
        lane = [path.pose_at_s(s, d)[:2] for s in np.arange(s_pts[0], path.end_s_mm, 10)]
        ax.plot(*np.array(lane).T, ":", color=COLORS[name], lw=1.4, label=f"{name} 車道（L 軌跡平移 {d:+.0f} mm）")
        s_f = s_refs[name]
        cx, cy, ch, _ = path.pose_at_s(s_f, 0.0)
        tangent_segment(ax, cx, cy, ch, COLORS[name], lw=2.2)
        lx, ly, lh, _ = path.pose_at_s(s_f, d)
        tangent_segment(ax, lx, ly, lh, COLORS[name], lw=2.2, ls="--")
        ax.annotate("", (lx, ly), (cx, cy), arrowprops=dict(arrowstyle="->", color=COLORS[name], lw=1.0, alpha=0.7))
        ax.plot(cx, cy, "D", color=COLORS[name], ms=6, zorder=8)
    for name in ("L", "F1", "F2"):
        draw_body(ax, poses[name], COLORS[name], "Leader" if name == "L" else name)
    ax.set_title(f"Breadcrumb：F1/F2 記錄 L 走過的點，視窗內擬合切線\n"
                 f"（2026-11-24 實跑 t ≈ {t_snap:.0f} s，車上估計；灰色短線 = 每點 MPU 原始航向）", fontsize=13)
    ax.set_xlim(view[:2]); ax.set_ylim(view[2:]); ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.legend(loc="lower left", fontsize=9, framealpha=0.9)

    # ---------------- top right: analytic
    ax = fig.add_subplot(g[0, 1])
    ss = np.linspace(0, ROUTE.end_s_mm, 600)
    ax.plot(*np.array([ROUTE.pose_at_s(s, 0)[:2] for s in ss]).T, "-", color=COLORS["L"], lw=1.6,
            label="L 完整路徑（事先告知的公式）")
    for name, d in LANES.items():
        ax.plot(*np.array([ROUTE.pose_at_s(s, d)[:2] for s in ss]).T, ":", color=COLORS[name], lw=1.4,
                label=f"{name} 車道（理想路徑平移 {d:+.0f} mm）")
        s_f, _ = ROUTE.project(*poses[name][:2], d)
        cx, cy, ch, _ = ROUTE.pose_at_s(s_f, 0.0)
        tangent_segment(ax, cx, cy, ch, COLORS[name], lw=2.2)
        lx, ly, lh, _ = ROUTE.pose_at_s(s_f, d)
        tangent_segment(ax, lx, ly, lh, COLORS[name], lw=2.2, ls="--")
        ax.annotate("", (lx, ly), (cx, cy), arrowprops=dict(arrowstyle="->", color=COLORS[name], lw=1.0, alpha=0.7))
        ax.plot(cx, cy, "D", color=COLORS[name], ms=6, zorder=8)
    for name in ("L", "F1", "F2"):
        draw_body(ax, poses[name], COLORS[name], "Leader" if name == "L" else name)
    ax.set_title("完整路徑：一開始就告知整條路線，切線直接由公式算出\n（同一時刻、同樣的車位置）", fontsize=13)
    ax.set_xlim(view[:2]); ax.set_ylim(view[2:]); ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlabel("x (mm)")
    ax.legend(loc="lower left", fontsize=9, framealpha=0.9)

    # ---------------- bottom: purity along the whole received track
    full = build_path(f1, len(f1) - 1, WINDOW)
    full160 = build_path(f1, len(f1) - 1, 160.0)
    s_all = np.array(full._s)
    ideal_s = np.array([ROUTE.project(x, y, 0.0)[0] for x, y in zip(full._x, full._y)])
    ideal_h = np.array([ROUTE.pose_at_s(s, 0.0)[2] for s in ideal_s])
    raw_h = np.degrees(np.array(full._psi))
    fit300 = np.array([full.pose_at_s(s, 0.0)[2] for s in s_all])
    fit160 = np.array([full160.pose_at_s(s, 0.0)[2] for s in s_all])
    turn = (ideal_s > 650) & (ideal_s < 1800)   # inside the arc, away from the two curvature steps

    def rms(e):
        return float(np.sqrt(np.mean(np.asarray(e)[turn] ** 2)))

    ax = fig.add_subplot(g[1, 0])
    ax.axvspan(600, 600 + 800 * math.pi / 2, color="#fde68a", alpha=0.25, lw=0)
    ax.plot(s_all, raw_h - ideal_h, ".", color="#9ca3af", ms=3.5, label=f"每點原始航向  彎道 RMS {rms(raw_h - ideal_h):.2f}°")
    ax.plot(s_all, fit160 - ideal_h, "-", color="#7c3aed", lw=1.2, label=f"視窗 160 mm（11/24 實跑）  彎道 RMS {rms(fit160 - ideal_h):.2f}°")
    ax.plot(s_all, fit300 - ideal_h, "-", color="#059669", lw=2.0, label=f"視窗 300 mm（現在）  彎道 RMS {rms(fit300 - ideal_h):.2f}°")
    ax.axhline(0, color="black", lw=1.2, ls="--", label="完整路徑（誤差 0）")
    ax.axvline(MOMENT_S_L - 400, color=COLORS["F1"], lw=0.8, ls=":")
    ax.set_xlabel("沿 L 軌跡的距離 s (mm)；黃底 = 彎道")
    ax.set_ylabel("切線角 − 完整路徑切線角 (°)")
    ax.set_title("切線純淨度：和事先告知的完整路徑相比差幾度", fontsize=12)
    ax.grid(alpha=0.3); ax.legend(fontsize=9, loc="lower left")

    ax = fig.add_subplot(g[1, 1])
    d = LANES["F1"]
    def lane_k(kc):
        return np.asarray(kc) / np.maximum(0.2, 1 - d * np.asarray(kc)) * 1000
    raw_kc = np.gradient(np.unwrap(np.array(full._psi)), s_all)
    k300 = [full.centre_curvature_at_s(s) for s in s_all]
    k160 = [full160.centre_curvature_at_s(s) for s in s_all]
    kid = [ROUTE.pose_at_s(s, d)[3] * 1000 for s in ideal_s]
    ax.axvspan(600, 600 + 800 * math.pi / 2, color="#fde68a", alpha=0.25, lw=0)
    ax.plot(s_all, np.clip(lane_k(raw_kc), -6, 12), ".", color="#9ca3af", ms=3.5,
            label=f"相鄰點直接差分  RMS 誤差 {rms(lane_k(raw_kc) - kid):.1f} /m")
    ax.plot(s_all, lane_k(k160), "-", color="#7c3aed", lw=1.2, label=f"視窗 160 mm  RMS 誤差 {rms(lane_k(k160) - kid):.2f} /m")
    ax.plot(s_all, lane_k(k300), "-", color="#059669", lw=2.0, label=f"視窗 300 mm  RMS 誤差 {rms(lane_k(k300) - kid):.2f} /m")
    ax.plot(s_all, kid, "--", color="black", lw=1.4, label="完整路徑（真值 2.5 /m）")
    ax.axvline(MOMENT_S_L - 400, color=COLORS["F1"], lw=0.8, ls=":")
    ax.set_ylim(-6, 12)
    ax.set_xlabel("沿 L 軌跡的距離 s (mm)；藍色虛線 = 上圖 F1 所在位置")
    ax.set_ylabel("F1 車道曲率 (1/m)")
    ax.set_title("曲率純淨度（F1 內側車道，轉彎前饋用的就是它）", fontsize=12)
    ax.grid(alpha=0.3); ax.legend(fontsize=9, loc="upper left")

    fig.suptitle("Breadcrumb（記錄 L 軌跡＋平滑視窗）vs 完整路徑（事先告知）— 路徑 D，2026-11-24 實跑資料",
                 fontsize=15, weight="bold", y=0.995)
    out = HERE / "breadcrumb_window_tangent.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(out, "snapshot t=%.1f s  s_L=%s" % (t_snap, snap["control_leader_progress_s_mm"]))
    print("turn-section RMS: heading raw %.2f, 160 %.2f, 300 %.2f deg" % (rms(raw_h - ideal_h), rms(fit160 - ideal_h), rms(fit300 - ideal_h)))
    print("F1 lane curvature RMS err: raw %.2f, 160 %.2f, 300 %.2f /m" % (rms(lane_k(raw_kc) - kid), rms(lane_k(k160) - kid), rms(lane_k(k300) - kid)))


if __name__ == "__main__":
    main()
