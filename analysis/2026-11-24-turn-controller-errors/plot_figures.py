#!/usr/bin/env python3
"""Two concise figures for the report (reads metrics.json / series.json).

fig1_error_definitions.png : what e_y, e_s, e_psi mean, and the route segments
fig2_controller_errors.png  : three turns, controller error under the
                              perfect-localisation assumption
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "pi_rover"))
from turn_path import TurnPath, TurnPathReference  # noqa: E402

FONT = "/System/Library/Fonts/STHeiti Medium.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({"font.family": font_manager.FontProperties(fname=FONT).get_name(),
                     "axes.unicode_minus": False, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#9ca3af", "axes.labelcolor": "#374151", "xtick.color": "#4b5563",
                     "ytick.color": "#4b5563", "grid.color": "#e5e7eb"})
INK, MUTED = "#111827", "#6b7280"
# Runs: validated categorical set (aqua, yellow, purple; CVD dE 9.1, normal 22.9) + line style as secondary code.
RUN_STYLE = {"B  11/18 30 mm/s 解析": ("#1baf7a", "-"),
             "D  11/18 38 mm/s 解析": ("#eda100", "--"),
             "D' 11/24 38 mm/s breadcrumb": ("#6250d6", ":")}
CAR = {"L": "#374151", "F1": "#2563eb", "F2": "#ea580c"}   # project convention (PLOT_CONVENTIONS.md)
STRAIGHT, RADIUS = 600.0, 800.0
ARC_END = STRAIGHT + RADIUS * math.pi / 2
SEGS = [("直線", -400, STRAIGHT), ("進彎", STRAIGHT, STRAIGHT + 300), ("穩定彎道", STRAIGHT + 300, ARC_END)]
BODY = np.array([[100, 75], [100, -75], [-150, -75], [-150, 75]], float)


def body(ax, x, y, h, color, alpha=0.18):
    hr = math.radians(h)
    rot = np.array([[math.cos(hr), -math.sin(hr)], [math.sin(hr), math.cos(hr)]])
    c = BODY @ rot.T + [x, y]
    ax.fill(c[:, 0], c[:, 1], color=color, alpha=alpha, lw=0)
    ax.plot(*np.vstack([c, c[:1]]).T, "--", color=color, lw=0.9)
    ax.plot(c[:2, 0], c[:2, 1], "-", color=color, lw=2.2)
    ax.plot(x, y, "o", mfc="white", mec=color, ms=7, mew=1.6, zorder=6)


def fig1() -> None:
    route = TurnPath(TurnPathReference(STRAIGHT, RADIUS, 400.0, 0, 0, 0))
    fig, (a, b) = plt.subplots(1, 2, figsize=(15, 6.6), gridspec_kw={"width_ratios": [1.15, 1]})

    # (a) error definitions: local, not-to-scale schematic
    R0, cx0, cy0 = 1500.0, 0.0, 1500.0           # gentle left-curving lane (centre above), like the route
    def lane(th):                                   # th: turned angle, travel to the right then bending left
        return cx0 + R0 * math.sin(th), cy0 - R0 * math.cos(th), math.degrees(th)
    ths = np.linspace(-0.32, 0.36, 200)
    a.plot(*np.array([lane(th)[:2] for th in ths]).T, ":", color=CAR["F1"], lw=2)
    a.text(*lane(-0.31)[:2], "參考車道 ", color=CAR["F1"], fontsize=12, ha="right", va="center")
    th_p, th_t = 0.0, 0.17
    px, py, ph = lane(th_p)
    tx, ty, _ = lane(th_t)
    hr = math.radians(ph); nx, ny = -math.sin(hr), math.cos(hr)
    ey = 150.0
    fx, fy, fh = px + ey * nx, py + ey * ny, ph + 18
    # tangent at P and its parallel through F
    a.plot([px - 380, px + 420], [py, py], color=MUTED, lw=1.2)
    a.text(px - 385, py, "車道切線 ", color=MUTED, fontsize=11, ha="right", va="center")
    a.plot([fx, fx + 330], [fy, fy], color=MUTED, lw=1.1, ls="--")
    fr = math.radians(fh)
    a.plot([fx, fx + 330 * math.cos(fr)], [fy, fy + 330 * math.sin(fr)], color=CAR["F1"], lw=1.8)
    ang = np.linspace(0, fr, 20)
    a.plot(fx + 250 * np.cos(ang), fy + 250 * np.sin(ang), color="#7c3aed", lw=2.4)
    a.text(fx + 275 * math.cos(fr / 2), fy + 275 * math.sin(fr / 2), "e_ψ 航向誤差", color="#7c3aed", fontsize=13, va="center")
    # small car (schematic size)
    small = np.array([[56, 34], [56, -34], [-84, -34], [-84, 34]], float)
    rot = np.array([[math.cos(fr), -math.sin(fr)], [math.sin(fr), math.cos(fr)]])
    c = small @ rot.T + [fx, fy]
    a.fill(c[:, 0], c[:, 1], color=CAR["F1"], alpha=0.18, lw=0)
    a.plot(*np.vstack([c, c[:1]]).T, "--", color=CAR["F1"], lw=0.9)
    a.plot(c[:2, 0], c[:2, 1], "-", color=CAR["F1"], lw=2.2)
    a.plot(fx, fy, "o", mfc="white", mec=CAR["F1"], ms=8, mew=1.8, zorder=6)
    a.text(fx - 95, fy + 45, "F1", color=CAR["F1"], fontsize=13, ha="right")
    # e_y
    a.annotate("", (fx, fy), (px, py), arrowprops=dict(arrowstyle="<->", color="#dc2626", lw=2.2, shrinkA=0, shrinkB=0))
    a.text(px - 20, py + ey / 2, "e_y 橫向誤差", color="#dc2626", fontsize=13, ha="right", va="center")
    a.plot(px, py, "D", color=CAR["F1"], ms=8, zorder=7)
    a.text(px, py - 45, "車道上最近點", color=CAR["F1"], fontsize=11, ha="center", va="top")
    # e_s along the lane, drawn just below it
    arc = np.array([lane(th)[:2] for th in np.linspace(th_p, th_t, 40)])
    a.plot(arc[:, 0], arc[:, 1] - 95, color="#059669", lw=3)
    for end in (arc[0], arc[-1]):
        a.plot([end[0], end[0]], [end[1] - 115, end[1] - 75], color="#059669", lw=2)
    mid = arc[len(arc) // 2]
    a.text(mid[0], mid[1] - 140, "e_s 前後間距誤差（沿車道量）", color="#059669", fontsize=13, ha="center", va="top")
    a.plot(tx, ty, "*", color=INK, ms=17, zorder=7)
    a.text(tx + 30, ty + 75, "目標點\n（L 的進度 − 400 mm）", color=INK, fontsize=11, va="bottom")
    a.set_xlim(-720, 820); a.set_ylim(-420, 520)
    a.set_aspect("equal"); a.axis("off")
    a.set_title("(a) 三種控制誤差（在車道座標裡量；示意圖，非比例）", fontsize=14, color=INK, loc="left")

    # (b) route + segments
    sa = np.linspace(-400, route.end_s_mm, 500)
    for lo, hi, shade, label in ((-400, STRAIGHT, "#f3f4f6", "直線"), (STRAIGHT, STRAIGHT + 300, "#fde68a", "進彎 0–300 mm"),
                                 (STRAIGHT + 300, ARC_END, "#fef3c7", "穩定彎道"), (ARC_END, route.end_s_mm, "#e5e7eb", "出彎")):
        seg = np.array([route.pose_at_s(s, 0)[:2] for s in sa if lo <= s <= hi])
        b.plot(seg[:, 0], seg[:, 1], color=shade, lw=34, solid_capstyle="butt", zorder=1)
        mid = route.pose_at_s((lo + hi) / 2, 0)
        off = {"直線": (0, -140), "進彎 0–300 mm": (40, -150), "穩定彎道": (150, -60), "出彎": (130, 0)}[label]
        b.text(mid[0] + off[0], mid[1] + off[1], label, fontsize=12, color=INK, ha="center")
    for name, d in (("L", 0.0), ("F1", 400.0), ("F2", -400.0)):
        b.plot(*np.array([route.pose_at_s(s, d)[:2] for s in sa]).T, "-" if name == "L" else ":", color=CAR[name], lw=1.6, zorder=2)
        sx, sy, sh, _ = route.pose_at_s(0 if name == "L" else -400, d)
        body(b, sx, sy, sh, CAR[name], 0.15)
        b.text(sx - 190, sy + (110 if name == "L" else 0), "Leader" if name == "L" else name, color=CAR[name], fontsize=12,
               ha="right" if name != "L" else "center", va="center")
    b.set_aspect("equal"); b.axis("off")
    b.set_title("(b) 路段劃分（依 Follower 在路線上的位置）\n路徑 D：直 600 → R800 左轉 90° → 直 400", fontsize=14, color=INK, loc="left")
    fig.tight_layout()
    fig.savefig(HERE / "fig1_error_definitions.png", dpi=150, bbox_inches="tight")


def fig2() -> None:
    m = json.loads((HERE / "metrics.json").read_text())
    ser = json.loads((HERE / "series.json").read_text())
    fig = plt.figure(figsize=(16, 10.5))
    g = fig.add_gridspec(2, 2, height_ratios=[1, 1.05], hspace=0.42, wspace=0.16)

    for i, name in enumerate(("F1", "F2")):
        ax = fig.add_subplot(g[0, i])
        for (label, lo, hi), shade in zip(SEGS, ("#f9fafb", "#fde68a", "#fef3c7")):
            ax.axvspan(lo, hi, color=shade, lw=0, zorder=0)
            ax.text((lo + hi) / 2, 36, label, ha="center", va="top", fontsize=10, color=MUTED)
        ax.axhline(0, color=MUTED, lw=0.8)
        for run, (col, ls) in RUN_STYLE.items():
            s, ey = np.array(ser[run][name]["s"]), np.array(ser[run][name]["e_y"])
            ax.plot(s, ey, ls, color=col, lw=2, label=run)
            nudge = {"B": 0, "D": 3.5, "D'": -3.5}[run.split()[0]]
            ax.text(s[-1] + 15, ey[-1] + nudge, run.split()[0], color=INK, fontsize=10, va="center")
        ax.set_xlim(-420, ARC_END + 120); ax.set_ylim(-38, 38)
        ax.set_xlabel(f"{name} 在路線上的位置 s (mm)")
        ax.set_ylabel("橫向誤差 e_y (mm)" if i == 0 else "")
        ax.set_title(f"({'ab'[i]}) {name}（{'內側 R400' if name == 'F1' else '外側 R1200'}）橫向誤差沿路線變化",
                     fontsize=13, color=INK, loc="left")
        ax.grid(axis="y", alpha=0.7)
        if i == 0:
            ax.legend(fontsize=9, loc="lower left", frameon=False)

    # (c) segment RMS dot plot
    ax = fig.add_subplot(g[1, 0])
    seg_names = ["直線", "進彎（前 300 mm）", "穩定彎道"]
    keys = ["直線", "進彎 0–300 mm", "穩定彎道"]
    for j, run in enumerate(RUN_STYLE):
        col, _ = RUN_STYLE[run]
        for k, seg in enumerate(keys):
            xbase = k + (j - 1) * 0.22
            for name, mk in (("F1", "o"), ("F2", "^")):
                v = m[run][name]["segments"][seg]["e_y"]["rms"]
                ax.plot(xbase, v, mk, color=col, ms=10, mec="white", mew=1.5, zorder=3)
            ax.plot([xbase, xbase], [m[run]["F1"]["segments"][seg]["e_y"]["rms"], m[run]["F2"]["segments"][seg]["e_y"]["rms"]],
                    color=col, lw=1.2, alpha=0.6, zorder=2)
        ax.plot([], [], "s", color=col, ms=9, label=run)
    lead = np.mean([m[run]["L"]["all"]["e_y"]["rms"] for run in RUN_STYLE])
    ax.axhline(lead, color=MUTED, lw=1, ls="--")
    ax.text(2.45, lead - 0.35, f"Leader 自己的 e_y RMS ≈ {lead:.0f} mm（三趟）", fontsize=10, color=MUTED, ha="right", va="top")
    ax.plot([], [], "o", color=MUTED, label="F1"); ax.plot([], [], "^", color=MUTED, label="F2")
    ax.set_xticks(range(3), seg_names); ax.set_ylim(0, 26)
    ax.set_ylabel("e_y RMS (mm)")
    ax.set_title("(c) 各路段橫向誤差 RMS：直線並不比彎道小", fontsize=13, color=INK, loc="left")
    ax.grid(axis="y", alpha=0.7)
    ax.legend(fontsize=9, ncol=2, loc="upper left", frameon=False)

    # (d) compact table: what the error is made of
    ax = fig.add_subplot(g[1, 1]); ax.axis("off")
    hdr = ["", "B 解析\n30 mm/s", "D 解析\n38 mm/s", "D' breadcrumb\n38 mm/s"]
    runs = list(RUN_STYLE)
    def pair(k1, k2, fmt="{:.0f}"):
        return [" / ".join(fmt.format(m[r][n]["all"][k1][k2]) for n in ("F1", "F2")) for r in runs]
    rows = [
        ["Leader e_y RMS (mm)"] + [f"{m[r]['L']['all']['e_y']['rms']:.1f}" for r in runs],
        ["F1 / F2 e_y RMS (mm)"] + pair("e_y", "rms"),
        ["F1 / F2 e_y 最大 (mm)"] + pair("e_y", "max_abs"),
        ["F1 / F2 e_s 最大 (mm)"] + pair("e_s", "max_abs"),
        ["F1 / F2 e_ψ RMS (°)"] + pair("e_psi", "rms", "{:.1f}"),
        ["擺動週期 L / F1 / F2 (s)"] + [f"{m[r]['L']['e_y_period_s']:.0f} / {m[r]['F1']['e_y_period_s']:.0f} / {m[r]['F2']['e_y_period_s']:.0f}" for r in runs],
        ["轉向偏差 ÷ 轉向命令（RMS）"] + [" / ".join(f"{m[r][n]['actuation']['yaw_rate_meas_minus_cmd_rms'] / m[r][n]['actuation']['yaw_rate_cmd_rms']:.2f}" for n in ("F1", "F2")) for r in runs],
        ["命令觸及限幅 (%)"] + [" / ".join(f"{m[r][n]['smoothness']['step_limit_hit_pct'] + m[r][n]['smoothness']['yaw_limit_hit_pct']:.1f}" for n in ("F1", "F2")) for r in runs],
        ["封包延遲 中位 / 最大 (ms)"] + [f"{1000 * m[r]['F1']['timing']['leader_age_median_s']:.0f} / {1000 * max(m[r][n]['timing']['leader_age_max_s'] for n in ('F1', 'F2')):.0f}" for r in runs],
    ]
    tab = ax.table(cellText=rows, colLabels=hdr, loc="center", cellLoc="center", colLoc="center",
                   colWidths=[0.39, 0.2, 0.2, 0.24])
    tab.auto_set_font_size(False); tab.set_fontsize(10.5); tab.scale(1, 2.1)
    for (r, c), cell in tab.get_celld().items():
        cell.set_edgecolor("#e5e7eb")
        if r == 0:
            cell.set_facecolor("#f3f4f6"); cell.set_text_props(color=INK)
        if c == 0 and r > 0:
            cell.set_text_props(ha="left", color="#374151"); cell._loc = "left"
        if r in (6, 7):
            cell.set_facecolor("#fef2f2")
    for c, run in enumerate(runs, start=1):
        tab[0, c].get_text().set_color(INK)
        tab[0, c].set_edgecolor(RUN_STYLE[run][0]); tab[0, c].set_linewidth(2.5)
    ax.set_title("(d) 誤差組成（假設定位完全準確；每個條件只有 1 趟）", fontsize=13, color=INK, loc="left")

    fig.suptitle("最近三次三車左轉：控制器誤差（車上位置當真值）", fontsize=16, color=INK, x=0.06, ha="left", y=0.995)
    fig.savefig(HERE / "fig2_controller_errors.png", dpi=150, bbox_inches="tight")


if __name__ == "__main__":
    fig1()
    fig2()
