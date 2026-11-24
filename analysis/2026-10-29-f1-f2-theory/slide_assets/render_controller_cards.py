"""Render slide-ready summaries of the two 2026-10-29 follower controllers."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyBboxPatch


HERE = Path(__file__).resolve().parent
CHINESE = FontProperties(fname="/System/Library/Fonts/STHeiti Medium.ttc")
INK = "#192638"
MUTED = "#526174"
OLD = "#2476B7"
NEW = "#CF6A24"


def panel(ax, y: float, height: float, label: str, equations: list[str], accent: str) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (0.055, y), 0.89, height,
            boxstyle="round,pad=0.012,rounding_size=0.018",
            linewidth=1.5, edgecolor="#D8E0E9", facecolor="white",
            transform=ax.transAxes,
        )
    )
    ax.text(0.078, y + height - 0.048, label, color=accent, fontsize=20,
            fontproperties=CHINESE, transform=ax.transAxes, va="center")
    top = y + height - 0.106
    gap = 0.067 if len(equations) == 2 else 0.052
    for i, equation in enumerate(equations):
        ax.text(0.085, top - i * gap, equation, color=INK, fontsize=25,
                transform=ax.transAxes, va="center")


def card(name: str, title: str, subtitle: str, accent: str,
         sections: list[tuple[str, list[str]]], footer: str) -> None:
    fig = plt.figure(figsize=(12, 7.5), dpi=200, facecolor="#F7F9FC")
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_axis_off()
    ax.add_patch(plt.Rectangle((0, 0.985), 1, 0.015, color=accent,
                               transform=ax.transAxes))
    ax.text(0.055, 0.938, title, fontsize=35, fontproperties=CHINESE,
            color=INK, transform=ax.transAxes, va="center")
    ax.text(0.055, 0.883, subtitle, fontsize=18, color=MUTED,
            transform=ax.transAxes, va="center")
    for (label, equations), (y, height) in zip(
        sections, ((0.623, 0.207), (0.354, 0.225), (0.085, 0.225)), strict=True
    ):
        panel(ax, y, height, label, equations, accent)
    ax.text(0.055, 0.034, footer, fontsize=16, fontproperties=CHINESE,
            color=MUTED, transform=ax.transAxes, va="center")
    fig.savefig(HERE / name, dpi=200, facecolor=fig.get_facecolor())
    plt.close(fig)


card(
    "old_body_relative_controller.png",
    "舊版：追隨 Leader 車身相對目標",
    "F1 in 2026-10-29 run  |  target rotates with Leader heading",
    OLD,
    [
        ("目標與誤差", [
            r"$p_F^*=p_L+R(\psi_L)\,(-400,+400)^T$",
            r"$(e_x,e_y)^T=R(-\psi_L)(p_F^*-p_F),\quad e_\psi=\mathrm{wrap}(\psi_L-\psi_F)$",
        ]),
        ("前進速度", [
            r"$k(\rho)=0.05+0.21\exp[-(\rho/120)^2],\quad \rho=\|p_F^*-p_F\|$",
            r"$v_F\simeq\mathrm{clip}\!\left[(v_L+k(\rho)e_x)\max(0.20,\cos e_\psi),\,0,75\right]$",
        ]),
        ("轉向角速度", [
            r"$\alpha=\arctan\!\left(\frac{e_y}{300+\max(e_x,0)}\right)$",
            r"$\omega_F=\mathrm{clip}(\bar\omega_L+0.60\alpha+0.50e_\psi,\,-0.25,0.25)$",
        ]),
    ],
    "速度式為主控制結構；實際程式另有限制追趕量、輪速與命令變化。角度用 rad，距離用 mm。",
)

card(
    "new_path_controller.png",
    "新版：追隨自己的規劃車道",
    "F2 in 2026-10-29 run  |  target follows route progress",
    NEW,
    [
        ("目標與誤差", [
            r"$P_{F2}(s)=P_L(s)+(0,-400)^T,\quad p_F^*=P_{F2}(s_L-400)$",
            r"$e_s=(s_L-400)-s_F,\quad e_\psi=\mathrm{wrap}(\psi_{P}(s_F)-\psi_F)$",
        ]),
        ("前進速度", [
            r"$v_{\mathrm{req}}=\max[0,\,v_L+\mathrm{clip}(0.18e_s,-25,18)]$",
            r"$v_F=\mathrm{clip}\!\left[\min(v_{\mathrm{req}},v_\kappa)\max(0.20,\cos e_\psi),\,0,75\right]$",
        ]),
        ("轉向角速度", [
            r"$\alpha=\arctan(e_y/300),\quad v_\kappa=\min\!\left(0.25/|\kappa|,\,75/(1+65|\kappa|)\right)$",
            r"$\omega_F=\mathrm{clip}(v_F\kappa+0.60\alpha+0.50e_\psi,\,-0.25,0.25)$",
        ]),
    ],
    "sF 是車在自身車道的投影進度；κ 為該處曲率。κ＝0 時不套用曲率限速。",
)
