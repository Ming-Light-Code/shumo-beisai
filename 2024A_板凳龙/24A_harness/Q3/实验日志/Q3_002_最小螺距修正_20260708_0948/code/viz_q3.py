"""Q3 可视化：最小螺距临界构型 + 可行性曲线 m(p)。依据 viz-standard skill。"""
import os
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solve_q3 as mod

FONT_NOTE = "字体回退 SimHei"
mpl.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['axes.unicode_minus'] = False
PALETTE = ["#2E86C1", "#E74C3C", "#27AE60", "#F39C12", "#8E44AD", "#16A085", "#7F8C8D"]
mpl.rcParams['axes.prop_cycle'] = mpl.cycler(color=PALETTE)
mpl.rcParams.update({
    'font.size': 10, 'axes.titlesize': 14, 'axes.labelsize': 12,
    'xtick.labelsize': 10, 'ytick.labelsize': 10, 'legend.fontsize': 10,
    'figure.dpi': 150, 'savefig.dpi': 300, 'savefig.bbox': 'tight',
    'axes.grid': True, 'grid.alpha': 0.3,
})

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "outputs")


def rect_corners(c, u, n, hl, hw):
    return np.array([c + hl * u + hw * n, c + hl * u - hw * n,
                     c - hl * u - hw * n, c - hl * u + hw * n])


def fig1_critical():
    cfg = np.load(os.path.join(OUT, "q3_critical_config.npz"))
    x, y = cfg["x"], cfg["y"]            # 临界接触时刻构型
    b_min = float(cfg["b_min"]); p_min = float(cfg["p_min"])
    R = float(cfg["R_bound"]); pair = cfg["pair"]

    th = np.linspace(0, 14 * 2 * np.pi, 4000)
    sx, sy = b_min * th * np.cos(th), b_min * th * np.sin(th)
    C, U, N = mod.benches_from_xy(x, y)

    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    ax = axes[0]
    ax.plot(sx, sy, color="#B0B0B0", lw=0.5, alpha=0.7, label="盘入螺线")
    ax.plot(x, y, '-', color="#2E86C1", lw=0.9, label="板凳龙")
    ax.add_patch(Circle((0, 0), R, fill=False, edgecolor="#8E44AD", lw=1.8, ls='--', label="调头空间边界 r=4.5m"))
    ax.plot(x[0], y[0], '*', color="#E74C3C", ms=13, label="龙头前把手")
    i, j = int(pair[0]) - 1, int(pair[1]) - 1
    for idx, color in [(i, "#E74C3C"), (j, "#F39C12")]:
        ax.add_patch(Polygon(rect_corners(C[idx], U[idx], N[idx], mod.HL[idx], mod.HW),
                             closed=True, facecolor=color, alpha=0.55, edgecolor='k', lw=1.0))
    ax.set_aspect('equal'); ax.set_xlim(-14, 14); ax.set_ylim(-14, 14)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"最小螺距临界构型 p_min={p_min:.6f} m")
    ax.legend(loc='upper right', fontsize=8)

    ax2 = axes[1]
    cx = 0.5 * (C[i][0] + C[j][0]); cy = 0.5 * (C[i][1] + C[j][1])
    for idx, color, lab in [(i, "#E74C3C", f"板凳{int(pair[0])}(龙头)"),
                            (j, "#F39C12", f"板凳{int(pair[1])}(第{int(pair[1])-1}节龙身)")]:
        ax2.add_patch(Polygon(rect_corners(C[idx], U[idx], N[idx], mod.HL[idx], mod.HW),
                              closed=True, facecolor=color, alpha=0.55, edgecolor='k', lw=1.4, label=lab))
    m = 0.6
    ax2.set_xlim(cx - m, cx + m); ax2.set_ylim(cy - m, cy + m); ax2.set_aspect('equal')
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
    ax2.set_title("临界接触对局部放大（恰好相切）")
    ax2.legend(fontsize=9)

    fig.suptitle(f"图3-1  Q3 最小螺距临界构型（{FONT_NOTE}）", fontsize=14)
    p = os.path.join(OUT, "fig3-1_最小螺距临界构型.png")
    fig.savefig(p); fig.savefig(p.replace(".png", ".pdf")); plt.close(fig)
    print("[fig]", p)


def fig2_feasibility():
    res = np.load(os.path.join(OUT, "q3_result.npz"))
    ps, mvals = res["ps"], res["mvals"]
    p_min = float(res["p_min"])
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.plot(ps, mvals, 'o-', color="#2E86C1", lw=1.4, label="盘入到边界最紧间隙 m(p)")
    ax.axhline(0, color="#7F8C8D", lw=1, ls='--')
    ax.axvline(p_min, color="#E74C3C", lw=1.3, ls=':', label=f"最小螺距 p_min={p_min:.6f} m")
    ax.plot([p_min], [0], 'o', color="#E74C3C", ms=9)
    ax.fill_between(ps, mvals, 0, where=(mvals < 0), color="#E74C3C", alpha=0.12)
    ax.annotate("不可行\n(盘入到边界前已碰撞)", xy=(ps[1], mvals[1]), fontsize=10, color="#E74C3C")
    ax.annotate("可行", xy=(ps[-2], mvals[-2]), fontsize=10, color="#27AE60")
    ax.set_xlabel("螺距 p (m)")
    ax.set_ylabel("盘入全程最紧间隙 m (m)")
    ax.set_title("图3-2  螺距可行性曲线 m(p)（m≥0 即可盘入到边界）")
    ax.legend()
    p = os.path.join(OUT, "fig3-2_可行性曲线.png")
    fig.savefig(p); fig.savefig(p.replace(".png", ".pdf")); plt.close(fig)
    print("[fig]", p)


if __name__ == "__main__":
    fig1_critical()
    fig2_feasibility()
    print("[done] figures ->", OUT)
