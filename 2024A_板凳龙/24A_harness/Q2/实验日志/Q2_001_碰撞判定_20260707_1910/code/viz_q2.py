"""Q2 可视化：终止时刻矩形快照 + 全局间隙 G(t) 演化曲线。依据 viz-standard skill。"""
import os
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solve_q2 as m

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


def spiral_curve(theta_max):
    th = np.linspace(0, theta_max, 4000)
    r = m.B * th
    return r * np.cos(th), r * np.sin(th)


def rect_corners(c, u, n, hl, hw):
    return np.array([
        c + hl * u + hw * n,
        c + hl * u - hw * n,
        c - hl * u - hw * n,
        c - hl * u + hw * n,
    ])


def fig1_termination_snapshot():
    data = np.load(os.path.join(OUT, "termination_snapshot.npz"))
    theta0 = float(data["theta0"])
    t_star = m.arc_len(m.THETA0_INIT) - m.arc_len(theta0)
    x, y = data["x"], data["y"]
    _, pair = m.clearance(theta0)
    C, U, N = m.benches_from_xy(x, y)

    sx, sy = spiral_curve(20 * 2 * np.pi)

    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    ax = axes[0]
    ax.plot(sx, sy, color="#B0B0B0", lw=0.6, alpha=0.7, label="盘入螺线")
    ax.plot(x, y, '-', color="#2E86C1", lw=1.0, label="板凳龙")
    ax.plot(x[0], y[0], '*', color="#E74C3C", ms=14, label="龙头前把手")
    ax.plot(x[-1], y[-1], 's', color="#27AE60", ms=6, label="龙尾后把手")
    i, j = pair[0] - 1, pair[1] - 1
    for idx, color in [(i, "#E74C3C"), (j, "#F39C12")]:
        poly = rect_corners(C[idx], U[idx], N[idx], m.HL[idx], m.HW)
        ax.add_patch(Polygon(poly, closed=True, fill=True, facecolor=color, alpha=0.5, edgecolor='k', lw=1.2))
    ax.set_aspect('equal')
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"整体视图  t*={t_star:.2f}s")
    ax.legend(loc='upper right', fontsize=8)

    ax2 = axes[1]
    margin = 0.6
    cx = 0.5 * (C[i][0] + C[j][0])
    cy = 0.5 * (C[i][1] + C[j][1])
    for idx, color, lab in [(i, "#E74C3C", f"板凳{pair[0]}(龙头)"), (j, "#F39C12", f"板凳{pair[1]}(第{pair[1]-1}节龙身)")]:
        poly = rect_corners(C[idx], U[idx], N[idx], m.HL[idx], m.HW)
        ax2.add_patch(Polygon(poly, closed=True, fill=True, facecolor=color, alpha=0.5,
                               edgecolor='k', lw=1.5, label=lab))
    ax2.set_xlim(cx - margin, cx + margin)
    ax2.set_ylim(cy - margin, cy + margin)
    ax2.set_aspect('equal')
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.set_title("临界碰撞对局部放大（恰好相切）")
    ax2.legend(fontsize=9)

    fig.suptitle(f"图2-1  Q2 盘入终止时刻状态（{FONT_NOTE}）", fontsize=14)
    p = os.path.join(OUT, "fig2-1_终止时刻快照.png")
    fig.savefig(p)
    fig.savefig(p.replace(".png", ".pdf"))
    plt.close(fig)
    print("[fig]", p)


def fig2_clearance_curve():
    data = np.load(os.path.join(OUT, "coarse_scan.npz"))
    t, G = data["t"], data["G"]
    snap = np.load(os.path.join(OUT, "termination_snapshot.npz"))
    theta0_star = float(snap["theta0"])
    t_star = m.arc_len(m.THETA0_INIT) - m.arc_len(theta0_star)
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(t, G, '-', color="#2E86C1", lw=1.3, label="全局间隙 G(t)（全穷举候选对的最小分离裕度）")
    ax.axhline(0, color="#7F8C8D", lw=1, ls='--')
    ax.axvline(t_star, color="#E74C3C", lw=1.2, ls=':', label=f"终止时刻 t*={t_star:.3f} s")
    ax.plot([t_star], [0], 'o', color="#E74C3C", ms=8)
    ax.set_xlabel("时间 t (s)")
    ax.set_ylabel("全局间隙 G (m)")
    ax.set_title("图2-2  全局间隙 G(t) 随时间演化（G≤0 即发生碰撞）")
    ax.legend()
    p = os.path.join(OUT, "fig2-2_间隙演化曲线.png")
    fig.savefig(p)
    fig.savefig(p.replace(".png", ".pdf"))
    plt.close(fig)
    print("[fig]", p)


if __name__ == "__main__":
    fig1_termination_snapshot()
    fig2_clearance_curve()
    print("[done] figures ->", OUT)
