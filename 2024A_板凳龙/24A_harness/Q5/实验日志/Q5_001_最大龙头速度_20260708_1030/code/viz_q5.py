"""Q5 可视化：全局放大系数扫描曲线 + 临界构型逐节点K_i + 缩放后速度剖面。依据 viz-standard。"""
import os
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geometry_q4 as g
from solve_q5 import amplification, chain_positions, V_CAP

mpl.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['axes.unicode_minus'] = False
PALETTE = ["#2E86C1", "#E74C3C", "#27AE60", "#F39C12", "#8E44AD", "#16A085", "#7F8C8D"]
mpl.rcParams['axes.prop_cycle'] = mpl.cycler(color=PALETTE)
mpl.rcParams.update({
    'font.size': 10, 'axes.titlesize': 14, 'axes.labelsize': 12,
    'xtick.labelsize': 10, 'ytick.labelsize': 10, 'legend.fontsize': 9,
    'figure.dpi': 150, 'savefig.dpi': 300, 'savefig.bbox': 'tight',
    'axes.grid': True, 'grid.alpha': 0.3,
})

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "outputs")


def fig1_scan_curve():
    d = np.load(os.path.join(OUT, "q5_scan.npz"))
    grid, M = d["grid"], d["M"]
    grid2, M2 = d["grid2"], d["M2"]
    s_star, k_star, LS = float(d["s_star"]), float(d["k_star"]), float(d["LS"])

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    ax = axes[0]
    ax.plot(grid2, M2, color="#7F8C8D", lw=0.8, label="S=350 扩大范围扫描")
    ax.plot(grid, M, color="#2E86C1", lw=1.2, label="S=200 主扫描")
    ax.axvspan(0, LS, color="#F9E79F", alpha=0.4, label="调头曲线段[0,LS]")
    ax.plot(s_star, k_star, '*', color="#E74C3C", ms=16, zorder=5,
            label=f"全局峰值 s*={s_star:.2f}, K_max={k_star:.4f}")
    ax.set_xlabel("龙头弧长坐标 s_head (m)"); ax.set_ylabel("速度放大系数 max_i K_i(s_head)")
    ax.set_title("全局放大系数扫描（远场收敛验证：S=200 vs S=350 一致）")
    ax.legend(loc='upper right', fontsize=8)

    ax2 = axes[1]
    mask = (grid > s_star - 15) & (grid < s_star + 15)
    ax2.plot(grid[mask], M[mask], color="#2E86C1", lw=1.5)
    ax2.plot(s_star, k_star, '*', color="#E74C3C", ms=18, zorder=5)
    ax2.axvline(LS, color="#8E44AD", ls='--', lw=1.2, label=f"LS={LS:.2f} (调头结束/盘出起点)")
    ax2.axvline(0, color="#27AE60", ls='--', lw=1.2, label="s=0 (P_in/调头起点)")
    ax2.set_xlabel("龙头弧长坐标 s_head (m)"); ax2.set_ylabel("max_i K_i(s_head)")
    ax2.set_title("峰值区域放大（调头曲线附近）")
    ax2.legend(loc='upper left', fontsize=8)
    fig.savefig(os.path.join(OUT, "fig5-1_放大系数全局扫描.png"))
    plt.close(fig)
    print("saved fig5-1")


def fig2_bottleneck():
    d = np.load(os.path.join(OUT, "q5_scan.npz"))
    s_star, k_star, v_head_max = float(d["s_star"]), float(d["k_star"]), float(d["v_head_max"])
    track = g.Track(ratio=2.0, theta_max=100.0, n_grid=300000)
    K = amplification(track, s_star)
    s, P, T = chain_positions(track, s_star)
    v = K * v_head_max

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    ax = axes[0]
    idx = np.arange(len(K))
    ax.plot(idx, K, '-', color="#2E86C1", lw=1.3)
    ax.axhline(k_star, color="#E74C3C", ls='--', lw=1, label=f"K_max={k_star:.4f}")
    ax.set_xlabel("把手序号 i (0=龙头)"); ax.set_ylabel("放大系数 K_i")
    ax.set_title(f"临界构型 (s_head*={s_star:.2f}m) 逐节点放大系数")
    ax.set_xlim(0, 30)
    ax.legend()

    ax2 = axes[1]
    ax2.plot(idx, v, '-', color="#27AE60", lw=1.3, label="缩放后速度 v_i")
    ax2.axhline(V_CAP, color="#E74C3C", ls='--', lw=1.5, label="速度上限 2 m/s")
    ax2.set_xlabel("把手序号 i (0=龙头)"); ax2.set_ylabel("速度 (m/s)")
    ax2.set_title(f"v_head={v_head_max:.6f} m/s 时全场速度剖面")
    ax2.legend()
    fig.savefig(os.path.join(OUT, "fig5-2_瓶颈节点与速度剖面.png"))
    plt.close(fig)
    print("saved fig5-2")


def fig3_geometry_context():
    """在S曲线几何图上标出临界构型时头部附近几节把手位置。"""
    d = np.load(os.path.join(OUT, "q5_scan.npz"))
    s_star = float(d["s_star"])
    track = g.Track(ratio=2.0, theta_max=100.0, n_grid=300000)
    sc = track.sc
    s, P, T = chain_positions(track, s_star)

    def spiral_xy(sign, tmax):
        th = np.linspace(g.THETA_C, tmax * 2 * np.pi, 4000)
        r = g.B * th
        return sign * r * np.cos(th), sign * r * np.sin(th)

    fig, ax = plt.subplots(figsize=(8, 8))
    inx, iny = spiral_xy(+1, 3)
    oux, ouy = spiral_xy(-1, 3)
    ax.plot(inx, iny, color="#D6EAF8", lw=1.0, label="盘入螺线")
    ax.plot(oux, ouy, color="#D5F5E3", lw=1.0, label="盘出螺线")
    ss = np.linspace(0, sc["LS"], 600)
    sxy = np.array([track.point(x) for x in ss])
    ax.plot(sxy[:, 0], sxy[:, 1], color="#F5B7B1", lw=1.6, label="S形调头曲线")
    ax.add_patch(Circle((0, 0), g.R_BOUND, fill=False, ec="#8E44AD", lw=1.2, ls='--'))

    n_show = 10
    ax.plot(P[:n_show, 0], P[:n_show, 1], '-o', color="#2E86C1", ms=5, lw=1.6,
            label=f"临界时刻前{n_show}节把手")
    ax.plot(P[0, 0], P[0, 1], '*', color="#E74C3C", ms=16, label="龙头(瞬时)")
    ax.plot(*sc["J"], 'x', color="#F39C12", ms=10, label="弧1/弧2切点J")
    ax.set_aspect('equal'); ax.set_xlim(-6, 6); ax.set_ylim(-6, 6)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"临界构型局部放大 (s_head*={s_star:.2f} m)")
    ax.legend(loc='upper left', fontsize=8)
    fig.savefig(os.path.join(OUT, "fig5-3_临界构型局部.png"))
    plt.close(fig)
    print("saved fig5-3")


if __name__ == "__main__":
    fig1_scan_curve()
    fig2_bottleneck()
    fig3_geometry_context()
    print("[done] figures ->", OUT)
