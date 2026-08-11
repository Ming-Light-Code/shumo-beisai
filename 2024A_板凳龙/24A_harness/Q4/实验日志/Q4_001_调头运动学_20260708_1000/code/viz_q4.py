"""Q4 可视化：调头曲线几何 + 复合轨道多时刻构型 + 速度 + 设计问题曲线。依据 viz-standard。"""
import os
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geometry_q4 as g

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


def spiral_xy(sign, tmax=12):
    th = np.linspace(g.THETA_C, tmax * 2 * np.pi, 6000)
    r = g.B * th
    return sign * r * np.cos(th), sign * r * np.sin(th)


def fig1_turn_geometry():
    """调头曲线几何：两螺线 + S曲线 + 两圆心/边界圆。"""
    tr = g.Track()
    sc = tr.sc
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))

    ax = axes[0]
    inx, iny = spiral_xy(+1, 9)
    oux, ouy = spiral_xy(-1, 9)
    ax.plot(inx, iny, color="#2E86C1", lw=0.8, label="盘入螺线 (p=1.7m)")
    ax.plot(oux, ouy, color="#27AE60", lw=0.8, label="盘出螺线 (中心对称)")
    ax.add_patch(Circle((0, 0), g.R_BOUND, fill=False, ec="#8E44AD", lw=1.6, ls='--',
                        label="调头空间 r=4.5m"))
    ss = np.linspace(0, sc["LS"], 800)
    sxy = np.array([tr.point(s) for s in ss])
    ax.plot(sxy[:, 0], sxy[:, 1], color="#E74C3C", lw=2.4, label="S形调头曲线")
    for pt, name, col in [(sc["P_in"], "P_in", "#E74C3C"), (sc["P_out"], "P_out", "#27AE60"),
                          (sc["J"], "切点J", "#F39C12")]:
        ax.plot(*pt, 'o', color=col, ms=8)
        ax.annotate(name, pt, textcoords="offset points", xytext=(6, 6), fontsize=9)
    for O, name in [(sc["O1"], "O1"), (sc["O2"], "O2")]:
        ax.plot(*O, 'x', color='k', ms=9)
        ax.annotate(name, O, textcoords="offset points", xytext=(6, -12), fontsize=9)
    ax.set_aspect('equal'); ax.set_xlim(-8, 8); ax.set_ylim(-8, 8)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"调头曲线构型  R1={sc['R1']:.3f}, R2={sc['R2']:.3f}, LS={sc['LS']:.3f} m")
    ax.legend(loc='upper left')

    # 右：S曲线放大
    ax2 = axes[1]
    ax2.plot(inx, iny, color="#2E86C1", lw=1.0)
    ax2.plot(oux, ouy, color="#27AE60", lw=1.0)
    ax2.add_patch(Circle((0, 0), g.R_BOUND, fill=False, ec="#8E44AD", lw=1.6, ls='--'))
    ax2.plot(sxy[:, 0], sxy[:, 1], color="#E74C3C", lw=2.6)
    ax2.add_patch(Circle(sc["O1"], sc["R1"], fill=False, ec="#7F8C8D", lw=0.8, ls=':'))
    ax2.add_patch(Circle(sc["O2"], sc["R2"], fill=False, ec="#7F8C8D", lw=0.8, ls=':'))
    for pt, col in [(sc["P_in"], "#E74C3C"), (sc["P_out"], "#27AE60"), (sc["J"], "#F39C12")]:
        ax2.plot(*pt, 'o', color=col, ms=8)
    ax2.plot(*sc["O1"], 'x', color='k'); ax2.plot(*sc["O2"], 'x', color='k')
    ax2.set_aspect('equal'); ax2.set_xlim(-5.2, 5.2); ax2.set_ylim(-5.2, 5.2)
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
    ax2.set_title("S形双圆弧放大（前弧半径=后弧2倍）")
    fig.savefig(os.path.join(OUT, "fig4-1_调头曲线几何.png"))
    plt.close(fig)
    print("saved fig4-1")


def fig2_snapshots():
    """多时刻整龙构型（复合轨道）。"""
    d = np.load(os.path.join(OUT, "q4_fields.npz"))
    t, X, Y = d["t"], d["X"], d["Y"]
    tr = g.Track(); sc = tr.sc
    times = [-100, -50, 0, 50, 100]
    inx, iny = spiral_xy(+1, 12); oux, ouy = spiral_xy(-1, 12)
    ss = np.linspace(0, sc["LS"], 400); sxy = np.array([tr.point(s) for s in ss])

    fig, axes = plt.subplots(1, 5, figsize=(24, 5.2))
    for ax, tt in zip(axes, times):
        j = list(t).index(tt)
        ax.plot(inx, iny, color="#D6EAF8", lw=0.7)
        ax.plot(oux, ouy, color="#D5F5E3", lw=0.7)
        ax.plot(sxy[:, 0], sxy[:, 1], color="#F5B7B1", lw=1.2)
        ax.add_patch(Circle((0, 0), g.R_BOUND, fill=False, ec="#8E44AD", lw=1.0, ls='--'))
        ax.plot(X[:, j], Y[:, j], '-', color="#2E86C1", lw=1.0)
        ax.plot(X[:, j], Y[:, j], '.', color="#34495E", ms=1.6)
        ax.plot(X[0, j], Y[0, j], '*', color="#E74C3C", ms=13)
        ax.set_aspect('equal'); ax.set_xlim(-16, 16); ax.set_ylim(-16, 16)
        ax.set_title(f"t = {tt} s"); ax.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    fig.suptitle("板凳龙调头过程构型（蓝=盘入, 绿=盘出, 红=调头曲线, ★=龙头）", fontsize=15)
    fig.savefig(os.path.join(OUT, "fig4-2_调头过程构型.png"))
    plt.close(fig)
    print("saved fig4-2")


def fig3_speed_design():
    d = np.load(os.path.join(OUT, "q4_fields.npz"))
    t, V = d["t"], d["V"]
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))

    ax = axes[0]
    for node, lab in [(0, "龙头"), (1, "第1节"), (51, "第51节"), (101, "第101节"),
                      (151, "第151节"), (223, "龙尾后")]:
        ax.plot(t, V[node], lw=1.3, label=lab)
    ax.axvspan(0, 13.62, color="#F9E79F", alpha=0.4, label="调头段(0~LS)")
    ax.set_xlabel("时间 t (s)"); ax.set_ylabel("速度 (m/s)")
    ax.set_title("各把手速度随时间（龙头恒1 m/s）"); ax.legend(loc='upper left', ncol=2)

    ax2 = axes[1]
    dr = np.load(os.path.join(OUT, "q4_design.npz"))
    de = np.load(os.path.join(OUT, "q4_design_entry.npz"))
    ax2.plot(dr["ratios"], dr["Ls"], color="#2E86C1", lw=2, label="主答案：变半径比(切点固定@4.5)，LS恒定=13.6212m")
    ax2b = ax2.twiny()
    ax2b.plot(de["rho"], de["LS"], 'o-', color="#E74C3C", lw=2, label="扩展讨论：变切点半径r_entry（非正式答案，无下界）")
    ax2.set_xlabel("半径比 R1/R2", color="#2E86C1"); ax2.set_ylabel("调头曲线长 LS (m)")
    ax2b.set_xlabel("切点半径 r_entry (m)", color="#E74C3C")
    ax2.set_title("设计问题：主答案(比例无关，不能变短) vs 扩展讨论(切点内移)")
    ax2.grid(True, alpha=0.3)
    lines = ax2.get_lines() + ax2b.get_lines()
    ax2.legend(lines, [l.get_label() for l in lines], loc='center right')
    fig.savefig(os.path.join(OUT, "fig4-3_速度与设计.png"))
    plt.close(fig)
    print("saved fig4-3")


if __name__ == "__main__":
    fig1_turn_geometry()
    fig2_snapshots()
    fig3_speed_design()
    print("[done] figures ->", OUT)
