"""Q1 盘入模型数值解：等距螺线上刚性链条的位置与速度。

依据 Q1/实现方案/Q1_v1_盘入模型数值解.md。
输出：result1.xlsx（位置/速度两 sheet，6 位小数）、论文表格 csv、快照数据 npz。
"""
import os
import numpy as np
from scipy.optimize import brentq

# ----------------------------- 常量 -----------------------------
P = 0.55                      # 螺距 (m)
B = P / (2.0 * np.pi)         # r = B*theta (m/rad)
V0 = 1.0                      # 龙头前把手速度 (m/s)
THETA0_INIT = 32.0 * np.pi    # t=0 龙头极角（第16圈正x轴）
N_NODE = 224                  # 把手总数 node 0..223
L_HEAD = 3.41 - 2 * 0.275     # 龙头把手间距 2.86 m
L_BODY = 2.20 - 2 * 0.275     # 龙身/龙尾把手间距 1.65 m
# 相邻节点距离 d[i] = node(i-1)->node(i)
D = np.empty(N_NODE)
D[0] = np.nan
D[1] = L_HEAD
D[2:] = L_BODY

T_LIST = np.arange(0, 301)    # 0..300 s
XTOL = 1e-12

# 论文表格抽取
PAPER_TIMES = [0, 60, 120, 180, 240, 300]
PAPER_NODES = [0, 1, 51, 101, 151, 201, 223]
PAPER_NODE_LABELS = ["龙头", "第1节龙身", "第51节龙身", "第101节龙身",
                     "第151节龙身", "第201节龙身", "龙尾(后)"]


# --------------------------- 几何函数 ---------------------------
def xy(theta):
    r = B * theta
    return r * np.cos(theta), r * np.sin(theta)


def dxy(theta):
    """dx/dtheta, dy/dtheta"""
    return (B * (np.cos(theta) - theta * np.sin(theta)),
            B * (np.sin(theta) + theta * np.cos(theta)))


def arc_len(theta):
    s = np.sqrt(theta * theta + 1.0)
    return 0.5 * B * (theta * s + np.log(theta + s))


# ------------------------- 求解单个时刻 -------------------------
def solve_head_theta(t):
    target = arc_len(THETA0_INIT) - V0 * t
    g = lambda th: arc_len(th) - target
    return brentq(g, 0.0, THETA0_INIT, xtol=XTOL)


def solve_next_theta(theta_prev, d):
    xp, yp = xy(theta_prev)

    def F(th):
        x, y = xy(th)
        return (x - xp) ** 2 + (y - yp) ** 2 - d * d

    lo = theta_prev + 1e-9
    # 初值估计 + 倍增扩张上界，直到 F(hi) > 0
    step = d / (B * np.sqrt(theta_prev ** 2 + 1.0))
    hi = theta_prev + step
    for _ in range(60):
        if F(hi) > 0:
            break
        hi = theta_prev + (hi - theta_prev) * 1.6
    else:
        raise RuntimeError(f"括号扩张失败 theta_prev={theta_prev}")
    return brentq(F, lo, hi, xtol=XTOL)


def solve_state(t):
    """返回该时刻 theta[224], x[224], y[224], vx[224], vy[224], v[224]."""
    theta = np.empty(N_NODE)
    theta[0] = solve_head_theta(t)
    for i in range(1, N_NODE):
        theta[i] = solve_next_theta(theta[i - 1], D[i])
    x, y = xy(theta)
    dx, dy = dxy(theta)

    vx = np.empty(N_NODE)
    vy = np.empty(N_NODE)
    # 龙头
    thd0 = -V0 / (B * np.sqrt(theta[0] ** 2 + 1.0))
    vx[0] = dx[0] * thd0
    vy[0] = dy[0] * thd0
    # 递推
    for i in range(1, N_NODE):
        ddx = x[i] - x[i - 1]
        ddy = y[i] - y[i - 1]
        denom = ddx * dx[i] + ddy * dy[i]
        thdi = (ddx * vx[i - 1] + ddy * vy[i - 1]) / denom
        vx[i] = dx[i] * thdi
        vy[i] = dy[i] * thdi
    v = np.sqrt(vx ** 2 + vy ** 2)
    return theta, x, y, vx, vy, v


# ------------------------------ 主流程 ------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(os.path.dirname(here), "outputs")
    os.makedirs(outdir, exist_ok=True)

    nt = len(T_LIST)
    X = np.empty((N_NODE, nt))
    Y = np.empty((N_NODE, nt))
    Vv = np.empty((N_NODE, nt))
    max_resid = 0.0
    mono_ok = True

    for j, t in enumerate(T_LIST):
        theta, x, y, vx, vy, v = solve_state(t)
        X[:, j] = x
        Y[:, j] = y
        Vv[:, j] = v
        # 校验：残差 & 单调
        for i in range(1, N_NODE):
            r = abs((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2 - D[i] ** 2)
            max_resid = max(max_resid, r)
        if not np.all(np.diff(theta) > 0):
            mono_ok = False

    print(f"[check] 最大定长残差 = {max_resid:.3e}")
    print(f"[check] theta 全程严格递增 = {mono_ok}")
    print(f"[check] 龙头速度 min/max = {Vv[0].min():.6f}/{Vv[0].max():.6f} m/s")
    print(f"[check] t=0 龙头坐标 = ({X[0,0]:.6f}, {Y[0,0]:.6f}) m")

    _write_excel(outdir, X, Y, Vv)
    _write_paper_tables(outdir, X, Y, Vv)
    _save_snapshots(outdir)
    print("[done] outputs ->", outdir)


def _row_labels():
    labels = ["龙头"]
    labels += [f"第{k}节龙身" for k in range(1, 222)]
    labels += ["龙尾", "龙尾(后)"]
    return labels  # 224


def _write_excel(outdir, X, Y, Vv):
    import openpyxl
    labels = _row_labels()
    time_headers = [f"{t} s" for t in T_LIST]
    wb = openpyxl.Workbook()

    ws1 = wb.active
    ws1.title = "位置"
    ws1.append([""] + time_headers)
    for i, lab in enumerate(labels):
        ws1.append([f"{lab}x (m)"] + [round(float(X[i, j]), 6) for j in range(len(T_LIST))])
        ws1.append([f"{lab}y (m)"] + [round(float(Y[i, j]), 6) for j in range(len(T_LIST))])

    ws2 = wb.create_sheet("速度")
    ws2.append([""] + time_headers)
    for i, lab in enumerate(labels):
        ws2.append([f"{lab} (m/s)"] + [round(float(Vv[i, j]), 6) for j in range(len(T_LIST))])

    path = os.path.join(outdir, "result1.xlsx")
    wb.save(path)
    print("[write]", path)


def _write_paper_tables(outdir, X, Y, Vv):
    import csv
    tcols = [list(T_LIST).index(t) for t in PAPER_TIMES]
    # 位置表
    with open(os.path.join(outdir, "paper_table1_position.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow([""] + [f"{t} s" for t in PAPER_TIMES])
        for node, lab in zip(PAPER_NODES, PAPER_NODE_LABELS):
            w.writerow([f"{lab} x (m)"] + [f"{X[node, j]:.6f}" for j in tcols])
            w.writerow([f"{lab} y (m)"] + [f"{Y[node, j]:.6f}" for j in tcols])
    # 速度表
    with open(os.path.join(outdir, "paper_table2_speed.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow([""] + [f"{t} s" for t in PAPER_TIMES])
        for node, lab in zip(PAPER_NODES, PAPER_NODE_LABELS):
            w.writerow([f"{lab} (m/s)"] + [f"{Vv[node, j]:.6f}" for j in tcols])
    print("[write] paper tables csv")


def _save_snapshots(outdir):
    snaps = {}
    for t in PAPER_TIMES:
        theta, x, y, vx, vy, v = solve_state(t)
        snaps[f"x_{t}"] = x
        snaps[f"y_{t}"] = y
        snaps[f"v_{t}"] = v
    np.savez(os.path.join(outdir, "snapshots.npz"), **snaps)
    print("[write] snapshots.npz")


if __name__ == "__main__":
    main()
