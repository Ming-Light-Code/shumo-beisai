"""Q3 最小螺距：复用Q1定位+Q2判碰，螺距参数化，二分求最小可行螺距。

依据 Q3/实现方案/Q3_v1_最小螺距方案.md。
核心：求 p_min 使龙头前把手盘入到边界 r=4.5m 时全局间隙恰为0（临界相切）。
"""
import os
import numpy as np
from scipy.optimize import brentq

# ----------------------------- 板凳物理常量（与螺距无关） -----------------------------
V0 = 1.0
N_NODE = 224
L_HEAD = 3.41 - 2 * 0.275      # 2.86 m
L_BODY = 2.20 - 2 * 0.275      # 1.65 m
D = np.empty(N_NODE)
D[0] = np.nan
D[1] = L_HEAD
D[2:] = L_BODY
XTOL = 1e-12

N_BENCH = N_NODE - 1           # 223
HL = np.empty(N_BENCH)
HL[0] = 3.41 / 2.0             # 龙头半长 1.705 m
HL[1:] = 2.20 / 2.0           # 龙身/龙尾半长 1.10 m
HW = 0.30 / 2.0               # 半宽 0.15 m
_IU, _JU = np.triu_indices(N_BENCH, k=2)   # 全穷举候选对（排除相邻）

R_BOUND = 4.5                  # 调头空间边界半径 (m)


# --------------------------- 几何函数（b 作参数） ---------------------------
def xy(theta, b):
    r = b * theta
    return r * np.cos(theta), r * np.sin(theta)


def solve_next_theta(theta_prev, d, b):
    xp = b * theta_prev * np.cos(theta_prev)
    yp = b * theta_prev * np.sin(theta_prev)

    def F(th):
        x = b * th * np.cos(th)
        y = b * th * np.sin(th)
        return (x - xp) ** 2 + (y - yp) ** 2 - d * d

    lo = theta_prev + 1e-9
    step = d / (b * np.sqrt(theta_prev ** 2 + 1.0))
    hi = theta_prev + step
    for _ in range(80):
        if F(hi) > 0:
            break
        hi = theta_prev + (hi - theta_prev) * 1.6
    else:
        raise RuntimeError(f"括号扩张失败 theta_prev={theta_prev}, b={b}")
    return brentq(F, lo, hi, xtol=XTOL)


def chain_theta(theta0, b):
    theta = np.empty(N_NODE)
    theta[0] = theta0
    for i in range(1, N_NODE):
        theta[i] = solve_next_theta(theta[i - 1], D[i], b)
    return theta


def benches_from_xy(x, y):
    P_ = np.stack([x, y], axis=1)
    p0, p1 = P_[:-1], P_[1:]
    seg = p1 - p0
    length = np.linalg.norm(seg, axis=1)
    U = seg / length[:, None]
    N = np.stack([-U[:, 1], U[:, 0]], axis=1)
    C = 0.5 * (p0 + p1)
    return C, U, N


def clearance(theta0, b):
    """全局间隙 G(theta0; b)：>0 全分离(不碰)，<=0 至少一对碰撞。"""
    theta = chain_theta(theta0, b)
    x, y = xy(theta, b)
    C, U, N = benches_from_xy(x, y)
    Ci, Cj = C[_IU], C[_JU]
    Ui, Uj = U[_IU], U[_JU]
    Ni, Nj = N[_IU], N[_JU]
    HLi, HLj = HL[_IU], HL[_JU]
    cdiff = Cj - Ci

    def margin(a):
        proj = np.abs(np.sum(cdiff * a, axis=1))
        ri = HLi * np.abs(np.sum(Ui * a, axis=1)) + HW * np.abs(np.sum(Ni * a, axis=1))
        rj = HLj * np.abs(np.sum(Uj * a, axis=1)) + HW * np.abs(np.sum(Nj * a, axis=1))
        return proj - (ri + rj)

    pm = np.maximum(np.maximum(margin(Ui), margin(Ni)), np.maximum(margin(Uj), margin(Nj)))
    k = int(np.argmin(pm))
    return float(pm[k]), (int(_IU[k]) + 1, int(_JU[k]) + 1)


def f_of_pitch(p):
    """龙头前把手在边界 r=4.5m 时的全局间隙。f>0 可行(螺距偏大)，f<0 不可行。"""
    b = p / (2.0 * np.pi)
    theta_h = R_BOUND / b            # = 9*pi/p
    g, _ = clearance(theta_h, b)
    return g


# ------------------------------ 主流程 ------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(os.path.dirname(here), "outputs")
    os.makedirs(outdir, exist_ok=True)

    # Step 0: 括号定位 —— 扫描 f(p) 找变号区间
    ps = np.round(np.arange(0.30, 0.561, 0.02), 4)
    print("[scan] p -> f(p)=G(head@4.5m):")
    fvals = []
    for p in ps:
        fp = f_of_pitch(float(p))
        fvals.append(fp)
        print(f"   p={p:.3f}  f={fp:+.6f}")
    fvals = np.array(fvals)
    # 找首个由负转正的相邻点
    sign = fvals > 0
    idx = None
    for i in range(len(ps) - 1):
        if (not sign[i]) and sign[i + 1]:
            idx = i
            break
    if idx is None:
        raise RuntimeError("未在扫描区间找到 f 的变号，需扩大 p 范围")
    p_lo, p_hi = float(ps[idx]), float(ps[idx + 1])
    print(f"[bracket] f({p_lo})={fvals[idx]:+.6f} (不可行), f({p_hi})={fvals[idx+1]:+.6f} (可行)")

    # Step 1: brentq 求 f(p)=0
    p_min = brentq(f_of_pitch, p_lo, p_hi, xtol=1e-8, rtol=1e-12)
    b_min = p_min / (2.0 * np.pi)
    theta_h = R_BOUND / b_min
    g_star, pair = clearance(theta_h, b_min)
    print(f"\n[result] p_min = {p_min:.6f} m")
    print(f"         b_min  = {b_min:.6f} m/rad,  龙头边界极角 theta_h = {theta_h:.6f} rad "
          f"= {theta_h/(2*np.pi):.4f} 圈")
    print(f"         临界间隙 G = {g_star:.3e}  临界碰撞对(板凳编号) = {pair}")

    # Step 2: 后验证 —— 边界是否为全程最紧约束（假设 Q3-A04）
    print("\n[verify] 在 p_min 下扫描 G(theta_h) 从边界向外若干圈：")
    thetas = theta_h + np.linspace(0, 6 * 2 * np.pi, 120)  # 向外 6 圈
    Gs = np.array([clearance(th, b_min)[0] for th in thetas])
    r_of = b_min * thetas
    argmin = int(np.argmin(Gs))
    print(f"   边界处 G={Gs[0]:+.6e} (r={r_of[0]:.3f}m)")
    print(f"   向外扫描最小 G={Gs[argmin]:+.6e} 出现在 r={r_of[argmin]:.4f}m "
          f"(theta={thetas[argmin]:.4f})")
    boundary_is_tightest = (argmin == 0) or (Gs[argmin] >= -1e-9)
    print(f"   边界即最紧约束(或全程G>=0): {boundary_is_tightest}")

    # 保存
    np.savez(os.path.join(outdir, "q3_result.npz"),
             p_min=p_min, b_min=b_min, theta_h=theta_h, g_star=g_star,
             pair=np.array(pair), ps=ps, fvals=fvals,
             verify_theta=thetas, verify_G=Gs, verify_r=r_of)
    theta_full = chain_theta(theta_h, b_min)
    x_full, y_full = xy(theta_full, b_min)
    np.savez(os.path.join(outdir, "q3_critical_config.npz"),
             x=x_full, y=y_full, b_min=b_min, p_min=p_min, theta_h=theta_h,
             pair=np.array(pair), R_bound=R_BOUND)

    _write_summary_csv(outdir, p_min, b_min, theta_h, g_star, pair, boundary_is_tightest)
    print("\n[done] outputs ->", outdir)


def _write_summary_csv(outdir, p_min, b_min, theta_h, g_star, pair, ok):
    import csv
    with open(os.path.join(outdir, "q3_summary.csv"), "w", newline="", encoding="utf-8-sig") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(["量", "值"])
        w.writerow(["最小螺距 p_min (m)", f"{p_min:.6f}"])
        w.writerow(["b_min = p_min/2pi (m/rad)", f"{b_min:.6f}"])
        w.writerow(["龙头边界极角 theta_h (rad)", f"{theta_h:.6f}"])
        w.writerow(["龙头边界处圈数", f"{theta_h/(2*np.pi):.4f}"])
        w.writerow(["临界间隙 G (m)", f"{g_star:.3e}"])
        w.writerow(["临界碰撞板凳对(1-based)", str(pair)])
        w.writerow(["边界为全程最紧约束", str(ok)])
    print("[write] q3_summary.csv")


if __name__ == "__main__":
    main()
