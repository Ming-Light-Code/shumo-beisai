"""Q3 最小螺距：复用Q1定位+Q2判碰，螺距参数化，二分求最小可行螺距。

依据 Q3/实现方案/Q3_v1_最小螺距方案.md。
核心：求 p_min 使龙头前把手盘入到边界 r=4.5m 时全局间隙恰为0（临界相切）。
"""
import os
import numpy as np
from scipy.optimize import brentq, minimize_scalar

# ----------------------------- 板凳物理常量（与螺距无关） -----------------------------
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


W_TURNS = 3.0        # 从边界向外扫描的窗口(圈)，覆盖近边界最紧接触
NPTS_SCAN = 240      # 每次 m(p) 评估的 theta_h 采样点数


def descent_min_clearance(p, w_turns=W_TURNS, npts=NPTS_SCAN):
    """盘入到边界过程的最紧间隙 m(p)=min_{theta_h>=9pi/p} G。
    返回 (m, theta_at_min, r_at_min, pair_at_min, k, ths, Gs)。
    龙头盘入 => theta_h 从大到 9pi/p；等价扫描 theta_h in [9pi/p, 9pi/p + w]。

    两级流程：(1) 粗网格扫描定位最小值所在区间（避免遗漏，用于全局把握 m(p) 形状）；
    (2) 在网格最小点的邻域用 minimize_scalar 连续精化，消除网格量化噪声
    （粗网格步长 dth=w_turns*2pi/npts 对应约 1e-3~1e-4 rad 的离散化误差，
    直接用网格值会导致 m(p)=0 的根依赖网格分辨率，不利于6位小数精度的复现）。
    m_of_pitch 与后续 brentq 求根统一使用此精化值，保证全流程数值自洽。
    """
    b = p / (2.0 * np.pi)
    th0 = R_BOUND / b
    ths = th0 + np.linspace(0.0, w_turns * 2 * np.pi, npts)
    Gs = np.empty(npts)
    pairs = [None] * npts
    for i, th in enumerate(ths):
        g, pr = clearance(th, b)
        Gs[i] = g
        pairs[i] = pr
    k = int(np.argmin(Gs))

    lo = ths[max(k - 1, 0)]
    hi = ths[min(k + 1, npts - 1)]
    res = minimize_scalar(lambda th: clearance(th, b)[0], bounds=(lo, hi),
                          method='bounded', options={'xatol': 1e-10})
    th_ref = float(res.x)
    m_ref, pair_ref = clearance(th_ref, b)
    return float(m_ref), th_ref, float(b * th_ref), pair_ref, k, ths, Gs


def m_of_pitch(p):
    return descent_min_clearance(p)[0]


# ------------------------------ 主流程 ------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(os.path.dirname(here), "outputs")
    os.makedirs(outdir, exist_ok=True)

    # Step 0: 括号定位 —— 扫描 m(p)=盘入全程最紧间隙 找变号区间
    ps = np.round(np.arange(0.40, 0.501, 0.01), 4)
    print("[scan] p -> m(p)=盘入到边界的最紧间隙:")
    mvals = []
    for p in ps:
        mp = m_of_pitch(float(p))
        mvals.append(mp)
        print(f"   p={p:.3f}  m={mp:+.6f}")
    mvals = np.array(mvals)
    sign = mvals > 0
    idx = None
    for i in range(len(ps) - 1):
        if (not sign[i]) and sign[i + 1]:
            idx = i
            break
    if idx is None:
        raise RuntimeError("未在扫描区间找到 m 的变号，需扩大 p 范围")
    p_lo, p_hi = float(ps[idx]), float(ps[idx + 1])
    print(f"[bracket] m({p_lo})={mvals[idx]:+.6f} (不可行), m({p_hi})={mvals[idx+1]:+.6f} (可行)")

    # Step 1: brentq 求 m(p)=0（m_of_pitch 内部已做连续精化，消除网格量化噪声）
    p_min = brentq(m_of_pitch, p_lo, p_hi, xtol=1e-9, rtol=1e-14)
    b_min = p_min / (2.0 * np.pi)
    theta_h_bnd = R_BOUND / b_min
    m_star, th_star, r_star, pair, kmin, ths, Gs = descent_min_clearance(p_min, npts=400)
    print(f"\n[result] p_min = {p_min:.6f} m")
    print(f"         b_min  = {b_min:.6f} m/rad,  龙头边界极角 theta_h = {theta_h_bnd:.6f} rad "
          f"= {theta_h_bnd/(2*np.pi):.4f} 圈")
    print(f"         盘入全程最紧间隙 m = {m_star:.3e}")
    print(f"         最紧接触发生在 r = {r_star:.6f} m (theta_h={th_star:.6f}), 临界碰撞对(板凳编号) = {pair}")
    print(f"         (边界半径 R = {R_BOUND} m; 最紧接触{'在边界处' if abs(r_star-R_BOUND)<1e-3 else '在边界外侧 '+f'{r_star-R_BOUND:.4f} m'})")

    # Step 2: 后验证 —— 边界处间隙应 >=0（龙头确能抵达边界）
    g_bnd, _ = clearance(theta_h_bnd, b_min)
    print(f"\n[verify] 边界处(r=4.5m) G={g_bnd:+.6e} (应>=0，龙头可抵达边界)")
    print(f"[verify] 最紧接触 m={m_star:+.3e} (应≈0，临界)")

    np.savez(os.path.join(outdir, "q3_result.npz"),
             p_min=p_min, b_min=b_min, theta_h_bnd=theta_h_bnd,
             m_star=m_star, th_star=th_star, r_star=r_star, pair=np.array(pair),
             ps=ps, mvals=mvals, scan_theta=ths, scan_G=Gs, g_bnd=g_bnd)
    theta_full = chain_theta(th_star, b_min)   # 临界接触时刻的构型
    x_full, y_full = xy(theta_full, b_min)
    theta_bnd_full = chain_theta(theta_h_bnd, b_min)  # 龙头到边界时的构型
    xb, yb = xy(theta_bnd_full, b_min)
    np.savez(os.path.join(outdir, "q3_critical_config.npz"),
             x=x_full, y=y_full, x_bnd=xb, y_bnd=yb,
             b_min=b_min, p_min=p_min, theta_h_bnd=theta_h_bnd, th_star=th_star,
             pair=np.array(pair), R_bound=R_BOUND)

    _write_summary_csv(outdir, p_min, b_min, theta_h_bnd, m_star, r_star, pair, g_bnd)
    print("\n[done] outputs ->", outdir)


def _write_summary_csv(outdir, p_min, b_min, theta_h, m_star, r_star, pair, g_bnd):
    import csv
    with open(os.path.join(outdir, "q3_summary.csv"), "w", newline="", encoding="utf-8-sig") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(["量", "值"])
        w.writerow(["最小螺距 p_min (m)", f"{p_min:.6f}"])
        w.writerow(["b_min = p_min/2pi (m/rad)", f"{b_min:.6f}"])
        w.writerow(["龙头边界极角 theta_h (rad)", f"{theta_h:.6f}"])
        w.writerow(["龙头边界处圈数", f"{theta_h/(2*np.pi):.4f}"])
        w.writerow(["盘入全程最紧间隙 m (m)", f"{m_star:.3e}"])
        w.writerow(["最紧接触半径 r_star (m)", f"{r_star:.6f}"])
        w.writerow(["临界碰撞板凳对(1-based)", str(pair)])
        w.writerow(["边界处间隙 G(r=4.5) (m)", f"{g_bnd:.6e}"])
    print("[write] q3_summary.csv")


if __name__ == "__main__":
    main()
