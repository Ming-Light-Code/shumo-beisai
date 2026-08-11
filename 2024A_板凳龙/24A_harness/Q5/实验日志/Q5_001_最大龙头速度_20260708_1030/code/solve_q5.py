"""Q5：沿问题4路径，龙头匀速行进，求最大龙头速度使各把手速度<=2 m/s。

核心：速度放大系数 K_i(s_head) = |dot(s_i)/dot(s_head)| 只依赖几何构型(s_head)，
与 v_head 数值无关(线性齐次)。故只需在 v_head=1 下对全路径 s_head 做全局扫描，
求 K_max = max_{s_head, i} K_i(s_head)，则 v_head_max = 2 / K_max。

依据 Q5/实现方案/Q5_v1_最大龙头速度方案.md。复用 geometry_q4.Track（copy 自 Q4_001）。
"""
import os
import numpy as np
from scipy.optimize import brentq, minimize_scalar
import geometry_q4 as g

N_NODE = 224
L_HEAD = 3.41 - 2 * 0.275     # 2.86
L_BODY = 2.20 - 2 * 0.275     # 1.65
D = np.empty(N_NODE)
D[0] = np.nan
D[1] = L_HEAD
D[2:] = L_BODY

V_CAP = 2.0                   # m/s 速度上限


def chain_positions(track, s_head):
    """给定龙头弧长坐标，返回各把手 s[224], P[224,2], T[224,2]（与 v_head 无关）。"""
    s = np.empty(N_NODE)
    P = np.empty((N_NODE, 2))
    T = np.empty((N_NODE, 2))
    s[0] = s_head
    P[0], T[0] = track.point_tangent(s_head)
    for i in range(1, N_NODE):
        s_prev = s[i - 1]
        Pprev = P[i - 1]
        Li = D[i]

        def gfun(si):
            return np.linalg.norm(Pprev - track.point(si)) - Li

        lo = s_prev - 2.2 * Li
        hi = s_prev - 1e-12
        for _ in range(40):
            if gfun(lo) > 0:
                break
            lo = s_prev - (s_prev - lo) * 1.5
        si = brentq(gfun, lo, hi, xtol=1e-11, rtol=1e-14)
        s[i] = si
        P[i], T[i] = track.point_tangent(si)
    return s, P, T


def amplification(track, s_head):
    """返回 K[224]：K_i = |dot(s_i)| when dot(s_head)=1（放大系数），及 argmax 节点。"""
    s, P, T = chain_positions(track, s_head)
    K = np.empty(N_NODE)
    K[0] = 1.0
    for i in range(1, N_NODE):
        dP = P[i - 1] - P[i]
        num = np.dot(dP, T[i - 1])
        den = np.dot(dP, T[i])
        K[i] = abs(num / den) * K[i - 1]
    return K


def max_amp(track, s_head):
    K = amplification(track, s_head)
    idx = int(np.argmax(K))
    return float(K[idx]), idx


def coarse_scan(track, s_lo, s_hi, step):
    grid = np.arange(s_lo, s_hi + step, step)
    M = np.empty_like(grid)
    ARG = np.empty_like(grid, dtype=int)
    for j, sh in enumerate(grid):
        M[j], ARG[j] = max_amp(track, float(sh))
    return grid, M, ARG


def refine_peak(track, s_lo, s_hi):
    res = minimize_scalar(lambda sh: -max_amp(track, sh)[0], bounds=(s_lo, s_hi),
                          method='bounded', options={'xatol': 1e-9})
    s_star = res.x
    k_star, idx = max_amp(track, s_star)
    return s_star, k_star, idx


def find_local_peaks(grid, M):
    """找出网格上的局部极大值索引（含端点特殊处理为非峰值，避免搜索区间外泄）。"""
    peaks = []
    for j in range(1, len(M) - 1):
        if M[j] >= M[j - 1] and M[j] >= M[j + 1]:
            peaks.append(j)
    return peaks


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(os.path.dirname(here), "outputs")
    os.makedirs(outdir, exist_ok=True)

    track = g.Track(ratio=2.0, theta_max=100.0, n_grid=300000)
    LS = track.LS
    print(f"[geom] LS={LS:.6f} m")

    # ---- 主扫描（S=200，覆盖远超题给 -100~100 窗口） ----
    S = 200.0
    step = 0.2
    grid, M, ARG = coarse_scan(track, -S, S + LS, step)
    peaks = find_local_peaks(grid, M)
    peaks_sorted = sorted(peaks, key=lambda j: -M[j])[:6]
    print("[coarse] top 局部极大值 (s_head, K, node):")
    for j in peaks_sorted:
        print(f"    s_head={grid[j]:9.3f}  K={M[j]:.6f}  node={ARG[j]}")

    j0 = int(np.argmax(M))
    lo = grid[max(j0 - 3, 0)]
    hi = grid[min(j0 + 3, len(grid) - 1)]
    s_star, k_star, idx = refine_peak(track, lo, hi)
    print(f"[refine] 全局峰值: s_head*={s_star:.6f} m, K_max={k_star:.8f}, node*={idx}")

    # ---- 收敛性验证：扩大到 S=350 ----
    S2 = 350.0
    grid2, M2, ARG2 = coarse_scan(track, -S2, S2 + LS, step)
    j0b = int(np.argmax(M2))
    print(f"[converge-check] S=350 粗扫描最大值 = {M2[j0b]:.6f} @ s_head={grid2[j0b]:.3f} "
          f"(对比 S=200 精化值 {k_star:.6f}) 差={abs(M2[j0b]-k_star):.2e}")

    v_head_max = V_CAP / k_star
    print(f"[answer] K_max={k_star:.6f} -> v_head_max = 2/{k_star:.6f} = {v_head_max:.6f} m/s")

    # ---- 验证：用 v_head_max 重算该临界构型附近，确认 max v_i = 2.000000 ----
    K_at_star = amplification(track, s_star)
    v_at_star = K_at_star * v_head_max
    print(f"[verify] 缩放后 max v_i = {v_at_star.max():.6f} (应=2.000000), "
          f"argmax node = {int(np.argmax(v_at_star))}")
    # 附近微扰确认确为局部/全局最大（非鞍点）
    for ds in [-0.05, -0.01, 0.01, 0.05]:
        k2, _ = max_amp(track, s_star + ds)
        assert k2 <= k_star + 1e-6, f"发现更大值于 s_head*+{ds}: {k2} > {k_star}"
    print("[verify] 邻域扰动确认为局部极大值 (非鞍点)")

    np.savez(os.path.join(outdir, "q5_scan.npz"), grid=grid, M=M, grid2=grid2, M2=M2,
             s_star=s_star, k_star=k_star, idx=idx, v_head_max=v_head_max, LS=LS)

    import csv
    with open(os.path.join(outdir, "q5_answer.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["量", "值"])
        w.writerow(["调头曲线长 LS (m)", f"{LS:.6f}"])
        w.writerow(["临界弧长坐标 s_head* (m)", f"{s_star:.6f}"])
        w.writerow(["瓶颈节点 index (0=龙头)", idx])
        w.writerow(["全局最大放大系数 K_max", f"{k_star:.8f}"])
        w.writerow(["龙头最大行进速度 v_head_max (m/s)", f"{v_head_max:.6f}"])
    print("[write] q5_answer.csv")
    print("[done]")


if __name__ == "__main__":
    main()
