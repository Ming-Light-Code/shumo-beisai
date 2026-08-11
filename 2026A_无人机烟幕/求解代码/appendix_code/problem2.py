# -*- coding: utf-8 -*-
"""problem2.py -- Q2: 单机单弹 快速NLP优化"""
import numpy as np, math, time, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fast_eval import fast_shield_center, sphere_shield_dur
from common import smoke_burst_pos, solve_shielding_duration, FY1_INIT, T_HIT

def optimize_q2(init_pos=FY1_INIT, grid_n=21, n_refine=3):
    tk = 0; di = 0
    t00 = time.time()
    best_dur, best_p = 0.0, None; top = []

    for ad in np.linspace(0, 25, grid_n):
        a = np.radians(ad)
        for v in np.linspace(100, 140, 9):
            for tb in np.linspace(0.3, 2.0, grid_n):
                d = fast_shield_center(di, a, v, tk, 0.0, tb)
                if d > best_dur * 0.7 and d > 1.0: top.append((d, a, v, tb))
                if d > best_dur: best_dur, best_p = d, (a, v, 0.0, tb)
    top.sort(key=lambda x: -x[0]); top = top[:30]

    bb = np.array([[0, 2*np.pi], [70, 140], [0.1, 20.0]])
    best_sph, best_x = 0.0, None
    for _, a0, v0, tb0 in top[:10]:
        x = np.array([a0, v0, tb0]); fb = sphere_shield_dur(di, x[0], x[1], tk, 0.0, x[2])
        ss = np.array([np.radians(0.8), 3.0, 0.05])
        for _ in range(n_refine):
            imp = False
            for d in range(3):
                for sgn in [-1, 1]:
                    xt = x.copy(); xt[d] = np.clip(xt[d]+sgn*ss[d], bb[d,0], bb[d,1])
                    ft = sphere_shield_dur(di, xt[0], xt[1], tk, 0.0, xt[2])
                    if ft > fb + 0.01: x, fb = xt.copy(), ft; imp = True
            ss *= 0.5
            if not imp: break
        if fb > best_sph + 0.05: best_sph, best_x = fb, x.copy()

    a, v, tb = best_x; td = 0.0
    bp = smoke_burst_pos(init_pos, v, a, td, tb)
    ts, te, dur5 = solve_shielding_duration([(bp, tb)], 0, dt_coarse=0.005)
    return {"alpha_deg": np.degrees(a), "v": v, "td": td, "tb": tb,
            "dur": dur5, "t_start": ts, "t_end": te, "elapsed": time.time()-t00}

if __name__ == "__main__":
    print("=" * 60)
    print("  问题二 -- 单机单弹最优策略（快速 NLP）")
    print("=" * 60)
    r = optimize_q2(FY1_INIT)
    print(f"  航向角 alpha = {r['alpha_deg']:.6f} deg")
    print(f"  飞行速度 v   = {r['v']:.6f} m/s")
    print(f"  投放时刻 td  = {r['td']:.6f} s")
    print(f"  起爆时刻 tb  = {r['tb']:.6f} s")
    print(f"  遮蔽区间     = [{r['t_start']:.6f}, {r['t_end']:.6f}] s")
    print(f"  有效遮蔽时长 = {r['dur']:.6f} s")
    print(f"  运行耗时     = {r['elapsed']:.1f} s")
    print("=" * 60)
