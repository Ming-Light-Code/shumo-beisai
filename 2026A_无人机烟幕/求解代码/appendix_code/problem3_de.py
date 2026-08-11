# -*- coding: utf-8 -*-
"""problem3_de.py -- Q3 DE: 定向枚举 + 位图加速"""
import numpy as np, math, time, random, sys, os, openpyxl
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fast_eval import (sphere_shield_dur, DI, VMIN, VMAX, DTS, NTS, TA, LP, CD, CS, CR, G as g_acc)
from common import solve_shielding_duration_sum, FY1_INIT, GRAVITY
from math import cos, sin, pi, radians, degrees

tk = 0; di = 0; G = g_acc

def precompute_coverage(th, v, td, tau):
    td_burst = td + tau
    r0 = DI[di]; ct = cos(th); st = sin(th)
    xd, yd = r0[0] + v*ct*td_burst, r0[1] + v*st*td_burst
    zd = r0[2] - 0.5*G*tau*tau
    tsi, tei = max(0, int(td_burst/DTS)), min(NTS, int((td_burst+CD)/DTS)+1)
    if tei <= tsi: return np.zeros(NTS, dtype=bool)
    ta_s = TA[tsi:tei]; active = ta_s >= td_burst
    if not np.any(active): return np.zeros(NTS, dtype=bool)
    ta_a = ta_s[active]; cz = zd - CS*(ta_a - td_burst)
    ttx = LP[tk, tsi:tei, 0][active]; tty = LP[tk, tsi:tei, 1][active]; ttz = LP[tk, tsi:tei, 2][active]
    mpx = LP[tk, tsi:tei, 3][active]; mpy = LP[tk, tsi:tei, 4][active]; mpz = LP[tk, tsi:tei, 5][active]
    wcx, wcy, wcz = xd - mpx, yd - mpy, cz - mpz
    c2 = ttx*ttx + tty*tty + ttz*ttz + 1e-10; c1 = wcx*ttx + wcy*tty + wcz*ttz
    s = np.clip(c1/c2, 0.0, 1.0)
    px, py, pz = mpx + s*ttx, mpy + s*tty, mpz + s*ttz
    covered = (xd-px)**2 + (yd-py)**2 + (cz-pz)**2 < CR*CR
    result = np.zeros(NTS, dtype=bool)
    for j, oi in enumerate(np.where(active)[0]): result[tsi+oi] = covered[j]
    return result

def de_optimize(th_range=21, v_steps=7, td_steps=15, tau_steps=9):
    t0 = time.time()
    best_dur, best_params = 0.0, None

    for thi in range(th_range):
        th = radians(175 + thi)
        for vi in range(v_steps):
            v = 130 + vi
            cands = []
            for td in np.linspace(0, 7, td_steps):
                for tau in np.linspace(0.001, 8.0, tau_steps):
                    dur = sphere_shield_dur(di, th, v, tk, td, tau)
                    if dur > 0.15: cands.append((dur, td, tau))
            cands.sort(key=lambda x: -x[0])
            if len(cands) < 3: continue

            top_n = min(15, len(cands))
            coverages = [precompute_coverage(th, v, c[1], c[2]) for c in cands[:top_n]]

            for i in range(top_n):
                for j in range(top_n):
                    if i == j: continue
                    u_ij = coverages[i] | coverages[j]
                    for k in range(top_n):
                        if k in (i,j): continue
                        dur = float(np.sum(u_ij | coverages[k])) * DTS
                        if dur > best_dur:
                            best_dur = dur
                            best_params = (th, v, [(cands[i][1],cands[i][2]),(cands[j][1],cands[j][2]),(cands[k][1],cands[k][2])])

    elapsed = time.time() - t0
    if best_params is None: return None

    th, v, combo = best_params; tds = [c[0] for c in combo]; tfs = [c[1] for c in combo]
    best_ref, best_xr = best_dur, (th, v, tds, tfs)
    for _ in range(30):
        th2, v2 = th + random.uniform(-0.03,0.03), np.clip(v+random.uniform(-2,2), VMIN, VMAX)
        tds2, tfs2 = tds.copy(), tfs.copy()
        for j in range(3):
            tds2[j] = max(0, tds[j]+random.uniform(-0.08,0.08))
            tfs2[j] = max(0.001, tfs[j]+random.uniform(-0.08,0.08))
        tds2[1] = max(tds2[1], tds2[0]+1.0); tds2[2] = max(tds2[2], tds2[1]+1.0)
        covs = [precompute_coverage(th2, v2, tds2[j], tfs2[j]) for j in range(3)]
        dur2 = float(np.sum(covs[0]|covs[1]|covs[2]))*DTS
        if dur2 > best_ref: best_ref, best_xr = dur2, (th2, v2, tds2, tfs2)

    th_f, v_f, tds_f, tfs_f = best_xr
    all_bp = []; d_vec = np.array([cos(th_f), sin(th_f), 0.])
    for j in range(3):
        bp = FY1_INIT + v_f*(tds_f[j]+tfs_f[j])*d_vec - np.array([0.,0.,0.5*GRAVITY*tfs_f[j]**2])
        all_bp.append((bp, tds_f[j]+tfs_f[j]))
    total_p, intervals = solve_shielding_duration_sum(all_bp, 0, dt_coarse=0.005)
    return th_f, v_f, [(tds_f[i], tfs_f[i]) for i in range(3)], total_p, intervals, elapsed


def write_result1(th, v, combo, dur_p, intervals, filename="result1.xlsx"):
    """Write Q3 results to result1.xlsx matching template format."""
    wb = openpyxl.Workbook()
    ws = wb.active; ws.title = "Sheet1"
    ws.append(["无人机运动方向", "无人机运动速度 (m/s)", "烟幕干扰弹编号",
               "烟幕干扰弹投放点的x坐标 (m)", "烟幕干扰弹投放点的y坐标 (m)", "烟幕干扰弹投放点的z坐标 (m)",
               "烟幕干扰弹起爆点的x坐标 (m)", "烟幕干扰弹起爆点的y坐标 (m)", "烟幕干扰弹起爆点的z坐标 (m)",
               "有效干扰时长 (s)"])
    heading_deg = degrees(th)
    ct, st = cos(th), sin(th)
    for j, (td, tf) in enumerate(combo):
        tb = td + tf
        rx = FY1_INIT[0] + v*ct*td; ry = FY1_INIT[1] + v*st*td; rz = FY1_INIT[2]
        bx = FY1_INIT[0] + v*ct*tb; by = FY1_INIT[1] + v*st*tb
        bz = FY1_INIT[2] - 0.5*GRAVITY*tf*tf
        # Individual smoke duration from intervals
        dur_j = 0.0
        if j < len(intervals): dur_j = intervals[j][1] - intervals[j][0]
        ws.append([f"{heading_deg:.6f}" if j==0 else "", f"{v:.6f}" if j==0 else "", j+1,
                   f"{rx:.2f}", f"{ry:.2f}", f"{rz:.2f}",
                   f"{bx:.2f}", f"{by:.2f}", f"{bz:.2f}", f"{dur_j:.6f}"])
    ws.append(["", "", "", "", "", "", "", "", "", ""])
    ws.append(["注：以x轴为正向，逆时针方向为正，取值0~360（度）。", "", "", "", "", "", "", "", "", ""])
    wb.save(filename)
    print(f"  result1.xlsx 已保存")

if __name__ == "__main__":
    print("=" * 60)
    print("  问题三 DE -- 定向枚举（位图加速）")
    print("=" * 60)
    result = de_optimize()
    if result:
        th, v, combo, dur_p, intervals, elapsed = result
        print(f"  航向角 theta = {degrees(th):.6f} deg")
        print(f"  飞行速度 v   = {v:.6f} m/s")
        for j, (td, tf) in enumerate(combo):
            print(f"  烟幕弹 {j+1}:  投放={td:.6f} s  下落={tf:.6f} s  起爆={td+tf:.6f} s")
        print(f"  有效遮蔽总时长 = {dur_p:.6f} s  (共 {len(intervals)} 段)")
        for i, (ts, te) in enumerate(intervals):
            print(f"    第{i+1}段: [{ts:.6f}, {te:.6f}]  时长 {te-ts:.6f} s")
        print(f"  运行耗时 = {elapsed:.0f} s")
        write_result1(th, v, combo, dur_p, intervals)
        print("=" * 60)
