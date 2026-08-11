# -*- coding: utf-8 -*-
"""problem3_nlp.py -- Q3 NLP: 多弹预筛选 + 波束搜索"""
import numpy as np, math, time, random, sys, os
from pathlib import Path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fast_eval import sphere_shield_dur, multi_sphere_shield, DI, VMIN, VMAX
from common import solve_shielding_duration_sum, FY1_INIT, GRAVITY
from math import cos, sin, pi, radians, degrees

tk = 0; di = 0; G = GRAVITY

def multi_smoke_prescreen(th, v):
    return multi_sphere_shield(di, th, v, tk, [(0.0,0.5),(1.5,4.0),(4.0,6.0)])

def gen_candidates(th, v, n_td=25, n_tau=8):
    cands = []
    for td in np.linspace(0, 10, n_td):
        for tau in np.linspace(0.001, 10.0, n_tau):
            dur = sphere_shield_dur(di, th, v, tk, td, tau)
            if dur > 0.15: cands.append((dur, td, tau))
    cands.sort(key=lambda x: -x[0])
    return cands[:60]

def beam_search_3(th, v, candidates):
    cands = candidates.copy(); best_dur, best_combo = 0, []
    for ai in range(min(15, len(cands))):
        chosen = [(cands[ai][1], cands[ai][2])]
        for _ in range(2):
            best_add_d, best_add = 0, None
            for cd, ctd, ctau in cands:
                if any(abs(ctd-c[0])<0.9 for c in chosen): continue
                dur = multi_sphere_shield(di, th, v, tk, chosen + [(ctd, ctau)])
                if dur > best_add_d: best_add_d, best_add = dur, (ctd, ctau)
            if best_add: chosen.append(best_add)
        if len(chosen) == 3:
            dur = multi_sphere_shield(di, th, v, tk, chosen)
            if dur > best_dur: best_dur, best_combo = dur, chosen
    return best_dur, best_combo

def nlp_optimize(th_range=180):
    t0 = time.time(); best_global, best_params = 0, None
    for thi in range(th_range):
        th = 2*pi*thi/th_range
        best_v, best_vd = 0, 0
        for v in np.linspace(70, VMAX, 10):
            d = multi_smoke_prescreen(th, v)
            if d > best_vd: best_vd, best_v = d, v
        if best_vd < 0.8: continue
        cands = gen_candidates(th, best_v, 25, 8)
        if len(cands) < 3: continue
        dur, combo = beam_search_3(th, best_v, cands)
        if dur > best_global: best_global, best_params = dur, (th, best_v, combo)
        if time.time()-t0 > 180: break

    if best_params is None: return None
    th, v, combo = best_params; tds = [c[0] for c in combo]; tfs = [c[1] for c in combo]

    best_ref, best_xr = best_global, (th, v, tds, tfs)
    for _ in range(50):
        th2, v2 = th+random.uniform(-0.05,0.05), np.clip(v+random.uniform(-5,5),VMIN,VMAX)
        tds2, tfs2 = tds.copy(), tfs.copy()
        for j in range(3):
            tds2[j] = max(0, tds[j]+random.uniform(-0.2,0.2))
            tfs2[j] = max(0.001, tfs[j]+random.uniform(-0.2,0.2))
        tds2[1]=max(tds2[1],tds2[0]+1.0); tds2[2]=max(tds2[2],tds2[1]+1.0)
        combo2 = [(tds2[j],tfs2[j]) for j in range(3)]
        dur2 = multi_sphere_shield(di, th2, v2, tk, combo2)
        if dur2 > best_ref: best_ref, best_xr = dur2, (th2, v2, tds2, tfs2)

    th_f, v_f, tds_f, tfs_f = best_xr
    all_bp = []; d_vec = np.array([cos(th_f), sin(th_f), 0.])
    for j in range(3):
        bp = FY1_INIT + v_f*(tds_f[j]+tfs_f[j])*d_vec - np.array([0.,0.,0.5*G*tfs_f[j]**2])
        all_bp.append((bp, tds_f[j]+tfs_f[j]))
    total_p, intervals = solve_shielding_duration_sum(all_bp, 0, dt_coarse=0.005)
    return th_f, v_f, [(tds_f[i],tfs_f[i]) for i in range(3)], total_p, intervals, time.time()-t0

def write_result(th: float, v: float, combo: list, dur_p: float,
                 intervals: list) -> None:
    """输出 Excel 结果文件 (参照 A题 result1 格式)."""
    from openpyxl import Workbook

    wb = Workbook()
    ws = wb.active
    ws.title = "Sheet1"

    headers = ["无人机运动方向", "无人机运动速度 (m/s)", "烟幕弹编号",
               "烟幕弹投放点x坐标 (m)", "烟幕弹投放点y坐标 (m)", "烟幕弹投放点z坐标 (m)",
               "烟幕弹起爆点x坐标 (m)", "烟幕弹起爆点y坐标 (m)", "烟幕弹起爆点z坐标 (m)",
               "有效干扰时间 (s)"]
    ws.append(headers)

    d_vec = np.array([cos(th), sin(th), 0.])
    dir_deg = degrees(th) % 360

    for j, (td, tf) in enumerate(combo):
        tb = td + tf
        drop_pos = FY1_INIT + v * td * d_vec
        burst_pos = FY1_INIT + v * tb * d_vec - np.array([0., 0., 0.5 * G * tf ** 2])
        dur_j = sphere_shield_dur(di, th, v, tk, td, tf)

        dir_val = round(dir_deg, 6) if j == 0 else ""
        spd_val = round(v, 6) if j == 0 else ""

        ws.append([dir_val, spd_val, j + 1,
                   round(float(drop_pos[0]), 2), round(float(drop_pos[1]), 2), round(float(drop_pos[2]), 2),
                   round(float(burst_pos[0]), 2), round(float(burst_pos[1]), 2), round(float(burst_pos[2]), 2),
                   round(dur_j, 4)])

    ws.append([round(dur_p, 4), "", "", "", "", "", "", "", "", ""])
    ws.append(["注：x轴为正北方向，逆时针方向为正，取值0~360度；", "", "", "", "", "", "", "", "", ""])

    out = Path(__file__).parent / "result1_NLP.xlsx"
    wb.save(str(out))
    print(f"\n结果已保存至 {out}")


if __name__ == "__main__":
    print("=" * 60)
    print("  问题三 NLP -- 波束搜索")
    print("=" * 60)
    result = nlp_optimize(th_range=180)
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
        print("=" * 60)
        write_result(th, v, combo, dur_p, intervals)
