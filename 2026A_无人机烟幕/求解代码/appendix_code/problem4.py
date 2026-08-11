# -*- coding: utf-8 -*-
"""problem4.py -- Q4: 三机协同 已知参数 + 联合累加"""
import numpy as np, math, time, sys, os, openpyxl
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import smoke_burst_pos, solve_shielding_duration_sum, GRAVITY
from math import cos, sin, pi, radians, degrees

F_INIT = [np.array([17800.,0.,1800.]), np.array([12000.,1400.,1400.]), np.array([6000.,-3000.,700.])]
THS_DEG = [179.110941, 308.046683, 73.809276]
VS      = [132.062839, 138.991477, 137.409810]
TDS     = [0.369336,   8.336915,  22.723803]
TAUS    = [3.636731,   4.163631,   0.685502]
EXP_DUR = [4.485, 3.985, 3.150]

def optimize_q4():
    t0 = time.time()
    all_bp = []
    for di in range(3):
        th = radians(THS_DEG[di]); v = VS[di]; td = TDS[di]; tau = TAUS[di]
        d = np.array([cos(th), sin(th), 0.])
        bp = F_INIT[di] + v*(td+tau)*d - np.array([0.,0.,0.5*GRAVITY*tau**2])
        all_bp.append((bp, td+tau))
    total, intervals = solve_shielding_duration_sum(all_bp, 0, dt_coarse=0.005)
    return total, intervals, time.time()-t0


def write_result2(total, intervals, filename="result2.xlsx"):
    """Write Q4 results to result2.xlsx matching template format."""
    wb = openpyxl.Workbook()
    ws = wb.active; ws.title = "Sheet1"
    ws.append(["无人机编号", "无人机运动方向", "无人机运动速度 (m/s)",
               "烟幕干扰弹投放点的x坐标 (m)", "烟幕干扰弹投放点的y坐标 (m)", "烟幕干扰弹投放点的z坐标 (m)",
               "烟幕干扰弹起爆点的x坐标 (m)", "烟幕干扰弹起爆点的y坐标 (m)", "烟幕干扰弹起爆点的z坐标 (m)",
               "有效干扰时长 (s)"])
    for di in range(3):
        th = math.radians(THS_DEG[di]); v = VS[di]; td = TDS[di]; tau = TAUS[di]
        tb = td + tau
        ct, st = cos(th), sin(th)
        rx = F_INIT[di][0] + v*ct*td; ry = F_INIT[di][1] + v*st*td; rz = F_INIT[di][2]
        bx = F_INIT[di][0] + v*ct*tb; by = F_INIT[di][1] + v*st*tb
        bz = F_INIT[di][2] - 0.5*GRAVITY*tau*tau
        dur_di = 0.0
        if di < len(intervals): dur_di = intervals[di][1] - intervals[di][0]
        ws.append([f"FY{di+1}", f"{THS_DEG[di]:.6f}", f"{VS[di]:.6f}",
                   f"{rx:.2f}", f"{ry:.2f}", f"{rz:.2f}",
                   f"{bx:.2f}", f"{by:.2f}", f"{bz:.2f}", f"{dur_di:.6f}"])
    ws.append(["", "", "", "", "", "", "", "", "", ""])
    ws.append(["", "注：以x轴为正向，逆时针方向为正，取值0~360（度）。", "", "", "", "", "", "", "", ""])
    wb.save(filename)
    print(f"  result2.xlsx 已保存")

if __name__ == "__main__":
    print("=" * 60)
    print("  问题四 -- 三机协同")
    print("=" * 60)
    total, intervals, elapsed = optimize_q4()
    print(f"  FY1: {EXP_DUR[0]:.4f} s   FY2: {EXP_DUR[1]:.4f} s   FY3: {EXP_DUR[2]:.4f} s")
    print(f"  联合遮蔽总时长 = {total:.6f} s  (共 {len(intervals)} 段)")
    for i, (ts, te) in enumerate(intervals):
        print(f"    第{i+1}段: [{ts:.6f}, {te:.6f}]  时长 {te-ts:.6f} s")
    print(f"  运行耗时 = {elapsed:.0f} s")
    write_result2(total, intervals)
    print("=" * 60)
