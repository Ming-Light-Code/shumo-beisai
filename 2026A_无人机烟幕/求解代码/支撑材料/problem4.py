# -*- coding: utf-8 -*-
"""problem4.py -- Q4: 三机协同 已知参数 + 联合累加 + Excel 输出"""
import numpy as np, math, time, sys, os
from pathlib import Path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import solve_shielding_duration_sum, GRAVITY, FY_INIT
from math import cos, sin, pi, radians, degrees

# 以下参数来源: Q4 独立优化求解器输出的最优解
# (在 Q1-Q3 模型基础上扩展为三机协同, 各机独立优化后联合验证)
F_INIT = [FY_INIT[0].copy(), FY_INIT[1].copy(), FY_INIT[2].copy()]
THS_DEG = [179.110941, 308.046683, 73.809276]   # 各机最优航向角 (deg)
VS      = [132.062839, 138.991477, 137.409810]   # 各机最优飞行速度 (m/s)
TDS     = [0.369336,   8.336915,  22.723803]     # 各弹投放时刻 (s)
TAUS    = [3.636731,   4.163631,   0.685502]     # 各弹起爆延迟 (s)
EXP_DUR = [4.485, 3.985, 3.150]                  # 各机单独期望遮蔽时长 (s)

UAV_NAMES = ["FY1", "FY2", "FY3"]


def optimize_q4():
    """计算三机协同联合遮蔽, 返回各机结果和联合结果."""
    t0 = time.time()
    all_bp = []
    per_uav = []  # 各机单独结果

    for di in range(3):
        th = radians(THS_DEG[di])
        v = VS[di]
        td = TDS[di]
        tau = TAUS[di]
        tb = td + tau

        d_vec = np.array([cos(th), sin(th), 0.])
        # 投放点
        drop_pos = F_INIT[di] + v * td * d_vec
        # 起爆点
        bp = F_INIT[di] + v * tb * d_vec - np.array([0., 0., 0.5 * GRAVITY * tau ** 2])
        all_bp.append((bp, tb))

        # 单机遮蔽时长 (仅该机)
        single_bp = [(bp, tb)]
        dur_single, _ = solve_shielding_duration_sum(single_bp, 0, dt_coarse=0.005)
        dur_single = round(dur_single, 4)

        per_uav.append({
            "name": UAV_NAMES[di],
            "dir_deg": round(THS_DEG[di], 6),
            "v": round(v, 6),
            "drop_x": round(float(drop_pos[0]), 2),
            "drop_y": round(float(drop_pos[1]), 2),
            "drop_z": round(float(drop_pos[2]), 2),
            "burst_x": round(float(bp[0]), 2),
            "burst_y": round(float(bp[1]), 2),
            "burst_z": round(float(bp[2]), 2),
            "duration": dur_single,
        })

    # 联合遮蔽总时长
    total, intervals = solve_shielding_duration_sum(all_bp, 0, dt_coarse=0.005)
    elapsed = time.time() - t0

    return per_uav, round(total, 4), intervals, elapsed


def write_result(per_uav: list, total: float, intervals: list) -> None:
    """输出 Excel 结果文件 (参照 A题 result2 格式)."""
    from openpyxl import Workbook

    wb = Workbook()
    ws = wb.active
    ws.title = "Sheet1"

    headers = ["无人机编号", "无人机运动方向", "无人机运动速度 (m/s)",
               "烟幕弹投放点x坐标 (m)", "烟幕弹投放点y坐标 (m)", "烟幕弹投放点z坐标 (m)",
               "烟幕弹起爆点x坐标 (m)", "烟幕弹起爆点y坐标 (m)", "烟幕弹起爆点z坐标 (m)",
               "有效干扰时间 (s)"]
    ws.append(headers)

    for uav in per_uav:
        ws.append([uav["name"], uav["dir_deg"], uav["v"],
                   uav["drop_x"], uav["drop_y"], uav["drop_z"],
                   uav["burst_x"], uav["burst_y"], uav["burst_z"],
                   uav["duration"]])

    # 汇总行
    ws.append([total, "", "", "", "", "", "", "", "", ""])
    # 注释行
    ws.append(["", "注：x轴为正北方向，逆时针方向为正，取值0~360度；", "", "", "", "", "", "", "", ""])

    out = Path(__file__).parent / "result2.xlsx"
    wb.save(str(out))
    print(f"\n结果已保存至 {out}")


if __name__ == "__main__":
    print("=" * 60)
    print("  问题四 -- 三机协同")
    print("=" * 60)

    per_uav, total, intervals, elapsed = optimize_q4()

    for uav in per_uav:
        print(f"  {uav['name']}: 方向={uav['dir_deg']}° 速度={uav['v']} m/s  单机时长={uav['duration']} s")
    print(f"  联合遮蔽总时长 = {total} s  (共 {len(intervals)} 段)")
    for i, (ts, te) in enumerate(intervals):
        print(f"    第{i+1}段: [{ts:.6f}, {te:.6f}]  时长 {te-ts:.6f} s")
    print(f"  运行耗时 = {elapsed:.0f} s")
    print("=" * 60)

    write_result(per_uav, total, intervals)
