# -*- coding: utf-8 -*-
"""
problem1.py —— 问题一: 单机单弹给定参数下计算有效遮蔽时长

已知条件:
  FY1 以 120 m/s 朝向假目标水平飞行,
  受领任务 1.5s 后投放烟幕弹, 间隔 3.6s 后起爆.
  计算烟幕对 M1 的有效遮蔽时长.

结果: 遮蔽区间 [8.056445, 9.448088], 时长 1.391643 s
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from common import (
    GRAVITY, smoke_burst_pos, solve_shielding_duration
)

# ========== 已知参数 ==========
FY1_INIT   = np.array([17800.0, 0.0, 1800.0])
FY1_SPEED  = 120.0
FY1_HEADING = np.pi           # 朝向假目标 (-x 方向), 即 π rad
T_RELEASE  = 1.5              # 投放时刻 (s)
T_DELAY    = 3.6              # 起爆延迟 (s)
T_BURST    = T_RELEASE + T_DELAY  # 起爆时刻 = 5.1 s

# ========== 计算起爆位置 ==========
burst_pos = smoke_burst_pos(FY1_INIT, FY1_SPEED, FY1_HEADING, T_RELEASE, T_DELAY)

# ========== 求解 ==========
print("=" * 60)
print("问题一: 单机单弹遮蔽时长计算")
print("=" * 60)
print(f"投放时刻:   {T_RELEASE} s")
print(f"起爆延迟:   {T_DELAY} s")
print(f"起爆时刻:   {T_BURST} s")
print(f"起爆位置:   ({burst_pos[0]:.6f}, {burst_pos[1]:.6f}, {burst_pos[2]:.6f})")
print()

smoke_params = [(burst_pos, T_BURST)]
t_start, t_end, duration = solve_shielding_duration(smoke_params, t_start_scan=T_BURST)

print(f"遮蔽开始:   {t_start:.6f} s")
print(f"遮蔽结束:   {t_end:.6f} s")
print(f"有效遮蔽时长: {duration:.6f} s")
print()

# ========== 简化分析 ==========
m1_start_dist = 20100.0  # 导弹初始距假目标约 20.1 km
m1_hit_time   = m1_start_dist / 300.0
print(f"导弹命中假目标时刻: {m1_hit_time:.2f} s")
print(f"遮蔽占飞行窗口: {duration / m1_hit_time * 100:.2f}%")