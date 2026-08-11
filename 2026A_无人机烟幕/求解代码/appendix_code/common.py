# -*- coding: utf-8 -*-
"""
common.py —— 问题一至问题四共用模块
包含导弹/无人机运动学、目标采样、遮蔽判据和区间搜索
"""
import numpy as np
from math import sqrt, sin, cos, pi

# ============================================================
#  全局物理常数
# ============================================================
GRAVITY      = 9.8      # 重力加速度 (m/s²)
R_SMOKE       = 10.0     # 烟幕有效半径 (m)
V_MISSILE     = 300.0    # 导弹飞行速度 (m/s)
V_SINK        = 3.0      # 烟幕下沉速度 (m/s)
R_TARGET      = 7.0      # 真目标圆柱底面半径 (m)
H_TARGET      = 10.0     # 真目标圆柱高度 (m)
T_SMOKE_MAX   = 20.0     # 烟幕有效持续时长 (s)
T_HIT         = 67.0     # 导弹命中假目标近似时刻 (s)

# 真目标下底面圆心
TARGET_CENTER = np.array([0.0, 200.0, 0.0])

# 导弹 M1 初始位置与方向 (直指原点)
R_M1_0 = np.array([20000.0, 0.0, 2000.0])
D_M1   = -R_M1_0 / np.linalg.norm(R_M1_0)

# 无人机 FY1 初始位置
FY1_INIT = np.array([17800.0, 0.0, 1800.0])

# ============================================================
#  运动学函数
# ============================================================
def solve_shielding_duration_sum(smoke_params, t_start_scan=0, dt_coarse=0.001, fast=False, t_end_scan=None):
    """Same as solve_shielding_duration but returns sum of ALL intervals, not just longest."""
    if fast: dt_coarse = 0.01
    t_current = t_start_scan
    states = []
    if t_end_scan is None: t_end_scan = T_HIT
    while t_current <= t_end_scan:
        states.append(is_shielded(t_current, smoke_params))
        t_current += dt_coarse
    intervals = []
    in_interval = False; idx_start = 0
    for i, s in enumerate(states):
        if s and not in_interval: idx_start = i; in_interval = True
        elif not s and in_interval: intervals.append((idx_start, i)); in_interval = False
    if in_interval: intervals.append((idx_start, len(states) - 1))
    if not intervals: return 0.0, []
    total = 0.0; refined = []
    for (si, ei) in intervals:
        t_rough_a = t_start_scan + si * dt_coarse
        t_rough_b = t_start_scan + ei * dt_coarse
        t_start = binary_search_boundary(t_rough_a - 2*dt_coarse, t_rough_a + dt_coarse, True, smoke_params)
        t_end = binary_search_boundary(t_rough_b - dt_coarse, t_rough_b + 2*dt_coarse, False, smoke_params)
        total += (t_end - t_start)
        refined.append((t_start, t_end))
    return total, refined
def missile_pos(t):
    """导弹 M1 在时刻 t 的位置"""
    return R_M1_0 + V_MISSILE * D_M1 * t


def drone_pos(t, init_pos, speed, heading_rad):
    """无人机在时刻 t 的位置 (等高度匀速直线飞行)"""
    d = np.array([cos(heading_rad), sin(heading_rad), 0.0])
    return init_pos + speed * t * d


def smoke_burst_pos(init_pos, speed, heading_rad, t_release, delay):
    """烟幕弹起爆位置 (投放后做平抛运动)"""
    d = np.array([cos(heading_rad), sin(heading_rad), 0.0])
    release_pos = init_pos + speed * t_release * d
    return release_pos + d * speed * delay - np.array([0.0, 0.0, 0.5 * GRAVITY * delay**2])


def smoke_center_at(t, burst_pos, t_burst):
    """烟幕云团中心在时刻 t 的位置 (起爆后匀速下沉)"""
    if t < t_burst or t > t_burst + T_SMOKE_MAX:
        return None
    return burst_pos - np.array([0.0, 0.0, V_SINK * (t - t_burst)])


# ============================================================
#  目标采样点生成
# ============================================================
def generate_target_samples(n_circle=256, n_silhouette=128):
    """
    对圆柱目标边界采样: 上底面圆周 + 下底面圆周 + 侧面纵线
    共 n_circle*2 + n_silhouette 个点 (默认 640)
    """
    pts = []
    # 上底面圆周 (z = H_TARGET)
    for i in range(n_circle):
        a = 2 * pi * i / n_circle
        pts.append(np.array([
            TARGET_CENTER[0] + R_TARGET * cos(a),
            TARGET_CENTER[1] + R_TARGET * sin(a),
            H_TARGET
        ]))
    # 下底面圆周 (z = 0)
    for i in range(n_circle):
        a = 2 * pi * i / n_circle
        pts.append(np.array([
            TARGET_CENTER[0] + R_TARGET * cos(a),
            TARGET_CENTER[1] + R_TARGET * sin(a),
            0.0
        ]))
    # 侧面纵线采样
    for i in range(n_silhouette):
        z = H_TARGET * i / max(n_silhouette - 1, 1)
        a = 2 * pi * i / n_silhouette
        pts.append(np.array([
            TARGET_CENTER[0] + R_TARGET * cos(a),
            TARGET_CENTER[1] + R_TARGET * sin(a),
            z
        ]))
    return pts


TARGET_SAMPLES = generate_target_samples(256, 128)


# ============================================================
#  遮蔽判据
# ============================================================
def is_shielded(t, smoke_params):
    """
    判断时刻 t 目标是否被完全遮蔽
    smoke_params: [(burst_pos, t_burst), ...]  各烟幕弹的起爆位置和起爆时刻
    """
    r_m = missile_pos(t)

    # 计算当前时刻所有活跃的烟幕中心
    centers = []
    for bp, tb in smoke_params:
        c = smoke_center_at(t, bp, tb)
        if c is not None:
            centers.append(c)

    if not centers:
        return False

    # 遍历目标采样点
    for P in TARGET_SAMPLES:
        direction = P - r_m
        norm_dist = np.linalg.norm(direction)
        if norm_dist < 1e-10:
            continue
        direction = direction / norm_dist

        occluded = False
        for sc in centers:
            v = sc - r_m
            dist_mc = np.linalg.norm(v)

            # 特殊处理: 导弹位于烟幕内部 → 必然遮蔽
            if dist_mc <= R_SMOKE:
                occluded = True
                break

            # 视线与烟幕球的最短距离
            tstar = np.dot(v, direction)
            if tstar <= 0:
                continue
            closest_point = r_m + tstar * direction
            dist_to_line = np.linalg.norm(closest_point - sc)

            if dist_to_line <= R_SMOKE:
                occluded = True
                break

        if not occluded:
            return False

    return True


# ============================================================
#  二分法精修
# ============================================================
def binary_search_boundary(t_left, t_right, target_shielded, smoke_params, max_iter=55):
    """在 [t_left, t_right] 内二分搜索遮蔽状态切换的时刻"""
    for _ in range(max_iter):
        t_mid = (t_left + t_right) * 0.5
        if is_shielded(t_mid, smoke_params) == target_shielded:
            t_right = t_mid
        else:
            t_left = t_mid
    return (t_left + t_right) * 0.5


# ============================================================
#  完整求解流程: 粗扫 → 二分精修
# ============================================================
def solve_shielding_duration(smoke_params, t_start_scan=0, dt_coarse=0.001, fast=False, t_end_scan=None):
    """
    主求解函数: 返回 (t_start, t_end, duration)
    先以 dt_coarse 步长粗扫定位区间, 再用二分法精修边界
    """
    if fast: dt_coarse = 0.01
    t_current = t_start_scan
    states = []
    if t_end_scan is None: t_end_scan = T_HIT
    sc = 0
    while t_current <= t_end_scan:
        states.append(is_shielded(t_current, smoke_params))
        t_current += dt_coarse

    # 提取连续遮蔽区间
    intervals = []
    in_interval = False
    idx_start = 0
    for i, s in enumerate(states):
        if s and not in_interval:
            idx_start = i
            in_interval = True
        elif not s and in_interval:
            intervals.append((idx_start, i))
            in_interval = False
    if in_interval:
        intervals.append((idx_start, len(states) - 1))

    if not intervals:
        return 0.0, 0.0, 0.0

    # 取最长区间
    best = max(intervals, key=lambda x: x[1] - x[0])
    t_rough_a = t_start_scan + best[0] * dt_coarse
    t_rough_b = t_start_scan + best[1] * dt_coarse

    # 二分精修边界
    t_start = binary_search_boundary(
        t_rough_a - 2 * dt_coarse, t_rough_a + dt_coarse, True, smoke_params
    )
    t_end = binary_search_boundary(
        t_rough_b - dt_coarse, t_rough_b + 2 * dt_coarse, False, smoke_params
    )

    return t_start, t_end, t_end - t_start
