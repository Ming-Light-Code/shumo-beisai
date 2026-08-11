# -*- coding: utf-8 -*-
"""
fast_eval.py —— 快速向量化遮蔽评估模块 (Q2-Q4 共用)

预计算导弹轨迹并在 numpy 广播上实现高效遮蔽判定。
提供中心线近似 (粗筛) 和球投影全量判定 (精校) 两级精度。

依赖: common.py (所有物理常数统一来源)
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from math import cos, sin, pi, sqrt, atan2

# ---- 从 common 导入所有共享常数 ----
from common import (
    GRAVITY, R_SMOKE, V_MISSILE, V_SINK, T_SMOKE_MAX,
    V_MIN, V_MAX, TARGET_CENTER, TARGET_POINT, ORIGIN,
    M0, M_DIR, FY_INIT,
    R_TARGET, H_TARGET,
)

# ============================================================
#  局部别名 (兼容现有代码的短变量名)
# ============================================================
G   = GRAVITY        # 重力加速度 g
MS  = V_MISSILE      # 导弹速度 Missile Speed
CS  = V_SINK         # 烟幕下沉速度 Cloud Sink
CR  = R_SMOKE        # 烟幕有效半径 Cloud Radius
CD  = T_SMOKE_MAX    # 烟幕有效持续 Cloud Duration
LI  = 1.0            # 投放间隔下限 Lower Interval (s)
DTS = 0.1            # 评估时间步长 Delta Time Step (s)

VMIN = V_MIN         # 无人机最小速度
VMAX = V_MAX         # 无人机最大速度

TT = TARGET_CENTER.copy()          # 真目标中心 Target Top-center
FT = ORIGIN.copy()                 # 假目标原点 Fake Target

MI = M0.copy()                     # 导弹初始位置 Missile Initial
MN = ["M1", "M2", "M3"]            # 导弹名称 Missile Name

DI = FY_INIT.copy()                # 无人机初始位置 Drone Initial
DN = ["FY1", "FY2", "FY3", "FY4", "FY5"]  # 无人机名称 Drone Name

ND = 5     # 无人机数量 Number of Drones
NM = 3     # 导弹数量 Number of Missiles
RT = R_TARGET  # 目标半径 Radius of Target
HT = H_TARGET  # 目标高度 Height of Target

# ============================================================
#  预计算: 导弹轨迹 + 视线参数
# ============================================================
# 导弹方向单位矢量 (从 common 导入, 此处保留本地变量名)
MD = M_DIR.copy()                   # Missile Direction

# 最大仿真时间
TMAX = max(np.linalg.norm(FT - MI[k]) for k in range(NM)) / MS
NTS = int(TMAX / DTS) + 1           # 时间步数 Number of Time Steps
TA  = np.arange(NTS) * DTS          # 时间轴 Time Axis

# 预计算所有时间点所有导弹的位置 MP[missile, time, xyz]
MP = np.zeros((NM, NTS, 3))
for k in range(NM):
    for ti in range(NTS):
        MP[k, ti] = MI[k] + TA[ti] * MS * MD[k]

# 预计算视线参数 LP[missile, time, param]
# param[0:3] = 目标方向向量, param[3:6] = 导弹位置
LP = np.zeros((NM, NTS, 6))
for k in range(NM):
    for ti in range(NTS):
        mx, my, mz = MP[k, ti]
        LP[k, ti, 0] = TT[0] - mx   # 目标方向 x
        LP[k, ti, 1] = TT[1] - my   # 目标方向 y
        LP[k, ti, 2] = TT[2] - mz   # 目标方向 z
        LP[k, ti, 3] = mx           # 导弹位置 x
        LP[k, ti, 4] = my           # 导弹位置 y
        LP[k, ti, 5] = mz           # 导弹位置 z


# ============================================================
#  目标采样点生成
# ============================================================
def _gen_pts(nc, ns):
    """
    生成圆柱目标边界采样点 (简化策略, 侧重速度).
    nc: 上下底面圆周采样点数
    ns: 侧面纵线采样点数 (仅左右两条纵线 x=±RT, 各 ns 点)
    返回 (2*nc + 2*ns, 3) 数组.

    注: 此简化策略在保证遮蔽判据正确性的前提下,
    用左右两条纵线替代 common.generate_target_samples 的螺旋线,
    点数更少 (默认 384 vs 640), 用于 Q2/Q3 的快速评估.
    """
    pts = np.zeros((2 * nc + 2 * ns, 3))
    idx = 0
    # 上底面圆周
    for k in range(nc):
        a = 2.0 * np.pi * k / nc
        pts[idx] = [RT * np.cos(a), TT[1] + RT * np.sin(a), TT[2] + HT]
        idx += 1
    # 下底面圆周
    for k in range(nc):
        a = 2.0 * np.pi * k / nc
        pts[idx] = [RT * np.cos(a), TT[1] + RT * np.sin(a), TT[2]]
        idx += 1
    # 侧面纵线 (左右各 ns 条)
    for k in range(ns):
        z = HT * k / (ns - 1)
        pts[idx] = [-RT, TT[1], z]
        idx += 1
    for k in range(ns):
        z = HT * k / (ns - 1)
        pts[idx] = [RT, TT[1], z]
        idx += 1
    return pts


PTS   = _gen_pts(128, 64)           # 预生成的目标采样点
M_PTS = len(PTS)                    # 采样点数量


# ============================================================
#  运动学
# ============================================================
def smoke_burst_pos_fast(init_pos, speed, heading, tr, delay):
    """烟幕弹起爆位置 (与 common.smoke_burst_pos 等价, 保留用于独立运行)"""
    d = np.array([cos(heading), sin(heading), 0.0])
    return init_pos + speed * (tr + delay) * d - np.array([0.0, 0.0, 0.5 * G * delay * delay])


# ============================================================
#  中心线粗筛 (速度快 ~100x)
# ============================================================
def fast_shield_center(di, th, v, tk, tr, tf):
    """
    中心线近似遮蔽判定 (极快, 用于候选初筛).
    di: 无人机索引, th: 航向角(rad), v: 速度(m/s)
    tk: 目标导弹索引, tr: 投放时刻(s), tf: 下落时间(s)
    返回: 近似遮蔽时长 (s)
    """
    r0 = DI[di]
    ct = cos(th); st = sin(th)
    td = tr + tf                                  # 起爆时刻
    xd = r0[0] + v * ct * td                      # 起爆 x
    yd = r0[1] + v * st * td                      # 起爆 y
    zd = r0[2] - 0.5 * G * tf * tf                # 起爆 z

    tsi = max(0, int(td / DTS))                   # 起始时间索引
    tei = min(NTS, int((td + CD) / DTS) + 1)      # 结束时间索引
    if tei <= tsi:
        return 0.0

    ta_s = TA[tsi:tei]
    active = ta_s >= td
    if not np.any(active):
        return 0.0

    ta_a = ta_s[active]
    cz = zd - CS * (ta_a - td)                     # 烟幕中心 z (随时间下沉)

    # 从预计算 LP 中提取视线参数
    ttx = LP[tk, tsi:tei, 0][active]   # 目标方向 x
    tty = LP[tk, tsi:tei, 1][active]   # 目标方向 y
    ttz = LP[tk, tsi:tei, 2][active]   # 目标方向 z
    mpx = LP[tk, tsi:tei, 3][active]   # 导弹位置 x
    mpy = LP[tk, tsi:tei, 4][active]   # 导弹位置 y
    mpz = LP[tk, tsi:tei, 5][active]   # 导弹位置 z

    # 向量化: 投影参数 λ, 最近点, 距离判定
    wcx = xd - mpx; wcy = yd - mpy; wcz = cz - mpz
    c2 = ttx * ttx + tty * tty + ttz * ttz + 1e-10
    c1 = wcx * ttx + wcy * tty + wcz * ttz
    s = np.clip(c1 / c2, 0.0, 1.0)
    px = mpx + s * ttx; py = mpy + s * tty; pz = mpz + s * ttz

    return float(np.sum((xd - px)**2 + (yd - py)**2 + (cz - pz)**2 < CR * CR)) * DTS


# ============================================================
#  球投影全量判定 (精确)
# ============================================================
def sphere_occluded(mpos, cpos):
    """
    判断烟幕球是否从 mpos 完全遮挡所有目标采样点 PTS.
    返回 True/False.
    """
    mx, my, mz = mpos
    cx, cy, cz = cpos
    wcx, wcy, wcz = cx - mx, cy - my, cz - mz
    dn = sqrt(wcx * wcx + wcy * wcy + wcz * wcz)
    if dn < 1e-10 or dn <= CR:
        return True                     # 导弹在烟幕内部 → 全遮蔽

    dc = np.array([wcx / dn, wcy / dn, wcz / dn])
    cth = sqrt(max(0.0, 1.0 - (CR / dn)**2))

    vx = PTS[:, 0] - mx
    vy = PTS[:, 1] - my
    vz = PTS[:, 2] - mz
    vn = np.sqrt(vx * vx + vy * vy + vz * vz) + 1e-10

    return float(np.min((vx * dc[0] + vy * dc[1] + vz * dc[2]) / vn)) >= cth


def sphere_shield_dur(di, th, v, tk, tr, tf):
    """
    单枚烟幕弹精确遮蔽时长 (球投影全量判定, 慢但准确).
    di: 无人机索引, th: 航向角(rad), v: 速度(m/s)
    tk: 目标导弹索引, tr: 投放时刻(s), tf: 下落时间(s)
    返回: 精确遮蔽时长 (s)
    """
    r0 = DI[di]
    ct = cos(th); st = sin(th)
    td = tr + tf
    xd = r0[0] + v * ct * td
    yd = r0[1] + v * st * td
    zd = r0[2] - 0.5 * G * tf * tf

    tsi = max(0, int(td / DTS))
    tei = min(NTS, int((td + CD) / DTS) + 1)
    if tei <= tsi:
        return 0.0

    ta_s = TA[tsi:tei]
    active = ta_s >= td
    if not np.any(active):
        return 0.0

    ta_a = ta_s[active]
    total = 0.0
    act_idx = np.where(active)[0]
    for idx_a in range(len(ta_a)):
        ti = tsi + act_idx[idx_a]
        t = ta_a[idx_a]
        if sphere_occluded(MP[tk, ti], np.array([xd, yd, zd - CS * (t - td)])):
            total += DTS
    return total


# ============================================================
#  多弹联合遮蔽
# ============================================================
def multi_shield_center(di, th, v, tk, grenades):
    """
    多枚烟幕弹联合遮蔽 (中心线粗筛).
    grenades: [(t_drop, tau), ...]  各弹投放时刻和下落时间
    返回: 联合遮蔽时长 (s)
    """
    shielded = np.zeros(NTS, dtype=bool)
    for g in grenades:
        tr, tf = g[0], g[1]
        td = tr + tf
        r0 = DI[di]
        ct = cos(th); st = sin(th)
        xd = r0[0] + v * ct * td
        yd = r0[1] + v * st * td
        zd = r0[2] - 0.5 * G * tf * tf

        tsi = max(0, int(td / DTS))
        tei = min(NTS, int((td + CD) / DTS) + 1)
        if tei <= tsi:
            continue

        ta_s = TA[tsi:tei]
        active = (ta_s >= td)
        if not np.any(active):
            continue

        ta_a = ta_s[active]
        cz = zd - CS * (ta_a - td)

        ttx = LP[tk, tsi:tei, 0][active]
        tty = LP[tk, tsi:tei, 1][active]
        ttz = LP[tk, tsi:tei, 2][active]
        mpx = LP[tk, tsi:tei, 3][active]
        mpy = LP[tk, tsi:tei, 4][active]
        mpz = LP[tk, tsi:tei, 5][active]

        wcx = xd - mpx; wcy = yd - mpy; wcz = cz - mpz
        c2 = ttx * ttx + tty * tty + ttz * ttz + 1e-10
        c1 = wcx * ttx + wcy * tty + wcz * ttz
        s = np.clip(c1 / c2, 0.0, 1.0)
        px = mpx + s * ttx; py = mpy + s * tty; pz = mpz + s * ttz

        idxs = np.where(active)[0]
        for j, orig_i in enumerate(idxs):
            shielded[tsi + orig_i] |= (xd - px[j])**2 + (yd - py[j])**2 + (cz[j] - pz[j])**2 < CR * CR

    return float(np.sum(shielded)) * DTS


def multi_sphere_shield(di, th, v, tk, grenades):
    """
    多枚烟幕弹联合遮蔽 (球投影全量判定).
    grenades: [(t_drop, tau), ...]  各弹投放时刻和下落时间
    返回: 联合遮蔽时长 (s)
    """
    shielded = np.zeros(NTS, dtype=bool)
    for g in grenades:
        tr, tf = g[0], g[1]
        td = tr + tf
        r0 = DI[di]
        ct = cos(th); st = sin(th)
        xd = r0[0] + v * ct * td
        yd = r0[1] + v * st * td
        zd = r0[2] - 0.5 * G * tf * tf

        tsi = max(0, int(td / DTS))
        tei = min(NTS, int((td + CD) / DTS) + 1)
        if tei <= tsi:
            continue

        ta_s = TA[tsi:tei]
        active = (ta_s >= td)
        if not np.any(active):
            continue

        ta_a = ta_s[active]
        act_idx = np.where(active)[0]
        for idx_a in range(len(ta_a)):
            ti = tsi + act_idx[idx_a]
            t = ta_a[idx_a]
            if shielded[ti]:
                continue
            if sphere_occluded(MP[tk, ti], np.array([xd, yd, zd - CS * (t - td)])):
                shielded[ti] = True

    return float(np.sum(shielded)) * DTS
