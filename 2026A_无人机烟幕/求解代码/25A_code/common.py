#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
25A shared module: kinematics, occlusion checking, sampling points.
Used by Q1-Q4 solvers.
"""

import numpy as np
from numpy import cos, sin, sqrt, arctan2, arcsin, arccos, pi
from typing import Tuple, List

G = 9.8
VM = 300.0
VS = 3.0
RS = 10.0
TMAX = 20.0
V_DRONE_MIN = 70.0
V_DRONE_MAX = 140.0
MIN_INTERVAL = 1.0
RT = 7.0
HT = 10.0
TGT_CENTER = np.array([0.0, 200.0, 5.0])
DECOY = np.array([0.0, 0.0, 0.0])

# ---- missile, drone init positions ----
MISSILES_INIT = {
    "M1": np.array([20000.0, 0.0, 2000.0]),
    "M2": np.array([19000.0, 600.0, 2100.0]),
    "M3": np.array([18000.0, -600.0, 1900.0]),
}

DRONES_INIT = {
    "FY1": np.array([17800.0, 0.0, 1800.0]),
    "FY2": np.array([12000.0, 1400.0, 1400.0]),
    "FY3": np.array([6000.0, -3000.0, 700.0]),
    "FY4": np.array([11000.0, 2000.0, 1800.0]),
    "FY5": np.array([13000.0, -2000.0, 1300.0]),
}

# ============================================================
# Target sample point generation
# ============================================================
def generate_target_sample_points(N_circle=64, N_side=16):
    """Generate sampling points on true target cylinder boundary."""
    points = []
    # Top circle
    for k in range(N_circle):
        angle = 2.0 * pi * k / N_circle
        px = RT * cos(angle)
        py = TGT_CENTER[1] + RT * sin(angle)
        pz = TGT_CENTER[2] + HT / 2.0
        points.append([px, py, pz])
    # Bottom circle
    for k in range(N_circle):
        angle = 2.0 * pi * k / N_circle
        px = RT * cos(angle)
        py = TGT_CENTER[1] + RT * sin(angle)
        pz = TGT_CENTER[2] - HT / 2.0
        points.append([px, py, pz])
    # Side silhouette
    for k in range(N_side):
        z_sample = HT * k / (N_side - 1)
        points.append([RT, TGT_CENTER[1], z_sample])
        points.append([-RT, TGT_CENTER[1], z_sample])
    return np.array(points)

def generate_target_sample_points_dense():
    return generate_target_sample_points(N_circle=64, N_side=16)


# ============================================================
# Kinematics
# ============================================================
def missile_position(missile_init, t):
    """Missile position at time t (constant speed toward decoy)."""
    d = DECOY - missile_init
    dist = np.linalg.norm(d)
    direction = d / dist
    return missile_init + VM * direction * t

def missile_direction(missile_init):
    """Unit direction vector of missile flight."""
    d = DECOY - missile_init
    return d / np.linalg.norm(d)

def missile_flight_time(missile_init):
    """Time for missile to reach decoy."""
    return np.linalg.norm(DECOY - missile_init) / VM

def drone_position(drone_init, alpha, v, t):
    """Drone position at time t (constant-altitude straight flight)."""
    return np.array([
        drone_init[0] + v * cos(alpha) * t,
        drone_init[1] + v * sin(alpha) * t,
        drone_init[2],
    ])

def smoke_pre_detonation_pos(drop_pos, alpha, v, td, t):
    """Smoke grenade trajectory after drop, before detonation."""
    dt = t - td
    return np.array([
        drop_pos[0] + v * cos(alpha) * dt,
        drop_pos[1] + v * sin(alpha) * dt,
        drop_pos[2] - 0.5 * G * dt * dt,
    ])

def smoke_center_pos(detonation_pos, tb, t):
    """Smoke cloud center after detonation (sinking at VS)."""
    dt = t - tb
    return np.array([
        detonation_pos[0],
        detonation_pos[1],
        detonation_pos[2] - VS * dt,
    ])

def compute_detonation_pos(drone_init, alpha, v, td, tb):
    """Compute smoke grenade detonation position."""
    drop_pos = drone_position(drone_init, alpha, v, td)
    return smoke_pre_detonation_pos(drop_pos, alpha, v, td, tb)


# ============================================================
# Occlusion checking (unit-sphere projection method)
# ============================================================
def check_occlusion_at_time(missile_pos, smoke_cntr, target_pts):
    """
    Check if smoke sphere fully occludes the true target at a given instant.
    Uses unit-sphere projection with smoke angular radius and target
    boundary sample points.
    """
    dc = smoke_cntr - missile_pos
    dc_norm = float(np.linalg.norm(dc))
    if dc_norm < 1e-9:
        return True
    dc_u = dc / dc_norm
    # Missile inside smoke sphere = always occluded
    if dc_norm <= RS:
        return True
    theta_s = arcsin(RS / dc_norm)
    for pt in target_pts:
        v = pt - missile_pos
        v_norm = float(np.linalg.norm(v))
        if v_norm < 1e-9:
            continue
        v_u = v / v_norm
        cos_ang = np.clip(float(np.dot(v_u, dc_u)), -1.0, 1.0)
        ang_dist = arccos(cos_ang)
        if ang_dist > theta_s:
            return False
    return True


# ============================================================
# Occlusion interval search (fixed-step scan + bisection)
# ============================================================
def find_occlusion_intervals(missile_init, drone_init,
                             alpha, v, td, tb,
                             t_max, target_pts,
                             dt_scan=0.001, t_tol=1e-12):
    """
    Find occlusion intervals with high precision.
    Step 1: fixed-step scan to locate switch points.
    Step 2: bisection to refine boundaries to ~t_tol precision.
    """
    t_begin = tb
    t_end = min(tb + TMAX, t_max)
    if t_begin >= t_end:
        return []
    n_steps = int((t_end - t_begin) / dt_scan) + 1
    t_grid = np.linspace(t_begin, t_end, n_steps)
    detonation_pos = compute_detonation_pos(drone_init, alpha, v, td, tb)
    occlusion_flags = np.zeros(n_steps, dtype=bool)
    for i, t in enumerate(t_grid):
        mp = missile_position(missile_init, t)
        sc = smoke_center_pos(detonation_pos, tb, t)
        occlusion_flags[i] = check_occlusion_at_time(mp, sc, target_pts)
    # Locate switches
    prev = occlusion_flags[0]
    switches = []
    for i in range(1, n_steps):
        if occlusion_flags[i] != prev:
            switches.append((t_grid[i - 1], t_grid[i], prev))
            prev = occlusion_flags[i]
    # Bisection refinement
    intervals = []
    for t_lo, t_hi, was_occluded in switches:
        t_lo_b = t_lo
        t_hi_b = t_hi
        for _ in range(60):
            t_mid = (t_lo_b + t_hi_b) / 2.0
            mp = missile_position(missile_init, t_mid)
            sc = smoke_center_pos(detonation_pos, tb, t_mid)
            is_occ = check_occlusion_at_time(mp, sc, target_pts)
            if is_occ == was_occluded:
                t_lo_b = t_mid
            else:
                t_hi_b = t_mid
            if t_hi_b - t_lo_b < t_tol:
                break
        boundary = (t_lo_b + t_hi_b) / 2.0
        if was_occluded:
            if intervals:
                intervals[-1] = (intervals[-1][0], boundary)
        else:
            intervals.append((boundary, None))
    if intervals and intervals[-1][1] is None:
        intervals[-1] = (intervals[-1][0], t_end)
    return intervals


# ============================================================
# Fast occlusion duration (for optimization loops)
# ============================================================
def occlusion_duration(missile_init, drone_init,
                       alpha, v, td, tb,
                       target_pts, dt=0.01):
    """Quick occlusion duration via discrete scan (no bisection)."""
    t_max = missile_flight_time(missile_init)
    t_begin = tb
    t_end = min(tb + TMAX, t_max)
    if t_begin >= t_end:
        return 0.0
    n_steps = int((t_end - t_begin) / dt) + 1
    if n_steps <= 0:
        return 0.0
    t_grid = np.linspace(t_begin, t_end, n_steps)
    detonation_pos = compute_detonation_pos(drone_init, alpha, v, td, tb)
    count = 0
    for t in t_grid:
        mp = missile_position(missile_init, t)
        sc = smoke_center_pos(detonation_pos, tb, t)
        if check_occlusion_at_time(mp, sc, target_pts):
            count += 1
    return count * dt


# ============================================================
# Multi-grenade union occlusion duration
# ============================================================
def occlusion_duration_union(missile_init, grenades,
                             target_pts, dt=0.01):
    """
    Compute union occlusion duration for multiple smoke grenades.
    grenades: list of (drone_init, alpha, v, td, tb) tuples.
    """
    t_max = missile_flight_time(missile_init)
    t_begin = min(g[4] for g in grenades)
    t_end = t_max
    if t_begin >= t_end:
        return 0.0
    n_steps = int((t_end - t_begin) / dt) + 1
    if n_steps <= 0:
        return 0.0
    t_grid = np.linspace(t_begin, t_end, n_steps)
    union_occ = np.zeros(n_steps, dtype=bool)
    for drone_init, alpha, v, td, tb in grenades:
        det_pos = compute_detonation_pos(drone_init, alpha, v, td, tb)
        for i, t in enumerate(t_grid):
            if t < tb or t > tb + TMAX:
                continue
            mp = missile_position(missile_init, t)
            sc = smoke_center_pos(det_pos, tb, t)
            if check_occlusion_at_time(mp, sc, target_pts):
                union_occ[i] = True
    return float(np.sum(union_occ)) * dt


# ============================================================
# Constraint verification
# ============================================================
def check_grenade_constraints(drone_init, alpha, v, td, tb):
    """Check if a single-grenade strategy satisfies all constraints."""
    if v < V_DRONE_MIN or v > V_DRONE_MAX:
        return False, "speed out of range"
    if td < 0:
        return False, "td < 0"
    if tb <= td:
        return False, "tb <= td"
    zb = drone_init[2] - 0.5 * G * (tb - td) ** 2
    if zb < -10:
        return False, "detonation below ground"
    return True, "OK"


# ============================================================
# Pre-computed target sample points (global)
# ============================================================
TARGET_SAMPLE_PTS = generate_target_sample_points_dense()

if __name__ == "__main__":
    print(f"Target sample points: {len(TARGET_SAMPLE_PTS)}")
    print(f"M1 flight time: {missile_flight_time(MISSILES_INIT['M1']):.2f} s")
    print(f"M2 flight time: {missile_flight_time(MISSILES_INIT['M2']):.2f} s")
    print(f"M3 flight time: {missile_flight_time(MISSILES_INIT['M3']):.2f} s")
    print("Common module loaded OK.")
