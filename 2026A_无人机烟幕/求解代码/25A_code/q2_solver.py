#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Q2: FY1 + 1 grenade vs M1 - optimized strategy.
Uses pure-Python DE for global search + coordinate-descent local refinement.
"""

import numpy as np
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
from pure_optimize import differential_evolution, minimize

M1_INIT = MISSILES_INIT["M1"]
FY1_INIT = DRONES_INIT["FY1"]
T_MAX = missile_flight_time(M1_INIT)

DT = 0.05
N_STEPS = int(T_MAX / DT) + 1
T_GRID = np.linspace(0, T_MAX, N_STEPS)
M1_TRAJ = np.array([missile_position(M1_INIT, t) for t in T_GRID])

print(f"M1 trajectory: {N_STEPS} steps at dt={DT}s")

def occlusion_fast(alpha, v, td, tb):
    """Fast occlusion duration for a single (alpha, v, td, tb)."""
    if tb >= T_MAX: return 0.0
    i0 = int(tb / DT)
    i1 = min(N_STEPS, int(min(tb + TMAX, T_MAX) / DT) + 1)
    if i0 >= i1: return 0.0
    det_pos = compute_detonation_pos(FY1_INIT, alpha, v, td, tb)
    cnt = 0
    for i in range(i0, i1):
        sc = smoke_center_pos(det_pos, tb, T_GRID[i])
        if check_occlusion_at_time(M1_TRAJ[i], sc, TARGET_SAMPLE_PTS):
            cnt += 1
    return cnt * DT


def objective(x):
    """Objective for minimization: negative occlusion + penalty."""
    alpha, v, td, tb = x
    # Hard penalty for constraint violations
    if v < 70 or v > 140: return 1e6
    if td < 0: return 1e6
    if tb <= td: return 1e6
    if tb >= T_MAX: return 1e6
    zb = FY1_INIT[2] - 0.5 * G * (tb - td)**2
    if zb < -10: return 1e6
    return -occlusion_fast(alpha, v, td, tb)


def solve_q2():
    print("=" * 65)
    print("  Q2: FY1 + 1 grenade vs M1 (DE global + CD local)")
    print(f"  M1 flight: {T_MAX:.1f}s")
    print("=" * 65)
    print()

    t0 = time.time()

    # ---- Stage 1: Differential Evolution global search ----
    print("  [Stage 1] Differential Evolution (popsize=40, maxiter=150)...")
    bounds = [
        (0, 2*np.pi),         # alpha
        (70, 140),            # v
        (0, min(15, T_MAX)),  # td
        (0.01, T_MAX - 1),    # tb
    ]

    de_result = differential_evolution(
        objective, bounds,
        popsize=40,
        maxiter=150,
        tol=1e-4,
        seed=42,
        disp=True,
        polish=True
    )

    t1 = time.time()
    print(f"  DE finished in {t1-t0:.1f}s")
    print(f"  DE best: neg-obj={de_result.fun:.4f}")
    print()

    # ---- Stage 2: Multi-start local refinement from DE elite ----
    print("  [Stage 2] Multi-start coordinate-descent refinement...")
    
    # Build constraints for penalized minimize
    constraints = [
        {"type": "ineq", "fun": lambda x: x[3] - x[2] - 1e-6},
        {"type": "ineq", "fun": lambda x: x[2] - 0},
        {"type": "ineq", "fun": lambda x: FY1_INIT[2] - 0.5*G*(x[3]-x[2])**2 + 1e-6},
    ]

    # Refine DE result
    x_opt, occ_opt = de_result.x.copy(), -de_result.fun

    for _ in range(3):
        # Perturb and refine
        x_start = x_opt.copy()
        x_start[0] += np.random.uniform(-np.pi/6, np.pi/6)
        x_start[1] += np.random.uniform(-10, 10)
        x_start = np.clip(x_start, [b[0] for b in bounds], [b[1] for b in bounds])
        x_start[2] = max(0, x_start[2])
        x_start[3] = max(x_start[2]+0.01, min(x_start[3], T_MAX-1))

        neg_obj = lambda x: -occlusion_fast(x[0], x[1], x[2], x[3])
        res = minimize(neg_obj, x_start, bounds=bounds,
                      constraints=constraints,
                      options={"maxiter": 300})
        occ = -res.fun
        if occ > occ_opt:
            x_opt = res.x.copy()
            occ_opt = occ

    t2 = time.time()
    print(f"  Refinement done ({t2-t1:.1f}s)")
    print(f"  Best occlusion: {occ_opt:.4f}s")
    print()


    # ---- Stage 3: Fine-grid sweep around optimum (dt=0.01) ---- 
    print("  [Stage 3] Fine-grid sweep (dt=0.01) around best...")
    # Recompute with finer time resolution
    DT_FINE = 0.01
    N_FINE = int(T_MAX / DT_FINE) + 1
    T_FINE = np.linspace(0, T_MAX, N_FINE)
    M1_FINE = np.array([missile_position(M1_INIT, t) for t in T_FINE])
    
    def occ_fine(alpha, v, td, tb):
        if tb >= T_MAX: return 0.0
        i0 = int(tb / DT_FINE)
        i1 = min(N_FINE, int(min(tb + TMAX, T_MAX) / DT_FINE) + 1)
        if i0 >= i1: return 0.0
        det_pos = compute_detonation_pos(FY1_INIT, alpha, v, td, tb)
        cnt = 0
        for i in range(i0, i1):
            sc = smoke_center_pos(det_pos, tb, T_FINE[i])
            if check_occlusion_at_time(M1_FINE[i], sc, TARGET_SAMPLE_PTS):
                cnt += 1
        return cnt * DT_FINE
    
    alpha_o, v_o, td_o, tb_o = x_opt
    t3 = time.time()
    # Fine sweep around best (alpha, v, td, tb)
    best_fine = occ_opt
    for da in np.linspace(-2, 2, 21):
        for dv in np.linspace(-10, 10, 21):
            for dtb in np.linspace(-0.1, 0.3, 21):
                na = (alpha_o + np.radians(da)) % (2*np.pi)
                nv = np.clip(v_o + dv, 70, 140)
                ntd = max(0, td_o)
                ntb = np.clip(tb_o + dtb, ntd + 0.001, T_MAX - 1)
                of = occ_fine(na, nv, ntd, ntb)
                if of > best_fine:
                    best_fine = of
                    alpha_o, v_o, td_o, tb_o = na, nv, ntd, ntb
                    occ_opt = best_fine
    t4 = time.time()
    print(f"  Fine sweep done ({t4-t3:.1f}s), best={occ_opt:.4f}s")
    print()


    alpha_o, v_o, td_o, tb_o = x_opt
    det_pos = compute_detonation_pos(FY1_INIT, alpha_o, v_o, td_o, tb_o)
    drop_pos = drone_position(FY1_INIT, alpha_o, v_o, td_o)

    print("=" * 65)
    print("  Q2 RESULT")
    print("=" * 65)
    print(f"  alpha:      {np.degrees(alpha_o):.6f} deg ({alpha_o:.6f} rad)")
    print(f"  speed:      {v_o:.4f} m/s")
    print(f"  drop td:    {td_o:.6f} s")
    print(f"  detonate:   {tb_o:.6f} s")
    print(f"  fall time:  {tb_o - td_o:.6f} s")
    print(f"  drop pos:   ({drop_pos[0]:.2f}, {drop_pos[1]:.2f}, {drop_pos[2]:.2f})")
    print(f"  det pos:    ({det_pos[0]:.2f}, {det_pos[1]:.2f}, {det_pos[2]:.2f})")
    print(f"  occlusion:  {occ_opt:.6f} s")
    ok, msg = check_grenade_constraints(FY1_INIT, alpha_o, v_o, td_o, tb_o)
    print(f"  constraints: {msg}")
    print(f"  total time: {time.time()-t0:.1f}s")

    # ---- Verify against paper optimum ----
    print()
    print("  [Verification] Paper optimal strategy:")
    paper_a = np.radians(6.850153)
    paper_v = 131.480767
    paper_td = 0.0
    paper_tb = 0.711947
    paper_occ = occlusion_fast(paper_a, paper_v, paper_td, paper_tb)
    det_p = compute_detonation_pos(FY1_INIT, paper_a, paper_v, paper_td, paper_tb)
    print(f"    alpha={np.degrees(paper_a):.6f}deg v={paper_v:.4f} td={paper_td} tb={paper_tb:.6f}")
    print(f"    det=({det_p[0]:.2f},{det_p[1]:.2f},{det_p[2]:.2f}) occ={paper_occ:.6f}s")
    print(f"    (Paper reports: 4.955000s)")
    print("=" * 65)

    return occ_opt


if __name__ == "__main__":
    result = solve_q2()
    print(f"Q2: {result:.6f}s")
