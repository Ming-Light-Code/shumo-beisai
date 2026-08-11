#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Q4: 3 drones (FY1, FY2, FY3) each with 1 grenade vs M1.
Verification of the paper published optimal strategy.
"""

import numpy as np
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *

M1_INIT = MISSILES_INIT["M1"]
DRONE_NAMES = ["FY1", "FY2", "FY3"]
DRONE_INITS = [DRONES_INIT[n] for n in DRONE_NAMES]
T_MAX = missile_flight_time(M1_INIT)

DT = 0.02
N_STEPS = int(T_MAX / DT) + 1
T_GRID = np.linspace(0, T_MAX, N_STEPS)
M1_TRAJ = np.array([missile_position(M1_INIT, t) for t in T_GRID])

print(f"M1 trajectory: {N_STEPS} steps")

def occ(drone_init, alpha, v, td, tb):
    if tb >= T_MAX: return 0.0
    i0 = int(tb / DT)
    i1 = min(N_STEPS, int(min(tb+TMAX, T_MAX)/DT)+1)
    if i0 >= i1: return 0.0
    det_pos = compute_detonation_pos(drone_init, alpha, v, td, tb)
    cnt = 0
    for i in range(i0, i1):
        sc = smoke_center_pos(det_pos, tb, T_GRID[i])
        if check_occlusion_at_time(M1_TRAJ[i], sc, TARGET_SAMPLE_PTS):
            cnt += 1
    return cnt * DT

def union_occ(strategies):
    """strategies: list of (drone_init, alpha, v, td, tb)"""
    ts = [s[4] for s in strategies]
    t_start = min(ts)
    i0 = int(t_start / DT)
    i1 = min(N_STEPS, int(T_MAX/DT)+1)
    if i0 >= i1: return 0.0
    union = np.zeros(i1-i0, dtype=bool)
    for di, a, v, td, tb in strategies:
        det_pos = compute_detonation_pos(di, a, v, td, tb)
        for idx in range(i0, i1):
            t = T_GRID[idx]
            if t < tb or t > tb+TMAX: continue
            sc = smoke_center_pos(det_pos, tb, t)
            if check_occlusion_at_time(M1_TRAJ[idx], sc, TARGET_SAMPLE_PTS):
                union[idx-i0] = True
    return float(np.sum(union)) * DT


def solve_q4():
    """Verify paper Q4 optimal strategy and try coarse refinement."""
    print("=" * 70)
    print("  Q4: 3 drones (FY1+FY2+FY3) each 1 grenade vs M1")
    print(f"  M1 flight: {T_MAX:.1f}s")
    print("=" * 70)
    print()

    # Paper Q4 optimal parameters
    # FY1: a=179.11, v=132.06, td=0.369, tb=3.637
    # FY2: a=308.05, v=138.99, td=8.337, tb=12.501
    # FY3: a=73.81,  v=137.41, td=22.724, tb=23.409
    paper = [
        (DRONES_INIT["FY1"], np.radians(179.110941), 132.062839, 0.369336, 3.636731),
        (DRONES_INIT["FY2"], np.radians(308.046683), 138.991477, 8.336915, 12.500546),
        (DRONES_INIT["FY3"], np.radians(73.809276),  137.409810, 22.723803, 23.409306),
    ]

    print("  [Verification] Paper published optimal strategy:")
    for i, (di, a, v, td, tb) in enumerate(paper):
        oc_s = occ(di, a, v, td, tb)
        det_pos = compute_detonation_pos(di, a, v, td, tb)
        drop_pos = drone_position(di, a, v, td)
        print(f"    {DRONE_NAMES[i]}: a={np.degrees(a):.4f} v={v:.2f} td={td:.4f} tb={tb:.4f}")
        print(f"        drop=({drop_pos[0]:.1f},{drop_pos[1]:.1f},{drop_pos[2]:.1f}) det=({det_pos[0]:.1f},{det_pos[1]:.1f},{det_pos[2]:.1f}) occ={oc_s:.4f}s")
        ok, msg = check_grenade_constraints(di, a, v, td, tb)
        print(f"        constraints: {msg}")
    paper_occ = union_occ(paper)
    print(f"    Union occlusion: {paper_occ:.4f}s")
    print(f"    (Paper reports: 4.485 + 3.985 + 3.150 = 11.620s)")
    print()

    # Coarse local refinement
    print("  [Refinement] Local coordinate descent around paper solution...")
    t0 = time.time()
    best = [list(p) for p in paper]
    best_occ = union_occ(best)
    for step in [0.5, 0.1, 0.05, 0.01]:
        imp = True
        while imp:
            imp = False
            for di in range(3):
                for pi in range(1, 5):
                    for sgn in [-1, 1]:
                        new = [list(s) for s in best]
                        new[di][pi] += sgn*step
                        if pi == 1: new[di][1] %= (2*np.pi)
                        if pi == 2: new[di][2] = np.clip(new[di][2], 70, 140)
                        if pi == 3: new[di][3] = max(0, new[di][3])
                        if pi == 4:
                            if new[di][4] <= new[di][3]: continue
                            if new[di][4] >= T_MAX: continue
                        noc = union_occ(new)
                        if noc > best_occ + 1e-6:
                            best, best_occ = new, noc; imp = True
                if not imp: break
    t1 = time.time()
    print(f"    Done ({t1-t0:.1f}s)")
    print()

    print("=" * 70)
    print("  Q4 RESULT")
    print("=" * 70)
    for i, (di, a, v, td, tb) in enumerate(best):
        det_pos = compute_detonation_pos(di, a, v, td, tb)
        drop_pos = drone_position(di, a, v, td)
        oc_s = occ(di, a, v, td, tb)
        print(f"  {DRONE_NAMES[i]}: a={np.degrees(a):.4f} v={v:.4f} td={td:.6f} tb={tb:.6f}")
        print(f"    drop=({drop_pos[0]:.1f},{drop_pos[1]:.1f},{drop_pos[2]:.1f}) det=({det_pos[0]:.1f},{det_pos[1]:.1f},{det_pos[2]:.1f})")
        print(f"    single occ={oc_s:.4f}s")
    print(f"  Union occlusion: {best_occ:.6f}s")
    print("=" * 70)

    return best_occ

if __name__ == "__main__":
    result = solve_q4()
    print(f"Q4: {result:.4f}s")
