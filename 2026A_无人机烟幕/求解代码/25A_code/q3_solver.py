#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Q3: Single drone FY1 with 3 smoke grenades vs M1.
Fixed heading and speed, optimize td/tb for 3 grenades.
Optimized grid search without scipy dependency.
"""

import numpy as np
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *

M1_INIT = MISSILES_INIT["M1"]
FY1_INIT = DRONES_INIT["FY1"]
T_MAX = missile_flight_time(M1_INIT)

# Fixed heading and speed (from paper Q2 optimal, used for Q3)
ALPHA_Q3 = np.radians(179.814307)
V_Q3 = 139.843372

# Pre-compute M1 trajectory for speed
DT_FAST = 0.05
N_FAST = int(T_MAX / DT_FAST) + 1
T_FAST = np.linspace(0, T_MAX, N_FAST)
M1_TRAJ = np.array([missile_position(M1_INIT, t) for t in T_FAST])

def quick_occlusion(td, tb):
    """Fast single-grenade occlusion using pre-computed M1 trajectory."""
    if tb >= T_MAX:
        return 0.0
    t0_idx = int(tb / DT_FAST)
    t1_idx = min(N_FAST, int(min(tb + TMAX, T_MAX) / DT_FAST) + 1)
    if t0_idx >= t1_idx:
        return 0.0
    det_pos = compute_detonation_pos(FY1_INIT, ALPHA_Q3, V_Q3, td, tb)
    count = 0
    for i in range(t0_idx, t1_idx):
        t = T_FAST[i]
        mp = M1_TRAJ[i]
        sc = smoke_center_pos(det_pos, tb, t)
        if check_occlusion_at_time(mp, sc, TARGET_SAMPLE_PTS):
            count += 1
    return count * DT_FAST

def quick_union(triplets):
    """Fast union occlusion for triplets."""
    if not triplets:
        return 0.0
    t_start = min(tb for _, tb in triplets)
    t_end = T_MAX
    i0 = int(t_start / DT_FAST)
    i1 = min(N_FAST, int(t_end / DT_FAST) + 1)
    if i0 >= i1:
        return 0.0
    union = np.zeros(i1 - i0, dtype=bool)
    for td, tb in triplets:
        det_pos = compute_detonation_pos(FY1_INIT, ALPHA_Q3, V_Q3, td, tb)
        for idx in range(i0, i1):
            t = T_FAST[idx]
            if t < tb or t > tb + TMAX:
                continue
            sc = smoke_center_pos(det_pos, tb, t)
            if check_occlusion_at_time(M1_TRAJ[idx], sc, TARGET_SAMPLE_PTS):
                union[idx - i0] = True
    return float(np.sum(union)) * DT_FAST


def generate_candidates(n=25):
    """Generate single-grenade candidates via grid search."""
    cands = []
    for td in np.linspace(0, 15, 31):
        for tb in np.linspace(td + 0.1, min(td + 15, T_MAX - 1), 31):
            oc = quick_occlusion(td, tb)
            if oc > 0.3:
                det_pos = compute_detonation_pos(FY1_INIT, ALPHA_Q3, V_Q3, td, tb)
                cands.append((td, tb, oc, det_pos))
    cands.sort(key=lambda c: -c[2])
    # Dedup
    filtered = []
    for c in cands:
        td, tb, oc, _ = c
        dup = False
        for f in filtered:
            if abs(td - f[0]) < 0.5 and abs(tb - f[1]) < 0.8:
                dup = True; break
        if not dup:
            filtered.append(c)
        if len(filtered) >= n:
            break
    return filtered


def greedy_fill(candidates, max_n=3):
    """Greedy search: add grenades that maximize union occlusion."""
    selected = []
    used = set()
    for _ in range(max_n):
        best_occ = quick_union(selected)
        best_idx = None
        for i, (td, tb, oc, _) in enumerate(candidates):
            if i in used:
                continue
            # Check min interval constraint
            ok = True
            for prev_td, prev_tb in selected:
                if td < prev_td + MIN_INTERVAL:
                    ok = False; break
            if not ok:
                continue
            trial = selected + [(td, tb)]
            trial_occ = quick_union(trial)
            if trial_occ > best_occ + 0.001:
                best_occ = trial_occ
                best_idx = i
        if best_idx is not None:
            selected.append((candidates[best_idx][0], candidates[best_idx][1]))
            used.add(best_idx)
        else:
            break
    return selected, quick_union(selected)


def local_refine(triplets):
    """Coordinate-descent refinement."""
    best = [list(t) for t in triplets]
    best_occ = quick_union(best)
    for step in [0.1, 0.05, 0.01, 0.005]:
        improved = True
        while improved:
            improved = False
            for i in range(len(best)):
                for j in [0, 1]:  # td, tb
                    for sgn in [-1, 1]:
                        new = [list(t) for t in best]
                        new[i][j] += sgn * step
                        if j == 0:  # td
                            if new[i][0] < 0: continue
                            prev_td = new[i-1][0] if i > 0 else -1
                            if new[i][0] < prev_td + MIN_INTERVAL: continue
                        if j == 1:  # tb
                            if new[i][1] <= new[i][0]: continue
                            if new[i][1] >= T_MAX: continue
                            zb = FY1_INIT[2] - 0.5*G*(new[i][1]-new[i][0])**2
                            if zb < -10: continue
                        occ = quick_union(new)
                        if occ > best_occ + 1e-8:
                            best = new; best_occ = occ; improved = True
            if not improved: break
    return best, best_occ


def solve_q3():
    print("=" * 70)
    print("  Q3: FY1 + 3 grenades vs M1")
    print(f"  Fixed: alpha={np.degrees(ALPHA_Q3):.4f}deg, v={V_Q3:.4f} m/s")
    print(f"  M1 flight: {T_MAX:.2f}s, dt={DT_FAST}s")
    print("=" * 70)
    print()

    t0 = time.time()
    print("  [1] Generating single-grenade candidates...")
    cands = generate_candidates(25)
    print(f"  Found {len(cands)} candidates ({time.time()-t0:.1f}s)")
    for i, (td, tb, oc, det_pos) in enumerate(cands[:5]):
        print(f"    {i+1}. td={td:.3f} tb={tb:.3f} occ={oc:.3f} det=({det_pos[0]:.0f},{det_pos[1]:.0f},{det_pos[2]:.0f})")
    print()

    print("  [2] Greedy multi-grenade selection...")
    t1 = time.time()
    triplets, occ_before = greedy_fill(cands, 3)
    print(f"  Found {len(triplets)} grenades, union occ={occ_before:.4f}s ({time.time()-t1:.1f}s)")
    print()

    print("  [3] Local refinement...")
    t2 = time.time()
    ref_triplets, ref_occ = local_refine(triplets)
    print(f"  Refined: {ref_occ:.4f}s ({time.time()-t2:.1f}s)")
    print()

    print("=" * 70)
    print("  Q3 RESULT")
    print("=" * 70)
    print(f"  Drone: heading={np.degrees(ALPHA_Q3):.4f}deg, v={V_Q3:.4f} m/s")
    for i, (td, tb) in enumerate(ref_triplets):
        det_pos = compute_detonation_pos(FY1_INIT, ALPHA_Q3, V_Q3, td, tb)
        drop_pos = drone_position(FY1_INIT, ALPHA_Q3, V_Q3, td)
        oc_s = quick_occlusion(td, tb)
        print(f"  G{i+1}: td={td:.6f} tb={tb:.6f} | drop=({drop_pos[0]:.2f},{drop_pos[1]:.2f},{drop_pos[2]:.2f}) det=({det_pos[0]:.2f},{det_pos[1]:.2f},{det_pos[2]:.2f}) occ={oc_s:.4f}s")
    print(f"  Total union: {ref_occ:.6f}s")
    print(f"  Total time: {time.time()-t0:.1f}s")
    print("=" * 70)
    return ref_occ

if __name__ == "__main__":
    result = solve_q3()
    print(f"Q3 result: {result:.6f} s")
