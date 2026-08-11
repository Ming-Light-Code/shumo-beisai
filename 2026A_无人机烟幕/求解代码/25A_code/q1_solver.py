#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Q1: Single drone FY1, single smoke grenade vs M1.
Given parameters -> compute effective occlusion time.
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *

# ============================================================
# Q1 given parameters
# ============================================================
# # FY1 flies at 120 m/s horizontally toward decoy (origin).
# # Drop at td=1.5s, detonate at tb=5.1s (fall duration = 3.6s).

M1_INIT = MISSILES_INIT["M1"]
FY1_INIT = DRONES_INIT["FY1"]

# FY1 heading: toward decoy = (-1, 0, 0) direction in xy-plane
# Since FY1 starts at (17800, 0, 1800) and decoy is at (0, 0, 0)
# alpha = pi (180 degrees) = direction toward negative x
alpha_q1 = pi
v_q1 = 120.0
td_q1 = 1.5
tb_q1 = 5.1


def solve_q1():
    """Compute occlusion intervals and total effective time for Q1."""
    print("=" * 70)
    print("  Q1: FY1 + 1 smoke grenade vs M1 (fixed parameters)")
    print("=" * 70)
    print()
    print("  Given parameters:")
    print(f"    FY1 initial: ({FY1_INIT[0]}, {FY1_INIT[1]}, {FY1_INIT[2]})")
    print(f"    M1 initial:  ({M1_INIT[0]}, {M1_INIT[1]}, {M1_INIT[2]})")
    print(f"    alpha = {np.degrees(alpha_q1):.4f}deg (toward decoy)")
    print(f"    v = {v_q1} m/s")
    print(f"    td = {td_q1} s (drop time)")
    print(f"    tb = {tb_q1} s (detonation time)")
    print(f"    fall duration = {tb_q1 - td_q1} s")
    print()

    # Compute drop and detonation positions
    drop_pos = drone_position(FY1_INIT, alpha_q1, v_q1, td_q1)
    det_pos = compute_detonation_pos(FY1_INIT, alpha_q1, v_q1, td_q1, tb_q1)
    print("  Derived positions:")
    print(f"    Drop point:      ({drop_pos[0]:.2f}, {drop_pos[1]:.2f}, {drop_pos[2]:.2f})")
    print(f"    Detonation point: ({det_pos[0]:.2f}, {det_pos[1]:.2f}, {det_pos[2]:.2f})")
    print()

    # M1 flight time
    t_max = missile_flight_time(M1_INIT)
    print(f"  M1 flight time to decoy: {t_max:.2f} s")
    print()

    # Find occlusion intervals with bisection
    print("  Searching occlusion intervals (scan + bisection)...")
    intervals = find_occlusion_intervals(
        M1_INIT, FY1_INIT, alpha_q1, v_q1, td_q1, tb_q1,
        t_max, TARGET_SAMPLE_PTS, dt_scan=0.001, t_tol=1e-12
    )

    print()
    print("  Occlusion intervals:")
    total = 0.0
    for i, (t_start, t_end) in enumerate(intervals):
        dur = t_end - t_start
        total += dur
        print(f"    [{t_start:.6f}s, {t_end:.6f}s]  duration = {dur:.6f}s")
    
    if not intervals:
        print("    (none found)")
    print()
    print(f"  === TOTAL EFFECTIVE OCCLUSION TIME: {total:.6f} s ===")
    print()

    # Verify using fast scan also
    fast_dur = occlusion_duration(M1_INIT, FY1_INIT, alpha_q1, v_q1, td_q1, tb_q1,
                                  TARGET_SAMPLE_PTS, dt=0.001)
    print(f"  Fast scan (dt=0.001s): {fast_dur:.6f} s")
    print()

    # Check constraints
    ok, msg = check_grenade_constraints(FY1_INIT, alpha_q1, v_q1, td_q1, tb_q1)
    print(f"  Constraints: {msg}")
    print()

    return total


if __name__ == "__main__":
    result = solve_q1()
    print(f"Q1 result: {result:.6f} s")
