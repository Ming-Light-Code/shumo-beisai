#!/usr/bin/env python3
"""Q2: Two-stage decomposition — optimal smoke position → FY1 parameters."""

import numpy as np
import time

# ============================================================
# Constants
# ============================================================
g = 9.8
V_MISSILE = 300.0
V_SINK = 3.0
R_SMOKE = 10.0
T_SMOKE_MAX = 20.0
DT = 0.02  # 20ms resolution for occlusion integration

FY1_0 = np.array([17800.0, 0.0, 1800.0])
M1_0  = np.array([20000.0, 0.0, 2000.0])
DECOY = np.array([0.0, 0.0, 0.0])
TARGET_CENTER = np.array([0.0, 200.0, 5.0])

# M1 precompute
d_M1 = DECOY - M1_0
dist_decoy = np.linalg.norm(d_M1)
d_M1_u = d_M1 / dist_decoy
T_FLIGHT = dist_decoy / V_MISSILE

N_TIME = int(T_FLIGHT / DT) + 1
TIME_GRID = np.linspace(0, T_FLIGHT, N_TIME)
M1_TRAJ = M1_0 + V_MISSILE * d_M1_u * TIME_GRID[:, np.newaxis]

TARGET = TARGET_CENTER  # using center of cylinder for stage 1

print(f"M1 flight time: {T_FLIGHT:.2f}s, grid steps: {N_TIME} at dt={DT}s")


def occlusion_time(xb, yb, zb, tb):
    """
    Compute total occlusion time for smoke detonating at (xb,yb,zb) at time tb.
    Smoke center sinks at 3 m/s thereafter.
    Returns total occlusion duration in seconds.
    """
    if tb >= T_FLIGHT:
        return 0.0

    t_start, t_end = tb, min(tb + T_SMOKE_MAX, T_FLIGHT)
    if t_start >= t_end:
        return 0.0

    i0 = int(t_start / DT)
    i1 = min(N_TIME, int(t_end / DT) + 1)
    n = i1 - i0
    if n <= 0:
        return 0.0

    t_slice  = TIME_GRID[i0:i1]
    M1_slice = M1_TRAJ[i0:i1]
    zc = zb - V_SINK * (t_slice - tb)

    # Distance from smoke center (xb,yb,zc) to line M1→target
    line_dirs = TARGET - M1_slice                   # (n, 3)
    line_lens = np.linalg.norm(line_dirs, axis=1)   # (n,)
    valid = line_lens > 1e-6

    C = np.column_stack([np.full(n, xb), np.full(n, yb), zc])
    to_smoke = C - M1_slice
    crosses = np.cross(line_dirs, to_smoke)
    cross_norms = np.linalg.norm(crosses, axis=1)
    distances = np.full(n, np.inf)
    distances[valid] = cross_norms[valid] / line_lens[valid]

    # Check if projection lies between M1 and target (0 ≤ λ ≤ 1)
    dots = np.sum(to_smoke * line_dirs, axis=1)
    lams = np.full(n, -1.0)
    lams[valid] = dots[valid] / (line_lens[valid] ** 2)

    mask = (distances <= R_SMOKE) & (lams >= 0.0) & (lams <= 1.0)
    return np.sum(mask) * DT


# ============================================================
# Stage 1: Grid search over (xb, yb, zb, tb) in the reachable
#          region defined by FY1's flight constraints.
# ============================================================
print("\n=== STAGE 1: Grid search for optimal smoke position ===")

# Grid parameters
N_TB   = 40   # detonation time
N_R    = 30   # radial distance from FY1(0)
N_TH   = 60   # azimuthal angle
N_ZB   = 15   # detonation height

tb_vals = np.linspace(1.0, T_FLIGHT - 1.0, N_TB)

best_occlusion = 0.0
best_params = None
results_stage1 = []

t0 = time.time()

for i_tb, tb in enumerate(tb_vals):
    # Reachable annulus radius: r ∈ [70*tb, 140*tb]
    r_min, r_max = 70.0 * tb, 140.0 * tb
    r_vals = np.linspace(r_min, r_max, N_R)
    th_vals = np.linspace(0, 2*np.pi, N_TH, endpoint=False)

    # Build (xb, yb) grid in polar coords
    RR, TT = np.meshgrid(r_vals, th_vals, indexing='ij')
    xb_grid = 17800.0 + RR * np.cos(TT)  # (N_R, N_TH)
    yb_grid = RR * np.sin(TT)

    # Feasible zb range: zb ∈ [max(0, 1800 - 0.5*g*tb²), 1800]
    zb_min = max(0.0, 1800.0 - 0.5 * g * tb * tb)
    zb_vals = np.linspace(zb_min, 1800.0, N_ZB)

    # Evaluate all (xb, yb, zb) combinations for this tb
    for iz, zb in enumerate(zb_vals):
        occ_grid = np.zeros((N_R, N_TH))
        for ir in range(N_R):
            for ith in range(N_TH):
                occ_grid[ir, ith] = occlusion_time(
                    xb_grid[ir, ith], yb_grid[ir, ith], zb, tb
                )

        local_best = np.max(occ_grid)
        if local_best > 0:
            idx = np.unravel_index(np.argmax(occ_grid), occ_grid.shape)
            xb_b = xb_grid[idx]
            yb_b = yb_grid[idx]
            results_stage1.append((local_best, xb_b, yb_b, zb, tb))

            if local_best > best_occlusion:
                best_occlusion = local_best
                best_params = (xb_b, yb_b, zb, tb)

    if (i_tb + 1) % 10 == 0:
        elapsed = time.time() - t0
        print(f"  tb={tb:5.1f}s ({i_tb+1}/{N_TB}) | "
              f"best so far: {best_occlusion:.3f}s | {elapsed:.1f}s elapsed")

elapsed = time.time() - t0
print(f"\nStage 1 complete: {elapsed:.1f}s")
print(f"Top occlusion found: {best_occlusion:.3f}s")
xb_opt, yb_opt, zb_opt, tb_opt = best_params
print(f"Optimal smoke: xb={xb_opt:.1f}, yb={yb_opt:.1f}, zb={zb_opt:.1f}, tb={tb_opt:.2f}")

# Show top 10
results_stage1.sort(key=lambda r: -r[0])
print("\nTop 10 candidates:")
for i, (occ, xb, yb, zb, tb) in enumerate(results_stage1[:10]):
    print(f"  {i+1}. occ={occ:.3f}s  xb={xb:.0f}  yb={yb:.0f}  zb={zb:.0f}  tb={tb:.2f}")

# ============================================================
# Stage 2: Back-solve FY1 parameters from optimal smoke position
# ============================================================
print("\n=== STAGE 2: Back-solve FY1 parameters ===")

xb, yb, zb, tb = best_params

# Speed and heading from horizontal displacement
dx = xb - FY1_0[0]
dy = yb - FY1_0[1]
dist_xy = np.sqrt(dx*dx + dy*dy)
v = dist_xy / tb
alpha = np.arctan2(dy, dx)

# Drop time from fall height
fall_height = FY1_0[2] - zb
if fall_height < 0:
    print("WARNING: zb above FY1 altitude — adjusting")
    zb = FY1_0[2]
    fall_height = 0.0

fall_time = np.sqrt(2.0 * fall_height / g) if fall_height > 0 else 0.0
td = tb - fall_time

print(f"\nDerived FY1 parameters:")
print(f"  Heading α:  {np.degrees(alpha):.2f}° ({alpha:.4f} rad)")
print(f"  Speed v:    {v:.2f} m/s  (valid: {70 <= v <= 140})")
print(f"  Drop td:    {td:.3f} s")
print(f"  Detonate tb:{tb:.3f} s")
print(f"  Fall dur.:  {tb - td:.3f} s")

# Drop point
drop_x = FY1_0[0] + v * np.cos(alpha) * td
drop_y = FY1_0[1] + v * np.sin(alpha) * td
drop_z = FY1_0[2]
print(f"\n  Drop point:      ({drop_x:.1f}, {drop_y:.1f}, {drop_z:.1f})")
print(f"  Detonation point: ({xb:.1f}, {yb:.1f}, {zb:.1f})")

# Verify with fine-grained check
print(f"\n  Verified occlusion: {occlusion_time(xb, yb, zb, tb):.3f}s")

# ============================================================
# Refinement: local search around the best candidate
# ============================================================
print("\n=== REFINEMENT: Local search around optimum ===")

def refine(xb0, yb0, zb0, tb0, step_xy=5.0, step_z=5.0, step_t=0.1, radius=5):
    """Simple hill-climb refinement around (xb0, yb0, zb0, tb0)."""
    best = occlusion_time(xb0, yb0, zb0, tb0)
    best_pt = (xb0, yb0, zb0, tb0)

    for dx in np.linspace(-step_xy*radius, step_xy*radius, 2*radius+1):
        for dy in np.linspace(-step_xy*radius, step_xy*radius, 2*radius+1):
            for dz in np.linspace(-step_z*radius, step_z*radius, 2*radius+1):
                for dtb in np.linspace(-step_t*radius, step_t*radius, 2*radius+1):
                    xb = xb0 + dx
                    yb = yb0 + dy
                    zb = zb0 + dz
                    tb = tb0 + dtb
                    if tb <= 0 or tb >= T_FLIGHT:
                        continue
                    occ = occlusion_time(xb, yb, zb, tb)
                    if occ > best:
                        best = occ
                        best_pt = (xb, yb, zb, tb)

    return best_pt, best

(xb_r, yb_r, zb_r, tb_r), occ_r = refine(xb_opt, yb_opt, zb_opt, tb_opt,
                                           step_xy=2.0, step_z=2.0, step_t=0.05, radius=8)
print(f"Refined occlusion: {occ_r:.3f}s (was {best_occlusion:.3f}s)")

# Back-solve again with refined params
xb, yb, zb, tb = xb_r, yb_r, zb_r, tb_r
dx = xb - FY1_0[0]
dy = yb - FY1_0[1]
dist_xy = np.sqrt(dx*dx + dy*dy)
v_r = dist_xy / tb
alpha_r = np.arctan2(dy, dx)
fall_height = max(0.0, FY1_0[2] - zb)
fall_time = np.sqrt(2.0 * fall_height / g) if fall_height > 0 else 0.0
td_r = tb - fall_time

print(f"\n=== FINAL OPTIMAL STRATEGY ===")
print(f"  Heading α:          {np.degrees(alpha_r):.2f}°")
print(f"  Speed v:            {v_r:.2f} m/s")
print(f"  Drop time td:       {td_r:.3f} s")
print(f"  Detonation time tb: {tb:.3f} s")
print(f"  Fall duration:      {tb - td_r:.3f} s")

drop_x = FY1_0[0] + v_r * np.cos(alpha_r) * td_r
drop_y = FY1_0[1] + v_r * np.sin(alpha_r) * td_r
print(f"\n  Drop point:         ({drop_x:.1f}, {drop_y:.1f}, {FY1_0[2]:.1f})")
print(f"  Detonation point:   ({xb:.1f}, {yb:.1f}, {zb:.1f})")
print(f"\n  Max occlusion:      {occ_r:.3f}s")

# Check speed constraint
if 70 <= v_r <= 140:
    print(f"  Speed constraint:   ✓ ({v_r:.1f} ∈ [70, 140])")
else:
    print(f"  Speed constraint:   ✗ ({v_r:.1f} ∉ [70, 140])")

# Check td ≥ 0
if td_r >= 0:
    print(f"  td ≥ 0:             ✓")
else:
    print(f"  td ≥ 0:             ✗ ({td_r:.3f})")
