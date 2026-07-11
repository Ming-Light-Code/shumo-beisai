#!/usr/bin/env python3
"""Q2: DE optimization for single smoke grenade deployment against M1."""

import numpy as np
import time
import sys

# ============================================================
# Physical constants
# ============================================================
g = 9.8
V_MISSILE = 300.0
V_SINK = 3.0
R_SMOKE = 10.0
T_SMOKE_MAX = 20.0
DT = 0.01

# Initial positions
FY1_0 = np.array([17800.0, 0.0, 1800.0])
M1_0  = np.array([20000.0, 0.0, 2000.0])
DECOY = np.array([0.0, 0.0, 0.0])

# Target cylinder: bottom center (0,200,0), r=7, h=10
# Sample 7 representative points for occlusion check
TARGET_POINTS = np.array([
    [0.0, 200.0, 5.0],    # center
    [7.0, 200.0, 5.0],    # +x edge
    [-7.0, 200.0, 5.0],   # -x edge
    [0.0, 207.0, 5.0],    # +y edge
    [0.0, 193.0, 5.0],    # -y edge
    [0.0, 200.0, 10.0],   # top
    [0.0, 200.0, 0.0],    # bottom
])

# Precompute M1 trajectory (independent of decision variables)
d_M1 = DECOY - M1_0
dist_to_decoy = np.linalg.norm(d_M1)
d_M1_unit = d_M1 / dist_to_decoy
T_FLIGHT = dist_to_decoy / V_MISSILE

N_TIME = int(T_FLIGHT / DT) + 1
TIME_GRID = np.linspace(0, T_FLIGHT, N_TIME)
M1_TRAJ = M1_0 + V_MISSILE * d_M1_unit * TIME_GRID[:, np.newaxis]

# DE bounds: [alpha, v, td, tb]
BOUNDS = np.array([
    [0.0, 2 * np.pi],
    [70.0, 140.0],
    [0.0, 60.0],
    [0.1, 66.0],
])


def occlusion_duration(x):
    """
    Compute total occlusion duration for decision vector x = [alpha, v, td, tb].
    Returns (occlusion_time, min_distance_to_line).
    min_distance helps debug / provides soft fitness gradient.
    """
    alpha, v, td, tb = x

    if tb <= td or tb >= T_FLIGHT:
        return 0.0, np.inf

    t_start, t_end = tb, min(tb + T_SMOKE_MAX, T_FLIGHT)
    if t_start >= t_end:
        return 0.0, np.inf

    i_start = int(t_start / DT)
    i_end   = min(N_TIME, int(t_end / DT) + 1)
    n_t = i_end - i_start
    if n_t <= 0:
        return 0.0, np.inf

    # Smoke center at detonation (xb, yb horizontally fixed after detonation)
    xb = FY1_0[0] + v * np.cos(alpha) * tb
    yb = FY1_0[1] + v * np.sin(alpha) * tb
    zb = FY1_0[2] - 0.5 * g * (tb - td) ** 2

    t_slice  = TIME_GRID[i_start:i_end]
    M1_slice = M1_TRAJ[i_start:i_end]
    zc = zb - V_SINK * (t_slice - tb)

    # Combine occlusion across all sample target points (OR logic)
    occluded = np.zeros(n_t, dtype=bool)
    all_min_dists = []

    for tp in TARGET_POINTS:
        line_dirs = tp - M1_slice
        line_lens = np.linalg.norm(line_dirs, axis=1)
        valid = line_lens > 1e-6

        if not np.any(valid):
            continue

        C = np.column_stack([np.full(n_t, xb), np.full(n_t, yb), zc])
        to_smoke = C - M1_slice
        crosses = np.cross(line_dirs, to_smoke)
        cross_norms = np.linalg.norm(crosses, axis=1)
        dists = np.full(n_t, np.inf)
        dists[valid] = cross_norms[valid] / line_lens[valid]
        all_min_dists.append(np.min(dists[valid]) if np.any(valid) else np.inf)

        dots = np.sum(to_smoke * line_dirs, axis=1)
        lams = np.full(n_t, -1.0)
        lams[valid] = dots[valid] / (line_lens[valid] ** 2)

        mask = (dists <= R_SMOKE) & (lams >= 0.0) & (lams <= 1.0)
        occluded |= mask

    occl_time = np.sum(occluded) * DT
    min_dist  = min(all_min_dists) if all_min_dists else np.inf
    return occl_time, min_dist


def occlusion_fitness(x):
    """Fitness for DE: occlusion time directly, 0 if none."""
    t, _ = occlusion_duration(x)
    return t


# ============================================================
# Differential Evolution (scratch implementation)
# ============================================================
def de_optimize(objective, bounds, popsize=80, maxiter=400,
                F=0.85, CR=0.9, seed=None, verbose=True):
    rng = np.random.RandomState(seed)
    dim = len(bounds)
    lo, hi = bounds[:, 0], bounds[:, 1]

    # Initialize population
    pop = rng.uniform(lo, hi, size=(popsize, dim))
    fit = np.array([objective(ind) for ind in pop])

    best_idx = np.argmax(fit)
    best_x, best_f = pop[best_idx].copy(), fit[best_idx]

    for gen in range(maxiter):
        for i in range(popsize):
            # DE/rand/1 mutation
            pool = [j for j in range(popsize) if j != i]
            a, b, c = pop[rng.choice(pool, 3, replace=False)]
            mutant = a + F * (b - c)
            mutant = np.clip(mutant, lo, hi)

            # Binomial crossover
            cross = rng.rand(dim) < CR
            if not np.any(cross):
                cross[rng.randint(dim)] = True
            trial = np.where(cross, mutant, pop[i])

            # Enforce tb > td
            if trial[3] <= trial[2]:
                trial[3] = trial[2] + 0.05 + rng.rand() * 2.0
                trial[3] = min(trial[3], hi[3])

            f_trial = objective(trial)
            if f_trial >= fit[i]:
                pop[i], fit[i] = trial, f_trial
                if f_trial > best_f:
                    best_x, best_f = trial.copy(), f_trial

        if verbose and (gen + 1) % 50 == 0:
            print(f"  Gen {gen+1:4d} | best={best_f:.3f}s | "
                  f"alive={np.sum(fit > 0)}/{popsize}", flush=True)

    return best_x, best_f


# ============================================================
# Main
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("Q2: DE Optimization — FY1 + 1 grenade vs M1")
    print("=" * 60)
    print(f"M1 flight time: {T_FLIGHT:.2f}s")
    print(f"Time resolution: dt={DT}s, {N_TIME} steps")
    print(f"DE config: popsize=80, maxiter=400, F=0.85, CR=0.9")
    print()

    n_runs = 5
    results = []

    for run in range(n_runs):
        seed = 42 + run * 137
        print(f"--- Run {run+1}/{n_runs} (seed={seed}) ---")
        t0 = time.time()
        x_opt, f_opt = de_optimize(
            occlusion_fitness, BOUNDS,
            popsize=80, maxiter=400, F=0.85, CR=0.9,
            seed=seed, verbose=True
        )
        elapsed = time.time() - t0
        print(f"  → {f_opt:.3f}s occlusion, {elapsed:.1f}s wall time\n")
        results.append((x_opt, f_opt, seed))

    # Sort by best occlusion
    results.sort(key=lambda r: -r[1])
    best_x, best_f, best_seed = results[0]
    alpha, v, td, tb = best_x

    # Derived quantities
    drop_pt = FY1_0 + np.array([v * np.cos(alpha) * td,
                                  v * np.sin(alpha) * td, 0.0])
    det_pt  = np.array([FY1_0[0] + v * np.cos(alpha) * tb,
                         FY1_0[1] + v * np.sin(alpha) * tb,
                         FY1_0[2] - 0.5 * g * (tb - td) ** 2])

    print("=" * 60)
    print("OPTIMAL STRATEGY (best of {} runs)".format(n_runs))
    print("=" * 60)
    print(f"  Heading α:          {np.degrees(alpha):.2f}°")
    print(f"  Speed v:            {v:.2f} m/s")
    print(f"  Drop time td:       {td:.3f} s")
    print(f"  Detonation time tb: {tb:.3f} s")
    print(f"  Fall duration:      {tb - td:.3f} s")
    print()
    print(f"  Drop point:         ({drop_pt[0]:.1f}, {drop_pt[1]:.1f}, {drop_pt[2]:.1f})")
    print(f"  Detonation point:   ({det_pt[0]:.1f}, {det_pt[1]:.1f}, {det_pt[2]:.1f})")
    print()
    print(f"  Max occlusion:      {best_f:.3f} s")
    print()

    # Show all runs for consistency check
    print("All runs (sorted):")
    for i, (x, f, s) in enumerate(results):
        a_deg = np.degrees(x[0])
        print(f"  {i+1}. seed={s:4d}  α={a_deg:7.2f}°  v={x[1]:6.1f}  "
              f"td={x[2]:5.2f}  tb={x[3]:5.2f}  →  {f:.3f}s")
    print()

    # Compute mean and std for consistency
    all_f = [r[1] for r in results]
    print(f"Mean occlusion: {np.mean(all_f):.3f}s ± {np.std(all_f):.3f}s")
    print(f"Min/Max:         {np.min(all_f):.3f}s / {np.max(all_f):.3f}s")
