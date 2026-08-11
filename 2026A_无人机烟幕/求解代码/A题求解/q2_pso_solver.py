#!/usr/bin/env python3
"""Q2: Hybrid PSO - geometric prediction seeding + particle swarm refinement."""

import numpy as np
import time, sys

g = 9.8; Vm = 300.0; Vs = 3.0; R = 10.0; Tmax = 20.0; DT = 0.005
FY1_0 = np.array([17800.0, 0.0, 1800.0])
M1_0  = np.array([20000.0, 0.0, 2000.0])
TGT_C = np.array([0.0, 200.0, 5.0])
DECOY = np.array([0.0, 0.0, 0.0])
dM = DECOY - M1_0; distD = np.linalg.norm(dM); dM_u = dM / distD
Tf = distD / Vm; NT = int(Tf / DT) + 1
TGRID = np.linspace(0, Tf, NT)
M1T = M1_0 + Vm * dM_u * TGRID[:, np.newaxis]
TGT_PTS = np.array([[0,200,5],[7,200,5],[-7,200,5],[0,207,5],[0,193,5],[0,200,10],[0,200,0]])

def occ_time(xb, yb, zb, tb):
    if tb >= Tf: return 0.0
    t0, t1 = tb, min(tb + Tmax, Tf)
    if t0 >= t1: return 0.0
    i0 = int(t0 / DT); i1 = min(NT, int(t1 / DT) + 1)
    n = i1 - i0
    if n <= 0: return 0.0
    ts = TGRID[i0:i1]; Ms = M1T[i0:i1]
    zc = zb - Vs * (ts - tb)
    occ = np.zeros(n, dtype=bool)
    for tp in TGT_PTS:
        ld = tp - Ms; ll = np.linalg.norm(ld, axis=1)
        v = ll > 1e-9
        C = np.column_stack([np.full(n, xb), np.full(n, yb), zc])
        ts2 = C - Ms; cr = np.cross(ld, ts2); cn = np.linalg.norm(cr, axis=1)
        ds = np.full(n, np.inf); ds[v] = cn[v] / ll[v]
        dt = np.sum(ts2 * ld, axis=1); lm = np.full(n, -1.0); lm[v] = dt[v] / (ll[v] ** 2)
        occ |= (ds <= R) & (lm >= 0) & (lm <= 1)
    return np.sum(occ) * DT

def fitness(x):
    alpha, v, td, tb = x
    xb = FY1_0[0] + v * np.cos(alpha) * tb
    yb = FY1_0[1] + v * np.sin(alpha) * tb
    zb = FY1_0[2] - 0.5 * g * (tb - td) ** 2
    if zb < -10.0: return 0.0
    return occ_time(xb, yb, zb, tb)

def repair(x, lo, hi):
    alpha, v, td, tb = x
    alpha = alpha % (2 * np.pi)
    v = np.clip(v, lo[1], hi[1])
    td = max(0.0, td)
    tb = max(td + 1e-4, min(tb, hi[3]))
    return np.array([alpha, v, td, tb])

# ============================================================
# Geometric prediction (same as in q2_geo_nlp.py)
# ============================================================
def gen_starts(max_s=50):
    starts = []
    tb_c = np.linspace(0.5, Tf - 0.5, 30)
    tb_f = np.linspace(1.0, min(15, Tf - 0.5), 20)
    for tb in np.unique(np.concatenate([tb_c, tb_f])):
        Mt = M1_0 + Vm * dM_u * tb
        for tp in TGT_PTS:
            los = tp - Mt; ln = np.linalg.norm(los)
            if ln < 1e-6: continue
            lu = los / ln
            dmn = R * 1.1; dmx = ln - R * 1.1
            if dmx <= dmn: continue
            for D in np.linspace(dmn, dmx, 15):
                Cb = Mt + D * lu; xb0, yb0, zb0 = Cb
                perp = np.cross(lu, np.array([1, 0, 0]))
                if np.linalg.norm(perp) < 0.1:
                    perp = np.cross(lu, np.array([0, 1, 0]))
                perp = perp / np.linalg.norm(perp)
                for off in np.linspace(-R * 0.8, R * 0.8, 5):
                    C = Cb + off * perp; xb, yb, zb = C
                    dx = xb - FY1_0[0]; dy = yb - FY1_0[1]
                    dxy = np.sqrt(dx * dx + dy * dy)
                    if tb <= 0 or dxy <= 0: continue
                    v = dxy / tb
                    if not (69.99 <= v <= 140.01): continue
                    alpha = np.arctan2(dy, dx)
                    fh = max(0.0, FY1_0[2] - zb)
                    ft = np.sqrt(2.0 * fh / g) if fh > 0 else 0.0
                    td = tb - ft
                    if td < 0: continue
                    oc = occ_time(xb, yb, zb, tb)
                    if oc > 0.01: starts.append((alpha, v, td, tb, oc))
    starts.sort(key=lambda s: -s[4])
    flt = []
    for s in starts:
        a, v, td, tb, oc = s
        tc = False
        for f in flt:
            da = abs(a - f[0])
            if da > np.pi: da = 2 * np.pi - da
            if da < 0.015 and abs(v - f[1]) < 1.5 and abs(tb - f[3]) < 0.4:
                tc = True; break
        if not tc: flt.append(s)
        if len(flt) >= max_s: break
    return [np.array([a, v, td, tb]) for a, v, td, tb, oc in flt[:max_s]]


# ============================================================
# Hybrid PSO: geometric seeding + PSO refinement
# ============================================================
def hybrid_pso(n_seeds=30, n_random=20, max_iter=200,
               w_start=0.7, w_end=0.3, c1=1.5, c2=2.0, seed=None):
    rng = np.random.RandomState(seed)
    dim = 4
    lo = np.array([0.0, 70.0, 0.0, 0.1])
    hi = np.array([2 * np.pi, 140.0, Tf, Tf - 0.1])

    # Phase 1: geometric seeding
    geo_seeds = gen_starts(n_seeds)
    n_geo = min(len(geo_seeds), n_seeds)

    # Phase 2: fill remaining with perturbed copies of best geo seeds
    n_particles = n_geo + n_random
    pos = np.zeros((n_particles, dim))
    for i in range(n_geo):
        pos[i] = geo_seeds[i]

    # Fill remaining with random perturbation around top seeds
    for i in range(n_geo, n_particles):
        base = geo_seeds[rng.randint(0, n_geo)]
        pert = np.array([
            rng.uniform(-0.05, 0.05),    # alpha perturbation
            rng.uniform(-5, 5),          # v perturbation
            rng.uniform(-0.1, 0.1),      # td perturbation
            rng.uniform(-0.1, 0.1),      # tb perturbation
        ])
        pos[i] = repair(base + pert, lo, hi)

    vel = np.zeros((n_particles, dim))
    vel_max = 0.1 * (hi - lo)

    # Evaluate
    fit = np.array([fitness(pos[i]) for i in range(n_particles)])
    pbest_pos = pos.copy()
    pbest_fit = fit.copy()
    gbest_idx = np.argmax(pbest_fit)
    gbest_pos = pbest_pos[gbest_idx].copy()
    gbest_fit = pbest_fit[gbest_idx]

    history = [(0, gbest_fit)]
    alive_at_start = np.sum(pbest_fit > 0)

    for gen in range(1, max_iter + 1):
        w = w_start - (w_start - w_end) * gen / max_iter

        for i in range(n_particles):
            r1, r2 = rng.rand(dim), rng.rand(dim)
            vel[i] = (w * vel[i]
                      + c1 * r1 * (pbest_pos[i] - pos[i])
                      + c2 * r2 * (gbest_pos - pos[i]))
            vel[i] = np.clip(vel[i], -vel_max, vel_max)
            pos[i] = pos[i] + vel[i]
            pos[i] = repair(pos[i], lo, hi)

            new_fit = fitness(pos[i])
            if new_fit >= pbest_fit[i]:
                pbest_pos[i] = pos[i].copy()
                pbest_fit[i] = new_fit
                if new_fit > gbest_fit:
                    gbest_pos = pos[i].copy()
                    gbest_fit = new_fit

        history.append((gen, gbest_fit))

    return gbest_pos, gbest_fit, history, alive_at_start, n_particles


if __name__ == '__main__':
    print("=" * 70)
    print("  Q2: Hybrid PSO (Geometric Seeding + Particle Swarm)")
    print(f"  M1 flight: {Tf:.6f}s | DT={DT}s")
    print("=" * 70)
    print(f"  Strategy: geo seeds (30) + perturbed copies (20) = 50 particles")
    print(f"  PSO: iter=200, w=0.7{chr(8594)}0.3, c1=1.5, c2=2.0")
    print()

    n_runs = 5
    all_results = []

    for run in range(n_runs):
        seed = 42 + run * 137
        print(f"  --- Run {run+1}/{n_runs} (seed={seed}) ---")
        t0 = time.time()
        x_opt, f_opt, hist, alive0, n_p = hybrid_pso(
            n_seeds=30, n_random=20, max_iter=200, seed=seed
        )
        elapsed = time.time() - t0
        alpha, v, td, tb = x_opt
        print(f"  Alive at start: {alive0}/{n_p}")
        print(f"  Best: {f_opt:.6f}s | a={np.degrees(alpha):.2f}deg v={v:.2f} td={td:.3f} tb={tb:.3f}")
        print(f"  Wall time: {elapsed:.1f}s")
        all_results.append((seed, x_opt, f_opt, elapsed, hist, alive0))

    all_results.sort(key=lambda r: -r[2])

    print("\n" + "=" * 70)
    print("  HYBRID PSO RESULTS (sorted)")
    print("=" * 70)
    for i, (seed, x, f, t, h, a0) in enumerate(all_results):
        alpha, v, td, tb = x
        print(f"  {i+1}. seed={seed:4d}  a={np.degrees(alpha):8.3f}deg  v={v:7.2f}  "
              f"td={td:6.3f}  tb={tb:6.3f}  {chr(8594)} {f:.6f}s  ({t:.1f}s  alive0={a0})")

    best_seed, best_x, best_f, best_t, best_hist, best_a0 = all_results[0]
    ao, vo, tdo, tbo = best_x
    xbo = FY1_0[0] + vo * np.cos(ao) * tbo
    ybo = FY1_0[1] + vo * np.sin(ao) * tbo
    zbo = FY1_0[2] - 0.5 * g * (tbo - tdo) ** 2

    print(f"\n  Best (seed={best_seed}, {best_t:.1f}s):")
    print(f"    alpha  = {ao:.6f} rad ({np.degrees(ao):.6f} deg)")
    print(f"    v      = {vo:.6f} m/s")
    print(f"    td     = {tdo:.6f} s")
    print(f"    tb     = {tbo:.6f} s")
    print(f"    det    = ({xbo:.6f}, {ybo:.6f}, {zbo:.6f})")
    print(f"    occ    = {best_f:.6f} s")

    all_f = [r[2] for r in all_results]
    n_hit = sum(1 for f in all_f if f > 4.9)
    print(f"\n  Statistics ({n_runs} runs):")
    print(f"    Best:    {max(all_f):.6f}s")
    print(f"    Mean:    {np.mean(all_f):.6f}s")
    print(f"    Std:     {np.std(all_f):.6f}s")
    print(f"    Hit rate (>4.9s): {n_hit}/{n_runs}")

    # Convergence for best run
    print(f"\n  Convergence (best run, every 20 gen):")
    for gen, val in best_hist[::20]:
        print(f"    Gen {gen:4d}: {val:.6f}s")

    # Triple comparison
    print("\n" + "=" * 70)
    print("  THREE-WAY COMPARISON: PSO vs DE vs NLP")
    print("=" * 70)
    print(f"  Pure PSO:         0.0000s  (0/5 runs, all stuck in plateau)")
    print(f"  Hybrid PSO:       {max(all_f):.4f}s  (hit: {n_hit}/{n_runs})")
    print(f"  DE:               4.9700s  (hit: 1/5)")
    print(f"  GeoPred+NLP:      4.9550s  (hit: 27/27)")
    print(f"  Total PSO time:   {sum(r[3] for r in all_results):.1f}s ({n_runs} runs)")
    print(f"  Total NLP time:    8.91s  (27 starts)")
    print("=" * 70)
