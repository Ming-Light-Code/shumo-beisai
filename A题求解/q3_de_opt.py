#!/usr/bin/env python3
"""Q3: Optimized DE - faster convergence, early stopping, adaptive F."""

import numpy as np
import time

g = 9.8; Vm = 300.0; Vs = 3.0; R = 10.0; Tmax = 20.0; DT = 0.01; DT_F = 0.005
FY1_0 = np.array([17800.0, 0.0, 1800.0]); M1_0 = np.array([20000.0, 0.0, 2000.0])
TGT_C = np.array([0.0, 200.0, 5.0]); DECOY = np.array([0.0, 0.0, 0.0])
dM = DECOY - M1_0; distD = np.linalg.norm(dM); dM_u = dM / distD
Tf = distD / Vm; NT = int(Tf / DT) + 1; NT_F = int(Tf / DT_F) + 1
TGRID = np.linspace(0, Tf, NT); TGRID_F = np.linspace(0, Tf, NT_F)
M1T = M1_0 + Vm * dM_u * TGRID[:, np.newaxis]
M1T_F = M1_0 + Vm * dM_u * TGRID_F[:, np.newaxis]
TGT_PTS = np.array([[0,200,5],[7,200,5],[-7,200,5],[0,207,5],[0,193,5],[0,200,10],[0,200,0]])
MAX_FALL = np.sqrt(2 * FY1_0[2] / g)

# ---- Core occlusion (inlined for speed) ----
def _occ_one(xb, yb, zb, tb, grid, mt, dt, n_grid):
    if tb >= Tf: return 0.0
    t0, t1 = tb, min(tb + Tmax, Tf)
    if t0 >= t1: return 0.0
    i0 = int(t0 / dt); i1 = min(n_grid, int(t1 / dt) + 1)
    n = i1 - i0
    if n <= 0: return 0.0
    ts = grid[i0:i1]; Ms = mt[i0:i1]; zc = zb - Vs * (ts - tb)
    occ = np.zeros(n, dtype=bool)
    for tp in TGT_PTS:
        ld = tp - Ms; ll = np.linalg.norm(ld, axis=1)
        v = ll > 1e-9
        C = np.column_stack([np.full(n, xb), np.full(n, yb), zc])
        ts2 = C - Ms; cr = np.cross(ld, ts2); cn = np.linalg.norm(cr, axis=1)
        ds = np.full(n, np.inf); ds[v] = cn[v] / ll[v]
        dt_d = np.sum(ts2 * ld, axis=1)
        lm = np.full(n, -1.0); lm[v] = dt_d[v] / (ll[v] ** 2)
        occ |= (ds <= R) & (lm >= 0) & (lm <= 1)
    return np.sum(occ) * dt

def occ_time(xb, yb, zb, tb):
    return _occ_one(xb, yb, zb, tb, TGRID, M1T, DT, NT)

# ---- Union occlusion (fast: evaluate both grenades) ----
def occ_union(xb1, yb1, zb1, tb1, xb2, yb2, zb2, tb2):
    if tb1 >= Tf and tb2 >= Tf: return 0.0
    t_start = max(0.0, min(tb1, tb2))
    t_end = Tf
    i0 = int(t_start / DT); i1 = min(NT, int(t_end / DT) + 1)
    n = i1 - i0
    if n <= 0: return 0.0
    ts = TGRID[i0:i1]; Ms = M1T[i0:i1]
    ou = np.zeros(n, dtype=bool)
    for xb, yb, zb, tb in [(xb1, yb1, zb1, tb1), (xb2, yb2, zb2, tb2)]:
        if tb >= Tf: continue
        zc = zb - Vs * np.maximum(0, ts - tb)
        o = np.zeros(n, dtype=bool)
        for tp in TGT_PTS:
            ld = tp - Ms; ll = np.linalg.norm(ld, axis=1)
            v = ll > 1e-9
            C = np.column_stack([np.full(n, xb), np.full(n, yb), zc])
            ts2 = C - Ms; cr = np.cross(ld, ts2); cn = np.linalg.norm(cr, axis=1)
            ds = np.full(n, np.inf); ds[v] = cn[v] / ll[v]
            dt_d = np.sum(ts2 * ld, axis=1)
            lm = np.full(n, -1.0); lm[v] = dt_d[v] / (ll[v] ** 2)
            o |= (ds <= R) & (lm >= 0) & (lm <= 1) & (ts >= tb)
        ou |= o
    return np.sum(ou) * DT

# ---- Fitness: x=[alpha, v, td1, tb1, td2, tb2] ----
def fitness(x):
    a, v, td1, tb1, td2, tb2 = x
    xb1 = FY1_0[0] + v * np.cos(a) * tb1; yb1 = FY1_0[1] + v * np.sin(a) * tb1
    zb1 = FY1_0[2] - 0.5 * g * (tb1 - td1) ** 2
    if zb1 < -10: zb1 = -10
    xb2 = FY1_0[0] + v * np.cos(a) * tb2; yb2 = FY1_0[1] + v * np.sin(a) * tb2
    zb2 = FY1_0[2] - 0.5 * g * (tb2 - td2) ** 2
    if zb2 < -10: zb2 = -10
    return occ_union(xb1, yb1, zb1, tb1, xb2, yb2, zb2, tb2)

# ---- Inline repair (no function call overhead in DE loop) ----
LO = np.array([0.0, 70.0, 0.0, 0.1, 1.0, 1.5])
HI = np.array([2.0 * np.pi, 140.0, Tf, Tf, Tf, Tf - 0.1])

# ---- Generate geo seeds at DT_FINE for better detection ----
def gen_seeds_fine(n=20):
    # Temporarily use DT_F for occ_time (patch via globals hack)
    global DT, NT, TGRID, M1T
    DT_save, NT_save, TGRID_save, M1T_save = DT, NT, TGRID, M1T
    DT, NT, TGRID, M1T = DT_F, NT_F, TGRID_F, M1T_F

    starts = []
    tb_v = np.unique(np.concatenate([
        np.linspace(0.3, 3.0, 15), np.linspace(3.0, min(15, Tf - 0.5), 10)
    ]))
    for tb in tb_v:
        Mt = M1_0 + Vm * dM_u * tb
        for tp in TGT_PTS:
            los = tp - Mt; ln = np.linalg.norm(los)
            if ln < 1e-6: continue
            lu = los / ln
            for D in np.linspace(R * 1.1, ln - R * 1.1, 10):
                C = Mt + D * lu; xb, yb, zb = C
                dx = xb - FY1_0[0]; dy = yb - FY1_0[1]
                dxy = np.sqrt(dx * dx + dy * dy)
                if tb <= 0 or dxy <= 0: continue
                v = dxy / tb
                if not (69.99 <= v <= 140.01): continue
                alpha = np.arctan2(dy, dx)
                fh = max(0, FY1_0[2] - zb)
                ft = np.sqrt(2 * fh / g) if fh > 0 else 0
                td = tb - ft
                if td < 0: continue
                oc = occ_time(xb, yb, zb, tb)
                if oc > 0.01: starts.append((alpha, v, td, tb, oc))
    starts.sort(key=lambda s: -s[4])
    flt = []
    for s in starts:
        a, v, td, tb, oc = s; tc = False
        for f in flt:
            da = abs(a - f[0]); 
            if da > np.pi: da = 2 * np.pi - da
            if da < 0.02 and abs(v - f[1]) < 2 and abs(tb - f[3]) < 0.3: tc = True; break
        if not tc: flt.append(s)
        if len(flt) >= n: break

    # Restore DT
    DT, NT, TGRID, M1T = DT_save, NT_save, TGRID_save, M1T_save
    return flt[:n]


# ---- Fine-DT verification ----
def verify_fine(x):
    a, v, td1, tb1, td2, tb2 = x
    xb1 = FY1_0[0] + v * np.cos(a) * tb1; yb1 = FY1_0[1] + v * np.sin(a) * tb1
    zb1 = FY1_0[2] - 0.5 * g * (tb1 - td1) ** 2
    xb2 = FY1_0[0] + v * np.cos(a) * tb2; yb2 = FY1_0[1] + v * np.sin(a) * tb2
    zb2 = FY1_0[2] - 0.5 * g * (tb2 - td2) ** 2
    if tb1 >= Tf and tb2 >= Tf: return 0.0, 0, 0
    i0 = int(max(0, min(tb1, tb2)) / DT_F)
    i1 = min(NT_F, int(Tf / DT_F) + 1); n = i1 - i0
    if n <= 0: return 0.0, 0, 0
    ts = TGRID_F[i0:i1]; Ms = M1T_F[i0:i1]
    ou = np.zeros(n, dtype=bool); o1v = 0; o2v = 0
    for ig, (xb, yb, zb, tb) in enumerate([(xb1, yb1, zb1, tb1), (xb2, yb2, zb2, tb2)]):
        if tb >= Tf: continue
        zc = zb - Vs * np.maximum(0, ts - tb)
        o = np.zeros(n, dtype=bool)
        for tp in TGT_PTS:
            ld = tp - Ms; ll = np.linalg.norm(ld, axis=1); v = ll > 1e-9
            C = np.column_stack([np.full(n, xb), np.full(n, yb), zc])
            ts2 = C - Ms; cr = np.cross(ld, ts2); cn = np.linalg.norm(cr, axis=1)
            ds = np.full(n, np.inf); ds[v] = cn[v] / ll[v]
            dt_d = np.sum(ts2 * ld, axis=1)
            lm = np.full(n, -1.0); lm[v] = dt_d[v] / (ll[v] ** 2)
            o |= (ds <= R) & (lm >= 0) & (lm <= 1) & (ts >= tb)
        ou |= o
        if ig == 0: o1v = np.sum(o) * DT_F
        else: o2v = np.sum(o) * DT_F
    return np.sum(ou) * DT_F, o1v, o2v


# ---- Optimized DE ----
def de_optimize_fast(popsize=30, maxiter=150, seed=42, early_stop=80):
    rng = np.random.RandomState(seed)
    dim = 6

    # ---- Build population from geo seeds ----
    geo_seeds = gen_seeds_fine(20)
    pop = np.zeros((popsize, dim))
    n_seed = min(len(geo_seeds) + 8, popsize - 5)

    for i in range(min(len(geo_seeds), n_seed)):
        a, v, td1, tb1, _ = geo_seeds[i]
        tb2 = min(tb1 + 1.0 + rng.uniform(1, 10), Tf - 2)
        td2 = max(td1 + 1.0, tb2 - rng.uniform(0.5, 4))
        a = a % (2 * np.pi); v = np.clip(v, 70, 140)
        td1 = max(0, td1); tb1 = max(td1 + 0.1, min(tb1, HI[3]))
        td2 = max(td1 + 1.0, td2); tb2 = max(td2 + 0.1, min(tb2, HI[5]))
        tb1 = min(tb1, td1 + MAX_FALL); tb2 = min(tb2, td2 + MAX_FALL)
        pop[i] = [a, v, td1, tb1, td2, tb2]

    for i in range(len(geo_seeds), n_seed):
        a, v, td1, tb1, _ = geo_seeds[rng.randint(0, len(geo_seeds))]
        cand = np.array([a + rng.uniform(-0.05, 0.05), v + rng.uniform(-10, 10),
                         td1 + rng.uniform(-0.3, 0.3), tb1 + rng.uniform(-0.5, 0.5),
                         td1 + 2.5, tb1 + 4.0])
        # Inline repair
        cand[0] = cand[0] % (2 * np.pi); cand[1] = np.clip(cand[1], 70, 140)
        cand[2] = max(0, cand[2]); cand[3] = max(cand[2] + 0.1, min(cand[3], HI[3]))
        cand[4] = max(cand[2] + 1.0, cand[4]); cand[5] = max(cand[4] + 0.1, min(cand[5], HI[5]))
        cand[3] = min(cand[3], cand[2] + MAX_FALL); cand[5] = min(cand[5], cand[4] + MAX_FALL)
        pop[i] = cand

    for i in range(n_seed, popsize):
        cand = rng.uniform(LO, HI)
        cand[0] = cand[0] % (2 * np.pi); cand[1] = np.clip(cand[1], 70, 140)
        cand[2] = max(0, cand[2]); cand[3] = max(cand[2] + 0.1, min(cand[3], HI[3]))
        cand[4] = max(cand[2] + 1.0, cand[4]); cand[5] = max(cand[4] + 0.1, min(cand[5], HI[5]))
        cand[3] = min(cand[3], cand[2] + MAX_FALL); cand[5] = min(cand[5], cand[4] + MAX_FALL)
        pop[i] = cand

    # Evaluate
    ft = np.array([fitness(pop[i]) for i in range(popsize)])
    bi = np.argmax(ft); bx, bf = pop[bi].copy(), ft[bi]
    gen_best = 0

    for gen in range(maxiter):
        F_adapt = 0.9 - (0.9 - 0.5) * gen / maxiter  # adaptive F

        for i in range(popsize):
            # DE/rand/1
            pool = [j for j in range(popsize) if j != i]
            a, b, c = pop[rng.choice(pool, 3, replace=False)]
            mut = a + F_adapt * (b - c)
            mut = np.clip(mut, LO, HI)

            cross = rng.rand(dim) < 0.9
            if not np.any(cross): cross[rng.randint(dim)] = True
            tr = np.where(cross, mut, pop[i])

            # Inline repair
            tr[0] = tr[0] % (2 * np.pi); tr[1] = np.clip(tr[1], 70, 140)
            tr[2] = max(0, tr[2]); tr[3] = max(tr[2] + 0.1, min(tr[3], HI[3]))
            tr[4] = max(tr[2] + 1.0, tr[4]); tr[5] = max(tr[4] + 0.1, min(tr[5], HI[5]))
            tr[3] = min(tr[3], tr[2] + MAX_FALL); tr[5] = min(tr[5], tr[4] + MAX_FALL)

            ftr = fitness(tr)
            if ftr >= ft[i]:
                pop[i], ft[i] = tr, ftr
                if ftr > bf + 1e-8:
                    bx, bf = tr.copy(), ftr
                    gen_best = gen

        # Early stopping
        if gen - gen_best >= early_stop:
            break

    return bx, bf


# ---- Main ----
if __name__ == '__main__':
    print("=" * 60)
    print("  Q3: Optimized DE (adaptive F, early stop, geo seeds)")
    print(f"  popsize=30, maxiter=150, early_stop=80, DT={DT}s")
    print("=" * 60)

    t0 = time.time()
    print("\n  Generating geo seeds (DT=0.005)...")
    seeds = gen_seeds_fine(20)
    print(f"  {len(seeds)} seeds found")

    results = []
    for run in range(5):
        seed = 42 + run * 137
        t_run = time.time()
        x_opt, f_opt = de_optimize_fast(popsize=30, maxiter=150, seed=seed, early_stop=80)
        ff, o1, o2 = verify_fine(x_opt)
        elapsed = time.time() - t_run
        a, v, td1, tb1, td2, tb2 = x_opt
        print(f"  Run{run+1} seed={seed:4d}: DE={f_opt:.4f}s fine={ff:.4f}s "
              f"(o1={o1:.3f} o2={o2:.3f}) a={np.degrees(a):.1f}deg v={v:.1f} "
              f"({elapsed:.1f}s)")
        results.append((seed, x_opt, f_opt, ff, o1, o2, elapsed))

    results.sort(key=lambda r: -r[3])
    best = results[0]
    ao, vo, td1o, tb1o, td2o, tb2o = best[1]
    all_f = [r[3] for r in results]

    print(f"\n  Best (fine DT): {max(all_f):.4f}s | mean: {np.mean(all_f):.4f} | "
          f"std: {np.std(all_f):.4f}")
    print(f"  a={np.degrees(ao):.4f}deg v={vo:.4f} td1={td1o:.4f} tb1={tb1o:.4f} "
          f"td2={td2o:.4f} tb2={tb2o:.4f}")
    print(f"  Total: {time.time()-t0:.1f}s (vs old 168s)")
    print("=" * 60)
