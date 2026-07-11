#!/usr/bin/env python3
"""Q3: Push occlusion beyond 5.85s – DE at DT=0.005 + local refinement."""

import numpy as np
import time

g = 9.8; Vm = 300.0; Vs = 3.0; R = 10.0; Tmax = 20.0; DT = 0.005
FY1_0 = np.array([17800.0, 0.0, 1800.0]); M1_0 = np.array([20000.0, 0.0, 2000.0])
TGT_C = np.array([0.0, 200.0, 5.0]); DECOY = np.array([0.0, 0.0, 0.0])
dM = DECOY - M1_0; distD = np.linalg.norm(dM); dM_u = dM / distD
Tf = distD / Vm; NT = int(Tf / DT) + 1
TGRID = np.linspace(0, Tf, NT)
M1T = M1_0 + Vm * dM_u * TGRID[:, np.newaxis]
TGT_PTS = np.array([[0,200,5],[7,200,5],[-7,200,5],[0,207,5],[0,193,5],[0,200,10],[0,200,0]])
MAX_FALL = np.sqrt(2 * FY1_0[2] / g)

def occ_time(xb, yb, zb, tb):
    if tb >= Tf: return 0.0
    t0, t1 = tb, min(tb + Tmax, Tf)
    if t0 >= t1: return 0.0
    i0 = int(t0 / DT); i1 = min(NT, int(t1 / DT) + 1); n = i1 - i0
    if n <= 0: return 0.0
    ts = TGRID[i0:i1]; Ms = M1T[i0:i1]; zc = zb - Vs * (ts - tb)
    occ = np.zeros(n, dtype=bool)
    for tp in TGT_PTS:
        ld = tp - Ms; ll = np.linalg.norm(ld, axis=1); v = ll > 1e-9
        C = np.column_stack([np.full(n, xb), np.full(n, yb), zc])
        ts2 = C - Ms; cr = np.cross(ld, ts2); cn = np.linalg.norm(cr, axis=1)
        ds = np.full(n, np.inf); ds[v] = cn[v] / ll[v]
        dt_d = np.sum(ts2 * ld, axis=1)
        lm = np.full(n, -1.0); lm[v] = dt_d[v] / (ll[v] ** 2)
        occ |= (ds <= R) & (lm >= 0) & (lm <= 1)
    return np.sum(occ) * DT

def occ_union(xb1, yb1, zb1, tb1, xb2, yb2, zb2, tb2):
    if tb1 >= Tf and tb2 >= Tf: return 0.0
    i0 = int(max(0, min(tb1, tb2)) / DT)
    i1 = min(NT, int(Tf / DT) + 1); n = i1 - i0
    if n <= 0: return 0.0
    ts = TGRID[i0:i1]; Ms = M1T[i0:i1]
    ou = np.zeros(n, dtype=bool)
    for xb, yb, zb, tb in [(xb1, yb1, zb1, tb1), (xb2, yb2, zb2, tb2)]:
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
    return np.sum(ou) * DT

def fitness(x):
    a, v, td1, tb1, td2, tb2 = x
    xb1 = FY1_0[0] + v * np.cos(a) * tb1; yb1 = FY1_0[1] + v * np.sin(a) * tb1
    zb1 = FY1_0[2] - 0.5 * g * (tb1 - td1) ** 2
    if zb1 < -10: zb1 = -10
    xb2 = FY1_0[0] + v * np.cos(a) * tb2; yb2 = FY1_0[1] + v * np.sin(a) * tb2
    zb2 = FY1_0[2] - 0.5 * g * (tb2 - td2) ** 2
    if zb2 < -10: zb2 = -10
    return occ_union(xb1, yb1, zb1, tb1, xb2, yb2, zb2, tb2)

LO = np.array([0.0, 70.0, 0.0, 0.1, 1.0, 1.5])
HI = np.array([2.0 * np.pi, 140.0, Tf, Tf, Tf, Tf - 0.1])

def grid_refine(x):
    """Systematic local grid search at DT=0.005."""
    xb = np.array(x, dtype=float); fb = fitness(xb)
    for sf in [5e-3, 2e-3, 1e-3, 5e-4, 1e-4]:
        imp = True
        while imp:
            imp = False
            for d in range(6):
                for sgn in [-1, 1]:
                    xt = xb.copy(); xt[d] += sgn * sf
                    xt[0] %= 2 * np.pi; xt[1] = np.clip(xt[1], 70, 140)
                    xt[2] = max(0, xt[2]); xt[3] = max(xt[2] + 0.01, min(xt[3], HI[3]))
                    xt[4] = max(xt[2] + 1.0, xt[4]); xt[5] = max(xt[4] + 0.01, min(xt[5], HI[5]))
                    xt[3] = min(xt[3], xt[2] + MAX_FALL)
                    xt[5] = min(xt[5], xt[4] + MAX_FALL)
                    ft = fitness(xt)
                    if ft > fb + 1e-10:
                        xb = xt; fb = ft; imp = True
            sf *= 0.1
    return xb, fb

print("=" * 60)
print("  Q3: Pushing beyond 5.85s")
print(f"  DT={DT}s – direct optimization at fine resolution")
print("=" * 60)

t0 = time.time()

# ===== Strategy 1: Grid-refine the existing best DE solution =====
print("\n  [1] Grid-refining existing DE best (a=5.8deg, v=70.9, tb1=0.1, tb2=1.5)...")
best_old = np.array([np.radians(5.8067), 70.915, 0.0, 0.1, 1.0, 1.5])
xr1, fr1 = grid_refine(best_old)
a1, v1, td11, tb11, td21, tb21 = xr1
print(f"  Refined: {fr1:.6f}s (was ~5.850s)")
print(f"    a={np.degrees(a1):.6f}deg v={v1:.6f} td1={td11:.6f} tb1={tb11:.6f} td2={td21:.6f} tb2={tb21:.6f}")

# ===== Strategy 2: Systematic perturbation sweep =====
print("\n  [2] Systematic perturbation sweep around best solution...")
rng = np.random.RandomState(0)
best_so_far = xr1.copy(); best_f_so_far = fr1

# Try different alpha values systematically
for a_try in np.linspace(2, 15, 30):
    for v_try in np.linspace(70, 100, 15):
        for tb1_try in [0.05, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20, 0.25, 0.30]:
            for tb2_try in np.linspace(1.2, 4.0, 15):
                td1_try = 0.0
                td2_try = td1_try + 1.0
                x_try = np.array([np.radians(a_try), v_try, td1_try, tb1_try, td2_try, tb2_try])
                if x_try[3] <= x_try[2] or x_try[5] <= x_try[4]:
                    continue
                if x_try[3] > x_try[2] + MAX_FALL or x_try[5] > x_try[4] + MAX_FALL:
                    continue
                ft = fitness(x_try)
                if ft > best_f_so_far:
                    best_f_so_far = ft
                    best_so_far = x_try.copy()
                    print(f"  NEW BEST: {ft:.6f}s a={a_try:.2f}deg v={v_try:.1f} tb1={tb1_try:.3f} tb2={tb2_try:.3f}")
                    break
            if ft > best_f_so_far: break
        if ft > best_f_so_far: break
    if ft > best_f_so_far: break

a2, v2, td12, tb12, td22, tb22 = best_so_far
print(f"\n  Best after sweep: {best_f_so_far:.6f}s")
print(f"    a={np.degrees(a2):.4f}deg v={v2:.4f} td1={td12:.4f} tb1={tb12:.4f} td2={td22:.4f} tb2={tb22:.4f}")

# Grid refine again
xr_final, fr_final = grid_refine(best_so_far)
a_f, v_f, td1f, tb1f, td2f, tb2f = xr_final
print(f"\n  After final grid refine: {fr_final:.6f}s")
print(f"    a={np.degrees(a_f):.6f}deg v={v_f:.6f} td1={td1f:.6f} tb1={tb1f:.6f} td2={td2f:.6f} tb2={tb2f:.6f}")

# Compute individual occlusions
xb1 = FY1_0[0] + v_f * np.cos(a_f) * tb1f
yb1 = FY1_0[1] + v_f * np.sin(a_f) * tb1f
zb1 = FY1_0[2] - 0.5 * g * (tb1f - td1f) ** 2
xb2 = FY1_0[0] + v_f * np.cos(a_f) * tb2f
yb2 = FY1_0[1] + v_f * np.sin(a_f) * tb2f
zb2 = FY1_0[2] - 0.5 * g * (tb2f - td2f) ** 2
o1 = occ_time(xb1, yb1, zb1, tb1f)
o2 = occ_time(xb2, yb2, zb2, tb2f)

print(f"\n  Grenade 1: occ={o1:.4f}s det=({xb1:.1f},{yb1:.1f},{zb1:.1f})")
print(f"  Grenade 2: occ={o2:.4f}s det=({xb2:.1f},{yb2:.1f},{zb2:.1f})")
print(f"  Overlap: {o1+o2-fr_final:.4f}s")
print(f"  Gain vs old best (5.850s): {fr_final-5.850:.4f}s")
print(f"\n  Total time: {time.time()-t0:.1f}s")
