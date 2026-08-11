#!/usr/bin/env python3
"""Q3: FY1 + 2 grenades vs M1 - fixed geometric prediction."""

import numpy as np
from scipy.optimize import minimize, Bounds
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

def occ_time_union(xb1, yb1, zb1, tb1, xb2, yb2, zb2, tb2):
    if tb1 >= Tf and tb2 >= Tf: return 0.0
    t_start = max(0.0, min(tb1, tb2))
    t_end = Tf
    i0 = int(t_start / DT); i1 = min(NT, int(t_end / DT) + 1)
    n = i1 - i0
    if n <= 0: return 0.0
    ts = TGRID[i0:i1]; Ms = M1T[i0:i1]
    occ1 = np.zeros(n, dtype=bool)
    occ2 = np.zeros(n, dtype=bool)
    for idx_g, (xb, yb, zb, tb) in enumerate([(xb1,yb1,zb1,tb1), (xb2,yb2,zb2,tb2)]):
        if tb >= Tf: continue
        zc = zb - Vs * np.maximum(0, ts - tb)
        occ = np.zeros(n, dtype=bool)
        for tp in TGT_PTS:
            ld = tp - Ms; ll = np.linalg.norm(ld, axis=1)
            v = ll > 1e-9
            C = np.column_stack([np.full(n, xb), np.full(n, yb), zc])
            ts2 = C - Ms; cr = np.cross(ld, ts2); cn = np.linalg.norm(cr, axis=1)
            ds = np.full(n, np.inf); ds[v] = cn[v] / ll[v]
            dt = np.sum(ts2 * ld, axis=1); lm = np.full(n, -1.0); lm[v] = dt[v] / (ll[v] ** 2)
            occ |= (ds <= R) & (lm >= 0) & (lm <= 1) & (ts >= tb)
        if idx_g == 0: occ1 = occ
        else: occ2 = occ
    return np.sum(occ1 | occ2) * DT

def occ_from_vars_q3(x):
    alpha, v, td1, tb1, td2, tb2 = x
    xb1 = FY1_0[0] + v * np.cos(alpha) * tb1
    yb1 = FY1_0[1] + v * np.sin(alpha) * tb1
    zb1 = FY1_0[2] - 0.5 * g * (tb1 - td1) ** 2
    if zb1 < -10: zb1 = -10
    xb2 = FY1_0[0] + v * np.cos(alpha) * tb2
    yb2 = FY1_0[1] + v * np.sin(alpha) * tb2
    zb2 = FY1_0[2] - 0.5 * g * (tb2 - td2) ** 2
    if zb2 < -10: zb2 = -10
    return occ_time_union(xb1, yb1, zb1, tb1, xb2, yb2, zb2, tb2)

def disc_obj_q3(x): return -occ_from_vars_q3(x)

# ============================================================
# Improved geometric prediction for pairs
# ============================================================
def gen_pair_starts(max_s=50):
    """Generate pairs by combining Q2-like geometric prediction for grenade 1
    with FY1-trajectory-based sampling for grenade 2."""
    starts = []

    # Get Q2 starts as grenade 1 candidates
    from q2_geo_nlp import gen_starts
    import importlib.util
    # Inline the Q2 starts generation
    tb_c = np.linspace(0.5, Tf - 0.5, 30)
    tb_f = np.linspace(1.0, min(15, Tf - 0.5), 20)
    tb_vals = np.unique(np.concatenate([tb_c, tb_f]))

    q2_starts = []
    for tb in tb_vals:
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
                if np.linalg.norm(perp) < 0.1: perp = np.cross(lu, np.array([0, 1, 0]))
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
                    if oc > 0.01: q2_starts.append((alpha, v, td, tb, oc))

    q2_starts.sort(key=lambda s: -s[4])
    # Dedup Q2 starts
    q2_filtered = []
    for s in q2_starts:
        a, v, td, tb, oc = s
        tc = False
        for f in q2_filtered:
            da = abs(a - f[0])
            if da > np.pi: da = 2 * np.pi - da
            if da < 0.015 and abs(v - f[1]) < 1.5 and abs(tb - f[3]) < 0.4:
                tc = True; break
        if not tc: q2_filtered.append(s)
        if len(q2_filtered) >= 25: break

    print(f"  Q2 starts for grenade 1: {len(q2_filtered)} candidates")

    # For each Q2 start, try second grenade at various tb2
    for a, v, td1, tb1, occ1 in q2_filtered:
        xb1 = FY1_0[0] + v * np.cos(a) * tb1
        yb1 = FY1_0[1] + v * np.sin(a) * tb1
        zb1 = FY1_0[2] - 0.5 * g * (tb1 - td1) ** 2

        # Second grenade: sample tb2 > td1 + 1
        tb2_min = max(tb1 + 0.5, td1 + 1.0)
        for tb2 in np.linspace(tb2_min, min(tb2_min + 25, Tf - 2), 20):
            xb2 = FY1_0[0] + v * np.cos(a) * tb2
            yb2 = FY1_0[1] + v * np.sin(a) * tb2

            # Try different td2 values (different smoke heights)
            for td2_fac in np.linspace(0.0, 0.95, 10):
                td2 = td1 + 1.0 + td2_fac * (tb2 - td1 - 1.0)
                if td2 >= tb2: continue
                zb2 = FY1_0[2] - 0.5 * g * (tb2 - td2) ** 2

                total = occ_time_union(xb1, yb1, zb1, tb1, xb2, yb2, zb2, tb2)
                if total > occ1 + 0.05:
                    starts.append((a, v, td1, tb1, td2, tb2, total))

    # Sort
    starts.sort(key=lambda s: -s[6])

    # Dedup
    filtered = []
    for s in starts:
        a, v, td1, tb1, td2, tb2, total = s
        tc = False
        for f in filtered:
            da = abs(a - f[0])
            if da > np.pi: da = 2 * np.pi - da
            if da < 0.02 and abs(v - f[1]) < 2.0 and abs(tb1 - f[3]) < 0.3 and abs(tb2 - f[5]) < 0.5:
                tc = True; break
        if not tc: filtered.append(s)
        if len(filtered) >= max_s: break

    return filtered[:max_s]

# Bounds and constraints
nlp_bounds_q3 = Bounds(
    [0.0,           70.0,   0.0,   0.1,   1.0,   1.1],
    [2.0 * np.pi,  140.0,  Tf,    Tf,    Tf,    Tf - 0.1]
)

nlp_cons_q3 = [
    {'type': 'ineq', 'fun': lambda x: x[3] - x[2] - 1e-6},
    {'type': 'ineq', 'fun': lambda x: x[5] - x[4] - 1e-6},
    {'type': 'ineq', 'fun': lambda x: x[4] - x[2] - 1.0},
    {'type': 'ineq', 'fun': lambda x: FY1_0[2] - 0.5*g*(x[3]-x[2])**2 + 1e-6},
    {'type': 'ineq', 'fun': lambda x: FY1_0[2] - 0.5*g*(x[5]-x[4])**2 + 1e-6},
]

def run_nlp_q3(x0, maxiter=600):
    x = np.array(x0, dtype=float)
    res = minimize(disc_obj_q3, x, method='SLSQP', bounds=nlp_bounds_q3,
                   constraints=nlp_cons_q3,
                   options={'maxiter': maxiter, 'ftol': 1e-10, 'disp': False})
    return res.x, -res.fun, res.success

def run_nm_q3(x0, maxiter=800):
    def po(x):
        a, v, td1, tb1, td2, tb2 = x; p = 0
        if v < 70: p += (70 - v) * 1000
        if v > 140: p += (v - 140) * 1000
        if td1 < 0: p += (-td1) * 1000
        if tb1 <= td1: p += (td1 - tb1 + 0.01) * 1000
        if tb1 >= Tf: p += (tb1 - Tf + 0.01) * 1000
        if td2 < td1 + 1: p += (td1 + 1 - td2) * 1000
        if tb2 <= td2: p += (td2 - tb2 + 0.01) * 1000
        if tb2 >= Tf: p += (tb2 - Tf + 0.01) * 1000
        zb1 = FY1_0[2] - 0.5 * g * (tb1 - td1) ** 2
        zb2 = FY1_0[2] - 0.5 * g * (tb2 - td2) ** 2
        if zb1 < -1: p += (-zb1) * 1000
        if zb2 < -1: p += (-zb2) * 1000
        return -occ_from_vars_q3(x) + p
    r = minimize(po, np.array(x0), method='Nelder-Mead',
                 options={'maxiter': maxiter, 'xatol': 1e-6, 'fatol': 1e-6, 'disp': False})
    return r.x, -r.fun, r.success

def grid_refine_q3(x):
    xb = np.array(x, dtype=float); fb = occ_from_vars_q3(xb)
    for sf in [1e-3, 5e-3, 1e-4]:
        imp = True
        while imp:
            imp = False
            for d in range(6):
                for sgn in [-1, 1]:
                    xt = xb.copy(); xt[d] += sgn * sf
                    xt[0] %= 2 * np.pi; xt[1] = np.clip(xt[1], 70, 140)
                    xt[2] = max(0, xt[2]); xt[3] = np.clip(xt[3], xt[2] + 1e-6, Tf - 1e-6)
                    xt[4] = max(xt[2] + 1.0, xt[4])
                    xt[5] = np.clip(xt[5], xt[4] + 1e-6, Tf - 1e-6)
                    ft = occ_from_vars_q3(xt)
                    if ft > fb + 1e-8: xb = xt; fb = ft; imp = True
            sf *= 0.1
    return xb, fb

if __name__ == '__main__':
    print("=" * 70)
    print("  Q3: FY1 + 2 Grenades vs M1 (Fixed)")
    print(f"  M1 flight: {Tf:.2f}s | DT={DT}s")
    print("=" * 70)

    t0 = time.time()
    np.random.seed(42)

    print("\n  [1] Geometric prediction for pairs...")
    pair_starts = gen_pair_starts(40)
    t1 = time.time()
    print(f"  Found {len(pair_starts)} valid pair starts ({t1-t0:.2f}s)")

    if not pair_starts:
        print("  ERROR: no starts!"); sys.exit(1)

    print(f"\n  Top 10 pair starts:")
    for i, s in enumerate(pair_starts[:10]):
        a, v, td1, tb1, td2, tb2, total = s
        xb1 = FY1_0[0] + v * np.cos(a) * tb1
        yb1 = FY1_0[1] + v * np.sin(a) * tb1
        zb1 = FY1_0[2] - 0.5 * g * (tb1 - td1) ** 2
        xb2 = FY1_0[0] + v * np.cos(a) * tb2
        yb2 = FY1_0[1] + v * np.sin(a) * tb2
        zb2 = FY1_0[2] - 0.5 * g * (tb2 - td2) ** 2
        o1 = occ_time(xb1, yb1, zb1, tb1)
        o2 = occ_time(xb2, yb2, zb2, tb2)
        print(f"    {i+1:2d}. a={np.degrees(a):6.2f} v={v:6.1f} tb1={tb1:5.2f} tb2={tb2:5.2f} total={total:.3f} (o1={o1:.3f} o2={o2:.3f})")

    print(f"\n  [2] Running NLP ({len(pair_starts)} starts)...")
    res = []
    for i, s in enumerate(pair_starts):
        a, v, td1, tb1, td2, tb2, oi = s
        x0 = np.array([a, v, td1, tb1, td2, tb2])
        try: xo, fo, ok = run_nlp_q3(x0)
        except: ok = False
        if not ok or fo < oi * 0.3:
            try: xo, fo, ok = run_nm_q3(x0)
            except: ok = False
        if not ok: xo, fo = x0, oi
        xr, fr = grid_refine_q3(xo)
        res.append({'i': i, 'io': oi, 'x': xr, 'f': fr})
        if (i + 1) % 8 == 0:
            bf = max(r['f'] for r in res)
            print(f"    ... {i+1}/{len(pair_starts)} done ({time.time()-t1:.1f}s), best={bf:.4f}s")

    t2 = time.time(); res.sort(key=lambda r: -r['f'])
    print(f"\n  NLP done: {t2-t1:.2f}s")
    print(f"\n  Top 15 results:")
    for i, r in enumerate(res[:15]):
        a, v, td1, tb1, td2, tb2 = r['x']; oc = r['f']
        print(f"    {i+1:2d}. a={np.degrees(a):8.3f} v={v:7.2f} td1={td1:6.3f} tb1={tb1:6.3f} td2={td2:6.3f} tb2={tb2:6.3f} occ={oc:.4f}s")

    best = res[0]; ao, vo, td1o, tb1o, td2o, tb2o = best['x']; oco = best['f']
    xb1o = FY1_0[0] + vo * np.cos(ao) * tb1o
    yb1o = FY1_0[1] + vo * np.sin(ao) * tb1o
    zb1o = FY1_0[2] - 0.5 * g * (tb1o - td1o) ** 2
    xb2o = FY1_0[0] + vo * np.cos(ao) * tb2o
    yb2o = FY1_0[1] + vo * np.sin(ao) * tb2o
    zb2o = FY1_0[2] - 0.5 * g * (tb2o - td2o) ** 2
    o1 = occ_time(xb1o, yb1o, zb1o, tb1o)
    o2 = occ_time(xb2o, yb2o, zb2o, tb2o)

    print("\n" + "=" * 70)
    print("  Q3 GLOBAL OPTIMAL")
    print("=" * 70)
    print(f"  FY1: a={np.degrees(ao):.4f}deg v={vo:.4f} m/s")
    print(f"  G1:  td1={td1o:.4f}s tb1={tb1o:.4f}s det=({xb1o:.1f},{yb1o:.1f},{zb1o:.1f}) occ={o1:.4f}s")
    print(f"  G2:  td2={td2o:.4f}s tb2={tb2o:.4f}s det=({xb2o:.1f},{yb2o:.1f},{zb2o:.1f}) occ={o2:.4f}s")
    print(f"  drop interval: {td2o-td1o:.4f}s")
    print(f"  Total occultation: {oco:.4f}s")
    print(f"  vs Q2 (1 grenade): {oco - 4.955:.4f}s gain ({(oco-4.955)/4.955*100:.1f}%)")
    print(f"  Overlap: {o1+o2-oco:.4f}s")
    print(f"  Total time: {time.time()-t0:.1f}s")
    print("=" * 70)
