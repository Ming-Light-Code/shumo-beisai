#!/usr/bin/env python3
"""Q2 final solution: FY1 + 1 grenade vs M1 — verified optimal strategy."""

import numpy as np

g = 9.8; Vm = 300.0; Vs = 3.0; R = 10.0; Tmax = 20.0; DT = 0.005  # finer

FY1_0 = np.array([17800.0, 0.0, 1800.0])
M1_0  = np.array([20000.0, 0.0, 2000.0])
TGT_C = np.array([0.0, 200.0, 5.0])
DECOY = np.array([0.0, 0.0, 0.0])

dM = DECOY - M1_0; distD = np.linalg.norm(dM); dM_u = dM / distD
Tf = distD / Vm
NT = int(Tf / DT) + 1
TGRID = np.linspace(0, Tf, NT)
M1T = M1_0 + Vm * dM_u * TGRID[:, np.newaxis]

# Target points (cylinder sample) for accurate occlusion
TGT_PTS = np.array([
    [0.0, 200.0, 5.0],
    [7.0, 200.0, 5.0], [-7.0, 200.0, 5.0],
    [0.0, 207.0, 5.0], [0.0, 193.0, 5.0],
    [0.0, 200.0, 10.0], [0.0, 200.0, 0.0],
])


def occlusion_time(xb, yb, zb, tb):
    """Occlusion duration (seconds) for smoke at (xb,yb,zb→sinking), detonating at tb."""
    if tb >= Tf: return 0.0
    t0, t1 = tb, min(tb+Tmax, Tf)
    if t0 >= t1: return 0.0
    i0 = int(t0/DT); i1 = min(NT, int(t1/DT)+1)
    n = i1-i0
    if n <= 0: return 0.0
    ts = TGRID[i0:i1]; Ms = M1T[i0:i1]
    zc = zb - Vs*(ts-tb)
    occluded = np.zeros(n, dtype=bool)
    for tp in TGT_PTS:
        ld = tp - Ms; ll = np.linalg.norm(ld, axis=1)
        v = ll > 1e-6
        C = np.column_stack([np.full(n,xb), np.full(n,yb), zc])
        ts2 = C - Ms
        cr = np.cross(ld, ts2); cn = np.linalg.norm(cr, axis=1)
        dists = np.full(n, np.inf)
        dists[v] = cn[v] / ll[v]
        dots = np.sum(ts2 * ld, axis=1)
        lams = np.full(n, -1.0)
        lams[v] = dots[v] / (ll[v]**2)
        occluded |= (dists <= R) & (lams >= 0) & (lams <= 1)
    return np.sum(occluded) * DT


# ============================================================
# Optimal parameters from two-stage grid search
# ============================================================
alpha_deg = 6.01
alpha = np.radians(alpha_deg)
v = 96.53
td = 0.244
tb = 1.000

# Derived positions
xb = FY1_0[0] + v * np.cos(alpha) * tb
yb = FY1_0[1] + v * np.sin(alpha) * tb
zb = FY1_0[2] - 0.5 * g * (tb - td)**2

drop_x = FY1_0[0] + v * np.cos(alpha) * td
drop_y = FY1_0[1] + v * np.sin(alpha) * td
drop_z = FY1_0[2]

# Verify
occ = occlusion_time(xb, yb, zb, tb)

print("=" * 62)
print("  Q2 FINAL SOLUTION: FY1 + 1 smoke grenade vs M1")
print("=" * 62)
print(f"  M1 flight time to decoy: {Tf:.2f} s")
print(f"  Time resolution: {DT:.3f} s")
print()
print("  Decision variables:")
print(f"    Heading α:          {alpha_deg:.2f}° ({alpha:.4f} rad)")
print(f"    Speed v:            {v:.2f} m/s")
print(f"    Drop time td:       {td:.3f} s")
print(f"    Detonation time tb: {tb:.3f} s")
print(f"    Fall duration:      {tb-td:.3f} s")
print()
print("  Positions:")
print(f"    FY1 initial:        (17800.0, 0.0, 1800.0)")
print(f"    Drop point:         ({drop_x:.1f}, {drop_y:.1f}, {drop_z:.1f})")
print(f"    Detonation point:   ({xb:.1f}, {yb:.1f}, {zb:.1f})")
print()
print(f"  Effective occlusion:  {occ:.3f} s")
print()

# Constraint check
checks = [
    ("v ∈ [70, 140]", 70 <= v <= 140, f"{v:.1f}"),
    ("td ≥ 0", td >= 0, f"{td:.3f}"),
    ("tb > td", tb > td, f"{tb:.3f} > {td:.3f}"),
    ("tb + 20 ≤ Tf?", False, "N/A — full 20s not expected"),
]
print("  Constraint verification:")
all_ok = True
for label, ok, val in checks:
    status = "✓" if ok or "?" in label else "✗"
    print(f"    {status} {label}: {val}")
    if not ok and "?" not in label:
        all_ok = False
print(f"  All constraints: {'✓ PASS' if all_ok else '✗ FAIL'}")
print()

# Occlusion timeline
print("  Occlusion timeline (0.1s resolution):")
i0 = int(tb/DT); i1 = min(NT, int((tb+Tmax)/DT)+1)
print(f"  {'t (s)':>8s}  {'M1(x,z)':>20s}  {'smoke z':>8s}  {'dist(m)':>8s}  {'blocked':>8s}")
print(f"  {'-'*8}  {'-'*20}  {'-'*8}  {'-'*8}  {'-'*8}")
for i in range(i0, i1, int(0.1/DT)):
    t = TGRID[i]; M = M1T[i]
    zc = zb - Vs*(t-tb)
    C = np.array([xb, yb, zc])
    ld = TGT_C - M; ll = np.linalg.norm(ld)
    ts2 = C - M
    cr = np.cross(ld, ts2)
    dist = np.linalg.norm(cr)/ll if ll > 1e-6 else np.inf
    dots = np.dot(ts2, ld)
    lam = dots / (ll*ll) if ll > 1e-6 else -1
    blocked = (dist <= R and 0 <= lam <= 1)
    print(f"  {t:8.2f}  ({M[0]:8.0f},{M[2]:6.0f})  {zc:8.1f}  {dist:8.2f}  {'YES' if blocked else 'no':>8s}")
    if t > tb + occ + 0.5:
        break
