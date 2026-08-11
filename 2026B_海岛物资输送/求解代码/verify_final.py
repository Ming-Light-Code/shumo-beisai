import numpy as np
from math import sqrt, sin, cos, pi

g = 9.8; Rs = 10.0; v_m = 300.0; v_c = 3.0
r_T = 7.0; h_T = 10.0; t_max_smoke = 20.0
target_center = np.array([0.0, 200.0, 0.0])
r_M1_0 = np.array([20000.0, 0.0, 2000.0])
d_M1 = -r_M1_0 / np.linalg.norm(r_M1_0)

def missile_pos(t):
    return r_M1_0 + v_m * d_M1 * t

def gen_samples(nc=256):
    pts = []
    for i in range(nc):
        a = 2*pi*i/nc; px=target_center[0]+r_T*cos(a); py=target_center[1]+r_T*sin(a)
        pts.append(np.array([px,py,target_center[2]]))
        pts.append(np.array([px,py,target_center[2]+h_T]))
    return pts

samples = gen_samples()

def smoke_at(t, bp, tb):
    if t < tb or t > tb + t_max_smoke: return None
    return bp - np.array([0.0, 0.0, v_c*(t-tb)])

def is_shielded(t, params):
    r_m = missile_pos(t)
    centers = [c for bp,tb in params if (c:=smoke_at(t,bp,tb)) is not None]
    if not centers: return False
    for P in samples:
        d = P - r_m; nd = np.linalg.norm(d)
        if nd < 1e-10: continue
        d = d / nd
        ok = False
        for sc in centers:
            v = sc - r_m; dmc = np.linalg.norm(v)
            if dmc <= Rs: ok = True; break
            tstar = np.dot(v, d)
            if tstar <= 0: continue
            dist = np.linalg.norm(r_m + tstar*d - sc)
            if dist <= Rs: ok = True; break
        if not ok: return False
    return True

def binary_search_boundary(t_left, t_right, target_state, params):
    for _ in range(50):
        t_mid = (t_left + t_right) / 2
        s = is_shielded(t_mid, params)
        if s == target_state: t_right = t_mid
        else: t_left = t_mid
    return (t_left + t_right) / 2

def solve_problem(params, t0=0, dt_coarse=0.001):
    # Coarse scan
    t, tmax = t0, 67.0
    states = []
    while t <= tmax:
        s = is_shielded(t, params)
        states.append(s); t += dt_coarse
    
    # Find intervals
    intervals = []; in_int = False; idx_start = 0
    for i, s in enumerate(states):
        if s and not in_int: idx_start = i; in_int = True
        elif not s and in_int:
            intervals.append((idx_start, i)); in_int = False
    if in_int: intervals.append((idx_start, len(states)-1))
    if not intervals: return 0, 0, 0
    
    best = max(intervals, key=lambda x: x[1]-x[0])
    t_rough_a = t0 + best[0]*dt_coarse
    t_rough_b = t0 + best[1]*dt_coarse
    
    # Binary refinement
    t_start = binary_search_boundary(t_rough_a - 2*dt_coarse, t_rough_a + dt_coarse, True, params)
    t_end = binary_search_boundary(t_rough_b - dt_coarse, t_rough_b + 2*dt_coarse, False, params)
    
    return t_start, t_end, t_end - t_start

# ========== P1 ==========
print("=== PROBLEM 1 ===")
FY1_0 = np.array([17800.0,0.0,1800.0]); v_d = np.array([-120.0,0.0,0.0])
bp1 = FY1_0 + v_d*1.5 + np.array([-120.0*3.6, 0.0, -0.5*g*3.6**2])
ts,te,dur = solve_problem([(bp1, 5.1)], 5.1)
print(f"ts={ts:.6f}  te={te:.6f}  dur={dur:.6f}s")
print(f"Paper: 8.056445  9.448088  1.391643")
err = abs(dur-1.391643)/1.391643*100
print(f"Error: {err:.3f}%")

# ========== P3 ==========
print("\n=== PROBLEM 3 ===")
F0 = np.array([17800.0,0.0,1800.0])
theta3 = np.radians(179.814307); v3 = 139.843372
d3 = np.array([cos(theta3), sin(theta3), 0.0])
tds = [0.001249, 1.380981, 4.267095]; taus = [0.003932, 4.144411, 5.707570]
p3 = []
for j in range(3):
    bp = F0 + v3*(tds[j]+taus[j])*d3 - np.array([0.0,0.0,0.5*g*taus[j]**2])
    p3.append((bp, tds[j]+taus[j]))
ts3,te3,dur3 = solve_problem(p3, 0)
print(f"ts={ts3:.6f}  te={te3:.6f}  dur={dur3:.6f}s")
print(f"Paper: 4.8600  12.0650  7.2100")
err3 = abs(dur3-7.2100)/7.2100*100
print(f"Error: {err3:.3f}%")

# ========== P4 ==========
print("\n=== PROBLEM 4 ===")
F4 = [np.array([17800.,0.,1800.]), np.array([12000.,1400.,1400.]), np.array([6000.,-3000.,700.])]
ths4 = [179.110941, 308.046683, 73.809276]
vs4 = [132.062839, 138.991477, 137.409810]
tds4 = [0.369336, 8.336915, 22.723803]; taus4 = [3.636731, 4.163631, 0.685502]
exp4 = [4.485, 3.985, 3.150]; total = 0
for i in range(3):
    th = np.radians(ths4[i]); di = np.array([cos(th), sin(th), 0.0])
    bp = F4[i] + vs4[i]*(tds4[i]+taus4[i])*di - np.array([0.,0.,0.5*g*taus4[i]**2])
    ts,te,dur = solve_problem([(bp, tds4[i]+taus4[i])], 0)
    total += dur
    err_i = abs(dur-exp4[i])/exp4[i]*100
    print(f"  FY{i+1}: ts={ts:.3f} te={te:.3f} dur={dur:.6f}s (exp:{exp4[i]}, err:{err_i:.3f}%)")
print(f"Total: {total:.6f}s (exp: 11.620, err: {abs(total-11.620)/11.620*100:.3f}%)")

print("\n=== FINAL SUMMARY ===")
print(f"P1: {dur:.6f}s (paper: 1.391643) error: {err:.4f}%")
print(f"P3: {dur3:.6f}s (paper: 7.210000) error: {err3:.4f}%")
print(f"P4: {total:.6f}s (paper: 11.620000) error: {abs(total-11.620)/11.620*100:.4f}%")