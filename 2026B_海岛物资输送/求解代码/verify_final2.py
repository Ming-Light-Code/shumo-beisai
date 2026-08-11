import numpy as np
from math import sqrt, sin, cos, pi, asin

g = 9.8; Rs = 10.0; v_m = 300.0; v_c = 3.0
r_T = 7.0; h_T = 10.0; t_max_smoke = 20.0
target_center = np.array([0.0, 200.0, 0.0])
r_M1_0 = np.array([20000.0, 0.0, 2000.0])
d_M1 = -r_M1_0 / np.linalg.norm(r_M1_0)

def missile_pos(t):
    return r_M1_0 + v_m * d_M1 * t

def gen_samples_paper(n_top=256, n_bot=256, n_sil=128):
    """Paper's method: top circle + bottom circle + silhouette lines"""
    pts = []
    # Top circle (z=h_T, center (0,200,h_T))
    for i in range(n_top):
        a = 2*pi*i/n_top
        pts.append(np.array([r_T*cos(a), 200+r_T*sin(a), h_T]))
    # Bottom circle (z=0, center (0,200,0))  
    for i in range(n_bot):
        a = 2*pi*i/n_bot
        pts.append(np.array([r_T*cos(a), 200+r_T*sin(a), 0.0]))
    # Silhouette: 128 lines * 2 directions? Actually 128 total points
    # Sample uniformly along height at 2 tangent azimuths
    for i in range(n_sil):
        z = h_T * i / max(n_sil-1, 1)
        # 2 tangent points at each height
        a1 = 2*pi*i/n_sil
        pts.append(np.array([r_T*cos(a1), 200+r_T*sin(a1), z]))
    return pts

samples = gen_samples_paper(256, 256, 128)
print(f"Using {len(samples)} sample points")

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

def solve_problem(params, t0=0, dt_coarse=0.001):
    t, tmax = t0, 67.0; states = []
    while t <= tmax:
        states.append(is_shielded(t, params)); t += dt_coarse
    intervals = []; in_int = False; idx_start = 0
    for i, s in enumerate(states):
        if s and not in_int: idx_start = i; in_int = True
        elif not s and in_int: intervals.append((idx_start, i)); in_int = False
    if in_int: intervals.append((idx_start, len(states)-1))
    if not intervals: return 0,0,0
    best = max(intervals, key=lambda x: x[1]-x[0])
    t_a = t0 + best[0]*dt_coarse; t_b = t0 + best[1]*dt_coarse
    
    # Binary refinement
    def bin_search(tL, tR, target):
        for _ in range(55):
            tM = (tL+tR)/2
            if is_shielded(tM, params) == target: tR = tM
            else: tL = tM
        return (tL+tR)/2
    ts = bin_search(t_a - 2*dt_coarse, t_a + dt_coarse, True)
    te = bin_search(t_b - dt_coarse, t_b + 2*dt_coarse, False)
    return ts, te, te-ts

# P1
FY1_0 = np.array([17800.0,0.0,1800.0]); v_d = np.array([-120.0,0.0,0.0])
bp1 = FY1_0 + v_d*1.5 + np.array([-120.0*3.6, 0.0, -0.5*g*3.6**2])
ts,te,dur = solve_problem([(bp1, 5.1)], 5.1)
print(f"P1: [{ts:.10f}, {te:.10f}] {dur:.10f}s (paper: 1.391643)")

# P3
F0 = np.array([17800.0,0.0,1800.0])
th3 = np.radians(179.814307); d3 = np.array([cos(th3), sin(th3), 0.0])
tds = [0.001249,1.380981,4.267095]; taus = [0.003932,4.144411,5.707570]
p3 = [(F0+139.843372*(tds[j]+taus[j])*d3-np.array([0.,0.,0.5*g*taus[j]**2]), tds[j]+taus[j]) for j in range(3)]
ts3,te3,dur3 = solve_problem(p3, 0)
print(f"P3: [{ts3:.10f}, {te3:.10f}] {dur3:.10f}s (paper: 7.210000)")

# P4
F4 = [np.array([17800.,0.,1800.]), np.array([12000.,1400.,1400.]), np.array([6000.,-3000.,700.])]
ths4 = [179.110941,308.046683,73.809276]; vs4 = [132.062839,138.991477,137.409810]
tds4 = [0.369336,8.336915,22.723803]; taus4 = [3.636731,4.163631,0.685502]
exp4 = [4.485,3.985,3.150]; tot=0
for i in range(3):
    th= np.radians(ths4[i]); di=np.array([cos(th),sin(th),0.])
    bp=F4[i]+vs4[i]*(tds4[i]+taus4[i])*di-np.array([0.,0.,0.5*g*taus4[i]**2])
    ts,te,dur=solve_problem([(bp,tds4[i]+taus4[i])],0)
    tot+=dur; err=abs(dur-exp4[i])/exp4[i]*100
    print(f"  FY{i+1}: {dur:.6f}s (err:{err:.4f}%%)")
print(f"P4 tot: {tot:.6f}s (err:{abs(tot-11.620)/11.620*100:.4f}%%)")