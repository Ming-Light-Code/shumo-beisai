import numpy as np
from math import sqrt, sin, cos, pi, asin

# ============================================================
# COMMON PARAMETERS
# ============================================================
g = 9.8
Rs = 10.0
v_m = 300.0
v_c = 3.0
r_T = 7.0
h_T = 10.0
target_center = np.array([0.0, 200.0, 0.0])
t_max_smoke = 20.0
t_hit = 67.0

r_M1_0 = np.array([20000.0, 0.0, 2000.0])
d_M1 = -r_M1_0 / np.linalg.norm(r_M1_0)

def missile_pos(t):
    return r_M1_0 + v_m * d_M1 * t

def generate_target_samples(n_circle=256, n_sil=128):
    points = []
    for i in range(n_circle):
        angle = 2 * pi * i / n_circle
        px = target_center[0] + r_T * np.cos(angle)
        py = target_center[1] + r_T * np.sin(angle)
        points.append(np.array([px, py, target_center[2]]))
        points.append(np.array([px, py, target_center[2] + h_T]))
    for i in range(n_sil):
        z = target_center[2] + h_T * i / max(n_sil - 1, 1)
        angle = 2 * pi * i / n_sil
        px = target_center[0] + r_T * np.cos(angle)
        py = target_center[1] + r_T * np.sin(angle)
        points.append(np.array([px, py, z]))
    return points

target_samples = generate_target_samples()
print(f"Total sample points: {len(target_samples)}")

def check_shielding(r_m, smoke_centers):
    for P in target_samples:
        direction = P - r_m
        dist_mp = np.linalg.norm(direction)
        if dist_mp < 1e-10:
            continue
        direction = direction / dist_mp
        occluded = False
        for sc in smoke_centers:
            to_sc = sc - r_m
            proj = np.dot(to_sc, direction)
            if proj <= 0:
                continue
            closest = r_m + proj * direction
            dist_to_line = np.linalg.norm(closest - sc)
            if dist_to_line <= Rs:
                occluded = True
                break
        if not occluded:
            return False
    return True

def smoke_center_at(t, burst_pos, t_burst):
    if t < t_burst or t > t_burst + t_max_smoke:
        return None
    return burst_pos - np.array([0.0, 0.0, v_c * (t - t_burst)])

def binary_search(t_left, t_right, target_state, smoke_params_list):
    def is_shielded(t):
        r_m = missile_pos(t)
        centers = []
        for bp, tb in smoke_params_list:
            sc = smoke_center_at(t, bp, tb)
            if sc is not None:
                centers.append(sc)
        return check_shielding(r_m, centers) if centers else False
    for _ in range(50):
        t_mid = (t_left + t_right) / 2
        if is_shielded(t_mid) == target_state:
            t_right = t_mid
        else:
            t_left = t_mid
    return t_left

def find_shielding_interval(smoke_params_list, t_start_scan=0, dt_coarse=0.002):
    t_end_scan = t_hit
    
    states = []
    t = t_start_scan
    while t <= t_end_scan:
        r_m = missile_pos(t)
        centers = []
        for bp, tb in smoke_params_list:
            sc = smoke_center_at(t, bp, tb)
            if sc is not None:
                centers.append(sc)
        shielded = check_shielding(r_m, centers) if centers else False
        states.append((t, shielded))
        t += dt_coarse
    
    intervals = []
    in_interval = False
    t_start = 0
    for i, (t_val, s) in enumerate(states):
        if s and not in_interval:
            t_start = states[max(0, i-1)][0]
            in_interval = True
        elif not s and in_interval:
            intervals.append((t_start, t_val))
            in_interval = False
    if in_interval:
        intervals.append((t_start, t_end_scan))
    
    if not intervals:
        return None, None, 0.0
    
    longest = max(intervals, key=lambda x: x[1] - x[0])
    
    t_start_refined = binary_search(longest[0] - dt_coarse*2, longest[0], True, smoke_params_list)
    t_end_refined = binary_search(longest[1], min(longest[1] + dt_coarse*2, t_end_scan), False, smoke_params_list)
    
    return t_start_refined, t_end_refined, t_end_refined - t_start_refined

# ============================================================
# PROBLEM 1 VERIFICATION
# ============================================================
print("=== PROBLEM 1 ===")
FY1_0 = np.array([17800.0, 0.0, 1800.0])
v_FY1 = np.array([-120.0, 0.0, 0.0])
td_1 = 1.5
tau_1 = 3.6
tb_1 = td_1 + tau_1

release_pos_1 = FY1_0 + v_FY1 * td_1
burst_pos_1 = release_pos_1 + np.array([-120.0 * tau_1, 0.0, -0.5 * g * tau_1**2])
print(f"Burst: ({burst_pos_1[0]:.6f}, {burst_pos_1[1]:.6f}, {burst_pos_1[2]:.6f})")
print(f"Expected: (17188.000000, 0.000000, 1736.496000)")

print("Computing shielding (coarse scan dt=0.01s)...")
t1_start, t1_end, t1_dur = find_shielding_interval([(burst_pos_1, tb_1)], t_start_scan=tb_1)
print(f"Start:  {t1_start:.6f}s (expected: 8.056445)")
print(f"End:    {t1_end:.6f}s (expected: 9.448088)")
print(f"Duration: {t1_dur:.6f}s (expected: 1.391643)")
diff = abs(t1_dur - 1.391643) / 1.391643 * 100
print(f"Relative error: {diff:.2f}%")