import math

# ===== Parameters =====
g = 9.8
R_s = 10.0          # smoke effective radius (m)
T_s = 20.0          # smoke effective duration (s)
v_m = 300.0         # missile speed (m/s)
v_d = 120.0         # drone speed (m/s)
v_c = 3.0           # smoke sink speed (m/s)
t_d = 1.5           # release time (s)
t_b = 5.1           # detonation time (s)
r_T = 7.0           # target radius (m)
h_T = 10.0          # target height (m)

# ===== Initial positions =====
M1_0 = (20000.0, 0.0, 2000.0)
FY1_0 = (17800.0, 0.0, 1800.0)
P_T_center = (0.0, 200.0, 5.0)       # true target geometric center

# ===== Missile M1 motion =====
d_M1 = (-M1_0[0], -M1_0[1], -M1_0[2])
norm_M1 = math.sqrt(d_M1[0]**2 + d_M1[1]**2 + d_M1[2]**2)
d_M1 = (d_M1[0]/norm_M1, d_M1[1]/norm_M1, d_M1[2]/norm_M1)
print(f"M1 dir: ({d_M1[0]:.6f}, {d_M1[1]:.6f}, {d_M1[2]:.6f})")
print(f"M1 dist to origin: {norm_M1:.2f} m, hit time: {norm_M1/v_m:.2f} s")

def M1_pos(t):
    return (M1_0[0] + v_m * d_M1[0] * t,
            M1_0[1] + v_m * d_M1[1] * t,
            M1_0[2] + v_m * d_M1[2] * t)

# ===== Drone FY1 motion =====
# Flies toward decoy direction at constant altitude
dx_fy = -FY1_0[0]
dy_fy = -FY1_0[1]
norm_fy_xy = math.sqrt(dx_fy**2 + dy_fy**2)
v_fy1_x = v_d * dx_fy / norm_fy_xy
v_fy1_y = v_d * dy_fy / norm_fy_xy
v_fy1_z = 0.0
print(f"FY1 vel: ({v_fy1_x:.2f}, {v_fy1_y:.2f}, {v_fy1_z:.2f})")

def FY1_pos(t):
    return (FY1_0[0] + v_fy1_x * t,
            FY1_0[1] + v_fy1_y * t,
            FY1_0[2])

# ===== Smoke pre-detonation =====
release_pos = FY1_pos(t_d)
print(f"Release pos: ({release_pos[0]:.2f}, {release_pos[1]:.2f}, {release_pos[2]:.2f})")

def smoke_pre_pos(t):
    dt = t - t_d
    return (release_pos[0] + v_fy1_x * dt,
            release_pos[1] + v_fy1_y * dt,
            release_pos[2] - 0.5 * g * dt * dt)

det_pos = smoke_pre_pos(t_b)
print(f"Detonation pos: ({det_pos[0]:.2f}, {det_pos[1]:.2f}, {det_pos[2]:.2f})")

# ===== Smoke center after detonation =====
def smoke_center(t):
    dt = t - t_b
    return (det_pos[0], det_pos[1], det_pos[2] - v_c * dt)

# ===== Shielding check: center approximation =====
def check_shielding_center(t):
    P_M = M1_pos(t)
    P_C = smoke_center(t)
    
    dx = P_T_center[0] - P_M[0]
    dy = P_T_center[1] - P_M[1]
    dz = P_T_center[2] - P_M[2]
    L2 = dx*dx + dy*dy + dz*dz
    
    cx = P_C[0] - P_M[0]
    cy = P_C[1] - P_M[1]
    cz = P_C[2] - P_M[2]
    t_star = (cx*dx + cy*dy + cz*dz) / L2
    
    if t_star < 0 or t_star > 1:
        return False, float('inf')
    
    proj_x = P_M[0] + t_star * dx
    proj_y = P_M[1] + t_star * dy
    proj_z = P_M[2] + t_star * dz
    
    dist = math.sqrt((P_C[0]-proj_x)**2 + (P_C[1]-proj_y)**2 + (P_C[2]-proj_z)**2)
    return dist <= R_s, dist

# ===== Shielding check: cylinder boundary sampling =====
def check_shielding_boundary(t, N_circle=64, N_side=16):
    P_M = M1_pos(t)
    P_C = smoke_center(t)
    
    # Unit direction to smoke center
    dc_x = P_C[0] - P_M[0]
    dc_y = P_C[1] - P_M[1]
    dc_z = P_C[2] - P_M[2]
    dc_norm = math.sqrt(dc_x**2 + dc_y**2 + dc_z**2)
    
    # Angular radius of smoke sphere
    theta_s = math.asin(R_s / dc_norm)
    
    # Sample points on cylinder boundary
    # Top circle (z=10)
    for k in range(N_circle):
        angle = 2 * math.pi * k / N_circle
        px = r_T * math.cos(angle)
        py = 200 + r_T * math.sin(angle)
        pz = 10.0
        # Direction from M1 to this point
        vx = px - P_M[0]
        vy = py - P_M[1]
        vz = pz - P_M[2]
        vn = math.sqrt(vx**2 + vy**2 + vz**2)
        # Angular distance to smoke center
        cos_ang = (vx*dc_x + vy*dc_y + vz*dc_z) / (vn * dc_norm)
        cos_ang = max(-1.0, min(1.0, cos_ang))
        ang_dist = math.acos(cos_ang)
        if ang_dist > theta_s:
            return False
    
    # Bottom circle (z=0)
    for k in range(N_circle):
        angle = 2 * math.pi * k / N_circle
        px = r_T * math.cos(angle)
        py = 200 + r_T * math.sin(angle)
        pz = 0.0
        vx = px - P_M[0]
        vy = py - P_M[1]
        vz = pz - P_M[2]
        vn = math.sqrt(vx**2 + vy**2 + vz**2)
        cos_ang = (vx*dc_x + vy*dc_y + vz*dc_z) / (vn * dc_norm)
        cos_ang = max(-1.0, min(1.0, cos_ang))
        ang_dist = math.acos(cos_ang)
        if ang_dist > theta_s:
            return False
    
    # Side silhouette (tangent lines from M1 to cylinder)
    # Project M1 to yz-plane of cylinder axis
    PM_zx = math.sqrt(P_M[0]**2 + (P_M[1] - 200)**2)  # horizontal distance to cylinder axis
    if PM_zx > r_T:
        # Half-angle of tangent cone
        alpha = math.asin(r_T / PM_zx)
        # Tangent points span z=[0,10]
        for k in range(N_side):
            z_sample = 10.0 * k / (N_side - 1)
            # For each z, the tangent points form two lines
            for sign in [-1, 1]:
                # Direction from M1 to cylinder axis at height z
                ax_x = -P_M[0]
                ax_y = 200 - P_M[1]
                ax_norm = math.sqrt(ax_x**2 + ax_y**2)
                # Unit direction to axis
                ux = ax_x / ax_norm
                uy = ax_y / ax_norm
                # Tangent direction perpendicular
                tx = sign * (-uy)
                ty = sign * ux
                # Point on cylinder surface
                px = r_T * tx
                py = 200 + r_T * ty
                pz = z_sample
                vx = px - P_M[0]
                vy = py - P_M[1]
                vz = pz - P_M[2]
                vn = math.sqrt(vx**2 + vy**2 + vz**2)
                cos_ang = (vx*dc_x + vy*dc_y + vz*dc_z) / (vn * dc_norm)
                cos_ang = max(-1.0, min(1.0, cos_ang))
                ang_dist = math.acos(cos_ang)
                if ang_dist > theta_s:
                    return False
    
    return True

# ===== Scan through time =====
print("\n=== Method 1: Center approximation ===")
dt = 0.001
t_start = t_b
t_end = t_b + T_s

in_shield = False
shield_start = None
intervals = []

t = t_start
while t <= t_end:
    ok, _ = check_shielding_center(t)
    if ok and not in_shield:
        in_shield = True
        shield_start = t
    elif not ok and in_shield:
        in_shield = False
        intervals.append((shield_start, t - dt))
    t += dt
if in_shield:
    intervals.append((shield_start, t_end))

total1 = 0
for s, e in intervals:
    dur = e - s
    total1 += dur
    print(f"  [{s:.3f}s, {e:.3f}s]  dur = {dur:.3f}s")
print(f"  Total: {total1:.3f}s")

if total1 == 0:
    print("  No shielding! Debugging...")
    for tt in [5.1, 5.5, 6.0, 7.0, 8.0, 10.0, 15.0, 20.0, 25.0]:
        ok, d = check_shielding_center(tt)
        pm = M1_pos(tt)
        pc = smoke_center(tt)
        print(f"  t={tt:.1f}: M1=({pm[0]:.0f},{pm[1]:.0f},{pm[2]:.0f}) "
              f"Smoke=({pc[0]:.0f},{pc[1]:.0f},{pc[2]:.0f}) dist={d:.1f}m ok={ok}")

print("\n=== Method 2: Cylinder boundary sampling ===")
in_shield = False
shield_start = None
intervals2 = []

t = t_start
while t <= t_end:
    ok = check_shielding_boundary(t)
    if ok and not in_shield:
        in_shield = True
        shield_start = t
    elif not ok and in_shield:
        in_shield = False
        intervals2.append((shield_start, t - dt))
    t += dt
if in_shield:
    intervals2.append((shield_start, t_end))

total2 = 0
for s, e in intervals2:
    dur = e - s
    total2 += dur
    print(f"  [{s:.3f}s, {e:.3f}s]  dur = {dur:.3f}s")
print(f"  Total: {total2:.3f}s")

print(f"\n=== Result ===")
print(f"Method 1 (center approx): {total1:.3f} s")
print(f"Method 2 (boundary sampling): {total2:.3f} s")
if total2 > 0:
    print(f"\nFinal answer (method 2): {total2:.3f} s")
