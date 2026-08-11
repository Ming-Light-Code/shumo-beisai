import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)

depth, yc, xc = q4.load_data()
NM2M = 1852.0
TAN60 = np.sqrt(3)

print("="*60)
print("VERIFICATION OF GOLD TIER (SA) RESULTS")
print("="*60)

# ---- Re-generate SA solution ----
gl = q4.solve_greedy(depth, yc, xc, "EW")
dl = q4.solve_dp(depth, yc, xc, "EW", alpha=0.5)
init = dl if len(dl) > 0 else gl
print(f"SA starting from {len(init)} lines (DP)...")

# SA with fixed seed for reproducibility, more iterations
sl = q4.solve_sa(depth, yc, xc, init, "EW", iters=2000)
lines = np.sort(sl)

print(f"SA result: {len(lines)} lines")
print(f"Line positions (first 5): {[round(x,4) for x in lines[:5]]}")
print(f"Line positions (last 5):  {[round(x,4) for x in lines[-5:]]}")
print(f"Min spacing: {np.min(np.diff(lines)):.4f} NM = {np.min(np.diff(lines))*NM2M:.1f}m")
print(f"Max spacing: {np.max(np.diff(lines)):.4f} NM = {np.max(np.diff(lines))*NM2M:.1f}m")
print(f"Mean spacing: {np.mean(np.diff(lines)):.4f} NM = {np.mean(np.diff(lines))*NM2M:.1f}m")
print()

# ---- VERIFICATION 1: Gap check at HIGH resolution ----
print("VERIFICATION 1: Gap detection at nx=600 resolution")
nx = 600
x_eval = np.linspace(0, 4.0, nx)
dx = 4.0 / nx

max_gap = 0.0
gap_positions = []
total_gap_area = 0.0

for xi in range(nx):
    x = x_eval[xi]
    intervals = []
    for yk in lines:
        d = q4.bilinear(depth, yc, xc, float(yk), float(x))
        w = d * TAN60 / NM2M
        if w > 0:
            intervals.append((float(yk) - w, float(yk) + w))
    intervals.sort(key=lambda v: v[0])
    
    # Merge intervals
    merged_start, merged_end = intervals[0]
    for lo, hi in intervals[1:]:
        if lo <= merged_end:
            merged_end = max(merged_end, hi)
        else:
            # GAP between merged_end and lo
            gap = lo - merged_end
            if gap > max_gap:
                max_gap = gap
            if gap > 0:
                total_gap_area += gap * dx
                if len(gap_positions) < 5:
                    gap_positions.append((float(x), merged_end, lo, gap))
            merged_start, merged_end = lo, hi
    
    # Check gaps at boundaries
    if merged_start > 0:
        gap = merged_start - 0
        max_gap = max(max_gap, gap)
        total_gap_area += gap * dx
    
    if merged_end < 5.0:
        gap = 5.0 - merged_end
        max_gap = max(max_gap, gap)
        total_gap_area += gap * dx

total_area = 20.0  # NM^2
gap_pct = 100.0 * total_gap_area / total_area

print(f"  Max single gap: {max_gap*NM2M:.2f} m = {max_gap:.6f} NM")
print(f"  Total gap area: {total_gap_area:.4f} NM^2 = {gap_pct:.4f}%")
if gap_positions:
    print(f"  First 5 gap locations:")
    for x, lo, hi, g in gap_positions:
        print(f"    at x={x:.3f} NM: gap from y={lo:.4f} to y={hi:.4f} ({g*NM2M:.1f}m)")

# ---- VERIFICATION 2: Overlap exceed check ----
print()
print("VERIFICATION 2: Overlap exceed check at nx=600")
total_exceed = 0.0
max_overlap_rate = 0.0
m = len(lines)

for k in range(m - 1):
    yk = float(lines[k])
    yj = float(lines[k + 1])
    for xi in range(nx):
        x = x_eval[xi]
        dk = q4.bilinear(depth, yc, xc, yk, x)
        dj = q4.bilinear(depth, yc, xc, yj, x)
        wk = dk * TAN60 / NM2M
        wj = dj * TAN60 / NM2M
        if wk <= 0 or wj <= 0:
            continue
        ov = (yk + wk) - (yj - wj)
        if ov > 0:
            tw = wk + wj
            rate = ov / tw
            max_overlap_rate = max(max_overlap_rate, rate)
            if rate > 0.20:
                total_exceed += (ov - 0.20 * tw) * dx

print(f"  Max overlap rate: {max_overlap_rate*100:.1f}%")
print(f"  Total exceed 20%: {total_exceed:.4f} NM = {total_exceed*NM2M:.1f} m")

# ---- VERIFICATION 3: Check at critical shallow points ----
print()
print("VERIFICATION 3: Coverage at shallowest depth locations")
# Find the 5 shallowest points in the dataset
flat_depths = depth.flatten()
shallow_indices = np.argsort(flat_depths)[:5]
for idx in shallow_indices:
    yi, xi_grid = np.unravel_index(idx, depth.shape)
    y_pos = yi * 0.02
    x_pos = xi_grid * 0.02
    d = depth[yi, xi_grid]
    w = d * TAN60 / NM2M
    
    # Check if any line covers this point
    covered = False
    for yk in lines:
        d_line = q4.bilinear(depth, yc, xc, float(yk), float(x_pos))
        w_line = d_line * TAN60 / NM2M
        if abs(y_pos - yk) <= w_line:
            covered = True
            covering_line_y = yk
            break
    
    status = "COVERED" if covered else "GAP!"
    print(f"  ({x_pos:.2f},{y_pos:.2f}) NM: depth={d:.1f}m, strip_hw={w*NM2M:.1f}m | {status}")

# ---- SUMMARY ----
print()
print("="*60)
print("VERIFICATION SUMMARY")
print(f"  Lines:       {len(lines)}")
print(f"  Total length: {len(lines)*4.0:.2f} NM")
print(f"  Max gap:      {max_gap*NM2M:.2f} m ({max_gap:.6f} NM)")
print(f"  Gap area:     {gap_pct:.4f}%")
print(f"  Exceed 20%:   {total_exceed:.4f} NM")
print(f"  Max overlap:  {max_overlap_rate*100:.1f}%")
print("="*60)
