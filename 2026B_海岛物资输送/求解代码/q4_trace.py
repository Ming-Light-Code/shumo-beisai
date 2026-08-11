import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)
depth, yc, xc = q4.load_data()
NM2M=1852.0; TAN60=np.sqrt(3)

dl = q4.solve_dp(depth, yc, xc, "EW", alpha=0.5)
np.random.seed(42)
sl = q4.solve_sa(depth, yc, xc, dl, "EW", iters=1500)

# Find EXACT location of the max gap
print("TRACING MAX GAP LOCATION (nx=1200 fine scan)")
x_fine = np.linspace(0, 4.0, 1200)
worst_x = 0; worst_gap = 0; worst_lo = 0; worst_hi = 0
for x in x_fine:
    intervals = []
    for yk in sl:
        d = q4.bilinear(depth, yc, xc, float(yk), float(x))
        w = d * TAN60 / NM2M
        intervals.append((float(yk)-w, float(yk)+w))
    intervals.sort()
    merged = intervals[0]
    for lo, hi in intervals[1:]:
        if lo <= merged[1]:
            merged = (merged[0], max(merged[1], hi))
        else:
            gap = lo - merged[1]
            if gap > worst_gap:
                worst_gap = gap; worst_x = x; worst_lo = merged[1]; worst_hi = lo
            merged = (lo, hi)
    # Check boundary gaps too
    if merged[0] > 0 and merged[0] > worst_gap:
        worst_gap = merged[0]; worst_x = x; worst_lo = 0; worst_hi = merged[0]
    if merged[1] < 5.0 and (5.0 - merged[1]) > worst_gap:
        worst_gap = 5.0 - merged[1]; worst_x = x; worst_lo = merged[1]; worst_hi = 5.0

print(f"Worst gap: {worst_gap*NM2M:.1f}m at x={worst_x:.3f}NM")
print(f"  Gap spans y={worst_lo:.5f} to y={worst_hi:.5f} NM")

# What depth is at this x and y?
dg = q4.bilinear(depth, yc, xc, (worst_lo+worst_hi)/2, worst_x)
print(f"  Depth at gap center: {dg:.1f}m")

# Which SA lines are adjacent to the gap?
print(f"  SA lines near gap:")
for i, yk in enumerate(sl):
    if abs(float(yk) - worst_lo) < 0.3 or abs(float(yk) - worst_hi) < 0.3:
        d = q4.bilinear(depth, yc, xc, float(yk), worst_x)
        w = d * TAN60 / NM2M
        print(f"    Line {i}: y={float(yk):.4f}, depth={d:.1f}m, half_w={w*NM2M:.1f}m, covers [{float(yk)-w:.4f}, {float(yk)+w:.4f}]")

# Calculate total gap area precisely
total_gap = 0.0
for xi, x in enumerate(x_fine):
    intervals = []
    for yk in sl:
        d = q4.bilinear(depth, yc, xc, float(yk), float(x))
        w = d * TAN60 / NM2M
        intervals.append((float(yk)-w, float(yk)+w))
    intervals.sort()
    merged = intervals[0]
    for lo, hi in intervals[1:]:
        if lo <= merged[1]: merged = (merged[0], max(merged[1], hi))
        else:
            total_gap += (lo - merged[1]) * (4.0/1200)
            merged = (lo, hi)
    if merged[0] > 0: total_gap += merged[0] * (4.0/1200)
    if merged[1] < 5.0: total_gap += (5.0 - merged[1]) * (4.0/1200)

print(f"\nTotal gap area (nx=1200): {total_gap:.6f} NM^2 = {100*total_gap/20:.4f}% of sea area")
print(f"Equivalent gap: {total_gap*NM2M**2:.0f} m^2")
