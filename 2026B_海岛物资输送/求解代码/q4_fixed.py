import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)
depth, yc, xc = q4.load_data()
NM2M=1852.0; TAN60=np.sqrt(3)

dl = q4.solve_dp(depth, yc, xc, "EW", alpha=0.5)
gl = q4.solve_greedy(depth, yc, xc, "EW")

# Custom SA version that uses nx=200 in its internal cost evaluation
def sa_highres(init_lines, nx_cost=200, iters=1500):
    def cost_fn(ls):
        if len(ls) < 2: return 1e10
        _, leak, exc, length = q4.metrics(depth, yc, xc, ls, "EW", nx=nx_cost)
        return leak * 500 + exc * 5 + length
    
    lines = np.array(sorted(init_lines))
    best_l = lines.copy(); best_c = cost_fn(best_l)
    cur_c = best_c
    T0=0.5; Te=0.001; cr=(Te/T0)**(1.0/iters); T=T0
    np.random.seed(42)
    
    for _ in range(iters):
        nl = lines.copy()
        op = np.random.randint(0, 3)
        if op == 0 and len(nl)>1:
            idx = np.random.randint(0, len(nl))
            nl[idx] += np.random.normal(0, 0.015)
            nl = np.clip(nl, 0.01, 4.99); nl = np.sort(nl)
        elif op == 1 and len(nl)>1:
            nl = np.delete(nl, np.random.randint(0, len(nl)))
        elif op == 2:
            nl = np.sort(np.append(nl, np.random.uniform(0.01, 4.99)))
        nc = cost_fn(nl); delta = nc - cur_c
        if delta < 0 or np.random.random() < np.exp(-delta/max(T,1e-8)):
            lines=nl; cur_c=nc
            if cur_c < best_c: best_c=cur_c; best_l=lines.copy()
        T *= cr
    return best_l

print("Running SA with nx=200 internal cost resolution...")
sl_hr = sa_highres(dl, nx_cost=200, iters=1500)
print(f"SA(highres): {len(sl_hr)} lines")

# Verify at multiple resolutions
print()
print("VERIFICATION (corrected SA):")
for name, ls in [("Greedy", gl), ("DP", dl), ("SA(orig)", q4.solve_sa(depth,yc,xc,dl,"EW",1500)), ("SA(highres)", sl_hr)]:
    a300,b300,c300,d300 = q4.metrics(depth, yc, xc, ls, "EW", nx=300)
    print(f"  {name:12s}: {len(ls):3d} lines, leak={b300:.2f}%, exceed={c300:.3f}NM, len={d300:.0f}NM")

# Detailed gap check on SA(highres)
print()
print("Gap check on SA(highres) at nx=600:")
nx600=600; x_ev = np.linspace(0,4.0,nx600); dx600=4.0/nx600
total_gap_area = 0.0; max_gap_val = 0.0
for xi in range(nx600):
    x = x_ev[xi]
    intervals = []
    for yk in sl_hr:
        d = q4.bilinear(depth, yc, xc, float(yk), float(x))
        w = d * TAN60 / NM2M
        intervals.append((float(yk)-w, float(yk)+w))
    intervals.sort()
    # Merge and track gaps properly
    merged = [intervals[0]]
    for lo, hi in intervals[1:]:
        if lo <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
        else:
            merged.append((lo, hi))
    # Check gaps: south boundary
    if merged[0][0] > 0:
        total_gap_area += merged[0][0] * dx600
        max_gap_val = max(max_gap_val, merged[0][0])
    # Gaps between merged intervals
    for k in range(len(merged)-1):
        gap = merged[k+1][0] - merged[k][1]
        if gap > 0:
            total_gap_area += gap * dx600
            max_gap_val = max(max_gap_val, gap)
    # North boundary
    if merged[-1][1] < 5.0:
        total_gap_area += (5.0 - merged[-1][1]) * dx600
        max_gap_val = max(max_gap_val, 5.0 - merged[-1][1])

print(f"  Max single gap: {max_gap_val*NM2M:.1f}m")
print(f"  Total gap area: {total_gap_area:.4f} NM^2 = {100*total_gap_area/20:.4f}%")
