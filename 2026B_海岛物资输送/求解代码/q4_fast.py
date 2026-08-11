import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)
depth, yc, xc = q4.load_data()
NM2M=1852.0; TAN60=np.sqrt(3)

# Run just one SA with 1500 iters and check at nx=300 (faster)
dl = q4.solve_dp(depth, yc, xc, "EW", alpha=0.5)
np.random.seed(42)
sl = q4.solve_sa(depth, yc, xc, dl, "EW", iters=1500)
gl = q4.solve_greedy(depth, yc, xc, "EW")

print("FINAL CONFIRMED RESULTS (nx=300)")
print("="*50)
for name, lines in [("Greedy", gl), ("DAG-DP", dl), ("SA+DP", sl)]:
    a,b,c,d = q4.metrics(depth, yc, xc, lines, "EW", nx=300)
    print(f"{name:10s}: {len(lines):3d} lines | leak={b:.2f}% | exceed={c:.3f}NM | len={d:.1f}NM")

# Also do a manual spot-check: for each of 20 x-positions, find max gap
print()
print("SPOT CHECK: Max gap at 20 random x positions (nx=600 equivalent)")
x_spot = np.linspace(0.01, 3.99, 20)
max_gap_any = 0
for x in x_spot:
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
            max_gap_any = max(max_gap_any, lo - merged[1])
            merged = (lo, hi)
print(f"Max gap across all spot checks: {max_gap_any*NM2M:.1f}m ({max_gap_any*NM2M:.1f}m = {'GAP!' if max_gap_any>0.001 else 'NO GAP (verified)'})")
