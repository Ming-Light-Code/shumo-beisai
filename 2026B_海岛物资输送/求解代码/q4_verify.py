import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)
depth, yc, xc = q4.load_data()

# EW direction - Greedy
gl = q4.solve_greedy(depth, yc, xc, 'EW')
print(f"EW Greedy lines (first 10): {gl[:10]}")
print(f"EW Greedy lines (last 10): {gl[-10:]}")
print(f"Total: {len(gl)} lines")

# Check at nx=300 for high-res verification
for method_name, lines in [("Greedy", gl)]:
    for nx in [100, 200, 300]:
        a,b,c,d = q4.metrics(depth, yc, xc, lines, 'EW', nx=nx)
        print(f"EW {method_name} nx={nx}: leak={b:.2f}%, exceed={c:.4f}NM, len={d:.2f}NM")

# EW - DP
dl = q4.solve_dp(depth, yc, xc, 'EW', alpha=0.5)
print(f"\nEW DP lines: {len(dl)}")
print(f"First 10: {dl[:10]}")
print(f"Last 10: {dl[-10:]}")
for nx in [100, 200, 300]:
    a,b,c,d = q4.metrics(depth, yc, xc, dl, 'EW', nx=nx)
    print(f"EW DP nx={nx}: leak={b:.2f}%, exceed={c:.4f}NM, len={d:.2f}NM")

# EW - SA
sl = q4.solve_sa(depth, yc, xc, dl, 'EW', iters=2000)
print(f"\nEW SA lines: {len(sl)}")
print(f"First 10: {sl[:10]}")
print(f"Last 10: {sl[-10:]}")
for nx in [100, 200, 300]:
    a,b,c,d = q4.metrics(depth, yc, xc, sl, 'EW', nx=nx)
    print(f"EW SA nx={nx}: leak={b:.2f}%, exceed={c:.4f}NM, len={d:.2f}NM")
