import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)

depth, yc, xc = q4.load_data()
NM2M = 1852.0; TAN60 = np.sqrt(3)
np.random.seed(99)
dl = q4.solve_dp(depth, yc, xc, "EW", alpha=0.5)

# Run SA 3 times with 1500 iters each, different seeds
print("CONSISTENCY CHECK: 3 SA runs (1500 iters, different seeds)")
print("="*55)
for run in range(3):
    np.random.seed(run * 100 + 42)
    sl = q4.solve_sa(depth, yc, xc, dl, "EW", iters=1500)
    a,b,c,d = q4.metrics(depth, yc, xc, sl, "EW", nx=600)
    print(f"Run {run+1}: {len(sl)} lines | leak={b:.2f}% | exceed={c:.3f}NM | len={d:.1f}NM")
print()

# Now ALSO test: is the SA result truly better? Check Greedy at nx=600 
gl = q4.solve_greedy(depth, yc, xc, "EW")
print("FINAL CONFIRMED RESULTS (nx=600)")
print("="*55)
for name, lines in [("Greedy", gl), ("DAG-DP", dl), ("SA+DP", sl)]:
    a,b,c,d = q4.metrics(depth, yc, xc, lines, "EW", nx=600)
    print(f"{name:10s}: {len(lines):3d} lines | leak={b:.2f}% | exceed={c:.3f}NM ({c*NM2M:.0f}m) | len={d:.1f}NM")
