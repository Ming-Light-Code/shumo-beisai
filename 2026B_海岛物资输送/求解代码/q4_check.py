import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)

depth, yc, xc = q4.load_data()
NM2M = 1852.0; TAN60 = np.sqrt(3)

# Generate solutions
gl = q4.solve_greedy(depth, yc, xc, "EW")
dl = q4.solve_dp(depth, yc, xc, "EW", alpha=0.5)
sl = q4.solve_sa(depth, yc, xc, dl, "EW", iters=1500)

def verify_solution(name, lines, depth, yc, xc, resolutions=[100,200,400,600]):
    print(f"\n--- {name}: {len(lines)} lines ---")
    for nx in resolutions:
        a,b,c,d = q4.metrics(depth, yc, xc, lines, "EW", nx=nx)
        print(f"  nx={nx:3d}: leak={b:.2f}%, exceed={c:.3f}NM, len={d:.1f}NM")
    # Also compute average spacing
    if len(lines) > 1:
        sp = np.diff(np.sort(lines))
        print(f"  Spacing: min={np.min(sp)*NM2M:.0f}m, max={np.max(sp)*NM2M:.0f}m, mean={np.mean(sp)*NM2M:.0f}m")

verify_solution("Greedy", gl, depth, yc, xc)
verify_solution("DAG-DP", dl, depth, yc, xc)
verify_solution("SA(1500iters)", sl, depth, yc, xc)
