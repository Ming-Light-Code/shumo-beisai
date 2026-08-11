import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)

depth, yc, xc = q4.load_data()

# Refine SA results with higher resolution coverage check
print("=== REFINED METRICS (nx=200, higher accuracy) ===\n")

methods = {}
for direction in ["EW", "NS"]:
    print(f"--- {direction} ---")
    
    # Greedy
    gl = q4.solve_greedy(depth, yc, xc, direction)
    a,b,c,d = q4.metrics(depth, yc, xc, gl, direction, nx=200)
    
    # DP
    dl = q4.solve_dp(depth, yc, xc, direction, alpha=0.5)
    a2,b2,c2,d2 = q4.metrics(depth, yc, xc, dl, direction, nx=200)
    
    # SA
    init = dl if len(dl) > 0 else gl
    sl = q4.solve_sa(depth, yc, xc, init, direction, iters=2000)
    a3,b3,c3,d3 = q4.metrics(depth, yc, xc, sl, direction, nx=200)
    
    methods[direction] = {
        "greedy": (len(gl), b, c, d, gl),
        "dp": (len(dl), b2, c2, d2, dl),
        "sa": (len(sl), b3, c3, d3, sl),
    }
    
    print(f"  Greedy: {len(gl):3d} lines, leak={b:.2f}%, exceed={c:.4f}NM, len={d:.2f}NM")
    print(f"  DP:     {len(dl):3d} lines, leak={b2:.2f}%, exceed={c2:.4f}NM, len={d2:.2f}NM")
    print(f"  SA:     {len(sl):3d} lines, leak={b3:.2f}%, exceed={c3:.4f}NM, len={d3:.2f}NM")
    print()

# NSGA-II with better parameters
print("=== NSGA-II (pop=50, gens=100) ===\n")
nsga_results = {}
for direction in ["EW", "NS"]:
    print(f"--- {direction} ---")
    nr = q4.solve_nsga2(depth, yc, xc, direction, pop=50, gens=100)
    if not nr:
        print("  No results")
        continue
    
    # Sort by leakage (ascending)
    nr_sorted = sorted(nr, key=lambda x: x["leak"])
    
    # Show top 5 by different criteria
    best_leak = nr_sorted[0]
    best_len = min(nr, key=lambda x: x["len"])
    best_exceed = min(nr, key=lambda x: x["exceed"])
    
    print(f"  Best (min leak):  {best_leak['n']:3d} lines, leak={best_leak['leak']:.2f}%, exceed={best_leak['exceed']:.4f}NM, len={best_leak['len']:.2f}NM")
    print(f"  Best (min len):   {best_len['n']:3d} lines, leak={best_len['leak']:.2f}%, exceed={best_len['exceed']:.4f}NM, len={best_len['len']:.2f}NM")
    print(f"  Best (min exceed):{best_exceed['n']:3d} lines, leak={best_exceed['leak']:.2f}%, exceed={best_exceed['exceed']:.4f}NM, len={best_exceed['len']:.2f}NM")
    
    nsga_results[direction] = {
        "best_leak": best_leak,
        "best_len": best_len,
        "best_exceed": best_exceed
    }
    
    # Show Pareto front (non-dominated)
    objs = np.array([(r["leak"], r["exceed"], r["len"]) for r in nr])
    n = len(nr)
    dominated = np.zeros(n, dtype=bool)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            if (objs[j][0] <= objs[i][0] and objs[j][1] <= objs[i][1] and objs[j][2] <= objs[i][2]) and \
               (objs[j][0] < objs[i][0] or objs[j][1] < objs[i][1] or objs[j][2] < objs[i][2]):
                dominated[i] = True
                break
    
    pareto = [nr[i] for i in range(n) if not dominated[i]]
    print(f"  Pareto front size: {len(pareto)} solutions")
    print()

# FINAL SUMMARY
print("="*60)
print("FINAL SUMMARY - ALL METHODS")
print("="*60)
print()
for d in ["EW", "NS"]:
    print(f"Direction: {d} ({'East-West lines' if d=='EW' else 'North-South lines'})")
    print(f"  Line length: {4 if d=='EW' else 5} NM each")
    print(f"  Spaced along: {'Y (5NM range)' if d=='EW' else 'X (4NM range)'}")
    print()
    print(f"  {'Method':<12} {'Lines':>6} {'Leak%':>8} {'Exceed(NM)':>12} {'Len(NM)':>10}")
    print(f"  {'-'*12} {'-'*6} {'-'*8} {'-'*12} {'-'*10}")
    
    for k, nm in [("greedy","Greedy"),("dp","DAG-DP"),("sa","SA+DP")]:
        n, leak, exc, ln, _ = methods[d][k]
        print(f"  {nm:<12} {n:>6} {leak:>7.2f}% {exc:>11.4f} {ln:>9.2f}")
    
    if d in nsga_results:
        for label, key in [("NSGA (minL)","best_leak"), ("NSGA (minX)","best_exceed")]:
            r = nsga_results[d][key]
            print(f"  {label:<12} {r['n']:>6} {r['leak']:>7.2f}% {r['exceed']:>11.4f} {r['len']:>9.2f}")
    print()
