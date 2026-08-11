import sys, importlib.util, numpy as np
spec = importlib.util.spec_from_file_location("q4_all", r"C:\Users\ming\Desktop\数模备赛\q4_all.py")
q4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q4)

depth, yc, xc = q4.load_data()
print(f"Data: {depth.shape}, depth {np.min(depth):.0f}-{np.max(depth):.0f}m")

results = {}
for direction in ["EW", "NS"]:
    print(f"\n=== DIRECTION: {direction} ===")
    results[direction] = {}

    gl = q4.solve_greedy(depth, yc, xc, direction)
    a,b,c,d = q4.metrics(depth, yc, xc, gl, direction, nx=100)
    print(f"Greedy: {len(gl)} lines | leak {b:.2f}% | exceed {c:.4f}NM | len {d:.2f}NM")
    results[direction]["greedy"] = {"n":len(gl),"leak":b,"exceed":c,"len":d}

    dl = q4.solve_dp(depth, yc, xc, direction, alpha=0.5)
    a,b,c,d = q4.metrics(depth, yc, xc, dl, direction, nx=100)
    print(f"DAG-DP: {len(dl)} lines | leak {b:.2f}% | exceed {c:.4f}NM | len {d:.2f}NM")
    results[direction]["dp"] = {"n":len(dl),"leak":b,"exceed":c,"len":d}

    init_val = dl if len(dl) > 0 else gl
    sl = q4.solve_sa(depth, yc, xc, init_val, direction, iters=1000)
    a,b,c,d = q4.metrics(depth, yc, xc, sl, direction, nx=100)
    print(f"SA: {len(sl)} lines | leak {b:.2f}% | exceed {c:.4f}NM | len {d:.2f}NM")
    results[direction]["sa"] = {"n":len(sl),"leak":b,"exceed":c,"len":d}

    ml = q4.solve_multi_dp(depth, yc, xc, direction)
    if ml:
        bml = min(ml.values(), key=lambda x: x["leak"])
        print(f"Multi-DP best: {bml['n']} lines | leak {bml['leak']:.2f}%")
        results[direction]["multi"] = bml

    nr = q4.solve_nsga2(depth, yc, xc, direction, pop=25, gens=40)
    if nr:
        bnr = min(nr, key=lambda x: x["leak"])
        print(f"NSGA-II best: {bnr['n']} lines | leak {bnr['leak']:.2f}%")
        results[direction]["nsga"] = bnr

print("\n" + "="*55)
print("FINAL COMPARISON")
print("="*55)
for d in ["EW", "NS"]:
    print(f"\nDirection: {d}")
    print(f"  {'Method':<12} {'Lines':>6} {'Leak%':>10} {'Exceed':>12} {'Length':>10}")
    print(f"  {'-'*12} {'-'*6} {'-'*10} {'-'*12} {'-'*10}")
    for k, nm in [("greedy","Greedy"),("dp","DAG-DP"),("sa","SA+DP"),("nsga","NSGA-II")]:
        if k in results[d]:
            r = results[d][k]
            print(f"  {nm:<12} {r['n']:>6} {r['leak']:>9.2f}% {r['exceed']:>11.4f} {r['len']:>9.2f}")
