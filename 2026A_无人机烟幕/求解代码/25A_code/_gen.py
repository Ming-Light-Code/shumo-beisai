import pathlib
dest = pathlib.Path(r"C:\Users\ming\Desktop\数模备赛\25A_code\q3_nlp.py")
code = dest.read_text("utf-8")

# 1. Revert eval_8var back to fast union
code = code.replace("return quick_union_vec(alpha, v, t)", "return quick_union_fast(alpha, v, t)")

# 2. Replace the solve loop: after grid search, do thorough full-union CD on best
old_post = """    # Sort
    results.sort(key=lambda r: -r["occ_opt"])
    print(f"  Searched {len(results)} (alpha, v) pairs ({time.time()-t_start:.1f}s)")"""

new_post = """    # Sort by fast-union score, then pick best for full-union polish
    results.sort(key=lambda r: -r["occ_opt"])
    print(f"  Searched {len(results)} (alpha, v) pairs ({time.time()-t_start:.1f}s)")

    # Thorough full-union CD on top-3 only (for solution quality)
    print("  [1b] Full-union CD polish on top-3 results...")
    t_polish = time.time()
    for r in results[:3]:
        x0 = r["x"]
        # Full CD with thorough step sizes
        xb = np.array(x0, dtype=float)
        fb = quick_union_vec(xb[0], xb[1], [(xb[2],xb[3]),(xb[4],xb[5]),(xb[6],xb[7])])
        for step in [0.1, 0.05, 0.02, 0.01, 0.005, 0.002]:
            improved = True
            while improved:
                improved = False
                for d in range(8):
                    for sgn in [-1, 1]:
                        xt = xb.copy(); xt[d] += sgn * step
                        xt[0] %= (2.0 * np.pi)
                        xt[1] = np.clip(xt[1], V_DRONE_MIN, V_DRONE_MAX)
                        xt[2] = max(0.0, xt[2])
                        xt[3] = max(xt[2] + 0.01, min(xt[3], T_MAX - 0.01))
                        xt[4] = max(xt[2] + MIN_INTERVAL, xt[4])
                        xt[5] = max(xt[4] + 0.01, min(xt[5], T_MAX - 0.01))
                        xt[6] = max(xt[4] + MIN_INTERVAL, xt[6])
                        xt[7] = max(xt[6] + 0.01, min(xt[7], T_MAX - 0.01))
                        for k in range(3):
                            dti = xt[3+2*k] - xt[2+2*k]
                            if dti > MAX_FALL: xt[3+2*k] = xt[2+2*k] + MAX_FALL
                        ft = quick_union_vec(xt[0], xt[1], [(xt[2],xt[3]),(xt[4],xt[5]),(xt[6],xt[7])])
                        if ft > fb + 1e-8: xb = xt; fb = ft; improved = True
        r["x"] = xb
        r["occ_opt"] = fb
    print(f"  Polish done ({time.time()-t_polish:.1f}s)")
    results.sort(key=lambda r: -r["occ_opt"])"""

code = code.replace(old_post, new_post)

dest.write_text(code, "utf-8")
print("Hybrid approach: fast for grid, full-union CD for top-3 polish")