with open(r"C:\Users\ming\Desktop\数模备赛\solution1_mpc\solver.py", "r", encoding="utf-8") as f:
    code = f.read()

# 1. Remove post-processing block (it was putting S2 after W2)
old_block = """    # Post-process: ensure S2 is BEFORE W2 for safe W2 work
    s2 = (21, 16); w2 = (15, 9)
    plist = [list(p) if isinstance(p, tuple) else list(p) for p in global_path]
    w2_idx = -1
    for i, p in enumerate(plist):
        if p[0] == 15 and p[1] == 9:
            w2_idx = i; break
    s2_present = any(p[0] == 21 and p[1] == 16 for p in plist)
    if not s2_present and w2_idx > 0:
        # Insert S2 BEFORE W2 so ship has O to work at W2
        prev = plist[w2_idx - 1]
        direct = abs(prev[0]-w2[0]) + abs(prev[1]-w2[1])
        via = abs(prev[0]-21) + abs(prev[1]-16) + abs(21-w2[0]) + abs(16-w2[1])
        if via <= direct + 15:
            plist.insert(w2_idx, [21, 16])
            global_path = [tuple(p) for p in plist]"""
code = code.replace(old_block, "")

# 2. Add S2-before-W2 insertion in plan_global_path return
old_return = "return best_path if best_path else [START, END]"
new_return = """    if best_path:
        pl = [list(p) for p in best_path]
        wi = -1; si = False
        for i, p in enumerate(pl):
            if p[0] == 15 and p[1] == 9: wi = i
            if p[0] == 21 and p[1] == 16: si = True
        if not si and wi > 0:
            pl.insert(wi, [21, 16])
            best_path = [tuple(p) for p in pl]
    return best_path if best_path else [START, END]"""
code = code.replace(old_return, new_return)

# 3. Remove O safety check (S2 before W2 provides sufficient O)
old_o = """        if wn and wd.get(wn, 0) < WORK_MAX_CONSECUTIVE[wn]:
            # O safety: enough to reach next supply/END after full work here?
            remaining = WORK_MAX_CONSECUTIVE[wn] - wd.get(wn, 0)
            work_O = remaining * (_PN*5 + _PS*8)
            nxt = None
            for j in range(path_idx[0]+1, len(global_path)):
                pt = global_path[j]
                is_sp = any(pt == sp for sn, sp in SUPPLY.items())
                if is_sp or pt == END:
                    nxt = pt; break
            if nxt:
                dist = _md(target, nxt)
                move_O = dist * (_PN*2 + _PS*8)
                if state.O < work_O + move_O + 10:
                    path_idx[0] += 1
                    if path_idx[0] >= len(global_path):
                        return "stay", None
                    target = global_path[path_idx[0]]
                else:
                    return "work", None
            else:
                return "work", None"""
new_o = """        if wn and wd.get(wn, 0) < WORK_MAX_CONSECUTIVE[wn]:
            return "work", None"""
if old_o in code:
    code = code.replace(old_o, new_o)
    print("O safety check removed")
else:
    print("O safety check not found - may already be simple")

with open(r"C:\Users\ming\Desktop\数模备赛\solution1_mpc\solver.py", "w", encoding="utf-8") as f:
    f.write(code)
print("All fixes applied: S2 before W2, O safety removed")