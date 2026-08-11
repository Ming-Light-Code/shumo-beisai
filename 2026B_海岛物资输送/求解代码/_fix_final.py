import os, shutil
base = r"C:\Users\ming\Desktop\数模备赛\solution1_mpc"
for f in ["result_mpc.json","result_mpc.csv","output.log"]:
    fp = os.path.join(base, f)
    if os.path.exists(fp): os.remove(fp)
pc = os.path.join(base, "__pycache__")
if os.path.exists(pc): shutil.rmtree(pc, ignore_errors=True)

with open(os.path.join(base, "solver.py"), "r", encoding="utf-8") as f:
    code = f.read()

# Fix 1: O ratio 5:3:2 -> 6:2:2
code = code.replace("buy * 5 // 10, space, state.M // PRICE_O)", "buy * 6 // 10, space, state.M // PRICE_O)")
code = code.replace("buy * 3 // 10, space - o, state.M // PRICE_H)", "buy * 2 // 10, space - o, state.M // PRICE_H)")
print("Fix1: O ratio 6:2:2")

# Fix 2: Add O safety check (replace simple work check)
old_work = """        if wn and wd.get(wn, 0) < WORK_MAX_CONSECUTIVE[wn]:
            return "work", None
        else:
            path_idx[0] += 1"""

new_work = """        if wn and wd.get(wn, 0) < WORK_MAX_CONSECUTIVE[wn]:
            # O safety: enough to reach next supply/END after full work?
            remaining = WORK_MAX_CONSECUTIVE[wn] - wd.get(wn, 0)
            work_O = remaining * (_PN*5 + _PS*8)
            nxt = None
            for j in range(path_idx[0]+1, len(global_path)):
                pt = global_path[j]
                is_sp = any(pt == sp for sn, sp in SUPPLY.items())
                if is_sp or pt == END: nxt = pt; break
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
                return "work", None
        else:
            path_idx[0] += 1"""

if old_work in code:
    code = code.replace(old_work, new_work)
    print("Fix2: O safety check added")
else:
    print("Fix2: old pattern not found, searching...")
    idx = code.find("if wn and wd.get(wn, 0) < WORK_MAX_CONSECUTIVE")
    if idx >= 0:
        print("Found at", idx, ":", repr(code[idx:idx+80]))

# Fix 3: W2 revisit after S2 replenish
old_repl = """    if sn:
        repl = _forward_replenish_amount(state, target)
        if repl is not None:
            return "replenish", repl"""

new_repl = """    if sn:
        repl = _forward_replenish_amount(state, target)
        if repl is not None:
            return "replenish", repl
        # After replenish at S2, revisit W2 if skipped
        if (x,y) == (21,16) and wd.get("W2",0) == 0:
            for i, pt in enumerate(global_path):
                if pt == (15,9): path_idx[0] = i; break"""

if old_repl in code:
    code = code.replace(old_repl, new_repl)
    print("Fix3: W2 revisit after S2")
else:
    print("Fix3: replenish pattern not found")

with open(os.path.join(base, "solver.py"), "w", encoding="utf-8") as f:
    f.write(code)
print("All fixes applied")