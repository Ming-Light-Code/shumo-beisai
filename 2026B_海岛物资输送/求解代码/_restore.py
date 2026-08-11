import os, shutil
base = r"C:\Users\ming\Desktop\数模备赛\solution1_mpc"
for f in ["result_mpc.json","result_mpc.csv","output.log"]:
    fp = os.path.join(base, f)
    if os.path.exists(fp): os.remove(fp)
pc = os.path.join(base, "__pycache__")
if os.path.exists(pc): shutil.rmtree(pc, ignore_errors=True)

with open(os.path.join(base, "solver.py"), "r", encoding="utf-8") as f:
    code = f.read()

# 1. Remove budget-balanced replenish, restore simple fill-to-85%+tol
old_budget = """def _forward_replenish_amount(state, next_target):
    \"\"\"Budget-balanced replenish: cap spending 50/25/25 on O/H/F\"\"\"
    current = state.O + state.H + state.F
    target_fill = int(LOAD_CAP * 0.85)
    if current >= target_fill - 10:
        return None
    space = LOAD_CAP - current
    buy = target_fill - current
    buy = min(buy, space)
    max_o = (state.M * 50 // 100) // PRICE_O
    max_h = (state.M * 25 // 100) // PRICE_H
    max_f = (state.M * 25 // 100) // PRICE_F
    o = min(buy * 5 // 10, space, max_o)
    h = min(buy * 3 // 10, space - o, max_h)
    f = min(buy * 2 // 10, space - o - h, max_f)
    return (max(0, o), max(0, h), max(0, f))"""

new_simple = """def _forward_replenish_amount(state, next_target):
    current = state.O + state.H + state.F
    target_fill = int(LOAD_CAP * 0.85)
    if current >= target_fill - 10:
        return None
    space = LOAD_CAP - current
    buy = min(target_fill - current, space)
    o = min(buy * 5 // 10, space, state.M // PRICE_O)
    h = min(buy * 3 // 10, space - o, state.M // PRICE_H)
    f = min(buy * 2 // 10, space - o - h, state.M // PRICE_F)
    return (max(0, o), max(0, h), max(0, f))"""

if old_budget in code:
    code = code.replace(old_budget, new_simple)
    print("1. Restored simple fill-to-85%")

# 2. Ensure O safety uses work_O + move_O*2.0 + 5 (aggressive skip W2)
old_margin = "state.O < work_O + move_O + 10:"
new_margin = "state.O < work_O + move_O * 2.0 + 5:"
if old_margin in code:
    code = code.replace(old_margin, new_margin)
    print("2. O safety threshold: 2x move_O + 5")
elif "state.O < work_O + move_O * 1.5 + 30:" in code:
    code = code.replace("state.O < work_O + move_O * 1.5 + 30:", new_margin)
    print("2. O safety restored from 1.5x to 2.0x")

# 3. Remove W2 revisit (it causes issues)
old_w2 = """        # After replenish at S2, revisit W2 if skipped
        if (x,y) == (21,16) and wd.get(\"W2\",0) == 0:
            for i, pt in enumerate(global_path):
                if pt == (15,9): path_idx[0] = i; break"""
if old_w2 in code:
    code = code.replace(old_w2, "")
    print("3. Removed W2 revisit")

with open(os.path.join(base, "solver.py"), "w", encoding="utf-8") as f:
    f.write(code)
print("All fixes applied")