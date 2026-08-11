with open(r"C:\Users\ming\Desktop\数模备赛\solution1_mpc\solver.py", "r", encoding="utf-8") as f:
    code = f.read()
# Replace _forward_replenish_amount with budget-balanced version
old = """def _forward_replenish_amount(state, next_target):
    \"\"\"Simple fill-to-capacity replenish strategy\"\"\"
    current = state.O + state.H + state.F
    target_fill = int(LOAD_CAP * 0.85)  # Fill to 75% capacity
    if current >= target_fill - 10:
        return None
    space = LOAD_CAP - current
    buy = target_fill - current
    buy = min(buy, space)
    o = min(buy * 5 // 10, space, state.M // PRICE_O)
    h = min(buy * 3 // 10, space - o, state.M // PRICE_H)
    f = min(buy * 2 // 10, space - o - h, state.M // PRICE_F)
    return (max(0, o), max(0, h), max(0, f))"""
new = """def _forward_replenish_amount(state, next_target):
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
if old in code:
    code = code.replace(old, new)
    print("Budget-balanced replenish applied")
else:
    print("NOT FOUND - checking alternative")
    if "Simple fill-to-capacity" in code:
        print("Found old comment, replacing differently")
        idx = code.find("Simple fill-to-capacity")
        end = code.find("return (max(0, o), max(0, h), max(0, f))", idx) + 40
        code = code[:idx-3] + new.split('"""')[2] + code[end:]
        print("Alternative replace done")
with open(r"C:\Users\ming\Desktop\数模备赛\solution1_mpc\solver.py", "w", encoding="utf-8") as f:
    f.write(code)