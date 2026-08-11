"""Solution 3: Two-Stage Stochastic Programming with Sample Average Approximation"""
import sys, os, csv, json, random
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from environment import (Environment, ShipState, GRID_SIZE, START, END,
                         SUPPLY, WORK, WORK_YIELD, WORK_MAX_CONSECUTIVE,
                         LOAD_CAP, TIME_LIMIT, PRICE_O, PRICE_H, PRICE_F,
                         P_NORMAL, P_STORM, CONSUME)

MOVE_DIRS = {"move_up":(-1,0),"move_down":(1,0),"move_left":(0,-1),"move_right":(0,1)}

def _md(p1,p2): return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def _generate_weather_scenarios(num_scenarios=20):
    """Generate independent weather scenarios for 90 days each"""
    scenarios = []
    for _ in range(num_scenarios):
        scenario = []
        for __ in range(TIME_LIMIT):
            scenario.append("normal" if random.random()<P_NORMAL else "storm")
        scenarios.append(scenario)
    return scenarios

def _solve_deterministic(env, weather_seq):
    """Solve a single deterministic scenario using greedy heuristic"""
    state = env.reset()
    plan = []
    while not state.terminated and state.day <= TIME_LIMIT:
        if (state.x, state.y) == END:
            break
        weather = weather_seq[min(state.day-1, len(weather_seq)-1)]
        # Greedy action
        x, y = state.x, state.y
        wd = state.work_days
        action = "stay"

        # Work if at work point and can
        for wn, wp in WORK.items():
            if (x, y) == wp and wd.get(wn, 0) < WORK_MAX_CONSECUTIVE[wn]:
                action = "work"
                break

        # Replenish if at supply point and resources are low
        if action == "stay":
            sn = env.get_supply_name(x, y)
            if sn and state.load < LOAD_CAP * 0.4:
                action = "replenish"

        # Move towards next target
        if action == "stay":
            targets = []
            for wn, wp in WORK.items():
                if wd.get(wn, 0) >= WORK_MAX_CONSECUTIVE[wn]:
                    continue
                d = _md((x, y), wp)
                targets.append((d, wp))
            targets.append((_md((x, y), END), END))
            targets.sort()
            if targets:
                tx, ty = targets[0][1]
                if tx > x: action = "move_down"
                elif tx < x: action = "move_up"
                elif ty > y: action = "move_right"
                elif ty < y: action = "move_left"

        repl = None
        if action == "replenish":
            sp = LOAD_CAP - state.load
            nd = max(0, 30*7 - state.load)
            am = min(nd, sp)
            o = min(am*2//7, sp, state.M//2)
            h = min(am*3//7, sp-o, state.M)
            f = min(am*2//7, sp-o-h, state.M//2)
            repl = (max(0, o), max(0, h), max(0, f))

        plan.append((action, repl))
        state, reward, done = env.step(state, action, weather, repl)
        if done:
            break

    success = state.terminated and (state.x, state.y) == END
    return plan, success, state.Z, state.M

def two_stage_policy(scenarios):
    """Create a policy based on consensus from multiple scenario solutions"""
    all_plans = []
    env = Environment(seed=42)
    for i, sc in enumerate(scenarios):
        plan, success, Z, M = _solve_deterministic(env, sc)
        env.reset()
        if success:
            all_plans.append((plan, Z, M))

    if not all_plans:
        # Fallback: use the first plan
        all_plans = [(plan, 0, 0)]

    # Find the plan with highest Z (primary) then M (secondary)
    best_plan = max(all_plans, key=lambda x: (x[1], x[2]))
    plan_actions = best_plan[0]

    day_idx = [0]

    def policy(state, weather):
        idx = day_idx[0]
        if idx < len(plan_actions):
            act, repl = plan_actions[idx]
            # Validate: if action is replenish but not at supply point, skip
            if act == "replenish":
                sn = env.get_supply_name(state.x, state.y)
                if not sn:
                    # Not at supply point, use stay instead
                    day_idx[0] += 1
                    return "stay", None
            # Validate: move must be legal
            if act.startswith("move_"):
                dx, dy = MOVE_DIRS.get(act, (0, 0))
                nx, ny = state.x + dx, state.y + dy
                if not (1 <= nx <= GRID_SIZE and 1 <= ny <= GRID_SIZE):
                    day_idx[0] += 1
                    return "stay", None
            # Validate: work must be at work point
            if act == "work":
                wn = env.get_work_name(state.x, state.y)
                if not wn or state.work_days.get(wn, 0) >= WORK_MAX_CONSECUTIVE.get(wn, 99):
                    day_idx[0] += 1
                    return "stay", None
            day_idx[0] += 1
            return act, repl
        return "stay", None

    return policy

def main():
    env = Environment(seed=42)
    print("="*60)
    print("Solution 3: Two-Stage Stochastic Programming")
    print("="*60)
    print("Generating weather scenarios...")
    scenarios = _generate_weather_scenarios(20)
    print("Solving deterministic scenarios...")
    policy = two_stage_policy(scenarios)

    state = env.reset(42)
    log = []
    total = 0
    while not state.terminated and state.day <= TIME_LIMIT:
        weather = env.sample_weather()
        act, repl = policy(state, weather)
        log.append(dict(day=state.day, x=state.x, y=state.y, weather=weather,
                        action=act, replenish=repl,
                        O=state.O, H=state.H, F=state.F, M=state.M, Z=state.Z))
        state, reward, done = env.step(state, act, weather, repl)
        total += 1
        if total % 15 == 0 or done:
            print("  Day", state.day, "pos", state.pos, "Z", state.Z,
                  "O", state.O, "H", state.H, "F", state.F, "M", state.M)

    success = state.terminated and (state.x, state.y) == END
    nlog = len(log)
    print("Result: Success={} Z={} M={} Day={}".format(success, state.Z, state.M, state.day))
    base = os.path.dirname(__file__)
    with open(os.path.join(base, "result_two_stage.json"), "w", encoding="utf-8") as f:
        json.dump(dict(success=success, final_Z=state.Z, final_M=state.M,
                       final_day=state.day, log=log), f, ensure_ascii=False, indent=2)
    with open(os.path.join(base, "result_two_stage.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["Day", "x", "y", "Weather", "Action", "Replenish", "O", "H", "F", "M", "Z"])
        for r in log:
            w.writerow([r["day"], r["x"], r["y"], r["weather"], r["action"],
                       str(r["replenish"]), r["O"], r["H"], r["F"], r["M"], r["Z"]])
    print("Saved result_two_stage.json and result_two_stage.csv ({} days)".format(nlog))

if __name__ == "__main__":
    main()