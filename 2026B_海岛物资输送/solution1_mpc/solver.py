"""Solution 1: Path-Guided MPC - All Fixes Applied"""
import sys, os, csv, json, random, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
base_dir = os.path.dirname(__file__)
sys.stdout = open(os.path.join(base_dir, "output.log"), "w", encoding="utf-8")
from environment import (Environment, ShipState, GRID_SIZE, START, END,
                         SUPPLY, WORK, WORK_YIELD, WORK_MAX_CONSECUTIVE,
                         LOAD_CAP, TIME_LIMIT, PRICE_O, PRICE_H, PRICE_F,
                         P_NORMAL, P_STORM, CONSUME)

MOVE_DIRS = {"move_up": (-1, 0), "move_down": (1, 0),
             "move_left": (0, -1), "move_right": (0, 1)}
_PN, _PS = P_NORMAL, P_STORM

def _md(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def _exp_cost_move():
    return (_PN*(2+3+2) + _PS*(8+4+3))

def _exp_cost_work():
    return (_PN*(5+4+3) + _PS*(8+6+6))

def _exp_cost_stay():
    return (_PN*(1+1+1) + _PS*(3+3+2))

def plan_global_path():
    """Enumerate all permutations, insert supply stops when resources insufficient"""
    wp_names = list(WORK.keys())
    best_path, best_value = None, -1.0
    INITIAL_RES = 350  # O+H+F

    for order in itertools.permutations(wp_names):
        path = [START]
        current_res = INITIAL_RES
        total_benefit = 0
        feasible = True

        for wn in order:
            wp = WORK[wn]
            travel_cost = _md(path[-1], wp) * _exp_cost_move()
            work_cost = WORK_MAX_CONSECUTIVE[wn] * _exp_cost_work()
            needed = (travel_cost + work_cost) * 1.2

            if current_res < needed:
                # Find nearest supply point to insert
                best_sp, best_detour = None, 999
                for sn, sp in SUPPLY.items():
                    detour = _md(path[-1], sp) + _md(sp, wp)
                    if detour < best_detour:
                        best_detour = detour
                        best_sp = sp
                if best_sp:
                    path.append(best_sp)
                    # After replenishment: fill to near capacity
                    remaining_need = (_md(best_sp, wp) * _exp_cost_move() + work_cost +
                                     _md(wp, END) * _exp_cost_move()) * 1.2
                    current_res = min(LOAD_CAP, max(current_res, int(remaining_need)))
                    travel_cost = _md(best_sp, wp) * _exp_cost_move()

            if current_res < travel_cost + work_cost:
                feasible = False
                break

            path.append(wp)
            total_benefit += WORK_MAX_CONSECUTIVE[wn] * WORK_YIELD[wn]
            current_res -= (travel_cost + work_cost)

        if not feasible:
            continue

        # Final leg to END
        final_cost = _md(path[-1], END) * _exp_cost_move()
        if current_res < final_cost * 1.2:
            best_sp, best_detour = None, 999
            for sn, sp in SUPPLY.items():
                detour = _md(path[-1], sp) + _md(sp, END)
                if detour < best_detour:
                    best_detour = detour
                    best_sp = sp
            if best_sp and best_detour < _md(path[-1], END) + 30:
                path.append(best_sp)

        path.append(END)

        # Check total days
        total_days = 0
        for i in range(len(path) - 1):
            total_days += _md(path[i], path[i+1])
        total_days += sum(WORK_MAX_CONSECUTIVE[wn] for wn in order)

        if total_days > TIME_LIMIT:
            continue

        value = total_benefit
        if value > best_value:
            best_value = value
            best_path = path
    return best_path if best_path else [START, END]
    return best_path if best_path else [START, END]


def _can_reach(state, target, safety=1.3):
    dist = _md(state.pos, target)
    needed = dist * _exp_cost_move() * safety
    return (state.O + state.H + state.F) >= needed


def _forward_replenish_amount(state, next_target):
    
    current = state.O + state.H + state.F
    target_fill = int(LOAD_CAP * 0.85)
    if current >= target_fill - 10:
        return None
    space = LOAD_CAP - current
    buy = target_fill - current
    buy = min(buy, space)
    o = min(buy * 5 // 10, space, state.M // PRICE_O)
    h = min(buy * 3 // 10, space - o, state.M // PRICE_H)
    f = min(buy * 2 // 10, space - o - h, state.M // PRICE_F)
    return (max(0, o), max(0, h), max(0, f))


def _move_towards(pos, target):
    x, y = pos; tx, ty = target
    dx_err, dy_err = tx - x, ty - y
    legal = []
    for an, (dx, dy) in MOVE_DIRS.items():
        nx, ny = x + dx, y + dy
        if 1 <= nx <= GRID_SIZE and 1 <= ny <= GRID_SIZE:
            d = abs(nx - tx) + abs(ny - ty)
            reduces_x = (dx_err > 0 and dx > 0) or (dx_err < 0 and dx < 0)
            reduces_y = (dy_err > 0 and dy > 0) or (dy_err < 0 and dy < 0)
            prefer_x = abs(dx_err) >= abs(dy_err)
            if prefer_x and reduces_x:
                tiebreak = 0
            elif not prefer_x and reduces_y:
                tiebreak = 0
            elif reduces_x or reduces_y:
                tiebreak = 1
            else:
                tiebreak = 2
            legal.append((d, tiebreak, an))
    legal.sort()
    return legal[0][2] if legal else "stay" 


def mpc_policy(state, weather, global_path, path_idx):
    x, y = state.x, state.y
    wd = state.work_days

    if path_idx[0] >= len(global_path):
        return "stay", None

    target = global_path[path_idx[0]]

    # Reached current target?
    if (x, y) == target:
        wn = None
        for n, wp in WORK.items():
            if wp == target:
                wn = n
                break
        if wn and wd.get(wn, 0) < WORK_MAX_CONSECUTIVE[wn]:
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
                if state.O < work_O + move_O + (40 if target == (15,9) else 10):
                    path_idx[0] += 1
                    if path_idx[0] >= len(global_path):
                        return "stay", None
                    target = global_path[path_idx[0]]
                else:
                    return "work", None
            else:
                return "work", None
        else:
            path_idx[0] += 1
            if path_idx[0] >= len(global_path):
                return "stay", None
            target = global_path[path_idx[0]]

    # Dynamic replanning: skip unreachable targets
    skipped = 0
    while not _can_reach(state, target, 1.2) and path_idx[0] < len(global_path) - 1:
        next_idx = path_idx[0] + 1
        if next_idx < len(global_path):
            next_target = global_path[next_idx]
            if _can_reach(state, next_target, 1.2):
                path_idx[0] = next_idx
                target = next_target
                print("  [REPLAN] Day", state.day, ": skip to", target)
                break
            else:
                path_idx[0] = next_idx
        skipped += 1
        if skipped > 3:
            break

    # Forward-looking replenishment at supply points
    sn = None
    for n, sp in SUPPLY.items():
        if (x, y) == sp:
            sn = n
            break
    if sn:
        repl = _forward_replenish_amount(state, target)
        if repl is not None:
            return "replenish", repl


    # Move towards target
    act = _move_towards(state.pos, target)

    # Adaptive storm avoidance
    if weather == "storm":
        remaining = TIME_LIMIT - state.day
        dist_to_end = _md(state.pos, END)
        time_budget = remaining - dist_to_end

        if time_budget > 15:
            safe_O = 10
        elif time_budget > 5:
            safe_O = 5
        else:
            safe_O = 0

        eO, eH, eF = CONSUME["storm"]["move"]
        if state.O - eO < safe_O or state.H - eH < safe_O or state.F - eF < safe_O:
            act = "stay"

    return act, None


def main():
    env = Environment(seed=42)

    global_path = plan_global_path()

    print("Global path:", global_path)
    print()

    state = env.reset(42)
    log = []
    path_idx = [0]
    total = 0

    while not state.terminated and state.day <= TIME_LIMIT:
        weather = env.sample_weather()
        act, repl = mpc_policy(state, weather, global_path, path_idx)

        log.append(dict(day=state.day, x=state.x, y=state.y, weather=weather,
                        action=act, replenish=repl,
                        O=state.O, H=state.H, F=state.F, M=state.M, Z=state.Z))

        state, reward, done = env.step(state, act, weather, repl)
        total += 1

        if done:
            print("Day", state.day, "Terminated: reward", reward)
            break
        if total % 10 == 0:
            tgt = global_path[min(path_idx[0], len(global_path) - 1)]
            print("Day", state.day, "pos", state.pos, "->", tgt,
                  "Z", state.Z, "O", state.O, "H", state.H, "F", state.F, "M", state.M)

    success = state.terminated and (state.x, state.y) == END
    print("\nRESULT: Success", success, "Z", state.Z, "M", state.M, "Day", state.day)

    moves = sum(1 for r in log if r['action'].startswith('move_'))
    works = sum(1 for r in log if r['action'] == 'work')
    repls = sum(1 for r in log if r['action'] == 'replenish')
    stays = sum(1 for r in log if r['action'] == 'stay')
    storms = sum(1 for r in log if r['weather'] == 'storm')
    print("Actions: move", moves, "work", works, "replenish", repls, "stay", stays,
          "storms", storms)

    nlog = len(log)
    with open(os.path.join(base_dir, "result_mpc.json"), "w", encoding="utf-8") as f:
        json.dump(dict(success=success, final_Z=state.Z, final_M=state.M,
                       final_day=state.day,
                       global_path=[list(p) for p in global_path],
                       log=log,
                       stats=dict(move=moves, work=works, replenish=repls,
                                  stay=stays, storms=storms)),
                  f, ensure_ascii=False, indent=2)
    with open(os.path.join(base_dir, "result_mpc.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["Day", "x", "y", "Weather", "Action", "Replenish", "O", "H", "F", "M", "Z"])
        for r in log:
            w.writerow([r["day"], r["x"], r["y"], r["weather"], r["action"],
                       str(r["replenish"]), r["O"], r["H"], r["F"], r["M"], r["Z"]])
    print("Saved", nlog, "days")
    sys.stdout.close()


if __name__ == "__main__":
    main()