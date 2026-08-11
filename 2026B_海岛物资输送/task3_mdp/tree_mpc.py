"""
Scenario Tree MPC - 纯场景树滚动时域控制
无离线DP依赖，在每日构建5层天气场景树，
叶节点用启发式策略评估价值。
"""
import numpy as np
from math import comb

# ============================================================
# Problem Parameters
# ============================================================
class P:
    T = 90; capacity = 400
    B = (1, 15); E = (30, 15)
    S = [(12, 16), (21, 16)]
    W = [(6, 21), (15, 9), (24, 24)]
    reward = [20, 15, 28]
    max_work = [4, 5, 3]
    price = [2, 1, 2]
    normal_move = [2, 3, 2]
    normal_idle = [1, 1, 1]
    normal_work = [5, 4, 3]
    storm_move = [8, 4, 3]
    storm_idle = [3, 3, 2]
    storm_work = [8, 6, 6]
    p_normal = 0.8; p_storm = 0.2
    init_O = 100; init_H = 150; init_F = 100
    init_M = 750; init_Z = 200


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def consumption(action, weather):
    if weather == 'normal': pref = 'normal'
    else: pref = 'storm'
    if action == 'move': return (getattr(P, f'{pref}_move')[0], getattr(P, f'{pref}_move')[1], getattr(P, f'{pref}_move')[2])
    if action == 'idle': return (getattr(P, f'{pref}_idle')[0], getattr(P, f'{pref}_idle')[1], getattr(P, f'{pref}_idle')[2])
    if action == 'work': return (getattr(P, f'{pref}_work')[0], getattr(P, f'{pref}_work')[1], getattr(P, f'{pref}_work')[2])
    return (0, 0, 0)


def is_work_point(pos):
    for i, w in enumerate(P.W):
        if pos == w: return i
    return None


def is_supply_point(pos):
    for i, s in enumerate(P.S):
        if pos == s: return i
    return None


def get_neighbors(pos):
    x, y = pos; adj = []
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        nx, ny = x + dx, y + dy
        if 1 <= nx <= 30 and 1 <= ny <= 30:
            adj.append((nx, ny))
    return adj


# ============================================================
# Heuristic Leaf Value Function
# ============================================================
Z_SCALE = 100000


def heuristic_value(pos, O, H, F, M, Z, consec_work, work_pt, t_rem):
    if pos == P.E: return Z * Z_SCALE + M
    if t_rem <= 0 or O < 0 or H < 0 or F < 0 or M < 0: return -1e9
    
    dE = manhattan(pos, P.E)
    exp_O = dE * P.normal_move[0]
    exp_H = dE * P.normal_move[1]
    exp_F = dE * P.normal_move[2]
    
    best_val = -1e9
    
    # Option 1: Direct to E (if feasible)
    if O >= exp_O and H >= exp_H and F >= exp_F and dE <= t_rem:
        best_val = Z * Z_SCALE + M
    
    # Option 2: Go to supply then work then E
    for si in range(2):
        sx, sy = P.S[si]
        dS = manhattan(pos, (sx, sy))
        if dS >= t_rem: continue
        exp_oS = dS * P.normal_move[0]
        exp_hS = dS * P.normal_move[1]
        exp_fS = dS * P.normal_move[2]
        if O < exp_oS or H < exp_hS or F < exp_fS: continue
        o_after = O - exp_oS; h_after = H - exp_hS; f_after = F - exp_fS
        # At supply point, we can buy resources
        capacity = P.capacity; current = o_after + h_after + f_after
        max_buy = min(capacity - current, M // sum(P.price))
        if max_buy > 0:
            o_after += max_buy * 0.35; h_after += max_buy * 0.30; f_after += max_buy * 0.35
            o_after = min(o_after, capacity - h_after - f_after)
        
        # Check each work point from supply
        for wi in range(3):
            wx, wy = P.W[wi]
            dSW = manhattan((sx, sy), (wx, wy))
            dWE = manhattan((wx, wy), P.E)
            total = dS + 1 + dSW + P.max_work[wi] + dWE
            if total > t_rem: continue
            # Move S->W
            exp_oSw = dSW * P.normal_move[0]
            exp_hSw = dSW * P.normal_move[1]
            exp_fSw = dSW * P.normal_move[2]
            ow = o_after - exp_oSw; hw = h_after - exp_hSw; fw = f_after - exp_fSw
            # Work at W
            exp_oWork = P.max_work[wi] * P.normal_work[0]
            exp_hWork = P.max_work[wi] * P.normal_work[1]
            exp_fWork = P.max_work[wi] * P.normal_work[2]
            ow -= exp_oWork; hw -= exp_hWork; fw -= exp_fWork
            # W->E
            exp_oE = dWE * P.normal_move[0]
            exp_hE = dWE * P.normal_move[1]
            exp_fE = dWE * P.normal_move[2]
            ow -= exp_oE; hw -= exp_hE; fw -= exp_fE
            if ow >= 0 and hw >= 0 and fw >= 0:
                dist_factor = 1.0 / (1.0 + dS / 5.0 + dSW / 10.0)
                bonus = P.max_work[wi] * P.reward[wi] * dist_factor
                # Subtract buy cost from M
                buy_cost = max_buy * sum(P.price) * 0.5
                val = (Z + bonus) * Z_SCALE + max(0, M - buy_cost)
                best_val = max(best_val, val)
    
    # Option 3: Direct to work point then E (if resources allow)
    for wi in range(3):
        wx, wy = P.W[wi]
        dW = manhattan(pos, (wx, wy))
        dWE = manhattan((wx, wy), P.E)
        total = dW + P.max_work[wi] + dWE
        if total > t_rem: continue
        exp_ow = dW * P.normal_move[0]
        exp_hw = dW * P.normal_move[1]
        exp_fw = dW * P.normal_move[2]
        exp_oWork = P.max_work[wi] * P.normal_work[0]
        exp_hWork = P.max_work[wi] * P.normal_work[1]
        exp_fWork = P.max_work[wi] * P.normal_work[2]
        exp_oE2 = dWE * P.normal_move[0]
        exp_hE2 = dWE * P.normal_move[1]
        exp_fE2 = dWE * P.normal_move[2]
        if O >= exp_ow + exp_oWork + exp_oE2 and H >= exp_hw + exp_hWork + exp_hE2 and F >= exp_fw + exp_fWork + exp_fE2:
            bonus = P.max_work[wi] * P.reward[wi]
            val = (Z + bonus) * Z_SCALE + M
            best_val = max(best_val, val)
    
    return best_val

def evaluate_action(pos, O, H, F, M, Z, consec_work, work_pt, t_rem, action, action_detail, weather_obs):
    """
    Evaluate an action using a D=4 scenario tree.
    
    Returns: expected_value, where value = Z * Z_SCALE + M (+ heuristic future)
    """
    # Apply action in observed weather
    if action == 'move':
        new_pos = action_detail['target']
        cO, cH, cF = consumption('move', weather_obs)
        new_O = O - cO; new_H = H - cH; new_F = F - cF
        new_M = M; new_Z = Z
        new_wc = 0; new_wp = 0
        wi = is_work_point(new_pos)
        if wi is not None: new_wp = wi + 1
        if new_pos == P.E: return new_Z * Z_SCALE + new_M
        if new_O < 0 or new_H < 0 or new_F < 0: return -1e9
        if new_O + new_H + new_F > P.capacity: return -1e9
    elif action == 'idle':
        new_pos = pos
        cO, cH, cF = consumption('idle', weather_obs)
        new_O = O - cO; new_H = H - cH; new_F = F - cF
        new_M = M; new_Z = Z
        new_wc = 0; new_wp = 0
        if new_O < 0 or new_H < 0 or new_F < 0: return -1e9
        if new_O + new_H + new_F > P.capacity: return -1e9
    elif action == 'work':
        new_pos = pos
        cO, cH, cF = consumption('work', weather_obs)
        gain = P.reward[work_pt - 1]
        new_O = O - cO; new_H = H - cH; new_F = F - cF
        new_M = M; new_Z = Z + gain
        new_wc = consec_work + 1
        new_wp = work_pt
        if new_O < 0 or new_H < 0 or new_F < 0: return -1e9
        if new_O + new_H + new_F > P.capacity: return -1e9
    elif action == 'buy':
        buy = action_detail['buy']
        cost = action_detail['cost']
        new_pos = pos
        new_O = O + buy[0]; new_H = H + buy[1]; new_F = F + buy[2]
        new_M = M - cost; new_Z = Z
        new_wc = 0; new_wp = 0
        if new_O + new_H + new_F > P.capacity: return -1e9
        if new_M < 0: return -1e9
    else:
        return -1e9
    
    # Scenario tree evaluation (depth 4)
    tree_depth = 3
    value = scenario_tree_value(new_pos, new_O, new_H, new_F, new_M, new_Z,
                                new_wc, new_wp, t_rem - 1, tree_depth)
    return value


def scenario_tree_value(pos, O, H, F, M, Z, wc, wp, t_rem, depth):
    """Recursive scenario tree evaluation."""
    if pos == P.E:
        return Z * Z_SCALE + M
    if t_rem <= 0:
        return -1e9
    if O < 0 or H < 0 or F < 0 or M < 0:
        return -1e9
    
    if depth <= 0:
        return heuristic_value(pos, O, H, F, M, Z, wc, wp, t_rem)
    
    # At this node, the ship observes weather and chooses best action
    # Weather is unknown (we're in the tree), so we take expectation
    best_val = -1e9
    
    # Generate candidate actions
    candidates = []
    
    # Move actions
    for nx, ny in get_neighbors(pos):
        # We don't know the weather at this tree node, use expected consumption
        exp_cO = P.p_normal * P.normal_move[0] + P.p_storm * P.storm_move[0]
        exp_cH = P.p_normal * P.normal_move[1] + P.p_storm * P.storm_move[1]
        exp_cF = P.p_normal * P.normal_move[2] + P.p_storm * P.storm_move[2]
        nO = O - exp_cO; nH = H - exp_cH; nF = F - exp_cF
        if nO < 0 or nH < 0 or nF < 0: continue
        if nO + nH + nF > P.capacity: continue
        if (nx, ny) == P.E:
            candidates.append(('term', Z * Z_SCALE + M, (nx, ny), nO, nH, nF, M, Z, 0, 0))
        else:
            nwp_val = is_work_point((nx, ny))
            nwp = nwp_val + 1 if nwp_val is not None else 0
            val = scenario_tree_value((nx, ny), nO, nH, nF, M, Z, 0, nwp, t_rem - 1, depth - 1)
            if val > -1e8:
                candidates.append(('move', val, (nx, ny), nO, nH, nF, M, Z, 0, nwp))
    
    # Idle
    exp_cO = P.p_normal * P.normal_idle[0] + P.p_storm * P.storm_idle[0]
    exp_cH = P.p_normal * P.normal_idle[1] + P.p_storm * P.storm_idle[1]
    exp_cF = P.p_normal * P.normal_idle[2] + P.p_storm * P.storm_idle[2]
    nO = O - exp_cO; nH = H - exp_cH; nF = F - exp_cF
    if nO >= 0 and nH >= 0 and nF >= 0 and nO + nH + nF <= P.capacity:
        val = scenario_tree_value(pos, nO, nH, nF, M, Z, 0, 0, t_rem - 1, depth - 1)
        if val > -1e8:
            candidates.append(('idle', val, pos, nO, nH, nF, M, Z, 0, 0))
    
    # Work (if at work point)
    wp_idx = is_work_point(pos)
    if wp_idx is not None and (wp == 0 or (wp == wp_idx + 1 and wc < P.max_work[wp_idx])):
        exp_cO = P.p_normal * P.normal_work[0] + P.p_storm * P.storm_work[0]
        exp_cH = P.p_normal * P.normal_work[1] + P.p_storm * P.storm_work[1]
        exp_cF = P.p_normal * P.normal_work[2] + P.p_storm * P.storm_work[2]
        nO = O - exp_cO; nH = H - exp_cH; nF = F - exp_cF
        if nO >= 0 and nH >= 0 and nF >= 0 and nO + nH + nF <= P.capacity:
            nZ = Z + P.reward[wp_idx]
            nwc = wc + 1 if wp == wp_idx + 1 else 1
            nwp = wp_idx + 1
            val = scenario_tree_value(pos, nO, nH, nF, M, nZ, nwc, nwp, t_rem - 1, depth - 1)
            if val > -1e8:
                candidates.append(('work', val, pos, nO, nH, nF, M, nZ, nwc, nwp))
    
    for c in candidates:
        best_val = max(best_val, c[1])
    return best_val


# ============================================================
# MPC Controller
# ============================================================
class TreeMPC:
    def __init__(self):
        pass
    
    def decide(self, state, weather, day):
        pos = state['position']; O, H, F = state['O'], state['H'], state['F']
        M, Z = state['M'], state['Z']
        wc, wp = state['consecutive_work'], state['work_point']
        t_rem = P.T - day + 1
        
        candidates = []
        
        # Move actions
        for nx, ny in get_neighbors(pos):
            cO, cH, cF = consumption('move', weather)
            nO, nH, nF = O - cO, H - cH, F - cF
            if nO < 0 or nH < 0 or nF < 0: continue
            if nO + nH + nF > P.capacity: continue
            if (nx, ny) == P.E:
                val = Z * Z_SCALE + M
            else:
                dE = manhattan((nx, ny), P.E)
                nwp_val = is_work_point((nx, ny))
                nwp = nwp_val + 1 if nwp_val is not None else 0
                val = scenario_tree_value((nx, ny), nO, nH, nF, M, Z, 0, nwp, t_rem - 1, 4)
                val = val + (manhattan(pos, P.E) - dE) * 0.5  # Reward progress toward E
            if val > -1e8:
                candidates.append({'action': 'move', 'target': (nx, ny), 'value': val,
                    'details': {'cons': (cO, cH, cF)}})
        
        # Idle
        cO, cH, cF = consumption('idle', weather)
        nO, nH, nF = O - cO, H - cH, F - cF
        if nO >= 0 and nH >= 0 and nF >= 0 and nO + nH + nF <= P.capacity:
            val = scenario_tree_value(pos, nO, nH, nF, M, Z, 0, 0, t_rem - 1, 4)
            if val > -1e8:
                candidates.append({'action': 'idle', 'target': pos, 'value': val,
                    'details': {'cons': (cO, cH, cF)}})
        
        # Work
        wp_idx = is_work_point(pos)
        if wp_idx is not None and wc < P.max_work[wp_idx]:
            cO, cH, cF = consumption('work', weather)
            nO, nH, nF = O - cO, H - cH, F - cF
            if nO >= 0 and nH >= 0 and nF >= 0 and nO + nH + nF <= P.capacity:
                nZ = Z + P.reward[wp_idx]
                nwc = wc + 1
                val = scenario_tree_value(pos, nO, nH, nF, M, nZ, nwc, wp_idx + 1, t_rem - 1, 4)
                if val > -1e8:
                    candidates.append({'action': 'work', 'target': pos, 'value': val,
                        'details': {'gain': P.reward[wp_idx], 'cons': (cO, cH, cF), 'nwc': nwc}})
        
        # Buy
        sp_idx = is_supply_point(pos)
        if sp_idx is not None:
            rem_cap = P.capacity - (O + H + F)
            buys_to_try = [(0, 0, 0)]
            if rem_cap > 0 and M > 0:
                for frac in [0.3, 0.6, 1.0]:
                    bo = int(frac * rem_cap * 0.35); bh = int(frac * rem_cap * 0.30); bf = int(frac * rem_cap * 0.35)
                    bo = min(bo, rem_cap); bh = min(bh, rem_cap - bo); bf = min(bf, rem_cap - bo - bh)
                    cost = bo * P.price[0] + bh * P.price[1] + bf * P.price[2]
                    if cost <= M: buys_to_try.append((bo, bh, bf))
            buys_to_try = list(set(buys_to_try))
            for bo, bh, bf in buys_to_try:
                if bo == 0 and bh == 0 and bf == 0: continue
                cost = bo * P.price[0] + bh * P.price[1] + bf * P.price[2]
                if cost > M: continue
                nO, nH, nF = O + bo, H + bh, F + bf
                nM = M - cost
                val = scenario_tree_value(pos, nO, nH, nF, nM, Z, 0, 0, t_rem, 3)
                val = val - cost * 1e-6
                if val > -1e8:
                    candidates.append({'action': 'buy', 'target': pos, 'value': val,
                        'details': {'buy': (bo, bh, bf), 'cost': cost}})
        
        if not candidates:
            return None, None
        best = max(candidates, key=lambda c: c['value'])
        return best['action'], best


# ============================================================
# Simulation
# ============================================================
def run_tree_mpc(weather_seq, verbose=True):
    ctrl = TreeMPC()
    S = {'position': P.B, 'O': P.init_O, 'H': P.init_H, 'F': P.init_F,
         'M': P.init_M, 'Z': P.init_Z, 'consecutive_work': 0, 'work_point': 0}
    recs = []; route = [P.B]; arr_day = 0
    
    for day in range(1, P.T + 1):
        w = weather_seq[day - 1]
        if S['position'] == P.E and not arr_day: arr_day = day
        if S['position'] == P.E and arr_day and day > arr_day: break
        
        act, ad = ctrl.decide(S, w, day)
        if act is None:
            return {'feasible': False, 'failure_day': day, 'records': recs}
        
        rec = {'Day': day, 'Weather': w, 'StartX': S['position'][0], 'StartY': S['position'][1],
            'Action': '', 'EndX': S['position'][0], 'EndY': S['position'][1],
            'BuyO': 0, 'BuyH': 0, 'BuyF': 0, 'Gain': 0,
            'O': S['O'], 'H': S['H'], 'F': S['F'], 'M': S['M'], 'Z': S['Z']}
        
        if act == 'move':
            np2 = ad['target']; cons = ad['details']['cons']
            rec['Action'] = 'Move(%d,%d)' % np2; rec['EndX'], rec['EndY'] = np2
            S['position'] = np2; S['O'] -= cons[0]; S['H'] -= cons[1]; S['F'] -= cons[2]
            S['consecutive_work'] = 0; S['work_point'] = 0
            wpn = is_work_point(np2)
            if wpn is not None: S['work_point'] = wpn + 1
            route.append(np2)
            if np2 == P.E: arr_day = day
        elif act == 'idle':
            cons = ad['details']['cons']; rec['Action'] = 'Stay'
            S['O'] -= cons[0]; S['H'] -= cons[1]; S['F'] -= cons[2]
            S['consecutive_work'] = 0; S['work_point'] = 0
        elif act == 'work':
            cons = ad['details']['cons']; gain = ad['details']['gain']
            rec['Action'] = 'Work(+%d)' % gain; rec['Gain'] = gain
            S['O'] -= cons[0]; S['H'] -= cons[1]; S['F'] -= cons[2]
            S['Z'] += gain; S['consecutive_work'] = ad['details']['nwc']
        elif act == 'buy':
            buy = ad['details']['buy']; cost = ad['details']['cost']
            rec['Action'] = 'Buy(%d,%d,%d)' % buy
            rec['BuyO'], rec['BuyH'], rec['BuyF'] = buy
            S['O'] += buy[0]; S['H'] += buy[1]; S['F'] += buy[2]; S['M'] -= cost
        
        rec['O'], rec['H'], rec['F'] = S['O'], S['H'], S['F']
        rec['M'], rec['Z'] = S['M'], S['Z']
        recs.append(rec)
        
        if S['O'] < 0 or S['H'] < 0 or S['F'] < 0 or S['M'] < 0:
            return {'feasible': False, 'failure_day': day, 'records': recs}
        if S['O'] + S['H'] + S['F'] > P.capacity:
            return {'feasible': False, 'failure_day': day, 'records': recs}
        
        if verbose and day % 15 == 0:
            print('  Day%d: pos=%s Z=%d M=%d O=%d' % (day, str(S['position']), S['Z'], S['M'], S['O']))
    
    return {'feasible': True if arr_day > 0 else False,
            'arrival_day': arr_day if arr_day > 0 else P.T,
            'final_Z': S['Z'], 'final_M': S['M'],
            'records': recs, 'route': route}


def monte_carlo_tree(n=200, seed=42):
    print('\nMC with scenario tree MPC (%d runs)...' % n)
    np.random.seed(seed)
    succ = 0; zs = []; ms = []
    for i in range(n):
        w = ['storm' if np.random.random() < P.p_storm else 'normal' for _ in range(P.T)]
        r = run_tree_mpc(w, verbose=False)
        if r['feasible']:
            succ += 1; zs.append(r['final_Z']); ms.append(r['final_M'])
        if (i + 1) % 50 == 0:
            print('  %d/%d, rate=%.3f' % (i + 1, n, succ / (i + 1)))
    rate = succ / n
    z2 = 1.96; denom = 1 + z2**2 / n
    center = (rate + z2**2 / (2 * n)) / denom
    margin = z2 * np.sqrt(rate * (1 - rate) / n + z2**2 / (4 * n**2)) / denom
    lo95 = max(0, center - margin)
    print('\nSuccess: %.4f (%d/%d), 95%% lower=%.4f' % (rate, succ, n, lo95))
    if zs:
        print('Z: %.0f+-%.0f, M: %.0f+-%.0f' % (np.mean(zs), np.std(zs), np.mean(ms), np.std(ms)))
    return {'samples': n, 'successes': succ, 'rate': rate, 'lower95': lo95,
            'z_mean': np.mean(zs) if zs else 0, 'm_mean': np.mean(ms) if ms else 0}


def plot_route(route, path='task3_route_tree.png'):
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 8))
        ra = np.array(route)
        ax.plot(ra[:, 0], ra[:, 1], '-o', lw=1.5, ms=3, color='steelblue')
        ax.plot(P.B[0], P.B[1], 'go', ms=12, label='B')
        ax.plot(P.E[0], P.E[1], 'ro', ms=12, label='E')
        for i, s in enumerate(P.S):
            ax.plot(s[0], s[1], 'bs', ms=10, label='S%d' % (i + 1) if i == 0 else '')
        for i, w in enumerate(P.W):
            ax.plot(w[0], w[1], 'm^', ms=10, label='W%d' % (i + 1) if i == 0 else '')
        ax.set_xlim(0.5, 30.5); ax.set_ylim(0.5, 30.5)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_title('Task3 Scenario Tree MPC Route')
        ax.grid(alpha=0.3); ax.legend(loc='upper right'); ax.set_aspect('equal')
        plt.tight_layout(); fig.savefig(path, dpi=150); plt.close()
        print('Route saved to %s' % path)
    except Exception as e:
        print('Plot failed: %s' % e)


def save_tree_results(result, mc, path='task3_tree_result.xlsx'):
    try:
        import pandas as pd
        if not result['feasible']:
            print('No feasible solution')
            return
        df = pd.DataFrame(result['records'])
        cols = ['Day', 'Weather', 'StartX', 'StartY', 'Action', 'EndX', 'EndY',
                'BuyO', 'BuyH', 'BuyF', 'Gain', 'O', 'H', 'F', 'M', 'Z']
        df = df[[c for c in cols if c in df.columns]]
        with pd.ExcelWriter(path, engine='openpyxl') as w:
            df.to_excel(w, sheet_name='DailyPlan', index=False)
            sm = pd.DataFrame({
                'Metric': ['Arrival', 'Z', 'M', 'MC Rate', 'MC 95% Lower'],
                'Value': [result['arrival_day'], result['final_Z'], result['final_M'],
                          '%.4f' % mc['rate'], '%.4f' % mc['lower95']]
            })
            sm.to_excel(w, sheet_name='Summary', index=False)
        print('Results saved to %s' % path)
    except Exception as e:
        print('Save failed: %s' % e)


if __name__ == '__main__':
    import time
    print('=' * 60)
    print('Task 3: Scenario Tree MPC')
    print('=' * 60)
    
    np.random.seed(2026)
    weather = ['storm' if np.random.random() < P.p_storm else 'normal' for _ in range(P.T)]
    
    t0 = time.time()
    result = run_tree_mpc(weather, verbose=True)
    print('Simulation in %.1fs' % (time.time() - t0))
    
    if result['feasible']:
        print('\n=== Result ===')
        print('Arrival: day %d, Z=%d, M=%d' % (result['arrival_day'], result['final_Z'], result['final_M']))
        plot_route(result['route'])
    else:
        print('Failed at day %s' % result.get('failure_day', '?'))
    
    t0 = time.time()
    mc = monte_carlo_tree(n=50, seed=42)
    print('MC in %.1fs' % (time.time() - t0))
    
    save_tree_results(result, mc)
    print('\nDone!')
