"""
=================================================================
 任务三：场景生成 + 随机MILP
 两层方法：
   Layer 1 - 战略MILP：确定功能点访问顺序和资源分配
   Layer 2 - 场景驱动贪婪导航：基于场景评估的逐日决策
=================================================================
"""
import numpy as np, random, time, json, csv, os, math, itertools
from pulp import (LpProblem, LpMaximize, LpVariable, LpInteger, LpContinuous,
                  lpSum, LpStatus, value, PULP_CBC_CMD)

# ========== 问题参数 ==========
T_DAYS = 90; PN = 0.8; PS = 0.2

B = (1, 15); E = (30, 15)
S = [(12, 16), (21, 16)]
W = [(6, 21), (15, 9), (24, 24)]
W_YIELD = [20, 15, 28]
W_MAX = [4, 5, 3]

INIT_O, INIT_H, INIT_F = 100, 150, 100
INIT_M, INIT_Z = 750, 200
CAP = 400
PRICE = [2, 1, 2]

MOVE_C = [(2,3,2),(8,4,3)]
IDLE_C = [(1,1,1),(3,3,2)]
WORK_C = [(5,4,3),(8,6,6)]

def md(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])
def wcost(wc,ct,r): return ct[wc][r]

# ========== 场景生成 ==========
def gen_scenarios(n, days, seed=None):
    rng = random.Random(seed); scs=[]
    for _ in range(n):
        sc=[0 if rng.random()<PN else 1 for __ in range(days)]
        scs.append(sc)
    return scs

# ========== Layer 1: 战略MILP ==========
def solve_strategic_milp(current_pos, current_res, today_w, rem_days):
    """
    紧凑战略MILP: 决定访问哪些功能点、顺序、工作天数、购买量。
    基于期望天气消耗。
    返回: (skeleton, work_days, buy_amounts, Z, M, feasible)
    """
    # 功能点: B(=当前), S1, S2, W1, W2, W3, E - 去重复
    all_func = [current_pos] + S + W + [E]
    dedup = []; seen = set()
    for p in all_func:
        t = tuple(p)
        if t not in seen: seen.add(t); dedup.append(p)
    FP = dedup  # functional points

    # 类型: 0=B/当前, 1=S, 2=W, 3=E
    ftype = []; fname = []
    for p in FP:
        t = tuple(p)
        if t == tuple(current_pos): ftype.append(0); fname.append('CUR')
        elif t == tuple(E): ftype.append(3); fname.append('E')
        elif t in [tuple(s) for s in S]:
            idx = [tuple(s) for s in S].index(t)
            ftype.append(1); fname.append(f'S{idx+1}')
        elif t in [tuple(w) for w in W]:
            idx = [tuple(w) for w in W].index(t)
            ftype.append(2); fname.append(f'W{idx+1}')
        else: ftype.append(0); fname.append('?')

    # 主框架: 枚举所有可能的NO WORK路径 (快速扫描)
    # 然后对候选路径进行MILP优化

    # 简化: 限制中间点 ≤ 7, 直接从MILP决定
    # 允许重复访问 (对补给平台)
    max_visits = 8  # 最多访问8个功能点
    nFP = len(FP)
    cur_idx = [i for i in range(nFP) if ftype[i]==0 and fname[i]=='CUR'][0]
    E_idx = [i for i in range(nFP) if ftype[i]==3][0]

    # 简化: 枚举候选骨架，对每个骨架求解MILP
    # 骨架: [cur, ..., E] 的序列
    # 中间点: S1,S2,W1,W2,W3 (最多5个不同点) + 可重复S
    mid_candidates = [i for i in range(nFP) if i != cur_idx and i != E_idx]

    best_Z = -1; best_M = -1; best_result = None

    # 枚举: 每个中点可选/不选, 最多 max_mid 个
    # 用排列枚举 (限制长度)
    for n_mid in range(0, min(6, len(mid_candidates)) + 1):
        for perm in itertools.permutations(mid_candidates, n_mid):
            skeleton = [cur_idx] + list(perm) + [E_idx]
            result = milp_for_skeleton(skeleton, FP, ftype, fname,
                                       current_res, today_w, rem_days)
            if result and result['feasible']:
                if result['Z'] > best_Z or (result['Z'] == best_Z and result['M'] > best_M):
                    best_Z = result['Z']; best_M = result['M']; best_result = result

    # 也尝试允许重复访问补给平台的骨架
    # 对最优骨架，尝试在补给平台间插入额外访问
    if best_result:
        skeleton = best_result['skeleton']
        # 在已有补给平台之后插入额外补给
        for repeat_S in [True, False]:
            if not repeat_S: break
            enhanced = []
            for idx in skeleton:
                enhanced.append(idx)
                if ftype[idx] == 1:  # supply point, consider repeating
                    enhanced.append(idx)
            if len(enhanced) > max_visits:
                continue
            result = milp_for_skeleton(enhanced, FP, ftype, fname,
                                       current_res, today_w, rem_days)
            if result and result['feasible']:
                if result['Z'] > best_Z or (result['Z'] == best_Z and result['M'] > best_M):
                    best_Z = result['Z']; best_M = result['M']; best_result = result

    return best_result


def milp_for_skeleton(skeleton, FP, ftype, fname, current_res, today_w, rem_days):
    """对给定骨架求解MILP（工作天数 + 购买量）"""
    m = len(skeleton)
    nW_visits = sum(1 for i in skeleton if ftype[i]==2)
    nS_visits = sum(1 for i in skeleton if ftype[i]==1)

    if m == 0: return None

    prob = LpProblem("Strategic", LpMaximize)

    # 变量: w[v]: 第v个作业点的天数 (按骨架中W出现的顺序)
    w_vars = []
    w_to_skel = {}  # 映射: w_var_index -> skeleton index
    for idx in skeleton:
        if ftype[idx] == 2:
            # 确定是哪个W
            wname = fname[idx]  # W1, W2, W3
            wnum = int(wname[1]) - 1
            max_w = W_MAX[wnum]
            var = LpVariable(f"w_{len(w_vars)}", lowBound=0, upBound=max_w, cat=LpInteger)
            w_vars.append(var)
            w_to_skel[len(w_vars)-1] = idx

    # 变量: buy[v][r][sidx]: 第v个补给访问购买资源r从平台sidx
    buy_vars = []  # list of (var_O, var_H, var_F, skel_idx, platform_idx)
    s_visit = 0
    for idx in skeleton:
        if ftype[idx] == 1:
            sname = fname[idx]
            snum = int(sname[1]) - 1
            bO = LpVariable(f"bO_{s_visit}", lowBound=0, cat=LpContinuous)
            bH = LpVariable(f"bH_{s_visit}", lowBound=0, cat=LpContinuous)
            bF = LpVariable(f"bF_{s_visit}", lowBound=0, cat=LpContinuous)
            buy_vars.append((bO, bH, bF, idx, snum))
            s_visit += 1

    # 变量: res[v][0..3] = O,H,F,M after visit v
    res_vars = []
    for v in range(m):
        rO = LpVariable(f"rO_{v}", lowBound=0, upBound=CAP, cat=LpContinuous)
        rH = LpVariable(f"rH_{v}", lowBound=0, upBound=CAP, cat=LpContinuous)
        rF = LpVariable(f"rF_{v}", lowBound=0, upBound=CAP, cat=LpContinuous)
        rM = LpVariable(f"rM_{v}", lowBound=0, cat=LpContinuous)
        res_vars.append((rO, rH, rF, rM))

    # 初始条件
    prob += res_vars[0][0] == current_res['Water']
    prob += res_vars[0][1] == current_res['Fuel']
    prob += res_vars[0][2] == current_res['Food']
    prob += res_vars[0][3] == current_res['M']

    # 载重约束
    for v in range(m):
        prob += res_vars[v][0] + res_vars[v][1] + res_vars[v][2] <= CAP

    # 段间转移方程
    # 段v: 从 skeleton[v] 到 skeleton[v+1]
    # 距离 = md(FP[skeleton[v]], FP[skeleton[v+1]])
    # 消耗 = 期望移动消耗 (基于期望天气)
    # 如果当前是作业点: 工作w天, 消耗 = w * 期望作业消耗
    # 如果当前是补给: 可以buy, 无时间消耗 (即时)

    w_counter = 0
    s_counter = 0
    Z_total = current_res['Z']
    total_days_used = 0

    for v in range(m - 1):
        fr = skeleton[v]; to = skeleton[v + 1]
        d = md(tuple(FP[fr]), tuple(FP[to]))
        total_days_used += d

        # 移动消耗: 第一段用实际天气, 其余用期望
        if v == 0 and total_days_used == d:
            # 只有第一段而且是第一次移动
            move_O = wcost(today_w, MOVE_C, 0) + (d-1) * (PN*MOVE_C[0][0] + PS*MOVE_C[1][0])
            move_H = wcost(today_w, MOVE_C, 1) + (d-1) * (PN*MOVE_C[0][1] + PS*MOVE_C[1][1])
            move_F = wcost(today_w, MOVE_C, 2) + (d-1) * (PN*MOVE_C[0][2] + PS*MOVE_C[1][2])
        else:
            move_O = d * (PN*MOVE_C[0][0] + PS*MOVE_C[1][0])
            move_H = d * (PN*MOVE_C[0][1] + PS*MOVE_C[1][1])
            move_F = d * (PN*MOVE_C[0][2] + PS*MOVE_C[1][2])

        # 当前点的工作/补给
        work_O = 0; work_H = 0; work_F = 0; work_days = 0; z_gain = 0
        buy_O = 0; buy_H = 0; buy_F = 0; buy_cost = 0

        if ftype[fr] == 2:  # work point
            wv = w_vars[w_counter]
            wdays_var = wv
            work_days = wdays_var
            total_days_used_var = None  # will handle below
            # 期望工作消耗
            if v == 0 and d == 0:  # 在原地工作
                exp_wO = wcost(today_w, WORK_C, 0)
                exp_wH = wcost(today_w, WORK_C, 1)
                exp_wF = wcost(today_w, WORK_C, 2)
            else:
                exp_wO = PN*WORK_C[0][0] + PS*WORK_C[1][0]
                exp_wH = PN*WORK_C[0][1] + PS*WORK_C[1][1]
                exp_wF = PN*WORK_C[0][2] + PS*WORK_C[1][2]
            wname = fname[fr]
            wnum = int(wname[1]) - 1
            z_gain = wv * W_YIELD[wnum]
            Z_total = Z_total + z_gain
            w_counter += 1
        elif ftype[fr] == 1:  # supply point
            bO, bH, bF, sk_idx, snum = buy_vars[s_counter]
            buy_O = bO; buy_H = bH; buy_F = bF
            buy_cost = bO*PRICE[0] + bH*PRICE[1] + bF*PRICE[2]
            s_counter += 1

        # 资源平衡
        prob += (res_vars[v+1][0] == res_vars[v][0] + buy_O - move_O
                 - (work_days*exp_wO if ftype[fr]==2 else 0))
        prob += (res_vars[v+1][1] == res_vars[v][1] + buy_H - move_H
                 - (work_days*exp_wH if ftype[fr]==2 else 0))
        prob += (res_vars[v+1][2] == res_vars[v][2] + buy_F - move_F
                 - (work_days*exp_wF if ftype[fr]==2 else 0))

        if ftype[fr] == 1:
            prob += res_vars[v+1][3] == res_vars[v][3] - buy_cost
        else:
            prob += res_vars[v+1][3] == res_vars[v][3]

        # 补给前非负性: 到达补给点时资源非负
        if ftype[fr+1] == 1 and d > 0:
            need_O = move_O + (work_days*exp_wO if ftype[fr]==2 else 0)
            need_H = move_H + (work_days*exp_wH if ftype[fr]==2 else 0)
            need_F = move_F + (work_days*exp_wF if ftype[fr]==2 else 0)
            prob += res_vars[v][0] >= need_O
            prob += res_vars[v][1] >= need_H
            prob += res_vars[v][2] >= need_F

        # 累积天数
        if ftype[fr] == 2:
            total_days_used_var = None  # 需要处理
            # 工作天数加到总天数

    # 总天数约束
    # 计算总天数 = sum(移动距离) + sum(工作天数)
    total_travel = sum(
        md(tuple(FP[skeleton[v]]), tuple(FP[skeleton[v+1]]))
        for v in range(m-1)
    )
    total_work = lpSum(w_vars) if w_vars else 0
    prob += total_travel + total_work <= rem_days

    # 最后一步必须到达E (skeleton最后是E)
    # 到达E时的资源必须 ≥ 0
    prob += res_vars[-1][0] >= 0; prob += res_vars[-1][1] >= 0
    prob += res_vars[-1][2] >= 0; prob += res_vars[-1][3] >= 0

    # 目标: max Z
    prob += Z_total - current_res['Z'], "Objective_Z"

    solver = PULP_CBC_CMD(msg=False, timeLimit=10, gapRel=0.01)
    prob.solve(solver)

    if LpStatus[prob.status] not in ('Optimal', 'Feasible'):
        return None

    z_final = int(round(value(Z_total)))
    m_final = int(round(value(res_vars[-1][3])))

    # Stage 2: fix Z, max M
    prob += Z_total >= z_final; prob += Z_total <= z_final
    prob.setObjective(res_vars[-1][3])
    prob.solve(solver)

    if LpStatus[prob.status] not in ('Optimal', 'Feasible'):
        return None

    z_final = int(round(value(Z_total)))
    m_final = int(round(value(res_vars[-1][3])))

    # 提取结果
    w_vals = [int(round(value(wv))) for wv in w_vars]
    buy_vals = []
    for bO,bH,bF,sk_idx,snum in buy_vars:
        buy_vals.append((int(round(value(bO))), int(round(value(bH))),
                         int(round(value(bF))), snum))

    return {
        'feasible': True,
        'skeleton': skeleton,
        'work_days': w_vals,
        'buy_amounts': buy_vals,
        'Z': z_final, 'M': m_final,
        'fp_names': [fname[i] for i in skeleton],
    }


# ========== Layer 2: 场景驱动贪婪导航 ==========
def greedy_simulate_one_scenario(strategic_plan, FP, ftype, fname,
                                 current_res, weather_seq, rem_days):
    """
    基于战略计划，在给定天气场景下进行贪婪模拟。
    返回: (final_Z, final_M, success, day_count)
    """
    pos = current_res['position']
    res = [current_res['Water'], current_res['Fuel'],
           current_res['Food'], current_res['M']]
    Z = current_res['Z']
    work_rem = list(strategic_plan['work_days'])  # 剩余工作天数
    buy_rem = list(strategic_plan['buy_amounts'])  # 剩余购买
    skeleton = strategic_plan['skeleton']

    # 当前目标索引
    target_idx = 1  # 骨架中下一个要去的点
    w_counter = 0
    s_counter = 0
    day = 0

    for day in range(rem_days):
        if pos == E:
            return Z, res[3], True, day + 1

        wc = weather_seq[day]  # 当天天气

        # 检查当前是什么类型的点
        cur_type = -1; cur_name = ''
        for i, fp in enumerate(FP):
            if tuple(fp) == pos:
                cur_type = ftype[i]; cur_name = fname[i]
                break

        # 决定行动
        action = None

        # 如果在作业点且有剩余工作天数
        if cur_type == 2 and w_counter < len(work_rem) and work_rem[w_counter] > 0:
            action = 'work'
            work_rem[w_counter] -= 1
            c = WORK_C[wc]
            res[0] -= c[0]; res[1] -= c[1]; res[2] -= c[2]
            wnum = int(cur_name[1]) - 1
            Z += W_YIELD[wnum]
        elif cur_type == 1 and s_counter < len(buy_rem):
            # 补给
            bo, bh, bf, snum = buy_rem[s_counter]
            cost = bo*PRICE[0]+bh*PRICE[1]+bf*PRICE[2]
            if cost <= res[3] and res[0]+res[1]+res[2]+bo+bh+bf <= CAP:
                res[0] += bo; res[1] += bh; res[2] += bf; res[3] -= cost
                buy_rem[s_counter] = (0, 0, 0, snum)  # 已使用
                s_counter += 1
            # 移动到下一个目标
            action = 'move'
        else:
            # 移动向目标
            action = 'move'

        if action == 'move':
            # 如果有目标，向目标移动
            if target_idx < len(skeleton):
                tgt = FP[skeleton[target_idx]]
                nx, ny = pos
                if nx < tgt[0]: nx += 1
                elif nx > tgt[0]: nx -= 1
                elif ny < tgt[1]: ny += 1
                elif ny > tgt[1]: ny -= 1
                pos = (nx, ny)
                c = MOVE_C[wc]
                res[0] -= c[0]; res[1] -= c[1]; res[2] -= c[2]
                if pos == tuple(tgt):
                    target_idx += 1
                    if ftype[skeleton[target_idx-1]] == 2:
                        w_counter += 1
            else:
                # 向E移动
                nx, ny = pos
                if nx < E[0]: nx += 1
                elif nx > E[0]: nx -= 1
                elif ny < E[1]: ny += 1
                elif ny > E[1]: ny -= 1
                pos = (nx, ny)
                c = MOVE_C[wc]
                res[0] -= c[0]; res[1] -= c[1]; res[2] -= c[2]

        # 检查合法性
        if res[0] < 0 or res[1] < 0 or res[2] < 0 or res[3] < 0:
            return Z, res[3], False, day + 1
        if res[0] + res[1] + res[2] > CAP:
            return Z, res[3], False, day + 1

    return Z, res[3], pos == E, day + 1


def scenario_driven_decision(strategic_plan, FP, ftype, fname,
                             current_state, today_w, scenarios, rem_days):
    """
    场景驱动的日决策：
    枚举候选行动 → 在每个场景中模拟 → 选期望Z最高的行动
    """
    pos = current_state['position']

    # 候选行动
    candidates = []
    # 移动
    for dx, dy, name in [(-1,0,'N'),(1,0,'S'),(0,-1,'W'),(0,1,'E')]:
        nx, ny = pos[0]+dx, pos[1]+dy
        if 1 <= nx <= 30 and 1 <= ny <= 30:
            candidates.append(('move', (nx, ny), name))
    candidates.append(('idle', pos, 'I'))

    # 工作
    cur_type = -1; cur_name = ''
    for i, fp in enumerate(FP):
        if tuple(fp) == pos:
            cur_type = ftype[i]; cur_name = fname[i]
            break
    if cur_type == 2:
        candidates.append(('work', pos, 'W'))

    # 补给
    if cur_type == 1:
        candidates.append(('buy', pos, 'B'))

    best_act = None; best_EZ = -99999; best_EM = -99999

    for act_type, new_pos, label in candidates:
        total_Z = 0; total_M = 0; succ = 0
        for sc in scenarios:
            # 模拟: 执行行动 → 后续贪婪
            cs = dict(current_state)
            wc = today_w  # Day 1天气相同

            if act_type == 'move':
                c = MOVE_C[wc]
                cs['Water'] -= c[0]; cs['Fuel'] -= c[1]; cs['Food'] -= c[2]
                cs['position'] = new_pos
            elif act_type == 'idle':
                c = IDLE_C[wc]
                cs['Water'] -= c[0]; cs['Fuel'] -= c[1]; cs['Food'] -= c[2]
            elif act_type == 'work':
                c = WORK_C[wc]
                cs['Water'] -= c[0]; cs['Fuel'] -= c[1]; cs['Food'] -= c[2]
                wnum = int(cur_name[1]) - 1
                cs['Z'] += W_YIELD[wnum]
            elif act_type == 'buy':
                # 简单购买策略
                sp = CAP - (cs['Water']+cs['Fuel']+cs['Food'])
                need = max(0, 30*7 - (cs['Water']+cs['Fuel']+cs['Food']))
                amt = min(need, sp)
                bo = min(amt*2//7, sp, cs['M']//2)
                bh = min(amt*3//7, sp-bo, cs['M'])
                bf = min(amt*2//7, sp-bo-bh, cs['M']//2)
                bo = max(0, bo); bh = max(0, bh); bf = max(0, bf)
                cs['Water'] += bo; cs['Fuel'] += bh; cs['Food'] += bf
                cs['M'] -= bo*PRICE[0] + bh*PRICE[1] + bf*PRICE[2]

            # 检查合法性
            if (cs['Water'] < 0 or cs['Fuel'] < 0 or cs['Food'] < 0 or cs['M'] < 0
                or cs['Water']+cs['Fuel']+cs['Food'] > CAP):
                total_Z -= 10000; continue

            # 贪婪继续模拟
            fz, fm, ok, _ = greedy_simulate_one_scenario(
                strategic_plan, FP, ftype, fname, cs, sc[1:], rem_days-1)
            total_Z += fz
            total_M += fm
            if ok: succ += 1

        avg_Z = total_Z / len(scenarios)
        avg_M = total_M / len(scenarios)

        if avg_Z > best_EZ or (avg_Z == best_EZ and avg_M > best_EM):
            best_EZ = avg_Z; best_EM = avg_M
            best_act = (act_type, new_pos, label)

    return best_act, best_EZ


# ========== 完整模拟 ==========
def simulate_full(actual_weather, scenario_seed=42):
    state = {
        'position': (1, 15), 'Water': INIT_O, 'Fuel': INIT_H,
        'Food': INIT_F, 'M': INIT_M, 'Z': INIT_Z,
        'workPoint': 0, 'consecutiveDays': 0, 'arrived': False
    }
    route = [(1, 15)]
    schedule = []
    strat_plan = None
    solve_time = 0.0
    rng = random.Random(scenario_seed)

    for t in range(T_DAYS):
        if state['arrived']: break
        rem = T_DAYS - t
        today_w = actual_weather[t]

        # 每5天或在关键点重新规划战略
        need_replan = (strat_plan is None or t % 8 == 0 or
                       state['position'] in S + W or
                       state['position'] == B)

        if need_replan:
            t0 = time.time()
            strat_plan = solve_strategic_milp(
                state['position'],
                {'Water': state['Water'], 'Fuel': state['Fuel'],
                 'Food': state['Food'], 'M': state['M'], 'Z': state['Z']},
                today_w, rem)
            solve_time += time.time() - t0

        # 如果没有可行计划，尝试停泊
        if strat_plan is None:
            c = IDLE_C[today_w]
            state['Water'] -= c[0]; state['Fuel'] -= c[1]; state['Food'] -= c[2]
            route.append(state['position'])
            schedule.append({'Day':t+1,'Weather':'Storm' if today_w else 'Normal',
                           'Start':str(state['position']),'Action':'idle(fallback)',
                           'End':str(state['position']),
                           'BuyO':0,'BuyH':0,'BuyF':0,'Gain':0,
                           'O':state['Water'],'H':state['Fuel'],'F':state['Food'],
                           'M':state['M'],'Z':state['Z']})
            continue

        # 建立功能点信息
        all_func = [state['position']] + S + W + [E]
        dedup = []; seen = set()
        for p in all_func:
            tp = tuple(p)
            if tp not in seen: seen.add(tp); dedup.append(p)
        FP = dedup
        ftype = []; fname = []
        for p in FP:
            tp = tuple(p)
            if tp == tuple(state['position']): ftype.append(0); fname.append('CUR')
            elif tp == E: ftype.append(3); fname.append('E')
            elif tp in [tuple(s) for s in S]:
                idx = [tuple(s) for s in S].index(tp)
                ftype.append(1); fname.append(f'S{idx+1}')
            elif tp in [tuple(w) for w in W]:
                idx = [tuple(w) for w in W].index(tp)
                ftype.append(2); fname.append(f'W{idx+1}')
            else: ftype.append(0); fname.append('?')

        # 生成场景
        S_NUM = 30
        scs = gen_scenarios(S_NUM, rem, rng.randint(0, 10**9))

        # 场景驱动决策
        old_state = dict(state)
        (act_type, new_pos, label), ez = scenario_driven_decision(
            strat_plan, FP, ftype, fname, state, today_w, scs, rem)

        # 执行行动
        gain = 0; buyO = 0; buyH = 0; buyF = 0
        if act_type == 'move':
            c = MOVE_C[today_w]
            state['Water'] -= c[0]; state['Fuel'] -= c[1]; state['Food'] -= c[2]
            state['position'] = new_pos
            action_str = f"->({new_pos[0]},{new_pos[1]})"
        elif act_type == 'idle':
            c = IDLE_C[today_w]
            state['Water'] -= c[0]; state['Fuel'] -= c[1]; state['Food'] -= c[2]
            action_str = "idle"
        elif act_type == 'work':
            c = WORK_C[today_w]
            state['Water'] -= c[0]; state['Fuel'] -= c[1]; state['Food'] -= c[2]
            ctype = -1; cname = ''
            for i, fp in enumerate(FP):
                if tuple(fp) == state['position']: ctype=ftype[i]; cname=fname[i]; break
            wnum = int(cname[1]) - 1
            gain = W_YIELD[wnum]; state['Z'] += gain
            action_str = f"work {cname}"
        elif act_type == 'buy':
            sp = CAP - (state['Water']+state['Fuel']+state['Food'])
            nd = max(0, 30*7 - (state['Water']+state['Fuel']+state['Food']))
            am = min(nd, sp)
            buyO = min(am*2//7, sp, state['M']//2)
            buyH = min(am*3//7, sp-buyO, state['M'])
            buyF = min(am*2//7, sp-buyO-buyH, state['M']//2)
            buyO=max(0,buyO); buyH=max(0,buyH); buyF=max(0,buyF)
            state['Water']+=buyO; state['Fuel']+=buyH; state['Food']+=buyF
            state['M']-=buyO*PRICE[0]+buyH*PRICE[1]+buyF*PRICE[2]
            action_str = f"buy(O={buyO},H={buyH},F={buyF})"

        route.append(state['position'])
        if state['position'] == E:
            state['arrived'] = True

        schedule.append({'Day':t+1,'Weather':'Storm' if today_w else 'Normal',
                       'Start':str(old_state['position']),
                       'Action':action_str,
                       'End':str(state['position']),
                       'BuyO':buyO,'BuyH':buyH,'BuyF':buyF,
                       'Gain':gain,
                       'O':state['Water'],'H':state['Fuel'],'F':state['Food'],
                       'M':state['M'],'Z':state['Z']})

        if t % 5 == 0 or state['arrived']:
            print(f"  Day{t+1}: pos={state['position']} Z={state['Z']} "
                  f"M={state['M']} O={state['Water']} H={state['Fuel']} "
                  f"F={state['Food']}")

    ad = len(schedule) if state['arrived'] else 0
    return {
        'feasible': state['arrived'],
        'arrival_day': ad,
        'teamZ': state['Z'], 'teamM': state['M'],
        'route': route, 'schedule': schedule,
        'solve_time': solve_time
    }


# ========== MC验证 ==========
def mc_validate(schedule, n=1000, seed=9999):
    rng = random.Random(seed); succ = 0
    for _ in range(n):
        rs = [100,150,100]; mn = INIT_M; ok = True
        for row in schedule:
            rs[0] += row['BuyO']; rs[1] += row['BuyH']; rs[2] += row['BuyF']
            mn -= row['BuyO']*PRICE[0]+row['BuyH']*PRICE[1]+row['BuyF']*PRICE[2]
            wc = 1 if row['Weather']=='Storm' else 0
            act = row['Action']
            if '->' in act: c = MOVE_C[wc]
            elif 'work' in act: c = WORK_C[wc]
            elif 'buy' in act: c = IDLE_C[wc]
            else: c = IDLE_C[wc]
            rs[0] -= c[0]; rs[1] -= c[1]; rs[2] -= c[2]
            if rs[0]<-1e-6 or rs[1]<-1e-6 or rs[2]<-1e-6 or mn<-1e-6 or sum(rs)>CAP:
                ok = False; break
        if ok: succ += 1
    rate = succ / n
    z = 1.96; p = rate; den = 1 + z**2/n
    lo = max(0,(p+z**2/(2*n)-z*math.sqrt(p*(1-p)/n+z**2/(4*n**2)))/den)
    return {'samples':n,'successes':succ,'rate':rate,'lower95':lo}


# ========== 主程序 ==========
def main():
    print("="*60)
    print("Task 3: Scenario Generation + Stochastic MILP")
    print("="*60)

    seed = 2026
    rng = random.Random(seed); aw = [0 if rng.random()<PN else 1 for _ in range(T_DAYS)]
    ns = sum(aw)
    print(f"Weather: {T_DAYS}d, Normal={T_DAYS-ns}, Storm={ns}")

    print("\nSimulating...")
    t0 = time.time()
    res = simulate_full(aw, scenario_seed=42)
    el = time.time() - t0

    print(f"\nDone: {el:.1f}s, MILP solve={res['solve_time']:.1f}s")
    print(f"Feasible: {res['feasible']}, Arrival: Day {res['arrival_day']}")
    print(f"Final Z={res['teamZ']}, M={res['teamM']}")

    if res['feasible']:
        print("\nMC Validation (1000)...")
        mc = mc_validate(res['schedule'], n=1000, seed=9999)
        print(f"Success rate: {mc['rate']:.4f}, 95% lower: {mc['lower95']:.4f}")

    # Save
    outdir = os.path.dirname(os.path.abspath(__file__))

    jp = os.path.join(outdir, "task3_stochastic_milp_result.json")
    jo = {k:res[k] for k in ['feasible','arrival_day','teamZ','teamM','solve_time','route','schedule']}
    jo['total_elapsed'] = el
    with open(jp, 'w', encoding='utf-8') as f:
        json.dump(jo, f, ensure_ascii=False, indent=2, default=str)
    print(f"\nJSON saved: {jp}")

    cp = os.path.join(outdir, "task3_stochastic_milp_result.csv")
    if res['schedule']:
        with open(cp, 'w', newline='', encoding='utf-8-sig') as f:
            w = csv.writer(f)
            w.writerow(['Day','Weather','Start','Action','End',
                       'BuyO','BuyH','BuyF','Gain',
                       'O','H','F','M','Z'])
            for row in res['schedule']:
                w.writerow([row['Day'],row['Weather'],row['Start'],row['Action'],
                           row['End'],row['BuyO'],row['BuyH'],row['BuyF'],
                           row['Gain'],row['O'],row['H'],row['F'],row['M'],row['Z']])
        print(f"CSV saved: {cp}")

    return res

if __name__=="__main__":
    main()
