% DEPRECATED: Use 优化版/solve_q3_online_opt.m instead
% This file is kept for reference only.

function solve_q3_online(weather_seq)
% =========================================================================
%  solve_q3_online.m — 任务3 在线随机天气决策模型
%  =========================================================================
%  功能:  每日观测实际天气后，以期望消耗参数运行CP搜索，执行计划首动作。
%         消耗双轨制: 规划期用期望值, 执行期用实际天气消耗。
%
%  用法:
%    solve_q3_online()             运行一次，天气随机生成
%    solve_q3_online(weather_seq)   使用指定的天气序列 (1×90 char, 'N'/'T')
%
%  决策架构 (滚动时域CP重规划):
%    for t = 1:90:
%      观测 w_t (P(正常)=0.8)
%      若需重规划 (首日/天气变化/每5天):
%        CP搜索: 从 S_t 出发, 消耗=E[消耗]
%        预展开为逐日动作序列
%      执行 a_t^* (消耗使用 w_t 对应的**实际**消耗参数)
%      状态转移 S_{t+1} = T(S_t, a_t^*, w_t)
%  =========================================================================

if nargin < 1 || isempty(weather_seq)
    rng('shuffle');
    weather_seq = generate_weather(90, 0.8);
    fprintf('天气序列已随机生成 (P(正常)=0.8)。\n');
else
    fprintf('使用指定的天气序列。\n');
end

% ===== Task 3 配置 =====
MAX_DAYS = 90; MAX_LOAD = 400;
all_xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
names   = {'B','E','W1','W2','W3','S1','S2'};
WY = [20, 15, 28]; WM = [4, 5, 3];
INIT_O = 100; INIT_H = 150; INIT_F = 100;
INIT_M = 750; INIT_Z = 200;

% 期望消耗 (规划用)
% 期望消耗 + 10%安全裕度 (保守规划以应对天气波动)
SAFETY = 1.10;
cons_exp = struct('MO',3.2*SAFETY,'MH',3.2*SAFETY,'MF',2.2*SAFETY, ...
                  'PO',1.4*SAFETY,'PH',1.4*SAFETY,'PF',1.2*SAFETY, ...
                  'WO',5.6*SAFETY,'WH',4.4*SAFETY,'WF',3.6*SAFETY, ...
                  'pO',2,'pH',1,'pF',2);
% 实际消耗: 正常天气
cons_N = struct('MO',2,'MH',3,'MF',2, ...
                'PO',1,'PH',1,'PF',1, ...
                'WO',5,'WH',4,'WF',3, ...
                'pO',2,'pH',1,'pF',2);
% 实际消耗: 雷暴天气
cons_T = struct('MO',8,'MH',4,'MF',3, ...
                'PO',3,'PH',3,'PF',2, ...
                'WO',8,'WH',6,'WF',6, ...
                'pO',2,'pH',1,'pF',2);

% ===== 距离矩阵 =====
n_pts = size(all_xy, 1);
dist = zeros(n_pts);
for i = 1:n_pts, for j = 1:n_pts
    dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
end; end
inter_idx = [3 4 5 6 7];

% ===== 初始化状态 =====
state = struct();
state.pt = 1;           % 当前点位 (1=B)
state.pos = all_xy(1,:);
state.O = INIT_O; state.H = INIT_H; state.F = INIT_F;
state.M = INIT_M; state.Z = INIT_Z;
state.consec = 0;        % 连续作业天数
state.day = 0;           % 当前天数 (0表示尚未开始)

% 计划缓存
plan_path = []; plan_parks = []; plan_works = [];
plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
wp_idx = 0; wd_done = 0;
prev_weather = ''; replan_count = 0;

% ===== 输出头 =====
fprintf('========================================\n');
fprintf('  任务3 在线随机天气决策模型\n');
fprintf('========================================\n');
fprintf('策略: 滚动时域CP重规划 | 消耗: 规划期望/执行实际\n');
fprintf('重规划触发: 首日 | 天气变化 | 每5天强制\n\n');

fprintf('--- 逐日在线决策 ---\n');
fprintf('Day  | W | Action              | Pos (x,y)   |   O    H    F   Load |    Z       M\n');
fprintf('-----|---|---------------------|--------------|---------------------|--------------\n');

% ===== 在线决策主循环 =====
while state.day < MAX_DAYS && state.pt ~= 2
    state.day = state.day + 1;
    w = weather_seq(state.day);

    % 选择当日实际消耗参数
    if w == 'T', cons_act = cons_T; wname = 'T';
    else,        cons_act = cons_N; wname = 'N';
    end

    % ---- 重规划触发 ----
    % 条件: 位于命名点 + 非移动中途 + 非作业点(作业点执行原计划)
    at_named_point = (step_in_leg == 0 && parked_in_leg == 0);
    is_work_pt_now = (state.pt >= 3 && state.pt <= 5);
    not_at_work_pt = ~is_work_pt_now;
    need_replan = isempty(plan_path) || ...
                  (state.day > 1 && w ~= weather_seq(max(1, state.day-1)) && at_named_point && not_at_work_pt) || ...
                  (mod(state.day, 3) == 1 && state.day > 1 && at_named_point && not_at_work_pt);

    if need_replan
        replan_count = replan_count + 1;
        elapsed = state.day - 1;
        init_s = struct('O',state.O,'H',state.H,'F',state.F,'M',state.M,'Z',state.Z);
        [plan_path, plan_parks, plan_works, feasible] = ...
            cp_plan_from_state(state.pt, elapsed, cons_exp, dist, inter_idx, ...
                               WY, WM, MAX_DAYS, MAX_LOAD, init_s);
        if ~feasible
            % 兜底策略：期望消耗不可行时，尝试乐观（正常天气消耗）规划
            fprintf('%4d | %s | (exp infeasible, trying normal...) |              |                     | %5d %8.0f\n', ...
                state.day, wname, state.Z, round(state.M));
            [plan_path, plan_parks, plan_works, feasible] = ...
                cp_plan_from_state(state.pt, elapsed, cons_N, dist, inter_idx, ...
                                   WY, WM, MAX_DAYS, MAX_LOAD, init_s);
        end
        if ~feasible
            fprintf('%4d | %s | NO FEASIBLE PLAN    |              |                     | %5d %8.0f\n', ...
                state.day, wname, state.Z, round(state.M));
            break;
        end
        plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
        wp_idx = 0; wd_done = 0;
    end

    if length(plan_path) < 2, break; end
    next_pt = plan_path(plan_leg + 1);

    % ---- 当日动作决策 ----
    is_work_pt = (state.pt >= 3 && state.pt <= 5);
    % 仅当plan中当前作业点匹配实际位置时才允许作业
    wp_match = true;
    if is_work_pt && wp_idx < length(plan_works)
        % 检查plan中第wp_idx+1个作业点是否匹配当前位置
        wa_local = []; for ii=2:length(plan_path)
            if plan_path(ii)>=3 && plan_path(ii)<=5, wa_local(end+1)=ii; end
        end
        if wp_idx+1 <= length(wa_local)
            wp_match = (plan_path(wa_local(wp_idx+1)) == state.pt);
        end
    end
    need_work  = is_work_pt && wp_match && wp_idx < length(plan_works) && wd_done < plan_works(wp_idx+1);

    act = ''; detail = '';

    if need_work
        wk_type = state.pt - 2;  % W1→1, W2→2, W3→3
        wm_val = WM(wk_type); yld = WY(wk_type);
        if state.consec < wm_val
            act = sprintf('work(%s)', names{state.pt});
            state.O = state.O - cons_act.WO; state.H = state.H - cons_act.WH;
            state.F = state.F - cons_act.WF; state.Z = state.Z + yld;
            state.consec = state.consec + 1; wd_done = wd_done + 1;
            if wd_done >= plan_works(wp_idx+1)
                wp_idx = wp_idx + 1; wd_done = 0;
            end
        else
            act = 'park(reset)';
            state.O = state.O - cons_act.PO; state.H = state.H - cons_act.PH;
            state.F = state.F - cons_act.PF; state.consec = 0;
        end
    elseif plan_leg <= length(plan_parks) && parked_in_leg < plan_parks(plan_leg)
        act = 'park(at sea)';
        state.O = state.O - cons_act.PO; state.H = state.H - cons_act.PH;
        state.F = state.F - cons_act.PF; state.consec = 0;
        parked_in_leg = parked_in_leg + 1;
    elseif step_in_leg < dist(state.pt, next_pt)
        state.O = state.O - cons_act.MO; state.H = state.H - cons_act.MH;
        state.F = state.F - cons_act.MF; state.consec = 0;
        step_in_leg = step_in_leg + 1;

        % 更新曼哈顿位置
        fr = all_xy(state.pt, :); to = all_xy(next_pt, :);
        dx_tot = to(1) - fr(1); dy_tot = to(2) - fr(2);
        sx = abs(dx_tot); sy = abs(dy_tot);
        if step_in_leg <= sx
            state.pos(1) = fr(1) + sign(dx_tot) * step_in_leg;
            state.pos(2) = fr(2);
        else
            state.pos(1) = to(1);
            state.pos(2) = fr(2) + sign(dy_tot) * (step_in_leg - sx);
        end
        act = sprintf('move -> (%d,%d)', state.pos(1), state.pos(2));

        % 到达下一目标点
        if step_in_leg >= dist(state.pt, next_pt)
            state.pt = next_pt; state.pos = all_xy(next_pt, :);
            step_in_leg = 0; parked_in_leg = 0;

            % 更新工作指针
            if state.pt >= 3 && state.pt <= 5
                wc = 0;
                for i = 2:plan_leg+1
                    if plan_path(i) >= 3 && plan_path(i) <= 5, wc = wc + 1; end
                end
                wp_idx = wc - 1; wd_done = 0;
            end

            % 到达补给点 → 触发采购
            if state.pt == 6 || state.pt == 7
                rem_t = 0; rem_p = 0;
                for k = plan_leg:length(plan_path)-2
                    rem_t = rem_t + dist(plan_path(k+1), plan_path(k+2));
                    if k+1 <= length(plan_parks), rem_p = rem_p + plan_parks(k+1); end
                end
                % 使用期望消耗计算需求 (与实际消耗无关)
                needO = rem_t * cons_exp.MO + rem_p * cons_exp.PO;
                needH = rem_t * cons_exp.MH + rem_p * cons_exp.PH;
                needF = rem_t * cons_exp.MF + rem_p * cons_exp.PF;
                sp = MAX_LOAD - (state.O + state.H + state.F);
                bO = max(0, needO - state.O); bH = max(0, needH - state.H);
                bF = max(0, needF - state.F);
                if bO + bH + bF <= sp + 1e-6
                    cost = bO * cons_act.pO + bH * cons_act.pH + bF * cons_act.pF;
                    if cost <= state.M + 1e-6
                        state.O = state.O + bO; state.H = state.H + bH;
                        state.F = state.F + bF; state.M = state.M - cost;
                        act = sprintf('SUPPLY(%s)', names{state.pt});
                        detail = sprintf('+O%.0f H%.0f F%.0f cost=%.0f', bO, bH, bF, cost);
                    else
                        act = 'SUPPLY FAIL (M)';
                    end
                else
                    act = 'SUPPLY FAIL (load)';
                end
            elseif state.pt == 2
                act = 'ARRIVE at E!';
            end
            plan_leg = plan_leg + 1;
        end
    else
        act = 'IDLE';
    end

    % 检查失败条件
    if state.O < -1e-6 || state.H < -1e-6 || state.F < -1e-6
        act = 'RESOURCE EXHAUSTED!';
        fprintf('%4d | %s | %-20s | (%2d,%2d)      | %4.0f %4.0f %4.0f %5.0f | %5d %8.0f  *** 失败 ***\n', ...
            state.day, wname, act, state.pos(1), state.pos(2), ...
            state.O, state.H, state.F, state.O+state.H+state.F, state.Z, round(state.M));
        break;
    end
    if state.O + state.H + state.F > MAX_LOAD + 1e-6
        act = 'OVERLOAD!';
        fprintf('%4d | %s | %-20s | (%2d,%2d)      | %4.0f %4.0f %4.0f %5.0f | %5d %8.0f  *** 失败 ***\n', ...
            state.day, wname, act, state.pos(1), state.pos(2), ...
            state.O, state.H, state.F, state.O+state.H+state.F, state.Z, round(state.M));
        break;
    end

    % 输出当日状态 (每10天或关键事件)
    is_key = need_replan || contains(act, 'SUPPLY') || contains(act, 'work') || ...
             contains(act, 'ARRIVE') || contains(act, 'FAIL') || mod(state.day, 10) == 1;
    if is_key
        fprintf('%4d | %s | %-20s | (%2d,%2d)      | %4.0f %4.0f %4.0f %5.0f | %5d %8.0f', ...
            state.day, wname, act, state.pos(1), state.pos(2), ...
            state.O, state.H, state.F, state.O+state.H+state.F, state.Z, round(state.M));
        if ~isempty(detail), fprintf('  %s', detail); end
        if need_replan, fprintf('  [重规划#%d]', replan_count); end
        fprintf('\n');
    end

    prev_weather = w;
end

% ===== 最终结果 =====
fprintf('-----|---|---------------------|--------------|---------------------|--------------\n');
fprintf('\n===== 最终结果 =====\n');
if state.pt == 2
    fprintf('状态: 第 %d 天成功抵达 E\n', state.day);
    fprintf('Z = %d  M = %.0f\n', state.Z, round(state.M));
    fprintf('重规划次数: %d\n', replan_count);
elseif state.day >= MAX_DAYS
    fprintf('状态: 超时未抵达 E (已用 %d 天)\n', MAX_DAYS);
    fprintf('最终位置: (%d,%d)  Z = %d  M = %.0f\n', state.pos(1), state.pos(2), state.Z, round(state.M));
else
    fprintf('状态: 第 %d 天失败 (资源耗尽/超载)\n', state.day);
    fprintf('最终位置: (%d,%d)  Z = %d  M = %.0f\n', state.pos(1), state.pos(2), state.Z, round(state.M));
end
fprintf('\nDone.\n');
end

% =====================================================================
%  CP规划函数 (从当前状态出发)
% =====================================================================
function [best_path, best_parks, best_works, feasible] = ...
    cp_plan_from_state(cur_pt, elapsed, cons, dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, init_s)
% 从当前状态 cur_pt 出发, 以期望消耗 cons 规划剩余路径
% elapsed: 已经过的天数

    best_Z = -inf; best_M = -inf;
    best_path = [cur_pt, 2]; best_works = []; best_parks = [];
    nodes = 0;
    eff_days = MAX_DAYS - elapsed;

    [best_Z, best_M, best_path, best_works, best_parks, nodes] = ...
        cp_search_online([cur_pt], 0, [], [], best_Z, best_M, best_path, best_works, best_parks, ...
        dist, inter_idx, WY, WM, cons, nodes, eff_days, MAX_LOAD, init_s);

    feasible = (best_Z > -inf);
end

% =====================================================================
%  CP搜索 (在线版, 与solve_q3_cp.m一致但使用eff_days)
% =====================================================================
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online(path, tsf, wa, ww, ...
    bZ, bM, bP, bWD, bPS, dist, inter_idx, WY, WM, cons, nodes, MAX_DAYS_EFF, MAX_LOAD, init_s)

    nodes = nodes + 1; lp = path(end);

    % 上界剪枝
    if lp ~= 2
        dE = dist(lp, 2); rem_total = MAX_DAYS_EFF - tsf;
        if rem_total < dE, return; end
        ub_work = max_work_with_park_q3(3, rem_total - dE);
        ub_Z = init_s.Z + ub_work * 28;
        if ub_Z <= bZ && bZ > -inf, return; end
    end

    % 叶节点评估
    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS_EFF
        fp = [path, 2]; m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1), tt = tt + dist(fp(k), fp(k+1)); end
        rem_days = MAX_DAYS_EFF - tt; nw = length(wa);

        if nw == 0
            [ok, Z, M] = simulate_q3_online(fp, m, dist, [], [], tt, [], [], ...
                cons, MAX_LOAD, init_s);
            if ok && (Z > bZ || (Z == bZ && M > bM))
                bZ = Z; bM = M; bP = fp; bWD = []; bPS = [];
            end
        else
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = max_work_with_park_q3(WM(ww(j)), rem_days);
            end
            sz = max_wk + 1; nc = prod(sz);
            for ci = 1:nc
                wd = zeros(1, nw); t2 = ci - 1;
                for j = nw:-1:1
                    wd(j) = mod(t2, sz(j)); t2 = floor(t2 / sz(j));
                end
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > MAX_DAYS_EFF, continue; end
                [ok, Z, M] = simulate_q3_online(fp, m, dist, wa, ww, tt, wd, [], ...
                    cons, MAX_LOAD, init_s);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bWD = wd; bPS = [];
                end
            end
        end
    end

    % 分支递归
    for ni = 1:5
        np = inter_idx(ni);
        if np == lp, continue; end
        d = dist(lp, np);
        if tsf + d > MAX_DAYS_EFF, continue; end
        dE2 = dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS_EFF, continue; end
        np2 = [path, np]; nt = tsf + d;
        nwa = wa; nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2); nww(end+1) = np - 2;
        end
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online(np2, nt, nwa, nww, ...
            bZ, bM, bP, bWD, bPS, dist, inter_idx, WY, WM, cons, nodes, MAX_DAYS_EFF, MAX_LOAD, init_s);
    end
end

function max_w = max_work_with_park_q3(mc, remaining)
    best = 0;
    for k = 1:(remaining + 1)
        stay = k * mc + (k - 1);
        if stay > remaining, break; end
        best = k * mc;
        slack = remaining - stay;
        if slack >= 1, best = max(best, k * mc + min(mc, slack - 1)); end
    end
    max_w = max(best, min(mc, remaining));
end

function [feasible, Zf, Mf] = simulate_q3_online(pid, m, dist_all, wa, ww, tt, wdays, park_seg, ...
    cons, MAX_LOAD, init_s)
% 期望消耗模拟 (与solve_q3_cp.m中的simulate_q3一致)
    if isempty(wdays), wdays = []; end
    if isempty(park_seg), park_seg = zeros(1, m+1); end

    WM_local = [4, 5, 3]; WY_local = [20, 15, 28];
    T_alloc = tt + sum(wdays) + sum(park_seg) + 100;
    cO = zeros(1, T_alloc); cH = zeros(1, T_alloc); cF = zeros(1, T_alloc);
    zG = zeros(1, T_alloc); isSup = false(1, T_alloc);

    day = 0;
    for k = 1:(m+1)
        d = dist_all(pid(k), pid(k+1));
        for pd = 1:park_seg(k)
            day = day + 1; cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
        end
        for dd = 1:d
            day = day + 1; cO(day) = cons.MO; cH(day) = cons.MH; cF(day) = cons.MF;
            if dd == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7, isSup(day) = true; end
            end
        end
        if ~isempty(wa)
            wk = find(wa == k+1, 1);
            if ~isempty(wk) && ~isempty(wdays) && wk <= length(wdays) && wdays(wk) > 0
                mc = WM_local(ww(wk)); yld = WY_local(ww(wk)); rem_val = wdays(wk);
                while rem_val > 0
                    chunk = min(rem_val, mc);
                    for w = 1:chunk
                        day = day + 1;
                        cO(day) = cons.WO; cH(day) = cons.WH; cF(day) = cons.WF;
                        zG(day) = yld;
                    end
                    rem_val = rem_val - chunk;
                    if rem_val > 0
                        day = day + 1;
                        cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
                    end
                end
            end
        end
    end
    T_actual = day;

    O = init_s.O; H = init_s.H; F = init_s.F;
    M = init_s.M; Zf = init_s.Z;

    for t = 1:T_actual
        O = O - cO(t); H = H - cH(t); F = F - cF(t); Zf = Zf + zG(t);
        if O < -1e-6 || H < -1e-6 || F < -1e-6
            feasible = false; Mf = 0; return;
        end
        if O + H + F > MAX_LOAD + 1e-6
            feasible = false; Mf = 0; return;
        end
        if isSup(t)
            ns = T_actual + 1;
            for tt2 = (t+1):T_actual
                if isSup(tt2), ns = tt2; break; end
            end
            nO = 0; nH = 0; nF = 0;
            for tt2 = (t+1):ns
                if tt2 > T_actual, break; end
                nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
            end
            sp = MAX_LOAD - (O + H + F);
            bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);
            if bO + bH + bF > sp + 1e-6
                feasible = false; Mf = 0; return;
            end
            if ns > T_actual
                if O + bO < nO - 1e-6 || H + bH < nH - 1e-6 || F + bF < nF - 1e-6
                    feasible = false; Mf = 0; return;
                end
            end
            cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
            if cost > M + 1e-6
                feasible = false; Mf = 0; return;
            end
            O = O + bO; H = H + bH; F = F + bF; M = M - cost;
        end
    end
    feasible = true; Mf = M;
end

% =====================================================================
%  生成随机天气序列
% =====================================================================
function wseq = generate_weather(n_days, p_normal)
    wseq = repmat('N', 1, n_days);
    for i = 1:n_days
        if rand() > p_normal
            wseq(i) = 'T';
        end
    end
end
