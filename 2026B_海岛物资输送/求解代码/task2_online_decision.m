function task2_online_decision()
% =========================================================================
%  任务2 — 在线约束规划决策模型（独立版）
% =========================================================================
%
%  【数学模型】
%
%  1. 状态空间
%     时刻 t（观测天气后、决策动作前）船舶状态：
%
%       S_t = ( pt, O_t, H_t, F_t, M_t, Z_t, c_t, w_t, d_t )
%
%       pt   ∈ {1..7}     当前所在点位 (1=B,2=E,3=W1,4=W2,5=W3,6=S1,7=S2)
%       O_t  ∈ Z_+         燃油余量
%       H_t  ∈ Z_+         淡水余量
%       F_t  ∈ Z_+         食物余量
%       M_t  ∈ Z_+         剩余资金
%       Z_t  ∈ Z_+         目标物资存量
%       c_t  ∈ [0,WM_i]    连续作业天数 (仅在作业点有效)
%       w_t  ∈ {N, T}      当日天气 (N=正常, T=雷暴)
%       d_t  ∈ {1..30}     当前天数
%
%  2. 动作空间
%
%       A(S_t) = { ↑, ↓, ←, →, 停泊, 作业, 补给 }
%
%     动作可行性受四重约束过滤：
%       • 移动：需 O_t ≥ c_O(w_t), H_t ≥ c_H(w_t), F_t ≥ c_F(w_t), d_t < 30
%       • 停泊：无条件（任何网格可执行）
%       • 作业：仅在 W1/W2/W3 且 c_t < WM_i
%       • 补给：仅在 S1/S2
%
%  3. 消耗参数
%
%      | 动作     | 正常天气 (O,H,F) | 雷暴天气 (O,H,F) |
%      |----------|:---:|:---:|
%      | 移动(每格) | (2,3,2) | (8,4,3) |
%      | 停泊(每天) | (1,1,1) | (3,3,2) |
%      | 作业(每天) | (5,4,3) | (8,6,6) |
%
%     补给价格不变：p_O=2, p_H=1, p_F=2
%
%  4. 决策函数（每日调用一次）
%
%       a_t* = argmax_{a ∈ A(S_t)}  V( T(S_t, a, w_t) )
%
%     其中 V 是 CP 前向搜索估计的词典序最优值 (先 max Z, 再 max M)。
%     CP 搜索假设"剩余天数天气 ≡ 今日观测"。
%
%  5. 状态转移
%
%       S_{t+1} = T(S_t, a_t*, w_t)
%
%     转移规则按动作类型查消耗表扣除资源，作业累加 Z 和 c_t，
%     停泊清零 c_t，补给日触发采购算法。
%
%  6. 在线决策架构（逐日循环）
%
%       for t = 1:30:
%           w_t ← 观测当日天气
%           a_t* ← CP_Search(S_t, assume(w_{t+1:30}=w_t))
%           执行 a_t*, 更新 S_{t+1}
%           if pt == E: 终止
%
%  【代码实现】
%  依赖：cp_common.m（提供 simulate, max_work_with_park, enumerate_park_combs）
% =========================================================================

MAX_DAYS = 30; MAX_LOAD = 120;
all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
names  = {'B','E','W1','W2','W3','S1','S2'};
WY = [20, 15, 28];  WM = [4, 5, 3];
INIT_O=35; INIT_H=45; INIT_F=30; INIT_M=240; INIT_Z=100;

% 距离矩阵
dist = zeros(7);
for i=1:7, for j=1:7, dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2)); end; end
inter_idx = [3 4 5 6 7];

% 消耗参数表
cons_N = struct('MO',2,'MH',3,'MF',2,'PO',1,'PH',1,'PF',1,'WO',5,'WH',4,'WF',3,'pO',2,'pH',1,'pF',2);
cons_T = struct('MO',8,'MH',4,'MF',3,'PO',3,'PH',3,'PF',2,'WO',8,'WH',6,'WF',6,'pO',2,'pH',1,'pF',2);

% ===== 天气序列（修改此行切换场景）=====
weather_mode = 'all_thunder';  % 'all_normal' | 'all_normal' | 'mixed'
switch weather_mode
    case 'all_normal', weather_seq = repmat('T', 1, MAX_DAYS);
    case 'all_normal',  weather_seq = repmat('N', 1, MAX_DAYS);
    case 'mixed',       weather_seq = [repmat('T',1,1), repmat('N',1,MAX_DAYS-1)];
end

fprintf('========================================\n');
fprintf('  任务2 在线CP决策模型（独立版）\n');
fprintf('========================================\n');
fprintf('天气模式: %s\n', weather_mode);
fprintf('模型: S_t = (pt, O,H,F, M,Z, c, w_t, d_t)\n');
fprintf('循环: 观测w_t → CP搜索 → 执行a_t* → S_{t+1}\n\n');

% ===== 初始化状态 =====
state.pt=1; state.pos=all_xy(1,:); state.O=INIT_O; state.H=INIT_H; state.F=INIT_F;
state.M=INIT_M; state.Z=INIT_Z; state.consec=0; state.day=0;

plan_path=[]; plan_parks=[]; plan_works=[]; plan_leg=1;
step_in_leg=0; parked_in_leg=0; wp_idx=0; wd_done=0;

fprintf('--- 初始状态 ---\n');
fprintf('B(1,5) | O=%d H=%d F=%d | M=%d Z=%d\n\n', state.O, state.H, state.F, state.M, state.Z);
fprintf('--- 逐日在线决策 ---\n');
fprintf('Day | W | Action         | Pos      |  O   H   F  Load |   Z     M\n');
fprintf('----|---|----------------|----------|------------------|------------\n');

% ===== 逐日主循环 =====
while state.day < MAX_DAYS && state.pt ~= 2
    state.day = state.day + 1;
    w = weather_seq(state.day);
    if w == 'T', cons = cons_T; wname = 'thunder';
    else,        cons = cons_N; wname = 'normal';
    end
    act = ''; detail = '';

    % ---- CP 重规划（天气变化或无计划）----
    if isempty(plan_path) || (state.day > 1 && w ~= weather_seq(max(1,state.day-1)))
        elapsed = state.day - 1;
        init_s = struct('O',state.O,'H',state.H,'F',state.F,'M',state.M,'Z',state.Z);
        [plan_path, plan_parks, plan_works, feasible] = cp_plan(state.pt, elapsed, cons, ...
            dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, init_s);
        if ~feasible
            fprintf('%3d | %s | NO FEASIBLE     | (%2d,%2d)  | %3d %3d %3d %4d | %4d %5d\n', ...
                state.day, wname, state.pos(1), state.pos(2), ...
                round(state.O), round(state.H), round(state.F), ...
                round(state.O+state.H+state.F), state.Z, round(state.M));
            break;
        end
        plan_leg = 1; step_in_leg = 0; parked_in_leg = 0; wp_idx = 0; wd_done = 0;
    end

    if length(plan_path) < 2, break; end
    next_pt = plan_path(plan_leg + 1);

    % ---- 当日动作决策（优先级：作业/停泊重置 → 海上停泊 → 移动）----
    is_work_pt = (state.pt >= 3 && state.pt <= 5);
    need_work  = is_work_pt && wp_idx < length(plan_works) && wd_done < plan_works(wp_idx+1);

    if need_work
        wk_type = state.pt - 2;  % W1→1, W2→2, W3→3
        wm_val = WM(wk_type); yld = WY(wk_type);
        if state.consec < wm_val
            act = sprintf('work(%s)', names{state.pt});
            state.O = state.O - cons.WO; state.H = state.H - cons.WH; state.F = state.F - cons.WF;
            state.Z = state.Z + yld; state.consec = state.consec + 1; wd_done = wd_done + 1;
            if wd_done >= plan_works(wp_idx+1), wp_idx = wp_idx + 1; wd_done = 0; end
        else
            act = 'park(reset)';
            state.O = state.O - cons.PO; state.H = state.H - cons.PH; state.F = state.F - cons.PF;
            state.consec = 0;
        end
    elseif plan_leg <= length(plan_parks) && parked_in_leg < plan_parks(plan_leg)
        act = 'park(at sea)';
        state.O = state.O - cons.PO; state.H = state.H - cons.PH; state.F = state.F - cons.PF;
        state.consec = 0; parked_in_leg = parked_in_leg + 1;
    elseif step_in_leg < dist(state.pt, next_pt)
        state.O = state.O - cons.MO; state.H = state.H - cons.MH; state.F = state.F - cons.MF;
        state.consec = 0; step_in_leg = step_in_leg + 1;

        % 更新网格位置（曼哈顿走法：先 x 后 y）
        fr = all_xy(state.pt, :); to = all_xy(next_pt, :);
        dx_tot = to(1)-fr(1); dy_tot = to(2)-fr(2);
        sx = abs(dx_tot); sy = abs(dy_tot);
        if step_in_leg <= sx
            state.pos(1) = fr(1) + sign(dx_tot)*step_in_leg; state.pos(2) = fr(2);
        else
            state.pos(1) = to(1); state.pos(2) = fr(2) + sign(dy_tot)*(step_in_leg - sx);
        end
        act = sprintf('move -> (%d,%d)', state.pos(1), state.pos(2));

        if step_in_leg >= dist(state.pt, next_pt)
            state.pt = next_pt; state.pos = all_xy(next_pt, :);
            step_in_leg = 0; parked_in_leg = 0;
            % 到达作业点：计算 wp_idx
            if state.pt >= 3 && state.pt <= 5
                wc = 0;
                for i = 2:plan_leg+1
                    if plan_path(i) >= 3 && plan_path(i) <= 5, wc = wc + 1; end
                end
                wp_idx = wc - 1; wd_done = 0;
            end
            % 到达补给点
            if state.pt == 6 || state.pt == 7
                rem_t = 0; rem_p = 0;
                for k = plan_leg:length(plan_path)-2
                    rem_t = rem_t + dist(plan_path(k+1), plan_path(k+2));
                    if k+1 <= length(plan_parks), rem_p = rem_p + plan_parks(k+1); end
                end
                needO = rem_t*cons.MO + rem_p*cons.PO;
                needH = rem_t*cons.MH + rem_p*cons.PH;
                needF = rem_t*cons.MF + rem_p*cons.PF;
                sp = MAX_LOAD - (state.O+state.H+state.F);
                bO = max(0, needO-state.O); bH = max(0, needH-state.H); bF = max(0, needF-state.F);
                if bO+bH+bF <= sp
                    cost = bO*cons.pO+bH*cons.pH+bF*cons.pF;
                    if cost <= state.M
                        state.O=state.O+bO; state.H=state.H+bH; state.F=state.F+bF; state.M=state.M-cost;
                        act = sprintf('SUPPLY(%s)', names{state.pt});
                        detail = sprintf('+O%d H%d F%d cost=%d', round(bO),round(bH),round(bF),round(cost));
                    end
                end
            elseif state.pt == 2
                act = 'ARRIVE at E';
            end
            plan_leg = plan_leg + 1;
        end
    end

    % ---- 资源检查 ----
    if state.O < 0 || state.H < 0 || state.F < 0
        act = 'RESOURCE EXHAUSTED';
    end
    if state.O+state.H+state.F > MAX_LOAD + 1
        act = 'OVERLOAD';
    end

    % ---- 输出当日状态 ----
    fprintf('%3d | %s | %-14s | (%2d,%2d)  | %3d %3d %3d %4d | %4d %5d', ...
        state.day, wname, act, state.pos(1), state.pos(2), ...
        round(state.O), round(state.H), round(state.F), ...
        round(state.O+state.H+state.F), state.Z, round(state.M));
    if ~isempty(detail), fprintf('  %s', detail); end
    fprintf('\n');
end

fprintf('----|---|----------------|----------|------------------|------------\n');
fprintf('\n===== 最终结果 =====\n');
if state.pt == 2
    fprintf('状态: 第 %d 天抵达 E\n', state.day);
else
    fprintf('状态: 未能在 %d 天内抵达 E\n', MAX_DAYS);
end
fprintf('Z = %d  (目标物资)\n', state.Z);
fprintf('M = %d  (剩余资金)\n', round(state.M));
fprintf('载重: O=%d H=%d F=%d Total=%d\n', round(state.O), round(state.H), round(state.F), round(state.O+state.H+state.F));
fprintf('\nDone.\n');
end

% ===== CP 在线规划 =====
function [best_path, best_parks, best_works, feasible] = cp_plan(cur_pt, elapsed, cons, ...
    dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, init_s)
    best_Z = -inf; best_M = -inf;
    best_path = [cur_pt, 2]; best_parks = []; best_works = [];
    nodes = 0;
    eff_days = MAX_DAYS - elapsed;
    [best_Z, best_M, best_path, best_works, best_parks, nodes] = ...
        cp_search_online([cur_pt], 0, [], [], best_Z, best_M, best_path, best_works, best_parks, ...
        dist, inter_idx, WY, WM, cons, nodes, eff_days, MAX_LOAD, init_s);
    feasible = (best_Z > -inf);
    if ~feasible
        best_path = [cur_pt, 2]; best_parks = []; best_works = [];
    end
end

% ===== CP 递归搜索 =====
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online(path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, ...
    dist, inter_idx, WY, WM, cons, nodes, MAX_DAYS_EFF, MAX_LOAD, init_s)
    nodes = nodes + 1; lp = path(end);
    if lp ~= 2
        dE = dist(lp, 2); rem = MAX_DAYS_EFF - tsf;
        if rem < dE, return; end
        ub = init_s.Z + cp_common('max_work_with_park', 3, rem-dE) * 28;
        if ub <= bZ && bZ > -inf, return; end
    end
    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS_EFF
        fp = [path, 2]; m = length(fp) - 2; tt = 0;
        for k = 1:m+1, tt = tt + dist(fp(k), fp(k+1)); end
        rem_days = MAX_DAYS_EFF - tt; nw = length(wa);
        if nw == 0
            n_seg = m + 1;
            park_combs = cp_common('enumerate_park_combs', n_seg+1, rem_days);
            for ci = 1:size(park_combs,1)
                ps = park_combs(ci, 1:n_seg);
                [ok, Z, M, ~] = cp_common('simulate', fp, m, dist, [], [], tt, [], ps, cons, MAX_LOAD, init_s);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bWD = []; bPS = ps;
                end
            end
        else
            max_wk = zeros(1, nw);
            for j = 1:nw, max_wk(j) = cp_common('max_work_with_park', WM(ww(j)), rem_days); end
            sz = max_wk + 1; nc = prod(sz);
            for ci = 1:nc
                wd = zeros(1, nw); t2 = ci - 1;
                for j = nw:-1:1, wd(j) = mod(t2, sz(j)); t2 = floor(t2/sz(j)); end
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0, total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/WM(ww(j)))-1); end
                end
                if tt + total_stay > MAX_DAYS_EFF, continue; end
                park_rem = MAX_DAYS_EFF - tt - total_stay; n_seg = m + 1;
                park_combs = cp_common('enumerate_park_combs', n_seg, park_rem);
                for pci = 1:size(park_combs,1)
                    ps = park_combs(pci, :);
                    if tt + total_stay + sum(ps) > MAX_DAYS_EFF, continue; end
                    [ok, Z, M, ~] = cp_common('simulate', fp, m, dist, wa, ww, tt, wd, ps, cons, MAX_LOAD, init_s);
                    if ok && (Z > bZ || (Z == bZ && M > bM))
                        bZ = Z; bM = M; bP = fp; bWD = wd; bPS = ps;
                    end
                end
            end
        end
    end
    for ni = 1:5
        np = inter_idx(ni); if np == lp, continue; end
        d = dist(lp, np); if tsf + d > MAX_DAYS_EFF, continue; end
        dE2 = dist(np, 2); if tsf + d + dE2 > MAX_DAYS_EFF, continue; end
        np2 = [path, np]; nt = tsf + d; nwa = wa; nww = ww;
        if np >= 3 && np <= 5, nwa(end+1) = length(np2); nww(end+1) = np-2; end
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online(np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, ...
            dist, inter_idx, WY, WM, cons, nodes, MAX_DAYS_EFF, MAX_LOAD, init_s);
    end
end
