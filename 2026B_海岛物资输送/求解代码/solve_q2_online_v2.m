function solve_q2_online_v2()
% =========================================================================
% solve_q2_online_v2.m — 任务2 在线约束规划决策模型 (v2)
% =========================================================================
%
% 数学模型：
% ─────────
% 时刻 t 的船舶状态（观测天气后、决策动作前）：
%
%   S_t = (pt, O_t, H_t, F_t, M_t, Z_t, c_t, w_t, d_t)
%
%   pt    ∈ {1..7}    当前所在点位 (1=B,2=E,3=W1,4=W2,5=W3,6=S1,7=S2)
%   O_t   ∈ Z_+       燃油余量
%   H_t   ∈ Z_+       淡水余量
%   F_t   ∈ Z_+       食物余量
%   M_t   ∈ Z_+       剩余资金
%   Z_t   ∈ Z_+       目标物资存量
%   c_t   ∈ [0, WM_i] 连续作业天数 (仅在作业点有效)
%   w_t   ∈ {N, T}    当日天气 (N=正常, T=雷暴)
%   d_t   ∈ {1..30}   当前天数
%
% 动作空间：
%
%   A(S_t) = { ↑, ↓, ←, →, 停泊, 作业, 补给 }
%
% 决策函数（每日调用一次）：
%
%   a_t* = argmax_{a ∈ A(S_t)} V( T(S_t, a, w_t) )
%
%   其中 V 是 CP 前向搜索估计的词典序最优值 (先 max Z, 再 max M)。
%
% 状态转移：
%
%   S_{t+1} = T(S_t, a_t*, w_t)
%
%   转移规则：
%     • 移动：O -= c_O(w), H -= c_H(w), F -= c_F(w); 坐标按方向更新
%     • 停泊：O -= c_O^park, H -= c_H^park, F -= c_F^park; c_t = 0
%     • 作业：O -= c_O^work, H -= c_H^work, F -= c_F^work; Z += yield; c_t += 1
%     • 补给：在 S1/S2 当日采购资源，削减 M，增加 O/H/F
%
% 在线决策架构（逐日循环）：
%
%   for t = 1:30:
%       w_t ← 观测当日天气
%       a_t* ← CP_Search(S_t, assume_future_weather = w_t)
%       执行 a_t*，更新 S_{t+1}
%       if pt == E: 终止
%
% 天气假设：CP 搜索假设剩余天数天气与今日观测一致（悲观且一致）。
% 全雷暴情形下假设 ≡ 真实 → 结果等价于确定性全局最优规划。
% =========================================================================

% ===== 配置 =====
cfg.MAX_DAYS = 30;
cfg.MAX_LOAD = 120;
cfg.all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];  % B,E,W1,W2,W3,S1,S2
cfg.names  = {'B','E','W1','W2','W3','S1','S2'};
cfg.WY = [20, 15, 28];
cfg.WM = [4, 5, 3];
cfg.init_O = 35; cfg.init_H = 45; cfg.init_F = 30;
cfg.init_M = 240; cfg.init_Z = 100;

% 两种天气的消耗参数
cfg.cons_normal = struct(...
    'MO', 2, 'MH', 3, 'MF', 2, 'PO', 1, 'PH', 1, 'PF', 1, ...
    'WO', 5, 'WH', 4, 'WF', 3, 'pO', 2, 'pH', 1, 'pF', 2);
cfg.cons_thunder = struct(...
    'MO', 8, 'MH', 4, 'MF', 3, 'PO', 3, 'PH', 3, 'PF', 2, ...
    'WO', 8, 'WH', 6, 'WF', 6, 'pO', 2, 'pH', 1, 'pF', 2);

% 距离矩阵
n_pts = size(cfg.all_xy, 1);
cfg.dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        cfg.dist(i,j) = abs(cfg.all_xy(i,1)-cfg.all_xy(j,1)) + abs(cfg.all_xy(i,2)-cfg.all_xy(j,2));
    end
end
cfg.inter_idx = [3 4 5 6 7];

% ===== 天气序列（可切换）=====
weather_mode = 'all_thunder';  % 'all_thunder' | 'all_normal' | 'mixed'
switch weather_mode
    case 'all_thunder'
        weather_seq = repmat('T', 1, cfg.MAX_DAYS);  % 全雷暴
    case 'all_normal'
        weather_seq = repmat('N', 1, cfg.MAX_DAYS);
    case 'mixed'
        % Day 1 thunderstorm, Days 2-30 normal (match user scenario)
        weather_seq = [repmat('T', 1, 1), repmat('N', 1, cfg.MAX_DAYS-1)];
end

fprintf('========================================\n');
fprintf('  Task 2: Online CP Decision Model (v2)\n');
fprintf('========================================\n');
fprintf('Weather mode: %s\n', weather_mode);
fprintf('Model: State S_t = (pt, O,H,F, M,Z, c, w_t)\n');
fprintf('Loop:  observe w_t -> CP-search -> execute a_t -> S_{t+1}\n\n');

% ===== 初始化状态 =====
state.pt     = 1;                    % B
state.pos    = cfg.all_xy(1, :);     % 当前网格坐标
state.O      = cfg.init_O;
state.H      = cfg.init_H;
state.F      = cfg.init_F;
state.M      = cfg.init_M;
state.Z      = cfg.init_Z;
state.consec = 0;                    % 连续作业天数
state.day    = 0;

fprintf('--- Initial State ---\n');
fprintf('Pos: B(1,5) | O=%d H=%d F=%d | M=%d Z=%d\n\n', ...
    state.O, state.H, state.F, state.M, state.Z);

% ===== 逐日在线决策主循环 =====
fprintf('--- Day-by-Day Online Decision ---\n');
fprintf('Day | W | Action         | Pos (x,y)  |  O   H   F  Load |   Z     M\n');
fprintf('----|---|----------------|------------|------------------|------------\n');

path_plan = [];      % CP 规划的路径骨架
parks_plan = [];     % 各段停泊日
works_plan = [];     % 工作天
leg = 1;             % 当前段索引
step_in_leg = 0;     % 当前段内步数 (移动了几格)
parked_in_leg = 0;   % 当前段已停泊天数
arrived_today = false;

while state.day < cfg.MAX_DAYS
    state.day = state.day + 1;
    w = weather_seq(state.day);

    % 选择消耗参数
    if w == 'T', cons = cfg.cons_thunder; wname = 'thunder';
    else,        cons = cfg.cons_normal;  wname = 'normal';
    end

    detail = '';

    % ---- 需要重新规划？----
    % 条件：路径为空 / 已到达上一段的命名点 / 当前在命名点且计划为空
    arrived = arrived_today;
    arrived_today = false;

    if isempty(path_plan) || (arrived && state.pt == path_plan(leg))
        % 从当前状态调用 CP 搜索
        elapsed = state.day - 1;
        init_s = struct('O', state.O, 'H', state.H, 'F', state.F, ...
                        'M', state.M, 'Z', state.Z);
        temp_cfg = cfg;
        temp_cfg.init = init_s;
        temp_cfg.cons = cons;
        temp_cfg.elapsed = elapsed;

        [path_plan, parks_plan, works_plan, feasible] = ...
            cp_plan_online(state.pt, elapsed, cons, temp_cfg);

        if ~feasible
            fprintf('%3d | %s | NO FEASIBLE     | (%2d,%2d)     | %3d %3d %3d %4d | %4d %5d\n', ...
                state.day, wname, state.pos(1), state.pos(2), ...
                round(state.O), round(state.H), round(state.F), ...
                round(state.O+state.H+state.F), state.Z, round(state.M));
            break;
        end

        leg = 1;
        step_in_leg = 0;
        parked_in_leg = 0;
    end

    if length(path_plan) < 2, break; end  % 已在 E

    next_pt = path_plan(leg + 1);

    % ---- 当日动作决策 ----
    % 规则优先级：停泊(未完成) > 移动 > 到达处理(补给/工作)

    if leg <= length(parks_plan) && parked_in_leg < parks_plan(leg)
        % 本段还有未完成的停泊日
        action = 'park(at sea)';
        state.O = state.O - cons.PO;
        state.H = state.H - cons.PH;
        state.F = state.F - cons.PF;
        state.consec = 0;
        parked_in_leg = parked_in_leg + 1;

    elseif step_in_leg < cfg.dist(state.pt, next_pt)
        % 移动中
        state.O = state.O - cons.MO;
        state.H = state.H - cons.MH;
        state.F = state.F - cons.MF;
        state.consec = 0;
        step_in_leg = step_in_leg + 1;

        % 更新位置
        fr = cfg.all_xy(state.pt, :);
        to = cfg.all_xy(next_pt, :);
        dx_total = to(1) - fr(1);
        dy_total = to(2) - fr(2);
        steps_x = abs(dx_total);
        steps_y = abs(dy_total);
        if step_in_leg <= steps_x
            state.pos(1) = fr(1) + sign(dx_total) * step_in_leg;
            state.pos(2) = fr(2);
        else
            state.pos(1) = to(1);
            state.pos(2) = fr(2) + sign(dy_total) * (step_in_leg - steps_x);
        end

        d_total = cfg.dist(state.pt, next_pt); if step_in_leg == d_total
            % 到达目标点
            state.pt = next_pt;
            arrived_today = true;
            state.pos = cfg.all_xy(next_pt, :);
            step_in_leg = 0;
            parked_in_leg = 0;

            if state.pt == 6 || state.pt == 7
                % ---- 补给决策 ----
                % 计算剩余路径所需资源
                rem_travel = 0; rem_park = 0;
                for k = leg:(length(path_plan)-2)
                    rem_travel = rem_travel + cfg.dist(path_plan(k+1), path_plan(k+2));
                    if k+2 <= length(parks_plan)
                        rem_park = rem_park + parks_plan(k+1);
                    end
                end
                needO = rem_travel * cons.MO + rem_park * cons.PO;
                needH = rem_travel * cons.MH + rem_park * cons.PH;
                needF = rem_travel * cons.MF + rem_park * cons.PF;
                % 加上后续工作消耗
                for jj = 1:length(works_plan)
                    if works_plan(jj) > 0
                        % 简化：用期望消耗估算 (在线模型不需要精确到每个组合)
                        needO = needO + works_plan(jj) * cons.WO;
                        needH = needH + works_plan(jj) * cons.WH;
                        needF = needF + works_plan(jj) * cons.WF;
                    end
                end
                sp = cfg.MAX_LOAD - (state.O + state.H + state.F);
                bO = max(0, needO - state.O);
                bH = max(0, needH - state.H);
                bF = max(0, needF - state.F);

                if bO + bH + bF <= sp
                    cost = bO*cons.pO + bH*cons.pH + bF*cons.pF;
                    if cost <= state.M
                        state.O = state.O + bO;
                        state.H = state.H + bH;
                        state.F = state.F + bF;
                        state.M = state.M - cost;
                        action = sprintf('SUPPLY(%s)', cfg.names{state.pt});
                        detail = sprintf('+O%d H%d F%d cost=%d', ...
                            round(bO), round(bH), round(bF), round(cost));
                    else
                        action = 'move';
                    end
                else
                    action = 'move';
                end
            elseif state.pt == 2
                action = 'ARRIVE at E';
            end

            % 移动到下一段
            leg = leg + 1;
        else
            action = 'move';
        end

    else
        % 在目标点停留（作业 或 停泊等待）
        wk_idx = find(state.pt == [3, 4, 5], 1);
        if ~isempty(wk_idx) && ~isempty(works_plan)
            % 统计当前点累计作业天数
            total_work_here = 0;
            wp_count = 0;
            for pp = 1:leg
                if pp <= length(path_plan) && path_plan(pp) >= 3 && path_plan(pp) <= 5
                    wp_count = wp_count + 1;
                    if wp_count <= length(works_plan) && pp == leg
                        total_work_here = works_plan(wp_count);
                    end
                end
            end
            % 检查是否还能继续作业
            wm_limit = cfg.WM(wk_idx);
            work_done_here = 0;  % 本点已工作天数 (简化：从 consec 推断)
            if state.consec < wm_limit && total_work_here > 0
                action = sprintf('work(%s)', cfg.names{state.pt});
                state.O = state.O - cons.WO;
                state.H = state.H - cons.WH;
                state.F = state.F - cons.WF;
                state.Z = state.Z + cfg.WY(wk_idx);
                state.consec = state.consec + 1;
            else
                action = 'park(reset)';
                state.O = state.O - cons.PO;
                state.H = state.H - cons.PH;
                state.F = state.F - cons.PF;
                state.consec = 0;
            end
        else
            action = 'park';
            state.O = state.O - cons.PO;
            state.H = state.H - cons.PH;
            state.F = state.F - cons.PF;
            state.consec = 0;
        end
    end

    % ---- 检查资源可行性 ----
    if state.O < 0 || state.H < 0 || state.F < 0
        action = 'RESOURCE EXHAUSTED';
    end
    if state.O + state.H + state.F > cfg.MAX_LOAD + 1
        action = 'OVERLOAD';
    end

    % ---- 输出当日状态 ----
    fprintf('%3d | %s | %-14s | (%2d,%2d)     | %3d %3d %3d %4d | %4d %5d', ...
        state.day, wname, action, state.pos(1), state.pos(2), ...
        round(state.O), round(state.H), round(state.F), ...
        round(state.O+state.H+state.F), state.Z, round(state.M));
    if ~isempty(detail), fprintf('  %s', detail); end
    fprintf('\n');

    if state.pt == 2
        break;
    end
end

fprintf('----|---|----------------|------------|------------------|------------\n');

% ===== 最终结果 =====
fprintf('\n===== FINAL RESULT =====\n');
if state.pt == 2
    fprintf('Status: Arrived at E on day %d\n', state.day);
else
    fprintf('Status: Did NOT reach E within %d days\n', cfg.MAX_DAYS);
end
fprintf('Z = %d  (target materials)\n', state.Z);
fprintf('M = %d  (remaining funds)\n', round(state.M));
fprintf('Load at E: O=%d H=%d F=%d Total=%d\n', ...
    round(state.O), round(state.H), round(state.F), round(state.O+state.H+state.F));
fprintf('\nDone.\n');
end

% ===== CP 在线规划函数 =====
function [best_path, best_parks, best_works, feasible] = cp_plan_online(cur_pt, elapsed, cons, cfg)
% 从当前状态 (cur_pt, elapsed 天已用) 出发，CP 搜索剩余天数内最优路径骨架。
% 天气假设：剩余天数全部与当日天气一致（cons 由当日观测决定）。

    best_Z = -inf; best_M = -inf;
    best_path = [cur_pt, 2];
    best_parks = [];
    best_works = [];
    nodes = 0;

    [best_Z, best_M, best_path, best_works, best_parks, nodes] = ...
        cp_search_online([cur_pt], elapsed, [], [], best_Z, best_M, ...
                         best_path, best_works, best_parks, cfg, cons, nodes);

    feasible = (best_Z > -inf);
    if ~feasible
        best_path = [cur_pt, 2];
        best_parks = [];
        best_works = [];
    end
end

% ===== CP 递归搜索（在线版）=====
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online(path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, cfg, cons, nodes)
    nodes = nodes + 1;
    lp = path(end);

    % 上界剪枝
    if lp ~= 2
        dE = cfg.dist(lp, 2);
        rem = cfg.MAX_DAYS - tsf;
        if rem < dE, return; end
        ub = cfg.init.Z + cp_common('max_work_with_park', 3, rem - dE) * 28;  % v2: use current Z
        if ub <= bZ && bZ > -inf, return; end
    end

    % 叶节点评估
    dE = cfg.dist(lp, 2);
    if tsf + dE <= cfg.MAX_DAYS
        fp = [path, 2];
        m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1), tt = tt + cfg.dist(fp(k), fp(k+1)); end

        rem_days = cfg.MAX_DAYS - tt;
        nw = length(wa);
        init_s = cfg.init;

        if nw == 0
            n_seg = m + 1;
            park_combs = cp_common('enumerate_park_combs', n_seg + 1, rem_days);
            for ci = 1:size(park_combs, 1)
                ps = park_combs(ci, 1:n_seg);
                [ok, Z, M, ~] = cp_common('simulate', fp, m, cfg.dist, [], [], tt, [], ps, ...
                    cons, cfg.MAX_LOAD, init_s);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bWD = []; bPS = ps;
                end
            end
        else
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = cp_common('max_work_with_park', cfg.WM(ww(j)), rem_days);
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
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/cfg.WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > cfg.MAX_DAYS, continue; end
                park_rem = cfg.MAX_DAYS - tt - total_stay;
                n_seg = m + 1;
                park_combs = cp_common('enumerate_park_combs', n_seg, park_rem);
                for pci = 1:size(park_combs, 1)
                    ps = park_combs(pci, :);
                    if tt + total_stay + sum(ps) > cfg.MAX_DAYS, continue; end
                    [ok, Z, M, ~] = cp_common('simulate', fp, m, cfg.dist, wa, ww, tt, wd, ps, ...
                        cons, cfg.MAX_LOAD, init_s);
                    if ok && (Z > bZ || (Z == bZ && M > bM))
                        bZ = Z; bM = M; bP = fp; bWD = wd; bPS = ps;
                    end
                end
            end
        end
    end

    % 分支
    for ni = 1:5
        np = cfg.inter_idx(ni);
        if np == lp, continue; end
        d = cfg.dist(lp, np);
        if tsf + d > cfg.MAX_DAYS, continue; end
        dE2 = cfg.dist(np, 2);
        if tsf + d + dE2 > cfg.MAX_DAYS, continue; end

        np2 = [path, np]; nt = tsf + d;
        nwa = wa; nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2);
            nww(end+1) = np - 2;
        end
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online(np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, cfg, cons, nodes);
    end
end
