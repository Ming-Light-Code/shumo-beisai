% DEPRECATED: Use 优化版/solve_q3_montecarlo_opt.m instead
% This file is kept for reference only.

function solve_q3_montecarlo(N)
% =========================================================================
%  solve_q3_montecarlo.m — 任务3 蒙特卡洛模拟验证
%  =========================================================================
%  功能:  生成 N 条独立随机天气序列，逐一驱动在线决策模型，
%         统计 Z/M 分布、成功率、失败模式，验证期望值方案的稳健性。
%
%  用法:
%    solve_q3_montecarlo()     默认 N=500
%    solve_q3_montecarlo(N)    指定模拟次数
%
%  输出:
%    - 成功率 (%)
%    - Z/M 的均值、标准差、最小值、最大值、中位数
%    - 失败原因分布
%    - 与离线最优解 (Z=480, M=9) 的对比
%  =========================================================================

if nargin < 1 || isempty(N), N = 500; end

fprintf('========================================\n');
fprintf('  任务3 蒙特卡洛模拟验证 (N = %d)\n', N);
fprintf('========================================\n');
fprintf('在线决策模型: 滚动时域CP重规划\n');
fprintf('天气模型: P(正常)=0.8, P(雷暴)=0.2, i.i.d.\n');
fprintf('离线最优: Z=480, M≈9\n\n');

% ===== 初始化统计数组 =====
Z_results = NaN(1, N);
M_results = NaN(1, N);
success   = false(1, N);
days_used = NaN(1, N);
fail_reason = cell(1, N);

% 进度输出间隔
report_interval = max(1, floor(N / 20));

fprintf('开始模拟...\n');
tic;

% ===== 批量运行 =====
parfor_progress_available = false;  % 不使用parfor, 保持简单

for sim = 1:N
    % 生成随机天气序列
    weather_seq = generate_weather_mc(90, 0.8);

    % 运行在线决策模型 (静默模式)
    [Z_final, M_final, arrived, days, reason] = run_online_silent(weather_seq);

    Z_results(sim) = Z_final;
    M_results(sim) = M_final;
    success(sim) = arrived;
    days_used(sim) = days;
    fail_reason{sim} = reason;

    % 进度报告
    if mod(sim, report_interval) == 0
        elapsed = toc;
        fprintf('  进度: %d/%d (%.0f%%) | 耗时: %.1f秒 | 当前成功率: %.1f%%\n', ...
            sim, N, 100*sim/N, elapsed, 100*sum(success(1:sim))/sim);
    end
end

total_time = toc;
fprintf('\n模拟完成。总耗时: %.1f秒\n\n', total_time);

% ===== 统计分析 =====
n_success = sum(success);
success_rate = 100 * n_success / N;

fprintf('========================================\n');
fprintf('  统计结果\n');
fprintf('========================================\n');
fprintf('总模拟次数: %d\n', N);
fprintf('成功抵达E:  %d (%.1f%%)\n', n_success, success_rate);
fprintf('失败:       %d (%.1f%%)\n', N - n_success, 100 - success_rate);
fprintf('\n');

if n_success > 0
    Z_success = Z_results(success);
    M_success = M_results(success);
    D_success = days_used(success);

    fprintf('--- 成功运行的统计 (n=%d) ---\n', n_success);
    fprintf('%-20s %8s %8s %8s %8s %8s\n', '指标', '均值', '标准差', '最小值', '最大值', '中位数');
    fprintf('%-20s %8.1f %8.1f %8d %8d %8d\n', ...
        'Z (目标物资)', mean(Z_success), std(Z_success), ...
        min(Z_success), max(Z_success), median(Z_success));
    fprintf('%-20s %8.2f %8.2f %8.2f %8.2f %8.2f\n', ...
        'M (剩余资金)', mean(M_success), std(M_success), ...
        min(M_success), max(M_success), median(M_success));
    fprintf('%-20s %8.1f %8.1f %8d %8d %8d\n', ...
        '天数', mean(D_success), std(D_success), ...
        min(D_success), max(D_success), median(D_success));
    fprintf('\n');

    % Z 分布
    fprintf('--- Z 分布 ---\n');
    z_edges = [0 200 300 350 400 420 440 460 480 500];
    z_hist = histcounts(Z_success, z_edges);
    for i = 1:length(z_edges)-1
        bar_len = round(50 * z_hist(i) / max(z_hist));
        fprintf('  [%3d-%3d): %4d |%s\n', z_edges(i), z_edges(i+1), z_hist(i), repmat('#', 1, bar_len));
    end
    fprintf('\n');

    % M 分布
    fprintf('--- M 分布 ---\n');
    m_edges = [-50 0 5 10 20 50 100 200 500];
    m_hist = histcounts(M_success, m_edges);
    for i = 1:length(m_edges)-1
        bar_len = round(50 * m_hist(i) / max(m_hist));
        fprintf('  [%4.0f-%4.0f): %4d |%s\n', m_edges(i), m_edges(i+1), m_hist(i), repmat('#', 1, bar_len));
    end
    fprintf('\n');

    % 百分位数
    fprintf('--- Z 百分位数 ---\n');
    percentiles = [5 10 25 50 75 90 95];
    z_pct = prctile(Z_success, percentiles);
    for i = 1:length(percentiles)
        fprintf('  P%02d: %d\n', percentiles(i), round(z_pct(i)));
    end
    fprintf('\n');
end

% 失败原因分析
if N - n_success > 0
    fprintf('--- 失败原因分布 ---\n');
    fail_reasons = fail_reason(~success);
    [unique_reasons, ~, ic] = unique(fail_reasons);
    counts = accumarray(ic, 1);
    [counts_sorted, idx] = sort(counts, 'descend');
    for i = 1:length(unique_reasons)
        fprintf('  %s: %d (%.1f%%)\n', unique_reasons{idx(i)}, ...
            counts_sorted(i), 100*counts_sorted(i)/(N-n_success));
    end
    fprintf('\n');
end

% ===== 与离线最优解对比 =====
fprintf('--- 与离线最优解对比 ---\n');
fprintf('  离线最优: Z=480, M≈9\n');
if n_success > 0
    pct_optimal_Z = 100 * sum(Z_success >= 480) / n_success;
    pct_near_optimal = 100 * sum(Z_success >= 460) / n_success;
    fprintf('  Z ≥ 480 (达到最优): %.1f%%\n', pct_optimal_Z);
    fprintf('  Z ≥ 460 (接近最优): %.1f%%\n', pct_near_optimal);
    fprintf('  Z 均值 / 最优: %.1f%%\n', 100 * mean(Z_success) / 480);
end
fprintf('\n');

fprintf('分析完成。\n');
end

% =====================================================================
%  静默运行在线决策模型 (不输出日志)
% =====================================================================
function [Z_final, M_final, arrived, days, reason] = run_online_silent(weather_seq)
% 运行在线决策模型, 返回最终结果 (不打印)

MAX_DAYS = 90; MAX_LOAD = 400;
all_xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
WY = [20, 15, 28]; WM = [4, 5, 3];
INIT_O = 100; INIT_H = 150; INIT_F = 100;
INIT_M = 750; INIT_Z = 200;

SAFETY = 1.10;
cons_exp = struct('MO',3.2*SAFETY,'MH',3.2*SAFETY,'MF',2.2*SAFETY, ...
                  'PO',1.4*SAFETY,'PH',1.4*SAFETY,'PF',1.2*SAFETY, ...
                  'WO',5.6*SAFETY,'WH',4.4*SAFETY,'WF',3.6*SAFETY, ...
                  'pO',2,'pH',1,'pF',2);
cons_N = struct('MO',2,'MH',3,'MF',2, ...
                'PO',1,'PH',1,'PF',1, ...
                'WO',5,'WH',4,'WF',3, ...
                'pO',2,'pH',1,'pF',2);
cons_T = struct('MO',8,'MH',4,'MF',3, ...
                'PO',3,'PH',3,'PF',2, ...
                'WO',8,'WH',6,'WF',6, ...
                'pO',2,'pH',1,'pF',2);

n_pts = size(all_xy, 1);
dist = zeros(n_pts);
for i = 1:n_pts, for j = 1:n_pts
    dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
end; end
inter_idx = [3 4 5 6 7];

% 初始化状态
state.pt = 1; state.pos = all_xy(1,:);
state.O = INIT_O; state.H = INIT_H; state.F = INIT_F;
state.M = INIT_M; state.Z = INIT_Z;
state.consec = 0; state.day = 0;

plan_path = []; plan_parks = []; plan_works = [];
plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
wp_idx = 0; wd_done = 0;

arrived = false; reason = '';

% 逐日决策
while state.day < MAX_DAYS && state.pt ~= 2
    state.day = state.day + 1;
    w = weather_seq(state.day);
    if w == 'T', cons_act = cons_T;
    else,        cons_act = cons_N;
    end

    % 重规划触发 (仅当位于非作业命名点时)
    at_named_point = (step_in_leg == 0 && parked_in_leg == 0);
    is_work_pt_now = (state.pt >= 3 && state.pt <= 5);
    not_at_work_pt = ~is_work_pt_now;
    need_replan = isempty(plan_path) || ...
                  (state.day > 1 && w ~= weather_seq(max(1, state.day-1)) && at_named_point && not_at_work_pt) || ...
                  (mod(state.day, 3) == 1 && state.day > 1 && at_named_point && not_at_work_pt);

    if need_replan
        elapsed = state.day - 1;
        init_s = struct('O',state.O,'H',state.H,'F',state.F,'M',state.M,'Z',state.Z);
        [plan_path, plan_parks, plan_works, feasible] = ...
            cp_plan_silent(state.pt, elapsed, cons_exp, dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, init_s);
        if ~feasible
            % fallback: try normal consumption
            [plan_path, plan_parks, plan_works, feasible] = ...
                cp_plan_silent(state.pt, elapsed, cons_N, dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, init_s);
        end
        if ~feasible
            reason = 'CP无可行方案';
            break;
        end
        plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
        wp_idx = 0; wd_done = 0;
    end

    if length(plan_path) < 2, reason = '计划路径为空'; break; end
    next_pt = plan_path(plan_leg + 1);

    % 动作决策
    is_work_pt = (state.pt >= 3 && state.pt <= 5);
    wp_match = true;
    if is_work_pt && wp_idx < length(plan_works)
        wa_local = []; for ii=2:length(plan_path)
            if plan_path(ii)>=3 && plan_path(ii)<=5, wa_local(end+1)=ii; end
        end
        if wp_idx+1 <= length(wa_local)
            wp_match = (plan_path(wa_local(wp_idx+1)) == state.pt);
        end
    end
    need_work  = is_work_pt && wp_match && wp_idx < length(plan_works) && wd_done < plan_works(wp_idx+1);

    if need_work
        wk_type = state.pt - 2;
        wm_val = WM(wk_type); yld = WY(wk_type);
        if state.consec < wm_val
            state.O = state.O - cons_act.WO; state.H = state.H - cons_act.WH;
            state.F = state.F - cons_act.WF; state.Z = state.Z + yld;
            state.consec = state.consec + 1; wd_done = wd_done + 1;
            if wd_done >= plan_works(wp_idx+1)
                wp_idx = wp_idx + 1; wd_done = 0;
            end
        else
            state.O = state.O - cons_act.PO; state.H = state.H - cons_act.PH;
            state.F = state.F - cons_act.PF; state.consec = 0;
        end
    elseif plan_leg <= length(plan_parks) && parked_in_leg < plan_parks(plan_leg)
        state.O = state.O - cons_act.PO; state.H = state.H - cons_act.PH;
        state.F = state.F - cons_act.PF; state.consec = 0;
        parked_in_leg = parked_in_leg + 1;
    elseif step_in_leg < dist(state.pt, next_pt)
        state.O = state.O - cons_act.MO; state.H = state.H - cons_act.MH;
        state.F = state.F - cons_act.MF; state.consec = 0;
        step_in_leg = step_in_leg + 1;
        if step_in_leg >= dist(state.pt, next_pt)
            state.pt = next_pt; state.pos = all_xy(next_pt, :);
            step_in_leg = 0; parked_in_leg = 0;
            if state.pt >= 3 && state.pt <= 5
                wc = 0;
                for i = 2:plan_leg+1
                    if plan_path(i) >= 3 && plan_path(i) <= 5, wc = wc + 1; end
                end
                wp_idx = wc - 1; wd_done = 0;
            end
            if state.pt == 6 || state.pt == 7
                rem_t = 0; rem_p = 0;
                for k = plan_leg:length(plan_path)-2
                    rem_t = rem_t + dist(plan_path(k+1), plan_path(k+2));
                    if k+1 <= length(plan_parks), rem_p = rem_p + plan_parks(k+1); end
                end
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
                    else
                        reason = '补给资金不足'; break;
                    end
                else
                    reason = '补给超载重'; break;
                end
            end
            plan_leg = plan_leg + 1;
        end
    end

    % 失败检查
    if state.O < -1e-6 || state.H < -1e-6 || state.F < -1e-6
        reason = '资源耗尽'; break;
    end
    if state.O + state.H + state.F > MAX_LOAD + 1e-6
        reason = '超载'; break;
    end
    if ~isempty(reason), break; end
end

% 整理结果
if state.pt == 2
    arrived = true; days = state.day;
elseif state.day >= MAX_DAYS
    if isempty(reason), reason = '超时未抵达E'; end
    days = MAX_DAYS;
else
    if isempty(reason), reason = '其他失败'; end
    days = state.day;
end

if arrived
    Z_final = state.Z; M_final = state.M;
else
    Z_final = 0; M_final = 0;  % 未抵达E, Z/M清零
end
end

% =====================================================================
%  静默CP规划 (与solve_q3_online.m中的cp_plan_from_state相同)
% =====================================================================
function [best_path, best_parks, best_works, feasible] = ...
    cp_plan_silent(cur_pt, elapsed, cons, dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, init_s)
    best_Z = -inf; best_M = -inf;
    best_path = [cur_pt, 2]; best_works = []; best_parks = [];
    nodes = 0;
    eff_days = MAX_DAYS - elapsed;
    [best_Z, best_M, best_path, best_works, best_parks, nodes] = ...
        cp_search_silent([cur_pt], 0, [], [], best_Z, best_M, best_path, best_works, best_parks, ...
        dist, inter_idx, WY, WM, cons, nodes, eff_days, MAX_LOAD, init_s);
    feasible = (best_Z > -inf);
end

function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_silent(path, tsf, wa, ww, ...
    bZ, bM, bP, bWD, bPS, dist, inter_idx, WY, WM, cons, nodes, MAX_DAYS_EFF, MAX_LOAD, init_s)
    nodes = nodes + 1; lp = path(end);
    if lp ~= 2
        dE = dist(lp, 2); rem_total = MAX_DAYS_EFF - tsf;
        if rem_total < dE, return; end
        ub_work = max_work_with_park_silent(3, rem_total - dE);
        if init_s.Z + ub_work * 28 <= bZ && bZ > -inf, return; end
    end
    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS_EFF
        fp = [path, 2]; m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1), tt = tt + dist(fp(k), fp(k+1)); end
        rem_days = MAX_DAYS_EFF - tt; nw = length(wa);
        if nw == 0
            [ok, Z, M] = simulate_silent(fp, m, dist, [], [], tt, [], [], cons, MAX_LOAD, init_s);
            if ok && (Z > bZ || (Z == bZ && M > bM))
                bZ = Z; bM = M; bP = fp; bWD = []; bPS = [];
            end
        else
            max_wk = zeros(1, nw);
            for j = 1:nw, max_wk(j) = max_work_with_park_silent(WM(ww(j)), rem_days); end
            sz = max_wk + 1; nc = prod(sz);
            for ci = 1:nc
                wd = zeros(1, nw); t2 = ci - 1;
                for j = nw:-1:1, wd(j) = mod(t2, sz(j)); t2 = floor(t2 / sz(j)); end
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > MAX_DAYS_EFF, continue; end
                [ok, Z, M] = simulate_silent(fp, m, dist, wa, ww, tt, wd, [], cons, MAX_LOAD, init_s);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bWD = wd; bPS = [];
                end
            end
        end
    end
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
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_silent(np2, nt, nwa, nww, ...
            bZ, bM, bP, bWD, bPS, dist, inter_idx, WY, WM, cons, nodes, MAX_DAYS_EFF, MAX_LOAD, init_s);
    end
end

function max_w = max_work_with_park_silent(mc, remaining)
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

function [feasible, Zf, Mf] = simulate_silent(pid, m, dist_all, wa, ww, tt, wdays, park_seg, ...
    cons, MAX_LOAD, init_s)
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
                        cO(day) = cons.WO; cH(day) = cons.WH; cF(day) = cons.WF; zG(day) = yld;
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
    O = init_s.O; H = init_s.H; F = init_s.F; M = init_s.M; Zf = init_s.Z;
    for t = 1:T_actual
        O = O - cO(t); H = H - cH(t); F = F - cF(t); Zf = Zf + zG(t);
        if O < -1e-6 || H < -1e-6 || F < -1e-6, feasible = false; Mf = 0; return; end
        if O + H + F > MAX_LOAD + 1e-6, feasible = false; Mf = 0; return; end
        if isSup(t)
            ns = T_actual + 1;
            for tt2 = (t+1):T_actual, if isSup(tt2), ns = tt2; break; end; end
            nO = 0; nH = 0; nF = 0;
            for tt2 = (t+1):ns
                if tt2 > T_actual, break; end
                nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
            end
            sp = MAX_LOAD - (O + H + F);
            bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);
            if bO + bH + bF > sp + 1e-6, feasible = false; Mf = 0; return; end
            if ns > T_actual && (O + bO < nO - 1e-6 || H + bH < nH - 1e-6 || F + bF < nF - 1e-6)
                feasible = false; Mf = 0; return;
            end
            cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
            if cost > M + 1e-6, feasible = false; Mf = 0; return; end
            O = O + bO; H = H + bH; F = F + bF; M = M - cost;
        end
    end
    feasible = true; Mf = M;
end

function wseq = generate_weather_mc(n_days, p_normal)
    wseq = repmat('N', 1, n_days);
    for i = 1:n_days
        if rand() > p_normal, wseq(i) = 'T'; end
    end
end
