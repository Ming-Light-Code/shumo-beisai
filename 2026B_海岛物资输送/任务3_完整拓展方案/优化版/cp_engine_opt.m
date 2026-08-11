function varargout = cp_engine_opt(action, varargin)
% =========================================================================
%  cp_engine_opt.m — 任务3 优化版公共CP引擎
%  三个求解器共用的核心函数库，消除代码重复
%
%  改进: P0差异化裕度 | P1预防性超购 | P2生存优先
% =========================================================================

switch action
    case 'plan'
        [varargout{1}, varargout{2}, varargout{3}, varargout{4}] = plan_from_state(varargin{:});
    case 'simulate'
        [varargout{1}, varargout{2}, varargout{3}] = simulate_path(varargin{:});
    case 'max_work'
        varargout{1} = max_work_with_park(varargin{:});
    case 'gen_weather'
        varargout{1} = generate_weather_seq(varargin{:});
    case 'task3_config'
        varargout{1} = get_task3_config();
    case 'plan_scenario'
        [varargout{1}, varargout{2}, varargout{3}, varargout{4}] = plan_scenario(varargin{:});
    case 'get_supply_needs'
        [varargout{1}, varargout{2}, varargout{3}] = get_supply_needs(varargin{:});
    case 'get_cons'
        varargout{1} = get_consumption_params(varargin{:});
    otherwise
        error('cp_engine_opt: unknown action "%s"', action);
end
end

% =====================================================================
%  任务3配置 (单一定义点)
% =====================================================================
function cfg = get_task3_config()
    cfg.MAX_DAYS = 90;
    cfg.MAX_LOAD = 400;
    cfg.all_xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
    cfg.names = {'B','E','W1','W2','W3','S1','S2'};
    cfg.WY = [20, 15, 28];
    cfg.WM = [4, 5, 3];
    cfg.init = struct('O',100,'H',150,'F',100,'M',750,'Z',200);
    cfg.inter_idx = [3 4 5 6 7];

    n = size(cfg.all_xy, 1);
    cfg.dist = zeros(n);
    for i = 1:n, for j = 1:n
        cfg.dist(i,j) = abs(cfg.all_xy(i,1)-cfg.all_xy(j,1)) + abs(cfg.all_xy(i,2)-cfg.all_xy(j,2));
    end; end
end

% =====================================================================
%  P0: 差异化安全裕度
% =====================================================================
function cons = get_consumption_params(mode, cfg)
% mode: 'expected' | 'normal' | 'thunder'
    if nargin < 2, cfg = get_task3_config(); end
    SAFETY_O = 1.05; SAFETY_H = 1.05; SAFETY_F = 1.05;

    switch mode
        case 'expected'
            cons.MO = 3.2 * SAFETY_O; cons.MH = 3.2 * SAFETY_H; cons.MF = 2.2 * SAFETY_F;
            cons.PO = 1.4 * SAFETY_O; cons.PH = 1.4 * SAFETY_H; cons.PF = 1.2 * SAFETY_F;
            cons.WO = 5.6 * SAFETY_O; cons.WH = 4.4 * SAFETY_H; cons.WF = 3.6 * SAFETY_F;
        case 'expected_pes'
            PES = 1.15;  % 有效裕度 = SAFETY(1.05) × PES(1.15) ≈ 1.2075
            cons.MO = 3.2 * SAFETY_O * PES; cons.MH = 3.2 * SAFETY_H * PES; cons.MF = 2.2 * SAFETY_F * PES;
            cons.PO = 1.4 * SAFETY_O * PES; cons.PH = 1.4 * SAFETY_H * PES; cons.PF = 1.2 * SAFETY_F * PES;
            cons.WO = 5.6 * SAFETY_O * PES; cons.WH = 4.4 * SAFETY_H * PES; cons.WF = 3.6 * SAFETY_F * PES;
        case 'normal'
            cons.MO = 2; cons.MH = 3; cons.MF = 2;
            cons.PO = 1; cons.PH = 1; cons.PF = 1;
            cons.WO = 5; cons.WH = 4; cons.WF = 3;
        case 'thunder'
            cons.MO = 8; cons.MH = 4; cons.MF = 3;
            cons.PO = 3; cons.PH = 3; cons.PF = 2;
            cons.WO = 8; cons.WH = 6; cons.WF = 6;
    end
    cons.pO = 2; cons.pH = 1; cons.pF = 2;
    cons.SAFETY_O = SAFETY_O; cons.SAFETY_H = SAFETY_H; cons.SAFETY_F = SAFETY_F;
end

% =====================================================================
%  CP规划入口
% =====================================================================
function [best_path, best_parks, best_works, feasible] = plan_from_state(cur_pt, elapsed, cons, cfg, survival_mode, init_s)
% 从当前状态 cur_pt 出发规划剩余路径
% survival_mode=true 时不含作业点
    if nargin < 5, survival_mode = false; end
    if nargin < 6, init_s = cfg.init; end

    best_Z = -inf; best_M = -inf;
    best_path = [cur_pt, 2]; best_works = []; best_parks = [];
    nodes = 0;
    eff_days = cfg.MAX_DAYS - elapsed;

    [best_Z, best_M, best_path, best_works, best_parks, nodes] = ...
        cp_search_engine([cur_pt], 0, [], [], best_Z, best_M, best_path, best_works, best_parks, ...
        cfg, cons, nodes, eff_days, init_s);
    feasible = (best_Z > -inf);
end

% =====================================================================
%  P3: 真正场景CP — 单路径三场景加权评估
%  策略: 用期望消耗找最优路径骨架，然后在正常/期望/悲观三种天气假设下
%        评估同一条路径，取加权期望Z作为决策依据。
%  权重: 正常0.3, 期望0.5, 悲观0.2 (反映天气分布+风险规避)
% =====================================================================
function [best_path, best_parks, best_works, feasible] = plan_scenario(cur_pt, elapsed, cfg, survival_mode, init_s)
    if nargin < 4, survival_mode = false; end
    if nargin < 5, init_s = cfg.init; end
    cons_list = {get_consumption_params('normal',cfg), get_consumption_params('expected',cfg), get_consumption_params('expected_pes',cfg)};
    weights = [0.3, 0.5, 0.2];  % normal/expected/pessimistic

    % 步骤1: 用期望消耗找到最优路径骨架
    [pth, prk, wrk, ok] = plan_from_state(cur_pt, elapsed, cons_list{2}, cfg, survival_mode, init_s);
    if ~ok
        feasible = false; best_path = [cur_pt, 2]; best_works = []; best_parks = [];
        return;
    end

    % 步骤2: 在同一条路径上用三种消耗场景评估
    m = length(pth)-2; tt = 0;
    for k = 1:(m+1), tt = tt + cfg.dist(pth(k), pth(k+1)); end
    wa = []; ww = [];
    for i = 2:length(pth)
        if pth(i)>=3 && pth(i)<=5, wa(end+1)=i; ww(end+1)=pth(i)-2; end
    end

    weighted_Z = 0; weighted_M = 0;
    for sc = 1:3
        [fe, Z, M] = simulate_path(pth, m, cfg.dist, wa, ww, tt, wrk, prk, cons_list{sc}, cfg, init_s);
        weighted_Z = weighted_Z + weights(sc) * Z;
        weighted_M = weighted_M + weights(sc) * M;
    end

    best_path = pth; best_parks = prk; best_works = wrk;
    feasible = true;
end

% =====================================================================
%  P4: 得到下一补给点的综合资源需求(含移动+作业+停泊)
% =====================================================================
function [needO, needH, needF] = get_supply_needs(plan_path, plan_parks, plan_works, plan_leg, cons, cfg)
    rem_t = 0; rem_p = 0; workO = 0; workH = 0; workF = 0;
    wp_count = 0;
    for k = plan_leg:length(plan_path)-2
        rem_t = rem_t + cfg.dist(plan_path(k+1), plan_path(k+2));
        if k+1 <= length(plan_parks), rem_p = rem_p + plan_parks(k+1); end
        if plan_path(k+1) >= 3 && plan_path(k+1) <= 5
            wp_count = wp_count + 1;
            if wp_count <= length(plan_works)
                wd = plan_works(wp_count); wi = plan_path(k+1) - 2;
                np = max(0, ceil(wd/cfg.WM(wi)) - 1);
                workO = workO + wd*cons.WO + np*cons.PO;
                workH = workH + wd*cons.WH + np*cons.PH;
                workF = workF + wd*cons.WF + np*cons.PF;
            end
        end
    end
    needO = rem_t*cons.MO + rem_p*cons.PO + workO;
    needH = rem_t*cons.MH + rem_p*cons.PH + workH;
    needF = rem_t*cons.MF + rem_p*cons.PF + workF;
end

% =====================================================================
%  CP递归搜索 (与任务1/2框架一致)
% =====================================================================
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_engine(path, tsf, wa, ww, ...
    bZ, bM, bP, bWD, bPS, cfg, cons, nodes, MAX_DAYS_EFF, init_s)

    nodes = nodes + 1; lp = path(end);

    % === 上界剪枝 ===
    if lp ~= 2
        dE = cfg.dist(lp, 2); rem_total = MAX_DAYS_EFF - tsf;
        if rem_total < dE, return; end
        ub_work = max_work_with_park(3, rem_total - dE);
        ub_Z = init_s.Z + ub_work * 28;  % 上界偏松(忽略到W3的旅行时间), 但保证不误剪
        if ub_Z <= bZ && bZ > -inf, return; end
    end

    % === 叶节点评估 ===
    dE = cfg.dist(lp, 2);
    if tsf + dE <= MAX_DAYS_EFF
        fp = [path, 2]; m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1), tt = tt + cfg.dist(fp(k), fp(k+1)); end
        rem_days = MAX_DAYS_EFF - tt; nw = length(wa);

        if nw == 0
            [ok, Z, M] = simulate_path(fp, m, cfg.dist, [], [], tt, [], [], cons, cfg, init_s);
            if ok && (Z > bZ || (Z == bZ && M > bM))
                bZ = Z; bM = M; bP = fp; bWD = []; bPS = [];
            end
        else
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = max_work_with_park(cfg.WM(ww(j)), rem_days);
            end
            sz = max_wk + 1; nc = prod(sz);
            for ci = 1:nc
                wd = zeros(1, nw); t2 = ci - 1;
                for j = nw:-1:1, wd(j) = mod(t2, sz(j)); t2 = floor(t2/sz(j)); end
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/cfg.WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > MAX_DAYS_EFF, continue; end
                [ok, Z, M] = simulate_path(fp, m, cfg.dist, wa, ww, tt, wd, [], cons, cfg, init_s);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bWD = wd; bPS = [];
                end
            end
        end
    end

    % === 分支递归 ===
    for ni = 1:5
        np = cfg.inter_idx(ni);
        if np == lp, continue; end
        d = cfg.dist(lp, np);
        if tsf + d > MAX_DAYS_EFF, continue; end
        dE2 = cfg.dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS_EFF, continue; end
        np2 = [path, np]; nt = tsf + d;
        nwa = wa; nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2); nww(end+1) = np - 2;
        end
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_engine(np2, nt, nwa, nww, ...
            bZ, bM, bP, bWD, bPS, cfg, cons, nodes, MAX_DAYS_EFF, init_s);
    end
end

% =====================================================================
%  路径模拟 (含P1预防性超购)
% =====================================================================
function [feasible, Zf, Mf] = simulate_path(pid, m, dist_all, wa, ww, tt, wdays, park_seg, ...
    cons, cfg, init_s)

    if isempty(wdays), wdays = []; end
    if isempty(park_seg), park_seg = zeros(1, m+1); end
    if nargin < 11, cfg = get_task3_config(); end
    if nargin < 12, init_s = cfg.init; end

    T_alloc = tt + sum(wdays) + sum(park_seg) + 100;
    cO = zeros(1, T_alloc); cH = zeros(1, T_alloc); cF = zeros(1, T_alloc);
    zG = zeros(1, T_alloc); isSup = false(1, T_alloc);

    day = 0;
    for k = 1:(m+1)
        d = dist_all(pid(k), pid(k+1));
        for pd = 1:park_seg(k)
            day = day + 1;
            cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
        end
        for dd = 1:d
            day = day + 1;
            cO(day) = cons.MO; cH(day) = cons.MH; cF(day) = cons.MF;
            if dd == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7, isSup(day) = true; end
            end
        end
        if ~isempty(wa)
            wk = find(wa == k+1, 1);
            if ~isempty(wk) && ~isempty(wdays) && wk <= length(wdays) && wdays(wk) > 0
                mc = cfg.WM(ww(wk)); yld = cfg.WY(ww(wk)); rem_val = wdays(wk);
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
    last_supply_idx = find(isSup, 1, 'last');
    if isempty(last_supply_idx), last_supply_idx = 0; end

    for t = 1:T_actual
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        Zf = Zf + zG(t);
        if O < -1e-6 || H < -1e-6 || F < -1e-6
            feasible = false; Mf = 0; return;
        end
        if O + H + F > cfg.MAX_LOAD + 1e-6
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
            sp = cfg.MAX_LOAD - (O + H + F);
            bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);

            % P1: 预防性超购——在基础采购上追加10%剩余空间作为缓冲
            is_last_supply = (t == last_supply_idx);
            if ~is_last_supply
                spare_after_basic = sp - (bO + bH + bF);
                if spare_after_basic > 1e-6
                    buffer_total = 0.10 * spare_after_basic;
                    bO = bO + buffer_total * (cons.MO / (cons.MO + cons.MH + cons.MF));
                    bH = bH + buffer_total * (cons.MH / (cons.MO + cons.MH + cons.MF));
                    bF = bF + buffer_total * (cons.MF / (cons.MO + cons.MH + cons.MF));
                end
            end

            if bO + bH + bF > sp + 1e-6
                feasible = false; Mf = 0; return;
            end
            if ns > T_actual && (O + bO < nO - 1e-6 || H + bH < nH - 1e-6 || F + bF < nF - 1e-6)
                feasible = false; Mf = 0; return;
            end
            cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
            if cost > M + 1e-6
                feasible = false; Mf = 0; return;
            end
            O = O + bO; H = H + bH; F = F + bF;
            M = M - cost;
        end
    end
    feasible = true; Mf = M;
end

% =====================================================================
%  工作天数上界
% =====================================================================
function max_w = max_work_with_park(mc, remaining)
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

% =====================================================================
%  随机天气序列生成
% =====================================================================
function wseq = generate_weather_seq(n_days, p_normal)
    wseq = repmat('N', 1, n_days);
    for i = 1:n_days
        if rand() > p_normal, wseq(i) = 'T'; end
    end
end
