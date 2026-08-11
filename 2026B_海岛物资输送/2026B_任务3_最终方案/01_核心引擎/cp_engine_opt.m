function varargout = cp_engine_opt(action, varargin)
% =========================================================================
%  cp_engine_opt.m — 任务3 优化版公共CP引擎 (v2.0 修复版)
%  三个求解器共用的核心函数库，消除代码重复
%
%  修复: P0差异化裕度(SAFETY_O=1.25) | get_supply_needs索引 | plan_scenario评分生效
%         plan_from_candidate(支持指定首目标) | simulate_with_weather(在线天气模拟)
% =========================================================================

switch action
    case 'plan'
        [varargout{1:nargout}] = plan_from_state(varargin{:});
    case 'simulate'
        [varargout{1:nargout}] = simulate_path(varargin{:});
    case 'max_work'
        varargout{1} = max_work_with_park(varargin{:});
    case 'gen_weather'
        varargout{1} = generate_weather_seq(varargin{:});
    case 'task3_config'
        varargout{1} = get_task3_config();
    case 'plan_scenario'
        [varargout{1:nargout}] = plan_scenario(varargin{:});
    case 'get_supply_needs'
        [varargout{1:nargout}] = get_supply_needs(varargin{:});
    case 'get_cons'
        varargout{1} = get_consumption_params(varargin{:});
    case 'simulate_with_weather'
        [varargout{1:nargout}] = simulate_with_weather(varargin{:});
    case 'plan_from_candidate'
        [varargout{1:nargout}] = plan_from_candidate(varargin{:});
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
%  P0: 差异化安全裕度 (FIXED: 按雷暴放大倍数分配)
%  移动时O雷暴/正常=4x, H=1.3x, F=1.5x → O需要最大裕度
% =====================================================================
function cons = get_consumption_params(mode, cfg)
    if nargin < 2, cfg = get_task3_config(); end
    % 差异化安全裕度: O(燃料)受雷暴影响最大, 给更高裕度
    SAFETY_O = 1.25;  % 燃料: 移动4倍放大, 停泊3倍
    SAFETY_H = 1.05;  % 淡水: 最大放大1.5倍
    SAFETY_F = 1.05;  % 食物: 最大放大2倍

    switch mode
        case 'expected'
            cons.MO = 3.2 * SAFETY_O; cons.MH = 3.2 * SAFETY_H; cons.MF = 2.2 * SAFETY_F;
            cons.PO = 1.4 * SAFETY_O; cons.PH = 1.4 * SAFETY_H; cons.PF = 1.2 * SAFETY_F;
            cons.WO = 5.6 * SAFETY_O; cons.WH = 4.4 * SAFETY_H; cons.WF = 3.6 * SAFETY_F;
        case 'expected_pes'
            PES = 1.15;
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
%  plan_from_candidate: 从cur_pt出发, 强制下一站为target_pt, 然后CP搜索剩余
% =====================================================================
function [best_path, best_parks, best_works, feasible] = plan_from_candidate(cur_pt, target_pt, elapsed, cons, cfg, init_s)
    if nargin < 6, init_s = cfg.init; end
    d = cfg.dist(cur_pt, target_pt);
    eff_days = cfg.MAX_DAYS - elapsed;
    if d > eff_days
        best_path = [cur_pt, 2]; best_parks = []; best_works = []; feasible = false;
        return;
    end

    % 扣除旅行消耗后从target_pt规划剩余路径
    travel_O = d * cons.MO; travel_H = d * cons.MH; travel_F = d * cons.MF;
    travel_init = struct('O', max(0, init_s.O - travel_O), ...
                         'H', max(0, init_s.H - travel_H), ...
                         'F', max(0, init_s.F - travel_F), ...
                         'M', init_s.M, 'Z', init_s.Z);
    [rest_path, rest_parks, rest_works, ok] = ...
        plan_from_state(target_pt, elapsed + d, cons, cfg, false, travel_init);
    if ~ok
        best_path = [cur_pt, 2]; best_parks = []; best_works = []; feasible = false;
        return;
    end

    % 组装完整路径: cur_pt → target_pt → rest
    % rest_path第一个元素就是target_pt, 需要去掉重复
    if rest_path(1) == target_pt
        best_path = [cur_pt, rest_path];
    else
        best_path = [cur_pt, target_pt, rest_path];
    end
    % 在cur_pt→target_pt段之前没有停泊
    best_parks = [0, rest_parks];
    best_works = rest_works;
    feasible = true;
end

% =====================================================================
%  P3修复: 真正场景CP — 多候选路径三场景加权评估
%  策略: 用期望消耗/正常消耗/悲观消耗分别搜索最优路径,
%        每条的加权期望Z作为决策依据, 选最优者返回
% =====================================================================
function [best_path, best_parks, best_works, feasible] = plan_scenario(cur_pt, elapsed, cfg, survival_mode, init_s)
    if nargin < 4, survival_mode = false; end
    if nargin < 5, init_s = cfg.init; end
    cons_list = {get_consumption_params('normal',cfg), get_consumption_params('expected',cfg), get_consumption_params('expected_pes',cfg)};
    weights = [0.2, 0.6, 0.2];  % normal/expected/pessimistic (60%% baseline, 20%% tails)

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
    scenario_ok = true;
    for sc = 1:3
        [fe, Z, M] = simulate_path(pth, m, cfg.dist, wa, ww, tt, wrk, prk, cons_list{sc}, cfg, init_s);
        if ~fe
            % 悲观场景不可行: 尝试用更保守的规划 (survival_mode)
            if sc == 3
                [pth2, prk2, wrk2, ok2] = plan_from_state(cur_pt, elapsed, cons_list{3}, cfg, true, init_s);
                if ok2
                    % 对保守路径重新评估全部三个场景
                    m2 = length(pth2)-2; tt2 = 0;
                    for k = 1:(m2+1), tt2 = tt2 + cfg.dist(pth2(k), pth2(k+1)); end
                    wa2 = []; ww2 = [];
                    for i = 2:length(pth2)
                        if pth2(i)>=3 && pth2(i)<=5, wa2(end+1)=i; ww2(end+1)=pth2(i)-2; end
                    end
                    % 重新计算 weighted_Z (覆盖之前的混合评估)
                    weighted_Z = 0; weighted_M = 0;
                    all_ok = true;
                    for sc2 = 1:3
                        [fe2, Z2, M2] = simulate_path(pth2, m2, cfg.dist, wa2, ww2, tt2, wrk2, prk2, cons_list{sc2}, cfg, init_s);
                        if ~fe2
                            all_ok = false; break;
                        end
                        weighted_Z = weighted_Z + weights(sc2) * Z2;
                        weighted_M = weighted_M + weights(sc2) * M2;
                    end
                    if all_ok
                        pth = pth2; prk = prk2; wrk = wrk2;
                        % 跳过外层sc循环中的后续场景(已完成三场景评估)
                        break;
                    end
                end
                scenario_ok = false;
                break;
            end
            weighted_Z = weighted_Z + weights(sc) * Z;
            weighted_M = weighted_M + weights(sc) * M;
        else
            weighted_Z = weighted_Z + weights(sc) * Z;
            weighted_M = weighted_M + weights(sc) * M;
        end
    end

    % 步骤3: 与简单路径(直赴E)比较加权得分
    [fe_direct, Zd, Md] = simulate_path([cur_pt, 2], 0, cfg.dist, [], [], ...
        cfg.dist(cur_pt, 2), [], [], cons_list{2}, cfg, init_s);
    direct_weighted = 0;
    if fe_direct
        for sc = 1:3
            [fes, Zs, Ms] = simulate_path([cur_pt, 2], 0, cfg.dist, [], [], ...
                cfg.dist(cur_pt, 2), [], [], cons_list{sc}, cfg, init_s);
            if fes
                direct_weighted = direct_weighted + weights(sc) * Zs;
            end
        end
    end

    if ~scenario_ok || (direct_weighted > weighted_Z && fe_direct)
        % 回退到保守方案
        [pth, prk, wrk, ok] = plan_from_state(cur_pt, elapsed, cons_list{2}, cfg, true, init_s);
        if ~ok
            feasible = false; best_path = [cur_pt, 2]; best_works = []; best_parks = [];
            return;
        end
    end

    best_path = pth; best_parks = prk; best_works = wrk;
    feasible = true;
end

% =====================================================================
%  P4修复: 得到下一补给点的综合资源需求
%  增加 completed_wp 参数, 跳过已完成的作业点
% =====================================================================
function [needO, needH, needF] = get_supply_needs(plan_path, plan_parks, plan_works, plan_leg, cons, cfg, completed_wp)
    if nargin < 7, completed_wp = 0; end
    rem_t = 0; rem_p = 0; workO = 0; workH = 0; workF = 0;
    wp_count = 0;
    for k = plan_leg:length(plan_path)-2
        rem_t = rem_t + cfg.dist(plan_path(k+1), plan_path(k+2));
        if k+1 <= length(plan_parks), rem_p = rem_p + plan_parks(k+1); end
        if plan_path(k+1) >= 3 && plan_path(k+1) <= 5
            wp_count = wp_count + 1;
            actual_idx = completed_wp + wp_count;
            if actual_idx <= length(plan_works)
                wd = plan_works(actual_idx); wi = plan_path(k+1) - 2;
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
%  新增: 在线天气轨迹模拟 (用于MDP Rollout)
%  给定完整路径计划 + 实际天气序列, 模拟执行并返回结果
% =====================================================================
function [Zf, Mf, arrived] = simulate_with_weather(plan_path, plan_parks, plan_works, ...
    cur_pt, elapsed, init_s, cfg, cons_N, cons_T, weather_seq)
    if isempty(plan_parks), plan_parks = zeros(1, length(plan_path)-1); end
    if isempty(plan_works), plan_works = []; end

    % 计算期望消耗用于补给需求估算 (而非正常天气消耗)
    cons_exp_supply = get_consumption_params('expected', cfg);

    O = init_s.O; H = init_s.H; F = init_s.F;
    M = init_s.M; Z = init_s.Z;
    day_offset = 0;
    pt = cur_pt;
    leg = 1;

    % 重建作业点映射
    wa_idx = []; ww_type = [];
    for ii = 2:length(plan_path)
        if plan_path(ii) >= 3 && plan_path(ii) <= 5
            wa_idx(end+1) = ii; ww_type(end+1) = plan_path(ii) - 2;
        end
    end
    wp_order = 0;

    % 如果当前已在作业点, 且plan中第一个作业点匹配当前位置
    if cur_pt >= 3 && cur_pt <= 5 && ~isempty(wa_idx) && plan_path(wa_idx(1)) == cur_pt
        wp_order = 1;
        if 1 <= length(plan_works) && plan_works(1) > 0
            wi = cur_pt - 2; wd = plan_works(1);
            npark = max(0, ceil(wd / cfg.WM(wi)) - 1);
            for wday = 1:wd
                day_offset = day_offset + 1;
                if day_offset > length(weather_seq), arrived = false; Zf = Z; Mf = M; return; end
                if weather_seq(day_offset) == 'T'
                    O = O - cons_T.WO; H = H - cons_T.WH; F = F - cons_T.WF;
                else
                    O = O - cons_N.WO; H = H - cons_N.WH; F = F - cons_N.WF;
                end
                Z = Z + cfg.WY(wi);
                if O < -1e-6 || H < -1e-6 || F < -1e-6, arrived = false; Zf = 0; Mf = 0; return; end
            end
            for pd = 1:npark
                day_offset = day_offset + 1;
                if day_offset > length(weather_seq), arrived = false; Zf = Z; Mf = M; return; end
                if weather_seq(day_offset) == 'T'
                    O = O - cons_T.PO; H = H - cons_T.PH; F = F - cons_T.PF;
                else
                    O = O - cons_N.PO; H = H - cons_N.PH; F = F - cons_N.PF;
                end
                if O < -1e-6 || H < -1e-6 || F < -1e-6, arrived = false; Zf = 0; Mf = 0; return; end
            end
        end
        leg = 2;
    end

    % 沿路径逐段移动 + 作业
    while leg <= length(plan_path) - 1
        next_pt = plan_path(leg + 1);
        dd = cfg.dist(pt, next_pt);

        % 段前停泊
        if leg <= length(plan_parks)
            for pd = 1:plan_parks(leg)
                day_offset = day_offset + 1;
                if day_offset > length(weather_seq), arrived = false; Zf = Z; Mf = M; return; end
                if weather_seq(day_offset) == 'T'
                    O = O - cons_T.PO; H = H - cons_T.PH; F = F - cons_T.PF;
                else
                    O = O - cons_N.PO; H = H - cons_N.PH; F = F - cons_N.PF;
                end
                if O < -1e-6 || H < -1e-6 || F < -1e-6, arrived = false; Zf = 0; Mf = 0; return; end
            end
        end

        % 移动
        for s = 1:dd
            day_offset = day_offset + 1;
            if day_offset > length(weather_seq), arrived = false; Zf = Z; Mf = M; return; end
            if weather_seq(day_offset) == 'T'
                O = O - cons_T.MO; H = H - cons_T.MH; F = F - cons_T.MF;
            else
                O = O - cons_N.MO; H = H - cons_N.MH; F = F - cons_N.MF;
            end
            if O < -1e-6 || H < -1e-6 || F < -1e-6, arrived = false; Zf = 0; Mf = 0; return; end
        end

        pt = next_pt;

        % 到达补给点: 采购补给
        if pt == 6 || pt == 7
            % 计算剩余需求 (从当前leg到终点)
            rem_t = 0; rem_p = 0; workO = 0; workH = 0; workF = 0;
            wp_cnt = 0;
            for k = leg:length(plan_path)-2
                rem_t = rem_t + cfg.dist(plan_path(k+1), plan_path(k+2));
                if k+1 <= length(plan_parks), rem_p = rem_p + plan_parks(k+1); end
                if plan_path(k+1) >= 3 && plan_path(k+1) <= 5
                    wp_cnt = wp_cnt + 1;
                    actual_idx = wp_order + wp_cnt;
                    if actual_idx <= length(plan_works)
                        wd = plan_works(actual_idx); wi = plan_path(k+1) - 2;
                        np = max(0, ceil(wd/cfg.WM(wi)) - 1);
                        workO = workO + wd*cons_exp_supply.WO + np*cons_exp_supply.PO;
                        workH = workH + wd*cons_exp_supply.WH + np*cons_exp_supply.PH;
                        workF = workF + wd*cons_exp_supply.WF + np*cons_exp_supply.PF;
                    end
                end
            end
            needO = rem_t*cons_exp_supply.MO + rem_p*cons_exp_supply.PO + workO;
            needH = rem_t*cons_exp_supply.MH + rem_p*cons_exp_supply.PH + workH;
            needF = rem_t*cons_exp_supply.MF + rem_p*cons_exp_supply.PF + workF;
            sp = cfg.MAX_LOAD - (O + H + F);
            bO = max(0, needO - O); bH = max(0, needH - H); bF = max(0, needF - F);
            if bO + bH + bF > sp + 1e-6
                sc = sp / (bO + bH + bF);
                bO = bO * sc; bH = bH * sc; bF = bF * sc;
            end
            cost = bO * cons_N.pO + bH * cons_N.pH + bF * cons_N.pF;
            if cost <= M + 1e-6
                O = O + bO; H = H + bH; F = F + bF; M = M - cost;
            end
        end

        % 作业处理
        wk_found = find(wa_idx == leg+1, 1);
        if ~isempty(wk_found) && wk_found > wp_order && wk_found <= length(plan_works)
            wd = plan_works(wk_found); wi = pt - 2;
            if wd > 0
                npark = max(0, ceil(wd / cfg.WM(wi)) - 1);
                for wday = 1:wd
                    day_offset = day_offset + 1;
                    if day_offset > length(weather_seq), arrived = false; Zf = Z; Mf = M; return; end
                    if weather_seq(day_offset) == 'T'
                        O = O - cons_T.WO; H = H - cons_T.WH; F = F - cons_T.WF;
                    else
                        O = O - cons_N.WO; H = H - cons_N.WH; F = F - cons_N.WF;
                    end
                    Z = Z + cfg.WY(wi);
                    if O < -1e-6 || H < -1e-6 || F < -1e-6, arrived = false; Zf = 0; Mf = 0; return; end
                end
                for pd = 1:npark
                    day_offset = day_offset + 1;
                    if day_offset > length(weather_seq), arrived = false; Zf = Z; Mf = M; return; end
                    if weather_seq(day_offset) == 'T'
                        O = O - cons_T.PO; H = H - cons_T.PH; F = F - cons_T.PF;
                    else
                        O = O - cons_N.PO; H = H - cons_N.PH; F = F - cons_N.PF;
                    end
                    if O < -1e-6 || H < -1e-6 || F < -1e-6, arrived = false; Zf = 0; Mf = 0; return; end
                end
            end
        end

        leg = leg + 1;
    end

    arrived = true; Zf = Z; Mf = M;
end

% =====================================================================
%  CP递归搜索 (与任务1/2框架一致)
% =====================================================================
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_engine(path, tsf, wa, ww, ...
    bZ, bM, bP, bWD, bPS, cfg, cons, nodes, MAX_DAYS_EFF, init_s)

    nodes = nodes + 1; lp = path(end);

    % === 上界剪枝 (收紧: 加入各工作点旅行时间) ===
    if lp ~= 2
        dE = cfg.dist(lp, 2); rem_total = MAX_DAYS_EFF - tsf;
        if rem_total < dE, return; end
        % 最乐观上界: 分别假设去每个工作点, 扣除旅行时间后计算最大Z
        best_ub_Z = init_s.Z;
        for wi = 1:3
            wp_idx = cfg.inter_idx(wi);
            d_to_wp = cfg.dist(lp, wp_idx);
            d_wp_to_E = cfg.dist(wp_idx, 2);
            days_at_wp = rem_total - d_to_wp - d_wp_to_E;
            if days_at_wp > 0
                ub_work = max_work_with_park(cfg.WM(wi), days_at_wp);
                ub_Z_wp = init_s.Z + ub_work * cfg.WY(wi);
                if ub_Z_wp > best_ub_Z, best_ub_Z = ub_Z_wp; end
            end
        end
        if best_ub_Z <= bZ && bZ > -inf, return; end
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
        if slack >= 1, best = max(best, k * mc + min(mc, slack - 1));
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
