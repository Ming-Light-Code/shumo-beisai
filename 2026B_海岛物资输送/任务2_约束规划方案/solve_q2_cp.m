function solve_q2_cp()
% solve_q2_cp.m - 约束规划方法求解任务2（30天全雷暴极端天气）
% 基于 solve_cp.m 的CP框架，适配雷暴消耗参数
% 核心机制：值域缩减 + 约束传播 + 分支回溯 + 三种剪枝
%
% 雷暴天气消耗参数：
%   移动(每格): O=8, H=4, F=3
%   停泊(每天): O=3, H=3, F=2
%   作业(每天): O=8, H=6, F=6
% 补给价格不变: O=2, H=1, F=2

MAX_DAYS = 30;
MAX_LOAD = 120;
INIT_O = 35; INIT_H = 45; INIT_F = 30;
INIT_M = 240; INIT_Z = 100;

% 网格坐标: B, E, W1, W2, W3, S1, S2
all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
n_pts = size(all_xy, 1);

% 预计算曼哈顿距离矩阵
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i, j) = abs(all_xy(i,1) - all_xy(j,1)) + abs(all_xy(i,2) - all_xy(j,2));
    end
end

% 可访问的中间点索引 (W1,W2,W3,S1,S2)
inter_idx = [3 4 5 6 7];
names = {'B','E','W1','W2','W3','S1','S2'};

% 作业点收益与单次连续作业上限
WY = [20, 15, 28];   % W1,W2,W3 每日收益
WM = [4, 5, 3];       % 单次最大连续作业天数

% ---- 雷暴消耗参数 ----
MO_move = 8;  MH_move = 4;  MF_move = 3;
PO_park = 3;  PH_park = 3;  PF_park = 2;
WO_work = 8;  WH_work = 6;  WF_work = 6;
pO_price = 2; pH_price = 1; pF_price = 2;

fprintf('========================================\n');
fprintf('  CP for Task 2: 30-Day Thunderstorm\n');
fprintf('========================================\n');
fprintf('Move cost per cell: O=%d H=%d F=%d\n', MO_move, MH_move, MF_move);
fprintf('Park cost per day:  O=%d H=%d F=%d\n', PO_park, PH_park, PF_park);
fprintf('Work cost per day:  O=%d H=%d F=%d\n', WO_work, WH_work, WF_work);
fprintf('Initial: O=%d H=%d F=%d M=%d Z=%d Load<=%d\n\n', INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, MAX_LOAD);

% 连通性分析
fprintf('--- Connectivity Analysis ---\n');
fprintf('B->S1: %d cells, O_need=%d <= %d (REACHABLE)\n', dist(1,6), dist(1,6)*MO_move, INIT_O);
fprintf('B->S2: %d cells, O_need=%d > %d (UNREACHABLE)\n', dist(1,7), dist(1,7)*MO_move, INIT_O);
for wi = 1:3
    w_pt = wi + 2;
    d_bw = dist(1, w_pt);
    fprintf('B->W%d: %d cells, O_need=%d %s\n', wi, d_bw, d_bw*MO_move, ...
        iif(d_bw*MO_move <= INIT_O, '(REACHABLE)', '(UNREACHABLE)'));
end
fprintf('\n');

% 全局最优解
best_Z = -inf; best_M = -inf;
best_path = [1, 2];
best_work_days = [];
best_park_seg = zeros(1, 1);
best_sched = struct();
nodes = 0;

% 将消耗参数打包传入
cons = struct('MO', MO_move, 'MH', MH_move, 'MF', MF_move, ...
              'PO', PO_park, 'PH', PH_park, 'PF', PF_park, ...
              'WO', WO_work, 'WH', WH_work, 'WF', WF_work, ...
              'pO', pO_price, 'pH', pH_price, 'pF', pF_price);

% ---- CP 递归搜索 ----
[best_Z, best_M, best_path, best_work_days, best_park_seg, best_sched, nodes] = ...
    cp_search_q2([1], 0, [], [], best_Z, best_M, best_path, best_work_days, best_park_seg, ...
                 best_sched, dist, inter_idx, WY, WM, cons, nodes, 0, MAX_DAYS, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);

% ---- 输出最优解 ----
fprintf('\n===== OPTIMAL SOLUTION (CP) =====\n');
fprintf('Z = %d\n', best_Z);
fprintf('M = %d\n', best_M);
pstr = '';
for i = 1:length(best_path)
    pstr = [pstr, ' ', names{best_path(i)}];
end
fprintf('Path: %s\n', pstr);

tt = 0;
for k = 1:(length(best_path)-1)
    tt = tt + dist(best_path(k), best_path(k+1));
end
total_park = sum(best_park_seg);
fprintf('Travel: %d days | Park at sea: %d days | Work: %d days | Total: %d days\n', ...
    tt, total_park, sum(best_work_days), tt + total_park + sum(best_work_days));
fprintf('Nodes explored: %d\n', nodes);

print_schedule_q2(best_path, best_work_days, best_park_seg, dist, WM, WY, cons, ...
                  MAX_DAYS, MAX_LOAD, all_xy, names, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);

fprintf('\nDone.\n');
end

function [bZ, bM, bP, bWD, bPS, bSched, nodes] = cp_search_q2(...
    path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, bSched, ...
    dist, inter_idx, WY, WM, cons, nodes, depth, MAX_DAYS, MAX_LOAD, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    nodes = nodes + 1;
    lp = path(end);

    if lp ~= 2
        dE = dist(lp, 2);
        rem = MAX_DAYS - tsf;
        if rem < dE
            return;
        end
        max_w = max_work_with_park(3, rem - dE);
        ub = INIT_Z + real(max_w) * 28;
        if ub <= bZ && bZ > -inf
            return;
        end
    end

    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS
        fp = [path, 2];
        m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1)
            tt = tt + dist(fp(k), fp(k+1));
        end
        
        rem_days = MAX_DAYS - tt;
        nw = length(wa);

        if nw == 0
            [bZ, bM, bP, bWD, bPS, bSched] = eval_leaf_no_work(...
                fp, m, tt, rem_days, dist, bZ, bM, bP, bWD, bPS, bSched, ...
                cons, MAX_DAYS, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
        else
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = max_work_with_park(WM(ww(j)), rem_days);
            end
            sz = max_wk + 1;
            nc = prod(sz);
            for ci = 1:nc
                wd = zeros(1, nw);
                t2 = ci - 1;
                for j = nw:-1:1
                    wd(j) = mod(t2, sz(j));
                    t2 = floor(t2 / sz(j));
                end
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > MAX_DAYS
                    continue;
                end
                park_rem = MAX_DAYS - tt - total_stay;
                [bZ, bM, bP, bWD, bPS, bSched] = eval_leaf_with_work_park(...
                    fp, m, tt, wd, park_rem, dist, wa, ww, bZ, bM, bP, bWD, bPS, bSched, ...
                    cons, MAX_DAYS, MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
            end
        end
    end

    for ni = 1:5
        np = inter_idx(ni);
        if np == lp
            continue;
        end
        d = dist(lp, np);
        if tsf + d > MAX_DAYS
            continue;
        end
        dE2 = dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS
            continue;
        end
        np2 = [path, np];
        nt = tsf + d;
        nwa = wa;
        nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2);
            nww(end+1) = np - 2;
        end
        [bZ, bM, bP, bWD, bPS, bSched, nodes] = cp_search_q2(...
            np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, bSched, ...
            dist, inter_idx, WY, WM, cons, nodes, depth+1, MAX_DAYS, MAX_LOAD, ...
            INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
    end
end

function [bZ, bM, bP, bWD, bPS, bSched] = eval_leaf_no_work(...
    fp, m, tt, rem_days, dist, bZ, bM, bP, bWD, bPS, bSched, ...
    cons, MAX_DAYS, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    n_seg = m + 1;  % 实际段数
    % 枚举实际段数+1（额外段吸收未使用天数），使各段停泊天数之和可 <= rem_days
    park_combs = enumerate_park_combinations(n_seg + 1, rem_days);
    
    for ci = 1:size(park_combs, 1)
        ps = park_combs(ci, 1:n_seg);  % 丢弃额外段
        [ok, Z, M, ~] = gsim_q2(fp, m, dist, [], [], tt, [], ps, cons, ...
            MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
        if ok && (Z > bZ || (Z == bZ && M > bM))
            bZ = Z; bM = M; bP = fp;
            bWD = []; bPS = ps;
        end
    end
end

function [bZ, bM, bP, bWD, bPS, bSched] = eval_leaf_with_work_park(...
    fp, m, tt, wd, park_rem, dist, wa, ww, bZ, bM, bP, bWD, bPS, bSched, ...
    cons, MAX_DAYS, MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    n_seg = m + 1;
    park_combs = enumerate_park_combinations(n_seg, park_rem);
    
    for ci = 1:size(park_combs, 1)
        ps = park_combs(ci, :);
        total_stay = sum(ps);
        for j = 1:length(wd)
            if wd(j) > 0
                total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/WM(ww(j))) - 1);
            end
        end
        if tt + total_stay > MAX_DAYS
            continue;
        end
        [ok, Z, M, ~] = gsim_q2(fp, m, dist, wa, ww, tt, wd, ps, cons, ...
            MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
        if ok && (Z > bZ || (Z == bZ && M > bM))
            bZ = Z; bM = M; bP = fp;
            bWD = wd; bPS = ps;
        end
    end
end

function combs = enumerate_park_combinations(n_seg, max_total)
% 使用嵌套函数实现正确的值返回递归枚举
% 将 max_total 个停泊日分配到 n_seg 个段中，每段 >=0，总和 <= max_total
% 但实际使用"精确等于 max_total"（允许不充分使用某段）
    combs = zeros(0, n_seg);
    current = zeros(1, n_seg);
    enumerate_rec(1, max_total);

    function enumerate_rec(pos, remain)
        if pos == n_seg
            current(pos) = remain;
            combs(end+1, :) = current;
            return;
        end
        for p = 0:remain
            current(pos) = p;
            enumerate_rec(pos + 1, remain - p);
        end
    end
end

function [feasible, Zf, Mf, sched] = gsim_q2(pid, m, dist_all, wa, ww, tt, wdays, park_seg, ...
    cons, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    T_alloc = tt + sum(wdays) + sum(park_seg) + 100;
    cO = zeros(1, T_alloc);
    cH = zeros(1, T_alloc);
    cF = zeros(1, T_alloc);
    zG = zeros(1, T_alloc);
    isSup = false(1, T_alloc);
    
    day = 0;
    
    for k = 1:(m+1)
        d = dist_all(pid(k), pid(k+1));
        % 1. Park at sea BEFORE moving (reduces load before supply point)
        for pd = 1:park_seg(k)
            day = day + 1;
            cO(day) = cons.PO;
            cH(day) = cons.PH;
            cF(day) = cons.PF;
        end
        % 2. Move to destination
        for dd = 1:d
            day = day + 1;
            cO(day) = cons.MO;
            cH(day) = cons.MH;
            cF(day) = cons.MF;
            if dd == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7
                    isSup(day) = true;
                end
            end
        end
        % 3. Work (at destination, after supply will trigger on move arrival day)
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && wdays(wk) > 0
            wm_val = 3;
            if ww(wk) == 1, wm_val = 4; end
            if ww(wk) == 2, wm_val = 5; end
            yld = 0;
            if ww(wk) == 1, yld = 20; end
            if ww(wk) == 2, yld = 15; end
            if ww(wk) == 3, yld = 28; end
            
            rem_val = wdays(wk);
            while rem_val > 0
                chunk = min(rem_val, wm_val);
                for w = 1:chunk
                    day = day + 1;
                    cO(day) = cons.WO;
                    cH(day) = cons.WH;
                    cF(day) = cons.WF;
                    zG(day) = yld;
                end
                rem_val = rem_val - chunk;
                if rem_val > 0
                    day = day + 1;
                    cO(day) = cons.PO;
                    cH(day) = cons.PH;
                    cF(day) = cons.PF;
                    zG(day) = 0;
                end
            end
        end
    end
    
    T_actual = day;
    
    O = INIT_O; H = INIT_H; F = INIT_F;
    M = INIT_M; Zf = INIT_Z;
    
    for t = 1:T_actual
        O = O - cO(t);
        H = H - cH(t);
        F = F - cF(t);
        Zf = Zf + zG(t);
        
        if O < 0 || H < 0 || F < 0
            feasible = false; Mf = 0; sched = struct();
            return;
        end
        
        if O + H + F > MAX_LOAD + 1e-9
            feasible = false; Mf = 0; sched = struct();
            return;
        end
        
        if isSup(t)
            ns = T_actual + 1;
            for tt2 = (t+1):T_actual
                if isSup(tt2)
                    ns = tt2;
                    break;
                end
            end
            
            nO = 0; nH = 0; nF = 0;
            for tt2 = (t+1):ns
                if tt2 > T_actual, break; end
                nO = nO + cO(tt2);
                nH = nH + cH(tt2);
                nF = nF + cF(tt2);
            end
            
            sp = MAX_LOAD - (O + H + F);
            bO = max(0, nO - O);
            bH = max(0, nH - H);
            bF = max(0, nF - F);
            
            if bO + bH + bF > sp
                feasible = false; Mf = 0; sched = struct();
                return;
            end
            
            if ns > T_actual && (O + bO < nO || H + bH < nH || F + bF < nF)
                feasible = false; Mf = 0; sched = struct();
                return;
            end
            
            cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
            if cost > M
                feasible = false; Mf = 0; sched = struct();
                return;
            end
            
            O = O + bO; H = H + bH; F = F + bF;
            M = M - cost;
        end
    end
    
    feasible = true;
    Mf = M;
    sched = struct();
end

function max_w = max_work_with_park(mc, remaining)
    best = 0;
    for k = 1:(remaining + 1)
        stay = k * mc + (k - 1);
        if stay > remaining
            break;
        end
        best = k * mc;
        slack = remaining - stay;
        if slack >= 1
            best = max(best, k * mc + min(mc, slack - 1));
        end
    end
    max_w = max(best, min(mc, remaining));
end

function s = iif(cond, t, f)
    if cond, s = t; else, s = f; end
end

function print_schedule_q2(bP, bWD, bPS, dist, WM, WY, cons, ...
    MAX_DAYS, MAX_LOAD, all_xy, names, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    m = length(bP) - 2;
    if m < 0
        fprintf('  No feasible path found.\n');
        return;
    end
    % 防御性处理：确保 bPS 大小正确
    if isempty(bPS)
        bPS = zeros(1, m+1);
    end
    wa = []; ww = [];
    for i = 2:length(bP)
        if bP(i) >= 3 && bP(i) <= 5
            wa(end+1) = i;
            ww(end+1) = bP(i) - 2;
        end
    end
    
    tt = 0;
    for k = 1:(m+1)
        tt = tt + dist(bP(k), bP(k+1));
    end
    
    T_max = tt + sum(bWD) + sum(bPS) + 100;
    cO = zeros(1, T_max);
    cH = zeros(1, T_max);
    cF = zeros(1, T_max);
    zG = zeros(1, T_max);
    isSup = false(1, T_max);
    
    day = 0;
    for k = 1:(m+1)
        d = dist(bP(k), bP(k+1));
        % 1. Park at sea
        for pd = 1:bPS(k)
            day = day + 1;
            cO(day) = cons.PO;
            cH(day) = cons.PH;
            cF(day) = cons.PF;
        end
        % 2. Move
        for dd = 1:d
            day = day + 1;
            cO(day) = cons.MO;
            cH(day) = cons.MH;
            cF(day) = cons.MF;
            if dd == d
                to_pt = bP(k+1);
                if to_pt == 6 || to_pt == 7
                    isSup(day) = true;
                end
            end
        end
        % 3. Work
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && bWD(wk) > 0
            wm_val = 3;
            if ww(wk) == 1, wm_val = 4; end
            if ww(wk) == 2, wm_val = 5; end
            yld = 0;
            if ww(wk) == 1, yld = 20; end
            if ww(wk) == 2, yld = 15; end
            if ww(wk) == 3, yld = 28; end
            rem_val = bWD(wk);
            while rem_val > 0
                chunk = min(rem_val, wm_val);
                for w = 1:chunk
                    day = day + 1;
                    cO(day) = cons.WO;
                    cH(day) = cons.WH;
                    cF(day) = cons.WF;
                    zG(day) = yld;
                end
                rem_val = rem_val - chunk;
                if rem_val > 0
                    day = day + 1;
                    cO(day) = cons.PO;
                    cH(day) = cons.PH;
                    cF(day) = cons.PF;
                end
            end
        end
    end
    
    T_actual = day;
    
    fprintf('\n===== DAY-BY-DAY SCHEDULE =====\n');
    fprintf('Day | Pos (x,y)  | Action         |  O   H   F  Load |   Z     M\n');
    fprintf('----|-------------|----------------|------------------|------------\n');
    
    O = INIT_O; H = INIT_H; F = INIT_F;
    M = INIT_M; Z = INIT_Z;
    day2 = 0;
    
    for k = 1:(m+1)
        fr = bP(k);
        to = bP(k+1);
        d = dist(fr, to);
        
        % 1. Park at sea (at current position, before moving)
        for pd = 1:bPS(k)
            day2 = day2 + 1;
            O = O - cO(day2);
            H = H - cH(day2);
            F = F - cF(day2);
            fprintf('%3d | (%2d,%2d)     | park(at sea)  | %3d %3d %3d %4d | %4d %5d\n', ...
                day2, all_xy(fr, 1), all_xy(fr, 2), O, H, F, O+H+F, Z, M);
        end
        
        % 2. Move
        for dd = 1:d
            day2 = day2 + 1;
            dx_total = all_xy(to, 1) - all_xy(fr, 1);
            dy_total = all_xy(to, 2) - all_xy(fr, 2);
            steps_x = abs(dx_total);
            steps_y = abs(dy_total);
            if dd <= steps_x
                x = all_xy(fr, 1) + sign(dx_total) * dd;
                y = all_xy(fr, 2);
            else
                x = all_xy(to, 1);
                y = all_xy(fr, 2) + sign(dy_total) * (dd - steps_x);
            end
            O = O - cO(day2);
            H = H - cH(day2);
            F = F - cF(day2);
            
            if isSup(day2)
                ns = T_actual + 1;
                for tt2 = (day2+1):T_actual
                    if isSup(tt2), ns = tt2; break; end
                end
                nO = 0; nH = 0; nF = 0;
                for tt2 = (day2+1):ns
                    if tt2 > T_actual, break; end
                    nO = nO + cO(tt2);
                    nH = nH + cH(tt2);
                    nF = nF + cF(tt2);
                end
                bO = max(0, nO - O);
                bH = max(0, nH - H);
                bF = max(0, nF - F);
                cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
                M = M - cost;
                O = O + bO; H = H + bH; F = F + bF;
                fprintf('%3d | (%2d,%2d)     | SUPPLY(%s)    | %3d %3d %3d %4d | %4d %5d  (+O%d H%d F%d)\n', ...
                    day2, x, y, names{to}, O, H, F, O+H+F, Z, M, bO, bH, bF);
            else
                fprintf('%3d | (%2d,%2d)     | move          | %3d %3d %3d %4d | %4d %5d\n', ...
                    day2, x, y, O, H, F, O+H+F, Z, M);
            end
        end
        
        % 3. Work
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && bWD(wk) > 0
            wm_val = 3;
            if ww(wk) == 1, wm_val = 4; end
            if ww(wk) == 2, wm_val = 5; end
            yld = 0;
            if ww(wk) == 1, yld = 20; end
            if ww(wk) == 2, yld = 15; end
            if ww(wk) == 3, yld = 28; end
            rem_val = bWD(wk);
            while rem_val > 0
                chunk = min(rem_val, wm_val);
                for w = 1:chunk
                    day2 = day2 + 1;
                    O = O - cO(day2);
                    H = H - cH(day2);
                    F = F - cF(day2);
                    Z = Z + zG(day2);
                    fprintf('%3d | (%2d,%2d)     | work(%s)      | %3d %3d %3d %4d | %4d %5d\n', ...
                        day2, all_xy(to, 1), all_xy(to, 2), names{to}, O, H, F, O+H+F, Z, M);
                end
                rem_val = rem_val - chunk;
                if rem_val > 0
                    day2 = day2 + 1;
                    O = O - cO(day2);
                    H = H - cH(day2);
                    F = F - cF(day2);
                    fprintf('%3d | (%2d,%2d)     | park(reset)   | %3d %3d %3d %4d | %4d %5d\n', ...
                        day2, all_xy(to, 1), all_xy(to, 2), O, H, F, O+H+F, Z, M);
                end
            end
        end
    end
    
    fprintf('----|-------------|----------------|------------------|------------\n');
    fprintf('  Final at E: Z=%d M=%d Day=%d\n', Z, M, day2);
end
