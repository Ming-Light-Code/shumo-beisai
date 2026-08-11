% DEPRECATED: Use 优化版/solve_q3_cp_opt.m instead
% This file is kept for reference only.

function solve_q3_cp()
% =========================================================================
%  solve_q3_cp.m — 任务3 通用期望消耗约束规划求解器
%  =========================================================================
%  功能:  在30×30网格、90天时限、载重400下，使用期望消耗参数运行完整CP搜索，
%         自动探索所有可行路径骨架，找出 Z 最大（其次 M 最大）的离线最优方案。
%
%  数学模型:  路径骨架 P = (B, p1,..., pk, E), pi ∈ {W1,W2,W3,S1,S2}
%            工作天数 wj ∈ Z+, 停泊日 ⌈wj/WM_i⌉-1
%            约束: 载重≤400, 非负资源, 连续作业≤WM_i, 时间≤90天, 资金≥0
%            目标: max Z, then max M (词典序)
%
%  期望消耗:  E[move]=(3.2,3.2,2.2)  E[park]=(1.4,1.4,1.2)  E[work]=(5.6,4.4,3.6)
%            价格不变: p_O=2, p_H=1, p_F=2
%
%  CP搜索:   路径骨架递归 → 三剪枝 (上界/前向/去重) → 叶节点simulate评估
%  =========================================================================

% ===== Task 3 配置 =====
MAX_DAYS = 90;
MAX_LOAD = 400;
all_xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];  % B,E,W1,W2,W3,S1,S2
names   = {'B','E','W1','W2','W3','S1','S2'};
WY = [20, 15, 28];   % 各作业点日收益
WM = [4,  5,  3];     % 各作业点最大连续作业天数

% 初始状态
INIT_O = 100; INIT_H = 150; INIT_F = 100;
INIT_M = 750; INIT_Z = 200;

% 期望消耗参数: E[c] = 0.8*c_normal + 0.2*c_thunder
MO = 3.2; MH = 3.2; MF = 2.2;   % 移动 (每格)
PO = 1.4; PH = 1.4; PF = 1.2;   % 停泊 (每天)
WO = 5.6; WH = 4.4; WF = 3.6;   % 作业 (每天)
pO = 2;   pH = 1;   pF = 2;      % 补给价格

% ===== 距离矩阵 =====
n_pts = size(all_xy, 1);
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
    end
end
inter_idx = [3 4 5 6 7];  % W1,W2,W3,S1,S2

% ===== 输出配置信息 =====
fprintf('========================================\n');
fprintf('  任务3 通用期望消耗CP求解器\n');
fprintf('========================================\n');
fprintf('网格: 30×30 | 时限: %d天 | 载重上限: %d\n', MAX_DAYS, MAX_LOAD);
fprintf('初始: O=%d H=%d F=%d M=%d Z=%d\n', INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
fprintf('期望移动消耗: (%.1f, %.1f, %.1f) /格\n', MO, MH, MF);
fprintf('期望停泊消耗: (%.1f, %.1f, %.1f) /天\n', PO, PH, PF);
fprintf('期望作业消耗: (%.1f, %.1f, %.1f) /天\n', WO, WH, WF);
fprintf('补给价格: O=%d H=%d F=%d\n\n', pO, pH, pF);

fprintf('点位坐标:\n');
for i = 1:7
    fprintf('  %s: (%2d, %2d)', names{i}, all_xy(i,1), all_xy(i,2));
    if i >= 3 && i <= 5
        fprintf('  收益=%d/天  max连续=%d天', WY(i-2), WM(i-2));
    end
    fprintf('\n');
end
fprintf('\n');

% ===== CP搜索 =====
fprintf('开始CP搜索...\n');
tic;

% 打包消耗参数
cons = struct('MO',MO,'MH',MH,'MF',MF, ...
             'PO',PO,'PH',PH,'PF',PF, ...
             'WO',WO,'WH',WH,'WF',WF, ...
             'pO',pO,'pH',pH,'pF',pF);

% 打包初始状态
init = struct('O',INIT_O,'H',INIT_H,'F',INIT_F,'M',INIT_M,'Z',INIT_Z);

best_Z = -inf; best_M = -inf;
best_path = [1, 2];   % 默认: B→E (直行)
best_wd = []; best_ps = [];
nodes = 0;

[best_Z, best_M, best_path, best_wd, best_ps, nodes] = ...
    cp_search([1], 0, [], [], best_Z, best_M, best_path, best_wd, best_ps, ...
              dist, inter_idx, WY, WM, cons, MAX_DAYS, MAX_LOAD, init, nodes, 0);

elapsed = toc;

% ===== 输出最优解 =====
fprintf('\n===== 最优解 =====\n');
fprintf('Z = %d\n', best_Z);
fprintf('M = %.2f\n', best_M);

path_parts = cell(1, length(best_path));
for i = 1:length(best_path), path_parts{i} = names{best_path(i)}; end
fprintf('路径: %s\n', strjoin(path_parts, ' → '));

tt = 0;
for k = 1:(length(best_path)-1)
    tt = tt + dist(best_path(k), best_path(k+1));
end

% 计算作业和停泊详情
total_work = sum(best_wd);
total_park_reset = 0;
wp_count = 0;
for i = 2:length(best_path)
    if best_path(i) >= 3 && best_path(i) <= 5
        wp_count = wp_count + 1;
        if wp_count <= length(best_wd) && best_wd(wp_count) > 0
            wi = best_path(i) - 2;
            total_park_reset = total_park_reset + max(0, ceil(best_wd(wp_count)/WM(wi)) - 1);
        end
    end
end
total_park_sea = sum(best_ps);
fprintf('旅行: %d天 | 作业: %d天 | 停泊(重置): %d天 | 停泊(海上): %d天 | 总计: %d天\n', ...
    tt, total_work, total_park_reset, total_park_sea, ...
    tt + total_work + total_park_reset + total_park_sea);
fprintf('CP搜索节点: %d | 耗时: %.2f秒\n', nodes, elapsed);

% 打印作业安排
if ~isempty(best_wd)
    fprintf('作业安排:\n');
    wp_count = 0;
    for i = 2:length(best_path)
        if best_path(i) >= 3 && best_path(i) <= 5
            wp_count = wp_count + 1;
            if wp_count <= length(best_wd)
                wi = best_path(i) - 2;
                np = max(0, ceil(best_wd(wp_count)/WM(wi)) - 1);
                fprintf('  %s: %d天作业 + %d天停泊重置\n', names{best_path(i)}, best_wd(wp_count), np);
            end
        end
    end
end

% ===== 逐日日程表 =====
print_schedule_q3(best_path, best_wd, best_ps, dist, all_xy, names, WY, WM, cons, MAX_DAYS, MAX_LOAD, init);

fprintf('\nDone. 最优方案已输出。\n');
end

% =====================================================================
%  CP递归搜索
% =====================================================================
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search(path, tsf, wa, ww, ...
    bZ, bM, bP, bWD, bPS, dist, inter_idx, WY, WM, cons, MAX_DAYS, MAX_LOAD, init, nodes, depth)
% path: 当前路径骨架 (不含E)
% tsf:  累计旅行天数 (travel so far)
% wa:   作业点在path中的位置索引
% ww:   作业点类型 (1=W1, 2=W2, 3=W3)

    nodes = nodes + 1;
    lp = path(end);   % last point

    % ---- 1. 上界剪枝 ----
    if lp ~= 2
        dE = dist(lp, 2);          % 到E的距离
        rem_total = MAX_DAYS - tsf; % 剩余总天数
        if rem_total < dE, return; end  % 连E都到不了, 剪枝

        % 最乐观: 剩余天数全在W3作业 (收益28/天, WM=3)
        ub_work = max_work_with_park(3, rem_total - dE);
        ub_Z = init.Z + ub_work * 28;
        if ub_Z <= bZ && bZ > -inf, return; end
    end

    % ---- 2. 叶节点评估 ----
    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS
        fp = [path, 2];   % 完整路径 (含E)
        m = length(fp) - 2;  % 中间段数
        tt = 0;
        for k = 1:(m+1), tt = tt + dist(fp(k), fp(k+1)); end
        rem_days = MAX_DAYS - tt;
        nw = length(wa);  % 作业点数量

        if nw == 0
            % 无作业点: 直接评估路径可行性
            [ok, Z, M] = simulate_q3(fp, m, dist, [], [], tt, [], [], ...
                cons, MAX_LOAD, init);
            if ok && (Z > bZ || (Z == bZ && M > bM))
                bZ = Z; bM = M; bP = fp; bWD = []; bPS = [];
            end
        else
            % 枚举工作天数组合
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

                % 计算含停泊重置的总停留天数
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > MAX_DAYS, continue; end

                [ok, Z, M] = simulate_q3(fp, m, dist, wa, ww, tt, wd, [], ...
                    cons, MAX_LOAD, init);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bWD = wd; bPS = [];
                end
            end
        end
    end

    % ---- 3. 分支递归 ----
    for ni = 1:5
        np = inter_idx(ni);
        if np == lp, continue; end  % 相邻去重

        d = dist(lp, np);
        if tsf + d > MAX_DAYS, continue; end  % 走不到

        dE2 = dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS, continue; end  % 前向检查

        np2 = [path, np];
        nt = tsf + d;
        nwa = wa; nww = ww;
        if np >= 3 && np <= 5   % 是作业点
            nwa(end+1) = length(np2);
            nww(end+1) = np - 2;
        end

        [bZ, bM, bP, bWD, bPS, nodes] = cp_search(np2, nt, nwa, nww, ...
            bZ, bM, bP, bWD, bPS, dist, inter_idx, WY, WM, cons, MAX_DAYS, MAX_LOAD, init, nodes, depth+1);
    end
end

% =====================================================================
%  max_work_with_park: 计算剩余天数内最多作业天数
% =====================================================================
function max_w = max_work_with_park(mc, remaining)
% mc: max consecutive work days
% remaining: available days (can include park-reset days)
    best = 0;
    for k = 1:(remaining + 1)
        stay = k * mc + (k - 1);  % k sessions + (k-1) park days
        if stay > remaining, break; end
        best = k * mc;
        slack = remaining - stay;
        if slack >= 1
            best = max(best, k * mc + min(mc, slack - 1));
        end
    end
    max_w = max(best, min(mc, remaining));
end

% =====================================================================
%  simulate_q3: 期望消耗路径模拟
% =====================================================================
function [feasible, Zf, Mf] = simulate_q3(pid, m, dist_all, wa, ww, tt, wdays, park_seg, ...
    cons, MAX_LOAD, init)
% 模拟整条路径的资源消耗和补给决策 (浮点期望消耗版)

    if isempty(wdays), wdays = []; end
    if isempty(park_seg), park_seg = zeros(1, m+1); end

    % 本地WM/WY (用于simulate内部)
    WM_local = [4, 5, 3];
    WY_local = [20, 15, 28];

    % 构建消耗数组
    T_alloc = tt + sum(wdays) + sum(park_seg) + 100;
    cO = zeros(1, T_alloc); cH = zeros(1, T_alloc); cF = zeros(1, T_alloc);
    zG = zeros(1, T_alloc); isSup = false(1, T_alloc);

    day = 0;
    for k = 1:(m+1)
        d = dist_all(pid(k), pid(k+1));

        % a. 海上停泊
        for pd = 1:park_seg(k)
            day = day + 1;
            cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
        end

        % b. 移动
        for dd = 1:d
            day = day + 1;
            cO(day) = cons.MO; cH(day) = cons.MH; cF(day) = cons.MF;
            if dd == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7  % S1 or S2
                    isSup(day) = true;
                end
            end
        end

        % c. 作业 (含停泊重置)
        if ~isempty(wa)
            wk = find(wa == k+1, 1);
            if ~isempty(wk) && ~isempty(wdays) && wk <= length(wdays) && wdays(wk) > 0
                mc = WM_local(ww(wk)); yld = WY_local(ww(wk));
                rem_val = wdays(wk);
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
                        zG(day) = 0;
                    end
                end
            end
        end
    end

    T_actual = day;

    % 逐日模拟
    O = init.O; H = init.H; F = init.F;
    M = init.M; Zf = init.Z;

    for t = 1:T_actual
        % 扣除当日消耗 (先扣再补给, 与任务1/2一致)
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        Zf = Zf + zG(t);

        % 非负检查 (浮点容差)
        if O < -1e-6 || H < -1e-6 || F < -1e-6
            feasible = false; Mf = 0; return;
        end

        % 载重检查
        if O + H + F > MAX_LOAD + 1e-6
            feasible = false; Mf = 0; return;
        end

        % 补给日处理
        if isSup(t)
            % 找下一补给日
            ns = T_actual + 1;
            for tt2 = (t+1):T_actual
                if isSup(tt2), ns = tt2; break; end
            end

            % 计算直到下一补给日的需求
            nO = 0; nH = 0; nF = 0;
            for tt2 = (t+1):ns
                if tt2 > T_actual, break; end
                nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
            end

            % 可用载重空间
            sp = MAX_LOAD - (O + H + F);
            % 需采购量
            bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);

            if bO + bH + bF > sp + 1e-6
                feasible = false; Mf = 0; return;
            end

            % 如果是最后一段补给, 确保采购足够到E
            if ns > T_actual
                if O + bO < nO - 1e-6 || H + bH < nH - 1e-6 || F + bF < nF - 1e-6
                    feasible = false; Mf = 0; return;
                end
            end

            cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
            if cost > M + 1e-6
                feasible = false; Mf = 0; return;
            end

            O = O + bO; H = H + bH; F = F + bF;
            M = M - cost;
        end
    end

    feasible = true;
    Mf = M;
end

% =====================================================================
%  print_schedule_q3: 打印逐日日程
% =====================================================================
function print_schedule_q3(bP, bWD, bPS, dist, all_xy, names, WY, WM, cons, MAX_DAYS, MAX_LOAD, init)
    m = length(bP) - 2;
    if m < 0
        fprintf('  无可行的路径。\n');
        return;
    end
    if isempty(bPS), bPS = zeros(1, m+1); end

    % 重建 wa/ww
    wa = []; ww = [];
    for i = 2:length(bP)
        if bP(i) >= 3 && bP(i) <= 5
            wa(end+1) = i;
            ww(end+1) = bP(i) - 2;
        end
    end

    % 计算总旅行天数
    tt = 0;
    for k = 1:(m+1), tt = tt + dist(bP(k), bP(k+1)); end

    % 重建消耗数组
    T_max = tt + sum(bWD) + sum(bPS) + 100;
    cO = zeros(1, T_max); cH = zeros(1, T_max); cF = zeros(1, T_max);
    zG = zeros(1, T_max); isSup = false(1, T_max);

    day = 0;
    for k = 1:(m+1)
        d = dist(bP(k), bP(k+1));
        for pd = 1:bPS(k)
            day = day + 1;
            cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
        end
        for dd = 1:d
            day = day + 1;
            cO(day) = cons.MO; cH(day) = cons.MH; cF(day) = cons.MF;
            if dd == d
                to_pt = bP(k+1);
                if to_pt == 6 || to_pt == 7, isSup(day) = true; end
            end
        end
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && bWD(wk) > 0
            mc = WM(ww(wk)); yld = WY(ww(wk)); rem_val = bWD(wk);
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
    T_actual = day;

    % 逐日回放
    fprintf('\n===== 逐日日程表 =====\n');
    fprintf('Day   | Pos (x,y)    | Action           |   O    H    F   Load |    Z       M\n');
    fprintf('------|---------------|------------------|---------------------|--------------\n');

    O = init.O; H = init.H; F = init.F;
    M = init.M; Z = init.Z;
    day2 = 0;

    for k = 1:(m+1)
        fr = bP(k); to = bP(k+1); d = dist(fr, to);
        fr_xy = all_xy(fr, :); to_xy = all_xy(to, :);

        % 海上停泊
        for pd = 1:bPS(k)
            day2 = day2 + 1;
            O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
            if mod(day2, 10) == 1 || day2 <= 5 || day2 >= T_actual - 5
                fprintf('%5d | (%2d,%2d)       | park(at sea)     | %4.0f %4.0f %4.0f %5.0f | %5d %8.0f\n', ...
                    day2, fr_xy(1), fr_xy(2), O, H, F, O+H+F, Z, M);
            end
        end

        % 移动
        dx_total = to_xy(1) - fr_xy(1); dy_total = to_xy(2) - fr_xy(2);
        steps_x = abs(dx_total); steps_y = abs(dy_total);
        for dd = 1:d
            day2 = day2 + 1;
            if dd <= steps_x
                x = fr_xy(1) + sign(dx_total) * dd;
                y = fr_xy(2);
            else
                x = to_xy(1);
                y = fr_xy(2) + sign(dy_total) * (dd - steps_x);
            end
            O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);

            show = (mod(day2, 10) == 1 || day2 <= 5 || day2 >= T_actual - 5);
            if isSup(day2)
                % 补给
                ns = T_actual + 1;
                for tt2 = (day2+1):T_actual
                    if isSup(tt2), ns = tt2; break; end
                end
                nO = 0; nH = 0; nF = 0;
                for tt2 = (day2+1):ns
                    if tt2 > T_actual, break; end
                    nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
                end
                bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);
                cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
                M = M - cost; O = O + bO; H = H + bH; F = F + bF;
                fprintf('%5d | (%2d,%2d)       | SUPPLY(%s)       | %4.0f %4.0f %4.0f %5.0f | %5d %8.0f  (+O%.0f H%.0f F%.0f)\n', ...
                    day2, x, y, names{to}, O, H, F, O+H+F, Z, M, bO, bH, bF);
            elseif show
                fprintf('%5d | (%2d,%2d)       | move             | %4.0f %4.0f %4.0f %5.0f | %5d %8.0f\n', ...
                    day2, x, y, O, H, F, O+H+F, Z, M);
            end
        end

        % 作业
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && bWD(wk) > 0
            mc = WM(ww(wk)); yld = WY(ww(wk)); rem_val = bWD(wk);
            pt_name = names{to};
            while rem_val > 0
                chunk = min(rem_val, mc);
                for w = 1:chunk
                    day2 = day2 + 1;
                    O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
                    Z = Z + zG(day2);
                    fprintf('%5d | (%2d,%2d)       | work(%s)         | %4.0f %4.0f %4.0f %5.0f | %5d %8.0f\n', ...
                        day2, to_xy(1), to_xy(2), pt_name, O, H, F, O+H+F, Z, M);
                end
                rem_val = rem_val - chunk;
                if rem_val > 0
                    day2 = day2 + 1;
                    O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
                    fprintf('%5d | (%2d,%2d)       | park(reset)      | %4.0f %4.0f %4.0f %5.0f | %5d %8.0f\n', ...
                        day2, to_xy(1), to_xy(2), O, H, F, O+H+F, Z, M);
                end
            end
        end
    end

    fprintf('------|---------------|------------------|---------------------|--------------\n');
    fprintf('  抵达E: Z=%d M=%.0f Day=%d\n', Z, M, day2);
end
