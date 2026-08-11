function solve_q2_online()
% solve_q2_online.m - 在线CP决策模型求解任务2
% ============================================
% 总体框架：
%   步骤1: 观测当日天气 w_t
%   步骤2: CP从当前状态搜索剩余天数内最优路径骨架
%   步骤3: 执行路径骨架的今日动作 (移动/停泊/作业/补给)
%   步骤4: 状态转移，返回步骤1
%
% 全雷暴极端情形: w_t = 雷暴 ∀t，模型退化为确定性CP (同 solve_q2_cp.m)
%
% 在线特性: 每日观测天气后重新规划，适应天气变化

MAX_DAYS = 30; MAX_LOAD = 120;
INIT_O = 35; INIT_H = 45; INIT_F = 30;
INIT_M = 240; INIT_Z = 100;

all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
n_pts = size(all_xy, 1);
names = {'B','E','W1','W2','W3','S1','S2'};
WY = [20, 15, 28];
WM = [4, 5, 3];

% 地图距离矩阵
dist_map = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist_map(i, j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
    end
end

% 消耗参数表
cons = struct(...
    'normal',   struct('MO',2,'MH',3,'MF',2,'PO',1,'PH',1,'PF',1,'WO',5,'WH',4,'WF',3,'pO',2,'pH',1,'pF',2), ...
    'thunder',  struct('MO',8,'MH',4,'MF',3,'PO',3,'PH',3,'PF',2,'WO',8,'WH',6,'WF',6,'pO',2,'pH',1,'pF',2));

% 极端天气序列
weather_seq = ones(1, MAX_DAYS) * 2;  % 2=雷暴

fprintf('========================================\n');
fprintf('  Task 2: Online CP Decision Model\n');
fprintf('========================================\n');
fprintf('Weather: 30 consecutive thunderstorm days\n');
fprintf('Model: State S_t = (pos, O,H,F, M,Z, c, w_t)\n');
fprintf('       Observe w_t -> CP search -> execute a_t -> S_{t+1}\n\n');

% 初始化状态
cur_pt = 1;   % B
pos = all_xy(1, :);  % [1, 5]
O = INIT_O; H = INIT_H; F = INIT_F;
M = INIT_M; Z = INIT_Z;
consec = 0; day = 0;

% 当前CP规划 (首次规划，后续每日可选重规划)
plan_path = [];      % 路径骨架
plan_parks = [];     % 各段停泊日
plan_works = [];     % 工作天
leg_idx = 1;         % 当前段
day_in_leg = 0;      % 本段已过天数
parked_in_leg = 0;   % 本段已停泊天数
supplied_today = false;

fprintf('--- Day-by-Day Decision ---\n');
fprintf('Day | Weather  | Action         | Pos (x,y)  |  O   H   F  Load |   Z     M\n');
fprintf('----|----------|----------------|------------|------------------|------------\n');

while day < MAX_DAYS
    day = day + 1;
    w = weather_seq(day);
    w_name = 'thunder';
    c = cons.thunder;
    
    detail_str = '';
    
    % ---- 需要重新规划? ----
    if isempty(plan_path) || (cur_pt ~= 1 && day_in_leg <= 0 && parked_in_leg <= 0)
        % 到达命名点位，调用CP搜索剩余最优路径
        [plan_path, plan_parks, plan_works, feasible] = ...
            cp_plan_from_state(cur_pt, day-1, O, H, F, M, Z, consec, ...
                               c, MAX_DAYS, MAX_LOAD, dist_map, all_xy, WY, WM, INIT_Z);
        leg_idx = 1;
        day_in_leg = 0;
        parked_in_leg = 0;
        supplied_today = false;
        
        if ~feasible
            fprintf('%3d | %s    | NO FEASIBLE    | (%2d,%2d)     | %3d %3d %3d %4d | %4d %5d\n', ...
                day, w_name, pos(1), pos(2), O, H, F, O+H+F, Z, M);
            break;
        end
    end
    
    if length(plan_path) < 2
        % 已在E或路径无效
        break;
    end
    
    next_pt = plan_path(leg_idx + 1);
    
    % ---- 决策: 本日动作 ----
    if leg_idx <= length(plan_parks) && parked_in_leg < plan_parks(leg_idx)
        % 本段还有未完成的停泊日
        action = 'park(at sea)';
        O = O - c.PO; H = H - c.PH; F = F - c.PF;
        consec = 0;
        parked_in_leg = parked_in_leg + 1;
        
    elseif day_in_leg < dist_map(cur_pt, next_pt) - 1
        % 移动中 (非最后一天)
        action = 'move';
        O = O - c.MO; H = H - c.MH; F = F - c.MF;
        consec = 0;
        day_in_leg = day_in_leg + 1;
        % 更新中间位置
        [pos] = move_toward(pos, all_xy(next_pt, :));
        
    elseif day_in_leg == dist_map(cur_pt, next_pt) - 1
        % 到达目标点的最后一步
        action = 'move';
        O = O - c.MO; H = H - c.MH; F = F - c.MF;
        consec = 0;
        day_in_leg = day_in_leg + 1;
        pos = all_xy(next_pt, :);
        cur_pt = next_pt;
        
        % 到达补给点: 当日采购
        if cur_pt == 6 || cur_pt == 7
            supplied_today = true;
            % 计算到达E所需资源
            rem_travel = 0;
            rem_park = 0;
            for k = (leg_idx+1):(length(plan_path)-1)
                rem_travel = rem_travel + dist_map(plan_path(k), plan_path(k+1));
                if k+1 <= length(plan_parks)
                    rem_park = rem_park + plan_parks(k+1);
                end
            end
            needO = rem_travel * c.MO + rem_park * c.PO;
            needH = rem_travel * c.MH + rem_park * c.PH;
            needF = rem_travel * c.MF + rem_park * c.PF;
            sp = MAX_LOAD - (O + H + F);
            bO = max(0, needO - O);
            bH = max(0, needH - H);
            bF = max(0, needF - F);
            if bO+bH+bF <= sp
                cost = bO*c.pO + bH*c.pH + bF*c.pF;
                if cost <= M
                    O = O + bO; H = H + bH; F = F + bF;
                    M = M - cost;
                    detail_str = sprintf('(+O%d H%d F%d cost=%d)', bO, bH, bF, cost);
                    action = sprintf('SUPPLY(%s)', names{cur_pt});
                end
            end
        end
        
        % 检查是否到达E
        if cur_pt == 2
            action = 'ARRIVE at E';
        end
        
        % 推进到下一段
        leg_idx = leg_idx + 1;
        day_in_leg = 0;
        parked_in_leg = 0;
        
    else
        % 在目标点停留 (作业或等待)
        wk_idx = find(cur_pt == [3,4,5], 1);
        if ~isempty(wk_idx) && ~isempty(plan_works) && ...
           sum(plan_works) > 0
            action = sprintf('work(%s)', names{cur_pt});
            O = O - c.WO; H = H - c.WH; F = F - c.WF;
            Z = Z + WY(wk_idx);
            consec = consec + 1;
        else
            action = 'park';
            O = O - c.PO; H = H - c.PH; F = F - c.PF;
            consec = 0;
        end
    end
    
    fprintf('%3d | %s    | %-14s | (%2d,%2d)     | %3d %3d %3d %4d | %4d %5d', ...
        day, w_name, action, pos(1), pos(2), O, H, F, O+H+F, Z, M);
    if ~isempty(detail_str)
        fprintf('  %s', detail_str);
    end
    fprintf('\n');
    
    if cur_pt == 2
        break;
    end
end

fprintf('----|----------|----------------|------------|------------------|------------\n');

fprintf('\n===== FINAL RESULT (Extreme: 30-day Thunderstorm) =====\n');
if cur_pt == 2
    fprintf('Status: Arrived at E on day %d\n', day);
else
    fprintf('Status: Did NOT reach E within %d days\n', MAX_DAYS);
end
fprintf('Z = %d  (target materials)\n', Z);
fprintf('M = %d  (remaining funds)\n', M);
fprintf('Load at E: O=%d H=%d F=%d Total=%d\n', O, H, F, O+H+F);
fprintf('\nDone.\n');
end

% ---- CP规划函数: 从当前状态搜索最优路径骨架 ----
function [best_path, best_parks, best_works, feasible] = ...
    cp_plan_from_state(cur_pt, elapsed, O, H, F, M, Z, consec, ...
                        cons, MAX_DAYS, MAX_LOAD, dist, all_xy, WY, WM, INIT_Z)
    inter_idx = [3 4 5 6 7];
    
    best_Z = -inf; best_M_val = -inf;
    best_path = [cur_pt, 2];
    best_parks = [];
    best_works = [];
    nodes = 0;
    
    [best_Z, best_M_val, best_path, best_works, best_parks, nodes] = ...
        cp_search_online2([cur_pt], elapsed, [], [], best_Z, best_M_val, ...
                          best_path, best_works, best_parks, ...
                          dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, ...
                          O, H, F, M, Z, cons, nodes, INIT_Z);
    
    feasible = (best_Z > -inf);
    if ~feasible
        best_path = [cur_pt, 2];
        best_parks = [];
        best_works = [];
    end
end

% ---- CP递归搜索 ----
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online2(...
    path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, ...
    dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, ...
    O_init, H_init, F_init, M_init, Z_init, cons, nodes, INIT_Z)

    nodes = nodes + 1;
    lp = path(end);
    
    % 上界剪枝
    if lp ~= 2
        dE = dist(lp, 2);
        rem = MAX_DAYS - tsf;
        if rem < dE, return; end
        max_w = max_work_with_park2(3, rem - dE);
        ub = Z_init + real(max_w) * 28;
        if ub <= bZ && bZ > -inf, return; end
    end
    
    % 叶节点评估
    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS
        fp = [path, 2];
        m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1), tt = tt + dist(fp(k), fp(k+1)); end
        
        rem_days = MAX_DAYS - tt;
        nw = length(wa);
        
        if nw == 0
            [bZ, bM, bP, bWD, bPS] = eval_leaf_online2(...
                fp, m, tt, rem_days, dist, [], bZ, bM, bP, bWD, bPS, ...
                cons, MAX_LOAD, O_init, H_init, F_init, M_init, Z_init);
        else
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = max_work_with_park2(WM(ww(j)), rem_days);
            end
            sz = max_wk + 1; nc = prod(sz);
            for ci = 1:nc
                wd = zeros(1, nw); t2 = ci - 1;
                for j = nw:-1:1, wd(j) = mod(t2, sz(j)); t2 = floor(t2/sz(j)); end
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/WM(ww(j)))-1);
                    end
                end
                if tt + total_stay > MAX_DAYS, continue; end
                park_rem = MAX_DAYS - tt - total_stay;
                [bZ, bM, bP, bWD, bPS] = eval_leaf_online2(...
                    fp, m, tt, park_rem, dist, wd, bZ, bM, bP, bWD, bPS, ...
                    cons, MAX_LOAD, O_init, H_init, F_init, M_init, Z_init);
            end
        end
    end
    
    % 分支
    for ni = 1:5
        np = inter_idx(ni);
        if np == lp, continue; end
        d = dist(lp, np);
        if tsf + d > MAX_DAYS, continue; end
        dE2 = dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS, continue; end
        np2 = [path, np]; nt = tsf + d;
        nwa = wa; nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2); nww(end+1) = np - 2;
        end
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online2(...
            np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, ...
            dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, ...
            O_init, H_init, F_init, M_init, Z_init, cons, nodes, INIT_Z);
    end
end

% ---- 叶节点评估 ----
function [bZ, bM, bP, bWD, bPS] = eval_leaf_online2(...
    fp, m, tt, park_rem, dist, wd_in, bZ, bM, bP, bWD, bPS, ...
    cons, MAX_LOAD, O_init, H_init, F_init, M_init, Z_init)
    n_seg = m + 1;
    park_combs = enumerate_park_combs2(n_seg + 1, park_rem);
    for ci = 1:size(park_combs, 1)
        ps = park_combs(ci, 1:n_seg);
        [ok, Z, M_out] = sim_online2(fp, m, tt, dist, wd_in, ps, ...
            cons, MAX_LOAD, O_init, H_init, F_init, M_init, Z_init);
        if ok && (Z > bZ || (Z == bZ && M_out > bM))
            bZ = Z; bM = M_out; bP = fp; bWD = wd_in; bPS = ps;
        end
    end
end

% ---- 在线模拟 ----
function [feasible, Zf, Mf] = sim_online2(pid, m, tt, dist_all, wd, park_seg, ...
    cons, MAX_LOAD, O, H, F, M, Z)
    T = tt + sum(wd) + sum(park_seg) + 100;
    cO = zeros(1,T); cH = zeros(1,T); cF = zeros(1,T);
    zG = zeros(1,T); isSup = false(1,T);
    WM_local = [4,5,3]; WY_local = [20,15,28];
    day = 0;

    % 重建 wa (work point indices in path)
    wa = []; ww = [];
    for i = 2:length(pid)
        if pid(i) >= 3 && pid(i) <= 5
            wa(end+1) = i; ww(end+1) = pid(i) - 2;
        end
    end

    for k = 1:(m+1)
        % 停泊
        for pd = 1:park_seg(k)
            day = day + 1; cO(day)=cons.PO; cH(day)=cons.PH; cF(day)=cons.PF;
        end
        % 移动
        d = dist_all(pid(k), pid(k+1));
        for dd = 1:d
            day = day + 1; cO(day)=cons.MO; cH(day)=cons.MH; cF(day)=cons.MF;
            if dd == d && (pid(k+1)==6 || pid(k+1)==7), isSup(day)=true; end
        end
        % 作业
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && ~isempty(wd) && wd(wk) > 0
            wm_val = WM_local(ww(wk)); yld = WY_local(ww(wk));
            rem_val = wd(wk);
            while rem_val > 0
                chunk = min(rem_val, wm_val);
                for w = 1:chunk
                    day = day + 1; cO(day)=cons.WO; cH(day)=cons.WH; cF(day)=cons.WF; zG(day)=yld;
                end
                rem_val = rem_val - chunk;
                if rem_val > 0
                    day = day + 1; cO(day)=cons.PO; cH(day)=cons.PH; cF(day)=cons.PF;
                end
            end
        end
    end
    T_actual = day;
    for t = 1:T_actual
        O = O - cO(t); H = H - cH(t); F = F - cF(t); Z = Z + zG(t);
        if O < 0 || H < 0 || F < 0, feasible = false; Zf = Z; Mf = M; return; end
        if O + H + F > MAX_LOAD + 1e-9, feasible = false; Zf = Z; Mf = M; return; end
        if isSup(t)
            ns = T_actual + 1;
            for tt2 = (t+1):T_actual, if isSup(tt2), ns = tt2; break; end; end
            nO=0; nH=0; nF=0;
            for tt2 = (t+1):ns, if tt2 > T_actual, break; end; nO=nO+cO(tt2); nH=nH+cH(tt2); nF=nF+cF(tt2); end
            sp = MAX_LOAD - (O+H+F);
            bO = max(0, nO-O); bH = max(0, nH-H); bF = max(0, nF-F);
            if bO+bH+bF > sp, feasible = false; Zf = Z; Mf = M; return; end
            if ns > T_actual && (O+bO < nO || H+bH < nH || F+bF < nF), feasible = false; Zf = Z; Mf = M; return; end
            cost = bO*cons.pO + bH*cons.pH + bF*cons.pF;
            if cost > M, feasible = false; Zf = Z; Mf = M; return; end
            O = O + bO; H = H + bH; F = F + bF; M = M - cost;
        end
    end
    feasible = true; Zf = Z; Mf = M;
end

% ---- 辅助 ----
function combs = enumerate_park_combs2(n_seg, max_total)
    combs = zeros(0, n_seg); current = zeros(1, n_seg);
    function rec(pos, rem)
        if pos == n_seg, current(pos)=rem; combs(end+1,:)=current; return; end
        for p = 0:rem, current(pos)=p; rec(pos+1, rem-p); end
    end
    rec(1, max_total);
end

function max_w = max_work_with_park2(mc, remaining)
    best = 0;
    for k = 1:(remaining+1)
        stay = k*mc + (k-1);
        if stay > remaining, break; end
        best = k*mc; slack = remaining - stay;
        if slack >= 1, best = max(best, k*mc + min(mc, slack-1)); end
    end
    max_w = max(best, min(mc, remaining));
end

function [new_pos] = move_toward(from_pos, to_pos)
    new_pos = from_pos;
    if from_pos(1) < to_pos(1), new_pos(1) = from_pos(1) + 1;
    elseif from_pos(1) > to_pos(1), new_pos(1) = from_pos(1) - 1;
    elseif from_pos(2) < to_pos(2), new_pos(2) = from_pos(2) + 1;
    elseif from_pos(2) > to_pos(2), new_pos(2) = from_pos(2) - 1;
    end
end
