%% ============================================================
%% 问题三：确定性等价路径引导MPC —— MATLAB实现（含全部修复）
%% 东南大学2026数学建模竞赛 B题
%% ============================================================

clear; clc; rng(42);

%% ========== 参数初始化 ======================================
GRID = 30;
START = [1, 15];
GOAL  = [30, 15];

S1 = [12, 16];  S2 = [21, 16];
SUPPLY_POS = [12 16; 21 16];
SUPPLY_NAMES = {'S1', 'S2'};

W1 = [6, 21];   W2 = [15, 9];   W3 = [24, 24];
WORK_POS = [6 21; 15 9; 24 24];
WORK_NAMES = {'W1', 'W2', 'W3'};
WORK_YIELD = [20, 15, 28];
WORK_MAX   = [4, 5, 3];

O0 = 100;  H0 = 150;  F0 = 100;
M0 = 750;  Z0 = 200;
LOAD_CAP = 400;
T_MAX = 90;

PRICE = [2, 1, 2];  % O, H, F

P_NORMAL = 0.8;
P_STORM  = 0.2;

% 资源消耗表: [移动; 停泊; 作业] 各列 [O, H, F]
CONSUME_NORMAL = [2 3 2;  1 1 1;  5 4 3];
CONSUME_STORM  = [8 4 3;  3 3 2;  8 6 6];

EXP_COST_MOVE = P_NORMAL*sum(CONSUME_NORMAL(1,:)) + P_STORM*sum(CONSUME_STORM(1,:));
EXP_COST_WORK = P_NORMAL*sum(CONSUME_NORMAL(3,:)) + P_STORM*sum(CONSUME_STORM(3,:));

fprintf('===== 问题三 MPC求解器 (MATLAB) =====\n');

%% ========== Fix 1: 全局路径规划 (全排列枚举+补给点插入) =====
fprintf('\n--- 全局路径规划 ---\n');
best_path = [];
best_value = -inf;
INITIAL_RES = O0 + H0 + F0;
perms = perms(1:3);  % 所有工作点排列

for p = 1:size(perms,1)
    order = perms(p,:);
    path = START;
    current_res = INITIAL_RES;
    total_benefit = 0;
    feasible = true;
    
    for i = 1:3
        wn = order(i);
        wp = WORK_POS(wn,:);
        
        travel_cost = md(path(end,:), wp) * EXP_COST_MOVE;
        work_cost = WORK_MAX(wn) * EXP_COST_WORK;
        needed = (travel_cost + work_cost) * 1.2;
        
        if current_res < needed
            % 插入最近补给点
            [best_sp, best_sp_idx] = find_nearest_supply(path(end,:), SUPPLY_POS);
            if ~isempty(best_sp)
                path = [path; best_sp];
                remain_need = (md(best_sp, wp)*EXP_COST_MOVE + work_cost + ...
                               md(wp, GOAL)*EXP_COST_MOVE) * 1.2;
                current_res = min(LOAD_CAP, max(current_res, ceil(remain_need)));
                travel_cost = md(best_sp, wp) * EXP_COST_MOVE;
            end
        end
        
        if current_res < travel_cost + work_cost
            feasible = false; break;
        end
        
        path = [path; wp];
        total_benefit = total_benefit + WORK_MAX(wn) * WORK_YIELD(wn);
        current_res = current_res - (travel_cost + work_cost);
    end
    
    if ~feasible, continue; end
    
    % 最后到终点
    final_cost = md(path(end,:), GOAL) * EXP_COST_MOVE;
    if current_res < final_cost * 1.2
        [best_sp, ~] = find_nearest_supply(path(end,:), SUPPLY_POS);
        if ~isempty(best_sp)
            path = [path; best_sp];
        end
    end
    path = [path; GOAL];
    
    % 总天数检查
    total_days = 0;
    for i = 1:size(path,1)-1
        total_days = total_days + md(path(i,:), path(i+1,:));
    end
    total_days = total_days + sum(WORK_MAX(order));
    
    if total_days > T_MAX, continue; end
    
    if total_benefit > best_value
        best_value = total_benefit;
        best_path = path;
    end
end

global_path = best_path;
fprintf('最优路径: ');
for i = 1:size(global_path,1)
    fprintf('(%d,%d) ', global_path(i,1), global_path(i,2));
end
fprintf('\n理论Z收益: %d\n', best_value);

%% ========== MPC主循环 ======================================
fprintf('\n--- MPC仿真开始 ---\n');

pos = START;
O = O0; H = H0; F = F0; M = M0; Z = Z0;
work_days = [0, 0, 0];
path_idx = 1;
day = 1;

% 日志预分配
MAX_LOG = 100;
log_day    = zeros(MAX_LOG,1);
log_pos    = zeros(MAX_LOG,2);
log_weather = cell(MAX_LOG,1);
log_action  = cell(MAX_LOG,1);
log_repl    = cell(MAX_LOG,1);
log_O  = zeros(MAX_LOG,1); log_H  = zeros(MAX_LOG,1);
log_F  = zeros(MAX_LOG,1); log_M  = zeros(MAX_LOG,1);
log_Z  = zeros(MAX_LOG,1);
log_idx = 0;

while day <= T_MAX
    % 采样天气
    if rand() < P_NORMAL
        weather = 'normal';
    else
        weather = 'storm';
    end
    
    % MPC决策
    [action, replenish, path_idx] = mpc_decide(pos, O, H, F, M, Z, day, ...
        work_days, global_path, path_idx, weather, ...
        GRID, SUPPLY_POS, WORK_POS, WORK_YIELD, WORK_MAX, WORK_NAMES, ...
        LOAD_CAP, PRICE, CONSUME_NORMAL, CONSUME_STORM, GOAL, T_MAX, ...
        EXP_COST_MOVE);
    
    % 记录日志
    log_idx = log_idx + 1;
    log_day(log_idx)    = day;
    log_pos(log_idx,:)  = pos;
    log_weather{log_idx} = weather;
    log_action{log_idx}  = action;
    log_repl{log_idx}    = replenish;
    log_O(log_idx) = O; log_H(log_idx) = H; log_F(log_idx) = F;
    log_M(log_idx) = M; log_Z(log_idx) = Z;
    
    % 执行动作
    [pos, O, H, F, M, Z, work_days, done] = env_step(pos, O, H, F, M, Z, ...
        work_days, action, replenish, weather, ...
        GRID, GOAL, SUPPLY_POS, WORK_POS, WORK_YIELD, WORK_MAX, WORK_NAMES, ...
        LOAD_CAP, T_MAX, PRICE, CONSUME_NORMAL, CONSUME_STORM);
    
    if mod(day, 15) == 0 || done
        tgt_idx = min(path_idx, size(global_path,1));
        fprintf('Day %2d | pos=(%2d,%2d) -> (%2d,%2d) | Z=%3d | O=%3d H=%3d F=%3d M=%3d\n', ...
            day, pos(1), pos(2), global_path(tgt_idx,1), global_path(tgt_idx,2), ...
            Z, O, H, F, M);
    end
    
    if done
        fprintf('终止: Day %d\n', day);
        break;
    end
    
    day = day + 1;
end

%% ========== 结果汇总 ======================================
success = isequal(pos, GOAL);
fprintf('\n===== 结果 =====\n');
fprintf('到达终点: %s\n', string(success));
fprintf('最终 Z = %d, M = %d, Day = %d\n', Z, M, day);

log_day = log_day(1:log_idx);
log_pos = log_pos(1:log_idx,:);
log_weather = log_weather(1:log_idx);
log_action = log_action(1:log_idx);
log_O = log_O(1:log_idx); log_H = log_H(1:log_idx);
log_F = log_F(1:log_idx); log_M = log_M(1:log_idx);
log_Z = log_Z(1:log_idx);

n_move = sum(startsWith(log_action, 'move'));
n_work = sum(strcmp(log_action, 'work'));
n_repl = sum(strcmp(log_action, 'replenish'));
n_stay = sum(strcmp(log_action, 'stay'));
n_storm = sum(strcmp(log_weather, 'storm'));
fprintf('移动: %d  作业: %d  补给: %d  停泊: %d  雷暴: %d\n', ...
    n_move, n_work, n_repl, n_stay, n_storm);

%% ========== 输出结果表 ====================================
T = table(log_day, log_pos(:,1), log_pos(:,2), log_weather, ...
    log_action, ...
    log_O, log_H, log_F, log_M, log_Z, ...
    'VariableNames', {'Day','x','y','Weather','Action',...
    'O','H','F','M','Z'});
writetable(T, 'result_mpc_matlab.csv');
fprintf('\n结果已保存至 result_mpc_matlab.csv\n');


%% ============================================================
%% 子函数: MPC决策 (含全部修复)
%% ============================================================
function [action, replenish, path_idx] = mpc_decide(pos, O, H, F, M, Z, day, ...
    work_days, global_path, path_idx, weather, ...
    GRID, SUPPLY_POS, WORK_POS, WORK_YIELD, WORK_MAX, WORK_NAMES, ...
    LOAD_CAP, PRICE, CONSUME_NORMAL, CONSUME_STORM, GOAL, T_MAX, ...
    EXP_COST_MOVE)

    action = 'stay';
    replenish = [0,0,0];
    
    if path_idx > size(global_path,1)
        return;
    end
    
    target = global_path(path_idx, :);
    
    % --- 到达当前目标? ---
    if isequal(pos, target)
        wn = find_work_name_idx(pos, WORK_POS, WORK_NAMES);
        if ~isempty(wn) && work_days(wn) < WORK_MAX(wn)
            action = 'work';
            return;
        else
            path_idx = path_idx + 1;
            if path_idx > size(global_path,1)
                return;
            end
            target = global_path(path_idx, :);
        end
    end
    
    % --- Fix 2: 动态重规划 (跳过不可达目标) ---
    skipped = 0;
    while ~can_reach(pos, O, H, F, target, EXP_COST_MOVE, 1.2) && ...
          path_idx < size(global_path,1)
        next_idx = path_idx + 1;
        next_target = global_path(next_idx, :);
        if can_reach(pos, O, H, F, next_target, EXP_COST_MOVE, 1.2)
            path_idx = next_idx;
            target = next_target;
            fprintf('  [REPLAN] Day %d: skip to (%d,%d)\n', day, target(1), target(2));
            break;
        else
            path_idx = next_idx;
        end
        skipped = skipped + 1;
        if skipped > 3, break; end
    end
    
    % --- Fix 3: 补给 (fill-to-75% + 容差) ---
    sn = find_supply_idx(pos, SUPPLY_POS);
    if ~isempty(sn)
        repl = forward_replenish(O, H, F, M, LOAD_CAP, PRICE);
        if ~isempty(repl)
            action = 'replenish';
            replenish = repl;
            return;
        end
    end
    
    % --- Fix 4: 向目标移动 (优先缩减大坐标误差) ---
    action = move_towards(pos, target, GRID);
    
    % --- Fix 5: 自适应风暴避险 ---
    if strcmp(weather, 'storm')
        remaining = T_MAX - day;
        dist_to_end = md(pos, GOAL);
        time_budget = remaining - dist_to_end;
        
        if time_budget > 15
            safe_O = 10;
        elseif time_budget > 5
            safe_O = 5;
        else
            safe_O = 0;
        end
        
        consume = CONSUME_STORM(1,:);  % 移动消耗
        if startsWith(action, 'move')
            if O - consume(1) < safe_O || H - consume(2) < safe_O || F - consume(3) < safe_O
                action = 'stay';
            end
        end
    end
end


%% ============================================================
%% 子函数: 环境步进
%% ============================================================
function [pos, O, H, F, M, Z, work_days, done] = env_step(pos, O, H, F, M, Z, ...
    work_days, action, replenish, weather, ...
    GRID, GOAL, SUPPLY_POS, WORK_POS, WORK_YIELD, WORK_MAX, WORK_NAMES, ...
    LOAD_CAP, T_MAX, PRICE, CONSUME_NORMAL, CONSUME_STORM)

    done = false;
    
    % 补给
    if strcmp(action, 'replenish') && any(replenish > 0)
        bo = replenish(1); bh = replenish(2); bf = replenish(3);
        cost = bo*PRICE(1) + bh*PRICE(2) + bf*PRICE(3);
        if cost <= M && (O+H+F+bo+bh+bf) <= LOAD_CAP
            O = O + bo; H = H + bh; F = F + bf;
            M = M - cost;
        end
    end
    
    % 资源消耗
    if strcmp(weather, 'normal')
        ctbl = CONSUME_NORMAL;
    else
        ctbl = CONSUME_STORM;
    end
    
    if startsWith(action, 'move')
        consume = ctbl(1,:);
        dirs = {'move_up', 'move_down', 'move_left', 'move_right'};
        deltas = [-1 0; 1 0; 0 -1; 0 1];
        for i = 1:4
            if strcmp(action, dirs{i})
                pos = pos + deltas(i,:);
                break;
            end
        end
        % 离开作业点，重置计数
        for j = 1:3
            if ~isequal(pos, WORK_POS(j,:))
                work_days(j) = 0;
            end
        end
    elseif strcmp(action, 'work')
        consume = ctbl(3,:);
        wn = find_work_name_idx(pos, WORK_POS, WORK_NAMES);
        if ~isempty(wn)
            work_days(wn) = work_days(wn) + 1;
            Z = Z + WORK_YIELD(wn);
        end
    else  % stay (含replenish后的消耗)
        consume = ctbl(2,:);
    end
    
    O = O - consume(1);
    H = H - consume(2);
    F = F - consume(3);
    
    % 合法性检查
    if O < 0 || H < 0 || F < 0 || M < 0 || (O+H+F) > LOAD_CAP
        done = true;
        return;
    end
    
    if isequal(pos, GOAL)
        done = true;
        return;
    end
end


%% ============================================================
%% 辅助函数
%% ============================================================
function d = md(p1, p2)
    d = abs(p1(1)-p2(1)) + abs(p1(2)-p2(2));
end

function r = can_reach(pos, O, H, F, target, exp_cost, safety)
    dist = md(pos, target);
    needed = dist * exp_cost * safety;
    r = (O + H + F) >= needed;
end

function wn = find_work_name_idx(pos, WORK_POS, WORK_NAMES)
    wn = [];
    for i = 1:size(WORK_POS,1)
        if isequal(pos, WORK_POS(i,:))
            wn = i;
            return;
        end
    end
end

function sn = find_supply_idx(pos, SUPPLY_POS)
    sn = [];
    for i = 1:size(SUPPLY_POS,1)
        if isequal(pos, SUPPLY_POS(i,:))
            sn = i;
            return;
        end
    end
end

function [sp, idx] = find_nearest_supply(pos, SUPPLY_POS)
    best_d = 999;
    sp = []; idx = [];
    for i = 1:size(SUPPLY_POS,1)
        d = md(pos, SUPPLY_POS(i,:));
        if d < best_d
            best_d = d; sp = SUPPLY_POS(i,:); idx = i;
        end
    end
end

function repl = forward_replenish(O, H, F, M, LOAD_CAP, PRICE)
    % Fix 3: fill to 75% capacity with tolerance
    current = O + H + F;
    target_fill = floor(LOAD_CAP * 0.85);
    if current >= target_fill - 10
        repl = [];
        return;
    end
    space = LOAD_CAP - current;
    buy = target_fill - current;
    buy = min(buy, space);
    % 4:4:2 比例 (O:H:F), O优先
    o = min(floor(buy * 5 / 10), space, floor(M / PRICE(1)));
    h = min(floor(buy * 5 / 10), space - o, floor(M / PRICE(2)));
    f = min(floor(buy * 2 / 10), space - o - h, floor(M / PRICE(3)));
    repl = [max(0,o), max(0,h), max(0,f)];
end

function action = move_towards(pos, target, GRID)
    % Fix 4: 优先缩减大坐标误差
    x = pos(1); y = pos(2);
    tx = target(1); ty = target(2);
    dx_err = tx - x; dy_err = ty - y;
    
    dirs = {'move_up', 'move_down', 'move_left', 'move_right'};
    deltas = [-1 0; 1 0; 0 -1; 0 1];
    
    % 优先列表: 先尝试缩减大误差方向
    preferred = {};
    if abs(dx_err) >= abs(dy_err)
        if dx_err > 0
            preferred{end+1} = 'move_down';
        elseif dx_err < 0
            preferred{end+1} = 'move_up';
        end
        if dy_err > 0
            preferred{end+1} = 'move_right';
        elseif dy_err < 0
            preferred{end+1} = 'move_left';
        end
    else
        if dy_err > 0
            preferred{end+1} = 'move_right';
        elseif dy_err < 0
            preferred{end+1} = 'move_left';
        end
        if dx_err > 0
            preferred{end+1} = 'move_down';
        elseif dx_err < 0
            preferred{end+1} = 'move_up';
        end
    end
    
    all_dirs = [preferred, setdiff(dirs, preferred, 'stable')];
    
    for i = 1:length(all_dirs)
        an = all_dirs{i};
        idx = find(strcmp(dirs, an));
        nx = x + deltas(idx,1);
        ny = y + deltas(idx,2);
        if nx >= 1 && nx <= GRID && ny >= 1 && ny <= GRID
            action = an;
            return;
        end
    end
    action = 'stay';
end

function p = perms(v)
    % 生成所有排列 (替代MATLAB的perms, 避免内置函数大输入问题)
    n = length(v);
    if n <= 1
        p = v;
        return;
    end
    p = [];
    for i = 1:n
        rest = v([1:i-1, i+1:end]);
        sub = perms(rest);
        p = [p; repmat(v(i), size(sub,1), 1), sub];
    end
end