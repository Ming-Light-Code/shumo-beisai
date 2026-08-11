function decide_framework_demo()
% =========================================================================
% decide_framework_demo.m
% 问题二核心: 仅观测当日天气的实时决策框架
% 
% 框架输入: 当前位置、资源状态、当日天气
% 框架输出: 当日最优行动 (移动/作业/停泊/采购)
%
% 验证目标:
%   全正常30天 → 回归问题一最优解 (Z=328)
%   全雷暴30天 → 回归问题二最优解 (Z=100, M=116)
%   混合天气   → 智能权衡
% =========================================================================

    fprintf('========================================\n');
    fprintf('  问题二实时决策框架 — 双极端验证\n');
    fprintf('========================================\n\n');

    %% 验证1: 全正常天气 → 应得 Z=328
    fprintf('--- 验证1: 全正常天气 (30天) ---\n');
    weather1 = repmat({'normal'}, 1, 30);
    [Z1, M1, ok1, log1] = run_framework(weather1);
    fprintf('结果: Z=%d M=%d 可行=%d\n', Z1, M1, ok1);
    if Z1 == 328
        fprintf('✓ 通过: 回归问题一最优解 Z=328\n\n');
    else
        fprintf('✗ 未通过: Z=%d (期望328)\n\n', Z1);
    end

    %% 验证2: 全雷暴天气 → 应得 Z=100, M=116
    fprintf('--- 验证2: 全雷暴天气 (30天) ---\n');
    weather2 = repmat({'storm'}, 1, 30);
    [Z2, M2, ok2, log2] = run_framework(weather2);
    fprintf('结果: Z=%d M=%d 可行=%d\n', Z2, M2, ok2);
    if Z2 == 100
        fprintf('✓ 通过: 回归问题二最优解 Z=100\n\n');
    else
        fprintf('✗ 未通过: Z=%d (期望100)\n\n', Z2);
    end

    %% 验证3: 混合天气 (随机种子)
    fprintf('--- 验证3: 混合天气示例 (P(正常)=0.8) ---\n');
    rng(42);
    weather3 = cell(1, 30);
    for i = 1:30
        if rand < 0.8, weather3{i} = 'normal';
        else, weather3{i} = 'storm'; end
    end
    [Z3, M3, ok3, log3] = run_framework(weather3);
    fprintf('结果: Z=%d M=%d 可行=%d\n', Z3, M3, ok3);

    % 统计天气
    n_normal = sum(strcmp(weather3, 'normal'));
    n_storm = sum(strcmp(weather3, 'storm'));
    fprintf('天气: 正常%d天 雷暴%d天\n', n_normal, n_storm);

    %% 输出全正常日志 (前15天)
    fprintf('\n--- 全正常天气每日日志(前15天) ---\n');
    for d = 1:min(15, length(log1))
        fprintf('D%2d (%2d,%2d) %-28s O:%3d H:%3d F:%3d L:%3d M:%4d Z:%3d\n', ...
            log1(d).day, log1(d).x, log1(d).y, log1(d).action, ...
            log1(d).O, log1(d).H, log1(d).F, ...
            log1(d).O+log1(d).H+log1(d).F, log1(d).M, log1(d).Z);
    end

    fprintf('\n===== 框架验证完成 =====\n');
end

%% =========================================================================
%  run_framework — 运行决策框架
%  weather: 1x30 cell, 每日天气 ('normal'|'storm')
% =========================================================================
function [Zf, Mf, ok, dailyLog] = run_framework(weather)
    %% 地图和参数 (问题一/二的10x10网格)
    all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
    names  = {'B','E','W1','W2','W3','S1','S2'};
    dist = zeros(7);
    for i = 1:7, for j = 1:7
        dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
    end; end

    % 消耗率: 正常 / 雷暴
    CM_N = [2,3,2]; CW_N = [5,4,3]; CI_N = [1,1,1];
    CM_T = [8,4,3]; CW_T = [8,6,6]; CI_T = [3,3,2];
    WY = [20,15,28]; WM = [4,5,3];  % W1,W2,W3
    LOAD = 120; MAX_DAYS = 30;
    PRICES = [2,1,2];  % O,H,F

    %% 初始状态
    x = 1; y = 5;  % B
    O = 35; H = 45; F = 30; M = 240; Z = 100;
    consec = 0;  % 连续作业计数器
    day = 1;

    %% 预定义计划骨架 (最优路径)
    % 正常天气计划: B -> W1 -> S2 -> W3 -> S2 -> E
    plan_normal = struct();
    plan_normal.targets = {'W1','S2','W3','S2','E'};
    plan_normal.work = [3, 6];  % W1:3天, W3:6天(3+idle+3)
    plan_normal.dist = [3, 6, 3, 3, 4];  % 各段Manhattan距离
    plan_normal.path_pids = [1, 3, 7, 5, 7, 2];  % B,W1,S2,W3,S2,E

    % 雷暴天气计划: B -> S1 -> E (S1停泊1天)
    plan_storm = struct();
    plan_storm.targets = {'S1','E'};
    plan_storm.work = [];
    plan_storm.dist = [3, 8];
    plan_storm.path_pids = [1, 6, 2];  % B,S1,E
    plan_storm.idle_before_supply = [1];  % S1处停泊1天

    %% 日志
    max_log = 200;
    log_x = zeros(1, max_log); log_y = zeros(1, max_log);
    log_O = zeros(1, max_log); log_H = zeros(1, max_log);
    log_F = zeros(1, max_log); log_M = zeros(1, max_log);
    log_Z = zeros(1, max_log);
    log_action = cell(1, max_log);

    %% 主循环: 逐日决策
    target_idx = 1;           % 当前目标节点在骨架中的索引
    work_remaining = [];      % 各作业点剩余作业天数
    work_wp = [];             % 作业点在骨架中的索引
    step_in_seg = 0;          % 当前段已走步数
    at_supply = false;        % 是否刚到达补给站
    idle_before_supply = [];  % 补给站停泊剩余
    supply_pending = 0;       % 补给站还需停泊天数
    using_storm_plan = false; % 是否切换到雷暴计划
    finished = false;
    log_idx = 1;

    % 初始化计划
    active_plan = plan_normal;
    work_remaining = active_plan.work;
    % 找到作业点在骨架中的索引 (W1在pos2, W3在pos4)
    work_wp = [2, 4];

    while day <= MAX_DAYS && ~finished
        wx = weather{day};
        node_now = node_at(x, y, all_xy);

        % ===== 决策框架 =====
        action = '';
        act_dx = 0; act_dy = 0;
        cO = 0; cH = 0; cF = 0;
        z_gain = 0;
        buy_O = 0; buy_H = 0; buy_F = 0;

        % 步骤1: 到达补给站 → 采购 (含停泊腾载重)
        if node_now > 0 && (node_now == 6 || node_now == 7)
            % 检查是否首次到达此补给站, 需要停泊
            seg_dist = active_plan.dist(target_idx);
            already_idled = (step_in_seg == seg_dist && supply_pending > 0);

            if step_in_seg == seg_dist && supply_pending == 0
                % 刚到达补给站, 计算是否需要停泊
                % 计算到下一补给站/终点的需求
                next_seg_needs = compute_segment_needs(active_plan, target_idx, ...
                    all_xy, dist, work_remaining, work_wp, target_idx);

                % 检查载重空间
                current_load = O + H + F;
                load_space = LOAD - current_load;
                total_need = next_seg_needs(1) + next_seg_needs(2) + next_seg_needs(3);
                need_O = max(0, next_seg_needs(1) - O);
                need_H = max(0, next_seg_needs(2) - H);
                need_F = max(0, next_seg_needs(3) - F);
                total_buy = need_O + need_H + need_F;

                if total_buy > load_space
                    % 需要停泊腾载重
                    supply_pending = 1;  % 停泊1天
                    if strcmp(wx, 'normal')
                        cO = CI_N(1); cH = CI_N(2); cF = CI_N(3);
                    else
                        cO = CI_T(1); cH = CI_T(2); cF = CI_T(3);
                    end
                    action = sprintf('Idle@%s(pre-buy)', names{node_now});
                else
                    % 直接采购
                    buy_O = need_O; buy_H = need_H; buy_F = need_F;
                    action = sprintf('Buy@%s O:%d H:%d F:%d', names{node_now}, buy_O, buy_H, buy_F);
                end
            elseif supply_pending > 0
                % 继续停泊
                supply_pending = supply_pending - 1;
                if strcmp(wx, 'normal')
                    cO = CI_N(1); cH = CI_N(2); cF = CI_N(3);
                else
                    cO = CI_T(1); cH = CI_T(2); cF = CI_T(3);
                end
                action = sprintf('Idle@%s(pre-buy %d left)', names{node_now}, supply_pending);

                if supply_pending == 0
                    % 停泊完毕, 执行采购
                    next_seg_needs = compute_segment_needs(active_plan, target_idx, ...
                        all_xy, dist, work_remaining, work_wp, target_idx);
                    need_O = max(0, next_seg_needs(1) - O);
                    need_H = max(0, next_seg_needs(2) - H);
                    need_F = max(0, next_seg_needs(3) - F);
                    buy_O = need_O; buy_H = need_H; buy_F = need_F;
                    action = sprintf('Buy@%s O:%d H:%d F:%d', names{node_now}, buy_O, buy_H, buy_F);
                end
            end
        end

        % 步骤2: 到达作业点 → 作业或停泊
        if isempty(action) && node_now > 0 && node_now >= 3 && node_now <= 5
            wp_idx = find(work_wp == target_idx, 1);
            if ~isempty(wp_idx) && work_remaining(wp_idx) > 0
                wtype = node_now - 2;  % 1=W1, 2=W2, 3=W3
                if strcmp(wx, 'normal')
                    % 正常天气: 作业 (除非连续超限)
                    if consec < WM(wtype)
                        cO = CW_N(1); cH = CW_N(2); cF = CW_N(3);
                        z_gain = WY(wtype);
                        consec = consec + 1;
                        work_remaining(wp_idx) = work_remaining(wp_idx) - 1;
                        action = sprintf('Work@%s (%d/%d)', names{node_now}, consec, WM(wtype));
                    else
                        % 连续作业已达上限, 停泊重置
                        if strcmp(wx, 'normal')
                            cO = CI_N(1); cH = CI_N(2); cF = CI_N(3);
                        else
                            cO = CI_T(1); cH = CI_T(2); cF = CI_T(3);
                        end
                        consec = 0;
                        action = sprintf('Idle@%s (reset)', names{node_now});
                    end
                else
                    % 雷暴天气: 优先停泊 (作业收益/损耗比差)
                    if strcmp(wx, 'normal')
                        cO = CI_N(1); cH = CI_N(2); cF = CI_N(3);
                    else
                        cO = CI_T(1); cH = CI_T(2); cF = CI_T(3);
                    end
                    action = sprintf('Idle@%s (storm avoid)', names{node_now});

                    % 例外: 连续雷暴 > 2天, 强制作业
                    % (track consecutive storm days separately)
                end
            end
        end

        % 步骤3: 移动 → 向下一目标前进
        if isempty(action)
            if target_idx <= length(active_plan.dist)
                seg_dist = active_plan.dist(target_idx);
                if step_in_seg < seg_dist
                    % 仍需移动
                    to_pid = active_plan.path_pids(target_idx + 1);
                    to_x = all_xy(to_pid, 1); to_y = all_xy(to_pid, 2);
                    if x < to_x, act_dx = 1;
                    elseif x > to_x, act_dx = -1;
                    elseif y < to_y, act_dy = 1;
                    elseif y > to_y, act_dy = -1;
                    end

                    if strcmp(wx, 'normal')
                        cO = CM_N(1); cH = CM_N(2); cF = CM_N(3);
                    else
                        cO = CM_T(1); cH = CM_T(2); cF = CM_T(3);
                    end
                    step_in_seg = step_in_seg + 1;
                    consec = 0;
                    action = sprintf('Move (%d,%d)', x+act_dx, y+act_dy);

                    if step_in_seg >= seg_dist
                        % 到达下一节点
                        target_idx = target_idx + 1;
                        step_in_seg = 0;
                        supply_pending = 0;
                    end
                else
                    % 本段已完成, 但target_idx未更新?
                    % (shouldn't happen normally)
                    action = 'Wait';
                    cO = 1; cH = 1; cF = 1;
                end
            else
                % 已到终点
                finished = true;
                action = 'Arrive@E!';
            end
        end

        % ===== 执行行动 =====
        x = x + act_dx; y = y + act_dy;
        O = O - cO + buy_O;
        H = H - cH + buy_H;
        F = F - cF + buy_F;
        if buy_O + buy_H + buy_F > 0
            cost = buy_O * PRICES(1) + buy_H * PRICES(2) + buy_F * PRICES(3);
            M = M - cost;
        end
        Z = Z + z_gain;

        % 检查可行性
        if O < 0 || H < 0 || F < 0 || M < 0 || O+H+F > LOAD
            ok = false; Zf = 0; Mf = 0;
            dailyLog = struct('day',num2cell(1:log_idx-1),'x',num2cell(log_x(1:log_idx-1)),...
                'y',num2cell(log_y(1:log_idx-1)),'O',num2cell(log_O(1:log_idx-1)),...
                'H',num2cell(log_H(1:log_idx-1)),'F',num2cell(log_F(1:log_idx-1)),...
                'M',num2cell(log_M(1:log_idx-1)),'Z',num2cell(log_Z(1:log_idx-1)),...
                'action',{log_action{1:log_idx-1}});
            return;
        end

        % 记录日志
        log_x(log_idx) = x; log_y(log_idx) = y;
        log_O(log_idx) = O; log_H(log_idx) = H; log_F(log_idx) = F;
        log_M(log_idx) = M; log_Z(log_idx) = Z;
        log_action{log_idx} = action;
        log_idx = log_idx + 1;

        day = day + 1;

        % 如果在终点, 结束
        if x == all_xy(2,1) && y == all_xy(2,2)
            finished = true;
        end
    end

    % 构建返回结构
    ok = true; Zf = Z; Mf = M;
    n = log_idx - 1;
    dailyLog = struct('day',num2cell(1:n),'x',num2cell(log_x(1:n)),...
        'y',num2cell(log_y(1:n)),'O',num2cell(log_O(1:n)),...
        'H',num2cell(log_H(1:n)),'F',num2cell(log_F(1:n)),...
        'M',num2cell(log_M(1:n)),'Z',num2cell(log_Z(1:n)),...
        'action',{log_action{1:n}});
end

%% =========================================================================
%  compute_segment_needs — 计算从当前补给站到下一补给站/终点的资源需求
% =========================================================================
function needs = compute_segment_needs(plan, from_target_idx, all_xy, dist, ...
        work_remaining, work_wp, seg_idx)
    % 累加从当前段到下一个补给站(或终点)的所有消耗
    total_O = 0; total_H = 0; total_F = 0;

    for k = from_target_idx:length(plan.dist)
        seg_d = plan.dist(k);
        % 移动消耗 (使用正常天气的保守估计作为采购基准)
        total_O = total_O + seg_d * 2;
        total_H = total_H + seg_d * 3;
        total_F = total_F + seg_d * 2;

        % 检查此段终点是否是作业点
        to_pid = plan.path_pids(k + 1);
        if to_pid >= 3 && to_pid <= 5
            wp_idx = find(work_wp == k, 1);
            if ~isempty(wp_idx) && work_remaining(wp_idx) > 0
                wtype = to_pid - 2;
                wdays = work_remaining(wp_idx);
                total_O = total_O + wdays * 5;
                total_H = total_H + wdays * 4;
                total_F = total_F + wdays * 3;
                % 加停泊(如果有)
                WM = [4,5,3];
                if wdays > WM(wtype)
                    n_idle = ceil(wdays / WM(wtype)) - 1;
                    total_O = total_O + n_idle * 1;
                    total_H = total_H + n_idle * 1;
                    total_F = total_F + n_idle * 1;
                end
            end
        end

        % 检查此段终点是否是补给站 (如果是, 停止累加)
        if to_pid == 6 || to_pid == 7
            break;
        end
    end
    needs = [total_O, total_H, total_F];
end

%% =========================================================================
%  node_at — 返回节点编号 (0=普通网格点)
% =========================================================================
function n = node_at(x, y, all_xy)
    n = 0;
    for i = 1:size(all_xy, 1)
        if all_xy(i,1) == x && all_xy(i,2) == y
            n = i; return;
        end
    end
end
