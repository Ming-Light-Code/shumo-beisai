% ============================================================
% 解法二：DP (DFS路径枚举 + 上界剪枝 + 完整贪婪模拟评估)
% 
% 设计思路:
%   - DFS 枚举路径骨架和工作天数组合
%   - 用 days_used 和 Z_so_far 做上界剪枝, 控制搜索空间
%   - 仅在到达终点 E 时运行完整贪婪前向模拟评估可行性
%   - 贪婪模拟已修复补给日消耗的边界 bug
%
% 相比原版的修复:
%   1. 多段路径累积: 路径递归构建, 不再仅做单段转移
%   2. 允许重复访问: 去除 bitmask, 用 day≤30 作为自然边界
%   3. 补给日消耗修复: 补给日先扣当日消耗, 再采购未来需求
%   4. 上界剪枝: 避免搜索组合爆炸
% ============================================================

function [best_Z, best_M, best_path, best_wdays] = method_dp_complete()
    all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
    CM = [2, 3, 2];   CW = [5, 4, 3];
    WY = [20, 15, 28];  WM = [4, 5, 3];
    MAX_WY = max(WY);  % 28, 用于上界剪枝
    
    n_pts = 7;
    dist = zeros(n_pts);
    for i = 1:n_pts
        for j = 1:n_pts
            dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
        end
    end
    
    % ---- 全局最优 (persistent across recursive calls) ----
    best_Z = -inf;  best_M = -inf;
    best_path = [];  best_wdays = [];
    
    % ---- 节点计数器 (用于调试) ----
    node_count = 0;
    
    % ---- 启动 DFS: 当前在 B(1), 天数为0, Z=100 ----
    dfs(1, 0, 100, [1], []);

    fprintf('探索节点数: %d\n', node_count);
    if best_Z > -inf
        fprintf('=== 解法二 (DP/DFS) 最优结果 ===\n');
        fprintf('Z = %d, M = %d\n', best_Z, best_M);
        names = {'B','E','W1','W2','W3','S1','S2'};
        fprintf('路径: ');
        for i = 1:length(best_path)
            fprintf('%s', names{best_path(i)});
            if i < length(best_path), fprintf(' -> '); end
        end
        fprintf('\n');
        if ~isempty(best_wdays)
            fprintf('工作天数: '); fprintf('%d ', best_wdays); fprintf('\n');
        end
    else
        fprintf('未找到可行解。\n');
    end

    % ==================== 嵌套函数 ====================

    function update_best(Z, M, path, wdays)
        if Z > best_Z || (Z == best_Z && M > best_M)
            best_Z = Z; best_M = M;
            best_path = path; best_wdays = wdays;
        end
    end

    function dfs(current, day_used, Z_so_far, path, wdays)
        % current:   当前点索引 (1..7)
        % day_used:  已用天数
        % Z_so_far:  当前累积的目标物资
        % path:      走过的点序列
        % wdays:     每个作业点的作业天数
        
        node_count = node_count + 1;
        
        % ---- 上界剪枝 ----
        dE = dist(current, 2);
        remain_for_work = 30 - day_used - dE;
        if remain_for_work < 0, return; end
        if Z_so_far + remain_for_work * MAX_WY <= best_Z
            return;
        end
        
        % ---- 尝试直接去终点 E ----
        full_path = [path, 2];
        [feas, Z_final, M_final] = greedy_sim_full(...
            full_path, wdays, dist, CM, CW, WY);
        if feas
            update_best(Z_final, M_final, full_path, wdays);
        end
        
        % ---- 尝试去各个中间点 ----
        for nxt = [3 4 5 6 7]  % W1,W2,W3,S1,S2
            d_nxt = dist(current, nxt);
            if nxt == current, continue; end
            if day_used + d_nxt > 30, continue; end
            
            new_day = day_used + d_nxt;
            is_work = (nxt >= 3 && nxt <= 5);
            
            if is_work
                wh = nxt - 2;
                max_work = min(WM(wh), 30 - new_day);
                for w = 0:max_work
                    new_Z = Z_so_far + w * WY(wh);
                    dfs(nxt, new_day + w, new_Z, [path, nxt], [wdays, w]);
                end
            else
                dfs(nxt, new_day, Z_so_far, [path, nxt], wdays);
            end
        end
    end

    function [feasible, Z_final, M_final] = greedy_sim_full(...
            pid, wdays, dist, CM, CW, WY)
        % 对完整路径运行贪婪前向模拟 (修复版)
        % 修复: 补给日先扣当日移动消耗, 再采购从 t+1 到下一补给日前一天的需求
        
        m = length(pid) - 2;
        total_travel = 0;
        for k = 1:(m+1)
            total_travel = total_travel + dist(pid(k), pid(k+1));
        end
        T = total_travel + sum(wdays);
        
        cO = zeros(1,T); cH = zeros(1,T); cF = zeros(1,T);
        zG = zeros(1,T); isSup = false(1,T);
        
        day = 0;
        wd_idx = 1;
        for k = 1:(m+1)
            d = dist(pid(k), pid(k+1));
            for dd = 1:d
                day = day + 1;
                cO(day) = CM(1); cH(day) = CM(2); cF(day) = CM(3);
                if dd == d
                    to_pt = pid(k+1);
                    if to_pt == 6 || to_pt == 7
                        isSup(day) = true;
                    end
                end
            end
            to_pt = pid(k+1);
            if to_pt >= 3 && to_pt <= 5 && wd_idx <= length(wdays) ...
                    && wdays(wd_idx) > 0
                wh = to_pt - 2;
                for w = 1:wdays(wd_idx)
                    day = day + 1;
                    cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
                    zG(day) = WY(wh);
                end
            end
            if to_pt >= 3 && to_pt <= 5
                wd_idx = wd_idx + 1;
            end
        end
        
        O = 35; H = 45; F = 30; M = 240;
        
        for t = 1:T
            if isSup(t)
                % 先扣当日消耗
                O = O - cO(t); H = H - cH(t); F = F - cF(t);
                if O < 0 || H < 0 || F < 0
                    feasible = false; Z_final = 0; M_final = 0; return;
                end
                if O + H + F > 120
                    feasible = false; Z_final = 0; M_final = 0; return;
                end
                
                % 找下一补给日
                nextSup = T + 1;
                for tt = t+1:T
                    if isSup(tt), nextSup = tt; break; end
                end
                
                % need = 从 t+1 到 nextSup-1 (不含下一补给日, 届时再扣)
                needO = 0; needH = 0; needF = 0;
                for tt = t+1:nextSup-1
                    needO = needO + cO(tt);
                    needH = needH + cH(tt);
                    needF = needF + cF(tt);
                end
                
                space = 120 - (O + H + F);
                buyO = max(0, needO - O);
                buyH = max(0, needH - H);
                buyF = max(0, needF - F);
                
                if buyO + buyH + buyF > space
                    feasible = false; Z_final = 0; M_final = 0; return;
                end
                
                % 最后一个补给站后必须满足到终点的需求
                if nextSup > T
                    if O + buyO < needO || H + buyH < needH || F + buyF < needF
                        feasible = false; Z_final = 0; M_final = 0; return;
                    end
                end
                
                cost = buyO * 2 + buyH * 1 + buyF * 2;
                if cost > M
                    feasible = false; Z_final = 0; M_final = 0; return;
                end
                
                O = O + buyO; H = H + buyH; F = F + buyF;
                M = M - cost;
            else
                O = O - cO(t); H = H - cH(t); F = F - cF(t);
                if O < 0 || H < 0 || F < 0 || M < 0
                    feasible = false; Z_final = 0; M_final = 0; return;
                end
                if O + H + F > 120
                    feasible = false; Z_final = 0; M_final = 0; return;
                end
            end
        end
        
        Z_final = 100 + sum(zG);
        M_final = M;
        feasible = true;
    end
end
