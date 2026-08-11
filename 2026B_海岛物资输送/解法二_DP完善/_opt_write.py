import sys
sys.stdout.reconfigure(encoding='utf-8')

code = r'''% ============================================================
% 解法二 问题2：在线天气决策模型 — 全雷暴极端场景特例求解
%
% 模型框架:
%   问题2的核心是一个在线决策问题——船舶每日观测天气 w_d，
%   根据当前状态 s_d 实时选择动作 a_d = pi(s_d, w_d)。
%   策略 pi 必须对任意天气序列 (w_1,...,w_30) 都有定义。
%
% 全雷暴特例:
%   当 w_d 恒为雷暴时，天气序列退化为确定型。
%   在线策略 pi 的最优行为等价于该确定型场景的离线最优解。
%   本代码求解此特例：通过 DFS 枚举所有 (路径骨架, 工作天数)，
%   经由贪婪前向模拟评估，找出全局最优的确定型方案。
%
% 消耗率（全雷暴）:
%   移动:  O=8, H=4, F=3  (CM)
%   作业:  O=8, H=6, F=6  (CW)
%   停泊:  O=3, H=3, F=2  (CI)
%
% 停泊-再作业策略:
%   对工作点 wh, 若总工作天数 w > WM(wh),
%   插入 I(w) = ceil(w/WM)-1 个停泊日打断连续作业链。
%   例如 W3(WM=3): 7天工作 = 3+1(停)+3+1(停)+1 = 9天总停留
%
% 剪枝策略:
%   节点级: Z_so_far + remain * 28 <= best_Z -> 剪枝 (乐观上界)
%   工作日级: new_Z + opt_remain * 28 <= best_Z -> 跳过该w
%
% 优化措施 (v2):
%   1. 中间点优先序: W3(28) > W2(15) > W1(20) > S1,S2 — 高收益优先
%   2. 跳过连续补给站 (S1->S2 / S2->S1) — 无作业纯消耗
%   3. W_max 计入返E距离: w+I(w)+dist(nxt,E) <= 剩余天数
%   4. 预判E可达性: 递归前检查 dist(nxt,E) 是否超预算
%   5. 剪枝计数器: 输出中被剪枝/探索节点数
% ============================================================

function [best_Z, best_M, best_path, best_wdays] = method_dp_complete_task2()
    all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
    CM = [8, 4, 3];      % 雷暴移动消耗: O=8, H=4, F=3
    CW = [8, 6, 6];      % 雷暴作业消耗: O=8, H=6, F=6
    CI = [3, 3, 2];      % 雷暴停泊消耗: O=3, H=3, F=2
    WY = [20, 15, 28];   % 工作点收益
    WM = [4, 5, 3];      % 单次最大连续作业天数
    MAX_WY = max(WY);    % 28，用于上界剪枝

    n_pts = 7;
    dist = zeros(n_pts);
    for i = 1:n_pts
        for j = 1:n_pts
            dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
        end
    end

    % 全局最优 (跨递归调用持久化)
    best_Z = -inf;  best_M = -inf;
    best_path = [];  best_wdays = [];
    node_count = 0;  pruned_count = 0;

    % 启动 DFS: 从 B(1) 出发，已用天数 0，Z=100
    dfs(1, 0, 100, [1], []);

    fprintf('探索节点数: %d  (被剪枝: %d)\n', node_count, pruned_count);
    if best_Z > -inf
        fprintf('=== 解法二 (DP/DFS) 问题2 最优结果 ===\n');
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
        % day_used:  已用天数（含移动、作业、停泊）
        % Z_so_far:  当前累积目标物资
        % path:      走过的点序列
        % wdays:     各工作点的作业天数（不含停泊日）

        node_count = node_count + 1;

        % ---- 节点级上界剪枝 ----
        dE = dist(current, 2);
        remain_for_work = 30 - day_used - dE;
        if remain_for_work < 0, return; end
        if Z_so_far + remain_for_work * MAX_WY <= best_Z
            pruned_count = pruned_count + 1;
            return;
        end

        % ---- 尝试直接去终点 E ----
        full_path = [path, 2];
        [feas, Z_final, M_final] = greedy_sim_full(full_path, wdays, dist, CM, CW, CI, WM, WY);
        if feas
            update_best(Z_final, M_final, full_path, wdays);
        end

        % ---- 尝试去各个中间点 ----
        % 优化1: W3(收益28)优先 -> W2(15) -> W1(20) -> S1,S2
        for nxt = [5 4 3 6 7]
            d_nxt = dist(current, nxt);
            if nxt == current, continue; end
            if day_used + d_nxt > 30, continue; end
            % 优化2: 跳过连续补给站访问 (无作业纯消耗)
            if current >= 6 && nxt >= 6, continue; end
            % 优化4: 预判从 nxt 能否在期限内抵达 E
            d_to_E_nxt = dist(nxt, 2);
            if day_used + d_nxt + d_to_E_nxt > 30, continue; end

            new_day = day_used + d_nxt;
            is_work = (nxt >= 3 && nxt <= 5);

            if is_work
                wh = nxt - 2;
                M_lim = WM(wh);

                % 优化3: W_max 计入返E距离，收紧上界
                max_work_at = 0;
                for test_w = 0:100
                    idle_days = max(0, ceil(test_w / M_lim) - 1);
                    if new_day + test_w + idle_days + d_to_E_nxt > 30
                        break;
                    end
                    max_work_at = test_w;
                end

                for w = 0:max_work_at
                    idle_days = max(0, ceil(w / M_lim) - 1);
                    new_Z = Z_so_far + w * WY(wh);
                    % 工作日级 Z 上界剪枝
                    opt_rem = 30 - new_day - w - idle_days - d_to_E_nxt;
                    if new_Z + opt_rem * MAX_WY <= best_Z
                        pruned_count = pruned_count + 1;
                        continue;
                    end
                    dfs(nxt, new_day + w + idle_days, new_Z, ...
                        [path, nxt], [wdays, w]);
                end
            else
                dfs(nxt, new_day, Z_so_far, [path, nxt], wdays);
            end
        end
    end

    function [feasible, Z_final, M_final] = greedy_sim_full(...
            pid, wdays, dist, CM, CW, CI, WM, WY)
        % 对完整路径运行贪婪前向模拟（含停泊日构建）
        % 修复: 补给日先扣当日移动消耗, 再采购从 t+1 到下一补给日前一天的需求

        m = length(pid) - 2;

        total_travel = 0;
        for k = 1:(m+1)
            total_travel = total_travel + dist(pid(k), pid(k+1));
        end

        % 计算总天数 T（含停泊日）
        T = total_travel;
        wd_idx = 1;
        for k = 1:(m+1)
            to_pt = pid(k+1);
            if to_pt >= 3 && to_pt <= 5 && wd_idx <= length(wdays) && wdays(wd_idx) > 0
                wh = to_pt - 2;
                M_lim = WM(wh);
                w = wdays(wd_idx);
                idle_days = max(0, ceil(w / M_lim) - 1);
                T = T + w + idle_days;
            end
            if to_pt >= 3 && to_pt <= 5
                wd_idx = wd_idx + 1;
            end
        end

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
            if to_pt >= 3 && to_pt <= 5 && wd_idx <= length(wdays) && wdays(wd_idx) > 0
                wh = to_pt - 2;
                M_lim = WM(wh);
                remaining = wdays(wd_idx);

                while remaining > 0
                    seg = min(remaining, M_lim);
                    for w = 1:seg
                        day = day + 1;
                        cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
                        zG(day) = WY(wh);
                    end
                    remaining = remaining - seg;
                    if remaining > 0
                        day = day + 1;
                        cO(day) = CI(1); cH(day) = CI(2); cF(day) = CI(3);
                    end
                end
            end
            if to_pt >= 3 && to_pt <= 5
                wd_idx = wd_idx + 1;
            end
        end

        O = 35; H = 45; F = 30; M = 240;

        for t = 1:T
            if isSup(t)
                O = O - cO(t); H = H - cH(t); F = F - cF(t);
                if O < 0 || H < 0 || F < 0
                    feasible = false; Z_final = 0; M_final = 0; return;
                end
                if O + H + F > 120
                    feasible = false; Z_final = 0; M_final = 0; return;
                end

                nextSup = T + 1;
                for tt = t+1:T
                    if isSup(tt), nextSup = tt; break; end
                end

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
'''

with open(r'C:\Users\ming\Desktop\数模备赛\解法二_DP完善\method_dp_complete_task2.m', 'w', encoding='utf-8') as f:
    f.write(code)
print('MATLAB file written successfully.')
