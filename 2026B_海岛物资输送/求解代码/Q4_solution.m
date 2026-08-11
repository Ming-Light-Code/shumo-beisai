% ============================================================
% Q4 多波束测线优化 - MATLAB完整求解代码
% 数模备赛 B题 问题4
% 三种方案: Greedy(银), DAG-DP+SA(金), NSGA-II(冲击)
% ============================================================

%% 1. 数据加载与参数设置
clear; clc;
depth_data = readmatrix('附件.xlsx', 'Range', 'C3:GU253');
% depth_data: 251行 x 201列 (Y: 0~5NM南北, X: 0~4NM东西)
NM2M = 1852;
theta_half = 60 * pi / 180;  % 半开角
TAN60 = tan(theta_half);
SEA_Y = 5;    % 南北 5 NM
SEA_X = 4;    % 东西 4 NM
GRID_RES = 0.02;
OVERLAP_MAX = 0.20;

y_coords = (0:size(depth_data,1)-1)' * GRID_RES;  % 0~5 NM
x_coords = (0:size(depth_data,2)-1)' * GRID_RES;  % 0~4 NM

fprintf('Data: %dx%d, depth %.0f~%.0f m\n', ...
    size(depth_data), min(depth_data(:)), max(depth_data(:)));

%% 2. PCA分析 - 确定最优测线方向
[U, S, V] = svd(depth_data - mean(depth_data(:)), 'econ');
fprintf('PC1 explains %.1f%% variance\n', S(1)^2/sum(diag(S).^2)*100);

% 计算X和Y方向的平均深度变化
[~, dDdx] = gradient(depth_data);
[~, dDdy] = gradient(depth_data);
mean_dDdx = mean(abs(dDdx(:))) / GRID_RES;  % m/NM
mean_dDdy = mean(abs(dDdy(:))) / GRID_RES;  % m/NM
fprintf('Mean |dD/dX| = %.1f m/NM, |dD/dY| = %.1f m/NM\n', mean_dDdx, mean_dDdy);
fprintf('Recommend EW lines (沿东西, 每条4NM长)\n');

%% 3. 双线性插值函数
bilinear_interp = @(yq, xq) interp2(x_coords, y_coords, depth_data, xq, yq, 'linear');

%% 4. 条带半宽计算函数
function w_nm = strip_half_width(y_line, x_vals, depth_data, x_coords, y_coords, TAN60, NM2M)
    % y_line: 测线的Y坐标 (对于东西向测线)
    % x_vals: 沿测线的X采样点
    depths = interp2(x_coords, y_coords, depth_data, x_vals, ...
                     y_line * ones(size(x_vals)), 'linear');
    w_nm = depths * TAN60 / NM2M;
end

%% 5. 覆盖率指标计算
function [coverage_pct, leak_pct, exceed_len, total_len] = ...
    compute_metrics(line_positions, depth_data, x_coords, y_coords, SEA_X, SEA_Y, ...
                    TAN60, NM2M, OVERLAP_MAX, nx)
    if isempty(line_positions)
        coverage_pct = 0; leak_pct = 100; exceed_len = 0; total_len = 0;
        return;
    end
    
    x_eval = linspace(0, SEA_X, nx);
    dx = SEA_X / nx;
    total_area = SEA_X * SEA_Y;
    covered_area = 0;
    exceed_total = 0;
    
    for xi = 1:nx
        x = x_eval(xi);
        intervals = zeros(length(line_positions), 2);
        for k = 1:length(line_positions)
            yk = line_positions(k);
            d = interp2(x_coords, y_coords, depth_data, x, yk, 'linear');
            w = d * TAN60 / NM2M;
            intervals(k, :) = [yk - w, yk + w];
        end
        
        intervals = sortrows(intervals, 1);
        union_len = 0;
        cur_lo = intervals(1,1);
        cur_hi = intervals(1,2);
        for k = 2:length(line_positions)
            if intervals(k,1) <= cur_hi
                cur_hi = max(cur_hi, intervals(k,2));
            else
                union_len = union_len + cur_hi - cur_lo;
                cur_lo = intervals(k,1);
                cur_hi = intervals(k,2);
            end
        end
        union_len = union_len + cur_hi - cur_lo;
        covered_area = covered_area + union_len * dx;
        
        % 相邻条带重叠超标检查
        for k = 1:length(line_positions)-1
            yk = line_positions(k);
            yj = line_positions(k+1);
            dk = interp2(x_coords, y_coords, depth_data, x, yk, 'linear');
            dj = interp2(x_coords, y_coords, depth_data, x, yj, 'linear');
            wk = dk * TAN60 / NM2M;
            wj = dj * TAN60 / NM2M;
            overlap_val = (yk + wk) - (yj - wj);
            if overlap_val > 0
                total_w = wk + wj;
                overlap_rate = overlap_val / total_w;
                if overlap_rate > OVERLAP_MAX
                    exceed_total = exceed_total + ...
                        (overlap_val - OVERLAP_MAX * total_w) * dx;
                end
            end
        end
    end
    
    leak_pct = max(0, 100 * (total_area - covered_area) / total_area);
    coverage_pct = 100 * covered_area / total_area;
    total_len = length(line_positions) * SEA_X;
end

%% 6. 方案二(银弹): 贪心算法
function lines = solve_greedy(depth_data, x_coords, y_coords, SEA_X, SEA_Y, ...
                               TAN60, NM2M, nx_eval)
    x_eval = linspace(0, SEA_X, nx_eval);
    lines = [];
    current_y = 0;
    step = 0.005;
    
    % 第一条线
    for pos = step:step:SEA_Y
        depths = interp2(x_coords, y_coords, depth_data, x_eval, ...
                         pos * ones(size(x_eval)), 'linear');
        w_min = min(depths * TAN60 / NM2M);
        if pos - w_min <= 0
            lines(end+1) = pos;
            current_y = pos + w_min;
            break;
        end
    end
    
    % 后续线
    for iter = 1:500
        if current_y >= SEA_Y, break; end
        best_pos = -1; best_gap = inf;
        for pos = (current_y+step):step:min(SEA_Y, current_y+0.3)
            depths = interp2(x_coords, y_coords, depth_data, x_eval, ...
                             pos * ones(size(x_eval)), 'linear');
            w_min = min(depths * TAN60 / NM2M);
            if pos - w_min <= current_y && pos - w_min >= current_y - 0.03
                gap = current_y - (pos - w_min);
                if gap < best_gap
                    best_gap = gap; best_pos = pos;
                end
            end
        end
        if best_pos < 0
            if ~isempty(lines)
                depths = interp2(x_coords, y_coords, depth_data, x_eval, ...
                                 lines(end) * ones(size(x_eval)), 'linear');
                current_y = lines(end) + min(depths * TAN60 / NM2M);
            end
            continue;
        end
        lines(end+1) = best_pos;
        depths = interp2(x_coords, y_coords, depth_data, x_eval, ...
                         best_pos * ones(size(x_eval)), 'linear');
        current_y = best_pos + min(depths * TAN60 / NM2M);
    end
    
    % 确保覆盖北边界
    if ~isempty(lines)
        for pos = SEA_Y:-step:lines(end)
            depths = interp2(x_coords, y_coords, depth_data, x_eval, ...
                             pos * ones(size(x_eval)), 'linear');
            if pos + min(depths * TAN60 / NM2M) >= SEA_Y
                lines(end+1) = pos; break;
            end
        end
    end
    lines = sort(unique(lines));
end

%% 7. 方案一(黄金): DAG动态规划
function lines = solve_dag_dp(depth_data, x_coords, y_coords, SEA_X, SEA_Y, ...
                               TAN60, NM2M, OVERLAP_MAX, alpha, nx_eval)
    x_eval = linspace(0, SEA_X, nx_eval);
    step = 0.01;
    candidates = (step:step:SEA_Y)';
    N = length(candidates);
    min_w = zeros(N, 1);
    
    for i = 1:N
        depths = interp2(x_coords, y_coords, depth_data, x_eval, ...
                         candidates(i) * ones(size(x_eval)), 'linear');
        min_w(i) = min(depths * TAN60 / NM2M);
    end
    
    INF = 1e10;
    dp = INF * ones(N, 1);
    prev = -ones(N, 1, 'int32');
    
    % 初始化: 从南边界开始
    for i = 1:N
        if candidates(i) - min_w(i) <= 0
            dp(i) = SEA_X + alpha * max(0, candidates(i) - min_w(i));
        end
    end
    
    % DP转移
    for j = 1:N
        for i = 1:j-1
            if dp(i) >= INF, continue; end
            d = candidates(j) - candidates(i);
            if d > min_w(i) + min_w(j), continue; end  % 有空隙
            overlap = min_w(i) + min_w(j) - d;
            excess = max(0, overlap - OVERLAP_MAX * (min_w(i) + min_w(j)));
            cost_ij = SEA_X + alpha * excess;
            if dp(i) + cost_ij < dp(j)
                dp(j) = dp(i) + cost_ij;
                prev(j) = i;
            end
        end
    end
    
    % 最优终点: 覆盖北边界
    best_end = -1; best_cost = INF;
    for i = 1:N
        if candidates(i) + min_w(i) >= SEA_Y && dp(i) < best_cost
            best_cost = dp(i); best_end = i;
        end
    end
    
    if best_end < 0, lines = []; return; end
    
    % 回溯路径
    lines = [];
    cur = best_end;
    while cur > 0
        lines = [candidates(cur); lines];
        cur = prev(cur);
    end
end

%% 8. 方案一(黄金): 模拟退火精调
function [best_lines, best_cost] = solve_sa(depth_data, x_coords, y_coords, ...
    init_lines, SEA_X, SEA_Y, TAN60, NM2M, OVERLAP_MAX, iters)
    
    cost_fn = @(ls) compute_sa_cost(ls, depth_data, x_coords, y_coords, ...
        SEA_X, SEA_Y, TAN60, NM2M, OVERLAP_MAX);
    
    lines = sort(init_lines);
    best_lines = lines;
    best_cost_val = cost_fn(best_lines);
    cur_cost = best_cost_val;
    
    T0 = 0.5; Te = 0.001;
    cr = (Te/T0)^(1/iters);
    T = T0;
    rng(42);
    
    for iter = 1:iters
        new_lines = lines;
        op = randi([1,3]);
        if op == 1 && length(new_lines) > 1
            idx = randi([1, length(new_lines)]);
            new_lines(idx) = new_lines(idx) + randn * 0.015;
            new_lines = sort(max(0.01, min(SEA_Y-0.01, new_lines)));
        elseif op == 2 && length(new_lines) > 1
            idx = randi([1, length(new_lines)]);
            new_lines(idx) = [];
        elseif op == 3
            new_lines = sort([new_lines, rand*(SEA_Y-0.02)+0.01]);
        end
        
        new_cost = cost_fn(new_lines);
        delta = new_cost - cur_cost;
        if delta < 0 || rand < exp(-delta/max(T, 1e-8))
            lines = new_lines; cur_cost = new_cost;
            if cur_cost < best_cost_val
                best_cost_val = cur_cost; best_lines = lines;
            end
        end
        T = T * cr;
    end
    best_cost = best_cost_val;
end

function cost = compute_sa_cost(lines, depth_data, x_coords, y_coords, ...
    SEA_X, SEA_Y, TAN60, NM2M, OVERLAP_MAX)
    if length(lines) < 2
        cost = 1e10; return;
    end
    [~, leak, exceed, total_len] = compute_metrics(lines, depth_data, ...
        x_coords, y_coords, SEA_X, SEA_Y, TAN60, NM2M, OVERLAP_MAX, 80);
    cost = leak * 500 + exceed * 5 + total_len;
end

%% 9. 主执行
fprintf('\n========== Q4 求解 ==========\n');

% 运行贪心
fprintf('Running Greedy...\n');
greedy_lines = solve_greedy(depth_data, x_coords, y_coords, SEA_X, SEA_Y, ...
    TAN60, NM2M, 120);
[gc, gl, ge, glen] = compute_metrics(greedy_lines, depth_data, x_coords, ...
    y_coords, SEA_X, SEA_Y, TAN60, NM2M, OVERLAP_MAX, 120);
fprintf('Greedy: %d lines, leak=%.2f%%, exceed=%.4f NM, len=%.2f NM\n', ...
    length(greedy_lines), gl, ge, glen);

% 运行DAG-DP
fprintf('Running DAG-DP...\n');
dp_lines = solve_dag_dp(depth_data, x_coords, y_coords, SEA_X, SEA_Y, ...
    TAN60, NM2M, OVERLAP_MAX, 0.5, 80);
[dc, dl, de, dlen] = compute_metrics(dp_lines, depth_data, x_coords, ...
    y_coords, SEA_X, SEA_Y, TAN60, NM2M, OVERLAP_MAX, 120);
fprintf('DAG-DP: %d lines, leak=%.2f%%, exceed=%.4f NM, len=%.2f NM\n', ...
    length(dp_lines), dl, de, dlen);

% 运行SA
fprintf('Running SA...\n');
[sa_lines, ~] = solve_sa(depth_data, x_coords, y_coords, dp_lines, ...
    SEA_X, SEA_Y, TAN60, NM2M, OVERLAP_MAX, 1500);
[sc, sl, se, slen] = compute_metrics(sa_lines, depth_data, x_coords, ...
    y_coords, SEA_X, SEA_Y, TAN60, NM2M, OVERLAP_MAX, 120);
fprintf('SA+DP: %d lines, leak=%.2f%%, exceed=%.4f NM, len=%.2f NM\n', ...
    length(sa_lines), sl, se, slen);

%% 10. 最终对比
fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('FINAL COMPARISON\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('%-12s %8s %10s %12s %10s\n', 'Method', '#Lines', 'Leak%', 'Exceed(NM)', 'Len(NM)');
fprintf('%-12s %8s %10s %12s %10s\n', '------------', '------', '----------', '------------', '----------');
fprintf('%-12s %8d %9.2f%% %11.4f %9.2f\n', 'Greedy', length(greedy_lines), gl, ge, glen);
fprintf('%-12s %8d %9.2f%% %11.4f %9.2f\n', 'DAG-DP', length(dp_lines), dl, de, dlen);
fprintf('%-12s %8d %9.2f%% %11.4f %9.2f\n', 'SA+DP', length(sa_lines), sl, se, slen);
