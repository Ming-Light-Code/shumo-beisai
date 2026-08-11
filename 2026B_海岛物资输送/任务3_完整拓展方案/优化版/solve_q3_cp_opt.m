function solve_q3_cp_opt()
% =========================================================================
%  solve_q3_cp_opt.m — 任务3 优化版离线CP求解器
%  改进: P0差异化裕度 + P1预防性超购 + P4代码重构(调用cp_engine_opt)
% =========================================================================

cfg = cp_engine_opt('task3_config');
cons = cp_engine_opt('get_cons', 'expected', cfg);

fprintf('========================================\n');
fprintf('  任务3 优化版离线CP求解器\n');
fprintf('========================================\n');
fprintf('P0 差异化裕度: O×%.2f H×%.2f F×%.2f\n', cons.SAFETY_O, cons.SAFETY_H, cons.SAFETY_F);
fprintf('P1 预防性超购: 每次补给追加10%%剩余空间\n');
fprintf('网格: 30×30 | 时限: %d天 | 载重: %d\n\n', cfg.MAX_DAYS, cfg.MAX_LOAD);

fprintf('开始CP搜索...\n'); tic;
[best_path, best_parks, best_works, feasible] = ...
    cp_engine_opt('plan', 1, 0, cons, cfg, false);
elapsed = toc;

if ~feasible, fprintf('未找到可行解。\n'); return; end

% 整理结果
m = length(best_path) - 2;
tt = 0;
for k = 1:(m+1), tt = tt + cfg.dist(best_path(k), best_path(k+1)); end

total_work = sum(best_works);
total_park_reset = 0; wp_count = 0;
for i = 2:length(best_path)
    if best_path(i) >= 3 && best_path(i) <= 5
        wp_count = wp_count + 1;
        if wp_count <= length(best_works) && best_works(wp_count) > 0
            total_park_reset = total_park_reset + max(0, ceil(best_works(wp_count)/cfg.WM(best_path(i)-2)) - 1);
        end
    end
end

% 模拟得到最终 Z/M
init_s = cfg.init;
[~, finalZ, finalM] = cp_engine_opt('simulate', best_path, m, cfg.dist, ...
    rebuild_wa(best_path), rebuild_ww(best_path), tt, best_works, best_parks, cons, cfg, init_s);

fprintf('\n===== 最优解 =====\n');
fprintf('Z = %d\n', round(finalZ));
fprintf('M = %.2f\n', finalM);
path_str = strjoin(cfg.names(best_path), ' → ');
fprintf('路径: %s\n', path_str);
fprintf('旅行: %d天 | 作业: %d天 | 停泊(重置): %d天 | 总计: %d天\n', ...
    tt, total_work, total_park_reset, tt + total_work + total_park_reset + sum(best_parks));
fprintf('耗时: %.2f秒\n\n', elapsed);

if ~isempty(best_works)
    fprintf('作业安排:\n');
    wp_count = 0;
    for i = 2:length(best_path)
        if best_path(i) >= 3 && best_path(i) <= 5
            wp_count = wp_count + 1;
            if wp_count <= length(best_works)
                np = max(0, ceil(best_works(wp_count)/cfg.WM(best_path(i)-2)) - 1);
                fprintf('  %s: %d天 + %d停泊\n', cfg.names{best_path(i)}, best_works(wp_count), np);
            end
        end
    end
end

fprintf('\nDone.\n');
end

function wa = rebuild_wa(bP)
    wa = []; for i = 2:length(bP)
        if bP(i) >= 3 && bP(i) <= 5, wa(end+1) = i; end
    end
end

function ww = rebuild_ww(bP)
    ww = []; for i = 2:length(bP)
        if bP(i) >= 3 && bP(i) <= 5, ww(end+1) = bP(i) - 2; end
    end
end
