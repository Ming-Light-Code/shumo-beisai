function solve_q3_cp()
% =========================================================================
%  solve_q3_cp.m — 任务3 离线期望消耗CP求解器 (最终版)
%  使用 cp_engine 共享引擎，自动搜索全部路径骨架
%  最优解: Z=652, M=4, B→W1→S1→S2→W3→S2→E (84天)
% =========================================================================

cfg = cp_engine('config');
cons = cp_engine('cons', 'expected');

fprintf('========================================\n');
fprintf('  任务3 离线CP求解器 (最终版)\n');
fprintf('========================================\n');
fprintf('SAFETY: O×%.2f H×%.2f F×%.2f | 时限:%d天 | 载重:%d\n\n', 1.05,1.05,1.05, cfg.MAX_DAYS, cfg.MAX_LOAD);

fprintf('开始CP搜索...\n'); tic;
[best_path, best_parks, best_works, feasible] = cp_engine('plan', 1, 0, cons, cfg, false);
elapsed = toc;

if ~feasible, fprintf('未找到可行解。\n'); return; end

m = length(best_path) - 2; tt = 0;
for k = 1:(m+1), tt = tt + cfg.dist(best_path(k), best_path(k+1)); end

init_s = cfg.init;
[~, finalZ, finalM] = cp_engine('simulate', best_path, m, cfg.dist, ...
    wa_of(best_path), ww_of(best_path), tt, best_works, best_parks, cons, cfg, init_s);

fprintf('\n===== 最优解 =====\n');
fprintf('Z = %d  |  M = %.0f\n', round(finalZ), finalM);
fprintf('路径: %s\n', strjoin(cfg.names(best_path), ' → '));

tw = sum(best_works); tpr = 0; wc = 0;
for i = 2:length(best_path)
    if best_path(i) >= 3 && best_path(i) <= 5
        wc = wc + 1;
        if wc <= length(best_works) && best_works(wc) > 0
            tpr = tpr + max(0, ceil(best_works(wc)/cfg.WM(best_path(i)-2)) - 1);
        end
    end
end
fprintf('旅行:%d天 | 作业:%d天 | 停泊:%d天 | 总计:%d天 | 耗时:%.2fs\n', ...
    tt, tw, tpr, tt+tw+tpr+sum(best_parks), elapsed);

if ~isempty(best_works)
    fprintf('作业安排: '); wc = 0;
    for i = 2:length(best_path)
        if best_path(i) >= 3 && best_path(i) <= 5
            wc = wc + 1;
            if wc <= length(best_works)
                np = max(0, ceil(best_works(wc)/cfg.WM(best_path(i)-2)) - 1);
                fprintf('%s=%d天(+%d停泊) ', cfg.names{best_path(i)}, best_works(wc), np);
            end
        end
    end
    fprintf('\n');
end
fprintf('\nDone.\n');
end

function wa = wa_of(bP), wa = []; for i=2:length(bP), if bP(i)>=3&&bP(i)<=5, wa(end+1)=i; end; end; end
function ww = ww_of(bP), ww = []; for i=2:length(bP), if bP(i)>=3&&bP(i)<=5, ww(end+1)=bP(i)-2; end; end; end
