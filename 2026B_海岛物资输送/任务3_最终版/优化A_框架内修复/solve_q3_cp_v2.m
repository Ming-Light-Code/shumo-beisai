function solve_q3_cp_v2()
% SOLVE_Q3_CP_V2  Task3 Offline CP v2
% Uses improved engine with accumulated-Z pruning

cfg = cp_engine_v2('config');
cons = cp_engine_v2('cons', 'expected');

fprintf('========================================
');
fprintf('  Task3 Offline CP v2 (Improved Engine)
');
fprintf('========================================
');
fprintf('SAFETY: Ox1.05 Hx1.05 Fx1.05 | Time:%dd | Load:%d

', cfg.MAX_DAYS, cfg.MAX_LOAD);

fprintf('Starting CP search...
'); tic;
[best_path, best_parks, best_works, feasible] = cp_engine_v2('plan', 1, 0, cons, cfg, false);
elapsed = toc;

if ~feasible, fprintf('No feasible solution found.
'); return; end

m = length(best_path) - 2; tt = 0;
for k = 1:(m+1), tt = tt + cfg.dist(best_path(k), best_path(k+1)); end

init_s = cfg.init;
[~, finalZ, finalM] = cp_engine_v2('simulate', best_path, m, cfg.dist, ...
    wa_of(best_path), ww_of(best_path), tt, best_works, best_parks, cons, cfg, init_s);

fprintf('
===== Optimal Solution =====
');
fprintf('Z = %d  |  M = %.0f
', round(finalZ), finalM);
fprintf('Path: %s
', strjoin(cfg.names(best_path), ' -> '));

tw = sum(best_works); tpr = 0; wc = 0;
for i = 2:length(best_path)
    if best_path(i) >= 3 && best_path(i) <= 5
        wc = wc + 1;
        if wc <= length(best_works) && best_works(wc) > 0
            tpr = tpr + max(0, ceil(best_works(wc)/cfg.WM(best_path(i)-2)) - 1);
        end
    end
end
fprintf('Travel:%dd | Work:%dd | Park:%dd | Total:%dd | Time:%.2fs
', ...
    tt, tw, tpr, tt+tw+tpr+sum(best_parks), elapsed);

if ~isempty(best_works)
    fprintf('Work plan: '); wc = 0;
    for i = 2:length(best_path)
        if best_path(i) >= 3 && best_path(i) <= 5
            wc = wc + 1;
            if wc <= length(best_works)
                np = max(0, ceil(best_works(wc)/cfg.WM(best_path(i)-2)) - 1);
                fprintf('%s=%dd(+%d park) ', cfg.names{best_path(i)}, best_works(wc), np);
            end
        end
    end
    fprintf('
');
end
fprintf('
Done.
');
end

function wa = wa_of(bP)
    wa = [];
    for i=2:length(bP)
        if bP(i)>=3&&bP(i)<=5, wa(end+1)=i; end
    end
end

function ww = ww_of(bP)
    ww = [];
    for i=2:length(bP)
        if bP(i)>=3&&bP(i)<=5, ww(end+1)=bP(i)-2; end
    end
end
