function solve_q3_rso()
% SOLVE_Q3_RSO  Task3 Main Solver (Skeleton-Based Stochastic Planning)
% 1. Enumerate all feasible path skeletons
% 2. Evaluate each skeleton with Monte Carlo (adaptive online policy)
% 3. Select the lexicographically best (max expected Z, then max expected M)
% 4. Output detailed results

cfg = cp_engine('config');
fprintf('========================================\n');
fprintf('  Task3 SBSP Solver (Skeleton-Based)\n');
fprintf('========================================\n');
fprintf('Grid: 30x30  |  Days: %d  |  p(normal)=%.1f\n', cfg.MAX_DAYS, cfg.p_normal);
fprintf('Init: O=%d H=%d F=%d M=%d Z=%d  |  Load<=%d\n', ...
    cfg.init.O, cfg.init.H, cfg.init.F, cfg.init.M, cfg.init.Z, cfg.MAX_LOAD);
fprintf('MC samples per skeleton: %d\n\n', cfg.MC_N);

% --- Phase 1: Skeleton Enumeration ---
fprintf('--- Phase 1: Enumerating Skeletons ---\n');
tic;
[skels, n_nodes] = cp_engine('skeletons');
fprintf('Explored %d nodes, found %d feasible skeletons (%.1fs)\n\n', ...
    n_nodes, length(skels), toc);

% --- Phase 2: Evaluate Each Skeleton ---
fprintf('--- Phase 2: Monte Carlo Evaluation ---\n');
n_skels = length(skels);
best_Z = -inf;  best_M = -inf;  best_skel = [];
best_succ = 0;  best_idx = 0;
all_results = cell(1, n_skels);

report_interval = max(1, floor(n_skels / 10));
tic_total = tic;

for si = 1:n_skels
    skel = skels{si};
    info = cp_engine('skeleton_info', skel);
    [mZ, mM, succ, res] = rso_solver('evaluate', skel, cfg.MC_N, cfg);
    all_results{si} = struct('skel',skel,'meanZ',mZ,'meanM',mM,'succ',succ,'info',info);
    
    % Lexicographic comparison: Z first, then M
    is_better = (mZ > best_Z) || (mZ == best_Z && mM > best_M);
    if is_better && succ > 0
        best_Z = mZ;  best_M = mM;  best_skel = skel;
        best_succ = succ;  best_idx = si;
    end
    
    if mod(si, report_interval) == 0 || si == n_skels
        fprintf('  %3d/%d (%.0f%%) | Best: Z=%.1f M=%.1f succ=%.1f%% | %.1fs\n', ...
            si, n_skels, 100*si/n_skels, best_Z, best_M, 100*best_succ, toc(tic_total));
    end
end
fprintf('\nTotal evaluation time: %.1fs\n\n', toc(tic_total));

% --- Phase 3: Report Best Solution ---
fprintf('========================================\n');
fprintf('  BEST SOLUTION\n');
fprintf('========================================\n');
if isempty(best_skel)
    fprintf('No feasible skeleton found!\n'); return;
end

info = cp_engine('skeleton_info', best_skel);
pstr = '';
for i = 1:length(best_skel)
    pstr = [pstr, ' -> ', cfg.names{best_skel(i)}];
end
fprintf('Skeleton:  %s\n', pstr);
fprintf('Expected Z:    %.1f\n', best_Z);
fprintf('Expected M:    %.1f\n', best_M);
fprintf('Success rate:  %.1f%%\n', 100*best_succ);
fprintf('Travel days:   %d\n', info.travel_days);
fprintf('Remaining:     %d days\n', info.remaining_for_work);

% Top-5 candidates
fprintf('\n--- Top-5 Candidates ---\n');
fprintf('%-4s %-6s %-8s %-8s %-6s %s\n', 'Rank','Succ%%','E[Z]','E[M]','Travel','Skeleton');
valid_results = {};
for si = 1:n_skels
    r = all_results{si};
    if r.succ > 0
        valid_results{end+1} = r;
    end
end
[~, sort_idx] = sort(cellfun(@(x) x.meanZ, valid_results), 'descend');
for ri = 1:min(5, length(sort_idx))
    r = valid_results{sort_idx(ri)};
    pstr = '';
    for i = 1:length(r.skel)
        pstr = [pstr, '->', cfg.names{r.skel(i)}];
    end
    fprintf('%-4d %5.1f%% %8.1f %8.1f %6d %s\n', ...
        ri, 100*r.succ, r.meanZ, r.meanM, r.info.travel_days, pstr);
end

fprintf('\nDone.\n');
end
