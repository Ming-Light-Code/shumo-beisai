function solve_q3_rso_mc(N)
% SOLVE_Q3_RSO_MC  Task3 Monte Carlo Verification
% Runs N independent weather realizations for the best skeleton
% Reports: success rate, E[Z], E[M], Z distribution, failure reasons

if nargin<1 || isempty(N), N = 500; end

cfg = cp_engine('config');
fprintf('========================================\n');
fprintf('  Task3 SBSP MC Verification (N=%d)\n', N);
fprintf('========================================\n\n');

% --- Find best skeleton first (quick evaluation) ---
fprintf('Finding best skeleton...\n');
[skels, ~] = cp_engine('skeletons');
best_Z = -inf;  best_M = -inf;  best_skel = [];

for si = 1:length(skels)
    skel = skels{si};
    [mZ, mM, succ, ~] = rso_solver('evaluate', skel, min(100, cfg.MC_N), cfg);
    is_better = (mZ > best_Z) || (mZ == best_Z && mM > best_M);
    if is_better && succ > 0
        best_Z = mZ;  best_M = mM;  best_skel = skel;
    end
end

if isempty(best_skel)
    fprintf('No feasible skeleton found!\n'); return;
end

info = cp_engine('skeleton_info', best_skel);
pstr = '';
for i = 1:length(best_skel)
    pstr = [pstr, ' -> ', cfg.names{best_skel(i)}];
end
fprintf('Best skeleton: %s\n\n', pstr);

% --- Full MC verification ---
fprintf('Running %d MC simulations...\n', N);
tic;
Zr = NaN(1,N);  Mr = NaN(1,N);  ok = false(1,N);
dy = NaN(1,N);  fr = cell(1,N);
ri = max(1, floor(N/10));

for sim = 1:N
    ws = cp_engine('weather', cfg.MAX_DAYS, cfg.p_normal);
    [fok, Zf, Mf, d, reason] = rso_solver('simulate', best_skel, ws, cfg);
    Zr(sim) = Zf;  Mr(sim) = Mf;  ok(sim) = fok;
    dy(sim) = d;  fr{sim} = reason;
    
    if mod(sim, ri) == 0
        fprintf('  %d/%d (%.0f%%) | Succ: %.1f%% | %.1fs\n', ...
            sim, N, 100*sim/N, 100*sum(ok(1:sim))/sim, toc);
    end
end
tt = toc;

% --- Results ---
ns = sum(ok);
fprintf('\n========================================\n');
fprintf('  RESULTS (N=%d, %.1fs)\n', N, tt);
fprintf('========================================\n');
fprintf('Success rate: %d/%d (%.1f%%)\n\n', ns, N, 100*ns/N);

if ns > 0
    Zs = Zr(ok);  Ms = Mr(ok);  Ds = dy(ok);
    fprintf('--- Successful runs (n=%d) ---\n', ns);
    fprintf('%-12s %8s %8s %8s %8s\n', '', 'Mean', 'Std', 'Min', 'Max');
    fprintf('%-12s %8.1f %8.1f %8.0f %8.0f\n', 'Z', mean(Zs), std(Zs), min(Zs), max(Zs));
    fprintf('%-12s %8.2f %8.2f %8.2f %8.2f\n', 'M', mean(Ms), std(Ms), min(Ms), max(Ms));
    fprintf('%-12s %8.1f %8.1f %8.0f %8.0f\n', 'Days', mean(Ds), std(Ds), min(Ds), max(Ds));
end

if N - ns > 0
    fprintf('\n--- Failure analysis (n=%d) ---\n', N-ns);
    [u,~,ic] = unique(fr(~ok));
    cnt = accumarray(ic, 1);
    for i = 1:length(u)
        fprintf('  %s: %d (%.1f%%)\n', u{i}, cnt(i), 100*cnt(i)/(N-ns));
    end
end

fprintf('\nDone.\n');
end
