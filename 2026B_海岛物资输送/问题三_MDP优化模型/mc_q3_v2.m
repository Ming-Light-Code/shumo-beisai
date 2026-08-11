function mc_q3_v2(N)
% MC_Q3_V2  Monte Carlo verification for Problem 3 (refactored).

if nargin < 1 || isempty(N), N = 100; end

fprintf('========================================\n');
fprintf('  Problem 3 MC Verification (N=%d)\n', N);
fprintf('========================================\n\n');

cfg = params_q3_v2();

Zr = NaN(1,N); Mr = NaN(1,N);
ok = false(1,N); dy = NaN(1,N); fr = cell(1,N);
ri = max(1, floor(N/20));
tic;

for sim = 1:N
    weather_seq = rand(1, cfg.T_MAX) < cfg.pN;
    result = sim_q3_v2(cfg, weather_seq);
    Zr(sim) = result.Z; Mr(sim) = result.M;
    ok(sim) = result.arrived; dy(sim) = result.day;
    fr{sim} = result.reason;

    if mod(sim, ri) == 0
        fprintf(' %d/%d (%.0f%%) | %.1fs | Success: %.1f%%\n', ...
            sim, N, 100*sim/N, toc, 100*sum(ok(1:sim))/sim);
    end
end
tt = toc; ns = sum(ok);
fprintf('\nDone. %.1fs\n\n', tt);

fprintf('========================================\n');
fprintf('  Results\n');
fprintf('========================================\n');
fprintf('Success rate: %d/%d (%.1f%%)\n', ns, N, 100*ns/N);

if ns > 0
    Zs = Zr(ok); Ms = Mr(ok); Ds = dy(ok);
    fprintf('\n--- Successful (n=%d) ---\n', ns);
    fprintf('%-10s %8s %8s %8s %8s\n', '', 'Mean', 'Std', 'Min', 'Max');
    fprintf('%-10s %8.1f %8.1f %8d %8d\n', 'Z', mean(Zs), std(Zs), min(Zs), max(Zs));
    fprintf('%-10s %8.1f %8.1f %8.1f %8.1f\n', 'M', mean(Ms), std(Ms), min(Ms), max(Ms));
    fprintf('%-10s %8.1f %8.1f %8d %8d\n', 'Days', mean(Ds), std(Ds), min(Ds), max(Ds));
    fprintf('%-10s %8.1f\n', 'Z_net', mean(Zs)-200);
end

if N - ns > 0
    fprintf('\n--- Failures (%d) ---\n', N-ns);
    [u,~,ic] = unique(fr(~ok));
    cnt = accumarray(ic, 1);
    for i = 1:length(u)
        fprintf('  %s: %d\n', u{i}, cnt(i));
    end
end

fprintf('\nDone.\n');
end
