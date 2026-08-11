function solve_cp_v2()
% solve_cp_v2.m - Improved CP solver for Task 1 (Normal Weather)
% Improvements over v1:
%   - Uses shared cp_common library (eliminates code duplication)
%   - Fixed path-string output (no stray quote)
%   - Clearer day-count display
%   - schedule struct properly populated
%   - Config struct for easy parameter tuning

% ===== Configuration =====
cfg.MAX_DAYS = 30;
cfg.MAX_LOAD = 120;
cfg.all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];  % B,E,W1,W2,W3,S1,S2
cfg.names  = {'B','E','W1','W2','W3','S1','S2'};
cfg.WY = [20, 15, 28];   % daily yield
cfg.WM = [4, 5, 3];       % max consecutive work days

% Initial state
cfg.init.O = 35; cfg.init.H = 45; cfg.init.F = 30;
cfg.init.M = 240; cfg.init.Z = 100;

% Normal weather consumption parameters
cfg.cons = struct(...
    'MO', 2, 'MH', 3, 'MF', 2, ...   % move per cell
    'PO', 1, 'PH', 1, 'PF', 1, ...   % park per day
    'WO', 5, 'WH', 4, 'WF', 3, ...   % work per day
    'pO', 2, 'pH', 1, 'pF', 2);      % supply prices

% ===== Precompute distances =====
n_pts = size(cfg.all_xy, 1);
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = abs(cfg.all_xy(i,1)-cfg.all_xy(j,1)) + abs(cfg.all_xy(i,2)-cfg.all_xy(j,2));
    end
end
cfg.dist = dist;
cfg.inter_idx = [3 4 5 6 7];  % W1,W2,W3,S1,S2

fprintf('========================================\n');
fprintf('  CP for Task 1 (v2: Normal Weather)\n');
fprintf('========================================\n');
fprintf('Move:  (O=%d,H=%d,F=%d)/cell | Park: (O=%d,H=%d,F=%d)/day | Work: (O=%d,H=%d,F=%d)/day\n', ...
    cfg.cons.MO, cfg.cons.MH, cfg.cons.MF, cfg.cons.PO, cfg.cons.PH, cfg.cons.PF, cfg.cons.WO, cfg.cons.WH, cfg.cons.WF);
fprintf('Init:  O=%d H=%d F=%d M=%d Z=%d Load<=%d\n\n', cfg.init.O, cfg.init.H, cfg.init.F, cfg.init.M, cfg.init.Z, cfg.MAX_LOAD);

% ===== CP Search =====
best_Z = -inf; best_M = -inf; best_path = [1, 2];
best_work_days = []; best_park_seg = [];
nodes = 0;

[best_Z, best_M, best_path, best_work_days, best_park_seg, nodes] = ...
    cp_search([1], 0, [], [], best_Z, best_M, best_path, best_work_days, best_park_seg, cfg, nodes, 0);

% ===== Output =====
fprintf('\n===== OPTIMAL SOLUTION =====\n');
fprintf('Z = %d\n', best_Z);
fprintf('M = %d\n', best_M);

% Fixed: clean path string without stray quote
path_parts = cell(1, length(best_path));
for i = 1:length(best_path), path_parts{i} = cfg.names{best_path(i)}; end
fprintf('Path: %s\n', strjoin(path_parts, ' -> '));

tt = 0;
for k = 1:(length(best_path)-1)
    tt = tt + dist(best_path(k), best_path(k+1));
end
total_park = sum(best_park_seg);
work_only = sum(best_work_days);
park_reset = 0;
wp_idx = 1;
for i = 2:length(best_path)
    if best_path(i) >= 3 && best_path(i) <= 5
        wi = best_path(i) - 2;
        if wp_idx <= length(best_work_days) && best_work_days(wp_idx) > 0
            park_reset = park_reset + max(0, ceil(best_work_days(wp_idx) / cfg.WM(wi)) - 1);
        end
        wp_idx = wp_idx + 1;
    end
end
fprintf('Travel: %d days | Work: %d days | Park (reset): %d days | Total trip: %d days\n', ...
    tt, work_only, park_reset, tt + work_only + park_reset + total_park);
fprintf('Nodes explored: %d\n', nodes);

% Print detailed schedule
cp_common('print_schedule', best_path, best_work_days, best_park_seg, ...
    dist, cfg.WM, cfg.WY, cfg.cons, cfg.MAX_DAYS, cfg.MAX_LOAD, cfg.all_xy, cfg.names, cfg.init);

fprintf('\nDone.\n');
end

% ===== CP Recursive Search =====
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search(path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, cfg, nodes, depth)
    nodes = nodes + 1;
    lp = path(end);

    % Upper-bound pruning
    if lp ~= 2
        dE = cfg.dist(lp, 2);
        rem = cfg.MAX_DAYS - tsf;
        if rem < dE, return; end
        ub = cfg.init.Z + cp_common('max_work_with_park', 3, rem - dE) * 28;
        if ub <= bZ && bZ > -inf, return; end
    end

    % Evaluate leaf: append E and try work/park combinations
    dE = cfg.dist(lp, 2);
    if tsf + dE <= cfg.MAX_DAYS
        fp = [path, 2];
        m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1), tt = tt + cfg.dist(fp(k), fp(k+1)); end
        rem_days = cfg.MAX_DAYS - tt;
        nw = length(wa);

        if nw == 0
            % No work points: just check if path is feasible
            [ok, Z, M, ~] = cp_common('simulate', fp, m, cfg.dist, [], [], tt, [], [], ...
                cfg.cons, cfg.MAX_LOAD, cfg.init);
            if ok && (Z > bZ || (Z == bZ && M > bM))
                bZ = Z; bM = M; bP = fp; bWD = []; bPS = [];
            end
        else
            % Enumerate work-day allocations for all work points
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = cp_common('max_work_with_park', cfg.WM(ww(j)), rem_days);
            end
            sz = max_wk + 1;
            nc = prod(sz);
            for ci = 1:nc
                wd = zeros(1, nw);
                t2 = ci - 1;
                for j = nw:-1:1
                    wd(j) = mod(t2, sz(j));
                    t2 = floor(t2 / sz(j));
                end
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/cfg.WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > cfg.MAX_DAYS, continue; end
                [ok, Z, M, ~] = cp_common('simulate', fp, m, cfg.dist, wa, ww, tt, wd, [], ...
                    cfg.cons, cfg.MAX_LOAD, cfg.init);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bWD = wd; bPS = [];
                end
            end
        end
    end

    % Branch: try each intermediate point
    for ni = 1:5
        np = cfg.inter_idx(ni);
        if np == lp, continue; end  % skip same-point repeats
        d = cfg.dist(lp, np);
        if tsf + d > cfg.MAX_DAYS, continue; end
        dE2 = cfg.dist(np, 2);
        if tsf + d + dE2 > cfg.MAX_DAYS, continue; end  % forward check

        np2 = [path, np];
        nt = tsf + d;
        nwa = wa; nww = ww;
        if np >= 3 && np <= 5   % work point
            nwa(end+1) = length(np2);
            nww(end+1) = np - 2;
        end
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search(np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, cfg, nodes, depth+1);
    end
end
