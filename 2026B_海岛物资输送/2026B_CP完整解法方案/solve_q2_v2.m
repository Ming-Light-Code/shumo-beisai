function solve_q2_v2()
% solve_q2_v2.m - Improved solver for Task 2 (30-Day All-Thunderstorm)
% Improvements over original solve_q2.m / solve_q2_cp.m:
%   - Uses shared cp_common library
%   - Fixed day-counter display bug (park + supply no longer on same day)
%   - Path coordinates computed from distance matrix (no hardcoded lists)
%   - Unified CP + simple-enumeration hybrid for cross-validation
%   - Config struct for easy parameter tuning

% ===== Configuration =====
cfg.MAX_DAYS = 30;
cfg.MAX_LOAD = 120;
cfg.all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];  % B,E,W1,W2,W3,S1,S2
cfg.names  = {'B','E','W1','W2','W3','S1','S2'};
cfg.WY = [20, 15, 28];
cfg.WM = [4, 5, 3];

% Initial state
cfg.init.O = 35; cfg.init.H = 45; cfg.init.F = 30;
cfg.init.M = 240; cfg.init.Z = 100;

% Thunderstorm consumption parameters
cfg.cons = struct(...
    'MO', 8, 'MH', 4, 'MF', 3, ...   % move per cell
    'PO', 3, 'PH', 3, 'PF', 2, ...   % park per day
    'WO', 8, 'WH', 6, 'WF', 6, ...   % work per day
    'pO', 2, 'pH', 1, 'pF', 2);      % supply prices (unchanged)

% ===== Precompute distances =====
n_pts = size(cfg.all_xy, 1);
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = abs(cfg.all_xy(i,1)-cfg.all_xy(j,1)) + abs(cfg.all_xy(i,2)-cfg.all_xy(j,2));
    end
end
cfg.dist = dist;
cfg.inter_idx = [3 4 5 6 7];

fprintf('========================================\n');
fprintf('  Task 2 (v2): 30-Day Thunderstorm Extreme\n');
fprintf('========================================\n');
fprintf('Move:  (O=%d,H=%d,F=%d)/cell | Park: (O=%d,H=%d,F=%d)/day | Work: (O=%d,H=%d,F=%d)/day\n', ...
    cfg.cons.MO, cfg.cons.MH, cfg.cons.MF, cfg.cons.PO, cfg.cons.PH, cfg.cons.PF, cfg.cons.WO, cfg.cons.WH, cfg.cons.WF);
fprintf('Init:  O=%d H=%d F=%d M=%d Z=%d Load<=%d\n\n', cfg.init.O, cfg.init.H, cfg.init.F, cfg.init.M, cfg.init.Z, cfg.MAX_LOAD);

% ===== Connectivity Analysis =====
fprintf('--- Connectivity Analysis ---\n');
for pt = [6, 7, 3, 4, 5]  % S1, S2, W1, W2, W3
    d_b = dist(1, pt);
    o_need = d_b * cfg.cons.MO;
    fprintf('B -> %s: %d cells, O_need = %d %s\n', cfg.names{pt}, d_b, o_need, ...
        iif(o_need <= cfg.init.O, '(REACHABLE)', '(UNREACHABLE)'));
end
fprintf('\n');

% ===== Method 1: CP Search (comprehensive) =====
fprintf('--- Method 1: CP Search ---\n');
best_Z = -inf; best_M = -inf; best_path = [1, 2];
best_work_days = []; best_park_seg = [];
nodes = 0;

[best_Z, best_M, best_path, best_work_days, best_park_seg, nodes] = ...
    cp_search_q2([1], 0, [], [], best_Z, best_M, best_path, best_work_days, best_park_seg, cfg, nodes, 0);

fprintf('\n===== OPTIMAL SOLUTION =====\n');
fprintf('Z = %d\n', best_Z);
fprintf('M = %d\n', best_M);

path_parts = cell(1, length(best_path));
for i = 1:length(best_path), path_parts{i} = cfg.names{best_path(i)}; end
fprintf('Path: %s\n', strjoin(path_parts, ' -> '));

tt = 0;
for k = 1:(length(best_path)-1), tt = tt + dist(best_path(k), best_path(k+1)); end
fprintf('Travel: %d days | Park at sea: %d days | Work: %d days | Total: %d days\n', ...
    tt, sum(best_park_seg), sum(best_work_days), tt + sum(best_park_seg) + sum(best_work_days));
fprintf('Nodes explored: %d\n', nodes);

% Print schedule
cp_common('print_schedule', best_path, best_work_days, best_park_seg, ...
    dist, cfg.WM, cfg.WY, cfg.cons, cfg.MAX_DAYS, cfg.MAX_LOAD, cfg.all_xy, cfg.names, cfg.init);

% ===== Method 2: Direct Enumeration (cross-validation) =====
fprintf('\n--- Method 2: Direct Enumeration (cross-check) ---\n');
validate_by_enumeration(cfg, best_Z, best_M);

fprintf('\nDone.\n');
end

% ===== CP Recursive Search (Task 2: with park-at-sea) =====
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_q2(path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, cfg, nodes, depth)
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

    % Evaluate leaf
    dE = cfg.dist(lp, 2);
    if tsf + dE <= cfg.MAX_DAYS
        fp = [path, 2];
        m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1), tt = tt + cfg.dist(fp(k), fp(k+1)); end
        rem_days = cfg.MAX_DAYS - tt;
        nw = length(wa);

        if nw == 0
            % No work: enumerate park-at-sea only
            n_seg = m + 1;
            park_combs = cp_common('enumerate_park_combs', n_seg + 1, rem_days);
            for ci = 1:size(park_combs, 1)
                ps = park_combs(ci, 1:n_seg);
                [ok, Z, M, ~] = cp_common('simulate', fp, m, cfg.dist, [], [], tt, [], ps, ...
                    cfg.cons, cfg.MAX_LOAD, cfg.init);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bWD = []; bPS = ps;
                end
            end
        else
            % Work + park-at-sea combo
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
                park_rem = cfg.MAX_DAYS - tt - total_stay;
                n_seg = m + 1;
                park_combs = cp_common('enumerate_park_combs', n_seg, park_rem);
                for pci = 1:size(park_combs, 1)
                    ps = park_combs(pci, :);
                    if tt + total_stay + sum(ps) > cfg.MAX_DAYS, continue; end
                    [ok, Z, M, ~] = cp_common('simulate', fp, m, cfg.dist, wa, ww, tt, wd, ps, ...
                        cfg.cons, cfg.MAX_LOAD, cfg.init);
                    if ok && (Z > bZ || (Z == bZ && M > bM))
                        bZ = Z; bM = M; bP = fp; bWD = wd; bPS = ps;
                    end
                end
            end
        end
    end

    % Branch
    for ni = 1:5
        np = cfg.inter_idx(ni);
        if np == lp, continue; end
        d = cfg.dist(lp, np);
        if tsf + d > cfg.MAX_DAYS, continue; end
        dE2 = cfg.dist(np, 2);
        if tsf + d + dE2 > cfg.MAX_DAYS, continue; end
        np2 = [path, np];
        nt = tsf + d;
        nwa = wa; nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2);
            nww(end+1) = np - 2;
        end
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_q2(np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, cfg, nodes, depth+1);
    end
end

% ===== Direct Enumeration for Cross-Validation =====
function validate_by_enumeration(cfg, cp_Z, cp_M)
% Enumerate B->S1->E paths directly: c cells travel + p park days, then 8 cells to E.
% This validates the CP result independently.
    d_BS1 = cfg.dist(1, 6);   % B->S1 distance
    d_S1E = cfg.dist(6, 2);   % S1->E distance
    mO = cfg.cons.MO; mH = cfg.cons.MH; mF = cfg.cons.MF;
    pO = cfg.cons.PO; pH = cfg.cons.PH; pF = cfg.cons.PF;

    bestM = -inf; bestC = 0; bestP = 0;

    for c = d_BS1:(d_BS1 + 7)  % Try from min cells to a few extra
        for p = 0:30
            O = cfg.init.O - mO*c - pO*p;
            H = cfg.init.H - mH*c - pH*p;
            F = cfg.init.F - mF*c - pF*p;
            if O < 0 || H < 0 || F < 0, break; end

            load_arr = O + H + F;
            sp = cfg.MAX_LOAD - load_arr;
            On = max(0, mO*d_S1E - O);
            Hn = max(0, mH*d_S1E - H);
            Fn = max(0, mF*d_S1E - F);
            need = On + Hn + Fn;

            if need <= sp
                cost = cfg.cons.pO*On + cfg.cons.pH*Hn + cfg.cons.pF*Fn;
                M = cfg.init.M - cost;
                if M > bestM
                    bestM = M; bestC = c; bestP = p;
                end
                fprintf('  c=%d p=%d: O=%d H=%d F=%d sp=%d need=%d M=%d\n', ...
                    c, p, round(O), round(H), round(F), round(sp), round(need), round(M));
            end
        end
    end

    fprintf('\n  Enumeration best: c=%d p=%d M=%d (CP: Z=%d M=%d)\n', bestC, bestP, round(bestM), cp_Z, round(cp_M));
    if cp_Z == cfg.init.Z && round(cp_M) == round(bestM)
        fprintf('  [PASS] CP result matches direct enumeration.\n');
    else
        fprintf('  [WARN] Mismatch! CP=(Z=%d,M=%d) vs Enum=(Z=%d,M=%d)\n', cp_Z, round(cp_M), cfg.init.Z, round(bestM));
    end

    % Show optimized day-by-day (fixed: park at B first, then move, then supply)
    fprintf('\n  Optimized schedule (park BEFORE reaching S1):\n');
    fprintf('  Day 1: park at B     -> O=%d H=%d F=%d\n', ...
        cfg.init.O-pO, cfg.init.H-pH, cfg.init.F-pF);
    fprintf('  Day 2-%d: move B->S1  -> O=%d H=%d F=%d (arrive S1)\n', ...
        1+bestC, cfg.init.O-pO-mO*bestC, cfg.init.H-pH-mH*bestC, cfg.init.F-pF-mF*bestC);
    fprintf('  Day %d: SUPPLY at S1  -> O=%d H=%d F=%d (buy O=%d,H=%d,F=%d)\n', ...
        2+bestC, ...
        cfg.init.O-pO-mO*bestC+max(0,mO*d_S1E-(cfg.init.O-pO-mO*bestC)), ...
        cfg.init.H-pH-mH*bestC+max(0,mH*d_S1E-(cfg.init.H-pH-mH*bestC)), ...
        cfg.init.F-pF-mF*bestC+max(0,mF*d_S1E-(cfg.init.F-pF-mF*bestC)), ...
        max(0,mO*d_S1E-(cfg.init.O-pO-mO*bestC)), ...
        max(0,mH*d_S1E-(cfg.init.H-pH-mH*bestC)), ...
        max(0,mF*d_S1E-(cfg.init.F-pF-mF*bestC)));
end

function s = iif(cond, t, f)
    if cond, s = t; else, s = f; end
end
