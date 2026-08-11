%% ========================================================================
%% solve_task2_cp.m - Task 2: CP Solver for Thunderstorm Extreme
%% ========================================================================
%% 功能: 约束规划求解全雷暴极端天气最优航行方案
%% 核心: 值域缩减 + 约束传播 + 分支回溯 + 三种剪枝
%% 场景: 30天全雷暴, 10×10网格
%% 输出: 最优骨架与逐日航线
%% ========================================================================

function solve_task2_cp()

%% ====== Parameters ======
MAX_DAYS = 30;  MAX_LOAD = 120;
INIT_O = 35;    INIT_H = 45;   INIT_F = 30;
INIT_M = 240;   INIT_Z = 100;

all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
n_pts  = size(all_xy, 1);
names  = {"B", "E", "W1", "W2", "W3", "S1", "S2"};
inter_idx = [3 4 5 6 7];
WY = [20, 15, 28];
WM = [4, 5, 3];

cons = struct( ...
    "MO", 8, "MH", 4, "MF", 3, ...
    "PO", 3, "PH", 3, "PF", 2, ...
    "WO", 8, "WH", 6, "WF", 6, ...
    "pO", 2, "pH", 1, "pF", 2);

dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i, j) = abs(all_xy(i,1) - all_xy(j,1)) + abs(all_xy(i,2) - all_xy(j,2));
    end
end

%% ====== Connectivity Analysis ======
fprintf("========================================\n");
fprintf("  CP for Task 2: 30-Day Thunderstorm\n");
fprintf("========================================\n");
fprintf("Move cost per cell: O=%d H=%d F=%d\n", cons.MO, cons.MH, cons.MF);
fprintf("Park cost per day:  O=%d H=%d F=%d\n", cons.PO, cons.PH, cons.PF);
fprintf("Work cost per day:  O=%d H=%d F=%d\n", cons.WO, cons.WH, cons.WF);
fprintf("Initial: O=%d H=%d F=%d M=%d Z=%d Load<=%d\n\n", INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, MAX_LOAD);

fprintf("--- Connectivity Analysis ---\n");
fprintf("B->S1: %d cells, O_need=%d <= %d (REACHABLE)\n", dist(1,6), dist(1,6)*cons.MO, INIT_O);
fprintf("B->S2: %d cells, O_need=%d > %d (UNREACHABLE)\n", dist(1,7), dist(1,7)*cons.MO, INIT_O);
for wi = 1:3
    d_bw  = dist(1, wi+2);
    needO = d_bw * cons.MO;
    tag   = iif(needO <= INIT_O, "REACHABLE", "UNREACHABLE");
    fprintf("B->W%d: %d cells, O_need=%d (%s)\n", wi, d_bw, needO, tag);
end
fprintf("\n");

%% ====== CP Search ======
best_Z    = -inf;  best_M    = -inf;
best_path = [1, 2]; best_wd   = [];
best_ps   = zeros(1, 1);
nodes = 0;

[best_Z, best_M, best_path, best_wd, best_ps, nodes] = ...
    cp_search_q2([1], 0, [], [], best_Z, best_M, best_path, best_wd, best_ps, ...
        dist, inter_idx, WY, WM, cons, nodes, 0, MAX_DAYS, MAX_LOAD, ...
        INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);

%% ====== Result Summary ======
fprintf("\n================ OPTIMAL SOLUTION (CP) ================\n");
fprintf("Z = %d\n", best_Z);
fprintf("M = %d\n", best_M);

pstr = "";
for i = 1:length(best_path), pstr = pstr + " " + names{best_path(i)}; end
fprintf("Path: %s\n", pstr);

tt = 0;
for k = 1:(length(best_path) - 1), tt = tt + dist(best_path(k), best_path(k+1)); end
total_park = sum(best_ps);
fprintf("Travel: %d days | Park at sea: %d days | Work: %d days | Total: %d days\n", ...
    tt, total_park, sum(best_wd), tt + total_park + sum(best_wd));
fprintf("Nodes explored: %d\n", nodes);

print_schedule_q2(best_path, best_wd, best_ps, dist, WM, WY, cons, ...
    MAX_DAYS, MAX_LOAD, all_xy, names, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);

fprintf("\nDone.\n");
end

%% ========================================================================
%% CP Recursive Search with Upper-Bound Pruning
%% ========================================================================
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_q2( ...
    path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, ...
    dist, inter_idx, WY, WM, cons, nodes, depth, MAX_DAYS, MAX_LOAD, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    nodes = nodes + 1;
    lp = path(end);

    % Upper-bound pruning
    if lp ~= 2
        dE  = dist(lp, 2);
        rem = MAX_DAYS - tsf;
        if rem < dE, return; end
        max_w = max_work_with_park2(3, rem - dE);
        ub    = INIT_Z + real(max_w) * 28;
        if ub <= bZ && bZ > -inf, return; end
    end

    % Leaf evaluation
    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS
        fp = [path, 2];
        m  = length(fp) - 2;
        tt = 0;
        for k = 1:(m + 1), tt = tt + dist(fp(k), fp(k+1)); end

        rem_days = MAX_DAYS - tt;
        nw = length(wa);

        if nw == 0
            [bZ, bM, bP, bWD, bPS] = eval_leaf_no_work( ...
                fp, m, tt, rem_days, dist, bZ, bM, bP, bWD, bPS, ...
                cons, MAX_DAYS, MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
        else
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = max_work_with_park2(WM(ww(j)), rem_days);
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
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j) / WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > MAX_DAYS, continue; end

                park_rem = MAX_DAYS - tt - total_stay;
                [bZ, bM, bP, bWD, bPS] = eval_leaf_with_work_park( ...
                    fp, m, tt, wd, park_rem, dist, wa, ww, bZ, bM, bP, bWD, bPS, ...
                    cons, MAX_DAYS, MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
            end
        end
    end

    % Branch to intermediate POIs
    for ni = 1:5
        np = inter_idx(ni);
        if np == lp, continue; end

        d = dist(lp, np);
        if tsf + d > MAX_DAYS, continue; end

        dE2 = dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS, continue; end

        np2 = [path, np];
        nt  = tsf + d;
        nwa = wa;
        nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2);
            nww(end+1) = np - 2;
        end

        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_q2( ...
            np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, ...
            dist, inter_idx, WY, WM, cons, nodes, depth + 1, MAX_DAYS, MAX_LOAD, ...
            INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
    end
end

%% ========================================================================
%% Leaf: No Work Points — Enumerate Park Only
%% ========================================================================
function [bZ, bM, bP, bWD, bPS] = eval_leaf_no_work( ...
    fp, m, tt, rem_days, dist, bZ, bM, bP, bWD, bPS,  ...
    cons, MAX_DAYS, MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    n_seg      = m + 1;
    park_combs = enumerate_park_combinations2(n_seg, rem_days);

    for ci = 1:size(park_combs, 1)
        ps = park_combs(ci, 1:n_seg);
        [ok, Z, M] = gsim_q2(fp, m, dist, [], [], tt, [], ps, cons, ...
            MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
        if ok && (Z > bZ || (Z == bZ && M > bM))
            bZ = Z; bM = M; bP = fp; bWD = []; bPS = ps;
        end
    end
end

%% ========================================================================
%% Leaf: With Work Points — Enumerate Work Days + Park
%% ========================================================================
function [bZ, bM, bP, bWD, bPS] = eval_leaf_with_work_park( ...
    fp, m, tt, wd, park_rem, dist, wa, ww, bZ, bM, bP, bWD, bPS,  ...
    cons, MAX_DAYS, MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    n_seg      = m + 1;
    park_combs = enumerate_park_combinations2(n_seg, park_rem);

    for ci = 1:size(park_combs, 1)
        ps = park_combs(ci, :);
        total_stay = sum(ps);
        for j = 1:length(wd)
            if wd(j) > 0
                total_stay = total_stay + wd(j) + max(0, ceil(wd(j) / WM(ww(j))) - 1);
            end
        end
        if tt + total_stay > MAX_DAYS, continue; end

        [ok, Z, M] = gsim_q2(fp, m, dist, wa, ww, tt, wd, ps, cons, ...
            MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
        if ok && (Z > bZ || (Z == bZ && M > bM))
            bZ = Z; bM = M; bP = fp; bWD = wd; bPS = ps;
        end
    end
end

%% ========================================================================
%% Enumerate Parking Combinations (Recursive)
%% ========================================================================
function combs = enumerate_park_combinations2(n_seg, max_total)
    total   = nchoosek(max_total + n_seg, n_seg);
    combs   = zeros(total, n_seg);
    idx     = 0;
    current = zeros(1, n_seg);
    enumerate_rec(1, max_total);

    function enumerate_rec(pos, remain)
        if pos == n_seg
            current(pos) = remain;
            idx = idx + 1;
            combs(idx, :) = current;
            return;
        end
        for p = 0:remain
            current(pos) = p;
            enumerate_rec(pos + 1, remain - p);
        end
    end
end

%% ========================================================================
%% Forward Resource Simulation
%% ========================================================================
function [feasible, Zf, Mf, sched] = gsim_q2(pid, m, dist_all, wa, ww, tt, ...
    wdays, park_seg, cons, MAX_LOAD, WM, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    [cO, cH, cF, zG, isSup, T_actual] = build_timeline_q2(pid, m, tt, park_seg, wdays, wa, ww, dist_all, cons, WM, WY);

    % Forward simulation
    O = INIT_O; H = INIT_H; F = INIT_F;
    M  = INIT_M; Zf = INIT_Z;

    for t = 1:T_actual
        O  = O  - cO(t);  H  = H  - cH(t);  F  = F  - cF(t);
        Zf = Zf + zG(t);

        if O < 0 || H < 0 || F < 0 || O + H + F > MAX_LOAD + 1e-9
            feasible = false; Mf = 0; return;
        end

        if isSup(t)
            ns = T_actual + 1;
            for tt2 = (t + 1):T_actual
                if isSup(tt2), ns = tt2; break; end
            end

            nO = 0; nH = 0; nF = 0;
            for tt2 = (t + 1):ns
                if tt2 > T_actual, break; end
                nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
            end

            sp   = MAX_LOAD - (O + H + F);
            buyO = max(0, nO - O);
            buyH = max(0, nH - H);
            buyF = max(0, nF - F);

            if buyO + buyH + buyF > sp || ...
               (ns > T_actual && (O + buyO < nO || H + buyH < nH || F + buyF < nF))
                feasible = false; Mf = 0; return;
            end

            cost = buyO * cons.pO + buyH * cons.pH + buyF * cons.pF;
            if cost > M, feasible = false; Mf = 0; return; end

            O = O + buyO; H = H + buyH; F = F + buyF;
            M = M - cost;
        end
    end

    feasible = true; Mf = M;
end

%% ========================================================================
%% Max Work Days with Park-Reset Strategy
%% ========================================================================
function max_w = max_work_with_park2(mc, remaining)
    best = 0;
    for k = 1:(remaining + 1)
        stay = k * mc + (k - 1);
        if stay > remaining, break; end
        best  = k * mc;
        slack = remaining - stay;
        if slack >= 1, best = max(best, k * mc + min(mc, slack - 1)); end
    end
    max_w = max(best, min(mc, remaining));
end

%% ========================================================================
%% Ternary Operator Helper
%% ========================================================================
function s = iif(cond, t, f)
    if cond, s = t; else s = f; end
end

%% ========================================================================
%% Print Day-by-Day Schedule
%% ========================================================================
function print_schedule_q2(bP, bWD, bPS, dist, WM, WY, cons, ...
    MAX_DAYS, MAX_LOAD, all_xy, names, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    m = length(bP) - 2;
    if m < 0, fprintf("  No feasible path found.\n"); return; end
    if isempty(bPS), bPS = zeros(1, m + 1); end

    wa = []; ww = [];
    for i = 2:length(bP)
        if bP(i) >= 3 && bP(i) <= 5
            wa(end+1) = i; ww(end+1) = bP(i) - 2;
        end
    end

    tt = 0;
    for k = 1:(m + 1), tt = tt + dist(bP(k), bP(k+1)); end

    [cO, cH, cF, zG, isSup, T_actual] = build_timeline_q2(bP, m, tt, bPS, bWD, wa, ww, dist, cons, WM, WY);

    fprintf("\n================ DAY-BY-DAY SCHEDULE ================\n");
    fprintf("Day | Pos (x,y)  | Action         |  O   H   F  Load |   Z     M\n");
    fprintf("----|-------------|----------------|------------------|------------\n");

    O = INIT_O; H = INIT_H; F = INIT_F;
    M = INIT_M; Z = INIT_Z;
    day2 = 0;

    for k = 1:(m + 1)
        fr = bP(k); to = bP(k+1); d = dist(fr, to);

        for pd = 1:bPS(k)
            day2 = day2 + 1;
            O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
            fprintf("%3d | (%2d,%2d)     | park(at sea)  | %3d %3d %3d %4d | %4d %5d\n", ...
                day2, all_xy(fr,1), all_xy(fr,2), O, H, F, O+H+F, Z, M);
        end

        for dd = 1:d
            day2 = day2 + 1;
            t_frac = dd / d;
            x = round(all_xy(fr,1) + (all_xy(to,1) - all_xy(fr,1)) * t_frac);
            y = round(all_xy(fr,2) + (all_xy(to,2) - all_xy(fr,2)) * t_frac);
            O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);

            if isSup(day2)
                ns = T_actual + 1;
                for tt2 = (day2 + 1):T_actual
                    if isSup(tt2), ns = tt2; break; end
                end
                nO = 0; nH = 0; nF = 0;
                for tt2 = (day2 + 1):ns
                    if tt2 > T_actual, break; end
                    nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
                end
                buyO = max(0, nO - O); buyH = max(0, nH - H); buyF = max(0, nF - F);
                cost = buyO * cons.pO + buyH * cons.pH + buyF * cons.pF;
                M = M - cost;
                O = O + buyO; H = H + buyH; F = F + buyF;
                fprintf("%3d | (%2d,%2d)     | SUPPLY(%s)    | %3d %3d %3d %4d | %4d %5d  (+O=%d H=%d F=%d)\n", ...
                    day2, x, y, names{to}, O, H, F, O+H+F, Z, M, buyO, buyH, buyF);
            else
                fprintf("%3d | (%2d,%2d)     | move          | %3d %3d %3d %4d | %4d %5d\n", ...
                    day2, x, y, O, H, F, O+H+F, Z, M);
            end
        end

        wk = find(wa == k + 1, 1);
        if ~isempty(wk) && bWD(wk) > 0
            wm_val  = WM(ww(wk));
            yld     = WY(ww(wk));
            rem_val = bWD(wk);

            while rem_val > 0
                chunk = min(rem_val, wm_val);
                for w = 1:chunk
                    day2 = day2 + 1;
                    O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
                    Z = Z + zG(day2);
                    fprintf("%3d | (%2d,%2d)     | work(%s)      | %3d %3d %3d %4d | %4d %5d\n", ...
                        day2, all_xy(to,1), all_xy(to,2), names{to}, O, H, F, O+H+F, Z, M);
                end
                rem_val = rem_val - chunk;
                if rem_val > 0
                    day2 = day2 + 1;
                    O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
                    fprintf("%3d | (%2d,%2d)     | park(reset)   | %3d %3d %3d %4d | %4d %5d\n", ...
                        day2, all_xy(to,1), all_xy(to,2), O, H, F, O+H+F, Z, M);
                end
            end
        end
    end

    fprintf("----|-------------|----------------|------------------|------------\n");
    fprintf("  Final at E: Z=%d M=%d Day=%d\n", Z, M, day2);
end
