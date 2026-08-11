%% ========================================================================
%% solve_task1.m - Task 1: Constraint Programming for Normal Weather
%% ========================================================================
%% 鍔熻兘: CP姹傝В姝ｅ父澶╂皵涓嬬殑鏈€浼樿埅琛屾柟妗堬紙鍚仠娉?鍐嶄綔涓氱瓥鐣ワ級
%% 鍦烘櫙: 30澶╂甯稿ぉ姘旓紝10脳10缃戞牸
%% 娑堣€? Move(O=2,H=3,F=2) | Park(O=1,H=1,F=1) | Work(O=5,H=4,F=3)
%% 鎼滅储: 鍊煎煙缂╁噺 + 绾︽潫浼犳挱 + 鍒嗘敮鍥炴函 + 涓夌鍓灊
%% 杈撳嚭: 鏈€浼橀鏋躲€侀€愭棩鑸嚎
%% 渚濊禆: 鏃犻澶栧伐鍏风
%% ========================================================================

function solve_task1()

%% ====== Parameters ======
MAX_DAYS  = 30;
MAX_LOAD  = 120;
INIT_O    = 35; INIT_H = 45; INIT_F = 30;
INIT_M    = 240; INIT_Z = 100;

all_xy    = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
inter_idx = [3 4 5 6 7];
names     = {'B', 'E', 'W1', 'W2', 'W3', 'S1', 'S2'};
WY        = [20, 15, 28];
WM        = [4, 5, 3];

% Precompute Manhattan distances (vectorized)
dist = abs(all_xy(:,1) - all_xy(:,1)') + abs(all_xy(:,2) - all_xy(:,2)');

%% ====== CP Search ======
fprintf('========================================\n');
fprintf('  CP for Task 1 (Normal Weather)\n');
fprintf('  Strategy: Park-at-workpoint enabled\n');
fprintf('========================================\n');

bZ     = -inf;
bM     = -inf;
bP     = [1, 2];
bW     = 0;
bWD    = [];
nodes  = 0;

[bZ, bM, bP, bW, bWD, nodes] = cp_search([1], 0, [], [], ...
    bZ, bM, bP, bW, bWD, dist, inter_idx, WY, WM, nodes, MAX_DAYS, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);

%% ====== Optimal Solution Summary ======
fprintf('\n================ OPTIMAL SOLUTION ================\n');
fprintf('Z = %d\n', bZ);
fprintf('M = %d\n', bM);
fprintf('Path: %s\n', strjoin(names(bP), ' -> '));

tt = sum(dist(sub2ind(size(dist), bP(1:end-1), bP(2:end))));
fprintf('Travel: %d days | Work: %d days | Total: %d days\n', tt, sum(bWD), bW);
fprintf('Nodes explored: %d\n', nodes);

print_schedule(bP, bWD, dist, WM, WY, names, MAX_DAYS, MAX_LOAD, all_xy);
fprintf('\nDone.\n');

end

%% ========================================================================
%% CP Recursive Search (Domain Reduction + Constraint Propagation)
%% ========================================================================
function [bZ, bM, bP, bW, bWD, nodes] = cp_search( ...
    path, tsf, wa, ww, bZ, bM, bP, bW, bWD, ...
    dist, inter_idx, WY, WM, nodes, MAX_DAYS, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    nodes = nodes + 1;
    lp = path(end);

    % Compute dE once; shared by upper-bound pruning and leaf evaluation
    dE = dist(lp, 2);

    % Upper-bound pruning
    if lp ~= 2
        rem  = MAX_DAYS - tsf;
        if rem < dE
            return;
        end
        ub = 100 + max_work_with_park(3, rem - dE) * 28;
        if ub <= bZ && bZ > -inf
            return;
        end
    end

    % Leaf evaluation
    if tsf + dE <= MAX_DAYS
        fp = [path, 2];
        m  = length(fp) - 2;
        tt = sum(dist(sub2ind(size(dist), fp(1:end-1), fp(2:end))));

        rem = MAX_DAYS - tt;
        nw  = length(wa);

        if nw == 0
            [ok, Z, M] = gsim(fp, m, dist, wa, ww, [], ...
                WM, WY, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
            if ok && (Z > bZ || (Z == bZ && M > bM))
                bZ = Z; bM = M; bP = fp; bW = 0; bWD = [];
            end
        else
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = max_work_with_park(WM(ww(j)), rem);
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

                if tt + total_stay > MAX_DAYS
                    continue;
                end

                [ok, Z, M] = gsim(fp, m, dist, wa, ww, wd, ...
                    WM, WY, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
                if ok && (Z > bZ || (Z == bZ && M > bM))
                    bZ = Z; bM = M; bP = fp; bW = total_stay; bWD = wd;
                end
            end
        end
    end

    % Branch to intermediate POIs
    for ni = 1:5
        np = inter_idx(ni);

        if np == lp
            continue;
        end

        d = dist(lp, np);
        if tsf + d > MAX_DAYS
            continue;
        end
        dE2 = dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS
            continue;
        end

        np2 = [path, np];
        nt  = tsf + d;
        nwa = wa;
        nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2);
            nww(end+1) = np - 2;
        end

        [bZ, bM, bP, bW, bWD, nodes] = cp_search( ...
            np2, nt, nwa, nww, bZ, bM, bP, bW, bWD, ...
            dist, inter_idx, WY, WM, nodes, MAX_DAYS, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
    end
end

%% ========================================================================
%% Build Daily Operations Plan (shared by gsim and print_schedule)
%% ========================================================================
function [cO, cH, cF, zG, isSup, T] = build_ops(pid, m, dist_all, wa, ww, wdays, WM, WY)
    total_extra = 0;
    for j = 1:length(wdays)
        if wdays(j) > 0
            total_extra = total_extra + max(0, ceil(wdays(j) / WM(ww(j))) - 1);
        end
    end

    tt = sum(dist_all(sub2ind(size(dist_all), pid(1:end-1), pid(2:end))));
    T   = tt + sum(wdays) + total_extra;
    cO  = zeros(1, T);
    cH  = zeros(1, T);
    cF  = zeros(1, T);
    zG  = zeros(1, T);
    isSup = false(1, T);

    day = 0;

    for k = 1:(m + 1)
        d = dist_all(pid(k), pid(k+1));

        for dd = 1:d
            day = day + 1;
            cO(day) = 2;
            cH(day) = 3;
            cF(day) = 2;
            if dd == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7
                    isSup(day) = true;
                end
            end
        end

        if ~isempty(wa)
            wk = find(wa == k + 1, 1);
            if ~isempty(wk) && wdays(wk) > 0
                mc  = WM(ww(wk));
                rem = wdays(wk);
                yld = WY(ww(wk));

                while rem > 0
                    chunk = min(rem, mc);
                    for w = 1:chunk
                        day = day + 1;
                        cO(day) = 5;
                        cH(day) = 4;
                        cF(day) = 3;
                        zG(day) = yld;
                    end
                    rem = rem - chunk;

                    % Park-reset counter
                    if rem > 0
                        day = day + 1;
                        cO(day) = 1;
                        cH(day) = 1;
                        cF(day) = 1;
                        zG(day) = 0;
                    end
                end
            end
        end
    end
end

%% ========================================================================
%% Greedy Simulation: Evaluate a complete skeleton with resource tracking
%% ========================================================================
function [feasible, Zf, Mf] = gsim(pid, m, dist_all, wa, ww, ...
    wdays, WM, WY, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    [cO, cH, cF, zG, isSup, T] = build_ops(pid, m, dist_all, wa, ww, wdays, WM, WY);

    % Forward simulation
    O  = INIT_O; H = INIT_H; F = INIT_F;
    M  = INIT_M; Zf = INIT_Z;

    for t = 1:T
        O  = O  - cO(t);
        H  = H  - cH(t);
        F  = F  - cF(t);
        Zf = Zf + zG(t);

        if O < 0 || H < 0 || F < 0
            feasible = false;
            Mf = 0;
            return;
        end
        if O + H + F > MAX_LOAD + 1e-9
            feasible = false;
            Mf = 0;
            return;
        end

        if isSup(t)
            ns = T + 1;
            for tt2 = (t + 1):T
                if isSup(tt2)
                    ns = tt2;
                    break;
                end
            end

            nO = 0; nH = 0; nF = 0;
            for tt2 = (t + 1):ns
                if tt2 > T
                    break;
                end
                nO = nO + cO(tt2);
                nH = nH + cH(tt2);
                nF = nF + cF(tt2);
            end

            sp  = MAX_LOAD - (O + H + F);
            buyO = max(0, nO - O);
            buyH = max(0, nH - H);
            buyF = max(0, nF - F);

            if buyO + buyH + buyF > sp
                feasible = false;
                Mf = 0;
                return;
            end
            if ns > T && (O + buyO < nO || H + buyH < nH || F + buyF < nF)
                feasible = false;
                Mf = 0;
                return;
            end

            cost = buyO * 2 + buyH * 1 + buyF * 2;
            if cost > M
                feasible = false;
                Mf = 0;
                return;
            end

            O = O + buyO;
            H = H + buyH;
            F = F + buyF;
            M = M - cost;
        end
    end

    feasible = true;
    Mf = M;
end

%% ========================================================================
%% Upper Bound Helper: Max work days achievable with park-reset
%% ========================================================================
function max_w = max_work_with_park(mc, remaining)
    best = 0;
    for k = 1:(remaining + 1)
        stay = k * mc + (k - 1);
        if stay > remaining
            break;
        end
        best  = k * mc;
        slack = remaining - stay;
        if slack >= 1
            best = max(best, k * mc + min(mc, slack - 1));
        end
    end
    max_w = max(best, min(mc, remaining));
end

%% ========================================================================
%% Print Day-by-Day Schedule
%% ========================================================================
function print_schedule(bP, bWD, dist, WM, WY, names, MAX_DAYS, MAX_LOAD, all_xy)

    wa = [];
    ww = [];
    for i = 2:length(bP)
        if bP(i) >= 3 && bP(i) <= 5
            wa(end+1) = i;
            ww(end+1) = bP(i) - 2;
        end
    end

    m = length(bP) - 2;
    [cO, cH, cF, zG, isSup, T] = build_ops(bP, m, dist, wa, ww, bWD, WM, WY);

    fprintf('\n================ DAY-BY-DAY SCHEDULE ================\n');
    fprintf('Day | Pos (x,y)  | Action      |  O   H   F  Load |   Z     M\n');
    fprintf('----|-------------|-------------|------------------|------------\n');

    O = 35; H = 45; F = 30;
    M = 240; Z = 100;
    day2 = 0;

    for k = 1:(m + 1)
        fr = bP(k);
        to = bP(k+1);
        d  = dist(fr, to);

        for dd = 1:d
            day2 = day2 + 1;
            t_frac = dd / d;
            x = round(all_xy(fr, 1) + (all_xy(to, 1) - all_xy(fr, 1)) * t_frac);
            y = round(all_xy(fr, 2) + (all_xy(to, 2) - all_xy(fr, 2)) * t_frac);
            O = O - 2;
            H = H - 3;
            F = F - 2;

            if isSup(day2)
                ns = T + 1;
                for tt2 = (day2 + 1):T
                    if isSup(tt2)
                        ns = tt2;
                        break;
                    end
                end

                nO = 0; nH = 0; nF = 0;
                for tt2 = (day2 + 1):ns
                    if tt2 > T
                        break;
                    end
                    nO = nO + cO(tt2);
                    nH = nH + cH(tt2);
                    nF = nF + cF(tt2);
                end

                buyO = max(0, nO - O);
                buyH = max(0, nH - H);
                buyF = max(0, nF - F);
                cost = buyO * 2 + buyH * 1 + buyF * 2;
                M = M - cost;
                O = O + buyO;
                H = H + buyH;
                F = F + buyF;
                fprintf('%3d | (%2d,%2d)     | SUPPLY      | %3d %3d %3d %4d | %4d %5d  (+O=%d H=%d F=%d)\n', ...
                    day2, x, y, O, H, F, O+H+F, Z, M, buyO, buyH, buyF);
            else
                fprintf('%3d | (%2d,%2d)     | move        | %3d %3d %3d %4d | %4d %5d\n', ...
                    day2, x, y, O, H, F, O+H+F, Z, M);
            end
        end

        wk = find(wa == k + 1, 1);
        if ~isempty(wk) && bWD(wk) > 0
            mc  = WM(ww(wk));
            rem = bWD(wk);
            yld = WY(ww(wk));
            while rem > 0
                chunk = min(rem, mc);
                for w = 1:chunk
                    day2 = day2 + 1;
                    O = O - 5; H = H - 4; F = F - 3;
                    Z = Z + yld;
                    fprintf('%3d | (%2d,%2d)     | work(%s)    | %3d %3d %3d %4d | %4d %5d\n', ...
                        day2, all_xy(to,1), all_xy(to,2), names{to}, O, H, F, O+H+F, Z, M);
                end
                rem = rem - chunk;
                if rem > 0
                    day2 = day2 + 1;
                    O = O - 1; H = H - 1; F = F - 1;
                    fprintf('%3d | (%2d,%2d)     | park(reset) | %3d %3d %3d %4d | %4d %5d\n', ...
                        day2, all_xy(to,1), all_xy(to,2), O, H, F, O+H+F, Z, M);
                end
            end
        end
    end

    fprintf('----|-------------|-------------|------------------|------------\n');
    fprintf('  Final at E: Z=%d M=%d Day=%d\n', Z, M, day2);
end