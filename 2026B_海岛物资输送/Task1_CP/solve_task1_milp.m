%% ========================================================================
%% solve_task1_milp.m - Task 1: MILP Solver with Stop-Work Strategy
%% ========================================================================
%% 闁告梻鍠曢崗? 濡ょ姰鍔嶉悘锕傚几濮橆偄顩?+ 閻犳劦浜滈埞鍡橈紣閸曨厾鎽?+ 濞戞挶鍊濆Ο浣糕枔缁€姹璍P婵懓鍊借婵繐绲介悥鑸靛緞閳哄倻姣滈柡鍫氬亾濞村吋蓱閺岀喎顩?%% 缂佹稒鐗滈弳? 婵絽绻戦鍏兼媴濠娾偓缁楃喓鎷嬮崸妤侊紪闁衡偓椤栨稑鐦?w1 + stop(1濠? + w2闁挎稑鐬奸悰濠囨儘绾惧绠剧紓渚囧幒缂嶆梹绋夊顐ょ憪闂?%% 闁革妇鍎ゅ▍? 30濠㈠灈鏅滈婊呮暜缁嬭￥浜慨? 10x10缂傚啯鍨堕悧?%% 濞村吋锚鐎? 閻庢稒顨呴崥鈧幖?max Z -> max M
%% 濞撴碍绻嗙粋? Optimization Toolbox (intlinprog)
%% 閺夊牊鎸搁崵? 闁哄牃鍋撳ù鍏忌戦弻鐔奉浖閸粎鐟㈤梺顐ｅ姈濡晠鎳滈鍡楁疇
%% ========================================================================

function solve_task1_milp()

%% ====== Parameters ======
CM = [2, 3, 2]; CW = [5, 4, 3]; CS = [1, 1, 1];
WY = [20, 15, 28]; WM = [4, 5, 3];
MAX_WY = max(WY);

MAX_DAYS   = 30;
LOAD_LIMIT = 120;
INIT_O  = 35; INIT_H = 45; INIT_F = 30;
INIT_M  = 240; INIT_Z = 100;

all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
names  = {'B', 'E', 'W1', 'W2', 'W3', 'S1', 'S2'};

n_pts = 7;
% Vectorized Manhattan distance
dist = abs(all_xy(:,1) - all_xy(:,1)') + abs(all_xy(:,2) - all_xy(:,2)');
inter_idx = [3 4 5 6 7];

if ~exist("intlinprog", "file")
    error("Requires Optimization Toolbox.");
end

%% ====== Skeleton Enumeration ======
min_B_inter = min(dist(1, inter_idx));
min_inter_E = min(dist(inter_idx, 2));
min_ii = min(min(dist(inter_idx, inter_idx) + 100 * eye(5)));
max_seq = floor((MAX_DAYS - min_B_inter - min_inter_E) / min_ii) + 1;
total_skeletons = sum(arrayfun(@(k) 5^k, 0:max_seq));

fprintf("================ TASK 1: MILP Stop-Work Solver ================\n");
fprintf("max_seq=%d, skeletons=%d\n", max_seq, total_skeletons);
fprintf("Stop-Work Strategy: ENABLED (w1 + stop + w2 per visit)\n\n");

%% ====== Enumerate All Skeletons ======
best_Z   = -inf; best_M   = -inf;
best_path = [];   best_w1  = [];
best_b   = [];    best_w2  = [];
best_buy = [];
count = 0; possible_milp = 0; feasible_count = 0;
milp_s1 = 0; milp_s2 = 0; greedy_only = 0;
skipped_ub = 0; skipped_pre = 0; skipped_s1 = 0;
overall_tic = tic;

for seq_len = 0:max_seq
    n_seqs = 5^seq_len;
    for si = 1:n_seqs
        count = count + 1;
        if mod(count, 2000) == 0 || count == 1
            e = toc(overall_tic);
            fprintf("  Prg:%6d/%d(%5.1f%%)|%5.1fs|Feas:%d|S1:%d S2:%d|Best Z=%d M=%d\n", ...
                count, total_skeletons, 100*count/total_skeletons, e, ...
                feasible_count, milp_s1, milp_s2, best_Z, best_M);
        end

        seq = zeros(1, seq_len);
        tmp = si - 1;
        for j = seq_len:-1:1
            seq(j) = mod(tmp, 5) + 1;
            tmp = floor(tmp / 5);
        end
        pid = [1, inter_idx(seq), 2];

        dup = false;
        for k = 2:length(pid)
            if pid(k) == pid(k-1), dup = true; break; end
        end
        if dup, continue; end

        m = length(pid) - 2;
        % Vectorized travel distance
        travel = dist(sub2ind(size(dist), pid(1:end-1), pid(2:end)));
        total_travel = sum(travel);
        if total_travel > MAX_DAYS, continue; end

        work_idx   = zeros(1, m+1);
        work_which = [];
        supp_idx   = zeros(1, m+1);
        n_work  = 0;
        n_supply = 0;
        for k = 1:(m + 1)
            pt = pid(k+1);
            if pt >= 3 && pt <= 5
                n_work = n_work + 1;
                work_idx(k) = n_work;
                work_which(n_work) = pt - 2;
            end
            if pt == 6 || pt == 7
                n_supply = n_supply + 1;
                supp_idx(k) = n_supply;
            end
        end

        remain = MAX_DAYS - total_travel;
        if n_work > 0
            z_cap = 0;
            for j = 1:n_work
                z_cap = z_cap + 2 * WM(work_which(j)) * WY(work_which(j));
            end
            Z_upper = INIT_Z + min(remain * MAX_WY, z_cap);
        else
            Z_upper = INIT_Z;
        end
        if Z_upper <= best_Z
            skipped_ub = skipped_ub + 1;
            continue;
        end
        possible_milp = possible_milp + 1;

        if n_work == 0
            [gfeas, gZ, gM, gBuy] = greedy_quick_sw(m, travel, work_idx, work_which, ...
                supp_idx, n_supply, total_travel, [], [], [], ...
                WY, CM, CW, CS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT);
            greedy_only = greedy_only + 1;
            if gfeas
                feasible_count = feasible_count + 1;
                if gZ > best_Z || (gZ == best_Z && gM > best_M)
                    best_Z = gZ; best_M = gM; best_path = pid;
                    best_w1 = []; best_b = []; best_w2 = []; best_buy = gBuy;
                end
            end
            continue;
        end

        [gfeas, ~, ~] = greedy_quick_sw(m, travel, work_idx, work_which, supp_idx, ...
            n_supply, total_travel, zeros(1, n_work), zeros(1, n_work), ...
            zeros(1, n_work), WY, CM, CW, CS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT);
        if ~gfeas
            skipped_pre = skipped_pre + 1;
            continue;
        end

        [w1_g, b_g, w2_g] = greedy_work_assign_sw(work_which, WM, WY, remain);
        [gfeas_max, gZ_max, gM_max, gBuy_max] = greedy_quick_sw(m, travel, work_idx, ...
            work_which, supp_idx, n_supply, total_travel, w1_g, b_g, w2_g, ...
            WY, CM, CW, CS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT);

        if gfeas_max
            feasible_count = feasible_count + 1;
            if gZ_max > best_Z || (gZ_max == best_Z && gM_max > best_M)
                best_Z = gZ_max; best_M = gM_max; best_path = pid;
                best_w1 = w1_g; best_b = b_g; best_w2 = w2_g; best_buy = gBuy_max;
            end
        end

        if gfeas_max && gZ_max >= Z_upper
            skipped_s1 = skipped_s1 + 1; milp_s2 = milp_s2 + 1;
            [feas2, ~, M2, w1_o2, b_o2, w2_o2, buy_o2] = milp_solve_sw(m, travel, work_idx, ...
                work_which, supp_idx, n_work, n_supply, total_travel, WY, WM, CM, CW, CS, ...
                LOAD_LIMIT, MAX_DAYS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, Z_upper);
            if feas2
                feasible_count = feasible_count + 1;
                if Z_upper > best_Z || (Z_upper == best_Z && M2 > best_M)
                    best_Z = Z_upper; best_M = M2; best_path = pid;
                    best_w1 = w1_o2; best_b = b_o2; best_w2 = w2_o2; best_buy = buy_o2;
                end
            end
        else
            milp_s1 = milp_s1 + 1; milp_s2 = milp_s2 + 1;
            [feas, Z, M, w1_o, b_o, w2_o, buy_o] = milp_solve_sw(m, travel, work_idx, ...
                work_which, supp_idx, n_work, n_supply, total_travel, WY, WM, CM, CW, CS, ...
                LOAD_LIMIT, MAX_DAYS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, []);
            if feas
                feasible_count = feasible_count + 1;
                if Z > best_Z || (Z == best_Z && M > best_M)
                    best_Z = Z; best_M = M; best_path = pid;
                    best_w1 = w1_o; best_b = b_o; best_w2 = w2_o; best_buy = buy_o;
                end
            end
        end
    end
end

elapsed_total = toc(overall_tic);

%% ====== Output Results ======
if best_Z <= 0
    fprintf("\nNO FEASIBLE SOLUTION FOUND.\n");
    return;
end

fprintf("\n================ OPTIMAL RESULT (MILP Stop-Work) ================\n");
fprintf("Total time:       %.2f s\n", elapsed_total);
fprintf("Skeletons:        %d scanned\n", count);
fprintf("  Greedy-only:    %d\n", greedy_only);
fprintf("  UB-skipped:     %d\n", skipped_ub);
fprintf("  Prefilter-skip: %d\n", skipped_pre);
fprintf("  Stage-1 calls:  %d\n", milp_s1);
fprintf("  Stage-2 calls:  %d\n", milp_s2);
fprintf("  Stage-1 saved:  %d\n", skipped_s1);
fprintf("Optimal Z:        %d\n", best_Z);
fprintf("Optimal M:        %d\n", best_M);
fprintf("Path:             ");
for i = 1:length(best_path)
    fprintf("%s", names{best_path(i)});
    if i < length(best_path), fprintf(" -> "); end
end
fprintf("\n");

if ~isempty(best_w1)
    fprintf("Work blocks:      ");
    for j = 1:length(best_w1)
        if isempty(best_b) || best_b(j) == 0
            fprintf("%d ", best_w1(j));
        else
            fprintf("%d|stop|%d ", best_w1(j), best_w2(j));
        end
    end
    fprintf("\n");
end

if ~isempty(best_buy) && size(best_buy, 1) > 0
    fprintf("Supply purchases (O,H,F):\n");
    m_bb = length(best_path) - 2;
    sidx = 0;
    for k = 1:(m_bb + 1)
        pt = best_path(k+1);
        if pt == 6 || pt == 7
            sidx = sidx + 1;
            fprintf("  %s: (%d, %d, %d)\n", names{pt}, best_buy(sidx, 1), best_buy(sidx, 2), best_buy(sidx, 3));
        end
    end
end
fprintf("\n");

print_daily_schedule_sw(best_path, best_w1, best_b, best_w2, best_buy, ...
    all_xy, dist, names, CM, CW, CS, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT);

end

%% ========================================================================
%% Greedy Work Assignment
%% ========================================================================
function [w1, b, w2] = greedy_work_assign_sw(work_which, WM, WY, remain)
    n_work = length(work_which);
    w1 = zeros(1, n_work); b = zeros(1, n_work); w2 = zeros(1, n_work);
    [~, order] = sort(WY(work_which), 'descend');
    for idx = 1:n_work
        j = order(idx);
        w1(j) = min(WM(work_which(j)), remain);
        remain = remain - w1(j);
        if remain <= 0, break; end
    end
    for idx = 1:n_work
        j = order(idx);
        if remain > 0 && w1(j) == WM(work_which(j))
            b(j) = 1; remain = remain - 1;
            w2(j) = min(WM(work_which(j)), remain);
            remain = remain - w2(j);
        end
        if remain <= 0, break; end
    end
end

%% ========================================================================
%% Greedy Resource Simulation (n_work parameter removed - unused)
%% ========================================================================
function [feasible, Z_final, M_final, buy_all] = greedy_quick_sw( ...
    m, travel, work_idx, work_which, supp_idx, n_supply, total_travel, ...
    w1, b, w2, WY, CM, CW, CS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT)
    total_stop = sum(b);
    T = total_travel + sum(w1) + total_stop + sum(w2);
    cO = zeros(1, T); cH = zeros(1, T); cF = zeros(1, T);
    zG = zeros(1, T); isSup = false(1, T);
    sup_day_to_k = zeros(1, T);
    day = 0; sidx = 0;
    for k = 1:(m + 1)
        d = travel(k);
        for dd = 1:d
            day = day + 1;
            cO(day) = CM(1); cH(day) = CM(2); cF(day) = CM(3);
            if dd == d && supp_idx(k) > 0
                isSup(day) = true; sidx = sidx + 1;
                sup_day_to_k(day) = sidx;
            end
        end
        widx = work_idx(k);
        if widx > 0
            wh = work_which(widx);
            if w1(widx) > 0
                for ww = 1:w1(widx)
                    day = day + 1;
                    cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
                    zG(day) = WY(wh);
                end
            end
            if b(widx) > 0
                day = day + 1;
                cO(day) = CS(1); cH(day) = CS(2); cF(day) = CS(3);
            end
            if w2(widx) > 0
                for ww = 1:w2(widx)
                    day = day + 1;
                    cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
                    zG(day) = WY(wh);
                end
            end
        end
    end
    buy_all = zeros(n_supply, 3);
    O = INIT_O; H = INIT_H; F = INIT_F; M = INIT_M;
    for t = 1:T
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        if O < 0 || H < 0 || F < 0
            feasible = false; Z_final = 0; M_final = 0; return;
        end
        if isSup(t)
            nextSup = T + 1;
            for tt = t + 1:T, if isSup(tt), nextSup = tt; break; end, end
            needO = 0; needH = 0; needF = 0;
            for tt = t + 1:nextSup
                if tt > T, break; end
                needO = needO + cO(tt); needH = needH + cH(tt); needF = needF + cF(tt);
            end
            buyO = max(0, needO - O); buyH = max(0, needH - H); buyF = max(0, needF - F);
            if buyO + buyH + buyF > LOAD_LIMIT - (O + H + F)
                feasible = false; Z_final = 0; M_final = 0; return;
            end
            cost = buyO * 2 + buyH * 1 + buyF * 2;
            if cost > M, feasible = false; Z_final = 0; M_final = 0; return; end
            O = O + buyO; H = H + buyH; F = F + buyF; M = M - cost;
            sk = sup_day_to_k(t);
            if sk > 0 && sk <= n_supply
                buy_all(sk, 1) = buyO; buy_all(sk, 2) = buyH; buy_all(sk, 3) = buyF;
            end
        end
        if M < 0 || O + H + F > LOAD_LIMIT
            feasible = false; Z_final = 0; M_final = 0; return;
        end
    end
    Z_final = INIT_Z + sum(zG); M_final = M; feasible = true;
end

%% ========================================================================
%% MILP Two-Stage Solver (unified full and stage-2 paths)
%%   If Z_fixed is empty, runs both stages; otherwise skips Stage 1.
%% ========================================================================
function [feasible, Z_opt, M_opt, w1_opt, b_opt, w2_opt, buy_opt] = milp_solve_sw( ...
    m, travel, work_idx, work_which, supp_idx, n_work, n_supply, total_travel, ...
    WY, WM, CM, CW, CS, LOAD_LIMIT, MAX_DAYS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, Z_fixed)

    run_stage1 = isempty(Z_fixed);

    n_vars = 3 * n_work + 3 * n_supply + 4 * (m + 2);
    intvars = 1:(3 * n_work + 3 * n_supply);
    off_w1 = 0; off_b = off_w1 + n_work; off_w2 = off_b + n_work;
    off_bO = off_w2 + n_work; off_bH = off_bO + n_supply; off_bF = off_bH + n_supply;
    off_O = off_bF + n_supply; off_H = off_O + (m + 2);
    off_F = off_H + (m + 2); off_M = off_F + (m + 2);
    [Aeq, beq, A, b] = build_milp_sw(m, travel, work_idx, work_which, supp_idx, ...
        n_work, n_supply, total_travel, CM, CW, CS, LOAD_LIMIT, MAX_DAYS, WM, ...
        off_w1, off_b, off_w2, off_bO, off_bH, off_bF, off_O, off_H, off_F, off_M, n_vars);
    lb = zeros(n_vars, 1); ub = inf(n_vars, 1);
    lb(off_O + 1) = INIT_O; ub(off_O + 1) = INIT_O;
    lb(off_H + 1) = INIT_H; ub(off_H + 1) = INIT_H;
    lb(off_F + 1) = INIT_F; ub(off_F + 1) = INIT_F;
    lb(off_M + 1) = INIT_M; ub(off_M + 1) = INIT_M;
    for j = 1:n_work
        ub(off_b  + j) = 1;
        ub(off_w1 + j) = WM(work_which(j));
        ub(off_w2 + j) = WM(work_which(j));
    end
    opts = optimoptions("intlinprog", "Display", "off");

    if run_stage1
        % Stage 1: maximize Z
        f1 = zeros(n_vars, 1);
        for j = 1:n_work
            f1(off_w1 + j) = -WY(work_which(j));
            f1(off_w2 + j) = -WY(work_which(j));
        end
        [x1, fval1, flag] = intlinprog(f1, intvars, A, b, Aeq, beq, lb, ub, opts);
        if flag <= 0 || isempty(x1)
            feasible = false; Z_opt = 0; M_opt = 0;
            w1_opt = []; b_opt = []; w2_opt = []; buy_opt = []; return;
        end
        Z_opt = INIT_Z + round(-fval1);
    else
        Z_opt = Z_fixed;
    end

    % Stage 2: maximize M with Z fixed
    if n_work > 0
        A2 = [A; zeros(2, n_vars)];
        b2 = [b; Z_opt - INIT_Z; -(Z_opt - INIT_Z)];
        nr = size(A, 1);
        for j = 1:n_work
            A2(nr + 1, off_w1 + j) =  WY(work_which(j));
            A2(nr + 1, off_w2 + j) =  WY(work_which(j));
            A2(nr + 2, off_w1 + j) = -WY(work_which(j));
            A2(nr + 2, off_w2 + j) = -WY(work_which(j));
        end
    else
        A2 = A; b2 = b;
    end
    f2 = zeros(n_vars, 1); f2(off_M + m + 2) = -1;
    [x2, fval2, flag2] = intlinprog(f2, intvars, A2, b2, Aeq, beq, lb, ub, opts);
    if flag2 <= 0 || isempty(x2)
        feasible = false; Z_opt = 0; M_opt = 0;
        w1_opt = []; b_opt = []; w2_opt = []; buy_opt = []; return;
    end
    M_opt = round(-fval2);
    w1_opt = round(x2(off_w1 + (1:n_work)));
    b_opt  = round(x2(off_b  + (1:n_work)));
    w2_opt = round(x2(off_w2 + (1:n_work)));
    buy_opt = zeros(n_supply, 3);
    for k = 1:n_supply
        buy_opt(k, 1) = round(x2(off_bO + k));
        buy_opt(k, 2) = round(x2(off_bH + k));
        buy_opt(k, 3) = round(x2(off_bF + k));
    end
    feasible = true;
end

%% ========================================================================
%% Build MILP Constraint Matrix
%% ========================================================================
function [Aeq, beq, A, b] = build_milp_sw(m, travel, work_idx, work_which, supp_idx, ...
    n_work, n_supply, total_travel, CM, CW, CS, LOAD_LIMIT, MAX_DAYS, WM, ...
    off_w1, off_b, off_w2, off_bO, off_bH, off_bF, off_O, off_H, off_F, off_M, n_vars)
    n_eq = 4 * (m + 1);
    Aeq = zeros(n_eq, n_vars); beq = zeros(n_eq, 1);
    eq = 0;
    for i = 1:(m + 1)
        d = travel(i); widx = work_idx(i); sidx = supp_idx(i);
        eq = eq + 1;
        Aeq(eq, off_O + 1 + i) = 1; Aeq(eq, off_O + i) = -1;
        beq(eq) = -d * CM(1);
        if widx > 0
            Aeq(eq, off_w1 + widx) = CW(1);
            Aeq(eq, off_w2 + widx) = CW(1);
            Aeq(eq, off_b  + widx) = CS(1);
        end
        if sidx > 0, Aeq(eq, off_bO + sidx) = -1; end

        eq = eq + 1;
        Aeq(eq, off_H + 1 + i) = 1; Aeq(eq, off_H + i) = -1;
        beq(eq) = -d * CM(2);
        if widx > 0
            Aeq(eq, off_w1 + widx) = CW(2);
            Aeq(eq, off_w2 + widx) = CW(2);
            Aeq(eq, off_b  + widx) = CS(2);
        end
        if sidx > 0, Aeq(eq, off_bH + sidx) = -1; end

        eq = eq + 1;
        Aeq(eq, off_F + 1 + i) = 1; Aeq(eq, off_F + i) = -1;
        beq(eq) = -d * CM(3);
        if widx > 0
            Aeq(eq, off_w1 + widx) = CW(3);
            Aeq(eq, off_w2 + widx) = CW(3);
            Aeq(eq, off_b  + widx) = CS(3);
        end
        if sidx > 0, Aeq(eq, off_bF + sidx) = -1; end

        eq = eq + 1;
        Aeq(eq, off_M + 1 + i) = 1; Aeq(eq, off_M + i) = -1;
        beq(eq) = 0;
        if sidx > 0
            Aeq(eq, off_bO + sidx) = 2;
            Aeq(eq, off_bH + sidx) = 1;
            Aeq(eq, off_bF + sidx) = 2;
        end
    end
    n_ineq = (m + 2) + 1 + 2 * n_work + 3 * n_supply;
    A = zeros(n_ineq, n_vars); b = zeros(n_ineq, 1);
    ineq = 0;
    for i = 0:(m + 1)
        ineq = ineq + 1;
        A(ineq, off_O + 1 + i) = 1;
        A(ineq, off_H + 1 + i) = 1;
        A(ineq, off_F + 1 + i) = 1;
        b(ineq) = LOAD_LIMIT;
    end
    ineq = ineq + 1;
    for j = 1:n_work
        A(ineq, off_w1 + j) = 1; A(ineq, off_b + j) = 1; A(ineq, off_w2 + j) = 1;
    end
    b(ineq) = MAX_DAYS - total_travel;
    for j = 1:n_work
        ineq = ineq + 1; A(ineq, off_w1 + j) = 1;
        b(ineq) = WM(work_which(j));
        ineq = ineq + 1; A(ineq, off_w2 + j) = 1;
        A(ineq, off_b + j) = -WM(work_which(j));
        b(ineq) = 0;
    end
    for i = 1:(m + 1)
        if supp_idx(i) > 0
            d = travel(i);
            ineq = ineq + 1; A(ineq, off_O + i) = -1; b(ineq) = -d * CM(1);
            ineq = ineq + 1; A(ineq, off_H + i) = -1; b(ineq) = -d * CM(2);
            ineq = ineq + 1; A(ineq, off_F + i) = -1; b(ineq) = -d * CM(3);
        end
    end
end

%% ========================================================================
%% Print Daily Schedule (with work_lookup and vectorized travel)
%% ========================================================================
function print_daily_schedule_sw(pid, w1, b, w2, buy, all_xy, dist, names, ...
    CM, CW, CS, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT)
    m = length(pid) - 2;
    travel = dist(sub2ind(size(dist), pid(1:end-1), pid(2:end)));
    total_travel = sum(travel); total_stop = sum(b);
    T = total_travel + sum(w1) + total_stop + sum(w2);
    work_at = []; work_wh = [];
    for k = 2:(m + 1)
        pt = pid(k);
        if pt >= 3 && pt <= 5
            work_at(end+1) = k; work_wh(end+1) = pt - 2;
        end
    end

    % Precompute work lookup: O(1) access instead of find()
    work_lookup = zeros(1, m + 2);
    for idx = 1:length(work_at)
        work_lookup(work_at(idx)) = idx;
    end

    px = zeros(1, T + 1); py = zeros(1, T + 1);
    cO = zeros(1, T); cH = zeros(1, T); cF = zeros(1, T);
    zG = zeros(1, T); isSup = false(1, T);
    action = cell(1, T);
    px(1) = all_xy(pid(1), 1); py(1) = all_xy(pid(1), 2);
    cur_x = px(1); cur_y = py(1); day = 0;
    for k = 1:(m + 1)
        d = travel(k);
        tgt_x = all_xy(pid(k+1), 1); tgt_y = all_xy(pid(k+1), 2);
        dx_sign = sign(tgt_x - cur_x); dy_sign = sign(tgt_y - cur_y);
        for step = 1:d
            day = day + 1;
            if abs(cur_x - tgt_x) > 0, cur_x = cur_x + dx_sign;
            else, cur_y = cur_y + dy_sign; end
            px(day + 1) = cur_x; py(day + 1) = cur_y;
            cO(day) = CM(1); cH(day) = CM(2); cF(day) = CM(3);
            if step == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7
                    isSup(day) = true;
                    action{day} = sprintf("Supply@%s", names{to_pt});
                else
                    action{day} = sprintf("Move->%s", names{to_pt});
                end
            else
                action{day} = "Move";
            end
        end
        wk = work_lookup(k + 1);
        if wk > 0
            if w1(wk) > 0
                for ww = 1:w1(wk)
                    day = day + 1; px(day + 1) = cur_x; py(day + 1) = cur_y;
                    cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
                    zG(day) = WY(work_wh(wk));
                    action{day} = sprintf("Work@%s", names{pid(k+1)});
                end
            end
            if b(wk) > 0
                day = day + 1; px(day + 1) = cur_x; py(day + 1) = cur_y;
                cO(day) = CS(1); cH(day) = CS(2); cF(day) = CS(3);
                action{day} = sprintf("Stop@%s", names{pid(k+1)});
            end
            if w2(wk) > 0
                for ww = 1:w2(wk)
                    day = day + 1; px(day + 1) = cur_x; py(day + 1) = cur_y;
                    cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
                    zG(day) = WY(work_wh(wk));
                    action{day} = sprintf("Work@%s", names{pid(k+1)});
                end
            end
        end
    end
    O = INIT_O; H = INIT_H; F = INIT_F; M = INIT_M; Z = INIT_Z;
    sup_day_idx = find(isSup);
    sep = repmat('-', 1, 75);
    fprintf('%s\n', sep);
    fprintf("Day | Location  | Action          |   O |   H |   F |   M |   Z | Buy(O,H,F)\n");
    fprintf('%s\n', sep);
    for t = 1:T
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        buy_str = "-";
        if isSup(t)
            sk = find(sup_day_idx == t, 1);
            if ~isempty(sk) && sk <= size(buy, 1)
                bO = buy(sk, 1); bH = buy(sk, 2); bF = buy(sk, 3);
                if bO + bH + bF <= LOAD_LIMIT - (O + H + F)
                    cost = bO * 2 + bH * 1 + bF * 2;
                    if cost <= M
                        O = O + bO; H = H + bH; F = F + bF; M = M - cost;
                        buy_str = sprintf("(%d,%d,%d)", bO, bH, bF);
                    end
                end
            end
        end
        Z = Z + zG(t);
        fprintf("%3d | (%2d,%-2d)  | %-15s | %3d | %3d | %3d | %3d | %3d | %s\n", ...
            t, px(t+1), py(t+1), action{t}, O, H, F, M, Z, buy_str);
    end
    fprintf('%s\n', sep);
    fprintf("Final at E: Z=%d, M=%d\n\n", Z, M);
end
