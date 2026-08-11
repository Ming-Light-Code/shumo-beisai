%% ========================================================================
%% solve_task2_storm_milp.m - Task 2: Storm-Only MILP with Supply Mooring
%% ========================================================================
%% 功能: 全雷暴极端天气下含补给站停泊策略的整数规划求解
%% 策略: 在补给站停泊(moor)消耗资源腾出载重空间后再采购
%%       同时支持 w1 + stop + w2 的停泊-再作业决策
%% 场景: 30天全雷暴, 10x10网格, Z=100(无法作业)
%% 优化: 字典序 max Z -> max M
%% 依赖: Optimization Toolbox (intlinprog, optimproblem)
%% 输出: 最优停泊-采购方案与逐日航线
%% ========================================================================

function solve_task2_storm_milp()

%% ====== Parameters ======
CM = [8, 4, 3]; CW = [8, 4, 6]; CS = [3, 3, 2];
WY = [20, 15, 28]; WM = [4, 5, 3];
MAX_DAYS = 30; LOAD_LIMIT = 120;
INIT_O = 35; INIT_H = 45; INIT_F = 30; INIT_M = 240; INIT_Z = 100;

all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
names = {"B", "E", "W1", "W2", "W3", "S1", "S2"};

n_pts = 7;
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i, j) = abs(all_xy(i,1) - all_xy(j,1)) + abs(all_xy(i,2) - all_xy(j,2));
    end
end
inter_idx = [3 4 5 6 7];
STORM_NO_WORK = true;

%% ====== Skeleton Enumeration ======
best_Z   = -inf; best_M   = -inf; best_info = [];
count = 0; feasible_count = 0;
tic_total = tic;

fprintf("================ TASK 2: Storm-Only MILP (Supply Mooring) ================\n");
fprintf("STORM_NO_WORK = %d\n\n", STORM_NO_WORK);

for seq_len = 0:4
    n_seq = 5^seq_len;
    for sid = 1:n_seq
        count = count + 1;
        seq = zeros(1, seq_len); tmp = sid - 1;
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
        if STORM_NO_WORK && any(pid >= 3 & pid <= 5), continue; end

        m = length(pid) - 2;
        travel = zeros(1, m + 1);
        for k = 1:(m + 1), travel(k) = dist(pid(k), pid(k+1)); end
        total_travel = sum(travel);
        if total_travel > MAX_DAYS, continue; end

        work_idx   = zeros(1, m + 1); work_which = [];
        supp_idx   = zeros(1, m + 1);
        n_work = 0; n_supp = 0;
        for k = 1:(m + 1)
            pt = pid(k+1);
            if pt >= 3 && pt <= 5
                n_work = n_work + 1; work_idx(k) = n_work;
                work_which(n_work) = pt - 2;
            elseif pt == 6 || pt == 7
                n_supp = n_supp + 1; supp_idx(k) = n_supp;
            end
        end

        [Zv, Mv, info] = solve_skeleton(pid, travel, work_idx, work_which, supp_idx, ...
            n_work, n_supp, total_travel, CM, CW, CS, WY, WM, ...
            INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS);

        if Zv > -inf
            feasible_count = feasible_count + 1;
            if Zv > best_Z || (Zv == best_Z && Mv > best_M)
                best_Z = Zv; best_M = Mv; best_info = info;
            end
        end
    end
end

elapsed = toc(tic_total);

%% ====== Output Results ======
fprintf("\n================ STORM-ONLY OPTIMAL ================\n");
fprintf("Scanned:        %d skeletons\n", count);
fprintf("Feasible:       %d skeletons\n", feasible_count);
fprintf("Solve time:     %.2f s\n", elapsed);

if best_Z <= -inf
    fprintf("No feasible solution found.\n");
    return;
end

fprintf("Optimal Z:      %d\n", best_Z);
fprintf("Optimal M:      %d\n", best_M);
fprintf("Path:           ");
for i = 1:length(best_info.pid)
    fprintf("%s", names{best_info.pid(i)});
    if i < length(best_info.pid), fprintf(" -> "); end
end
fprintf("\n");

print_schedule(best_info, names, all_xy, CM, CW, CS, WY, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT);

end

%% ========================================================================
%% Problem-Based Optimization for a Single Skeleton
%% ========================================================================
function [Zv, Mv, info] = solve_skeleton(pid, travel, work_idx, work_which, supp_idx, ...
    n_work, n_supp, total_travel, CM, CW, CS, WY, WM, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS)
    Zv = -inf; Mv = -inf; info = [];
    m = length(travel) - 1;

    if n_work == 0 && n_supp == 0
        O_end = INIT_O - total_travel * CM(1);
        H_end = INIT_H - total_travel * CM(2);
        F_end = INIT_F - total_travel * CM(3);
        if O_end >= 0 && H_end >= 0 && F_end >= 0 && total_travel <= MAX_DAYS
            Zv = INIT_Z; Mv = INIT_M;
            info.pid = pid; info.travel = travel;
            info.work_idx = work_idx; info.work_which = work_which;
            info.supp_idx = supp_idx;
            info.w1 = []; info.b = []; info.w2 = [];
            info.moor = []; info.buyO = []; info.buyH = []; info.buyF = [];
        end
        return;
    end

    prob = optimproblem("ObjectiveSense", "maximize");

    % Decision variables: work blocks with optional stop break
    if n_work > 0
        w1 = optimvar("w1", n_work, 1, "LowerBound", 0, "Type", "integer");
        b  = optimvar("b",  n_work, 1, "LowerBound", 0, "UpperBound", 1, "Type", "integer");
        w2 = optimvar("w2", n_work, 1, "LowerBound", 0, "Type", "integer");
        for j = 1:n_work
            prob.Constraints.(["w1ub", num2str(j)]) = w1(j) <= WM(work_which(j));
            prob.Constraints.(["w2ub", num2str(j)]) = w2(j) <= WM(work_which(j)) * b(j);
        end
    else
        w1 = []; b = []; w2 = [];
    end

    % Decision variables: moor at supply + purchase quantities
    if n_supp > 0
        moor = optimvar("moor", n_supp, 1, "LowerBound", 0, "UpperBound", 1, "Type", "integer");
        buyO = optimvar("buyO", n_supp, 1, "LowerBound", 0, "Type", "integer");
        buyH = optimvar("buyH", n_supp, 1, "LowerBound", 0, "Type", "integer");
        buyF = optimvar("buyF", n_supp, 1, "LowerBound", 0, "Type", "integer");
    else
        moor = []; buyO = []; buyH = []; buyF = [];
    end

    % Day limit
    day_expr = total_travel;
    if n_work > 0, day_expr = day_expr + sum(w1) + sum(b) + sum(w2); end
    if n_supp > 0, day_expr = day_expr + sum(moor); end
    prob.Constraints.daylim = day_expr <= MAX_DAYS;

    % Resource balance along path
    O = INIT_O; H = INIT_H; F = INIT_F; Money = INIT_M;
    s = 0;
    for k = 1:(m + 1)
        O = O - travel(k) * CM(1); H = H - travel(k) * CM(2); F = F - travel(k) * CM(3);

        if work_idx(k) > 0
            j = work_idx(k);
            O = O - w1(j) * CW(1) - b(j) * CS(1) - w2(j) * CW(1);
            H = H - w1(j) * CW(2) - b(j) * CS(2) - w2(j) * CW(2);
            F = F - w1(j) * CW(3) - b(j) * CS(3) - w2(j) * CW(3);
        elseif supp_idx(k) > 0
            s = s + 1;
            prob.Constraints.(["moorO", num2str(s)]) = O >= moor(s) * CS(1);
            prob.Constraints.(["moorH", num2str(s)]) = H >= moor(s) * CS(2);
            prob.Constraints.(["moorF", num2str(s)]) = F >= moor(s) * CS(3);
            O = O - moor(s) * CS(1); H = H - moor(s) * CS(2); F = F - moor(s) * CS(3);
            O = O + buyO(s); H = H + buyH(s); F = F + buyF(s);
            Money = Money - 2 * buyO(s) - buyH(s) - 2 * buyF(s);
            prob.Constraints.(["load", num2str(s)]) = O + H + F <= LOAD_LIMIT;
            prob.Constraints.(["cash", num2str(s)]) = Money >= 0;
        end
    end

    prob.Constraints.endO = O >= 0;
    prob.Constraints.endH = H >= 0;
    prob.Constraints.endF = F >= 0;
    prob.Constraints.endM = Money >= 0;

    inc = optimexpr(1);
    if n_work > 0
        for j = 1:n_work
            inc = inc + WY(work_which(j)) * (w1(j) + w2(j));
        end
    end
    prob.Objective = inc;

    % Stage 1: max Z
    opts = optimoptions("intlinprog", "Display", "off", "MaxTime", 5);
    try
        [sol1, fval1, exit1] = solve(prob, "Options", opts);
    catch
        return;
    end
    if exit1 <= 0 || (n_work > 0 && fval1 < -1e6), return; end

    % Stage 2: max M with Z fixed
    prob2 = prob;
    if n_work > 0, prob2.Constraints.Zfix = inc == fval1; end
    prob2.Objective = -Money;
    try
        [sol2, fval2, exit2] = solve(prob2, "Options", opts);
    catch
        return;
    end
    if exit2 <= 0, return; end

    Zv = INIT_Z + fval1;
    Mv = -fval2;
    info.pid = pid; info.travel = travel;
    info.work_idx = work_idx; info.work_which = work_which;
    info.supp_idx = supp_idx;
    if n_work > 0
        info.w1 = sol2.w1; info.b = sol2.b; info.w2 = sol2.w2;
    else
        info.w1 = []; info.b = []; info.w2 = [];
    end
    if n_supp > 0
        info.moor = sol2.moor;
        info.buyO = sol2.buyO; info.buyH = sol2.buyH; info.buyF = sol2.buyF;
    else
        info.moor = []; info.buyO = []; info.buyH = []; info.buyF = [];
    end
end

%% ========================================================================
%% Print Daily Schedule
%% ========================================================================
function print_schedule(info, names, all_xy, CM, CW, CS, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT)
    pid = info.pid; travel = info.travel;
    work_idx = info.work_idx; work_which = info.work_which; supp_idx = info.supp_idx;
    w1 = info.w1; b = info.b; w2 = info.w2; moor = info.moor;
    buyO = info.buyO; buyH = info.buyH; buyF = info.buyF;
    O = INIT_O; H = INIT_H; F = INIT_F; M = INIT_M; Z = INIT_Z;
    s = 0; day = 0;

    fprintf("\n================ DAY-BY-DAY SCHEDULE ================\n");
    fprintf("%-4s %-8s %-12s %-26s %-6s %-s\n", ...
        "Day", "Pos", "Action", "Detail", "Load", "State (O,H,F,M,Z)");

    for k = 1:length(travel)
        from = pid(k); to = pid(k+1);
        steps = get_path_points(all_xy(from, :), all_xy(to, :));
        for step = 2:size(steps, 1)
            day = day + 1;
            O = O - CM(1); H = H - CM(2); F = F - CM(3);
            fprintf("%-4d (%2d,%2d) %-12s %-26s %-6d (%d,%d,%d,%d,%d)\n", ...
                day, steps(step,1), steps(step,2), "Move", ...
                sprintf("to %s", names{to}), O+H+F, round(O), round(H), round(F), round(M), round(Z));
        end

        to_pt = pid(k+1);
        if work_idx(k) > 0
            j = work_idx(k); wh = work_which(j);
            for ww = 1:w1(j)
                day = day + 1; O = O - CW(1); H = H - CW(2); F = F - CW(3); Z = Z + WY(wh);
                fprintf("%-4d (%2d,%2d) %-12s %-26s %-6d (%d,%d,%d,%d,%d)\n", ...
                    day, all_xy(to_pt,1), all_xy(to_pt,2), "Work", ...
                    sprintf("Work %s +Z=%d", names{to_pt}, WY(wh)), O+H+F, round(O), round(H), round(F), round(M), round(Z));
            end
            if b(j) > 0.5
                day = day + 1; O = O - CS(1); H = H - CS(2); F = F - CS(3);
                fprintf("%-4d (%2d,%2d) %-12s %-26s %-6d (%d,%d,%d,%d,%d)\n", ...
                    day, all_xy(to_pt,1), all_xy(to_pt,2), "Stop", ...
                    "Stop-work (reset)", O+H+F, round(O), round(H), round(F), round(M), round(Z));
            end
            for ww = 1:w2(j)
                day = day + 1; O = O - CW(1); H = H - CW(2); F = F - CW(3); Z = Z + WY(wh);
                fprintf("%-4d (%2d,%2d) %-12s %-26s %-6d (%d,%d,%d,%d,%d)\n", ...
                    day, all_xy(to_pt,1), all_xy(to_pt,2), "Work", ...
                    sprintf("Work %s +Z=%d", names{to_pt}, WY(wh)), O+H+F, round(O), round(H), round(F), round(M), round(Z));
            end
        elseif supp_idx(k) > 0
            s = s + 1;
            if moor(s) > 0.5
                day = day + 1; O = O - CS(1); H = H - CS(2); F = F - CS(3);
                fprintf("%-4d (%2d,%2d) %-12s %-26s %-6d (%d,%d,%d,%d,%d)\n", ...
                    day, all_xy(to_pt,1), all_xy(to_pt,2), "Moor", ...
                    "Park at supply", O+H+F, round(O), round(H), round(F), round(M), round(Z));
            end
            O = O + buyO(s); H = H + buyH(s); F = F + buyF(s);
            cost = 2 * buyO(s) + buyH(s) + 2 * buyF(s);
            M = M - cost;
            fprintf("%-4d (%2d,%2d) %-12s O%+d H%+d F%+d $%d  %-6d (%d,%d,%d,%d,%d)\n", ...
                day, all_xy(to_pt,1), all_xy(to_pt,2), "Supply", ...
                buyO(s), buyH(s), buyF(s), cost, O+H+F, round(O), round(H), round(F), round(M), round(Z));
        end
    end

    fprintf("--------------------------------------------------------\n");
    fprintf("Final at E: Z = %d, M = %d, Total Days = %d\n", Z, M, day);
    fprintf("Load check: %d/%d\n", O+H+F, LOAD_LIMIT);
end

%% ========================================================================
%% Get Grid Path from p1 to p2
%% ========================================================================
function pts = get_path_points(p1, p2)
    d = abs(p2(1) - p1(1)) + abs(p2(2) - p1(2));
    pts = zeros(d + 1, 2);
    pts(1, :) = p1;
    cur = p1;
    idx = 1;
    while cur(1) ~= p2(1)
        cur(1) = cur(1) + sign(p2(1) - cur(1));
        idx = idx + 1;
        pts(idx, :) = cur;
    end
    while cur(2) ~= p2(2)
        cur(2) = cur(2) + sign(p2(2) - cur(2));
        idx = idx + 1;
        pts(idx, :) = cur;
    end
end
