function milp_task1()
% ================================================================
% Solution 3: MILP + Skeleton Enumeration (Optimized)
%
% Improvements over original:
%   1. max_seq uses theoretical formula (matching Solution 1)
%   2. buyO/buyH/buyF added to intvars for strict integer solutions
%   3. Full daily schedule output (position, action, resources, purchases)
%   4. Real-time progress indicator
%   5. milp_skeleton returns buy_opt for schedule reconstruction
%   6. Timer for total runtime
%
% Dependency: MATLAB Optimization Toolbox (intlinprog)
% ================================================================

%% ===== Parameters =====
CM = [2, 3, 2];
CW = [5, 4, 3];
WY = [20, 15, 28];
WM = [4, 5, 3];

MAX_DAYS   = 30;
LOAD_LIMIT = 120;
INIT_O = 35;  INIT_H = 45;  INIT_F = 30;  INIT_M = 240;  INIT_Z = 100;

all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
names  = {'B','E','W1','W2','W3','S1','S2'};

n_pts = 7;
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
    end
end

inter_idx = [3 4 5 6 7];

if ~exist('intlinprog','file')
    error('This solution requires Optimization Toolbox (intlinprog).');
end

%% ===== Compute max_seq =====
min_B_inter  = min(dist(1, inter_idx));
min_inter_E  = min(dist(inter_idx, 2));
min_ii       = min(dist(inter_idx, inter_idx) + 100*eye(5));
max_seq = floor((MAX_DAYS - min_B_inter - min_inter_E) / min_ii) + 1;

total_skeletons = sum(arrayfun(@(k) 5^k, 0:max_seq));
fprintf('Theoretical max intermediate points: %d\n', max_seq);
fprintf('Total skeletons (before filtering): %d\n\n', total_skeletons);

%% ===== Main Enumeration =====
best_Z = -inf;  best_M = -inf;
best_path = []; best_wdays = []; best_buy = [];
count = 0;  feasible_count = 0;
overall_tic = tic;

for seq_len = 0:max_seq
    n_seqs = 5^seq_len;
    for si = 1:n_seqs
        count = count + 1;

        if mod(count, 2000) == 0 || count == 1
            elapsed = toc(overall_tic);
            fprintf('  Progress: %6d/%d (%5.1f%%) | Time: %5.1fs | Feasible: %d | Best Z=%d M=%d\n', ...
                count, total_skeletons, 100*count/total_skeletons, elapsed, feasible_count, best_Z, best_M);
        end

        seq = zeros(1, seq_len); tmp = si - 1;
        for j = seq_len:-1:1
            seq(j) = mod(tmp, 5) + 1; tmp = floor(tmp / 5);
        end
        pid = [1, inter_idx(seq), 2];

        has_dup = false;
        for k = 2:length(pid)
            if pid(k) == pid(k-1), has_dup = true; break; end
        end
        if has_dup, continue; end

        m = length(pid) - 2;
        travel = zeros(1, m+1); total_travel = 0;
        for k = 1:(m+1)
            travel(k) = dist(pid(k), pid(k+1));
            total_travel = total_travel + travel(k);
        end
        if total_travel > MAX_DAYS, continue; end

        work_idx = zeros(1, m+1);  work_which = [];
        supp_idx = zeros(1, m+1);
        n_work = 0;  n_supply = 0;
        for k = 1:(m+1)
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

        [feasible, Z, M, w_opt, buy_opt] = milp_skeleton( ...
            m, travel, work_idx, work_which, supp_idx, ...
            n_work, n_supply, total_travel, WY, WM, CM, CW, ...
            LOAD_LIMIT, MAX_DAYS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);

        if feasible
            feasible_count = feasible_count + 1;
            if Z > best_Z || (Z == best_Z && M > best_M)
                best_Z = Z;  best_M = M;
                best_path = pid;  best_wdays = w_opt;
                best_buy  = buy_opt;
            end
        end
    end
end

elapsed_total = toc(overall_tic);

fprintf('\n========================================\n');
fprintf('  MILP Solution 3 - Optimal Result\n');
fprintf('========================================\n');
fprintf('Total time:    %.2f seconds\n', elapsed_total);
fprintf('Skeletons:     %d evaluated, %d feasible\n', count, feasible_count);
fprintf('Optimal Z:     %d\n', best_Z);
fprintf('Optimal M:     %d\n', best_M);
fprintf('Path:          ');
for i = 1:length(best_path)
    fprintf('%s', names{best_path(i)});
    if i < length(best_path), fprintf(' -> '); end
end
fprintf('\n');
if ~isempty(best_wdays)
    fprintf('Work days:     '); fprintf('%d ', best_wdays); fprintf('\n');
end
if ~isempty(best_buy) && size(best_buy,1) > 0
    fprintf('Supply purchases (O,H,F):\n');
    m_bb = length(best_path) - 2;
    sidx = 0;
    for k = 1:(m_bb+1)
        pt = best_path(k+1);
        if pt == 6 || pt == 7
            sidx = sidx + 1;
            fprintf('  %s: (%d, %d, %d)\n', ...
                names{pt}, best_buy(sidx,1), best_buy(sidx,2), best_buy(sidx,3));
        end
    end
end
fprintf('\n');
print_daily_schedule_milp(best_path, best_wdays, best_buy, ...
    all_xy, dist, names, CM, CW, WY, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT);
end

% =================================================================
% milp_skeleton - Build and solve event-driven MILP for a skeleton
%   Returns buy_opt (n_supply x 3), purchase quantities per station
% =================================================================
function [feasible, Z_opt, M_opt, w_opt, buy_opt] = milp_skeleton( ...
    m, travel, work_idx, work_which, supp_idx, ...
    n_work, n_supply, total_travel, WY, WM, CM, CW, LOAD_LIMIT, MAX_DAYS, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

n_vars  = n_work + 3*n_supply + 4*(m+2);
intvars = 1:(n_work + 3*n_supply);

off_w  = 0;
off_bO = n_work;
off_bH = n_work + n_supply;
off_bF = n_work + 2*n_supply;
off_O  = n_work + 3*n_supply;
off_H  = off_O + (m+2);
off_F  = off_H + (m+2);
off_M  = off_F + (m+2);

% ---- Equality constraints ----
n_eq = 4*(m+1);
Aeq = zeros(n_eq, n_vars); beq = zeros(n_eq, 1);
eq = 0;

for i = 1:(m+1)
    d    = travel(i);
    widx = work_idx(i);
    sidx = supp_idx(i);

    eq = eq + 1;
    Aeq(eq, off_O+1+i) =  1;   Aeq(eq, off_O+i) = -1;
    beq(eq) = -d * CM(1);
    if widx > 0, Aeq(eq, off_w + widx)  =  CW(1); end
    if sidx > 0, Aeq(eq, off_bO + sidx) = -1;     end

    eq = eq + 1;
    Aeq(eq, off_H+1+i) =  1;   Aeq(eq, off_H+i) = -1;
    beq(eq) = -d * CM(2);
    if widx > 0, Aeq(eq, off_w + widx)  =  CW(2); end
    if sidx > 0, Aeq(eq, off_bH + sidx) = -1;     end

    eq = eq + 1;
    Aeq(eq, off_F+1+i) =  1;   Aeq(eq, off_F+i) = -1;
    beq(eq) = -d * CM(3);
    if widx > 0, Aeq(eq, off_w + widx)  =  CW(3); end
    if sidx > 0, Aeq(eq, off_bF + sidx) = -1;     end

    eq = eq + 1;
    Aeq(eq, off_M+1+i) =  1;   Aeq(eq, off_M+i) = -1;
    beq(eq) = 0;
    if sidx > 0
        Aeq(eq, off_bO + sidx) = 2;
        Aeq(eq, off_bH + sidx) = 1;
        Aeq(eq, off_bF + sidx) = 2;
    end
end

% ---- Inequality constraints ----
n_ineq = (m+2) + 1 + n_work + 3*n_supply;
A = zeros(n_ineq, n_vars); b = zeros(n_ineq, 1);
ineq = 0;

for i = 0:(m+1)
    ineq = ineq + 1;
    A(ineq, off_O+1+i) = 1; A(ineq, off_H+1+i) = 1; A(ineq, off_F+1+i) = 1;
    b(ineq) = LOAD_LIMIT;
end

ineq = ineq + 1;
for j = 1:n_work, A(ineq, off_w+j) = 1; end
b(ineq) = MAX_DAYS - total_travel;

for j = 1:n_work
    ineq = ineq + 1;
    A(ineq, off_w+j) = 1;
    b(ineq) = WM(work_which(j));
end

for i = 1:(m+1)
    if supp_idx(i) > 0
        d = travel(i);
        ineq = ineq + 1;
        A(ineq, off_O+i) = -1;   b(ineq) = -d*CM(1);
        ineq = ineq + 1;
        A(ineq, off_H+i) = -1;   b(ineq) = -d*CM(2);
        ineq = ineq + 1;
        A(ineq, off_F+i) = -1;   b(ineq) = -d*CM(3);
    end
end

% ---- Bounds ----
lb = zeros(n_vars, 1); ub = inf(n_vars, 1);
lb(off_O+1) = INIT_O; ub(off_O+1) = INIT_O;
lb(off_H+1) = INIT_H; ub(off_H+1) = INIT_H;
lb(off_F+1) = INIT_F; ub(off_F+1) = INIT_F;
lb(off_M+1) = INIT_M; ub(off_M+1) = INIT_M;

opts = optimoptions('intlinprog', 'Display', 'off');

% ---- Stage 1: max Z ----
f1 = zeros(n_vars, 1);
for j = 1:n_work, f1(off_w + j) = -WY(work_which(j)); end

[x1, fval1, flag] = intlinprog(f1, intvars, A, b, Aeq, beq, lb, ub, opts);
if flag <= 0 || isempty(x1)
    feasible = false; Z_opt = 0; M_opt = 0; w_opt = []; buy_opt = []; return;
end
Z_opt = INIT_Z + round(-fval1);

% ---- Stage 2: fix Z, max M ----
if n_work > 0
    A2 = [A; zeros(2, n_vars)];
    b2 = [b; Z_opt - INIT_Z; -(Z_opt - INIT_Z)];
    nr = size(A, 1);
    for j = 1:n_work
        A2(nr+1, off_w+j) =  WY(work_which(j));
        A2(nr+2, off_w+j) = -WY(work_which(j));
    end
else
    A2 = A; b2 = b;
end

f2 = zeros(n_vars, 1);
f2(off_M + m + 2) = -1;

[x2, fval2, flag2] = intlinprog(f2, intvars, A2, b2, Aeq, beq, lb, ub, opts);
if flag2 <= 0 || isempty(x2)
    feasible = false; Z_opt = 0; M_opt = 0; w_opt = []; buy_opt = []; return;
end

M_opt  = round(-fval2);
w_opt  = round(x2(off_w + (1:n_work)));
buy_opt = zeros(n_supply, 3);
for k = 1:n_supply
    buy_opt(k,1) = round(x2(off_bO + k));
    buy_opt(k,2) = round(x2(off_bH + k));
    buy_opt(k,3) = round(x2(off_bF + k));
end
feasible = true;
end

% =================================================================
% print_daily_schedule_milp - Reconstruct daily schedule from MILP result
% =================================================================
function print_daily_schedule_milp(pid, wdays, buy, all_xy, dist, ...
    names, CM, CW, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT)

m = length(pid) - 2;
travel = zeros(1, m+1);
for k = 1:(m+1), travel(k) = dist(pid(k), pid(k+1)); end
total_travel = sum(travel);

if isempty(wdays)
    T = total_travel;
else
    T = total_travel + sum(wdays);
end

work_at = []; work_wh = [];
for k = 2:(m+1)
    pt = pid(k);
    if pt >= 3 && pt <= 5
        work_at(end+1) = k; work_wh(end+1) = pt - 2;
    end
end

px = zeros(1, T+1); py = zeros(1, T+1);
cO = zeros(1, T); cH = zeros(1, T); cF = zeros(1, T);
zG = zeros(1, T); isSup = false(1, T);
action = cell(1, T);

px(1) = all_xy(pid(1),1); py(1) = all_xy(pid(1),2);
cur_x = px(1); cur_y = py(1);
day = 0;

for k = 1:(m+1)
    d = travel(k);
    tgt_x = all_xy(pid(k+1),1); tgt_y = all_xy(pid(k+1),2);
    dx_sign = sign(tgt_x - cur_x); dy_sign = sign(tgt_y - cur_y);

    for step = 1:d
        day = day + 1;
        if abs(cur_x - tgt_x) > 0
            cur_x = cur_x + dx_sign;
        else
            cur_y = cur_y + dy_sign;
        end
        px(day+1) = cur_x; py(day+1) = cur_y;
        cO(day) = CM(1); cH(day) = CM(2); cF(day) = CM(3);

        if step == d
            to_pt = pid(k+1);
            if to_pt == 6 || to_pt == 7
                isSup(day) = true;
                action{day} = sprintf('Supply@%s', names{to_pt});
            else
                action{day} = sprintf('Move->%s', names{to_pt});
            end
        else
            action{day} = 'Move';
        end
    end

    wk = find(work_at == k+1, 1);
    if ~isempty(wk) && wdays(wk) > 0
        for w = 1:wdays(wk)
            day = day + 1;
            px(day+1) = cur_x; py(day+1) = cur_y;
            cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
            zG(day) = WY(work_wh(wk));
            action{day} = sprintf('Work@%s', names{pid(k+1)});
        end
    end
end

O = INIT_O; H = INIT_H; F = INIT_F; M = INIT_M; Z = INIT_Z;
sup_day_idx = find(isSup);

fprintf('----+-----------+-----------------+-----+-----+-----+-----+-----+------------\n');
fprintf('Day | Location  | Action          |   O |   H |   F |   M |   Z | Buy(O,H,F)\n');
fprintf('----+-----------+-----------------+-----+-----+-----+-----+-----+------------\n');

for t = 1:T
    O = O - cO(t); H = H - cH(t); F = F - cF(t);

    buy_str = '-';
    if isSup(t)
        sk = find(sup_day_idx == t, 1);
        if ~isempty(sk) && sk <= size(buy, 1)
            bO = buy(sk,1); bH = buy(sk,2); bF = buy(sk,3);
            if bO + bH + bF <= LOAD_LIMIT - (O + H + F)
                cost = bO*2 + bH*1 + bF*2;
                if cost <= M
                    O = O + bO; H = H + bH; F = F + bF;
                    M = M - cost;
                    buy_str = sprintf('(%d,%d,%d)', bO, bH, bF);
                end
            end
        end
    end

    Z = Z + zG(t);

    fprintf('%3d | (%2d,%-2d)  | %-15s | %3d | %3d | %3d | %3d | %3d | %s\n', ...
        t, px(t+1), py(t+1), action{t}, ...
        round(O), round(H), round(F), round(M), round(Z), buy_str);
end

fprintf('----+-----------+-----------------+-----+-----+-----+-----+-----+------------\n');
fprintf('Final: Z=%d, M=%d\n\n', round(Z), round(M));
end
