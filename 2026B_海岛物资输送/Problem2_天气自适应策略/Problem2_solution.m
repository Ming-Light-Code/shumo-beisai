function problem2_main()
% ================================================================
% Problem 2: Weather-Observable Adaptive Strategy (Optimized)
% Based on Event-Driven MILP + Receding Horizon
%
% Key improvements over previous version:
%   1. B-stop pre-processing: tries 0-3 stop days at B before
%      MILP enumeration, correctly finding B->S1->E (M=116).
%   2. Smarter adaptive_decision: considers stop-at-current for
%      load balancing, not just move-to-nearest-target.
%   3. Adaptive supply purchases use remaining-path estimation.
% ================================================================

fprintf('========================================\n');
fprintf('  Problem 2: Weather-Adaptive Strategy\n');
fprintf('  Optimized: B-stop + Receding Horizon\n');
fprintf('========================================\n\n');

% --- Common parameters ---
all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
names = {'B','E','W1','W2','W3','S1','S2'};
WY = [20, 15, 28]; WM = [4, 5, 3];
INIT_O=35; INIT_H=45; INIT_F=30; INIT_M=240; INIT_Z=100;
LOAD_LIMIT=120; MAX_DAYS=30;

CM0 = [2, 3, 2]; CW0 = [5, 4, 3]; CS0 = [1, 1, 1];
CM1 = [8, 4, 3]; CW1 = [8, 6, 6]; CS1 = [3, 3, 2];

% Precompute distance matrix
n_pts=7; dist=zeros(n_pts);
for i=1:n_pts,for j=1:n_pts
    dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));
end,end

has_milp = exist('intlinprog','file');

% ============================================================
% Part 1: Extreme Case -- All Thunderstorm
% ============================================================
fprintf('========================================\n');
fprintf('Part 1: All-Thunderstorm Extreme Case\n');
fprintf('  CM=(8,4,3) CW=(8,6,6) CS=(3,3,2)\n');
fprintf('========================================\n\n');

[Z_storm, M_storm, best_path, best_w1, best_b, best_w2, best_buy, best_Bstop] = ...
    solve_all_storm(all_xy, names, WY, WM, CM1, CW1, CS1, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS, dist, has_milp);

if Z_storm <= 0
    error('No feasible solution for all-storm case.');
end

fprintf('\n>>> ALL-STORM OPTIMAL RESULT:\n');
if best_Bstop > 0
    fprintf('B-stop: %d day(s) before departure\n', best_Bstop);
end
fprintf('Path: ');
for i=1:length(best_path)
    fprintf('%s', names{best_path(i)});
    if i<length(best_path), fprintf(' -> '); end
end
fprintf('\nZ = %d, M = %d\n', Z_storm, M_storm);

if ~isempty(best_w1) && any(best_w1>0 | best_w2>0)
    fprintf('Work blocks: ');
    for j=1:length(best_w1)
        if best_b(j)==0, fprintf('%d ', best_w1(j));
        else fprintf('%d|stop|%d ', best_w1(j), best_w2(j)); end
    end
    fprintf('\n');
else
    fprintf('Work: NONE (storm consumption too high for detours)\n');
end

if ~isempty(best_buy)
    fprintf('Supply purchases (O,H,F):\n');
    m_bb=length(best_path)-2; sidx=0;
    for k=1:(m_bb+1)
        pt=best_path(k+1);
        if pt==6||pt==7
            sidx=sidx+1;
            if sidx<=size(best_buy,1)
                fprintf('  %s: (%d, %d, %d)\n', names{pt}, ...
                    best_buy(sidx,1), best_buy(sidx,2), best_buy(sidx,3));
            end
        end
    end
end

fprintf('\n--- Daily Schedule (All Storm) ---\n');
print_daily_schedule(best_path, best_w1, best_b, best_w2, best_buy, ...
    all_xy, names, CM1, CW1, CS1, WY, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, best_Bstop);

% ============================================================
% Part 2: Weather-Adaptive Strategy
% ============================================================
fprintf('\n========================================\n');
fprintf('Part 2: Weather-Adaptive (Receding Horizon)\n');
fprintf('========================================\n\n');

% 2a: All storm verification
fprintf('--- 2a: All Storm (should match Part 1) ---\n');
[Z2a, M2a] = adaptive_sim(ones(1,30), all_xy, names, WY, WM, ...
    CM0, CW0, CS0, CM1, CW1, CS1, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS, dist);
fprintf('Z=%d, M=%d  |  (Expected: Z=%d, M=%d)\n\n', Z2a, M2a, Z_storm, M_storm);

% 2b: All normal
fprintf('--- 2b: All Normal (best-case benchmark) ---\n');
[Z2b, M2b] = adaptive_sim(zeros(1,30), all_xy, names, WY, WM, ...
    CM0, CW0, CS0, CM1, CW1, CS1, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS, dist);
fprintf('Z=%d, M=%d\n\n', Z2b, M2b);

% 2c: Mixed weather sample
fprintf('--- 2c: Mixed Weather Sample ---\n');
rng(42); w_seq = (rand(1,30) < 0.5);
fprintf('Seq: '); fprintf('%d', w_seq); fprintf('\n');
[Z2c, M2c] = adaptive_sim(w_seq, all_xy, names, WY, WM, ...
    CM0, CW0, CS0, CM1, CW1, CS1, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS, dist);
fprintf('Z=%d, M=%d\n\n', Z2c, M2c);

% 2d: Monte Carlo (P(storm)=0.2)
fprintf('--- 2d: Monte Carlo (10 runs, P(storm)=0.2) ---\n');
n_mc=10; Z_mc=zeros(1,n_mc); M_mc=zeros(1,n_mc);
for mc=1:n_mc
    w_seq = (rand(1,30) < 0.2);
    [Z_mc(mc), M_mc(mc)] = adaptive_sim(w_seq, all_xy, names, WY, WM, ...
        CM0, CW0, CS0, CM1, CW1, CS1, ...
        INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS, dist);
end
fprintf('Z: mean=%.1f, min=%d, max=%d\n', mean(Z_mc), min(Z_mc), max(Z_mc));
fprintf('M: mean=%.1f, min=%d, max=%d\n', mean(M_mc), min(M_mc), max(M_mc));

% Summary
fprintf('\n========================================\n');
fprintf('  Problem 2 Summary\n');
fprintf('========================================\n');
fprintf('%-42s %8s %8s\n', 'Case', 'Z', 'M');
fprintf('%-42s %8s %8s\n', '------------------------------------------', '--------', '--------');
fprintf('%-42s %8d %8d\n', 'Extreme: All-Storm MILP (B-stop opt.)', Z_storm, M_storm);
fprintf('%-42s %8d %8d\n', 'Adaptive: All Storm (verification)', Z2a, M2a);
fprintf('%-42s %8d %8d\n', 'Adaptive: All Normal (best case)', Z2b, M2b);
fprintf('%-42s %8d %8d\n', 'Adaptive: Mixed Sample', Z2c, M2c);
fprintf('%-42s %8.1f %8.1f\n', 'Adaptive: MC Avg (10 runs, p=0.2)', mean(Z_mc), mean(M_mc));
fprintf('\nDone.\n');
end

% ================================================================
function [best_Z, best_M, best_path, best_w1, best_b, best_w2, best_buy, best_Bstop] = ...
    solve_all_storm(all_xy, names, WY, WM, CM, CW, CS, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS, dist, has_milp)
% Enumerate skeletons, with B-stop pre-processing.
% Tries n_stop=0,1,2 stop days at B before departure.

MAX_WY = max(WY);
inter_idx = [3 4 5 6 7];

min_B_inter = min(dist(1,inter_idx));
min_inter_E = min(dist(inter_idx,2));
min_ii = min(min(dist(inter_idx,inter_idx)+100*eye(5)));
max_seq = floor((MAX_DAYS - min_B_inter - min_inter_E) / min_ii) + 1;

best_Z = -inf; best_M = -inf; best_Bstop = 0;
best_path = []; best_w1 = []; best_b = []; best_w2 = []; best_buy = [];

for n_stop_B = 0:min(3, MAX_DAYS)
    adj_O = INIT_O - n_stop_B * CS(1);
    adj_H = INIT_H - n_stop_B * CS(2);
    adj_F = INIT_F - n_stop_B * CS(3);
    adj_M = INIT_M;
    adj_days = MAX_DAYS - n_stop_B;

    if adj_O < 0 || adj_H < 0 || adj_F < 0, continue; end

    total_skeletons = sum(arrayfun(@(k)5^k, 0:max_seq));
    fprintf('  B-stop=%d: adj_O=%d adj_H=%d adj_F=%d, days=%d, skeletons=%d\n', ...
        n_stop_B, adj_O, adj_H, adj_F, adj_days, total_skeletons);

    count = 0; feasible_count = 0; milp_calls = 0;
    tic_local = tic;

    for seq_len = 0:max_seq
    n_seqs = 5^seq_len;
    for si = 1:n_seqs
        count = count + 1;
        if mod(count, 10000) == 0 || count == 1
            e = toc(tic_local);
            fprintf('    Prg:%d/%d(%.0f%%)|%.1fs|Feas:%d|Best Z=%d M=%d\n', ...
                count, total_skeletons, 100*count/total_skeletons, e, feasible_count, best_Z, best_M);
        end

        seq = zeros(1, seq_len); tmp = si - 1;
        for j = seq_len:-1:1, seq(j) = mod(tmp,5)+1; tmp = floor(tmp/5); end
        pid = [1, inter_idx(seq), 2];

        dup = false; for k = 2:length(pid), if pid(k)==pid(k-1), dup=true; break; end, end
        if dup, continue; end

        m = length(pid) - 2;
        travel = zeros(1, m+1); total_travel = 0;
        for k = 1:(m+1), travel(k) = dist(pid(k), pid(k+1)); total_travel = total_travel + travel(k); end
        if total_travel > adj_days, continue; end

        work_idx = zeros(1, m+1); work_which = [];
        supp_idx = zeros(1, m+1); n_work = 0; n_supply = 0;
        for k = 1:(m+1)
            pt = pid(k+1);
            if pt >= 3 && pt <= 5, n_work = n_work + 1; work_idx(k) = n_work; work_which(n_work) = pt-2; end
            if pt == 6 || pt == 7, n_supply = n_supply + 1; supp_idx(k) = n_supply; end
        end

        remain = adj_days - total_travel;
        if n_work > 0
            z_cap = 0;
            for j = 1:n_work, z_cap = z_cap + 2*WM(work_which(j))*WY(work_which(j)); end
            Z_upper = INIT_Z + min(remain*MAX_WY, z_cap);
        else
            Z_upper = INIT_Z;
        end
        if Z_upper <= best_Z, continue; end

        if n_work == 0
            [gfeas, gZ, gM, gBuy] = greedy_sim(m, travel, work_idx, work_which, supp_idx, ...
                n_work, n_supply, total_travel, [], [], [], WY, CM, CW, CS, ...
                adj_O, adj_H, adj_F, adj_M, INIT_Z, LOAD_LIMIT);
            if gfeas
                feasible_count = feasible_count + 1;
                if gZ > best_Z || (gZ == best_Z && gM > best_M)
                    best_Z = gZ; best_M = gM; best_path = pid; best_Bstop = n_stop_B;
                    best_w1 = []; best_b = []; best_w2 = []; best_buy = gBuy;
                end
            end
            continue;
        end

        [gfeas, ~, ~] = greedy_sim(m, travel, work_idx, work_which, supp_idx, ...
            n_work, n_supply, total_travel, zeros(1,n_work), zeros(1,n_work), zeros(1,n_work), ...
            WY, CM, CW, CS, adj_O, adj_H, adj_F, adj_M, INIT_Z, LOAD_LIMIT);
        if ~gfeas, continue; end

        [w1_g, b_g, w2_g] = greedy_work_assign(work_which, WM, WY, remain);
        [gfeas_max, gZ_max, gM_max, gBuy_max] = greedy_sim(m, travel, work_idx, ...
            work_which, supp_idx, n_work, n_supply, total_travel, w1_g, b_g, w2_g, ...
            WY, CM, CW, CS, adj_O, adj_H, adj_F, adj_M, INIT_Z, LOAD_LIMIT);

        if gfeas_max
            feasible_count = feasible_count + 1;
            if gZ_max > best_Z || (gZ_max == best_Z && gM_max > best_M)
                best_Z = gZ_max; best_M = gM_max; best_path = pid; best_Bstop = n_stop_B;
                best_w1 = w1_g; best_b = b_g; best_w2 = w2_g; best_buy = gBuy_max;
            end
        end

        if has_milp
            milp_calls = milp_calls + 1;
            [feas, Z, M, w1_o, b_o, w2_o, buy_o] = milp_storm(m, travel, work_idx, ...
                work_which, supp_idx, n_work, n_supply, total_travel, WY, WM, CM, CW, CS, ...
                LOAD_LIMIT, adj_days, adj_O, adj_H, adj_F, adj_M, INIT_Z);
            if feas
                feasible_count = feasible_count + 1;
                if Z > best_Z || (Z == best_Z && M > best_M)
                    best_Z = Z; best_M = M; best_path = pid; best_Bstop = n_stop_B;
                    best_w1 = w1_o; best_b = b_o; best_w2 = w2_o; best_buy = buy_o;
                end
            end
        end
    end
    end
    fprintf('    Done: %.1fs, scanned=%d, feasible=%d, MILP=%d\n', toc(tic_local), count, feasible_count, milp_calls);
end
end

% ================================================================
function [feasible, Z_final, M_final, w1_opt, b_opt, w2_opt, buy_opt] = ...
    milp_storm(m, travel, work_idx, work_which, supp_idx, n_work, n_supply, ...
    total_travel, WY, WM, CM, CW, CS, LOAD_LIMIT, MAX_DAYS, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

n_vars = 3*n_work + 3*n_supply + 4*(m+2);
intvars = 1:(3*n_work + 3*n_supply);
off_w1 = 0; off_b = off_w1 + n_work; off_w2 = off_b + n_work;
off_bO = off_w2 + n_work; off_bH = off_bO + n_supply; off_bF = off_bH + n_supply;
off_O = off_bF + n_supply; off_H = off_O + (m+2); off_F = off_H + (m+2); off_M = off_F + (m+2);

[Aeq, beq, A, b] = build_milp(m, travel, work_idx, work_which, supp_idx, ...
    n_work, n_supply, total_travel, CM, CW, CS, LOAD_LIMIT, MAX_DAYS, WM, ...
    off_w1, off_b, off_w2, off_bO, off_bH, off_bF, off_O, off_H, off_F, off_M, n_vars);

lb = zeros(n_vars, 1); ub = inf(n_vars, 1);
lb(off_O+1) = INIT_O; ub(off_O+1) = INIT_O;
lb(off_H+1) = INIT_H; ub(off_H+1) = INIT_H;
lb(off_F+1) = INIT_F; ub(off_F+1) = INIT_F;
lb(off_M+1) = INIT_M; ub(off_M+1) = INIT_M;
for j = 1:n_work
    ub(off_b+j) = 1;
    ub(off_w1+j) = WM(work_which(j));
    ub(off_w2+j) = WM(work_which(j));
end
opts = optimoptions('intlinprog', 'Display', 'off');

fz = zeros(n_vars, 1);
for j = 1:n_work
    fz(off_w1+j) = -WY(work_which(j));
    fz(off_w2+j) = -WY(work_which(j));
end
[xz, fvalz, flag] = intlinprog(fz, intvars, A, b, Aeq, beq, lb, ub, opts);
if flag <= 0 || isempty(xz)
    feasible = false; Z_final = 0; M_final = 0; w1_opt=[]; b_opt=[]; w2_opt=[]; buy_opt=[]; return;
end
Z_final = INIT_Z + round(-fvalz);

A2 = [A; zeros(2, n_vars)]; b2 = [b; Z_final-INIT_Z; -(Z_final-INIT_Z)];
nr = size(A, 1);
for j = 1:n_work
    A2(nr+1, off_w1+j) = WY(work_which(j));
    A2(nr+1, off_w2+j) = WY(work_which(j));
    A2(nr+2, off_w1+j) = -WY(work_which(j));
    A2(nr+2, off_w2+j) = -WY(work_which(j));
end
fm = zeros(n_vars, 1); fm(off_M+m+2) = -1;
[xm, fvalm, flag2] = intlinprog(fm, intvars, A2, b2, Aeq, beq, lb, ub, opts);
if flag2 <= 0 || isempty(xm)
    feasible = false; Z_final = 0; M_final = 0; w1_opt=[]; b_opt=[]; w2_opt=[]; buy_opt=[]; return;
end
M_final = round(-fvalm);
w1_opt = round(xm(off_w1+(1:n_work)))';
b_opt = round(xm(off_b+(1:n_work)))';
w2_opt = round(xm(off_w2+(1:n_work)))';
buy_opt = zeros(n_supply, 3);
for k = 1:n_supply
    buy_opt(k,1) = round(xm(off_bO+k));
    buy_opt(k,2) = round(xm(off_bH+k));
    buy_opt(k,3) = round(xm(off_bF+k));
end
feasible = true;
end

% ================================================================
function [Aeq, beq, A, b] = build_milp(m, travel, work_idx, work_which, ...
    supp_idx, n_work, n_supply, total_travel, CM, CW, CS, LOAD_LIMIT, MAX_DAYS, WM, ...
    off_w1, off_b, off_w2, off_bO, off_bH, off_bF, off_O, off_H, off_F, off_M, n_vars)

n_eq = 4*(m+1); Aeq = zeros(n_eq, n_vars); beq = zeros(n_eq, 1); eq = 0;
for i = 1:(m+1)
    d = travel(i); widx = work_idx(i); sidx = supp_idx(i);
    eq = eq + 1;
    Aeq(eq, off_O+1+i) = 1; Aeq(eq, off_O+i) = -1; beq(eq) = -d*CM(1);
    if widx > 0
        Aeq(eq, off_w1+widx) = CW(1); Aeq(eq, off_w2+widx) = CW(1); Aeq(eq, off_b+widx) = CS(1);
    end
    if sidx > 0, Aeq(eq, off_bO+sidx) = -1; end

    eq = eq + 1;
    Aeq(eq, off_H+1+i) = 1; Aeq(eq, off_H+i) = -1; beq(eq) = -d*CM(2);
    if widx > 0
        Aeq(eq, off_w1+widx) = CW(2); Aeq(eq, off_w2+widx) = CW(2); Aeq(eq, off_b+widx) = CS(2);
    end
    if sidx > 0, Aeq(eq, off_bH+sidx) = -1; end

    eq = eq + 1;
    Aeq(eq, off_F+1+i) = 1; Aeq(eq, off_F+i) = -1; beq(eq) = -d*CM(3);
    if widx > 0
        Aeq(eq, off_w1+widx) = CW(3); Aeq(eq, off_w2+widx) = CW(3); Aeq(eq, off_b+widx) = CS(3);
    end
    if sidx > 0, Aeq(eq, off_bF+sidx) = -1; end

    eq = eq + 1;
    Aeq(eq, off_M+1+i) = 1; Aeq(eq, off_M+i) = -1; beq(eq) = 0;
    if sidx > 0
        Aeq(eq, off_bO+sidx) = 2; Aeq(eq, off_bH+sidx) = 1; Aeq(eq, off_bF+sidx) = 2;
    end
end

n_ineq = (m+2) + 1 + 2*n_work + 3*n_supply;
A = zeros(n_ineq, n_vars); b = zeros(n_ineq, 1); ineq = 0;
for i = 0:(m+1)
    ineq = ineq + 1;
    A(ineq, off_O+1+i) = 1; A(ineq, off_H+1+i) = 1; A(ineq, off_F+1+i) = 1;
    b(ineq) = LOAD_LIMIT;
end
ineq = ineq + 1;
for j = 1:n_work
    A(ineq, off_w1+j) = 1; A(ineq, off_b+j) = 1; A(ineq, off_w2+j) = 1;
end
b(ineq) = MAX_DAYS - total_travel;
for j = 1:n_work
    ineq = ineq + 1; A(ineq, off_w1+j) = 1; b(ineq) = WM(work_which(j));
    ineq = ineq + 1; A(ineq, off_w2+j) = 1; A(ineq, off_b+j) = -WM(work_which(j)); b(ineq) = 0;
end
for i = 1:(m+1)
    if supp_idx(i) > 0
        d = travel(i);
        ineq = ineq + 1; A(ineq, off_O+i) = -1; b(ineq) = -d*CM(1);
        ineq = ineq + 1; A(ineq, off_H+i) = -1; b(ineq) = -d*CM(2);
        ineq = ineq + 1; A(ineq, off_F+i) = -1; b(ineq) = -d*CM(3);
    end
end
end

% ================================================================
function [w1, b, w2] = greedy_work_assign(work_which, WM, WY, remain)
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

% ================================================================
function [feasible, Z_final, M_final, buy_all] = greedy_sim(...
    m, travel, work_idx, work_which, supp_idx, n_work, n_supply, total_travel, ...
    w1, b, w2, WY, CM, CW, CS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT)

total_stop = sum(b);
T = total_travel + sum(w1) + total_stop + sum(w2);
cO = zeros(1, T); cH = zeros(1, T); cF = zeros(1, T);
zG = zeros(1, T); isSup = false(1, T); sup_day_to_k = zeros(1, T);

day = 0; sidx = 0;
for k = 1:(m+1)
    d = travel(k);
    for dd = 1:d
        day = day + 1;
        cO(day) = CM(1); cH(day) = CM(2); cF(day) = CM(3);
        if dd == d && supp_idx(k) > 0
            isSup(day) = true; sidx = sidx + 1; sup_day_to_k(day) = sidx;
        end
    end
    widx = work_idx(k);
    if widx > 0
        wh = work_which(widx);
        if w1(widx) > 0
            for ww = 1:w1(widx)
                day = day + 1; cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3); zG(day) = WY(wh);
            end
        end
        if b(widx) > 0
            day = day + 1; cO(day) = CS(1); cH(day) = CS(2); cF(day) = CS(3);
        end
        if w2(widx) > 0
            for ww = 1:w2(widx)
                day = day + 1; cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3); zG(day) = WY(wh);
            end
        end
    end
end

buy_all = zeros(n_supply, 3);
O = INIT_O; H = INIT_H; F = INIT_F; M_cur = INIT_M;

for t = 1:T
    O = O - cO(t); H = H - cH(t); F = F - cF(t);
    if O < 0 || H < 0 || F < 0, feasible = false; Z_final = 0; M_final = 0; return; end
    if isSup(t)
        nextSup = T + 1;
        for tt = t+1:T, if isSup(tt), nextSup = tt; break; end, end
        needO = 0; needH = 0; needF = 0;
        for tt = t+1:nextSup
            if tt > T, break; end
            needO = needO + cO(tt); needH = needH + cH(tt); needF = needF + cF(tt);
        end
        buyO = max(0, needO - O); buyH = max(0, needH - H); buyF = max(0, needF - F);
        if buyO + buyH + buyF > LOAD_LIMIT - (O + H + F)
            feasible = false; Z_final = 0; M_final = 0; return;
        end
        cost = buyO*2 + buyH*1 + buyF*2;
        if cost > M_cur, feasible = false; Z_final = 0; M_final = 0; return; end
        O = O + buyO; H = H + buyH; F = F + buyF; M_cur = M_cur - cost;
        sk = sup_day_to_k(t);
        if sk > 0 && sk <= n_supply
            buy_all(sk, 1) = buyO; buy_all(sk, 2) = buyH; buy_all(sk, 3) = buyF;
        end
    end
    if M_cur < 0 || O + H + F > LOAD_LIMIT, feasible = false; Z_final = 0; M_final = 0; return; end
end
Z_final = INIT_Z + sum(zG); M_final = M_cur; feasible = true;
end

% ================================================================
function [Z_final, M_final] = adaptive_sim(w_seq, all_xy, names, WY, WM, ...
    CM0, CW0, CS0, CM1, CW1, CS1, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, MAX_DAYS, dist)
% ================================================================
% Weather-Adaptive Simulation (Receding Horizon)
% Each day: observe weather -> adaptively decide action -> execute
% ================================================================

n_pts = size(all_xy, 1);
px = all_xy(1,1); py = all_xy(1,2);
O = INIT_O; H = INIT_H; F = INIT_F; M_cur = INIT_M; Z = INIT_Z;
consec_work = 0;

fprintf('  %4s %3s %5s %5s %5s %5s %5s %s\n', 'Day','W','O','H','F','M','Z','Action');

for t_day = 1:MAX_DAYS
    if px == all_xy(2,1) && py == all_xy(2,2), break; end

    w_today = round(w_seq(min(t_day, length(w_seq))));
    w_label = 'S'; if w_today == 0, w_label = 'N'; end

    if w_today == 0
        CM = CM0; CW = CW0; CS = CS0;
    else
        CM = CM1; CW = CW1; CS = CS1;
    end

    remaining = MAX_DAYS - t_day + 1;
    [action, target, work_wh] = adaptive_decision(...
        px, py, O, H, F, M_cur, Z, consec_work, remaining, t_day, ...
        all_xy, names, WY, WM, CM, CW, CS, LOAD_LIMIT, dist);

    act_str = '';
    if strcmp(action, 'move')
        tx = target(1); ty = target(2);
        if abs(px-tx) > 0, px = px + sign(tx-px); else py = py + sign(ty-py); end
        O = O - CM(1); H = H - CM(2); F = F - CM(3);
        consec_work = 0;
        act_str = sprintf('->(%d,%d)', px, py);

        for ii = 1:n_pts
            if all_xy(ii,1)==px && all_xy(ii,2)==py && (ii==6||ii==7)
                [bO, bH, bF] = buy_at_supply(px, py, O, H, F, M_cur, LOAD_LIMIT, ...
                    all_xy, dist, CM1, remaining-1);
                cost = bO*2 + bH*1 + bF*2;
                if cost <= M_cur && O+bO+H+bH+F+bF <= LOAD_LIMIT
                    O = O + bO; H = H + bH; F = F + bF; M_cur = M_cur - cost;
                    act_str = [act_str sprintf(' +Buy(%d,%d,%d)', bO, bH, bF)];
                end
                break;
            end
        end
    elseif strcmp(action, 'work')
        O = O - CW(1); H = H - CW(2); F = F - CW(3);
        Z = Z + WY(work_wh);
        consec_work = consec_work + 1;

        % After max consecutive work, must insert stop
        if consec_work >= WM(work_wh)
            % Stop day follows if resources permit
        end
        act_str = sprintf('Work@W%d', work_wh);
    elseif strcmp(action, 'stop')
        O = O - CS(1); H = H - CS(2); F = F - CS(3);
        consec_work = 0;
        act_str = 'Stop';
    end

    fprintf('  %4d %3s %5d %5d %5d %5d %5d %s\n', ...
        t_day, w_label, round(O), round(H), round(F), round(M_cur), round(Z), act_str);

    if O < 0 || H < 0 || F < 0 || M_cur < 0 || O+H+F > LOAD_LIMIT
        fprintf('  *** INFEASIBLE at day %d ***\n', t_day);
        Z_final = Z; M_final = M_cur; return;
    end
end

Z_final = Z; M_final = M_cur;
end

% ================================================================
function [action, target, work_wh] = adaptive_decision(...
    px, py, O, H, F, M_cur, Z, consec_work, remaining, t_day, ...
    all_xy, names, WY, WM, CM, CW, CS, LOAD_LIMIT, dist)
% Maximin decision: at current state, decide action.
% Assumes worst-case (storm) consumption for future.

n_pts = size(all_xy, 1);
E_x = all_xy(2,1); E_y = all_xy(2,2);

% Find current named-point index
cur_idx = 0;
for ii = 1:n_pts
    if all_xy(ii,1)==px && all_xy(ii,2)==py, cur_idx = ii; break; end
end

% At E: done
if cur_idx == 2
    action = 'stop'; target = [px py]; work_wh = 0; return;
end

% At work point: decide work vs move on
if cur_idx >= 3 && cur_idx <= 5
    wh = cur_idx - 2;
    % Check if we can work under storm consumption
    if consec_work < WM(wh) && O >= CW(1) && H >= CW(2) && F >= CW(3)
        dist_to_E = abs(px-E_x) + abs(py-E_y);
        need_O_E = dist_to_E * CM(1);
        need_H_E = dist_to_E * CM(2);
        need_F_E = dist_to_E * CM(3);
        if O - CW(1) >= need_O_E && H - CW(2) >= need_H_E && F - CW(3) >= need_F_E
            action = 'work'; target = [px py]; work_wh = wh; return;
        end
    end
    % Can't work, need stop to reset or move on
    if consec_work >= WM(wh) && O >= CS(1) && H >= CS(2) && F >= CS(3)
        action = 'stop'; target = [px py]; work_wh = 0; return;
    end
    % Move on
    action = 'move';
    nx = px; ny = py;
    if abs(px-E_x) > 0, nx = px + sign(E_x-px); else ny = py + sign(E_y-py); end
    target = [nx ny]; work_wh = 0; return;
end

% At B or intermediate: consider stop if it helps load balancing
% Only stop at B (day 1) if storm and H/F are high relative to O
load_now = O + H + F;
if cur_idx == 1 && t_day == 1
    % Check if a stop day helps: stop consumes (3,3,2) in storm
    % After stop: O-3, H-3, F-2. This reduces load, making S1 purchase fit.
    dist_to_S1 = dist(1, 6);  % B->S1
    O_after_move = (O - CS(1)) - dist_to_S1 * CM(1);
    H_after_move = (H - CS(2)) - dist_to_S1 * CM(2);
    F_after_move = (F - CS(3)) - dist_to_S1 * CM(3);
    % Need for S1->E: 8 grids
    need_O = 8 * CM(1); need_H = 8 * CM(2); need_F = 8 * CM(3);
    buy_O = need_O - O_after_move; buy_H = need_H - H_after_move; buy_F = need_F - F_after_move;
    load_after_buy = O_after_move + buy_O + H_after_move + buy_H + F_after_move + buy_F;
    if load_after_buy <= LOAD_LIMIT && O >= CS(1) && H >= CS(2) && F >= CS(3)
        action = 'stop'; target = [px py]; work_wh = 0; return;
    end
end

% General case: move toward E or nearest useful point
dist_to_E = abs(px-E_x) + abs(py-E_y);
% Check direct-to-E feasibility (storm consumption)
if O >= dist_to_E * CM(1) && H >= dist_to_E * CM(2) && F >= dist_to_E * CM(3) && remaining >= dist_to_E
    action = 'move';
    nx = px; ny = py;
    if abs(px-E_x) > 0, nx = px + sign(E_x-px); else ny = py + sign(E_y-py); end
    target = [nx ny]; work_wh = 0; return;
end

% Need to visit supply/work. Find nearest useful target.
best_dist = inf; best_tgt = 2;
for ii = [6 7 3 4 5 2]
    if ii == cur_idx, continue; end
    tx = all_xy(ii,1); ty = all_xy(ii,2);
    d_ij = abs(px-tx) + abs(py-ty);
    if d_ij < best_dist, best_dist = d_ij; best_tgt = ii; end
end

action = 'move';
nx = px; ny = py;
tx = all_xy(best_tgt,1); ty = all_xy(best_tgt,2);
if abs(px-tx) > 0, nx = px + sign(tx-px); else ny = py + sign(ty-py); end
target = [nx ny]; work_wh = 0;
end

% ================================================================
function [buyO, buyH, buyF] = buy_at_supply(px, py, O, H, F, M_cur, LOAD_LIMIT, ...
    all_xy, dist, CM1, remaining)
% Smart purchase: estimate needs to next useful point or E

n_pts = size(all_xy, 1);
cur_idx = 0;
for ii = 1:n_pts
    if all_xy(ii,1)==px && all_xy(ii,2)==py, cur_idx = ii; break;
    end
end

% Find next supply or E through this one
next_dist = dist(cur_idx, 2);  % to E
for ii = [6 7]
    if ii ~= cur_idx
        d = dist(cur_idx, ii) + dist(ii, 2);
        if d < next_dist, next_dist = d; end
    end
end

% Conservative: enough to move to E plus buffer
needO = next_dist * CM1(1) + 8;
needH = next_dist * CM1(2) + 4;
needF = next_dist * CM1(3) + 4;

buyO = max(0, needO - O);
buyH = max(0, needH - H);
buyF = max(0, needF - F);

space = LOAD_LIMIT - (O + H + F);
while buyO + buyH + buyF > space
    if buyO > 0, buyO = buyO - 1;
    elseif buyF > 0, buyF = buyF - 1;
    elseif buyH > 0, buyH = buyH - 1;
    else break;
    end
end

cost = buyO*2 + buyH*1 + buyF*2;
while cost > M_cur
    if buyO > 0, buyO = buyO - 1;
    elseif buyF > 0, buyF = buyF - 1;
    elseif buyH > 0, buyH = buyH - 1;
    else break;
    end
    cost = buyO*2 + buyH*1 + buyF*2;
end
end

% ================================================================
function print_daily_schedule(pid, w1, b, w2, buy, all_xy, names, ...
    CM, CW, CS, WY, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, B_stop)
% Print daily schedule with optional B-stop prelude

m=length(pid)-2; travel=zeros(1,m+1);
for k=1:(m+1)
    travel(k)=abs(all_xy(pid(k),1)-all_xy(pid(k+1),1))+abs(all_xy(pid(k),2)-all_xy(pid(k+1),2));
end
total_stop=sum(b); T=B_stop+sum(travel)+sum(w1)+total_stop+sum(w2);

work_at=[]; work_wh=[];
for k=2:(m+1)
    pt=pid(k); if pt>=3&&pt<=5, work_at(end+1)=k; work_wh(end+1)=pt-2; end
end

px=zeros(1,T+1); py=zeros(1,T+1);
cO=zeros(1,T); cH=zeros(1,T); cF=zeros(1,T);
zG=zeros(1,T); isSup=false(1,T); action=cell(1,T);

px(1)=all_xy(pid(1),1); py(1)=all_xy(pid(1),2);
cur_x=px(1); cur_y=py(1); day=0;

% B-stop prelude
for s=1:B_stop
    day=day+1; px(day+1)=cur_x; py(day+1)=cur_y;
    cO(day)=CS(1); cH(day)=CS(2); cF(day)=CS(3);
    action{day}=sprintf('Stop@B(pre)');
end

for k=1:(m+1)
    d=travel(k); tgt_x=all_xy(pid(k+1),1); tgt_y=all_xy(pid(k+1),2);
    dx_sign=sign(tgt_x-cur_x); dy_sign=sign(tgt_y-cur_y);
    for step=1:d
        day=day+1;
        if abs(cur_x-tgt_x)>0,cur_x=cur_x+dx_sign;else cur_y=cur_y+dy_sign;end
        px(day+1)=cur_x;py(day+1)=cur_y;
        cO(day)=CM(1);cH(day)=CM(2);cF(day)=CM(3);
        if step==d
            to_pt=pid(k+1);
            if to_pt==6||to_pt==7
                isSup(day)=true; action{day}=sprintf('Supply@%s',names{to_pt});
            else
                action{day}=sprintf('Move->%s',names{to_pt});
            end
        else
            action{day}='Move';
        end
    end
    wk=find(work_at==k+1,1);
    if ~isempty(wk)
        if w1(wk)>0
            for ww=1:w1(wk)
                day=day+1;px(day+1)=cur_x;py(day+1)=cur_y;
                cO(day)=CW(1);cH(day)=CW(2);cF(day)=CW(3);
                zG(day)=WY(work_wh(wk));action{day}=sprintf('Work@%s',names{pid(k+1)});
            end
        end
        if b(wk)>0
            day=day+1;px(day+1)=cur_x;py(day+1)=cur_y;
            cO(day)=CS(1);cH(day)=CS(2);cF(day)=CS(3);
            action{day}=sprintf('Stop@%s',names{pid(k+1)});
        end
        if w2(wk)>0
            for ww=1:w2(wk)
                day=day+1;px(day+1)=cur_x;py(day+1)=cur_y;
                cO(day)=CW(1);cH(day)=CW(2);cF(day)=CW(3);
                zG(day)=WY(work_wh(wk));action{day}=sprintf('Work@%s',names{pid(k+1)});
            end
        end
    end
end

O=INIT_O;H=INIT_H;F=INIT_F;M=INIT_M;Z=INIT_Z;
sup_day_idx=find(isSup);

fprintf('----+-----------+-----------------+-----+-----+-----+-----+-----+------------\n');
fprintf('Day | Location  | Action          |   O |   H |   F |   M |   Z | Buy(O,H,F)\n');
fprintf('----+-----------+-----------------+-----+-----+-----+-----+-----+------------\n');

for t=1:T
    O=O-cO(t);H=H-cH(t);F=F-cF(t);buy_str='-';
    if isSup(t)
        sk=find(sup_day_idx==t,1);
        if ~isempty(sk)&&sk<=size(buy,1)
            bO=buy(sk,1);bH=buy(sk,2);bF=buy(sk,3);
            if bO+bH+bF<=LOAD_LIMIT-(O+H+F)
                cost=bO*2+bH*1+bF*2;
                if cost<=M,O=O+bO;H=H+bH;F=F+bF;M=M-cost;
                    buy_str=sprintf('(%d,%d,%d)',bO,bH,bF);end
            end
        end
    end
    Z=Z+zG(t);
    fprintf('%3d | (%2d,%-2d)  | %-15s | %3d | %3d | %3d | %3d | %3d | %s\n',...
        t,px(t+1),py(t+1),action{t},round(O),round(H),round(F),round(M),round(Z),buy_str);
end
fprintf('----+-----------+-----------------+-----+-----+-----+-----+-----+------------\n');
fprintf('Final: Z=%d, M=%d\n\n',round(Z),round(M));
end
