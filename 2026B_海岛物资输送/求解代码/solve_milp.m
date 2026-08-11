function [best_Z, best_M, best_sol] = solve_milp()
% solve_milp - MILP: enumerate skeletons + intlinprog for joint optimization
global B E S W O0 H0 F0 M0 Z0 LOAD_LIMIT MAX_DAYS PRICE
global CONSUME_MOVE CONSUME_STAY CONSUME_WORK WORK_YIELD WORK_MAX_CONSEC
global INTERMEDIATE_PTS N_INTERMEDIATE manhattan

common_params;

all_pts = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
n_pts = 7;
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = manhattan(all_pts(i,:), all_pts(j,:));
    end
end

intermed_idx = [3 4 5 6 7];
n_inter = 5;

best_Z = -inf; best_M = -inf; best_sol = [];
total_checked = 0; feasible_count = 0;
max_seq_len = min(7, MAX_DAYS - dist(1,2));

fprintf('MILP: enumerating skeletons...\n');
tic;

for seq_len = 0:max_seq_len
    n_seqs = n_inter^seq_len;
    for si = 1:n_seqs
        seq = zeros(1, seq_len);
        temp = si - 1;
        for j = seq_len:-1:1
            seq(j) = mod(temp, n_inter) + 1;
            temp = floor(temp / n_inter);
        end
        
        path_idx = [1, intermed_idx(seq), 2];
        m = length(path_idx) - 2;
        
        travel = zeros(1, m+1);
        total_travel = 0;
        for k = 1:(m+1)
            travel(k) = dist(path_idx(k), path_idx(k+1));
            total_travel = total_travel + travel(k);
        end
        
        if total_travel > MAX_DAYS, continue; end
        
        work_at = []; work_which = []; supply_at = [];
        for k = 2:(m+1)
            pt = path_idx(k);
            if pt >= 3 && pt <= 5
                work_at = [work_at, k]; work_which = [work_which, pt-2];
            elseif pt >= 6
                supply_at = [supply_at, k];
            end
        end
        
        n_work = length(work_at); n_supply = length(supply_at);
        total_checked = total_checked + 1;
        
        [feas, Z, M, ~] = milp_skeleton(m, travel, work_at, work_which, ...
            supply_at, n_work, n_supply, total_travel);
        
        if feas
            feasible_count = feasible_count + 1;
            if Z > best_Z || (Z == best_Z && M > best_M)
                best_Z = Z; best_M = M;
                best_sol.path = path_idx;
                best_sol.Z = Z; best_sol.M = M;
            end
        end
    end
end

elapsed = toc;
fprintf('  Checked %d skeletons, %d feasible, %.2fs\n', total_checked, feasible_count, elapsed);
fprintf('  Best: Z=%d, M=%d\n', best_Z, best_M);
end

function [feasible, Z_opt, M_opt, w_opt] = milp_skeleton(m, travel, work_at, ...
    work_which, supply_at, n_work, n_supply, total_travel)

    global O0 H0 F0 M0 Z0 LOAD_LIMIT MAX_DAYS PRICE
    global CONSUME_MOVE CONSUME_STAY CONSUME_WORK WORK_YIELD WORK_MAX_CONSEC
    
    if m == 0
        d = travel(1);
        if O0 >= d*CONSUME_MOVE(1) && H0 >= d*CONSUME_MOVE(2) && F0 >= d*CONSUME_MOVE(3)
            feasible = true; Z_opt = Z0; M_opt = M0; w_opt = []; return;
        else
            feasible = false; Z_opt = 0; M_opt = 0; w_opt = []; return;
        end
    end
    
    n_vars = n_work + 3*n_supply + 4*(m+2);
    intvars = 1:n_work;
    
    off_w = 0;
    off_buyO = n_work;
    off_buyH = n_work + n_supply;
    off_buyF = n_work + 2*n_supply;
    off_O = n_work + 3*n_supply;
    off_H = off_O + (m+2);
    off_F = off_H + (m+2);
    off_M = off_F + (m+2);
    
    sup_map = zeros(1, m+2);
    for s = 1:n_supply, sup_map(supply_at(s)) = s; end
    wrk_map = zeros(1, m+2);
    for w = 1:n_work, wrk_map(work_at(w)) = w; end
    
    n_eq = 4*(m+1);
    Aeq = zeros(n_eq, n_vars);
    beq = zeros(n_eq, 1);
    eq_row = 0;
    
    for i = 1:(m+1)
        d = travel(i); to_pt = i + 1;
        
        eq_row = eq_row + 1;
        Aeq(eq_row, off_O + i) = 1; Aeq(eq_row, off_O + i - 1) = -1;
        beq(eq_row) = -d*CONSUME_MOVE(1);
        if wrk_map(to_pt) > 0
            Aeq(eq_row, off_w + wrk_map(to_pt)) = CONSUME_WORK(1);
        end
        if sup_map(to_pt) > 0
            Aeq(eq_row, off_buyO + sup_map(to_pt)) = -1;
        end
        
        eq_row = eq_row + 1;
        Aeq(eq_row, off_H + i) = 1; Aeq(eq_row, off_H + i - 1) = -1;
        beq(eq_row) = -d*CONSUME_MOVE(2);
        if wrk_map(to_pt) > 0
            Aeq(eq_row, off_w + wrk_map(to_pt)) = CONSUME_WORK(2);
        end
        if sup_map(to_pt) > 0
            Aeq(eq_row, off_buyH + sup_map(to_pt)) = -1;
        end
        
        eq_row = eq_row + 1;
        Aeq(eq_row, off_F + i) = 1; Aeq(eq_row, off_F + i - 1) = -1;
        beq(eq_row) = -d*CONSUME_MOVE(3);
        if wrk_map(to_pt) > 0
            Aeq(eq_row, off_w + wrk_map(to_pt)) = CONSUME_WORK(3);
        end
        if sup_map(to_pt) > 0
            Aeq(eq_row, off_buyF + sup_map(to_pt)) = -1;
        end
        
        eq_row = eq_row + 1;
        Aeq(eq_row, off_M + i) = 1; Aeq(eq_row, off_M + i - 1) = -1;
        beq(eq_row) = 0;
        if sup_map(to_pt) > 0
            s = sup_map(to_pt);
            Aeq(eq_row, off_buyO + s) = PRICE(1);
            Aeq(eq_row, off_buyH + s) = PRICE(2);
            Aeq(eq_row, off_buyF + s) = PRICE(3);
        end
    end
    
    n_ineq = 4*(m+2) + (m+2) + 1 + n_work;
    A = zeros(n_ineq, n_vars);
    b = zeros(n_ineq, 1);
    ineq_row = 0;
    
    for i = 0:(m+1)
        for r = 1:4
            ineq_row = ineq_row + 1;
            A(ineq_row, off_O + (r-1)*(m+2) + i) = -1;
        end
    end
    for i = 0:(m+1)
        ineq_row = ineq_row + 1;
        A(ineq_row, off_O + i) = 1; A(ineq_row, off_H + i) = 1; A(ineq_row, off_F + i) = 1;
        b(ineq_row) = LOAD_LIMIT;
    end
    
    ineq_row = ineq_row + 1;
    for w = 1:n_work, A(ineq_row, off_w + w) = 1; end
    b(ineq_row) = MAX_DAYS - total_travel;
    
    for w = 1:n_work
        ineq_row = ineq_row + 1;
        A(ineq_row, off_w + w) = 1;
        b(ineq_row) = WORK_MAX_CONSEC(work_which(w));
    end
    
    lb = zeros(n_vars, 1); ub = inf(n_vars, 1);
    lb(off_O + 1) = O0; ub(off_O + 1) = O0;
    lb(off_H + 1) = H0; ub(off_H + 1) = H0;
    lb(off_F + 1) = F0; ub(off_F + 1) = F0;
    lb(off_M + 1) = M0; ub(off_M + 1) = M0;
    for w = 1:n_work, ub(off_w + w) = WORK_MAX_CONSEC(work_which(w)); end
    
    % Phase 1: maximize Z
    f1 = zeros(n_vars, 1);
    for w = 1:n_work, f1(off_w + w) = -WORK_YIELD(work_which(w)); end
    
    [x1, fval1, exitflag1] = intlinprog(f1, intvars, A, b, Aeq, beq, lb, ub);
    
    if exitflag1 ~= 1
        feasible = false; Z_opt = 0; M_opt = 0; w_opt = []; return;
    end
    
    Z_opt = Z0 + round(-fval1);
    w_opt = round(x1(off_w + (1:n_work)));
    
    % Phase 2: fix Z, maximize M
    A2 = [A; zeros(2, n_vars)];
    b2 = [b; Z_opt - Z0; -(Z_opt - Z0)];
    for w = 1:n_work
        A2(end-1, off_w + w) = WORK_YIELD(work_which(w));
        A2(end, off_w + w) = -WORK_YIELD(work_which(w));
    end
    
    f2 = zeros(n_vars, 1);
    f2(off_M + m + 1) = -1;
    
    [x2, fval2, exitflag2] = intlinprog(f2, intvars, A2, b2, Aeq, beq, lb, ub);
    
    if exitflag2 ~= 1
        feasible = false; Z_opt = 0; M_opt = 0; w_opt = []; return;
    end
    
    M_opt = round(-fval2);
    feasible = true;
end
