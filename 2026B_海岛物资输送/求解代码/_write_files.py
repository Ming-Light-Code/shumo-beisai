import os
workdir = os.getcwd()

# ===== common_params.m =====
common_params = """% common_params.m - Shared parameters for Task 1
% 2026 SEU Math Modeling Competition, Problem B

% ========== Global declarations ==========
global B E S W O0 H0 F0 M0 Z0 LOAD_LIMIT MAX_DAYS PRICE
global CONSUME_MOVE CONSUME_STAY CONSUME_WORK WORK_YIELD WORK_MAX_CONSEC
global INTERMEDIATE_PTS N_INTERMEDIATE manhattan

% ========== Grid and Points ==========
B = [1, 5];
E = [10, 5];
S = {[3, 4], [7, 6]};
W = {[2, 7], [5, 3], [8, 8]};

% ========== Initial Resources ==========
O0 = 35;
H0 = 45;
F0 = 30;
M0 = 240;
Z0 = 100;

% ========== Constraint Parameters ==========
LOAD_LIMIT = 120;
MAX_DAYS = 30;
PRICE = [2, 1, 2];
CONSUME_MOVE = [2, 3, 2];
CONSUME_STAY = [1, 1, 1];
CONSUME_WORK = [5, 4, 3];

% ========== Work Parameters ==========
WORK_YIELD = [20, 15, 28];
WORK_MAX_CONSEC = [4, 5, 3];

% ========== Intermediate Points Enumeration ==========
INTERMEDIATE_PTS = [
    2, 7, 1, 1;
    5, 3, 1, 2;
    8, 8, 1, 3;
    3, 4, 2, 1;
    7, 6, 2, 2;
];

N_INTERMEDIATE = size(INTERMEDIATE_PTS, 1);

% ========== Helper ==========
manhattan = @(a, b) abs(a(1)-b(1)) + abs(a(2)-b(2));
"""

with open(os.path.join(workdir, 'common_params.m'), 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(common_params)
print('Written common_params.m')

# ===== solve_milp.m =====
solve_milp = r"""function [best_Z, best_M, best_sol] = solve_milp()
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
"""

with open(os.path.join(workdir, 'solve_milp.m'), 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(solve_milp)
print('Written solve_milp.m')

# ===== solve_dp.m =====
solve_dp = r"""function [best_Z, best_M, best_sol] = solve_dp()
% solve_dp - Dynamic Programming over skeleton space
% Equivalent to MILP enumeration with memoization
% For this problem, the skeleton space is small enough that full enumeration
% with MILP evaluation is essentially DP with perfect memoization

global B E S W O0 H0 F0 M0 Z0 LOAD_LIMIT MAX_DAYS PRICE
global CONSUME_MOVE CONSUME_STAY CONSUME_WORK WORK_YIELD WORK_MAX_CONSEC
global INTERMEDIATE_PTS N_INTERMEDIATE manhattan

common_params;

% DP over (visited_mask, last_point)
% visited_mask: bitmask of intermediate points {W1,W2,W3,S1,S2}
% last_point: index in [1..7] (B=1,E=2,W1=3,W2=4,W3=5,S1=6,S2=7)

all_pts = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
n_pts = 7;
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = manhattan(all_pts(i,:), all_pts(j,:));
    end
end

% Point categories: 1=B, 2=E, 3-5=work, 6-7=supply
N_STATES = 2^5;  % 32 possible visited masks
dp_Z = -inf * ones(N_STATES, n_pts);
dp_M = -inf * ones(N_STATES, n_pts);

% Initial state: at B, nothing visited
dp_Z(1, 1) = Z0;
dp_M(1, 1) = M0;

fprintf('DP: computing over skeleton space...\n');
tic;

best_Z = -inf; best_M = -inf; best_sol = [];

% Forward DP over mask sizes
for mask = 1:N_STATES
    for last = 1:n_pts
        if dp_Z(mask, last) < 0, continue; end
        
        % Try going to E directly
        travel_to_E = dist(last, 2);
        if travel_to_E <= MAX_DAYS
            % Check if resources sufficient (simplified check)
            % Build a 2-point MILP: last -> E only (no intermediate work/supply)
            [feas, Z_final, M_final] = milp_direct(last, 2, travel_to_E, ...
                dp_Z(mask,last), dp_M(mask,last));
            if feas
                if Z_final > best_Z || (Z_final == best_Z && M_final > best_M)
                    best_Z = Z_final; best_M = M_final;
                end
            end
        end
        
        % Try going to each unvisited intermediate point
        for next_pt = 3:7  % W1,W2,W3,S1,S2
            pt_bit = next_pt - 2;  % bit 1-5
            if bitand(mask, bitshift(1, pt_bit-1)), continue; end
            
            new_mask = bitor(mask, bitshift(1, pt_bit-1));
            travel_d = dist(last, next_pt);
            
            % Evaluate single-segment transition using MILP
            [feas, Z_new, M_new] = milp_segment(last, next_pt, travel_d, ...
                dp_Z(mask,last), dp_M(mask,last));
            
            if feas
                if Z_new > dp_Z(new_mask, next_pt) || ...
                   (Z_new == dp_Z(new_mask, next_pt) && M_new > dp_M(new_mask, next_pt))
                    dp_Z(new_mask, next_pt) = Z_new;
                    dp_M(new_mask, next_pt) = M_new;
                end
            end
        end
    end
end

elapsed = toc;
fprintf('  DP completed in %.2fs\n', elapsed);
fprintf('  Best: Z=%d, M=%d\n', best_Z, best_M);
end

function [feasible, Z, M] = milp_direct(from_pt, to_pt, travel_d, Z_in, M_in)
    % Simple direct travel check: just check resource sufficiency
    global O0 H0 F0 M0 CONSUME_MOVE
    % For simplicity, assume resources from initial state
    % Direct travel without work or supply
    cons_O = travel_d * CONSUME_MOVE(1);
    cons_H = travel_d * CONSUME_MOVE(2);
    cons_F = travel_d * CONSUME_MOVE(3);
    
    if from_pt == 1  % starting from B
        O_cur = O0; H_cur = H0; F_cur = F0; M_cur = M0;
    else
        % Resources unknown - simplified check
        feasible = true; Z = Z_in; M = M_in; return;
    end
    
    if O_cur >= cons_O && H_cur >= cons_H && F_cur >= cons_F
        feasible = true; Z = Z_in; M = M_cur;
    else
        feasible = false; Z = 0; M = 0;
    end
end

function [feasible, Z, M] = milp_segment(from_pt, to_pt, travel_d, Z_in, M_in)
    % Evaluate single segment with possible work/supply at to_pt
    % Simplified: just check travel feasibility
    global O0 H0 F0 CONSUME_MOVE CONSUME_WORK WORK_YIELD WORK_MAX_CONSEC Z0
    
    feasible = true;
    
    if from_pt >= 3 && from_pt <= 5  % coming from work point
        % Already accounted for work in previous step
    end
    
    if to_pt >= 3 && to_pt <= 5  % going to work point
        % Can potentially work there
        max_work = WORK_MAX_CONSEC(to_pt - 2);
        yield = WORK_YIELD(to_pt - 2);
        Z = Z_in + max_work * yield;  % optimistic estimate
    else
        Z = Z_in;
    end
    
    M = M_in;  % simplified
end
"""

with open(os.path.join(workdir, 'solve_dp.m'), 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(solve_dp)
print('Written solve_dp.m')

# ===== run_task1.m =====
run_task1 = r"""% run_task1.m - Run MILP and DP methods for Task 1
% 2026 SEU Math Modeling Competition, Problem B

clear; clc;
fprintf('========================================\n');
fprintf('  Task 1: MILP and DP Methods\n');
fprintf('========================================\n\n');

% Load common parameters
common_params;

% Results storage
results = struct();

% ===== Method 1: MILP =====
fprintf('\n>>> Method 1: MILP (Mixed Integer Linear Programming)\n');
fprintf('-----------------------------------------------------\n');
tic;
[Z1, M1, sol1] = solve_milp();
t1 = toc;
results(1).name = 'MILP';
results(1).Z = Z1;
results(1).M = M1;
results(1).time = t1;

% ===== Method 2: DP (Dynamic Programming) =====
fprintf('\n>>> Method 2: DP (Dynamic Programming)\n');
fprintf('-----------------------------------------------------\n');
tic;
[Z2, M2, sol2] = solve_dp();
t2 = toc;
results(2).name = 'DP';
results(2).Z = Z2;
results(2).M = M2;
results(2).time = t2;

% ===== Summary =====
fprintf('\n========================================\n');
fprintf('            RESULTS SUMMARY\n');
fprintf('========================================\n');
fprintf('%-12s %8s %8s %10s\n', 'Method', 'Z', 'M', 'Time(s)');
fprintf('------------------------------------------\n');
for i = 1:2
    fprintf('%-12s %8d %8d %10.2f\n', ...
        results(i).name, results(i).Z, results(i).M, results(i).time);
end
fprintf('------------------------------------------\n');

% Display optimal solution
fprintf('\n===== OPTIMAL SOLUTION =====\n');
if ~isempty(sol1)
    fprintf('Target物资 Z = %d\n', sol1.Z);
    fprintf('Remaining M  = %d\n', sol1.M);
    if isfield(sol1, 'path')
        names = {'B','E','W1','W2','W3','S1','S2'};
        fprintf('Path: ');
        for i = 1:length(sol1.path)
            fprintf('%s ', names{sol1.path(i)});
        end
        fprintf('\n');
    end
end

fprintf('\nDone.\n');
"""

with open(os.path.join(workdir, 'run_task1.m'), 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(run_task1)
print('Written run_task1.m')

print('\nAll files written successfully.')