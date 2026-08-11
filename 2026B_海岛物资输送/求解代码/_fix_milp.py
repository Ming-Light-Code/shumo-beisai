import os
workdir = os.path.join(os.environ['USERPROFILE'], 'Desktop', '新建文件夹')

code = """function [best_Z, best_M, best_sol] = solve_milp()
% solve_milp - Enumerate skeletons, greedy forward simulation
% No toolbox dependencies

global B E S W O0 H0 F0 M0 Z0 LOAD_LIMIT MAX_DAYS PRICE
global CONSUME_MOVE CONSUME_STAY CONSUME_WORK WORK_YIELD WORK_MAX_CONSEC

all_pts = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
n_pts = 7;
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = abs(all_pts(i,1)-all_pts(j,1)) + abs(all_pts(i,2)-all_pts(j,2));
    end
end

intermed_idx = [3 4 5 6 7];
n_inter = 5;

best_Z = -inf; best_M = -inf; best_sol = [];
total_checked = 0; feasible_count = 0;
max_seq_len = min(7, MAX_DAYS - dist(1,2));

fprintf('MILP: enumerating skeletons...\\n');
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
        
        total_checked = total_checked + 1;
        [feas, Z, M] = eval_skel(m, travel, path_idx, work_at, work_which, total_travel);
        
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
fprintf('  Checked %d skeletons, %d feasible, %.2fs\\n', total_checked, feasible_count, elapsed);
fprintf('  Best: Z=%d, M=%d\\n', best_Z, best_M);
end

function [feasible, Z_best, M_best] = eval_skel(m, travel, path_idx, ...
    work_at, work_which, total_travel)

    global MAX_DAYS WORK_MAX_CONSEC WORK_YIELD
    
    Z_best = -inf; M_best = -inf;
    feasible = false;
    n_work = length(work_at);
    
    if n_work == 0
        [feas, Z, M] = greedy_sim(m, travel, path_idx, work_at, ...
            work_which, total_travel, zeros(1,0));
        if feas
            Z_best = Z; M_best = M; feasible = true;
        end
        return;
    end
    
    max_days = WORK_MAX_CONSEC(work_which);
    sizes = max_days + 1;
    total_combos = prod(sizes);
    
    for ci = 1:total_combos
        wdays = zeros(1, n_work);
        temp = ci - 1;
        for j = n_work:-1:1
            wdays(j) = mod(temp, sizes(j));
            temp = floor(temp / sizes(j));
        end
        
        if total_travel + sum(wdays) > MAX_DAYS, continue; end
        
        [feas, Z, M] = greedy_sim(m, travel, path_idx, work_at, ...
            work_which, total_travel, wdays);
        
        if feas
            feasible = true;
            if Z > Z_best || (Z == Z_best && M > M_best)
                Z_best = Z; M_best = M;
            end
        end
    end
end

function [feasible, Z_final, M_final] = greedy_sim(m, travel, path_idx, ...
    work_at, work_which, total_travel, wdays)

    global O0 H0 F0 M0 Z0 LOAD_LIMIT PRICE
    global CONSUME_MOVE CONSUME_WORK WORK_YIELD
    
    T = total_travel + sum(wdays);
    consume_O = zeros(1, T); consume_H = zeros(1, T); consume_F = zeros(1, T);
    z_gain = zeros(1, T); is_supply = false(1, T);
    
    day = 0;
    for k = 1:(m+1)
        d = travel(k);
        for dd = 1:d
            day = day + 1;
            consume_O(day) = CONSUME_MOVE(1);
            consume_H(day) = CONSUME_MOVE(2);
            consume_F(day) = CONSUME_MOVE(3);
            if dd == d
                to_pt = path_idx(k+1);
                if to_pt == 6 || to_pt == 7
                    is_supply(day) = true;
                end
            end
        end
        
        wk_pos = find(work_at == k+1, 1);
        if ~isempty(wk_pos) && ~isempty(wdays) && wdays(wk_pos) > 0
            nw = wdays(wk_pos);
            yield = WORK_YIELD(work_which(wk_pos));
            for w = 1:nw
                day = day + 1;
                consume_O(day) = CONSUME_WORK(1);
                consume_H(day) = CONSUME_WORK(2);
                consume_F(day) = CONSUME_WORK(3);
                z_gain(day) = yield;
            end
        end
    end
    
    O_cur = O0; H_cur = H0; F_cur = F0; M_cur = M0;
    
    for t = 1:T
        if is_supply(t)
            O_cur = O_cur - consume_O(t);
            H_cur = H_cur - consume_H(t);
            F_cur = F_cur - consume_F(t);
            
            if O_cur < 0 || H_cur < 0 || F_cur < 0
                feasible = false; Z_final = 0; M_final = 0; return;
            end
            
            next_sup = T + 1;
            for tt = t+1:T
                if is_supply(tt), next_sup = tt; break; end
            end
            
            need_O = 0; need_H = 0; need_F = 0;
            for tt = t+1:(next_sup-1)
                if tt > T, break; end
                need_O = need_O + consume_O(tt);
                need_H = need_H + consume_H(tt);
                need_F = need_F + consume_F(tt);
            end
            
            space = LOAD_LIMIT - (O_cur + H_cur + F_cur);
            buy_O = max(0, need_O - O_cur);
            buy_H = max(0, need_H - H_cur);
            buy_F = max(0, need_F - F_cur);
            
            if buy_O + buy_H + buy_F > space
                total_req = buy_O + buy_H + buy_F;
                if total_req > 0 && space > 0
                    scale = space / total_req;
                    buy_O = floor(buy_O * scale);
                    buy_H = floor(buy_H * scale);
                    buy_F = floor(buy_F * scale);
                end
            end
            
            if next_sup > T
                if O_cur + buy_O < need_O || H_cur + buy_H < need_H || F_cur + buy_F < need_F
                    feasible = false; Z_final = 0; M_final = 0; return;
                end
            end
            
            cost = buy_O*PRICE(1) + buy_H*PRICE(2) + buy_F*PRICE(3);
            if cost > M_cur
                feasible = false; Z_final = 0; M_final = 0; return;
            end
            
            O_cur = O_cur + buy_O;
            H_cur = H_cur + buy_H;
            F_cur = F_cur + buy_F;
            M_cur = M_cur - cost;
        else
            O_cur = O_cur - consume_O(t);
            H_cur = H_cur - consume_H(t);
            F_cur = F_cur - consume_F(t);
        end
        
        if O_cur < 0 || H_cur < 0 || F_cur < 0 || M_cur < 0
            feasible = false; Z_final = 0; M_final = 0; return;
        end
        if O_cur + H_cur + F_cur > LOAD_LIMIT
            feasible = false; Z_final = 0; M_final = 0; return;
        end
    end
    
    Z_final = Z0 + sum(z_gain);
    M_final = M_cur;
    feasible = true;
end
"""

fpath = os.path.join(workdir, 'solve_milp.m')
with open(fpath, 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(code)
print(f'Written solve_milp.m ({len(code)} chars)')