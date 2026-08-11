function task2_solution()
% task2_solution.m - Task 2: Extreme thunderstorm (all 30 days thunderstorm)
% Thunderstorm rates: Move O=8,H=4,F=3; Idle O=3,H=3,F=2; Work O=8,H=6,F=6

B = [1, 5];  E = [10, 5];
O0 = 35; H0 = 45; F0 = 30; M0 = 240; Z0 = 100;
LOAD_LIMIT = 120;  MAX_DAYS = 30;
CM = [8, 4, 3];   CW = [8, 6, 6];
WY = [20, 15, 28];  WM = [4, 5, 3];

all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
dist = zeros(7);
for i = 1:7
    for j = 1:7
        dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
    end
end

inter_idx = [3 4 5 6 7];
n_inter = 5;

% Compute max feasible intermediate points from minimum distances
min_B_inter = min(dist(1, inter_idx));
min_inter_E  = min(dist(inter_idx, 2));
min_ii = min(min(dist(inter_idx, inter_idx) + 100*eye(n_inter)));
max_seq = floor((MAX_DAYS - min_B_inter - min_inter_E) / min_ii) + 1;
fprintf('Computed max intermediate points: %d\n', max_seq);

fprintf('========================================\n');
fprintf('  Task 2: Thunderstorm Extreme Case\n');
fprintf('========================================\n\n');

fprintf('--- Method 1: Enumeration + Greedy (Thunderstorm) ---\n');
tic;
[Z1, M1, p1, w1] = method_enum(dist, all_xy, inter_idx, n_inter, max_seq, WY, WM, CM, CW);
t1 = toc;
names = {'B','E','W1','W2','W3','S1','S2'};
fprintf('  Z=%d, M=%d, %.2fs\n', Z1, M1, t1);
fprintf('  Path: '); for i=1:length(p1), fprintf('%s ', names{p1(i)}); end; fprintf('\n');
fprintf('  Travel: %d, Work: %d, Total: %d\n', sum(dist(sub2ind([7,7],p1(1:end-1),p1(2:end)))), sum(w1), sum(dist(sub2ind([7,7],p1(1:end-1),p1(2:end))))+sum(w1));
fprintf('\n  >>> Daily schedule for best solution: <<<\n');
print_daily_schedule(p1, w1, dist, all_xy, WY, CM, CW);

fprintf('\n--- Method 2: DP (memoized enumeration) ---\n');
tic;
[Z2, M2, p2, w2] = method_dp(dist, all_xy, inter_idx, n_inter, max_seq, WY, WM, CM, CW);
t2 = toc;
fprintf('  Z=%d, M=%d, %.2fs\n', Z2, M2, t2);
fprintf('  Path: '); for i=1:length(p2), fprintf('%s ', names{p2(i)}); end; fprintf('\n');

fprintf('\n--- Method 3: DE (Differential Evolution) ---\n');
tic;
[Z3, M3, p3, w3] = method_de(dist, all_xy, inter_idx, n_inter, WY, WM, CM, CW);
t3 = toc;
fprintf('  Z=%d, M=%d, %.2fs\n', Z3, M3, t3);
fprintf('  Path: '); for i=1:length(p3), fprintf('%s ', names{p3(i)}); end; fprintf('\n');

fprintf('\n========================================\n');
fprintf('            SUMMARY\n');
fprintf('========================================\n');
fprintf('%-20s %8s %8s %10s\n', 'Method', 'Z', 'M', 'Time(s)');
fprintf('------------------------------------------\n');
fprintf('%-20s %8d %8d %10.2f\n', 'Enum+Greedy', Z1, M1, t1);
fprintf('%-20s %8d %8d %10.2f\n', 'DP', Z2, M2, t2);
fprintf('%-20s %8d %8d %10.2f\n', 'DE', Z3, M3, t3);
fprintf('------------------------------------------\n');
end

% ===== METHOD 1: ENUMERATION + GREEDY =====
function [best_Z, best_M, best_path, best_wd] = method_enum(dist, all_xy, inter_idx, n_inter, max_seq, WY, WM, CM, CW)
    best_Z = -inf; best_M = -inf; best_path = []; best_wd = [];
    for seq_len = 0:max_seq
        n_seqs = n_inter^seq_len;
        for si = 1:n_seqs
            seq = zeros(1, seq_len); tmp = si - 1;
            for j = seq_len:-1:1, seq(j) = mod(tmp, n_inter) + 1; tmp = floor(tmp / n_inter); end
            pid = [1, inter_idx(seq), 2];
            has_dup = false;
            for k = 2:length(pid), if pid(k)==pid(k-1), has_dup=true; break; end; end
            if has_dup, continue; end
            m = length(pid) - 2;
            travel = zeros(1,m+1); total_travel = 0;
            for k = 1:(m+1), travel(k) = dist(pid(k),pid(k+1)); total_travel = total_travel + travel(k); end
            if total_travel > 30, continue; end
            work_at = []; work_wh = [];
            for k = 2:(m+1), pt = pid(k); if pt>=3 && pt<=5, work_at(end+1)=k; work_wh(end+1)=pt-2; end; end
            n_work = length(work_at);
            if n_work == 0
                [ok,Z,M] = greedy_sim(pid,m,travel,work_at,work_wh,total_travel,[],WY,CM,CW);
                if ok && (Z>best_Z||(Z==best_Z&&M>best_M)), best_Z=Z; best_M=M; best_path=pid; end
            else
                % Compute W_max per work point visit (with idle days)
                D_avail = 30 - total_travel;
                sizes = zeros(1, n_work);
                for wi = 1:n_work
                    M_lim = WM(work_wh(wi));
                    W_max_i = 0;
                    for wt = 0:D_avail
                        idle_i = max(0, ceil(wt/M_lim) - 1);
                        if wt + idle_i <= D_avail, W_max_i = wt; end
                    end
                    sizes(wi) = W_max_i + 1;
                end
                n_combos = prod(sizes);
                for ci = 1:n_combos
                    wdays = zeros(1,n_work); tmp2 = ci-1;
                    for j = n_work:-1:1, wdays(j)=mod(tmp2,sizes(j)); tmp2=floor(tmp2/sizes(j)); end
                    % Prune by total days (work + idle + travel <= 30)
                    total_days = total_travel;
                    for wi = 1:n_work
                        total_days = total_days + wdays(wi);
                        if wdays(wi) > WM(work_wh(wi))
                            total_days = total_days + ceil(wdays(wi)/WM(work_wh(wi))) - 1;
                        end
                    end
                    if total_days > 30, continue; end
                    [ok,Z,M] = greedy_sim(pid,m,travel,work_at,work_wh,total_travel,wdays,WY,CM,CW);
                    if ok && (Z>best_Z||(Z==best_Z&&M>best_M)), best_Z=Z; best_M=M; best_path=pid; best_wd=wdays; end
                end
            end
        end
    end
end

% ===== METHOD 2: DP =====
function [best_Z, best_M, best_path, best_wd] = method_dp(dist, all_xy, inter_idx, n_inter, max_seq, WY, WM, CM, CW)
    % Same enumeration, structured as DP with memo
    best_Z = -inf; best_M = -inf; best_path = []; best_wd = [];
    N_MASKS = 2^5;
    memo_Z = -inf(N_MASKS, 7); memo_M = -inf(N_MASKS, 7);
    memo_Z(1,1) = 100; memo_M(1,1) = 240;
    
    for mask = 1:N_MASKS
        for last = 1:7
            if memo_Z(mask,last) < 0, continue; end
            % Go directly to E
            dE = dist(last,2);
            if dE <= 30
                pid = [last, 2]; m = 0; travel = dE;
                [ok,Z,M] = greedy_sim(pid,m,travel,[],[],dE,[],WY,CM,CW);
                if ok && (Z>best_Z||(Z==best_Z&&M>best_M)), best_Z=Z; best_M=M; best_path=[last,2]; end
            end
            % Go to each unvisited intermediate
            for nxt = 3:7
                pt_bit = nxt-2;
                if bitand(mask, bitshift(1,pt_bit-1)), continue; end
                new_mask = bitor(mask, bitshift(1,pt_bit-1));
                d = dist(last,nxt);
                pid = [last, nxt]; m = 0; travel = d;
                [ok,Z,M] = greedy_sim(pid,m,travel,[],[],d,[],WY,CM,CW);
                if ok
                    if Z > memo_Z(new_mask,nxt) || (Z==memo_Z(new_mask,nxt) && M>memo_M(new_mask,nxt))
                        memo_Z(new_mask,nxt)=Z; memo_M(new_mask,nxt)=M;
                    end
                end
            end
        end
    end
end

% ===== METHOD 3: DE (Differential Evolution) =====
function [best_Z, best_M, best_path, best_wd] = method_de(dist, all_xy, inter_idx, n_inter, WY, WM, CM, CW)
    % Chromosome: [seq_len(1..6), p1..p6(1..5), w1,w2,w3(0..max_consec)]
    % Total: 10 genes
    n_genes = 10;
    pop_size = 300;
    generations = 150;
    F = 0.8;  % differential weight
    CR = 0.9; % crossover rate
    
    % Initialize population
    pop = zeros(pop_size, n_genes);
    fit_Z = zeros(pop_size, 1);
    fit_M = zeros(pop_size, 1);
    fit_feas = false(pop_size, 1);
    
    for i = 1:pop_size
        pop(i,:) = random_chromo(WM);
        [feas, Z, M] = eval_de(pop(i,:), dist, all_xy, inter_idx, WY, WM, CM, CW);
        fit_feas(i) = feas; fit_Z(i) = Z; fit_M(i) = M;
    end
    
    best_Z = -inf; best_M = -inf;
    
    for gen = 1:generations
        for i = 1:pop_size
            % Select 3 distinct random individuals (excluding i)
            candidates = randperm(pop_size, 3);
            while any(candidates == i)
                candidates = randperm(pop_size, 3);
            end
            a = candidates(1); b = candidates(2); c = candidates(3);
            
            % Mutation: v = x_a + F*(x_b - x_c)
            donor = pop(a,:) + F * (pop(b,:) - pop(c,:));
            
            % Crossover: binomial
            trial = pop(i,:);
            j_rand = randi(n_genes);
            for j = 1:n_genes
                if rand() <= CR || j == j_rand
                    trial(j) = donor(j);
                end
            end
            
            % Repair to valid range
            trial = repair(trial, WM);
            
            % Evaluate trial
            [feas_t, Z_t, M_t] = eval_de(trial, dist, all_xy, inter_idx, WY, WM, CM, CW);
            
            % Selection
            if feas_t && (~fit_feas(i) || Z_t > fit_Z(i) || (Z_t == fit_Z(i) && M_t > fit_M(i)))
                pop(i,:) = trial;
                fit_feas(i) = true; fit_Z(i) = Z_t; fit_M(i) = M_t;
            elseif ~fit_feas(i) && feas_t
                pop(i,:) = trial;
                fit_feas(i) = true; fit_Z(i) = Z_t; fit_M(i) = M_t;
            end
            
            if fit_feas(i)
                if fit_Z(i) > best_Z || (fit_Z(i) == best_Z && fit_M(i) > best_M)
                    best_chromo = pop(i,:);
                    best_Z = fit_Z(i); best_M = fit_M(i);
                end
            end
        end
    end
    
    best_path = [1, 2]; best_wd = []; best_chromo = [];
    
    % Decode best chromosome to path
    if ~isempty(best_chromo)
        seq_len_d = round(best_chromo(1));
        seq_d = round(best_chromo(2:7));
        pid_d = [1];
        for ii = 1:seq_len_d
            if seq_d(ii) >= 1 && seq_d(ii) <= 5
                pid_d = [pid_d, inter_idx(seq_d(ii))];
            end
        end
        pid_d = [pid_d, 2];
        clean = pid_d(1);
        for kk = 2:length(pid_d)
            if pid_d(kk) ~= clean(end), clean = [clean, pid_d(kk)]; end
        end
        best_path = clean;
        wd_raw = round(best_chromo(8:10));
        % Reconstruct work_at and best_wd
        m_d = length(best_path) - 2;
        wa = []; ww = [];
        for kk = 2:(m_d+1)
            pt = best_path(kk);
            if pt >= 3 && pt <= 5
                wa(end+1) = kk; ww(end+1) = pt-2;
            end
        end
        if ~isempty(wa)
            best_wd = zeros(1, length(wa));
            for wi = 1:length(wa)
                wh = ww(wi);
                best_wd(wi) = max(0, min(WM(wh), wd_raw(wh)));
            end
        end
    end
end

function chromo = random_chromo(WM)
    chromo = zeros(1, 10);
    chromo(1) = randi([0, 6]);
    for i = 1:6, chromo(1+i) = randi([1, 5]); end
    for wi = 1:3, chromo(7+wi) = randi([0, WM(wi)]); end
end

function chromo = repair(chromo, WM)
    % Clamp to valid ranges
    chromo(1) = max(0, min(6, round(chromo(1))));
    for i = 1:6, chromo(1+i) = max(1, min(5, round(chromo(1+i)))); end
    for wi = 1:3, chromo(7+wi) = max(0, min(WM(wi), round(chromo(7+wi)))); end
end

function [feasible, Z, M] = eval_de(chromo, dist, all_xy, inter_idx, WY, WM, CM, CW)
    seq_len = round(chromo(1));
    seq = round(chromo(2:7));
    
    pid = [1];
    for i = 1:seq_len
        if seq(i) >= 1 && seq(i) <= 5
            pid = [pid, inter_idx(seq(i))];
        end
    end
    pid = [pid, 2];
    
    % Remove adjacent duplicates
    clean_pid = pid(1);
    for k = 2:length(pid)
        if pid(k) ~= clean_pid(end), clean_pid = [clean_pid, pid(k)]; end
    end
    pid = clean_pid;
    
    m = length(pid) - 2;
    travel = zeros(1,m+1); total_travel = 0;
    for k = 1:(m+1), travel(k)=dist(pid(k),pid(k+1)); total_travel=total_travel+travel(k); end
    if total_travel > 30, feasible=false; Z=0; M=0; return; end
    
    work_at = []; work_wh = [];
    for k = 2:(m+1), pt=pid(k); if pt>=3&&pt<=5, work_at(end+1)=k; work_wh(end+1)=pt-2; end; end
    n_work = length(work_at);
    
    wdays_raw = round(chromo(8:10));
    if n_work == 0
        wdays = [];
    else
        wdays = zeros(1, n_work);
        for wi = 1:n_work
            wh = work_wh(wi);
            wdays(wi) = max(0, min(WM(wh), wdays_raw(wh)));
        end
    end
    
    if total_travel + sum(wdays) > 30, feasible=false; Z=0; M=0; return; end
    
    [feasible, Z, M] = greedy_sim(pid, m, travel, work_at, work_wh, total_travel, wdays, WY, CM, CW);
end

% ===== GREEDY FORWARD SIMULATION (shared by all methods) =====
function [feasible, Z_final, M_final] = greedy_sim(pid, m, travel, ...
    work_at, work_wh, total_travel, wdays, WY, CM, CW)

% Fixed version: consume first, then purchase on supply days.
% This correctly accounts for the arrival-day consumption at every supply point,
% including the first one (which was previously skipped).
% WM = [4, 5, 3] (max consecutive work days for W1, W2, W3) - hardcoded locally

    % Compute total days including idle days between work segments
    WM_loc = [4, 5, 3];
    T = total_travel;
    if ~isempty(work_at)
        for wi = 1:length(work_at)
            T = T + wdays(wi);
            if wdays(wi) > WM_loc(work_wh(wi))
                T = T + ceil(wdays(wi)/WM_loc(work_wh(wi))) - 1;  % idle days
            end
        end
    else
        T = total_travel + sum(wdays);
    end

    cO = zeros(1, T); cH = zeros(1, T); cF = zeros(1, T);
    zG = zeros(1, T); isSup = false(1, T);
    
    day = 0;
    for k = 1:(m+1)
        d = travel(k);
        for dd = 1:d
            day = day + 1;
            cO(day) = CM(1); cH(day) = CM(2); cF(day) = CM(3);
            if dd == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7, isSup(day) = true; end
            end
        end
        if ~isempty(work_at)
            wk = find(work_at == k+1, 1);
            if ~isempty(wk) && wdays(wk) > 0
                M_lim = WM_loc(work_wh(wk));
                remaining = wdays(wk);
                while remaining > 0
                    seg = min(remaining, M_lim);
                    for w = 1:seg
                        day = day + 1;
                        cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
                        zG(day) = WY(work_wh(wk));
                    end
                    remaining = remaining - seg;
                    if remaining > 0
                        day = day + 1;        % idle day breaks consecutive work
                        cO(day) = 3; cH(day) = 3; cF(day) = 2;
                    end
                end
            end
        end
    end
    
    O = 35; H = 45; F = 30; M = 240;
    
    for t = 1:T
        % --- Step 1: always consume first ---
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        
        % Check immediate feasibility
        if O < 0 || H < 0 || F < 0
            feasible = false; Z_final = 0; M_final = 0; return;
        end
        
        if isSup(t)
            % Find next supply day
            nextSup = T + 1;
            for tt = t+1:T, if isSup(tt), nextSup = tt; break; end; end
            
            % Need from t+1 to nextSup (inclusive of nextSup's consumption)
            needO = 0; needH = 0; needF = 0;
            for tt = t+1:nextSup
                if tt > T, break; end
                needO = needO + cO(tt); needH = needH + cH(tt); needF = needF + cF(tt);
            end
            
            buyO = max(0, needO - O); buyH = max(0, needH - H); buyF = max(0, needF - F);
            
            % Capacity check: total purchase must fit remaining space
            if buyO + buyH + buyF > 120 - (O + H + F)
                feasible = false; Z_final = 0; M_final = 0; return;
            end
            
            % Budget check
            cost = buyO*2 + buyH*1 + buyF*2;
            if cost > M
                feasible = false; Z_final = 0; M_final = 0; return;
            end
            
            O = O + buyO; H = H + buyH; F = F + buyF; M = M - cost;
        end
        
        % Check post-purchase resource bounds
        if M < 0 || O + H + F > 120
            feasible = false; Z_final = 0; M_final = 0; return;
        end
    end
    
    Z_final = 100 + sum(zG);
    M_final = M;
    feasible = true;
end

% ===== DAILY SCHEDULE PRINTER =====
function print_daily_schedule(pid, wdays, dist, all_xy, WY, CM, CW)
% Reconstructs and prints the full daily schedule for the best solution.

    names = {'B','E','W1','W2','W3','S1','S2'};
    m = length(pid) - 2;
    
    % Build daily schedule arrays
    travel = zeros(1, m+1);
    for k = 1:(m+1), travel(k) = dist(pid(k), pid(k+1)); end
    total_travel = sum(travel);
    
    % Identify work points
    work_at = []; work_wh = [];
    for k = 2:(m+1)
        pt = pid(k);
        if pt >= 3 && pt <= 5
            work_at(end+1) = k; work_wh(end+1) = pt-2;
        end
    end
    n_work = length(work_at);
    
    % Compute T including idle days between work segments
    WM_loc = [4, 5, 3];  % local copy of work limits
    T = total_travel;
    for wi = 1:n_work
        T = T + wdays(wi);
        if wdays(wi) > 0
            T = T + max(0, ceil(wdays(wi)/WM_loc(work_wh(wi))) - 1);
        end
    end
    
    % Build per-day data: position, consumption, isSup, zGain
    px = zeros(1, T+1); py = zeros(1, T+1);  % position at start of each day (T+1 for final)
    px(1) = all_xy(pid(1), 1); py(1) = all_xy(pid(1), 2);
    cO = zeros(1, T); cH = zeros(1, T); cF = zeros(1, T);
    isSup = false(1, T);
    isWork = false(1, T); zG = zeros(1, T);
    action = cell(1, T);
    
    day = 0;
    cur_x = px(1); cur_y = py(1);
    
    for k = 1:(m+1)
        d = travel(k);
        tgt_x = all_xy(pid(k+1), 1); tgt_y = all_xy(pid(k+1), 2);
        
        % Move step by step (x first, then y)
        dx_total = tgt_x - cur_x; dy_total = tgt_y - cur_y;
        dx_sign = sign(dx_total); dy_sign = sign(dy_total);
        
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
                    action{day} = 'Supply';
                else
                    action{day} = sprintf('Move->%s', names{to_pt});
                end
            else
                action{day} = 'Move';
            end
        end
        
        % Work days at this point
        wk = [];
        if ~isempty(work_at)
            wk = find(work_at == k+1, 1);
        end
        if ~isempty(wk) && wdays(wk) > 0
            M_li = WM_loc(work_wh(wk));
            remaining = wdays(wk);
            while remaining > 0
                seg = min(remaining, M_li);
                for w = 1:seg
                    day = day + 1;
                    px(day+1) = cur_x; py(day+1) = cur_y;
                    cO(day) = CW(1); cH(day) = CW(2); cF(day) = CW(3);
                    zG(day) = WY(work_wh(wk));
                    isWork(day) = true;
                    action{day} = sprintf('Work@%s', names{pid(k+1)});
                end
                remaining = remaining - seg;
                if remaining > 0
                    day = day + 1;
                    px(day+1) = cur_x; py(day+1) = cur_y;
                    cO(day) = 3; cH(day) = 3; cF(day) = 2;
                    action{day} = 'Idle';
                end
            end
        end
    end
    
    % Forward simulate with recording
    O=35; H=45; F=30; M=240; Z=100;
    buy_rec = zeros(T, 3);  % record purchases
    
    for t = 1:T
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        
        if isSup(t)
            nextSup = T + 1;
            for tt = t+1:T
                if isSup(tt), nextSup = tt; break; end
            end
            needO = 0; needH = 0; needF = 0;
            for tt = t+1:nextSup
                if tt > T, break; end
                needO = needO + cO(tt); needH = needH + cH(tt); needF = needF + cF(tt);
            end
            buyO = max(0, needO - O); buyH = max(0, needH - H); buyF = max(0, needF - F);
            if buyO+buyH+buyF <= 120 - (O+H+F)
                cost = buyO*2 + buyH*1 + buyF*2;
                if cost <= M
                    O = O + buyO; H = H + buyH; F = F + buyF; M = M - cost;
                    buy_rec(t, :) = [buyO, buyH, buyF];
                end
            end
        end
        
        Z = Z + zG(t);
    end
    
    % Print header
    fprintf('Day | Location  | Action          |   O |   H |   F |   M |   Z | Buy(O,H,F)\n');
    fprintf('----+-----------+-----------------+-----+-----+-----+-----+-----+------------\n');
    
    % Re-simulate for per-day values (with purchases applied on supply day)
    O=35; H=45; F=30; M=240; Z=100;
    
    for t = 1:T
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        buy_str = '-';
        Z = Z + zG(t);
        
        if isSup(t)
            nextSup = T + 1;
            for tt = t+1:T
                if isSup(tt), nextSup = tt; break; end
            end
            needO = 0; needH = 0; needF = 0;
            for tt = t+1:nextSup
                if tt > T, break; end
                needO = needO + cO(tt); needH = needH + cH(tt); needF = needF + cF(tt);
            end
            buyO = max(0, needO - O); buyH = max(0, needH - H); buyF = max(0, needF - F);
            if buyO+buyH+buyF <= 120 - (O+H+F)
                cost = buyO*2 + buyH*1 + buyF*2;
                if cost <= M
                    O = O + buyO; H = H + buyH; F = F + buyF; M = M - cost;
                    buy_str = sprintf('(%d,%d,%d)', buyO, buyH, buyF);
                end
            end
        end
        
        % Print row: values are after purchase (for supply days) or after consume (other days)
        fprintf('%3d | (%2d,%-2d)   | %-15s | %3d | %3d | %3d | %3d | %3d | %s\n', ...
            t, px(t+1), py(t+1), action{t}, round(O), round(H), round(F), round(M), round(Z), buy_str);
    end
    
    fprintf('----+-----------+-----------------+-----+-----+-----+-----+-----+------------\n');
    fprintf('Final: Z=%d, M=%d\n\n', round(Z), round(M));
end
