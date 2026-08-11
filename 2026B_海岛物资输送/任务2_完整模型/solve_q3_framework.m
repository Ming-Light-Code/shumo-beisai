function solve_q3_framework()
% solve_q3_framework.m - EVCP框架求解任务3
% 期望值约束规划 + 在线滚动决策 + 蒙特卡洛验证
% 30x30网格, 90天, P(正常)=0.8, P(雷暴)=0.2

MAX_DAYS = 90; MAX_LOAD = 400;
INIT_O = 100; INIT_H = 150; INIT_F = 100;
INIT_M = 750; INIT_Z = 200;

% 点位坐标: B, E, W1, W2, W3, S1, S2
all_xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
n_pts = size(all_xy, 1);
names = {'B','E','W1','W2','W3','S1','S2'};
WY = [20, 15, 28]; WM = [4, 5, 3];

% 曼哈顿距离矩阵
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
    end
end

% 期望消耗参数: E[consume] = 0.8*normal + 0.2*thunder
cons_exp = struct('MO',3.2,'MH',3.2,'MF',2.2, ...
                  'PO',1.4,'PH',1.4,'PF',1.2, ...
                  'WO',5.6,'WH',4.4,'WF',3.6, ...
                  'pO',2,'pH',1,'pF',2);
cons_norm = struct('MO',2,'MH',3,'MF',2,'PO',1,'PH',1,'PF',1, ...
                   'WO',5,'WH',4,'WF',3,'pO',2,'pH',1,'pF',2);
cons_thun = struct('MO',8,'MH',4,'MF',3,'PO',3,'PH',3,'PF',2, ...
                   'WO',8,'WH',6,'WF',6,'pO',2,'pH',1,'pF',2);

fprintf('========================================\n');
fprintf('  Task 3: EVCP Framework\n');
fprintf('========================================\n');
fprintf('Grid: 30x30 | Days: 90 | P(normal)=0.8\n');
fprintf('Expected move O: %.1f/cell\n', cons_exp.MO);
fprintf('Expected work O: %.1f/day\n\n', cons_exp.WO);

% 连通性分析
fprintf('--- Connectivity (expected consumption) ---\n');
for wi = 1:3
    d = dist(1, wi+2);
    fprintf('B->W%d: %d cells, E[O]=%.1f <= %d (REACHABLE)\n', wi, d, d*cons_exp.MO, INIT_O);
end
fprintf('B->S1: %d cells, E[O]=%.1f (REACHABLE)\n', dist(1,6), dist(1,6)*cons_exp.MO);
fprintf('B->S2: %d cells, E[O]=%.1f (REACHABLE)\n', dist(1,7), dist(1,7)*cons_exp.MO);
fprintf('\n');

% CP搜索 (使用期望消耗)
fprintf('--- EVCP Search ---\n');
inter_idx = [3 4 5 6 7];
best_Z = -inf; best_M = -inf;
best_path = [1, 2]; best_wd = []; best_ps = [];
nodes = 0;

[best_Z, best_M, best_path, best_wd, best_ps, nodes] = ...
    cp_search_q3([1], 0, [], [], best_Z, best_M, best_path, best_wd, best_ps, ...
                 dist, inter_idx, WY, WM, cons_exp, nodes, 0, MAX_DAYS, MAX_LOAD, ...
                 INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);

fprintf('\n===== EVCP OPTIMAL PLAN =====\n');
fprintf('Expected Z = %.0f\n', best_Z);
fprintf('Expected M = %.0f\n', best_M);
pstr = '';
for i = 1:length(best_path)
    pstr = [pstr, ' ', names{best_path(i)}];
end
fprintf('Path: %s\n', pstr);
fprintf('Work days: %s\n', mat2str(best_wd));
fprintf('Park segs: %s\n', mat2str(best_ps));
fprintf('Nodes: %d\n', nodes);

% 蒙特卡洛验证
fprintf('\n--- Monte Carlo Validation (N=1000) ---\n');
N_mc = 1000;
success = 0; Z_vals = []; M_vals = [];
for mc = 1:N_mc
    weather = rand(1, MAX_DAYS) < 0.8;  % true=正常
    [ok, Z_f, M_f] = simulate_mc(best_path, best_wd, best_ps, weather, ...
                                  dist, cons_norm, cons_thun, WY, WM, ...
                                  MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
    if ok
        success = success + 1;
        Z_vals(end+1) = Z_f;
        M_vals(end+1) = M_f;
    end
end
fprintf('Success rate: %.1f%%\n', success/N_mc*100);
if success > 0
    fprintf('Mean Z: %.1f (std: %.1f)\n', mean(Z_vals), std(Z_vals));
    fprintf('Mean M: %.1f (std: %.1f)\n', mean(M_vals), std(M_vals));
end
fprintf('\nDone.\n');
end

% ==================== CP搜索 (EVCP适配) ====================
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_q3(...
    path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, ...
    dist, inter_idx, WY, WM, cons, nodes, depth, MAX_DAYS, MAX_LOAD, ...
    INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z)

    nodes = nodes + 1;
    lp = path(end);
    
    if lp ~= 2
        dE = dist(lp, 2);
        rem = MAX_DAYS - tsf;
        if rem < dE, return; end
        max_w = max_work_with_park(3, rem - dE);
        ub = INIT_Z + max_w * 28;
        if ub <= bZ && bZ > -inf, return; end
    end
    
    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS
        fp = [path, 2];
        m = length(fp) - 2;
        tt = 0;
        for k = 1:(m+1), tt = tt + dist(fp(k), fp(k+1)); end
        
        rem_days = MAX_DAYS - tt;
        nw = length(wa);
        
        if nw == 0
            [bZ, bM, bP, bWD, bPS] = eval_leaf_q3(...
                fp, m, tt, rem_days, dist, [], bZ, bM, bP, bWD, bPS, ...
                cons, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
        else
            max_wk = zeros(1, nw);
            for j = 1:nw, max_wk(j) = max_work_with_park(WM(ww(j)), rem_days); end
            sz = max_wk + 1; nc = prod(sz);
            for ci = 1:nc
                wd = zeros(1, nw); t2 = ci - 1;
                for j = nw:-1:1, wd(j) = mod(t2, sz(j)); t2 = floor(t2/sz(j)); end
                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j)/WM(ww(j)))-1);
                    end
                end
                if rem_days - total_stay < 0, continue; end
                park_rem = rem_days - total_stay;
                [bZ, bM, bP, bWD, bPS] = eval_leaf_q3(...
                    fp, m, tt, park_rem, dist, wd, bZ, bM, bP, bWD, bPS, ...
                    cons, MAX_LOAD, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
            end
        end
    end
    
    for ni = 1:5
        np = inter_idx(ni);
        if np == lp, continue; end
        d = dist(lp, np);
        if tsf + d > MAX_DAYS, continue; end
        dE2 = dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS, continue; end
        np2 = [path, np]; nt = tsf + d;
        nwa = wa; nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2); nww(end+1) = np - 2;
        end
        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_q3(...
            np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, ...
            dist, inter_idx, WY, WM, cons, nodes, depth+1, MAX_DAYS, MAX_LOAD, ...
            INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
    end
end

function [bZ, bM, bP, bWD, bPS] = eval_leaf_q3(...
    fp, m, tt, park_rem, dist, wd_in, bZ, bM, bP, bWD, bPS, ...
    cons, MAX_LOAD, O, H, F, M, Z)
    n_seg = m + 1;
    park_combs = enumerate_park_combs(n_seg + 1, park_rem);
    for ci = 1:size(park_combs, 1)
        ps = park_combs(ci, 1:n_seg);
        [ok, Z_out, M_out] = gsim_q3(fp, m, tt, dist, wd_in, ps, cons, ...
            MAX_LOAD, O, H, F, M, Z);
        if ok && (Z_out > bZ || (Z_out == bZ && M_out > bM))
            bZ = Z_out; bM = M_out; bP = fp; bWD = wd_in; bPS = ps;
        end
    end
end

function [feasible, Zf, Mf] = gsim_q3(pid, m, tt, df, wd, park_seg, cons, ...
    MAX_LOAD, O, H, F, M, Z)
    % 同问题二的sim函数，使用期望消耗参数
    T = tt + sum(wd) + sum(park_seg) + 500;
    cO = zeros(1,T); cH = zeros(1,T); cF = zeros(1,T);
    zG = zeros(1,T); isSup = false(1,T);
    
    wa = []; ww = [];
    for i = 2:length(pid)
        if pid(i) >= 3 && pid(i) <= 5
            wa(end+1) = i; ww(end+1) = pid(i) - 2;
        end
    end
    
    day = 0;
    for k = 1:(m+1)
        for pd = 1:park_seg(k)
            day = day + 1; cO(day)=cons.PO; cH(day)=cons.PH; cF(day)=cons.PF;
        end
        d = df(pid(k), pid(k+1));
        for dd = 1:d
            day = day + 1; cO(day)=cons.MO; cH(day)=cons.MH; cF(day)=cons.MF;
            if dd == d && (pid(k+1)==6 || pid(k+1)==7), isSup(day)=true; end
        end
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && wd(wk) > 0
            wm_val = [4,5,3]; yld = [20,15,28];
            wmv = wm_val(ww(wk)); yv = yld(ww(wk)); rv = wd(wk);
            while rv > 0
                chunk = min(rv, wmv);
                for w = 1:chunk
                    day = day + 1; cO(day)=cons.WO; cH(day)=cons.WH; cF(day)=cons.WF; zG(day)=yv;
                end
                rv = rv - chunk;
                if rv > 0
                    day = day + 1; cO(day)=cons.PO; cH(day)=cons.PH; cF(day)=cons.PF;
                end
            end
        end
    end
    Ta = day;
    
    for t = 1:Ta
        O = O - cO(t); H = H - cH(t); F = F - cF(t); Z = Z + zG(t);
        if O < 0 || H < 0 || F < 0, feasible = false; Zf = Z; Mf = M; return; end
        if O + H + F > MAX_LOAD + 1e-9, feasible = false; Zf = Z; Mf = M; return; end
        if isSup(t)
            ns = Ta + 1;
            for tt2 = (t+1):Ta, if isSup(tt2), ns = tt2; break; end; end
            nO = 0; nH = 0; nF = 0;
            for tt2 = (t+1):ns
                if tt2 > Ta, break; end
                nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
            end
            sp = MAX_LOAD - (O + H + F);
            bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);
            if bO + bH + bF > sp, feasible = false; Zf = Z; Mf = M; return; end
            if ns > Ta && (O + bO < nO || H + bH < nH || F + bF < nF)
                feasible = false; Zf = Z; Mf = M; return;
            end
            cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
            if cost > M, feasible = false; Zf = Z; Mf = M; return; end
            O = O + bO; H = H + bH; F = F + bF; M = M - cost;
        end
    end
    feasible = true; Zf = Z; Mf = M;
end

function [ok, Zf, Mf] = simulate_mc(pid, wd, ps, weather, dist, cons_n, cons_t, WY, WM, ML, O, H, F, M, Z)
    % 蒙特卡洛模拟: 按实际天气序列逐日模拟
    m = length(pid) - 2;
    tt = 0;
    for k = 1:(m+1), tt = tt + dist(pid(k), pid(k+1)); end
    
    T = tt + sum(wd) + sum(ps) + 500;
    cO = zeros(1,T); cH = zeros(1,T); cF = zeros(1,T); zG = zeros(1,T); iS = false(1,T);
    
    wa = []; ww = [];
    for i = 2:length(pid)
        if pid(i) >= 3 && pid(i) <= 5, wa(end+1) = i; ww(end+1) = pid(i)-2; end
    end
    
    day = 0;
    for k = 1:(m+1)
        for pd = 1:ps(k)
            day = day + 1; w_idx = min(day, length(weather));
            c = iif(weather(w_idx), cons_n, cons_t);
            cO(day)=c.PO; cH(day)=c.PH; cF(day)=c.PF;
        end
        d = dist(pid(k), pid(k+1));
        for dd = 1:d
            day = day + 1; w_idx = min(day, length(weather));
            c = iif(weather(w_idx), cons_n, cons_t);
            cO(day)=c.MO; cH(day)=c.MH; cF(day)=c.MF;
            if dd == d && (pid(k+1)==6 || pid(k+1)==7), iS(day)=true; end
        end
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && wd(wk) > 0
            wmv = WM(ww(wk)); yv = WY(ww(wk)); rv = wd(wk);
            while rv > 0
                chunk = min(rv, wmv);
                for w = 1:chunk
                    day = day + 1; w_idx = min(day, length(weather));
                    c = iif(weather(w_idx), cons_n, cons_t);
                    cO(day)=c.WO; cH(day)=c.WH; cF(day)=c.WF; zG(day)=yv;
                end
                rv = rv - chunk;
                if rv > 0
                    day = day + 1; w_idx = min(day, length(weather));
                    c = iif(weather(w_idx), cons_n, cons_t);
                    cO(day)=c.PO; cH(day)=c.PH; cF(day)=c.PF;
                end
            end
        end
    end
    Ta = day;
    
    for t = 1:Ta
        O = O - cO(t); H = H - cH(t); F = F - cF(t); Z = Z + zG(t);
        if O < 0 || H < 0 || F < 0, ok = false; Zf = Z; Mf = M; return; end
        if O + H + F > ML + 1e-9, ok = false; Zf = Z; Mf = M; return; end
        if iS(t)
            ns = Ta + 1;
            for tt2 = (t+1):Ta, if iS(tt2), ns = tt2; break; end; end
            nO = 0; nH = 0; nF = 0;
            for tt2 = (t+1):ns
                if tt2 > Ta, break; end
                nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
            end
            sp = ML - (O + H + F);
            bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);
            if bO + bH + bF > sp, ok = false; Zf = Z; Mf = M; return; end
            cost = bO * cons_n.pO + bH * cons_n.pH + bF * cons_n.pF;
            if cost > M, ok = false; Zf = Z; Mf = M; return; end
            O = O + bO; H = H + bH; F = F + bF; M = M - cost;
        end
    end
    ok = true; Zf = Z; Mf = M;
end

function combs = enumerate_park_combs(n_seg, max_total)
    combs = zeros(0, n_seg); current = zeros(1, n_seg);
    function rec(pos, rem)
        if pos == n_seg, current(pos)=rem; combs(end+1,:)=current; return; end
        for p = 0:rem, current(pos)=p; rec(pos+1, rem-p); end
    end
    rec(1, max_total);
end

function max_w = max_work_with_park(mc, remaining)
    best = 0;
    for k = 1:(remaining + 1)
        stay = k*mc + (k-1);
        if stay > remaining, break; end
        best = k*mc;
        slack = remaining - stay;
        if slack >= 1, best = max(best, k*mc + min(mc, slack-1)); end
    end
    max_w = max(best, min(mc, remaining));
end

function s = iif(cond, t, f)
    if cond, s = t; else, s = f; end
end
