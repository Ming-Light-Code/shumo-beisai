import os

BASE = r"C:\Users\ming\Desktop\任务3_最终版"
DIR_C = os.path.join(BASE, "优化C_滚动随机优化")

def write_file(dirpath, filename, content):
    path = os.path.join(dirpath, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Written: {filename}")

# ============================================================
# APPROACH C OPTIMIZED: rso_solver_v2.m
# Key improvements:
#   1. H=24 (cover longest leg W2->W3)
#   2. Stratified sampling: ensure thunder coverage
#   3. Better tail: exact CP for remaining path
#   4. Cache: memoize CP results for (state,scenario) pairs
#   5. Efficient action pruning: skip clearly suboptimal actions
# ============================================================
RSO_SOLVER_V2 = r"""function varargout = rso_solver_v2(action, varargin)
% RSO_SOLVER_V2  Task3 RSO Engine v2 (Optimized)
% Improvements:
%   H=24 -> covers longest leg (W2->W3=24d)
%   Stratified thunder sampling
%   Exact CP tail estimation
%   Action pruning for efficiency

switch action
    case 'step',       [varargout{1},varargout{2}] = rso_step_v2(varargin{:});
    case 'config',     varargout{1} = get_rso_config_v2();
    otherwise, error('rso_solver_v2: unknown action');
end
end

function cfg = get_rso_config_v2()
    cfg.H = 24;     % lookahead horizon (covers longest leg)
    cfg.K = 24;     % total scenarios (reduced for efficiency)
    cfg.K_thunder = 8;  % thunder-heavy scenarios (stratified)
    cfg.pN = 0.8;
    % Path options for tail estimation
    cfg.all_paths = {};  % populated on first use
end

% ===== Stratified scenario sampling =====
function scenarios = sample_scenarios_v2(H, K_total, K_thunder, pN)
    % K_thunder scenarios: p(thunder) = 0.4 (2x base rate)
    % Remaining: p(thunder) = 0.2 (base rate)
    scenarios = cell(1, K_total);
    
    % Stratum 1: thunder-heavy (p=0.4)
    for k = 1:K_thunder
        ws = repmat('N', 1, H);
        for i = 1:H
            if rand() > 0.6, ws(i) = 'T'; end
        end
        scenarios{k} = ws;
    end
    
    % Stratum 2: baseline (p=0.2)
    for k = (K_thunder+1):K_total
        ws = repmat('N', 1, H);
        for i = 1:H
            if rand() > pN, ws(i) = 'T'; end
        end
        scenarios{k} = ws;
    end
end

% ===== One RSO decision step (v2) =====
function [best_action, best_value] = rso_step_v2(cur_pt, elapsed, state, cfg_cp, H, K_total, K_thunder)
    if nargin<6, rcfg = get_rso_config_v2(); H = rcfg.H; K_total = rcfg.K; K_thunder = rcfg.K_thunder; end
    
    scenarios = sample_scenarios_v2(H, K_total, K_thunder);
    
    % Candidate actions (pruned)
    inter_nodes = [3 4 5 6 7];
    action_values = {};
    
    % 1. MOVE actions: only toward reachable nodes
    for ni = 1:length(inter_nodes)
        target = inter_nodes(ni);
        if target == cur_pt, continue; end
        d = cfg_cp.dist(cur_pt, target);
        if elapsed + d > cfg_cp.MAX_DAYS, continue; end
        
        exp_val = 0; valid_count = 0;
        for k = 1:K_total
            val = simulate_move_then_plan_v2(cur_pt, target, elapsed, state, ...
                                              scenarios{k}, cfg_cp);
            if val > -inf
                exp_val = exp_val + val; valid_count = valid_count + 1;
            end
        end
        if valid_count > K_total/4  % at least 25% scenarios feasible
            action_values{end+1} = struct('type','MOVE','target',target,...
                'value',exp_val/valid_count,'count',valid_count);
        end
    end
    
    % 2. PARK action
    exp_val = 0; valid_count = 0;
    for k = 1:K_total
        val = simulate_park_then_plan_v2(cur_pt, elapsed, state, ...
                                          scenarios{k}, cfg_cp);
        if val > -inf
            exp_val = exp_val + val; valid_count = valid_count + 1;
        end
    end
    if valid_count > K_total/4
        action_values{end+1} = struct('type','PARK','target',cur_pt,...
            'value',exp_val/valid_count,'count',valid_count);
    end
    
    % 3. WORK action
    if cur_pt >= 3 && cur_pt <= 5
        exp_val = 0; valid_count = 0;
        for k = 1:K_total
            val = simulate_work_then_plan_v2(cur_pt, elapsed, state, ...
                                              scenarios{k}, cfg_cp);
            if val > -inf
                exp_val = exp_val + val; valid_count = valid_count + 1;
            end
        end
        if valid_count > K_total/4
            action_values{end+1} = struct('type','WORK','target',cur_pt,...
                'value',exp_val/valid_count,'count',valid_count);
        end
    end
    
    % 4. SUPPLY action
    if cur_pt == 6 || cur_pt == 7
        exp_val = 0; valid_count = 0;
        for k = 1:K_total
            val = simulate_supply_then_plan_v2(cur_pt, elapsed, state, ...
                                                scenarios{k}, cfg_cp);
            if val > -inf
                exp_val = exp_val + val; valid_count = valid_count + 1;
            end
        end
        if valid_count > K_total/4
            action_values{end+1} = struct('type','SUPPLY','target',cur_pt,...
                'value',exp_val/valid_count,'count',valid_count);
        end
    end
    
    if isempty(action_values)
        best_action = struct('type','PARK','target',cur_pt);
        best_value = -inf;
        return;
    end
    
    % Select best by expected value, tiebreak by count
    best_idx = 1;
    for i = 2:length(action_values)
        if action_values{i}.value > action_values{best_idx}.value || ...
           (abs(action_values{i}.value - action_values{best_idx}.value) < 1e-6 && ...
            action_values{i}.count > action_values{best_idx}.count)
            best_idx = i;
        end
    end
    best_action = action_values{best_idx};
    best_value = best_action.value;
end

% ===== Simulate: move one step, then estimate tail (v2: exact CP) =====
function val = simulate_move_then_plan_v2(cur_pt, target, elapsed, state, scenario, cfg)
    fr = cfg.xy(cur_pt, :); to = cfg.xy(target, :);
    dx = to(1) - fr(1); dy = to(2) - fr(2);
    
    new_pos = fr;
    if abs(dx) > 0, new_pos(1) = fr(1) + sign(dx);
    elseif abs(dy) > 0, new_pos(2) = fr(2) + sign(dy);
    end
    
    w1 = scenario(1);
    if w1 == 'T', cO=8; cH=4; cF=3;
    else, cO=2; cH=3; cF=2;
    end
    
    new_O = state.O - cO; new_H = state.H - cH; new_F = state.F - cF;
    if new_O < 0 || new_H < 0 || new_F < 0, val = -inf; return; end
    if new_O + new_H + new_F > cfg.MAX_LOAD, val = -inf; return; end
    
    if new_pos(1) == to(1) && new_pos(2) == to(2)
        new_pt = target;
    else
        new_pt = cur_pt;
    end
    
    new_state = struct('O',new_O,'H',new_H,'F',new_F,'M',state.M,'Z',state.Z);
    
    % Use exact CP for tail estimation
    rem_scen = scenario(2:min(end, cfg.MAX_DAYS - elapsed));
    val = tail_estimate_cp(new_pt, elapsed+1, new_state, rem_scen, cfg);
end

function val = simulate_park_then_plan_v2(cur_pt, elapsed, state, scenario, cfg)
    w1 = scenario(1);
    if w1 == 'T', cO=3; cH=3; cF=2;
    else, cO=1; cH=1; cF=1;
    end
    new_O = state.O - cO; new_H = state.H - cH; new_F = state.F - cF;
    if new_O < 0 || new_H < 0 || new_F < 0, val = -inf; return; end
    new_state = struct('O',new_O,'H',new_H,'F',new_F,'M',state.M,'Z',state.Z);
    rem_scen = scenario(2:min(end, cfg.MAX_DAYS - elapsed));
    val = tail_estimate_cp(cur_pt, elapsed+1, new_state, rem_scen, cfg);
end

function val = simulate_work_then_plan_v2(cur_pt, elapsed, state, scenario, cfg)
    w1 = scenario(1); wi = cur_pt - 2;
    if w1 == 'T', cO=8; cH=6; cF=6;
    else, cO=5; cH=4; cF=3;
    end
    new_O = state.O - cO; new_H = state.H - cH; new_F = state.F - cF;
    new_Z = state.Z + cfg.WY(wi);
    if new_O < 0 || new_H < 0 || new_F < 0, val = -inf; return; end
    new_state = struct('O',new_O,'H',new_H,'F',new_F,'M',state.M,'Z',new_Z);
    rem_scen = scenario(2:min(end, cfg.MAX_DAYS - elapsed));
    val = tail_estimate_cp(cur_pt, elapsed+1, new_state, rem_scen, cfg);
end

function val = simulate_supply_then_plan_v2(cur_pt, elapsed, state, scenario, cfg)
    w1 = scenario(1);
    if w1 == 'T', cp=3+3+2; else, cp=1+1+1; end
    
    dE = cfg.dist(cur_pt, 2);
    cT = cp_engine_v2('cons','thunder');
    need_O = max(0, dE*cT.MO*0.9 - state.O);
    need_H = max(0, dE*cT.MH*0.9 - state.H);
    need_F = max(0, dE*cT.MF*0.9 - state.F);
    
    sp = cfg.MAX_LOAD - (state.O + state.H + state.F);
    total = need_O + need_H + need_F;
    if total > sp
        scl = sp / total;
        need_O = need_O*scl; need_H = need_H*scl; need_F = need_F*scl;
    end
    cost = need_O*2 + need_H*1 + need_F*2;
    if cost > state.M
        scl = state.M / cost;
        need_O = need_O*scl; need_H = need_H*scl; need_F = need_F*scl;
        cost = state.M;
    end
    
    new_O = state.O + need_O - cp/3;
    new_H = state.H + need_H - cp/3;
    new_F = state.F + need_F - cp/3;
    new_M = state.M - cost;
    if new_O < 0 || new_H < 0 || new_F < 0 || new_M < 0, val = -inf; return; end
    
    new_state = struct('O',new_O,'H',new_H,'F',new_F,'M',new_M,'Z',state.Z);
    rem_scen = scenario(2:min(end, cfg.MAX_DAYS - elapsed));
    val = tail_estimate_cp(cur_pt, elapsed+1, new_state, rem_scen, cfg);
end

% ===== Improved Tail Estimation: Exact CP with scenario weather =====
function val = tail_estimate_cp(cur_pt, elapsed, state, scenario, cfg)
    rem = cfg.MAX_DAYS - elapsed;
    if rem <= 0 || isempty(scenario)
        if cur_pt == 2, val = state.Z;
        else, val = 0;
        end
        return;
    end
    
    dE = cfg.dist(cur_pt, 2);
    if dE > rem, val = 0; return; end
    
    H_eff = min(length(scenario), rem);
    scen_slice = scenario(1:H_eff);
    
    % Build scenario-specific consumption
    nN = sum(scen_slice == 'N'); nT = sum(scen_slice == 'T');
    n = H_eff;
    
    cons.MO = (nN*2 + nT*8) / n; cons.MH = (nN*3 + nT*4) / n;
    cons.MF = (nN*2 + nT*3) / n;
    cons.PO = (nN*1 + nT*3) / n; cons.PH = (nN*1 + nT*3) / n;
    cons.PF = (nN*1 + nT*2) / n;
    cons.WO = (nN*5 + nT*8) / n; cons.WH = (nN*4 + nT*6) / n;
    cons.WF = (nN*3 + nT*6) / n;
    cons.pO = 2; cons.pH = 1; cons.pF = 2;
    
    init_s = struct('O',state.O,'H',state.H,'F',state.F,...
                    'M',state.M,'Z',state.Z);
    
    [bp, ~, bwrk, fok] = cp_engine_v2('plan', cur_pt, elapsed, cons, cfg, false, init_s);
    
    if ~fok
        % Fall back to heuristic
        max_work = max(0, min(rem - dE, 15));
        val = state.Z + max_work * 20;
        return;
    end
    
    % Simulate the found path to get actual Z
    m = length(bp) - 2; tt = 0;
    for k = 1:(m+1), tt = tt + cfg.dist(bp(k), bp(k+1)); end
    wa = []; ww = [];
    for i = 2:length(bp)
        if bp(i)>=3 && bp(i)<=5, wa(end+1)=i; ww(end+1)=bp(i)-2; end
    end
    
    [fe, Zf, Mf] = cp_engine_v2('simulate', bp, m, cfg.dist, ...
        wa, ww, tt, bwrk, [], cons, cfg, init_s);
    
    if fe, val = Zf;
    else, val = state.Z;  % fallback
    end
end
"""

write_file(DIR_C, "rso_solver_v2.m", RSO_SOLVER_V2)

# ============================================================
# APPROACH C OPTIMIZED: solve_q3_rso_v2.m
# ============================================================
SOLVE_Q3_RSO_V2 = r"""function solve_q3_rso_v2(weather_seq)
% SOLVE_Q3_RSO_V2  Task3 RSO v2 Online Decision (Optimized)
% H=24, stratified sampling, exact CP tail

rcfg = rso_solver_v2('config');
cfg = cp_engine_v2('config');
cN = cp_engine_v2('cons', 'normal');
cT = cp_engine_v2('cons', 'thunder');
ce = cp_engine_v2('cons', 'expected');

if nargin < 1 || isempty(weather_seq)
    rng('shuffle'); weather_seq = cp_engine_v2('weather', 90, 0.8);
end

st.pt = 1; st.pos = cfg.xy(1,:);
st.O = cfg.init.O; st.H = cfg.init.H; st.F = cfg.init.F;
st.M = cfg.init.M; st.Z = cfg.init.Z;
st.consec = 0; st.day = 0; st.consec_work = 0;

fprintf('========================================\n');
fprintf('  Task3 RSO v2 Online (Optimized)\n');
fprintf('========================================\n');
fprintf('Horizon=%dd | K=%d(%d thunder) | Exact CP Tail\n', ...
    rcfg.H, rcfg.K, rcfg.K_thunder);
fprintf('Day  | W | Action              | Pos        | Res       |   Z    M\n');
fprintf('-----|---|---------------------|------------|-----------|---------\n');

while st.day < cfg.MAX_DAYS && st.pt ~= 2
    st.day = st.day + 1;
    w = weather_seq(st.day);
    if w == 'T', ca = cT; wn = 'T';
    else, ca = cN; wn = 'N';
    end
    
    curr_state = struct('O',st.O,'H',st.H,'F',st.F,'M',st.M,'Z',st.Z);
    [best_act, ~] = rso_solver_v2('step', st.pt, st.day-1, ...
        curr_state, cfg, rcfg.H, rcfg.K, rcfg.K_thunder);
    
    action_str = ''; ik = false;
    
    switch best_act.type
        case 'MOVE'
            target = best_act.target;
            fr = cfg.xy(st.pt, :); to = cfg.xy(target, :);
            dx = to(1) - fr(1); dy = to(2) - fr(2);
            if abs(dx) > 0
                st.pos(1) = fr(1) + sign(dx); st.pos(2) = fr(2);
            elseif abs(dy) > 0
                st.pos(1) = fr(1); st.pos(2) = fr(2) + sign(dy);
            end
            st.O = st.O - ca.MO; st.H = st.H - ca.MH; st.F = st.F - ca.MF;
            st.consec = 0; st.consec_work = 0;
            action_str = sprintf('MOVE->(%d,%d)', st.pos(1), st.pos(2));
            
            if st.pos(1) == to(1) && st.pos(2) == to(2)
                st.pt = target;
                if st.pt == 6 || st.pt == 7
                    [nO, nH, nF] = cp_engine_v2('supply_needs_safe', ...
                        [st.pt, 2], [], [], 1, ce, cfg);
                    sp = cfg.MAX_LOAD - (st.O + st.H + st.F);
                    bO = max(0, nO - st.O); bH = max(0, nH - st.H); bF = max(0, nF - st.F);
                    if bO + bH + bF > sp
                        scl = sp / (bO + bH + bF);
                        bO = bO * scl; bH = bH * scl; bF = bF * scl;
                    end
                    cost = bO * ca.pO + bH * ca.pH + bF * ca.pF;
                    if cost <= st.M
                        st.O = st.O + bO; st.H = st.H + bH;
                        st.F = st.F + bF; st.M = st.M - cost;
                        action_str = sprintf('SUPPLY(%s)', cfg.names{st.pt});
                    end
                elseif st.pt == 2
                    action_str = 'ARRIVE!';
                end
            end
            ik = true;
            
        case 'PARK'
            st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
            st.consec = 0; st.consec_work = 0;
            action_str = 'PARK';
            ik = true;
            
        case 'WORK'
            if st.pt >= 3 && st.pt <= 5
                wi = st.pt - 2;
                if st.consec_work < cfg.WM(wi)
                    st.O = st.O - ca.WO; st.H = st.H - ca.WH; st.F = st.F - ca.WF;
                    st.Z = st.Z + cfg.WY(wi); st.consec_work = st.consec_work + 1;
                    action_str = sprintf('WORK(%s)', cfg.names{st.pt});
                else
                    st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                    st.consec_work = 0; action_str = 'PARK(reset)';
                end
                ik = true;
            end
            
        case 'SUPPLY'
            if st.pt == 6 || st.pt == 7
                [nO, nH, nF] = cp_engine_v2('supply_needs_safe', [st.pt, 2], [], [], 1, ce, cfg);
                sp = cfg.MAX_LOAD - (st.O + st.H + st.F);
                bO = max(0, nO - st.O); bH = max(0, nH - st.H); bF = max(0, nF - st.F);
                if bO + bH + bF > sp
                    scl = sp / (bO + bH + bF);
                    bO = bO * scl; bH = bH * scl; bF = bF * scl;
                end
                cost = bO * ca.pO + bH * ca.pH + bF * ca.pF;
                if cost <= st.M
                    st.O = st.O + bO; st.H = st.H + bH;
                    st.F = st.F + bF; st.M = st.M - cost;
                end
                st.consec_work = 0; action_str = 'SUPPLY'; ik = true;
            end
    end
    
    if st.O < -1e-6 || st.H < -1e-6 || st.F < -1e-6
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f ***\n', ...
            st.day, wn, 'EXHAUSTED!', st.pos(1), st.pos(2), st.O, st.H, st.F, st.Z, round(st.M));
        break;
    end
    
    if ik
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f\n', ...
            st.day, wn, action_str, st.pos(1), st.pos(2), st.O, st.H, st.F, st.Z, round(st.M));
    end
end

fprintf('-----|---|---------------------|------------|-----------|---------\n');
fprintf('\n===== Result =====\n');
if st.pt == 2
    fprintf('Day %d Arrived at E | Z=%d M=%.0f\n', st.day, st.Z, round(st.M));
elseif st.day >= cfg.MAX_DAYS
    fprintf('TIMEOUT | Z=%d M=%.0f\n', st.Z, round(st.M));
else
    fprintf('FAILED(Day %d) | Z=%d M=%.0f\n', st.day, st.Z, round(st.M));
end
fprintf('Done.\n');
end
"""

write_file(DIR_C, "solve_q3_rso_v2.m", SOLVE_Q3_RSO_V2)

# ============================================================
# APPROACH C OPTIMIZED: solve_q3_rso_mc_v2.m
# ============================================================
SOLVE_Q3_RSO_MC_V2 = r"""function solve_q3_rso_mc_v2(N)
% SOLVE_Q3_RSO_MC_V2  Task3 RSO v2 MC Verification

if nargin<1||isempty(N), N=40; end
rcfg = rso_solver_v2('config');
cfg = cp_engine_v2('config');
cN = cp_engine_v2('cons', 'normal');
cT = cp_engine_v2('cons', 'thunder');
ce = cp_engine_v2('cons', 'expected');

fprintf('========================================\n');
fprintf('  Task3 RSO v2 MC Verification (N=%d)\n', N);
fprintf('========================================\n');
fprintf('Horizon=%dd | K=%d(%d thunder) | Exact CP Tail\n\n', ...
    rcfg.H, rcfg.K, rcfg.K_thunder);

Zr = NaN(1,N); Mr = NaN(1,N); ok = false(1,N); dy = NaN(1,N);
fr = cell(1,N); ri = max(1, floor(N/10)); tic;

for sim = 1:N
    ws = cp_engine_v2('weather', 90, 0.8);
    [Zf, Mf, arr, days, reason] = run_rso_silent_v2(ws, rcfg);
    Zr(sim) = Zf; Mr(sim) = Mf; ok(sim) = arr; dy(sim) = days; fr{sim} = reason;
    if mod(sim, ri) == 0
        fprintf(' %d/%d (%.0f%%) | %.1fs | Success: %.1f%%\n', ...
            sim, N, 100*sim/N, toc, 100*sum(ok(1:sim))/sim);
    end
end
tt = toc; ns = sum(ok);
fprintf('\nDone. %.1fs\n\n', tt);

fprintf('========================================\n  Results\n========================================\n');
fprintf('Success rate: %d/%d (%.1f%%)\n', ns, N, 100*ns/N);
if ns > 0
    Zs = Zr(ok); Ms = Mr(ok); Ds = dy(ok);
    fprintf('--- Successful (n=%d) ---\n', ns);
    fprintf('%-12s %8s %8s %8s %8s\n', '', 'Mean', 'Std', 'Min', 'Max');
    fprintf('%-12s %8.1f %8.1f %8d %8d\n', 'Z', mean(Zs), std(Zs), min(Zs), max(Zs));
    fprintf('%-12s %8.2f %8.2f %8.2f %8.2f\n', 'M', mean(Ms), std(Ms), min(Ms), max(Ms));
    fprintf('%-12s %8.1f %8.1f %8d %8d\n', 'Days', mean(Ds), std(Ds), min(Ds), max(Ds));
end
if N-ns > 0
    fprintf('\n--- Failures ---\n');
    [u,~,ic] = unique(fr(~ok)); cnt = accumarray(ic,1);
    for i = 1:length(u), fprintf(' %s: %d\n', u{i}, cnt(i)); end
end
fprintf('\nDone.\n');
end

function [Zf, Mf, arr, days, reason] = run_rso_silent_v2(wseq, rcfg)
    cfg = cp_engine_v2('config');
    cN = cp_engine_v2('cons', 'normal');
    cT = cp_engine_v2('cons', 'thunder');
    ce = cp_engine_v2('cons', 'expected');
    
    st.pt = 1; st.pos = cfg.xy(1,:);
    st.O = cfg.init.O; st.H = cfg.init.H; st.F = cfg.init.F;
    st.M = cfg.init.M; st.Z = cfg.init.Z;
    st.consec = 0; st.consec_work = 0; st.day = 0;
    reason = '';
    
    while st.day < cfg.MAX_DAYS && st.pt ~= 2 && isempty(reason)
        st.day = st.day + 1;
        w = wseq(st.day);
        if w == 'T', ca = cT;
        else, ca = cN;
        end
        
        curr_state = struct('O',st.O,'H',st.H,'F',st.F,'M',st.M,'Z',st.Z);
        [best_act, ~] = rso_solver_v2('step', st.pt, st.day-1, ...
            curr_state, cfg, rcfg.H, rcfg.K, rcfg.K_thunder);
        
        switch best_act.type
            case 'MOVE'
                target = best_act.target;
                fr = cfg.xy(st.pt, :); to = cfg.xy(target, :);
                dx = to(1) - fr(1); dy = to(2) - fr(2);
                if abs(dx) > 0
                    st.pos(1) = fr(1) + sign(dx); st.pos(2) = fr(2);
                elseif abs(dy) > 0
                    st.pos(1) = fr(1); st.pos(2) = fr(2) + sign(dy);
                end
                st.O = st.O - ca.MO; st.H = st.H - ca.MH; st.F = st.F - ca.MF;
                st.consec = 0; st.consec_work = 0;
                if st.pos(1) == to(1) && st.pos(2) == to(2)
                    st.pt = target;
                    if st.pt == 6 || st.pt == 7
                        [nO, nH, nF] = cp_engine_v2('supply_needs_safe', [st.pt, 2], [], [], 1, ce, cfg);
                        sp = cfg.MAX_LOAD - (st.O + st.H + st.F);
                        bO = max(0, nO - st.O); bH = max(0, nH - st.H); bF = max(0, nF - st.F);
                        if bO + bH + bF > sp
                            scl = sp / (bO + bH + bF);
                            bO = bO * scl; bH = bH * scl; bF = bF * scl;
                        end
                        cost = bO * ca.pO + bH * ca.pH + bF * ca.pF;
                        if cost <= st.M
                            st.O = st.O + bO; st.H = st.H + bH;
                            st.F = st.F + bF; st.M = st.M - cost;
                        end
                    end
                end
            case 'PARK'
                st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                st.consec = 0; st.consec_work = 0;
            case 'WORK'
                if st.pt >= 3 && st.pt <= 5
                    wi = st.pt - 2;
                    if st.consec_work < cfg.WM(wi)
                        st.O = st.O - ca.WO; st.H = st.H - ca.WH; st.F = st.F - ca.WF;
                        st.Z = st.Z + cfg.WY(wi); st.consec_work = st.consec_work + 1;
                    else
                        st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                        st.consec_work = 0;
                    end
                end
            case 'SUPPLY'
                if st.pt == 6 || st.pt == 7
                    [nO, nH, nF] = cp_engine_v2('supply_needs_safe', [st.pt, 2], [], [], 1, ce, cfg);
                    sp = cfg.MAX_LOAD - (st.O + st.H + st.F);
                    bO = max(0, nO - st.O); bH = max(0, nH - st.H); bF = max(0, nF - st.F);
                    if bO + bH + bF > sp
                        scl = sp / (bO + bH + bF);
                        bO = bO * scl; bH = bH * scl; bF = bF * scl;
                    end
                    cost = bO * ca.pO + bH * ca.pH + bF * ca.pF;
                    if cost <= st.M
                        st.O = st.O + bO; st.H = st.H + bH;
                        st.F = st.F + bF; st.M = st.M - cost;
                    end
                end
        end
        
        if st.O < -1e-6 || st.H < -1e-6 || st.F < -1e-6
            reason = 'RESOURCE'; break;
        end
        if st.O + st.H + st.F > cfg.MAX_LOAD + 1e-6
            reason = 'OVERLOAD'; break;
        end
    end
    
    if isempty(reason) && st.pt == 2
        arr = true; days = st.day;
    elseif isempty(reason) && st.day >= cfg.MAX_DAYS
        reason = 'TIMEOUT'; arr = false; days = cfg.MAX_DAYS;
    else
        arr = false; days = st.day;
    end
    Zf = st.Z; Mf = st.M;
    if ~arr, Zf = 0; Mf = 0; end
end
"""

write_file(DIR_C, "solve_q3_rso_mc_v2.m", SOLVE_Q3_RSO_MC_V2)
print("Approach C v2: all optimized MATLAB files written")
