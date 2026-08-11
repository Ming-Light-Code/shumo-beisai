function varargout = rso_solver_v2(action, varargin)
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
