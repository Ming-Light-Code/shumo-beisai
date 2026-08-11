function varargout = rso_solver(action, varargin)
% RSO_SOLVER  Task3 Rolling Stochastic Optimization Engine
% At each step: sample K scenarios for H-day horizon,
% run CP for each scenario, choose action with best expected value

switch action
    case 'step',       [varargout{1},varargout{2}] = rso_step(varargin{:});
    case 'weather_sample', varargout{1} = sample_scenarios(varargin{:});
    case 'config',     varargout{1} = get_rso_config();
    otherwise, error('rso_solver: unknown action');
end
end

function cfg = get_rso_config()
    cfg.H = 10;     % lookahead horizon (days)
    cfg.K = 30;     % number of scenarios
    cfg.pN = 0.8;   % normal weather probability
end

% ===== Sample K weather scenarios for H days =====
function scenarios = sample_scenarios(H, K, pN)
    if nargin<3, pN=0.8; end
    scenarios = cell(1, K);
    for k = 1:K
        ws = repmat('N', 1, H);
        for i = 1:H
            if rand() > pN, ws(i) = 'T'; end
        end
        scenarios{k} = ws;
    end
end

% ===== One RSO decision step =====
function [best_action, best_value] = rso_step(cur_pt, elapsed, state, cfg_cp, H, K)
% Returns the best first action and its expected value
% Actions considered: {MOVE_TO(target), PARK, WORK, SUPPLY}

    if nargin<6, K=30; end
    if nargin<7, H=10; end
    
    % Sample scenarios
    scenarios = sample_scenarios(H, K);
    
    % Candidate actions
    inter_nodes = [3 4 5 6 7];  % W1, W2, W3, S1, S2
    
    % Action values: (action_type, target_node, expected_Z)
    action_values = {};
    
    % 1. MOVE actions: move toward each reachable node
    for ni = 1:length(inter_nodes)
        target = inter_nodes(ni);
        if target == cur_pt, continue; end
        d = cfg_cp.dist(cur_pt, target);
        if elapsed + d > cfg_cp.MAX_DAYS, continue; end
        
        % Simulate: move one step toward target, then evaluate
        exp_val = 0;
        valid_count = 0;
        for k = 1:K
            val = simulate_move_then_plan(cur_pt, target, elapsed, state, ...
                                          scenarios{k}, cfg_cp);
            if val > -inf
                exp_val = exp_val + val;
                valid_count = valid_count + 1;
            end
        end
        if valid_count > 0
            action_values{end+1} = struct('type', 'MOVE', 'target', target, ...
                'value', exp_val / K);
        end
    end
    
    % 2. PARK action: stay in place one day
    exp_val = 0;
    for k = 1:K
        val = simulate_park_then_plan(cur_pt, elapsed, state, scenarios{k}, cfg_cp);
        if val > -inf, exp_val = exp_val + val; end
    end
    action_values{end+1} = struct('type', 'PARK', 'target', cur_pt, ...
        'value', exp_val / K);
    
    % 3. WORK action (only at work points)
    if cur_pt >= 3 && cur_pt <= 5
        exp_val = 0;
        for k = 1:K
            val = simulate_work_then_plan(cur_pt, elapsed, state, scenarios{k}, cfg_cp);
            if val > -inf, exp_val = exp_val + val; end
        end
        action_values{end+1} = struct('type', 'WORK', 'target', cur_pt, ...
            'value', exp_val / K);
    end
    
    % 4. SUPPLY action (only at supply points)
    if cur_pt == 6 || cur_pt == 7
        exp_val = 0;
        for k = 1:K
            val = simulate_supply_then_plan(cur_pt, elapsed, state, scenarios{k}, cfg_cp);
            if val > -inf, exp_val = exp_val + val; end
        end
        action_values{end+1} = struct('type', 'SUPPLY', 'target', cur_pt, ...
            'value', exp_val / K);
    end
    
    % Select best action
    if isempty(action_values)
        best_action = struct('type', 'PARK', 'target', cur_pt);
        best_value = -inf;
        return;
    end
    
    best_idx = 1;
    best_value = action_values{1}.value;
    for i = 2:length(action_values)
        if action_values{i}.value > best_value
            best_value = action_values{i}.value;
            best_idx = i;
        end
    end
    best_action = action_values{best_idx};
end

% ===== Simulate: move one step, then CP from new state =====
function val = simulate_move_then_plan(cur_pt, target, elapsed, state, scenario, cfg)
    % Move one step toward target
    fr = cfg.xy(cur_pt, :); to = cfg.xy(target, :);
    dx = to(1) - fr(1); dy = to(2) - fr(2);
    
    % Determine move direction
    new_pos = fr;
    if abs(dx) > 0
        new_pos(1) = fr(1) + sign(dx);
    elseif abs(dy) > 0
        new_pos(2) = fr(2) + sign(dy);
    end
    
    % Apply day-1 weather consumption
    w1 = scenario(1);
    if w1 == 'T'
        cO = 8; cH = 4; cF = 3;
    else
        cO = 2; cH = 3; cF = 2;
    end
    
    new_O = state.O - cO;
    new_H = state.H - cH;
    new_F = state.F - cF;
    
    if new_O < 0 || new_H < 0 || new_F < 0
        val = -inf; return;
    end
    if new_O + new_H + new_F > cfg.MAX_LOAD
        val = -inf; return;
    end
    
    % Check if arrived at target
    if new_pos(1) == to(1) && new_pos(2) == to(2)
        new_pt = target;
    else
        new_pt = cur_pt;  % position unchanged (we only track node-level)
    end
    
    % Plan from new state for remaining scenario
    new_state = struct('O', new_O, 'H', new_H, 'F', new_F, ...
                       'M', state.M, 'Z', state.Z);
    val = plan_and_evaluate(new_pt, elapsed + 1, new_state, ...
                            scenario(2:min(end, cfg.MAX_DAYS-elapsed)), cfg);
end

% ===== Simulate: park one day, then CP =====
function val = simulate_park_then_plan(cur_pt, elapsed, state, scenario, cfg)
    w1 = scenario(1);
    if w1 == 'T'
        cO = 3; cH = 3; cF = 2;
    else
        cO = 1; cH = 1; cF = 1;
    end
    
    new_O = state.O - cO;
    new_H = state.H - cH;
    new_F = state.F - cF;
    
    if new_O < 0 || new_H < 0 || new_F < 0
        val = -inf; return;
    end
    
    new_state = struct('O', new_O, 'H', new_H, 'F', new_F, ...
                       'M', state.M, 'Z', state.Z);
    val = plan_and_evaluate(cur_pt, elapsed + 1, new_state, ...
                            scenario(2:min(end, cfg.MAX_DAYS-elapsed)), cfg);
end

% ===== Simulate: work one day, then CP =====
function val = simulate_work_then_plan(cur_pt, elapsed, state, scenario, cfg)
    w1 = scenario(1);
    wi = cur_pt - 2;
    if w1 == 'T'
        cO = 8; cH = 6; cF = 6;
    else
        cO = 5; cH = 4; cF = 3;
    end
    
    new_O = state.O - cO;
    new_H = state.H - cH;
    new_F = state.F - cF;
    new_Z = state.Z + cfg.WY(wi);
    
    if new_O < 0 || new_H < 0 || new_F < 0
        val = -inf; return;
    end
    
    new_state = struct('O', new_O, 'H', new_H, 'F', new_F, ...
                       'M', state.M, 'Z', new_Z);
    val = plan_and_evaluate(cur_pt, elapsed + 1, new_state, ...
                            scenario(2:min(end, cfg.MAX_DAYS-elapsed)), cfg);
end

% ===== Simulate: supply, then CP =====
function val = simulate_supply_then_plan(cur_pt, elapsed, state, scenario, cfg)
    w1 = scenario(1);
    if w1 == 'T'
        cP = 3 + 3 + 2;  % park consumption
    else
        cP = 1 + 1 + 1;
    end
    
    % Simple supply: buy enough for remaining journey
    dE = cfg.dist(cur_pt, 2);
    cT = cp_engine_v2('cons', 'thunder');
    need_O = max(0, dE * cT.MO * 0.8 - state.O);
    need_H = max(0, dE * cT.MH * 0.8 - state.H);
    need_F = max(0, dE * cT.MF * 0.8 - state.F);
    
    sp = cfg.MAX_LOAD - (state.O + state.H + state.F);
    total_need = need_O + need_H + need_F;
    if total_need > sp
        scl = sp / total_need;
        need_O = need_O * scl; need_H = need_H * scl; need_F = need_F * scl;
    end
    
    cost = need_O * 2 + need_H * 1 + need_F * 2;
    if cost > state.M
        scl = state.M / cost;
        need_O = need_O * scl; need_H = need_H * scl; need_F = need_F * scl;
        cost = state.M;
    end
    
    new_O = state.O + need_O - cP;
    new_H = state.H + need_H - cP / 3;
    new_F = state.F + need_F - cP / 3;
    new_M = state.M - cost;
    
    if new_O < 0 || new_H < 0 || new_F < 0 || new_M < 0
        val = -inf; return;
    end
    
    new_state = struct('O', new_O, 'H', new_H, 'F', new_F, ...
                       'M', new_M, 'Z', state.Z);
    val = plan_and_evaluate(cur_pt, elapsed + 1, new_state, ...
                            scenario(2:min(end, cfg.MAX_DAYS-elapsed)), cfg);
end

% ===== Plan from state with given weather scenario =====
function val = plan_and_evaluate(cur_pt, elapsed, state, scenario, cfg)
    if isempty(scenario) || elapsed >= cfg.MAX_DAYS
        if cur_pt == 2
            val = state.Z;
        else
            val = 0;  % cannot reach E
        end
        return;
    end
    
    % Build deterministic consumption for this scenario
    rem = cfg.MAX_DAYS - elapsed;
    H_scen = min(length(scenario), rem);
    
    % Quick feasibility check: can we reach E?
    dE = cfg.dist(cur_pt, 2);
    if dE > rem
        val = 0; return;
    end
    
    % Compute expected remaining work yield (simplified: CP with scenario weather)
    init_s = struct('O', state.O, 'H', state.H, 'F', state.F, ...
                    'M', state.M, 'Z', state.Z);
    
    cons = build_scenario_cons(scenario(1:H_scen));
    
    [~, ~, ~, fok] = cp_engine_v2('plan', cur_pt, elapsed, cons, cfg, false, init_s);
    
    if ~fok
        val = 0;  % no feasible plan
    else
        % Approximate: use optimistic estimate
        max_work = min(rem - dE, 15);
        val = state.Z + max_work * 20;  % rough estimate
    end
end

% ===== Build consumption parameters for a weather scenario =====
function cons = build_scenario_cons(scenario)
    nN = sum(scenario == 'N');
    nT = sum(scenario == 'T');
    n = length(scenario);
    if n == 0
        cons = cp_engine_v2('cons', 'expected');
        return;
    end
    pN = nN / n;
    pT = nT / n;
    
    cons.MO = pN*2 + pT*8;
    cons.MH = pN*3 + pT*4;
    cons.MF = pN*2 + pT*3;
    cons.PO = pN*1 + pT*3;
    cons.PH = pN*1 + pT*3;
    cons.PF = pN*1 + pT*2;
    cons.WO = pN*5 + pT*8;
    cons.WH = pN*4 + pT*6;
    cons.WF = pN*3 + pT*6;
    cons.pO = 2; cons.pH = 1; cons.pF = 2;
end
