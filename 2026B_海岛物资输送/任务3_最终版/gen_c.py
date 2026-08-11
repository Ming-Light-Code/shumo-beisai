import os

BASE = r"C:\Users\ming\Desktop\任务3_最终版"
DIR_C = os.path.join(BASE, "优化C_滚动随机优化")

os.makedirs(DIR_C, exist_ok=True)

def write_file(dirpath, filename, content):
    path = os.path.join(dirpath, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Written: {filename}")

# ============================================================
# APPROACH C: rso_solver.m (Rolling Stochastic Optimization)
# ============================================================
RSO_SOLVER = r"""function varargout = rso_solver(action, varargin)
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
"""

write_file(DIR_C, "rso_solver.m", RSO_SOLVER)

# ============================================================
# APPROACH C: solve_q3_rso.m
# ============================================================
SOLVE_Q3_RSO = r"""function solve_q3_rso(weather_seq)
% SOLVE_Q3_RSO  Task3 Rolling Stochastic Optimization Online Decision
% At each step: sample scenarios, evaluate candidate actions, pick best

rso_cfg = rso_solver('config');
cfg = cp_engine_v2('config');
cN = cp_engine_v2('cons', 'normal');
cT = cp_engine_v2('cons', 'thunder');
ce = cp_engine_v2('cons', 'expected');

if nargin < 1 || isempty(weather_seq)
    rng('shuffle'); weather_seq = cp_engine_v2('weather', 90, 0.8);
end

st.pt = 1; st.pos = cfg.xy(1,:);
st.O = cfg.init.O; st.H = cfg.init.H; st.F = cfg.init.F;
st.M = cfg.init.M; st.Z = cfg.init.Z; st.consec = 0; st.day = 0;
st.consec_work = 0;

fprintf('========================================\n');
fprintf('  Task3 RSO Online Decision\n');
fprintf('========================================\n');
fprintf('Horizon=%dd | Scenarios=%d | Expectation-Max\n', rso_cfg.H, rso_cfg.K);
fprintf('Day  | W | Action              | Pos        | Res       |   Z    M\n');
fprintf('-----|---|---------------------|------------|-----------|---------\n');

while st.day < cfg.MAX_DAYS && st.pt ~= 2
    st.day = st.day + 1;
    w = weather_seq(st.day);
    if w == 'T', ca = cT; wn = 'T';
    else, ca = cN; wn = 'N';
    end
    
    % Run RSO step
    curr_state = struct('O', st.O, 'H', st.H, 'F', st.F, ...
                        'M', st.M, 'Z', st.Z);
    [best_act, ~] = rso_solver('step', st.pt, st.day - 1, ...
                               curr_state, cfg, rso_cfg.H, rso_cfg.K);
    
    action_str = '';
    ik = false;
    
    switch best_act.type
        case 'MOVE'
            target = best_act.target;
            fr = cfg.xy(st.pt, :); to = cfg.xy(target, :);
            dx = to(1) - fr(1); dy = to(2) - fr(2);
            if abs(dx) > 0
                st.pos(1) = fr(1) + sign(dx);
                st.pos(2) = fr(2);
            elseif abs(dy) > 0
                st.pos(1) = fr(1);
                st.pos(2) = fr(2) + sign(dy);
            end
            st.O = st.O - ca.MO; st.H = st.H - ca.MH; st.F = st.F - ca.MF;
            st.consec = 0; st.consec_work = 0;
            action_str = sprintf('MOVE->(%d,%d)', st.pos(1), st.pos(2));
            
            if st.pos(1) == to(1) && st.pos(2) == to(2)
                st.pt = target;
                if st.pt == 6 || st.pt == 7
                    dE = cfg.dist(st.pt, 2);
                    ct = cp_engine_v2('cons', 'thunder');
                    need_O = max(0, dE * ct.MO * 0.9 - st.O);
                    need_H = max(0, dE * ct.MH * 0.9 - st.H);
                    need_F = max(0, dE * ct.MF * 0.9 - st.F);
                    sp = cfg.MAX_LOAD - (st.O + st.H + st.F);
                    total = need_O + need_H + need_F;
                    if total > sp
                        scl = sp / total;
                        need_O = need_O * scl; need_H = need_H * scl; need_F = need_F * scl;
                    end
                    cost = need_O * 2 + need_H * 1 + need_F * 2;
                    if cost <= st.M
                        st.O = st.O + need_O; st.H = st.H + need_H;
                        st.F = st.F + need_F; st.M = st.M - cost;
                    end
                    action_str = sprintf('SUPPLY(%s)', cfg.names{st.pt});
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
                    st.Z = st.Z + cfg.WY(wi);
                    st.consec_work = st.consec_work + 1;
                    action_str = sprintf('WORK(%s)', cfg.names{st.pt});
                else
                    st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                    st.consec_work = 0;
                    action_str = 'PARK(reset)';
                end
                ik = true;
            end
            
        case 'SUPPLY'
            if st.pt == 6 || st.pt == 7
                dE = cfg.dist(st.pt, 2);
                ct = cp_engine_v2('cons', 'thunder');
                need_O = max(0, dE * ct.MO * 0.9 - st.O);
                need_H = max(0, dE * ct.MH * 0.9 - st.H);
                need_F = max(0, dE * ct.MF * 0.9 - st.F);
                sp = cfg.MAX_LOAD - (st.O + st.H + st.F);
                total = need_O + need_H + need_F;
                if total > sp
                    scl = sp / total;
                    need_O = need_O * scl; need_H = need_H * scl; need_F = need_F * scl;
                end
                cost = need_O * 2 + need_H * 1 + need_F * 2;
                if cost <= st.M
                    st.O = st.O + need_O; st.H = st.H + need_H;
                    st.F = st.F + need_F; st.M = st.M - cost;
                end
                action_str = 'SUPPLY';
                ik = true;
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

write_file(DIR_C, "solve_q3_rso.m", SOLVE_Q3_RSO)

# ============================================================
# APPROACH C: solve_q3_rso_mc.m
# ============================================================
SOLVE_Q3_RSO_MC = r"""function solve_q3_rso_mc(N)
% SOLVE_Q3_RSO_MC  Task3 RSO MC Verification

if nargin<1||isempty(N), N=50; end
rso_cfg = rso_solver('config');
cfg = cp_engine_v2('config');
cN = cp_engine_v2('cons', 'normal');
cT = cp_engine_v2('cons', 'thunder');
ce = cp_engine_v2('cons', 'expected');

fprintf('========================================\n');
fprintf('  Task3 RSO MC Verification (N=%d)\n', N);
fprintf('========================================\n');
fprintf('Horizon=%dd | Scenarios=%d\n\n', rso_cfg.H, rso_cfg.K);

Zr = NaN(1,N); Mr = NaN(1,N); ok = false(1,N); dy = NaN(1,N);
fr = cell(1,N); ri = max(1, floor(N/10)); tic;

for sim = 1:N
    ws = cp_engine_v2('weather', 90, 0.8);
    [Zf, Mf, arr, days, reason] = run_rso_silent(ws, rso_cfg);
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

function [Zf, Mf, arr, days, reason] = run_rso_silent(wseq, rso_cfg)
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
        
        curr_state = struct('O', st.O, 'H', st.H, 'F', st.F, ...
                            'M', st.M, 'Z', st.Z);
        [best_act, ~] = rso_solver('step', st.pt, st.day - 1, ...
                                   curr_state, cfg, rso_cfg.H, rso_cfg.K);
        
        switch best_act.type
            case 'MOVE'
                target = best_act.target;
                fr = cfg.xy(st.pt, :); to = cfg.xy(target, :);
                dx = to(1) - fr(1); dy = to(2) - fr(2);
                if abs(dx) > 0
                    st.pos(1) = fr(1) + sign(dx);
                    st.pos(2) = fr(2);
                elseif abs(dy) > 0
                    st.pos(1) = fr(1);
                    st.pos(2) = fr(2) + sign(dy);
                end
                st.O = st.O - ca.MO; st.H = st.H - ca.MH; st.F = st.F - ca.MF;
                st.consec = 0; st.consec_work = 0;
                
                if st.pos(1) == to(1) && st.pos(2) == to(2)
                    st.pt = target;
                    if st.pt == 6 || st.pt == 7
                        dE = cfg.dist(st.pt, 2);
                        ct = cp_engine_v2('cons', 'thunder');
                        need_O = max(0, dE * ct.MO * 0.9 - st.O);
                        need_H = max(0, dE * ct.MH * 0.9 - st.H);
                        need_F = max(0, dE * ct.MF * 0.9 - st.F);
                        sp = cfg.MAX_LOAD - (st.O + st.H + st.F);
                        total = need_O + need_H + need_F;
                        if total > sp
                            scl = sp / total;
                            need_O = need_O * scl; need_H = need_H * scl; need_F = need_F * scl;
                        end
                        cost = need_O * 2 + need_H * 1 + need_F * 2;
                        if cost <= st.M
                            st.O = st.O + need_O; st.H = st.H + need_H;
                            st.F = st.F + need_F; st.M = st.M - cost;
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
                    dE = cfg.dist(st.pt, 2);
                    ct = cp_engine_v2('cons', 'thunder');
                    need_O = max(0, dE * ct.MO * 0.9 - st.O);
                    need_H = max(0, dE * ct.MH * 0.9 - st.H);
                    need_F = max(0, dE * ct.MF * 0.9 - st.F);
                    sp = cfg.MAX_LOAD - (st.O + st.H + st.F);
                    total = need_O + need_H + need_F;
                    if total > sp
                        scl = sp / total;
                        need_O = need_O * scl; need_H = need_H * scl; need_F = need_F * scl;
                    end
                    cost = need_O * 2 + need_H * 1 + need_F * 2;
                    if cost <= st.M
                        st.O = st.O + need_O; st.H = st.H + need_H;
                        st.F = st.F + need_F; st.M = st.M - cost;
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

write_file(DIR_C, "solve_q3_rso_mc.m", SOLVE_Q3_RSO_MC)
print("Approach C: all MATLAB files written")
