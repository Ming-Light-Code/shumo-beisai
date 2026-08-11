import os

BASE = r"C:\Users\ming\Desktop\任务3_最终版"
DIR_B = os.path.join(BASE, "优化B_MDP方向")

os.makedirs(DIR_B, exist_ok=True)

def write_file(dirpath, filename, content):
    path = os.path.join(dirpath, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Written: {filename}")

# ============================================================
# APPROACH B: mdp_solver.m  (Value Iteration on Aggregated States)
# ============================================================
MDP_SOLVER = r"""function varargout = mdp_solver(action, varargin)
% MDP_SOLVER  Task3 MDP Engine (Value Iteration on Aggregated State Space)
% States: (node_id, weather_today, resource_level, day_bucket)
%   node_id: 1-7 (B,E,W1,W2,W3,S1,S2)
%   weather_today: 1=Normal, 2=Thunder
%   resource_level: 1=critical, 2=low, 3=normal
%   day_bucket: 1=early(1-30), 2=mid(31-60), 3=late(61-90)
% Total states: 7x2x3x3 = 126
% Actions: 1=Move-to-next-node, 2=Park-one-day, 3=Work(if-at-W),
%          4=Supply(if-at-S)

switch action
    case 'solve',    varargout{1} = solve_MDP(varargin{:});
    case 'policy',   [varargout{1},varargout{2},varargout{3}] = get_policy(varargin{:});
    case 'config',   varargout{1} = get_mdp_config();
    otherwise, error('mdp_solver: unknown action');
end
end

function cfg = get_mdp_config()
    cfg = cp_engine_v2('config');
    cfg.pN = 0.8;
    cfg.pT = 0.2;
    cfg.gamma = 1.0;  % undiscounted (finite horizon)
    cfg.max_iter = 200;
end

% ===== Main MDP Solver =====
function V = solve_MDP()
    cfg = get_mdp_config();
    nNodes = 7; nWeather = 2; nRes = 3; nDay = 3;
    nStates = nNodes * nWeather * nRes * nDay;
    
    % Action space: 1=move-to-next, 2=park, 3=work, 4=supply
    nActions = 4;
    
    % Initialize value function (optimistic: use max possible Z)
    V = zeros(nNodes, nWeather, nRes, nDay);
    % Terminal state: at E with any resources, V = current_Z (approximate)
    for w=1:nWeather, for r=1:nRes, for d=1:nDay
        V(2,w,r,d) = 500;  % optimistic terminal value
    end; end; end
    
    % Precompute transitions
    fprintf('MDP: %d states, running value iteration...\n', nStates);
    
    for iter = 1:cfg.max_iter
        delta = 0;
        V_old = V;
        
        for node = 1:nNodes
            if node == 2, continue; end  % E is terminal
            
            for w = 1:nWeather
                for r = 1:nRes
                    for d = 1:nDay
                        best_val = -inf;
                        
                        % Action 1: Move toward best next node
                        val_move = eval_move(node, w, r, d, V_old, cfg);
                        best_val = max(best_val, val_move);
                        
                        % Action 2: Park one day
                        val_park = eval_park(node, w, r, d, V_old, cfg);
                        best_val = max(best_val, val_park);
                        
                        % Action 3: Work (only at W nodes)
                        if node >= 3 && node <= 5
                            val_work = eval_work(node, w, r, d, V_old, cfg);
                            best_val = max(best_val, val_work);
                        end
                        
                        % Action 4: Supply (only at S nodes)
                        if node == 6 || node == 7
                            val_supply = eval_supply(node, w, r, d, V_old, cfg);
                            best_val = max(best_val, val_supply);
                        end
                        
                        V(node,w,r,d) = best_val;
                        delta = max(delta, abs(V(node,w,r,d) - V_old(node,w,r,d)));
                    end
                end
            end
        end
        
        if mod(iter,20)==0
            fprintf('  Iter %d, delta=%.2f\n', iter, delta);
        end
        if delta < 1e-3, fprintf('  Converged at iter %d\n', iter); break; end
    end
    fprintf('MDP solved.\n');
end

% ===== Evaluate Move Action =====
function val = eval_move(node, w, r, d, V, cfg)
    % Find best next node to move to
    best_val = -inf;
    for next = cfg.inter
        dist = cfg.dist(node, next);
        % Check if feasible with resource level
        if ~can_move(node, next, r, w, d, cfg), continue; end
        
        % Expected value after moving: weather changes
        next_w_n = 1; next_w_t = 2;
        next_r = resource_after_move(node, next, r, w, cfg);
        next_d = day_after_move(d, dist, cfg);
        
        if next_r == 0 || next_d == 0
            continue;  % infeasible
        end
        
        exp_val = cfg.pN * V(next, next_w_n, next_r, next_d) + ...
                  cfg.pT * V(next, next_w_t, next_r, next_d);
        
        % Subtract resource cost from value
        [cost_O, cost_H, cost_F] = move_cost(dist, w, cfg);
        exp_val = exp_val - cost_O * 0.1 - cost_H * 0.05 - cost_F * 0.1;
        
        best_val = max(best_val, exp_val);
    end
    val = best_val;
end

% ===== Evaluate Park Action =====
function val = eval_park(node, w, r, d, V, cfg)
    next_w_n = 1; next_w_t = 2;
    next_r = resource_after_park(r, w, cfg);
    next_d = day_after_move(d, 1, cfg);
    
    if next_r == 0 || next_d == 0
        val = -inf; return;
    end
    
    val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
          cfg.pT * V(node, next_w_t, next_r, next_d);
    
    % Cost of parking
    if w == 1  % normal
        val = val - 1*0.1 - 1*0.05 - 1*0.1;
    else  % thunder
        val = val - 3*0.1 - 3*0.05 - 2*0.1;
    end
end

% ===== Evaluate Work Action =====
function val = eval_work(node, w, r, d, V, cfg)
    wi = node - 2;  % work index 1..3
    yield = cfg.WY(wi);
    
    next_w_n = 1; next_w_t = 2;
    next_r = resource_after_work(r, w, cfg);
    next_d = day_after_move(d, 1, cfg);
    
    if next_r == 0 || next_d == 0
        val = -inf; return;
    end
    
    exp_val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
              cfg.pT * V(node, next_w_t, next_r, next_d);
    
    % Add yield, subtract resource cost
    if w == 1
        val = yield + exp_val - 5*0.1 - 4*0.05 - 3*0.1;
    else
        val = yield + exp_val - 8*0.1 - 6*0.05 - 6*0.1;
    end
end

% ===== Evaluate Supply Action =====
function val = eval_supply(node, w, r, d, V, cfg)
    % Supply improves resource level
    next_r = 3;  % goes to normal after supply
    next_w_n = 1; next_w_t = 2;
    next_d = day_after_move(d, 1, cfg);
    
    if next_d == 0
        val = -inf; return;
    end
    
    exp_val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
              cfg.pT * V(node, next_w_t, next_r, next_d);
    
    % Cost of supply day (park consumption + supply cost)
    if w == 1
        val = exp_val - 1*0.1 - 1*0.05 - 1*0.1 - 50*0.01;
    else
        val = exp_val - 3*0.1 - 3*0.05 - 2*0.1 - 50*0.01;
    end
end

% ===== Helper Functions =====
function ok = can_move(node, next, r, w, d, cfg)
    dist = cfg.dist(node, next);
    day_idx = (d-1)*30 + 1 + dist;
    ok = (dist > 0) && (day_idx <= 90) && (r > 1 || (r==1 && dist < 5));
end

function next_r = resource_after_move(node, next, r, w, cfg)
    dist = cfg.dist(node, next);
    % Rough check: can resources sustain the move?
    if r == 3, next_r = max(1, r - (w==2));  % normal->ok, or normal->low if thunder
    elseif r == 2, next_r = max(1, r - 1);
    else, next_r = 1;
    end
end

function next_d = day_after_move(d, dist, cfg)
    day_idx = (d-1)*30 + 1 + dist;
    if day_idx > 90, next_d = 0;
    elseif day_idx <= 30, next_d = 1;
    elseif day_idx <= 60, next_d = 2;
    else, next_d = 3;
    end
end

function next_r = resource_after_park(r, w, cfg)
    next_r = r;  % parking doesn't change resource level much
end

function next_r = resource_after_work(r, w, cfg)
    if w == 2, next_r = max(1, r-1);  % thunder work depletes
    else, next_r = r;
    end
end

function [cO, cH, cF] = move_cost(dist, w, cfg)
    if w == 1
        cO = dist * 2; cH = dist * 3; cF = dist * 2;
    else
        cO = dist * 8; cH = dist * 4; cF = dist * 3;
    end
end

% ===== Get Policy for a State =====
function [best_action, best_target, best_val] = get_policy(node, w, r, d, V, cfg)
    if nargin < 6, cfg = get_mdp_config(); end
    
    best_val = -inf;
    best_action = 2;  % default: park
    best_target = node;
    
    % Move
    for next = cfg.inter
        if ~can_move(node, next, r, w, d, cfg), continue; end
        next_w_n = 1; next_w_t = 2;
        next_r = resource_after_move(node, next, r, w, cfg);
        next_d = day_after_move(d, cfg.dist(node, next), cfg);
        if next_r == 0 || next_d == 0, continue; end
        exp_val = cfg.pN * V(next, next_w_n, next_r, next_d) + ...
                  cfg.pT * V(next, next_w_t, next_r, next_d);
        if exp_val > best_val
            best_val = exp_val;
            best_action = 1;
            best_target = next;
        end
    end
    
    % Park
    next_w_n = 1; next_w_t = 2;
    next_r = resource_after_park(r, w, cfg);
    next_d = day_after_move(d, 1, cfg);
    if next_r > 0 && next_d > 0
        exp_val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
                  cfg.pT * V(node, next_w_t, next_r, next_d);
        if exp_val > best_val
            best_val = exp_val;
            best_action = 2;
            best_target = node;
        end
    end
    
    % Work
    if node >= 3 && node <= 5
        next_r = resource_after_work(r, w, cfg);
        next_d = day_after_move(d, 1, cfg);
        wi = node - 2;
        if next_r > 0 && next_d > 0
            exp_val = cfg.WY(wi) + cfg.pN * V(node, next_w_n, next_r, next_d) + ...
                      cfg.pT * V(node, next_w_t, next_r, next_d);
            if exp_val > best_val
                best_val = exp_val;
                best_action = 3;
                best_target = node;
            end
        end
    end
    
    % Supply
    if node == 6 || node == 7
        next_r = 3;
        next_d = day_after_move(d, 1, cfg);
        if next_d > 0
            exp_val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
                      cfg.pT * V(node, next_w_t, next_r, next_d);
            if exp_val > best_val
                best_val = exp_val;
                best_action = 4;
                best_target = node;
            end
        end
    end
end
"""

write_file(DIR_B, "mdp_solver.m", MDP_SOLVER)

# ============================================================
# APPROACH B: solve_q3_mdp.m
# ============================================================
SOLVE_Q3_MDP = r"""function solve_q3_mdp()
% SOLVE_Q3_MDP  Task3 MDP Online Decision
% Uses value iteration on aggregated state space

cfg = cp_engine_v2('config');
cN = cp_engine_v2('cons', 'normal');
cT = cp_engine_v2('cons', 'thunder');
ce = cp_engine_v2('cons', 'expected');

% Pre-compute MDP value function
V = mdp_solver('solve');

% Generate weather or use provided
rng('shuffle');
weather_seq = cp_engine_v2('weather', 90, 0.8);

% Initialize state
st.pt = 1; st.pos = cfg.xy(1,:);
st.O = cfg.init.O; st.H = cfg.init.H; st.F = cfg.init.F;
st.M = cfg.init.M; st.Z = cfg.init.Z;
st.consec = 0; st.day = 0;
st.work_done = 0;

fprintf('========================================\n');
fprintf('  Task3 MDP Online Decision\n');
fprintf('========================================\n');
fprintf('Value Iteration | State Aggregation | Optimal Policy\n');
fprintf('Day  | W | Action              | Pos        | Res       |   Z    M\n');
fprintf('-----|---|---------------------|------------|-----------|---------\n');

while st.day < cfg.MAX_DAYS && st.pt ~= 2
    st.day = st.day + 1;
    w = weather_seq(st.day);
    if w == 'T', ca = cT; w_idx = 2; wn = 'T';
    else, ca = cN; w_idx = 1; wn = 'N';
    end
    
    % Determine resource level
    dE = cfg.dist(st.pt, 2);
    if st.O < dE * cT.MO || st.H < dE * cT.MH || st.F < dE * cT.MF
        r = 1;  % critical
    elseif st.O < dE * cT.MO * 1.5 || st.H < dE * cT.MH * 1.5 || st.F < dE * cT.MF * 1.5
        r = 2;  % low
    else
        r = 3;  % normal
    end
    
    % Determine day bucket
    if st.day <= 30, d = 1;
    elseif st.day <= 60, d = 2;
    else, d = 3;
    end
    
    % Get MDP policy
    [act, target, ~] = mdp_solver('policy', st.pt, w_idx, r, d, V);
    
    action_str = '';
    ik = false;
    
    switch act
        case 1  % Move toward target
            if target ~= st.pt
                % Move one step toward target
                fr = cfg.xy(st.pt, :);
                to = cfg.xy(target, :);
                dx = to(1) - fr(1); dy = to(2) - fr(2);
                if abs(dx) > 0
                    st.pos(1) = fr(1) + sign(dx);
                    st.pos(2) = fr(2);
                elseif abs(dy) > 0
                    st.pos(1) = fr(1);
                    st.pos(2) = fr(2) + sign(dy);
                end
                st.O = st.O - ca.MO; st.H = st.H - ca.MH; st.F = st.F - ca.MF;
                st.consec = 0;
                action_str = sprintf('MOVE->(%d,%d)', st.pos(1), st.pos(2));
                
                % Check arrival at target
                if st.pos(1) == to(1) && st.pos(2) == to(2)
                    st.pt = target;
                    if st.pt == 6 || st.pt == 7
                        % Supply
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
                            st.O = st.O + bO; st.H = st.H + bH; st.F = st.F + bF;
                            st.M = st.M - cost;
                            action_str = sprintf('SUPPLY(%s)', cfg.names{st.pt});
                        end
                    elseif st.pt == 2
                        action_str = 'ARRIVE!';
                    end
                end
                ik = true;
            end
            
        case 2  % Park
            st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
            st.consec = 0;
            action_str = 'PARK';
            ik = true;
            
        case 3  % Work
            if st.pt >= 3 && st.pt <= 5
                wi = st.pt - 2;
                if st.consec < cfg.WM(wi)
                    st.O = st.O - ca.WO; st.H = st.H - ca.WH; st.F = st.F - ca.WF;
                    st.Z = st.Z + cfg.WY(wi);
                    st.consec = st.consec + 1;
                    st.work_done = st.work_done + 1;
                    action_str = sprintf('WORK(%s)', cfg.names{st.pt});
                else
                    st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                    st.consec = 0;
                    action_str = 'PARK(reset)';
                end
                ik = true;
            end
            
        case 4  % Supply
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
                    st.O = st.O + bO; st.H = st.H + bH; st.F = st.F + bF;
                    st.M = st.M - cost;
                end
                action_str = 'SUPPLY';
                ik = true;
            end
    end
    
    % Check resource exhaustion
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
    fprintf('Day %d Arrived at E | Z=%d M=%.0f | Work=%d\n', st.day, st.Z, round(st.M), st.work_done);
elseif st.day >= cfg.MAX_DAYS
    fprintf('TIMEOUT | Z=%d M=%.0f\n', st.Z, round(st.M));
else
    fprintf('FAILED(Day %d) | Z=%d M=%.0f\n', st.day, st.Z, round(st.M));
end
fprintf('Done.\n');
end
"""

write_file(DIR_B, "solve_q3_mdp.m", SOLVE_Q3_MDP)

# ============================================================
# APPROACH B: solve_q3_mdp_mc.m
# ============================================================
SOLVE_Q3_MDP_MC = r"""function solve_q3_mdp_mc(N)
% SOLVE_Q3_MDP_MC  Task3 MDP MC Verification

if nargin<1||isempty(N), N=100; end
cfg = cp_engine_v2('config');
cN = cp_engine_v2('cons', 'normal');
cT = cp_engine_v2('cons', 'thunder');
ce = cp_engine_v2('cons', 'expected');

% Pre-compute MDP value function once
V = mdp_solver('solve');

fprintf('========================================\n');
fprintf('  Task3 MDP MC Verification (N=%d)\n', N);
fprintf('========================================\n\n');

Zr = NaN(1,N); Mr = NaN(1,N); ok = false(1,N); dy = NaN(1,N);
fr = cell(1,N); ri = max(1, floor(N/20)); tic;

for sim = 1:N
    ws = cp_engine_v2('weather', 90, 0.8);
    [Zf, Mf, arr, days, reason] = run_mdp_silent(ws, V);
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

function [Zf, Mf, arr, days, reason] = run_mdp_silent(wseq, V)
    cfg = cp_engine_v2('config');
    cN = cp_engine_v2('cons', 'normal');
    cT = cp_engine_v2('cons', 'thunder');
    ce = cp_engine_v2('cons', 'expected');
    
    st.pt = 1; st.pos = cfg.xy(1,:);
    st.O = cfg.init.O; st.H = cfg.init.H; st.F = cfg.init.F;
    st.M = cfg.init.M; st.Z = cfg.init.Z; st.consec = 0; st.day = 0;
    reason = '';
    
    while st.day < cfg.MAX_DAYS && st.pt ~= 2 && isempty(reason)
        st.day = st.day + 1;
        w = wseq(st.day);
        if w == 'T', ca = cT; w_idx = 2;
        else, ca = cN; w_idx = 1;
        end
        
        dE = cfg.dist(st.pt, 2);
        if st.O < dE * cT.MO || st.H < dE * cT.MH || st.F < dE * cT.MF
            r = 1;
        elseif st.O < dE * cT.MO * 1.5 || st.H < dE * cT.MH * 1.5 || st.F < dE * cT.MF * 1.5
            r = 2;
        else, r = 3;
        end
        
        if st.day <= 30, d = 1;
        elseif st.day <= 60, d = 2;
        else, d = 3;
        end
        
        [act, target, ~] = mdp_solver('policy', st.pt, w_idx, r, d, V);
        
        switch act
            case 1
                if target ~= st.pt
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
                    st.consec = 0;
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
                                st.O = st.O + bO; st.H = st.H + bH; st.F = st.F + bF;
                                st.M = st.M - cost;
                            end
                        end
                    end
                end
            case 2
                st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                st.consec = 0;
            case 3
                if st.pt >= 3 && st.pt <= 5
                    wi = st.pt - 2;
                    if st.consec < cfg.WM(wi)
                        st.O = st.O - ca.WO; st.H = st.H - ca.WH; st.F = st.F - ca.WF;
                        st.Z = st.Z + cfg.WY(wi); st.consec = st.consec + 1;
                    else
                        st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                        st.consec = 0;
                    end
                end
            case 4
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
                        st.O = st.O + bO; st.H = st.H + bH; st.F = st.F + bF;
                        st.M = st.M - cost;
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

write_file(DIR_B, "solve_q3_mdp_mc.m", SOLVE_Q3_MDP_MC)
print("Approach B: all MATLAB files written")
