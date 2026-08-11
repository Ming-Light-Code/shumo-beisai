import os

BASE = r"C:\Users\ming\Desktop\任务3_最终版"
DIR_B = os.path.join(BASE, "优化B_MDP方向")

def write_file(dirpath, filename, content):
    path = os.path.join(dirpath, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Written: {filename}")

# ============================================================
# APPROACH B OPTIMIZED: mdp_solver_v2.m
# Key improvements:
#   1. Resource: 5 levels (critical/low/medium/high/full) instead of 3
#   2. Consec work: 5 levels (0,1,2,3,4+) tracked
#   3. Lower-bound V init instead of optimistic 500
#   4. Accurate resource transition using actual consumption parameters
#   5. Supply action: uses safe-supply-needs computation
# ============================================================
MDP_SOLVER_V2 = r"""function varargout = mdp_solver_v2(action, varargin)
% MDP_SOLVER_V2  Task3 MDP Engine v2 (Optimized)
% States: (node, weather, res_level, day_bucket, consec_work)
%   node: 1-7 (B,E,W1,W2,W3,S1,S2)
%   weather: 1=N, 2=T
%   res_level: 1=critical, 2=low, 3=medium, 4=high, 5=full
%   day_bucket: 1=early(1-30), 2=mid(31-60), 3=late(61-90)
%   consec_work: 1=0, 2=1, 3=2, 4=3, 5=4+ days
% Total: 7x2x5x3x5 = 1050 states

switch action
    case 'solve',    varargout{1} = solve_MDP();
    case 'policy',   [varargout{1},varargout{2},varargout{3}] = get_policy(varargin{:});
    case 'config',   varargout{1} = get_mdp_config();
    otherwise, error('mdp_solver_v2: unknown action');
end
end

function cfg = get_mdp_config()
    cfg = cp_engine_v2('config');
    cfg.pN = 0.8;  cfg.pT = 0.2;
    cfg.gamma = 1.0;
    cfg.max_iter = 300;
    % Resource thresholds (based on consumption to reach E from farthest node)
    % W3->E = 32d, thunder move: O=256, expected: O=102
    cfg.thresh_O = [60, 120, 180, 260];   % critical/low/medium/high
    cfg.thresh_H = [60, 120, 180, 260];
    cfg.thresh_F = [50, 100, 150, 220];
end

% ===== Main MDP Solver (v2) =====
function V = solve_MDP()
    cfg = get_mdp_config();
    nN=7; nW=2; nR=5; nD=3; nC=5;
    nStates = nN*nW*nR*nD*nC;
    
    V = zeros(nN, nW, nR, nD, nC);
    
    % Initialize terminal states at E with lower-bound achievable Z
    % Basic path B->W1->S1->W2->W3->S2->E max Z=492 with good weather
    for w=1:nW, for r=1:nR, for d=1:nD, for c=1:nC
        V(2,w,r,d,c) = 200 + r*50;  % ~200-450 depending on resource level
    end; end; end; end
    
    % Initialize non-terminal states optimistically but bounded
    for node=1:nN
        if node==2, continue; end
        for w=1:nW, for r=1:nR, for d=1:nD, for c=1:nC
            dist_to_E = cfg.dist(node, 2);
            max_remain = cfg.MAX_DAYS - (d-1)*30;
            if dist_to_E <= max_remain
                % Heuristic: max Z = current_res + max_work_yield
                max_work = min(max_remain - dist_to_E, 20);
                V(node,w,r,d,c) = 100 + r*40 + max_work*25;
            else
                V(node,w,r,d,c) = 0;
            end
        end; end; end; end
    end
    
    cN = cp_engine_v2('cons','normal');
    cT = cp_engine_v2('cons','thunder');
    
    fprintf('MDP v2: %d states, running value iteration...\n', nStates);
    
    for iter = 1:cfg.max_iter
        delta = 0;
        V_old = V;
        
        for node = 1:nN
            if node == 2, continue; end
            for w = 1:nW
                for r = 1:nR
                    for d = 1:nD
                        for c = 1:nC
                            best_val = -inf;
                            
                            % Action: MOVE toward each reachable node
                            for next = cfg.inter
                                if next == node, continue; end
                                v = eval_move_v2(node, next, w, r, d, c, V_old, cfg, cN, cT);
                                if v > best_val, best_val = v; end
                            end
                            
                            % Action: PARK
                            v = eval_park_v2(node, w, r, d, c, V_old, cfg, cN, cT);
                            if v > best_val, best_val = v; end
                            
                            % Action: WORK (at W nodes, if not at max consec)
                            if node >= 3 && node <= 5 && c < 5
                                v = eval_work_v2(node, w, r, d, c, V_old, cfg, cN, cT);
                                if v > best_val, best_val = v; end
                            end
                            
                            % Action: SUPPLY (at S nodes)
                            if node == 6 || node == 7
                                v = eval_supply_v2(node, w, r, d, c, V_old, cfg, cN, cT);
                                if v > best_val, best_val = v; end
                            end
                            
                            V(node,w,r,d,c) = best_val;
                            delta = max(delta, abs(V(node,w,r,d,c) - V_old(node,w,r,d,c)));
                        end
                    end
                end
            end
        end
        
        if mod(iter,30)==0
            fprintf('  Iter %d, delta=%.4f\n', iter, delta);
        end
        if delta < 1e-4, fprintf('  Converged at iter %d\n', iter); break; end
    end
    fprintf('MDP v2 solved.\n');
end

% ===== Evaluate Move (v2: accurate consumption) =====
function val = eval_move_v2(node, next, w, r, d, c, V, cfg, cN, cT)
    dist = cfg.dist(node, next);
    day_idx = (d-1)*30 + 1;
    if day_idx + dist > cfg.MAX_DAYS, val = -inf; return; end
    
    % Resource consumption for one move step
    if w == 1  % normal
        dO = cN.MO; dH = cN.MH; dF = cN.MF;
    else  % thunder
        dO = cT.MO; dH = cT.MH; dF = cT.MF;
    end
    
    % Resource level after move (one step toward target, not full arrival)
    r_after = resource_after_consume(r, dO, dH, dF, cfg);
    if r_after == 0, val = -inf; return; end
    
    d_after = day_bucket_after(day_idx + 1);
    if d_after == 0, val = -inf; return; end
    
    % Weather transitions
    val_N = V(next, 1, r_after, d_after, 1);  % work reset on move
    val_T = V(next, 2, r_after, d_after, 1);
    
    % NOTE: Simplified - assumes instant arrival at target.
    % Real transition would take dist steps with weather changes.
    % We compensate by using appropriate resource level change.
    val = cfg.pN * val_N + cfg.pT * val_T;
    
    % Subtract approximate resource cost
    val = val - dO*0.15 - dH*0.10 - dF*0.15;
end

% ===== Evaluate Park (v2) =====
function val = eval_park_v2(node, w, r, d, c, V, cfg, cN, cT)
    day_idx = (d-1)*30 + 1;
    if day_idx + 1 > cfg.MAX_DAYS, val = -inf; return; end
    
    if w == 1
        dO = cN.PO; dH = cN.PH; dF = cN.PF;
    else
        dO = cT.PO; dH = cT.PH; dF = cT.PF;
    end
    
    r_after = resource_after_consume(r, dO, dH, dF, cfg);
    if r_after == 0, val = -inf; return; end
    
    d_after = day_bucket_after(day_idx + 1);
    if d_after == 0, val = -inf; return; end
    
    % Park resets consecutive work if we were working
    c_after = 1;  % reset to 0 work days
    
    val_N = V(node, 1, r_after, d_after, c_after);
    val_T = V(node, 2, r_after, d_after, c_after);
    
    val = cfg.pN * val_N + cfg.pT * val_T;
    val = val - dO*0.12 - dH*0.08 - dF*0.12;
end

% ===== Evaluate Work (v2: track consec, check WM limit) =====
function val = eval_work_v2(node, w, r, d, c, V, cfg, cN, cT)
    wi = node - 2;
    max_consec = cfg.WM(wi);
    yield = cfg.WY(wi);
    
    % Check if we can work another consecutive day
    consec_count = c;  % c=1 means 0 days, c=2 means 1 day, etc.
    if consec_count > max_consec, val = -inf; return; end
    
    day_idx = (d-1)*30 + 1;
    if day_idx + 1 > cfg.MAX_DAYS, val = -inf; return; end
    
    if w == 1
        dO = cN.WO; dH = cN.WH; dF = cN.WF;
    else
        dO = cT.WO; dH = cT.WH; dF = cT.WF;
    end
    
    r_after = resource_after_consume(r, dO, dH, dF, cfg);
    if r_after == 0, val = -inf; return; end
    
    d_after = day_bucket_after(day_idx + 1);
    if d_after == 0, val = -inf; return; end
    
    c_after = min(5, c + 1);  % increment consec work
    
    val_N = V(node, 1, r_after, d_after, c_after);
    val_T = V(node, 2, r_after, d_after, c_after);
    
    val = yield + cfg.pN * val_N + cfg.pT * val_T;
    val = val - dO*0.12 - dH*0.08 - dF*0.12;
end

% ===== Evaluate Supply (v2: improves resource level) =====
function val = eval_supply_v2(node, w, r, d, c, V, cfg, cN, cT)
    day_idx = (d-1)*30 + 1;
    if day_idx + 1 > cfg.MAX_DAYS, val = -inf; return; end
    
    if w == 1
        dO = cN.PO; dH = cN.PH; dF = cN.PF;
    else
        dO = cT.PO; dH = cT.PH; dF = cT.PF;
    end
    
    % Supply improves resource level significantly
    r_after = min(5, r + 2);  % jump 2 levels after supply
    
    d_after = day_bucket_after(day_idx + 1);
    if d_after == 0, val = -inf; return; end
    
    c_after = 1;  % reset consecutive work
    
    val_N = V(node, 1, r_after, d_after, c_after);
    val_T = V(node, 2, r_after, d_after, c_after);
    
    val = cfg.pN * val_N + cfg.pT * val_T;
    % Supply cost: resource consumption + money cost
    val = val - dO*0.12 - dH*0.08 - dF*0.12 - 80*0.02;
end

% ===== Helper: resource level after consumption =====
function r2 = resource_after_consume(r, dO, dH, dF, cfg)
    % Map resource level to approximate quantities
    O_vals = [30, 80, 130, 200, 300];
    H_vals = [30, 80, 130, 200, 300];
    F_vals = [25, 60, 100, 160, 240];
    
    if r < 1 || r > 5, r2 = 0; return; end
    
    rem_O = O_vals(r) - dO;
    rem_H = H_vals(r) - dH;
    rem_F = F_vals(r) - dF;
    
    if rem_O < 0 || rem_H < 0 || rem_F < 0
        r2 = 0; return;
    end
    
    % Determine new level based on minimum resource
    for lvl = 5:-1:1
        th_O = [0, 15, 50, 100, 180];
        th_H = [0, 15, 50, 100, 180];
        th_F = [0, 10, 35, 70, 130];
        if rem_O >= th_O(lvl) && rem_H >= th_H(lvl) && rem_F >= th_F(lvl)
            r2 = lvl; return;
        end
    end
    r2 = 1;
end

function db = day_bucket_after(day)
    if day > 90, db = 0;
    elseif day <= 30, db = 1;
    elseif day <= 60, db = 2;
    else, db = 3;
    end
end

% ===== Get Policy for a State (v2) =====
function [best_action, best_target, best_val] = get_policy(node, w, r, d, c, V, cfg)
    if nargin < 7, cfg = get_mdp_config(); end
    cN = cp_engine_v2('cons','normal');
    cT = cp_engine_v2('cons','thunder');
    
    best_val = -inf;
    best_action = 2;  % default: park
    best_target = node;
    
    % Evaluate MOVE to each node
    for next = cfg.inter
        if next == node, continue; end
        v = eval_move_v2(node, next, w, r, d, c, V, cfg, cN, cT);
        if v > best_val
            best_val = v; best_action = 1; best_target = next;
        end
    end
    
    % Evaluate PARK
    v = eval_park_v2(node, w, r, d, c, V, cfg, cN, cT);
    if v > best_val
        best_val = v; best_action = 2; best_target = node;
    end
    
    % Evaluate WORK
    if node >= 3 && node <= 5 && c < 5
        v = eval_work_v2(node, w, r, d, c, V, cfg, cN, cT);
        if v > best_val
            best_val = v; best_action = 3; best_target = node;
        end
    end
    
    % Evaluate SUPPLY
    if node == 6 || node == 7
        v = eval_supply_v2(node, w, r, d, c, V, cfg, cN, cT);
        if v > best_val
            best_val = v; best_action = 4; best_target = node;
        end
    end
end
"""

write_file(DIR_B, "mdp_solver_v2.m", MDP_SOLVER_V2)

# ============================================================
# APPROACH B OPTIMIZED: solve_q3_mdp_v2.m
# ============================================================
SOLVE_Q3_MDP_V2 = r"""function solve_q3_mdp_v2(weather_seq)
% SOLVE_Q3_MDP_V2  Task3 MDP Online Decision v2 (Optimized)
% Uses 1050-state MDP with resource tracking + consec work

cfg = cp_engine_v2('config');
cN = cp_engine_v2('cons', 'normal');
cT = cp_engine_v2('cons', 'thunder');
ce = cp_engine_v2('cons', 'expected');
cfg_mdp = mdp_solver_v2('config');

% Pre-compute MDP value function
V = mdp_solver_v2('solve');

if nargin < 1 || isempty(weather_seq)
    rng('shuffle'); weather_seq = cp_engine_v2('weather', 90, 0.8);
end

st.pt = 1; st.pos = cfg.xy(1,:);
st.O = cfg.init.O; st.H = cfg.init.H; st.F = cfg.init.F;
st.M = cfg.init.M; st.Z = cfg.init.Z;
st.consec = 0; st.day = 0; st.consec_work = 0;

fprintf('========================================\n');
fprintf('  Task3 MDP v2 Online (Optimized)\n');
fprintf('========================================\n');
fprintf('1050 states | 5 res levels | consec work tracking\n');
fprintf('Day  | W | Action              | Pos        | Res       |   Z    M\n');
fprintf('-----|---|---------------------|------------|-----------|---------\n');

while st.day < cfg.MAX_DAYS && st.pt ~= 2
    st.day = st.day + 1;
    w = weather_seq(st.day);
    if w == 'T', ca = cT; w_idx = 2; wn = 'T';
    else, ca = cN; w_idx = 1; wn = 'N';
    end
    
    % Determine resource level (5-level)
    r = get_res_level(st.O, st.H, st.F, cfg_mdp);
    
    % Day bucket
    if st.day <= 30, d = 1;
    elseif st.day <= 60, d = 2;
    else, d = 3;
    end
    
    % Consec work level
    c = min(5, st.consec_work + 1);
    
    % Get MDP policy
    [act, target, ~] = mdp_solver_v2('policy', st.pt, w_idx, r, d, c, V, cfg_mdp);
    
    action_str = '';
    ik = false;
    
    switch act
        case 1  % Move toward target
            if target ~= st.pt
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
            end
            
        case 2  % Park
            st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
            st.consec = 0; st.consec_work = 0;
            action_str = 'PARK';
            ik = true;
            
        case 3  % Work
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
                    st.O = st.O + bO; st.H = st.H + bH;
                    st.F = st.F + bF; st.M = st.M - cost;
                end
                st.consec_work = 0;
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
    fprintf('Day %d Arrived at E | Z=%d M=%.0f | Work=%d\n', st.day, st.Z, round(st.M), st.consec_work);
elseif st.day >= cfg.MAX_DAYS
    fprintf('TIMEOUT | Z=%d M=%.0f\n', st.Z, round(st.M));
else
    fprintf('FAILED(Day %d) | Z=%d M=%.0f\n', st.day, st.Z, round(st.M));
end
fprintf('Done.\n');
end

function r = get_res_level(O, H, F, cfg_mdp)
    % Map actual resources to 5-level discretization
    if O < cfg_mdp.thresh_O(1) || H < cfg_mdp.thresh_H(1) || F < cfg_mdp.thresh_F(1)
        r = 1;  % critical
    elseif O < cfg_mdp.thresh_O(2) || H < cfg_mdp.thresh_H(2) || F < cfg_mdp.thresh_F(2)
        r = 2;  % low
    elseif O < cfg_mdp.thresh_O(3) || H < cfg_mdp.thresh_H(3) || F < cfg_mdp.thresh_F(3)
        r = 3;  % medium
    elseif O < cfg_mdp.thresh_O(4) || H < cfg_mdp.thresh_H(4) || F < cfg_mdp.thresh_F(4)
        r = 4;  % high
    else
        r = 5;  % full
    end
end
"""

write_file(DIR_B, "solve_q3_mdp_v2.m", SOLVE_Q3_MDP_V2)

# ============================================================
# APPROACH B OPTIMIZED: solve_q3_mdp_mc_v2.m
# Simplified MC - reuses run_mdp_silent pattern
# ============================================================
SOLVE_Q3_MDP_MC_V2 = r"""function solve_q3_mdp_mc_v2(N)
% SOLVE_Q3_MDP_MC_V2  Task3 MDP v2 MC Verification

if nargin<1||isempty(N), N=100; end
cfg = cp_engine_v2('config');
cN = cp_engine_v2('cons', 'normal');
cT = cp_engine_v2('cons', 'thunder');
ce = cp_engine_v2('cons', 'expected');
cfg_mdp = mdp_solver_v2('config');
V = mdp_solver_v2('solve');

fprintf('========================================\n');
fprintf('  Task3 MDP v2 MC Verification (N=%d)\n', N);
fprintf('========================================\n\n');

Zr = NaN(1,N); Mr = NaN(1,N); ok = false(1,N); dy = NaN(1,N);
fr = cell(1,N); ri = max(1, floor(N/20)); tic;

for sim = 1:N
    ws = cp_engine_v2('weather', 90, 0.8);
    [Zf, Mf, arr, days, reason] = run_mdp_silent_v2(ws, V, cfg_mdp);
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

function [Zf, Mf, arr, days, reason] = run_mdp_silent_v2(wseq, V, cfg_mdp)
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
        w_idx = 1; if w == 'T', w_idx = 2; end
        ca = cN; if w == 'T', ca = cT; end
        
        % Get resource level
        r = 1;
        if st.O >= cfg_mdp.thresh_O(1) && st.H >= cfg_mdp.thresh_H(1) && st.F >= cfg_mdp.thresh_F(1), r = 2; end
        if st.O >= cfg_mdp.thresh_O(2) && st.H >= cfg_mdp.thresh_H(2) && st.F >= cfg_mdp.thresh_F(2), r = 3; end
        if st.O >= cfg_mdp.thresh_O(3) && st.H >= cfg_mdp.thresh_H(3) && st.F >= cfg_mdp.thresh_F(3), r = 4; end
        if st.O >= cfg_mdp.thresh_O(4) && st.H >= cfg_mdp.thresh_H(4) && st.F >= cfg_mdp.thresh_F(4), r = 5; end
        
        if st.day <= 30, d = 1;
        elseif st.day <= 60, d = 2;
        else, d = 3;
        end
        
        c = min(5, st.consec_work + 1);
        
        [act, target, ~] = mdp_solver_v2('policy', st.pt, w_idx, r, d, c, V, cfg_mdp);
        
        switch act
            case 1
                if target ~= st.pt
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
                end
            case 2
                st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                st.consec = 0; st.consec_work = 0;
            case 3
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

write_file(DIR_B, "solve_q3_mdp_mc_v2.m", SOLVE_Q3_MDP_MC_V2)
print("Approach B v2: all optimized MATLAB files written")
