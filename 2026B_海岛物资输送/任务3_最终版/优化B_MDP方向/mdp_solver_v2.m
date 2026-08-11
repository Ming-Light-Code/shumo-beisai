function varargout = mdp_solver_v2(action, varargin)
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
