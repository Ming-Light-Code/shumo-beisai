function varargout = mdp_solver_v3(action, varargin)
% MDP_SOLVER_V3  Task3 MDP Engine v3 (Corrected)
% Fixes over v2:
%   P0: Move uses FULL distance resource cost (not instant arrival)
%   P1: Consistent resource quantization (same thresholds both ways)
%   P1: Distance-aware value initialization
%   P2: Thresholds derived from actual requirements
%   P2: Supply triggered once per arrival, not per policy step
% States: 7x2x5x3x5 = 1050

switch action
    case 'solve',    varargout{1} = solve_MDP();
    case 'policy',   [varargout{1},varargout{2},varargout{3}] = get_policy(varargin{:});
    case 'config',   varargout{1} = get_mdp_config();
    otherwise, error('mdp_solver_v3: unknown action');
end
end

function cfg = get_mdp_config()
    cfg = cp_engine_v2('config');
    cfg.pN = 0.8;  cfg.pT = 0.2;
    cfg.gamma = 1.0;  cfg.max_iter = 400;
    % Unified resource thresholds (used for BOTH directions)
    % Derivation: 
    %   critical: < distance_to_nearest_supply * thunder_move_O (~12*8=96->60)
    %   low:      < B->W1 thunder_move_O (11*8=88->120)
    %   medium:   < W3->E thunder_move_O (32*8=256->260->180)
    %   high:     < 260 (can reach E from anywhere in thunder)
    cfg.thresh = [60, 120, 180, 260];  % O and H thresholds
    cfg.thresh_F = [50, 100, 150, 220];   % F has lower consumption
    % Level midpoints (consistent with thresholds)
    cfg.mid_O = [30, 90, 150, 220, 310];
    cfg.mid_H = [30, 90, 150, 220, 310];
    cfg.mid_F = [25, 75, 125, 185, 250];
    % Opportunity cost per travel day (Z-equivalent)
    cfg.travel_cost_per_day = 20;
end

function V = solve_MDP()
    cfg = get_mdp_config();
    nN=7; nW=2; nR=5; nD=3; nC=5;
    cN = cp_engine_v2('cons','normal');
    cT = cp_engine_v2('cons','thunder');
    
    V = zeros(nN, nW, nR, nD, nC);
    
    % Distance-aware value initialization
    for node=1:nN
        dE = cfg.dist(node, 2);
        for w=1:nW, for r=1:nR, for d=1:nD, for c=1:nC
            day_idx = (d-1)*30 + 1;
            rem = cfg.MAX_DAYS - day_idx;
            if node==2
                V(node,w,r,d,c) = 200 + (r-1)*60;  % E terminal
            elseif dE <= rem
                max_work = max(0, min(rem - dE, 20));
                V(node,w,r,d,c) = 100 + r*40 + max_work*22 - dE*cfg.travel_cost_per_day*0.3;
            else
                V(node,w,r,d,c) = 0;
            end
        end; end; end; end
    end
    
    fprintf('MDP v3: %d states, value iteration...\n', nN*nW*nR*nD*nC);
    
    for iter = 1:cfg.max_iter
        delta = 0; V_old = V;
        for node=1:nN
            if node==2, continue; end
            for w=1:nW, for r=1:nR, for d=1:nD, for c=1:nC
                best_val = -inf;
                
                % MOVE: full-distance resource cost (FIX P0)
                for next=cfg.inter
                    if next==node, continue; end
                    v = eval_move_v3(node,next,w,r,d,c,V_old,cfg,cN,cT);
                    if v > best_val, best_val = v; end
                end
                
                % PARK
                v = eval_park_v3(node,w,r,d,c,V_old,cfg,cN,cT);
                if v > best_val, best_val = v; end
                
                % WORK (only if not at max consec)
                if node>=3 && node<=5
                    wi = node-2;
                    if c <= cfg.WM(wi)  % c=1 means 0 days worked
                        v = eval_work_v3(node,w,r,d,c,V_old,cfg,cN,cT);
                        if v > best_val, best_val = v; end
                    end
                end
                
                V(node,w,r,d,c) = best_val;
                delta = max(delta, abs(best_val - V_old(node,w,r,d,c)));
            end; end; end; end
        end
        
        if mod(iter,50)==0, fprintf('  Iter %d, delta=%.4f\n',iter,delta); end
        if delta < 1e-4, fprintf('  Converged at iter %d\n',iter); break; end
    end
    fprintf('MDP v3 solved.\n');
end

% ===== MOVE: full distance cost (FIX P0) =====
function val = eval_move_v3(node, next, w, r, d, c, V, cfg, cN, cT)
    dist = cfg.dist(node, next);
    day_idx = (d-1)*30 + 1;
    if day_idx + dist > cfg.MAX_DAYS, val = -inf; return; end
    
    % Full-distance expected resource consumption
    exp_O = dist * (cfg.pN*cN.MO + cfg.pT*cT.MO);
    exp_H = dist * (cfg.pN*cN.MH + cfg.pT*cT.MH);
    exp_F = dist * (cfg.pN*cN.MF + cfg.pT*cT.MF);
    
    r_after = level_after_consume(r, exp_O, exp_H, exp_F, cfg);
    if r_after == 0, val = -inf; return; end
    
    d_after = day_bucket_after(day_idx + dist);
    if d_after == 0, val = -inf; return; end
    
    val_N = V(next, 1, r_after, d_after, 1);
    val_T = V(next, 2, r_after, d_after, 1);
    val = cfg.pN*val_N + cfg.pT*val_T - dist*cfg.travel_cost_per_day*0.15;
end

% ===== PARK =====
function val = eval_park_v3(node, w, r, d, c, V, cfg, cN, cT)
    day_idx = (d-1)*30 + 1;
    if day_idx+1 > cfg.MAX_DAYS, val = -inf; return; end
    
    if w==1, dO=cN.PO; dH=cN.PH; dF=cN.PF;
    else, dO=cT.PO; dH=cT.PH; dF=cT.PF;
    end
    
    % Park only uses park consumption (no move cost)
    r_after = level_after_consume(r, dO, dH, dF, cfg);
    if r_after==0, val=-inf; return; end
    
    d_after = day_bucket_after(day_idx+1);
    if d_after==0, val=-inf; return; end
    
    c_after = 1;  % reset consecutive work
    val_N = V(node,1,r_after,d_after,c_after);
    val_T = V(node,2,r_after,d_after,c_after);
    val = cfg.pN*val_N + cfg.pT*val_T;
end

% ===== WORK =====
function val = eval_work_v3(node, w, r, d, c, V, cfg, cN, cT)
    wi = node - 2;
    yield = cfg.WY(wi);
    
    day_idx = (d-1)*30 + 1;
    if day_idx+1 > cfg.MAX_DAYS, val=-inf; return; end
    
    if w==1, dO=cN.WO; dH=cN.WH; dF=cN.WF;
    else, dO=cT.WO; dH=cT.WH; dF=cT.WF;
    end
    
    r_after = level_after_consume(r, dO, dH, dF, cfg);
    if r_after==0, val=-inf; return; end
    
    d_after = day_bucket_after(day_idx+1);
    if d_after==0, val=-inf; return; end
    
    c_after = min(5, c+1);
    val_N = V(node,1,r_after,d_after,c_after);
    val_T = V(node,2,r_after,d_after,c_after);
    val = yield + cfg.pN*val_N + cfg.pT*val_T;
end

% ===== SUPPLY =====
function val = eval_supply_v3(node, w, r, d, c, V, cfg, cN, cT)
    day_idx = (d-1)*30 + 1;
    if day_idx+1 > cfg.MAX_DAYS, val=-inf; return; end
    
    if w==1, dO=cN.PO; dH=cN.PH; dF=cN.PF;
    else, dO=cT.PO; dH=cT.PH; dF=cT.PF;
    end
    
    % Supply restores resources significantly
    r_after = min(5, r+2);
    d_after = day_bucket_after(day_idx+1);
    if d_after==0, val=-inf; return; end
    
    c_after = 1;
    val_N = V(node,1,r_after,d_after,c_after);
    val_T = V(node,2,r_after,d_after,c_after);
    val = cfg.pN*val_N + cfg.pT*val_T - 50*0.05;  % supply money cost
end

% ===== Consistent level-after-consume (FIX P1) =====
function r2 = level_after_consume(r, dO, dH, dF, cfg)
    if r<1||r>5, r2=0; return; end
    rem_O = cfg.mid_O(r) - dO;
    rem_H = cfg.mid_H(r) - dH;
    rem_F = cfg.mid_F(r) - dF;
    if rem_O<0||rem_H<0||rem_F<0, r2=0; return; end
    r2 = quantity_to_level(rem_O, rem_H, rem_F, cfg);
end

function r2 = quantity_to_level(O, H, F, cfg)
    for lvl=5:-1:1
        if O>=cfg.thresh(min(lvl,4)) && H>=cfg.thresh(min(lvl,4)) && F>=cfg.thresh_F(min(lvl,4))
            r2=lvl; return;
        end
    end
    r2=1;
end

function db = day_bucket_after(day)
    if day>90, db=0;
    elseif day<=30, db=1;
    elseif day<=60, db=2;
    else, db=3;
    end
end

% ===== Policy query =====
function [best_action, best_target, best_val] = get_policy(node, w, r, d, c, V, cfg)
    if nargin<7, cfg=get_mdp_config(); end
    cN=cp_engine_v2('cons','normal'); cT=cp_engine_v2('cons','thunder');
    best_val=-inf; best_action=2; best_target=node;
    
    for next=cfg.inter
        if next==node, continue; end
        v=eval_move_v3(node,next,w,r,d,c,V,cfg,cN,cT);
        if v>best_val, best_val=v; best_action=1; best_target=next; end
    end
    v=eval_park_v3(node,w,r,d,c,V,cfg,cN,cT);
    if v>best_val, best_val=v; best_action=2; best_target=node; end
    if node>=3&&node<=5
        wi=node-2;
        if c<=cfg.WM(wi)
            v=eval_work_v3(node,w,r,d,c,V,cfg,cN,cT);
            if v>best_val, best_val=v; best_action=3; best_target=node; end
        end
    end
    if node==6||node==7
        v=eval_supply_v3(node,w,r,d,c,V,cfg,cN,cT);
        if v>best_val, best_val=v; best_action=4; best_target=node; end
    end
end
