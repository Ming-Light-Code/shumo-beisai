import os

BASE = r"C:\Users\ming\Desktop\任务3_最终版"
DIR_B = os.path.join(BASE, "优化B_MDP方向")

def write_file(dirpath, filename, content):
    path = os.path.join(dirpath, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Written: {filename}")

MDP_SOLVER_V3 = r"""function varargout = mdp_solver_v3(action, varargin)
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
"""

write_file(DIR_B, "mdp_solver_v3.m", MDP_SOLVER_V3)

# ============================================================
# B v3 online solver (fixes double supply trigger)
# ============================================================
SOLVE_Q3_MDP_V3 = r"""function solve_q3_mdp_v3(weather_seq)
% SOLVE_Q3_MDP_V3  Task3 MDP v3 Online (Corrected move model)

cfg=cp_engine_v2('config'); cfg_mdp=mdp_solver_v3('config');
cN=cp_engine_v2('cons','normal'); cT=cp_engine_v2('cons','thunder');
ce=cp_engine_v2('cons','expected');
V=mdp_solver_v3('solve');

if nargin<1||isempty(weather_seq)
    rng('shuffle'); weather_seq=cp_engine_v2('weather',90,0.8);
end

st.pt=1; st.pos=cfg.xy(1,:);
st.O=cfg.init.O; st.H=cfg.init.H; st.F=cfg.init.F;
st.M=cfg.init.M; st.Z=cfg.init.Z; st.consec_work=0; st.day=0;
supplied_today=false;

fprintf('========================================\n');
fprintf('  Task3 MDP v3 Online (Corrected)\n');
fprintf('========================================\n');
fprintf('Full-dist move | Consistent quant | Dist-aware init\n');
fprintf('Day  | W | Action              | Pos        | Res       |   Z    M\n');
fprintf('-----|---|---------------------|------------|-----------|---------\n');

while st.day<cfg.MAX_DAYS && st.pt~=2
    st.day=st.day+1; supplied_today=false;
    w=weather_seq(st.day);
    if w=='T', ca=cT; w_idx=2; wn='T';
    else, ca=cN; w_idx=1; wn='N';
    end
    
    r=get_res_level(st.O,st.H,st.F,cfg_mdp);
    if st.day<=30, d=1; elseif st.day<=60, d=2; else, d=3; end
    c=min(5,st.consec_work+1);
    
    [act,target,~]=mdp_solver_v3('policy',st.pt,w_idx,r,d,c,V,cfg_mdp);
    action_str=''; ik=false;
    
    switch act
        case 1  % MOVE
            if target~=st.pt
                fr=cfg.xy(st.pt,:); to=cfg.xy(target,:);
                dx=to(1)-fr(1); dy=to(2)-fr(2);
                if abs(dx)>0, st.pos(1)=fr(1)+sign(dx); st.pos(2)=fr(2);
                elseif abs(dy)>0, st.pos(1)=fr(1); st.pos(2)=fr(2)+sign(dy);
                end
                st.O=st.O-ca.MO; st.H=st.H-ca.MH; st.F=st.F-ca.MF;
                st.consec_work=0;
                action_str=sprintf('MOVE->(%d,%d)',st.pos(1),st.pos(2));
                if st.pos(1)==to(1)&&st.pos(2)==to(2)
                    st.pt=target;
                    action_str=sprintf('ARR(%s)',cfg.names{st.pt});
                    if st.pt==6||st.pt==7
                        do_supply();
                    elseif st.pt==2, action_str='ARRIVE!'; end
                end
                ik=true;
            end
        case 2  % PARK
            st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF;
            st.consec_work=0; action_str='PARK'; ik=true;
        case 3  % WORK
            if st.pt>=3&&st.pt<=5
                wi=st.pt-2;
                if st.consec_work<cfg.WM(wi)
                    st.O=st.O-ca.WO; st.H=st.H-ca.WH; st.F=st.F-ca.WF;
                    st.Z=st.Z+cfg.WY(wi); st.consec_work=st.consec_work+1;
                    action_str=sprintf('WORK(%s)',cfg.names{st.pt});
                else
                    st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF;
                    st.consec_work=0; action_str='PARK(reset)';
                end
                ik=true;
            end
        case 4  % SUPPLY (only if not already supplied today)
            if (st.pt==6||st.pt==7) && ~supplied_today
                do_supply(); ik=true;
            end
    end
    
    if st.O<-1e-6||st.H<-1e-6||st.F<-1e-6
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f ***\n',...
            st.day,wn,'EXHAUSTED!',st.pos(1),st.pos(2),st.O,st.H,st.F,st.Z,round(st.M));
        break;
    end
    if ik
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f\n',...
            st.day,wn,action_str,st.pos(1),st.pos(2),st.O,st.H,st.F,st.Z,round(st.M));
    end
end

fprintf('-----|---|---------------------|------------|-----------|---------\n');
fprintf('\n===== Result =====\n');
if st.pt==2, fprintf('Day %d Arrived | Z=%d M=%.0f\n',st.day,st.Z,round(st.M));
else, fprintf('FAIL/TIMEOUT Day %d | Z=%d\n',st.day,st.Z); end
fprintf('Done.\n');

    function do_supply()
        [nO,nH,nF]=cp_engine_v2('supply_needs_safe',[st.pt,2],[],[],1,ce,cfg);
        sp=cfg.MAX_LOAD-(st.O+st.H+st.F);
        bO=max(0,nO-st.O); bH=max(0,nH-st.H); bF=max(0,nF-st.F);
        if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
        cost=bO*ca.pO+bH*ca.pH+bF*ca.pF;
        if cost<=st.M
            st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=st.M-cost;
            action_str=sprintf('SUPPLY(%s)',cfg.names{st.pt});
        end
        supplied_today=true; st.consec_work=0;
    end
end

function r=get_res_level(O,H,F,cfg_mdp)
    if O<cfg_mdp.thresh(1)||H<cfg_mdp.thresh(1)||F<cfg_mdp.thresh_F(1), r=1;
    elseif O<cfg_mdp.thresh(2)||H<cfg_mdp.thresh(2)||F<cfg_mdp.thresh_F(2), r=2;
    elseif O<cfg_mdp.thresh(3)||H<cfg_mdp.thresh(3)||F<cfg_mdp.thresh_F(3), r=3;
    elseif O<cfg_mdp.thresh(4)||H<cfg_mdp.thresh(4)||F<cfg_mdp.thresh_F(4), r=4;
    else, r=5;
    end
end
"""

write_file(DIR_B, "solve_q3_mdp_v3.m", SOLVE_Q3_MDP_V3)
print("Approach B v3: solver + online written")
