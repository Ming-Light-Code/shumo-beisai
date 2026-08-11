function varargout = rso_solver_v4(action, varargin)
% RSO_SOLVER_V4  Task3 RSO Engine v4 (Full-distance move + fixes)
% Fixes over v3:
%   P0: sim_move_FULL_then_tail: simulate entire move to target node
%       -> work points ARE reached in tail, capturing their value
%   P1: supply uses actual per-resource consumption (not cp/3)
%   P1: equal weights (conservative bias is desirable for safety)
%   P2: tail simulation unchanged (day-by-day actual weather)

switch action
    case 'step',   [varargout{1},varargout{2}] = rso_step_v4(varargin{:});
    case 'config', varargout{1} = get_rso_config_v4();
    otherwise, error('rso_solver_v4: unknown');
end
end

function cfg = get_rso_config_v4()
    cfg.H = 24;
    cfg.K_total = 48;
    cfg.K_thunder = 16;
    cfg.pN = 0.8;
end

% ===== Stratified sampling (equal weights) =====
function scenarios = sample_scenarios_v4(H, K_total, K_thunder, pN)
    scenarios = cell(1, K_total);
    for k = 1:K_thunder
        ws = repmat('N',1,H);
        for i=1:H, if rand()>0.6, ws(i)='T'; end; end
        scenarios{k}=ws;
    end
    for k=(K_thunder+1):K_total
        ws = repmat('N',1,H);
        for i=1:H, if rand()>pN, ws(i)='T'; end; end
        scenarios{k}=ws;
    end
end

% ===== RSO step (v4: full-distance move) =====
function [best_action,best_value] = rso_step_v4(cur_pt,elapsed,state,cfg_cp,H,K_total,K_thunder)
    if nargin<6, rc=get_rso_config_v4(); H=rc.H; K_total=rc.K_total; K_thunder=rc.K_thunder; end
    scenarios = sample_scenarios_v4(H,K_total,K_thunder);
    inter_nodes = [3 4 5 6 7];
    action_values = {};
    
    % 1. MOVE: full-distance simulation to target (FIX P0)
    for ni=1:length(inter_nodes)
        target=inter_nodes(ni);
        if target==cur_pt, continue; end
        d=cfg_cp.dist(cur_pt,target);
        if elapsed+d>cfg_cp.MAX_DAYS, continue; end
        
        vsum=0; valid=0;
        for k=1:K_total
            val=sim_move_full_then_tail(cur_pt,target,elapsed,state,scenarios{k},cfg_cp);
            if val>-inf, vsum=vsum+val; valid=valid+1; end
        end
        if valid>K_total/6
            action_values{end+1}=struct('type','MOVE','target',target,'value',vsum/valid);
        end
    end
    
    % 2. PARK
    vsum=0; valid=0;
    for k=1:K_total
        val=sim_park_then_tail(cur_pt,elapsed,state,scenarios{k},cfg_cp);
        if val>-inf, vsum=vsum+val; valid=valid+1; end
    end
    if valid>K_total/6
        action_values{end+1}=struct('type','PARK','target',cur_pt,'value',vsum/valid);
    end
    
    % 3. WORK
    if cur_pt>=3&&cur_pt<=5
        vsum=0; valid=0;
        for k=1:K_total
            val=sim_work_then_tail(cur_pt,elapsed,state,scenarios{k},cfg_cp);
            if val>-inf, vsum=vsum+val; valid=valid+1; end
        end
        if valid>K_total/6
            action_values{end+1}=struct('type','WORK','target',cur_pt,'value',vsum/valid);
        end
    end
    
    % 4. SUPPLY
    if cur_pt==6||cur_pt==7
        vsum=0; valid=0;
        for k=1:K_total
            val=sim_supply_then_tail(cur_pt,elapsed,state,scenarios{k},cfg_cp);
            if val>-inf, vsum=vsum+val; valid=valid+1; end
        end
        if valid>K_total/6
            action_values{end+1}=struct('type','SUPPLY','target',cur_pt,'value',vsum/valid);
        end
    end
    
    if isempty(action_values)
        best_action=struct('type','PARK','target',cur_pt); best_value=-inf; return;
    end
    [~,idx]=max(cellfun(@(x)x.value, action_values));
    best_action=action_values{idx}; best_value=best_action.value;
end

% ===== FULL-DISTANCE move simulation (FIX P0) =====
function val=sim_move_full_then_tail(cur_pt,target,elapsed,state,scenario,cfg)
    dist=cfg.dist(cur_pt,target);
    if elapsed+dist>cfg.MAX_DAYS, val=-inf; return; end
    if dist>length(scenario), val=0; return; end
    
    sim_O=state.O; sim_H=state.H; sim_F=state.F;
    sim_M=state.M; sim_Z=state.Z;
    
    % Simulate ALL dist steps of the move with actual scenario weather
    for step=1:dist
        w=scenario(step);
        if w=='T'
            sim_O=sim_O-8; sim_H=sim_H-4; sim_F=sim_F-3;
        else
            sim_O=sim_O-2; sim_H=sim_H-3; sim_F=sim_F-2;
        end
        if sim_O<0||sim_H<0||sim_F<0, val=-inf; return; end
    end
    
    % Arrived at target node -> tail simulation FROM target
    rem_scen=scenario(dist+1:min(end,cfg.MAX_DAYS-elapsed));
    ns=struct('O',sim_O,'H',sim_H,'F',sim_F,'M',sim_M,'Z',sim_Z);
    val=tail_simulate(target,elapsed+dist,ns,rem_scen,cfg,0);
end

function val=sim_park_then_tail(cur_pt,elapsed,state,scenario,cfg)
    w1=scenario(1);
    if w1=='T', cO=3;cH=3;cF=2; else, cO=1;cH=1;cF=1; end
    nO=state.O-cO; nH=state.H-cH; nF=state.F-cF;
    if nO<0||nH<0||nF<0, val=-inf; return; end
    ns=struct('O',nO,'H',nH,'F',nF,'M',state.M,'Z',state.Z);
    rem=scenario(2:min(end,cfg.MAX_DAYS-elapsed));
    val=tail_simulate(cur_pt,elapsed+1,ns,rem,cfg,0);
end

function val=sim_work_then_tail(cur_pt,elapsed,state,scenario,cfg)
    w1=scenario(1); wi=cur_pt-2;
    if w1=='T', cO=8;cH=6;cF=6; else, cO=5;cH=4;cF=3; end
    nO=state.O-cO; nH=state.H-cH; nF=state.F-cF;
    if nO<0||nH<0||nF<0, val=-inf; return; end
    ns=struct('O',nO,'H',nH,'F',nF,'M',state.M,'Z',state.Z+cfg.WY(wi));
    rem=scenario(2:min(end,cfg.MAX_DAYS-elapsed));
    val=tail_simulate(cur_pt,elapsed+1,ns,rem,cfg,1);
end

% ===== Supply: FIXED cp/3 -> per-resource consumption =====
function val=sim_supply_then_tail(cur_pt,elapsed,state,scenario,cfg)
    w1=scenario(1); ce=cp_engine_v2('cons','expected');
    if w1=='T', pO=3;pH=3;pF=2; else, pO=1;pH=1;pF=1; end
    
    dE_sup=cfg.dist(cur_pt,2); cT_sup=cp_engine_v2('cons','thunder');
    nO=dE_sup*(0.7*ce.MO+0.3*cT_sup.MO);
    nH=dE_sup*(0.7*ce.MH+0.3*cT_sup.MH);
    nF=dE_sup*(0.7*ce.MF+0.3*cT_sup.MF);
    sp=cfg.MAX_LOAD-(state.O+state.H+state.F);
    bO=max(0,nO-state.O); bH=max(0,nH-state.H); bF=max(0,nF-state.F);
    if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
    cost=bO*2+bH*1+bF*2;
    if cost>state.M, val=-inf; return; end
    
    % FIXED: deduct actual per-resource park consumption
    ns=struct('O',state.O+bO-pO,'H',state.H+bH-pH,'F',state.F+bF-pF,...
             'M',state.M-cost,'Z',state.Z);
    rem=scenario(2:min(end,cfg.MAX_DAYS-elapsed));
    val=tail_simulate(cur_pt,elapsed+1,ns,rem,cfg,0);
end

% ===== TAIL SIMULATION: day-by-day with actual weather =====
function val=tail_simulate(cur_pt,elapsed,state,scenario,cfg,consec_work)
    rem=min(length(scenario),cfg.MAX_DAYS-elapsed);
    if rem<=0, val=iif(cur_pt==2,state.Z,0); return; end
    
    cN=cp_engine_v2('cons','normal'); cT=cp_engine_v2('cons','thunder');
    ce=cp_engine_v2('cons','expected');
    
    sim_O=state.O; sim_H=state.H; sim_F=state.F;
    sim_M=state.M; sim_Z=state.Z;
    sim_pt=cur_pt; sim_pos=cfg.xy(cur_pt,:);
    cw=consec_work; to_E=cfg.xy(2,:);
    
    for t=1:rem
        if sim_pt==2, break; end
        w=scenario(t);
        if w=='T', ca=cT; else, ca=cN; end
        
        action_taken=false;
        
        % 1. WORK at work point
        if sim_pt>=3&&sim_pt<=5
            wi=sim_pt-2;
            if cw<cfg.WM(wi)
                sim_O=sim_O-ca.WO; sim_H=sim_H-ca.WH; sim_F=sim_F-ca.WF;
                sim_Z=sim_Z+cfg.WY(wi); cw=cw+1; action_taken=true;
            end
        end
        
        % 2. SUPPLY at supply point if resources low
        if ~action_taken && (sim_pt==6||sim_pt==7)
            dE=cfg.dist(sim_pt,2);
            if sim_O<dE*cT.MO||sim_H<dE*cT.MH||sim_F<dE*cT.MF
                dE_sup2=cfg.dist(sim_pt,2); cT_sup2=cp_engine_v2('cons','thunder');
                nO=dE_sup2*(0.7*ce.MO+0.3*cT_sup2.MO);
                nH=dE_sup2*(0.7*ce.MH+0.3*cT_sup2.MH);
                nF=dE_sup2*(0.7*ce.MF+0.3*cT_sup2.MF);
                sp=cfg.MAX_LOAD-(sim_O+sim_H+sim_F);
                bO=max(0,nO-sim_O); bH=max(0,nH-sim_H); bF=max(0,nF-sim_F);
                if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
                cost=bO*2+bH*1+bF*2;
                if cost<=sim_M
                    sim_O=sim_O+bO; sim_H=sim_H+bH; sim_F=sim_F+bF; sim_M=sim_M-cost;
                end
                cw=0; action_taken=true;
            end
        end
        
        % 3. PARK during thunder (not at W/S)
        if ~action_taken && w=='T' && sim_pt~=6&&sim_pt~=7 && ~(sim_pt>=3&&sim_pt<=5)
            sim_O=sim_O-ca.PO; sim_H=sim_H-ca.PH; sim_F=sim_F-ca.PF;
            cw=0; action_taken=true;
        end
        
        % 4. MOVE: toward E, or toward nearest supply if resources low
        if ~action_taken
            dE_chk=cfg.dist(sim_pt,2);
            need_supply=(sim_O<dE_chk*cT.MO||sim_H<dE_chk*cT.MH||sim_F<dE_chk*cT.MF);
            if need_supply && sim_pt~=6 && sim_pt~=7
                % Head to nearest supply point
                dS1=cfg.dist(sim_pt,6); dS2=cfg.dist(sim_pt,7);
                if dS1<=dS2, tgt=cfg.xy(6,:); else, tgt=cfg.xy(7,:); end
            else
                tgt=to_E;
            end
            dx=tgt(1)-sim_pos(1); dy=tgt(2)-sim_pos(2);
            if abs(dx)>0, sim_pos(1)=sim_pos(1)+sign(dx);
            elseif abs(dy)>0, sim_pos(2)=sim_pos(2)+sign(dy);
            end
            sim_O=sim_O-ca.MO; sim_H=sim_H-ca.MH; sim_F=sim_F-ca.MF; cw=0;
            for nd=1:7
                if sim_pos(1)==cfg.xy(nd,1)&&sim_pos(2)==cfg.xy(nd,2)
                    sim_pt=nd; break;
                end
            end
        end
        
        if sim_O<-1e-6||sim_H<-1e-6||sim_F<-1e-6, val=0; return; end
        if sim_O+sim_H+sim_F>cfg.MAX_LOAD+1e-6, val=0; return; end
    end
    val=iif(sim_pt==2,sim_Z,0);
end

function v=iif(cond,tval,fval)
    if cond, v=tval; else, v=fval; end
end
