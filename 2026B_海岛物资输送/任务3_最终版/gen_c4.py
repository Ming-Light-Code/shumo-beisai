import os

BASE = r"C:\Users\ming\Desktop\任务3_最终版"
DIR_C = os.path.join(BASE, "优化C_滚动随机优化")

def write_file(dirpath, filename, content):
    path = os.path.join(dirpath, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Written: {filename}")

RSO_SOLVER_V4 = r"""function varargout = rso_solver_v4(action, varargin)
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
    
    [nO,nH,nF]=cp_engine_v2('supply_needs_safe',[cur_pt,2],[],[],1,ce,cfg);
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
                [nO,nH,nF]=cp_engine_v2('supply_needs_safe',[sim_pt,2],[],[],1,ce,cfg);
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
        
        % 4. MOVE toward E
        if ~action_taken
            dx=to_E(1)-sim_pos(1); dy=to_E(2)-sim_pos(2);
            if abs(dx)>0, sim_pos(1)=sim_pos(1)+sign(dx);
            elseif abs(dy)>0, sim_pos(2)=sim_pos(2)+sign(dy);
            end
            sim_O=sim_O-ca.MO; sim_H=sim_H-ca.MH; sim_F=sim_F-ca.MF; cw=0;
            % Check arrival at any node
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
"""

write_file(DIR_C, "rso_solver_v4.m", RSO_SOLVER_V4)

# C v4 online (uses v4 solver, identical to v3 online but calls v4)
SOLVE_Q3_RSO_V4 = r"""function solve_q3_rso_v4(weather_seq)
% SOLVE_Q3_RSO_V4  Task3 RSO v4 Online (Full-distance move)

rcfg=rso_solver_v4('config');
cfg=cp_engine_v2('config');
cN=cp_engine_v2('cons','normal'); cT=cp_engine_v2('cons','thunder');
ce=cp_engine_v2('cons','expected');

if nargin<1||isempty(weather_seq)
    rng('shuffle'); weather_seq=cp_engine_v2('weather',90,0.8);
end

st.pt=1; st.pos=cfg.xy(1,:);
st.O=cfg.init.O; st.H=cfg.init.H; st.F=cfg.init.F;
st.M=cfg.init.M; st.Z=cfg.init.Z; st.consec_work=0; st.day=0;

fprintf('========================================\n');
fprintf('  Task3 RSO v4 Online (Full-dist move)\n');
fprintf('========================================\n');
fprintf('H=%dd K=%d(%dT) | Full-dist eval | Per-res supply\n',...
    rcfg.H,rcfg.K_total,rcfg.K_thunder);
fprintf('Day  | W | Action              | Pos        | Res       |   Z    M\n');
fprintf('-----|---|---------------------|------------|-----------|---------\n');

while st.day<cfg.MAX_DAYS && st.pt~=2
    st.day=st.day+1;
    w=weather_seq(st.day);
    if w=='T', ca=cT; wn='T'; else, ca=cN; wn='N'; end
    curr=struct('O',st.O,'H',st.H,'F',st.F,'M',st.M,'Z',st.Z);
    [best_act,~]=rso_solver_v4('step',st.pt,st.day-1,curr,cfg,rcfg.H,rcfg.K_total,rcfg.K_thunder);
    action_str=''; ik=false;
    
    switch best_act.type
        case 'MOVE'
            target=best_act.target;
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
                if st.pt==6||st.pt==7
                    [nO,nH,nF]=cp_engine_v2('supply_needs_safe',[st.pt,2],[],[],1,ce,cfg);
                    sp=cfg.MAX_LOAD-(st.O+st.H+st.F);
                    bO=max(0,nO-st.O); bH=max(0,nH-st.H); bF=max(0,nF-st.F);
                    if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
                    cost=bO*2+bH*1+bF*2;
                    if cost<=st.M
                        st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=st.M-cost;
                        action_str=sprintf('SUPPLY(%s)',cfg.names{st.pt});
                    end
                elseif st.pt==2, action_str='ARRIVE!'; end
            end
            ik=true;
        case 'PARK'
            st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF;
            st.consec_work=0; action_str='PARK'; ik=true;
        case 'WORK'
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
        case 'SUPPLY'
            if st.pt==6||st.pt==7
                [nO,nH,nF]=cp_engine_v2('supply_needs_safe',[st.pt,2],[],[],1,ce,cfg);
                sp=cfg.MAX_LOAD-(st.O+st.H+st.F);
                bO=max(0,nO-st.O); bH=max(0,nH-st.H); bF=max(0,nF-st.F);
                if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
                cost=bO*2+bH*1+bF*2;
                if cost<=st.M
                    st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=st.M-cost;
                end
                st.consec_work=0; action_str='SUPPLY'; ik=true;
            end
    end
    
    if st.O<-1e-6||st.H<-1e-6||st.F<-1e-6
        fprintf('%4d | %s | EXHAUSTED!           | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f ***\n',...
            st.day,wn,st.pos(1),st.pos(2),st.O,st.H,st.F,st.Z,round(st.M));
        break;
    end
    if ik
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f\n',...
            st.day,wn,action_str,st.pos(1),st.pos(2),st.O,st.H,st.F,st.Z,round(st.M));
    end
end

fprintf('-----|---|---------------------|------------|-----------|---------\n');
if st.pt==2, fprintf('Day %d Arrived | Z=%d M=%.0f\n',st.day,st.Z,round(st.M));
else, fprintf('FAIL/TIMEOUT Day %d\n',st.day); end
fprintf('Done.\n');
end
"""

write_file(DIR_C, "solve_q3_rso_v4.m", SOLVE_Q3_RSO_V4)
print("C v4: rso_solver_v4.m + solve_q3_rso_v4.m written")
