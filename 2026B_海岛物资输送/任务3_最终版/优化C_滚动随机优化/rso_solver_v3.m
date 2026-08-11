function varargout = rso_solver_v3(action, varargin)
% RSO_SOLVER_V3  Task3 RSO Engine v3 (Corrected)
% Fixes over v2:
%   P0: tail_simulate uses actual scenario weather day-by-day (not avg CP)
%   P1: importance-weighted expected value for stratified sampling
%   P1: consistent actual-weather consumption throughout
%   P2: proper supply needs (not 0.9 heuristic)
%   P2: consecutive work tracking in simulation
%   P3: K=48 (doubled coverage)

switch action
    case 'step',  [varargout{1},varargout{2}] = rso_step_v3(varargin{:});
    case 'config', varargout{1} = get_rso_config_v3();
    otherwise, error('rso_solver_v3: unknown');
end
end

function cfg = get_rso_config_v3()
    cfg.H = 24;          % lookahead horizon
    cfg.K_total = 48;    % total scenarios (v2: 24)
    cfg.K_thunder = 16;  % thunder-heavy (v2: 8)
    cfg.pN = 0.8;
end

% ===== Stratified sampling with importance weights =====
function [scenarios, weights] = sample_scenarios_v3(H, K_total, K_thunder, pN)
    scenarios = cell(1, K_total);
    weights = zeros(1, K_total);
    pT = 1 - pN;
    
    % Stratum 1: thunder-heavy (p_thunder=0.4)
    qT1 = 0.4;
    for k = 1:K_thunder
        ws = repmat('N', 1, H);
        for i = 1:H, if rand() > (1-qT1), ws(i)='T'; end; end
        scenarios{k} = ws;
        % Importance weight: true_prob / sampling_prob
        nT = sum(ws=='T'); nN = H - nT;
        true_prob = pN^nN * pT^nT;
        samp_prob = (1-qT1)^nN * qT1^nT;
        weights(k) = true_prob / max(samp_prob, 1e-10);
    end
    
    % Stratum 2: baseline (p_thunder=0.2)
    for k = (K_thunder+1):K_total
        ws = repmat('N', 1, H);
        for i = 1:H, if rand() > pN, ws(i)='T'; end; end
        scenarios{k} = ws;
        nT = sum(ws=='T'); nN = H - nT;
        true_prob = pN^nN * pT^nT;
        samp_prob = pN^nN * pT^nT;
        weights(k) = true_prob / max(samp_prob, 1e-10);
    end
    
    % Normalize weights
    weights = weights / sum(weights);
end

% ===== One RSO step (v3: importance-weighted) =====
function [best_action, best_value] = rso_step_v3(cur_pt, elapsed, state, cfg_cp, H, K_total, K_thunder)
    if nargin<6, rc= get_rso_config_v3(); H=rc.H; K_total=rc.K_total; K_thunder=rc.K_thunder; end
    
    [scenarios, weights] = sample_scenarios_v3(H, K_total, K_thunder);
    inter_nodes = [3 4 5 6 7];
    action_values = {};
    
    % 1. MOVE toward each reachable node
    for ni = 1:length(inter_nodes)
        target = inter_nodes(ni);
        if target==cur_pt, continue; end
        d = cfg_cp.dist(cur_pt, target);
        if elapsed+d > cfg_cp.MAX_DAYS, continue; end
        
        wsum=0; vsum=0; valid=0;
        for k = 1:K_total
            val = sim_move_then_tail(cur_pt,target,elapsed,state,scenarios{k},cfg_cp);
            if val > -inf
                vsum = vsum + val*weights(k); wsum = wsum + weights(k); valid=valid+1;
            end
        end
        if valid > K_total/6
            action_values{end+1}=struct('type','MOVE','target',target,'value',vsum/wsum);
        end
    end
    
    % 2. PARK
    wsum=0; vsum=0; valid=0;
    for k=1:K_total
        val=sim_park_then_tail(cur_pt,elapsed,state,scenarios{k},cfg_cp);
        if val>-inf, vsum=vsum+val*weights(k); wsum=wsum+weights(k); valid=valid+1; end
    end
    if valid>K_total/6
        action_values{end+1}=struct('type','PARK','target',cur_pt,'value',vsum/wsum);
    end
    
    % 3. WORK
    if cur_pt>=3&&cur_pt<=5
        wsum=0; vsum=0; valid=0;
        for k=1:K_total
            val=sim_work_then_tail(cur_pt,elapsed,state,scenarios{k},cfg_cp);
            if val>-inf, vsum=vsum+val*weights(k); wsum=wsum+weights(k); valid=valid+1; end
        end
        if valid>K_total/6
            action_values{end+1}=struct('type','WORK','target',cur_pt,'value',vsum/wsum);
        end
    end
    
    % 4. SUPPLY
    if cur_pt==6||cur_pt==7
        wsum=0; vsum=0; valid=0;
        for k=1:K_total
            val=sim_supply_then_tail(cur_pt,elapsed,state,scenarios{k},cfg_cp);
            if val>-inf, vsum=vsum+val*weights(k); wsum=wsum+weights(k); valid=valid+1; end
        end
        if valid>K_total/6
            action_values{end+1}=struct('type','SUPPLY','target',cur_pt,'value',vsum/wsum);
        end
    end
    
    if isempty(action_values)
        best_action=struct('type','PARK','target',cur_pt); best_value=-inf; return;
    end
    [~,idx]=max(cellfun(@(x)x.value, action_values));
    best_action=action_values{idx}; best_value=best_action.value;
end

% ===== One-step simulations (use actual weather for step 1) =====
function val=sim_move_then_tail(cur_pt,target,elapsed,state,scenario,cfg)
    w1=scenario(1); fr=cfg.xy(cur_pt,:); to=cfg.xy(target,:);
    if w1=='T', cO=8;cH=4;cF=3; else, cO=2;cH=3;cF=2; end
    nO=state.O-cO; nH=state.H-cH; nF=state.F-cF;
    if nO<0||nH<0||nF<0, val=-inf; return; end
    dx=to(1)-fr(1); dy=to(2)-fr(2);
    if abs(dx)>0, np=[fr(1)+sign(dx),fr(2)];
    else, np=[fr(1),fr(2)+sign(dy)]; end
    if np(1)==to(1)&&np(2)==to(2), npt=target; else, npt=cur_pt; end
    ns=struct('O',nO,'H',nH,'F',nF,'M',state.M,'Z',state.Z);
    rem=scenario(2:min(end,cfg.MAX_DAYS-elapsed));
    val=tail_simulate(npt,elapsed+1,ns,rem,cfg,0);
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

function val=sim_supply_then_tail(cur_pt,elapsed,state,scenario,cfg)
    w1=scenario(1); ce=cp_engine_v2('cons','expected');
    if w1=='T', cp=3+3+2; else, cp=1+1+1; end
    [nO,nH,nF]=cp_engine_v2('supply_needs_safe',[cur_pt,2],[],[],1,ce,cfg);
    sp=cfg.MAX_LOAD-(state.O+state.H+state.F);
    bO=max(0,nO-state.O); bH=max(0,nH-state.H); bF=max(0,nF-state.F);
    if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
    cost=bO*2+bH*1+bF*2;
    if cost>state.M, val=-inf; return; end
    ns=struct('O',state.O+bO-cp/3,'H',state.H+bH-cp/3,'F',state.F+bF-cp/3,...
             'M',state.M-cost,'Z',state.Z);
    rem=scenario(2:min(end,cfg.MAX_DAYS-elapsed));
    val=tail_simulate(cur_pt,elapsed+1,ns,rem,cfg,0);
end

% ===== TAIL SIMULATION: day-by-day with actual weather (FIX P0) =====
function val = tail_simulate(cur_pt, elapsed, state, scenario, cfg, consec_work)
    rem = min(length(scenario), cfg.MAX_DAYS - elapsed);
    if rem <= 0
        val = iif(cur_pt==2, state.Z, 0); return;
    end
    
    cN = cp_engine_v2('cons','normal');
    cT = cp_engine_v2('cons','thunder');
    ce = cp_engine_v2('cons','expected');
    
    sim_O = state.O; sim_H = state.H; sim_F = state.F;
    sim_M = state.M; sim_Z = state.Z;
    sim_pt = cur_pt; sim_pos = cfg.xy(cur_pt,:);
    cw = consec_work;
    to_E = cfg.xy(2,:);
    
    for t = 1:rem
        if sim_pt == 2, break; end  % reached E
        
        w = scenario(t);
        if w == 'T', ca = cT; else, ca = cN; end
        
        % ==== Greedy heuristic policy ====
        action_taken = false;
        
        % 1. If at work point and can work: WORK
        if sim_pt >= 3 && sim_pt <= 5
            wi = sim_pt - 2;
            if cw < cfg.WM(wi)
                sim_O = sim_O - ca.WO; sim_H = sim_H - ca.WH; sim_F = sim_F - ca.WF;
                sim_Z = sim_Z + cfg.WY(wi);
                cw = cw + 1;
                action_taken = true;
            end
        end
        
        % 2. If at supply point and resources low: SUPPLY
        if ~action_taken && (sim_pt == 6 || sim_pt == 7)
            dE = cfg.dist(sim_pt, 2);
            if sim_O < dE*cT.MO || sim_H < dE*cT.MH || sim_F < dE*cT.MF
                [nO,nH,nF] = cp_engine_v2('supply_needs_safe',[sim_pt,2],[],[],1,ce,cfg);
                sp = cfg.MAX_LOAD - (sim_O+sim_H+sim_F);
                bO = max(0,nO-sim_O); bH = max(0,nH-sim_H); bF = max(0,nF-sim_F);
                if bO+bH+bF > sp, scl = sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
                cost = bO*2 + bH*1 + bF*2;
                if cost <= sim_M
                    sim_O = sim_O + bO; sim_H = sim_H + bH;
                    sim_F = sim_F + bF; sim_M = sim_M - cost;
                end
                cw = 0; action_taken = true;
            end
        end
        
        % 3. If thunder and not at W/S: PARK
        if ~action_taken && w == 'T' && sim_pt ~= 6 && sim_pt ~= 7 ...
           && ~(sim_pt>=3 && sim_pt<=5)
            sim_O = sim_O - ca.PO; sim_H = sim_H - ca.PH; sim_F = sim_F - ca.PF;
            cw = 0; action_taken = true;
        end
        
        % 4. Default: MOVE one step toward E
        if ~action_taken
            % Move one Manhattan step toward E
            dx = to_E(1) - sim_pos(1);
            dy = to_E(2) - sim_pos(2);
            if abs(dx) > 0
                sim_pos(1) = sim_pos(1) + sign(dx);
            elseif abs(dy) > 0
                sim_pos(2) = sim_pos(2) + sign(dy);
            end
            sim_O = sim_O - ca.MO; sim_H = sim_H - ca.MH; sim_F = sim_F - ca.MF;
            cw = 0;
            
            % Check arrival at any known node
            for nd = 1:7
                if sim_pos(1)==cfg.xy(nd,1) && sim_pos(2)==cfg.xy(nd,2)
                    sim_pt = nd; break;
                end
            end
        end
        
        % Check resource constraints
        if sim_O < -1e-6 || sim_H < -1e-6 || sim_F < -1e-6
            val = 0; return;  % resource exhaustion
        end
        if sim_O+sim_H+sim_F > cfg.MAX_LOAD + 1e-6
            val = 0; return;  % overload
        end
    end
    
    if sim_pt == 2
        val = sim_Z;
    else
        val = 0;  % didn't reach E
    end
end

function v = iif(cond, tval, fval)
    if cond, v = tval; else, v = fval; end
end
