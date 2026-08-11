function solve_q3_mdp_v2(weather_seq)
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
