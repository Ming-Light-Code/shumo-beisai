function solve_q3_mdp()
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
