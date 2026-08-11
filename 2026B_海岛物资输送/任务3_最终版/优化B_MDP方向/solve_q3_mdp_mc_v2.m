function solve_q3_mdp_mc_v2(N)
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
