function varargout = rso_solver(action, varargin)
switch action
    case 'simulate',    [varargout{1},varargout{2},varargout{3},varargout{4},varargout{5}] = simulate_skeleton(varargin{:});
    case 'evaluate',    [varargout{1},varargout{2},varargout{3},varargout{4}] = evaluate_skeleton(varargin{:});
    otherwise, error('rso_solver: unknown action');
end
end

function [ok, Zf, Mf, days, reason] = simulate_skeleton(skel, weather_seq, cfg)
    if nargin<3, cfg = cp_engine('config'); end
    cN = cp_engine('cons','normal');  cT = cp_engine('cons','thunder');
    
    st.pt = 1;  st.pos = cfg.xy(1,:);
    st.O = cfg.init.O;  st.H = cfg.init.H;  st.F = cfg.init.F;
    st.M = cfg.init.M;  st.Z = cfg.init.Z;
    st.consec_work = 0;  st.day = 0;
    leg = 1;  step_in_leg = 0;  supplied_here = false;
    
    while st.day < cfg.MAX_DAYS && st.pt ~= 2
        st.day = st.day + 1;
        if st.day > length(weather_seq), w = N;
        else, w = weather_seq(st.day); end
        if w == 'T', ca = cT; else, ca = cN; end
        next_pt = skel(leg + 1);
        acted = false;
        
        % Compute travel days remaining to E from current leg
        travel_to_e = 0;
        for k = leg:length(skel)-1
            travel_to_e = travel_to_e + cfg.dist(skel(k), skel(k+1));
        end
        remaining = cfg.MAX_DAYS - st.day;
        
        % 1. At work point: work if time permits, else leave
        if st.pt >= 3 && st.pt <= 5
            wi = st.pt - 2;
            if remaining > travel_to_e + 5
                if st.consec_work < cfg.WM(wi) && w == 'N'
                    st.O = st.O - ca.WO; st.H = st.H - ca.WH; st.F = st.F - ca.WF;
                    st.Z = st.Z + cfg.WY(wi); st.consec_work = st.consec_work + 1;
                else
                    st.O = st.O - ca.PO; st.H = st.H - ca.PH; st.F = st.F - ca.PF;
                    if st.consec_work >= cfg.WM(wi), st.consec_work = 0; end
                end
                acted = true;
            end
        end
        
        % 2. At supply point: supply on first day, then move
        if ~acted && (st.pt == 6 || st.pt == 7)
            if ~supplied_here
                [bO, bH, bF, cost] = compute_supply(st, skel, leg, cfg);
                if cost <= st.M
                    st.O = st.O + bO; st.H = st.H + bH; st.F = st.F + bF;
                    st.M = st.M - cost;
                end
                supplied_here = true;
            end
            st.consec_work = 0;
            % Move toward next node (supply day counts as movement day)
            if leg < length(skel)
                st = move_one_step(st, next_pt, ca, cfg);
                step_in_leg = step_in_leg + 1;
                if step_in_leg >= cfg.dist(skel(leg), next_pt)
                    st.pt = next_pt;  st.pos = cfg.xy(next_pt,:);
                    step_in_leg = 0;  leg = leg + 1;  supplied_here = false;
                end
            end
            acted = true;
        end
        
        % 3. Moving between nodes
        if ~acted && leg < length(skel)
            if step_in_leg < cfg.dist(skel(leg), next_pt)
                st = move_one_step(st, next_pt, ca, cfg);
                step_in_leg = step_in_leg + 1;
                if step_in_leg >= cfg.dist(skel(leg), next_pt)
                    st.pt = next_pt;  st.pos = cfg.xy(next_pt,:);
                    step_in_leg = 0;  leg = leg + 1;  supplied_here = false;
                end
                acted = true;
            end
        end
        
        if ~acted
            ok = false; Zf = 0; Mf = 0; days = st.day;
            reason = 'STUCK'; return;
        end
        
        if st.O < -1e-6 || st.H < -1e-6 || st.F < -1e-6
            ok = false; Zf = 0; Mf = 0; days = st.day;
            reason = 'RESOURCE_EXHAUSTED'; return;
        end
        if st.O + st.H + st.F > cfg.MAX_LOAD + 1e-6
            ok = false; Zf = 0; Mf = 0; days = st.day;
            reason = 'OVERLOAD'; return;
        end
    end
    
    if st.pt == 2
        ok = true; Zf = st.Z; Mf = st.M; days = st.day; reason = 'ARRIVED';
    elseif st.day >= cfg.MAX_DAYS
        ok = false; Zf = 0; Mf = 0; days = st.day; reason = 'TIMEOUT';
    else
        ok = false; Zf = 0; Mf = 0; days = st.day; reason = 'UNKNOWN';
    end
end
function st = move_one_step(st, target, ca, cfg)
    fr = st.pos;  to = cfg.xy(target, :);
    dx = to(1) - fr(1);  dy = to(2) - fr(2);
    if abs(dx) > 0
        st.pos(1) = fr(1) + sign(dx);
    elseif abs(dy) > 0
        st.pos(1) = fr(1);  st.pos(2) = fr(2) + sign(dy);
    end
    st.O = st.O - ca.MO;  st.H = st.H - ca.MH;  st.F = st.F - ca.MF;
    st.consec_work = 0;
end

function [bO, bH, bF, cost] = compute_supply(state, skel, leg, cfg)
    ce = cp_engine('cons','expected');
    ct = cp_engine('cons','thunder');
    alpha = cfg.SAFETY_ALPHA;
    need_O = 0;  need_H = 0;  need_F = 0;
    
    for k = leg:length(skel)-1
        d = cfg.dist(skel(k), skel(k+1));
        np = skel(k+1);
        need_O = need_O + d * ((1-alpha)*ce.MO + alpha*ct.MO);
        need_H = need_H + d * ((1-alpha)*ce.MH + alpha*ct.MH);
        need_F = need_F + d * ((1-alpha)*ce.MF + alpha*ct.MF);
        if np >= 3 && np <= 5
            wi = np - 2;  mc = cfg.WM(wi);
            est_w = mc;
            need_O = need_O + est_w * ((1-alpha)*ce.WO + alpha*ct.WO);
            need_H = need_H + est_w * ((1-alpha)*ce.WH + alpha*ct.WH);
            need_F = need_F + est_w * ((1-alpha)*ce.WF + alpha*ct.WF);
        end
        if np == 6 || np == 7 || np == 2, break; end
    end
    
    sp = cfg.MAX_LOAD - (state.O + state.H + state.F);
    bO = max(0, need_O - state.O);
    bH = max(0, need_H - state.H);
    bF = max(0, need_F - state.F);
    if bO + bH + bF > sp
        scl = sp / (bO + bH + bF);
        bO = bO * scl;  bH = bH * scl;  bF = bF * scl;
    end
    cn = cp_engine('cons','normal');
    cost = bO*cn.pO + bH*cn.pH + bF*cn.pF;
end

function [mean_Z, mean_M, succ_rate, results] = evaluate_skeleton(skel, N, cfg)
    if nargin<3, cfg = cp_engine('config'); end
    if nargin<2, N = cfg.MC_N; end
    Zr = NaN(1,N);  Mr = NaN(1,N);  ok = false(1,N);
    dy = NaN(1,N);  fr = cell(1,N);
    for sim = 1:N
        ws = cp_engine('weather', cfg.MAX_DAYS, cfg.p_normal);
        [fok, Zf, Mf, d, reason] = simulate_skeleton(skel, ws, cfg);
        Zr(sim) = Zf;  Mr(sim) = Mf;  ok(sim) = fok;
        dy(sim) = d;  fr{sim} = reason;
    end
    n_ok = sum(ok);  succ_rate = n_ok / N;
    if n_ok > 0
        mean_Z = mean(Zr(ok));  mean_M = mean(Mr(ok));
    else
        mean_Z = 0;  mean_M = 0;
    end
    results.Zr = Zr;  results.Mr = Mr;  results.ok = ok;
    results.days = dy;  results.reasons = fr;
end