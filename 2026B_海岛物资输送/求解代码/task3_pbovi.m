%% ==========================================================================
% Problem 3: PBOVI (Point-Based Online Value Iteration) Solver - MATLAB
% Reference: [1] "Dynamic Uncertain Environment Agent Sequential Decision
%            Methods and Applications" by Wu Bo, Ch2.1 MDP, Ch3.3 PBOVI
% ==========================================================================

function task3_pbovi_main(seed, nsb, nsa, nsf)
    if nargin < 1, seed = 42; end
    if nargin < 2, nsb  = 200; end
    if nargin < 3, nsa  = 150; end
    if nargin < 4, nsf  = 2000; end

    t0 = tic;
    fprintf('=================================================================\n');
    fprintf(' Problem 3: PBOVI Solver (MATLAB)\n');
    fprintf(' Reference: [1] Wu Bo PhD Thesis Ch2.1 MDP, Ch3.3 PBOVI\n');
    fprintf('=================================================================\n');

    rng(seed);
    p = get_params();

    % ---- Phase 1: Skeleton Enumeration ----
    fprintf('\n[Phase 1] Skeleton enumeration\n');
    [skels, nodePos] = enum_skeletons(p, 10, 10000);
    fprintf('  Generated %d candidates\n', length(skels));

    uq = unique_skeletons(skels);
    uq = sort_skels_by_dist(uq, nodePos);
    hw = {};
    for i = 1:length(uq)
        sk = uq{i};
        for j = 2:length(sk)-1
            nm = sk{j};
            if strcmp(nm,'W1') || strcmp(nm,'W2') || strcmp(nm,'W3')
                hw{end+1} = sk; break;
            end
        end
    end
    cand = [hw, {{'B','E'}}];
    fprintf('  Dedup: %d, with work: %d, total: %d\n', length(uq), length(hw), length(cand));

    % ---- Phase 2: Base Policy MC Rollout ----
    fprintf('\n[Phase 2] Base policy MC evaluation (%d runs each)\n', nsb);
    br = {};
    for i = 1:length(cand)
        sk = cand{i};
        if skel_dist(sk, nodePos) > 80, continue; end
        if mod(i,100) == 0, fprintf('    %d/%d (%.1fs)\n', i, length(cand), toc(t0)); end
        tZ = 0; tM = 0; sc = 0;
        for k = 1:nsb
            rng(randi(2^31-1));
            [ok, Zf, Mf] = sim_base(p, sk, nodePos);
            if ok, tZ = tZ + Zf; tM = tM + Mf; sc = sc + 1; end
        end
        if sc >= nsb * 0.1
            br{end+1} = struct('sk', {sk}, 'avgZ', tZ/sc, 'avgM', tM/sc, 'sc', sc);
        end
    end
    br = sort_br(br);
    fprintf('  Feasible: %d skeletons\n', length(br));
    for i = 1:min(5, length(br))
        fprintf('    #%d: %s  Z=%.1f M=%.1f (%d/%d)\n', i, ...
            strjoin(br{i}.sk,' -> '), br{i}.avgZ, br{i}.avgM, br{i}.sc, nsb);
    end

    % ---- Phase 3: API Iteration 1 ----
    fprintf('\n[Phase 3] API Iter1: fit value functions\n');
    sZ = {}; sM = {};
    n_train = min(500, length(cand));
    for i = 1:n_train
        sk = cand{i};
        if skel_dist(sk, nodePos) > 80, continue; end
        for k = 1:nsb
            rng(randi(2^31-1));
            [ok, ~, ~, svs] = sim_base_full(p, sk, nodePos);
            if ok
                for j = 1:length(svs)
                    sZ{end+1} = struct('feats', state_feats(p, svs{j}.state), 'ret', svs{j}.ret_Z);
                    sM{end+1} = struct('feats', state_feats(p, svs{j}.state), 'ret', svs{j}.ret_M);
                end
            end
        end
        if mod(i,100) == 0, fprintf('    %d skels, %d samples\n', i, length(sZ)); end
    end
    fprintf('  Total: %d samples\n', length(sZ));
    if length(sZ) > 50000
        idx = randperm(length(sZ), 50000);
        sZ = sZ(idx); sM = sM(idx);
    end
    thZ = fit_theta(sZ);
    thM = fit_theta(sM);
    fprintf('  theta_Z range: [%.1f, %.1f]\n', min(thZ), max(thZ));

    % ---- Phase 4: ADP Evaluation ----
    fprintf('\n[Phase 4] ADP eval (top 50, %d MC)\n', nsa);
    n_test = min(50, length(br));
    tsk = cell(1, n_test+1);
    for i = 1:n_test, tsk{i} = br{i}.sk; end
    tsk{n_test+1} = {'B','E'};
    bsk = {}; bZ = -1; bM = -1; bL = {};
    for i = 1:length(tsk)
        sk = tsk{i};
        tZ = 0; tM = 0; sc = 0; lZ = -1; lM = -1; lL = {};
        for k = 1:nsa
            rng(randi(2^31-1));
            [ok, Zf, Mf, lg] = sim_adp(p, sk, nodePos, thZ, thM);
            if ok
                tZ = tZ + Zf; tM = tM + Mf; sc = sc + 1;
                if Zf > lZ || (Zf == lZ && Mf > lM), lZ = Zf; lM = Mf; lL = lg; end
            end
        end
        if sc >= nsa * 0.15
            aZ = tZ / sc; aM = tM / sc;
            if aZ > bZ || (aZ == bZ && aM > bM)
                bZ = aZ; bM = aM; bsk = sk; bL = lL;
            end
        end
    end
    fprintf('  Best ADP: %s\n', strjoin(bsk,' -> '));
    fprintf('  E[Z]=%.1f, E[M]=%.1f\n', bZ, bM);

    % ---- Phase 5: API Iter2 ----
    fprintf('\n[Phase 5] API Iter2: on-policy sampling\n');
    sZ2 = {}; sM2 = {};
    n_train2 = min(300, length(cand));
    for i = 1:n_train2
        sk = cand{i};
        if skel_dist(sk, nodePos) > 80, continue; end
        for k = 1:30
            rng(randi(2^31-1));
            [ok, ~, ~, svs] = sim_adp_full(p, sk, nodePos, thZ, thM);
            if ok
                for j = 1:length(svs)
                    sZ2{end+1} = struct('feats', state_feats(p, svs{j}.state), 'ret', svs{j}.ret_Z);
                    sM2{end+1} = struct('feats', state_feats(p, svs{j}.state), 'ret', svs{j}.ret_M);
                end
            end
        end
    end
    fprintf('  Added %d on-policy samples\n', length(sZ2));
    allZ = [sZ, sZ2]; allM = [sM, sM2];
    if length(allZ) > 80000
        idx = randperm(length(allZ), 80000);
        allZ = allZ(idx); allM = allM(idx);
    end
    thZ2 = fit_theta(allZ); thM2 = fit_theta(allM);
    fprintf('  Refitted theta_Z2: [%.1f, %.1f]\n', min(thZ2), max(thZ2));

    % ---- Phase 6: Online Tree Refinement ----
    fprintf('\n[Phase 6] Online tree refinement (Sec 3.3)\n');
    rz = 0; rm = 0; sc = 0;
    for k = 1:nsa
        rng(randi(2^31-1));
        [ok, Zf, Mf, lg] = refine_sim(p, thZ2, thM2, bsk, nodePos, 4);
        if ok
            rz = rz + Zf; rm = rm + Mf; sc = sc + 1;
            if Zf > bZ || (Zf == bZ && Mf > bM), bZ = Zf; bM = Mf; bL = lg; end
        end
    end
    if sc > 0, fprintf('  Refined E[Z]=%.1f, E[M]=%.1f (%d/%d)\n', rz/sc, rm/sc, sc, nsa); end

    % ---- Phase 7: MC Validation ----
    fprintf('\n[Phase 7] MC validation (%d runs)\n', nsf);
    tZ = 0; tM = 0; sc = 0; lZb = -1; lMb = -1; lLb = {};
    for k = 1:nsf
        rng(randi(2^31-1));
        [ok, Zf, Mf, lg] = refine_sim(p, thZ2, thM2, bsk, nodePos, 4);
        if ok
            tZ = tZ + Zf; tM = tM + Mf; sc = sc + 1;
            if Zf > lZb || (Zf == lZb && Mf > lMb), lZb = Zf; lMb = Mf; lLb = lg; end
        end
    end
    mZ = tZ / max(1, sc); mM = tM / max(1, sc);
    fprintf('  Success: %d/%d (%.1f%%)\n', sc, nsf, 100*sc/nsf);
    fprintf('  MC E[Z]=%.1f, E[M]=%.1f\n', mZ, mM);
    fprintf('  Best episode: Z=%d, M=%d\n', lZb, lMb);

    % ---- Phase 8: Deterministic Replay ----
    fprintf('\n[Phase 8] Deterministic replay (seed=42)\n');
    rng(42);
    [ok, dZ, dM, dl] = refine_sim(p, thZ2, thM2, bsk, nodePos, 4);
    if ok
        fprintf('  Sample weather: Z=%d, M=%d\n', dZ, dM);
        if ~isempty(dl) && length(dl) > length(lLb)
            lLb = dl; lZb = dZ; lMb = dM;
        end
    else
        fprintf('  Deterministic replay failed, using best MC log\n');
    end

    elapsed = toc(t0);
    fprintf('\n  Total time: %.1fs\n', elapsed);

    % Build result struct
    result = struct('skeleton', {bsk}, 'mc_Z', mZ, 'mc_M', mM, ...
        'best_Z', lZb, 'best_M', lMb, 'best_log', {lLb}, ...
        'success_rate', sc/nsf, 'elapsed_s', elapsed);

    % Output
    if ~isempty(result.best_log)
        t = cell2table(build_xls_data(p, result));
        t.Properties.VariableNames = {'Day','PosX','PosY','Weather','Action','Point', ...
            'FuelO','WaterH','FoodF','MoneyM','TargetZ','c','BuyO','BuyH','BuyF'};
        writetable(t, 'result.xlsx', 'Sheet', 'PBOVI');
        fprintf('\n  Excel output: result.xlsx\n');
    end

    sm = struct('problem', 'Task 3 (PBOVI)', ...
        'method', 'Point-Based Online Value Iteration', ...
        'reference', '[1] Ch2.1, Ch3.3', ...
        'skeleton', {result.skeleton}, ...
        'mc_expected_Z', result.mc_Z, ...
        'mc_expected_M', result.mc_M, ...
        'best_episode_Z', result.best_Z, ...
        'best_episode_M', result.best_M, ...
        'success_rate', result.success_rate, ...
        'elapsed_s', result.elapsed_s);
    fid = fopen('task3_pbovi_result.json', 'w');
    fprintf(fid, '{
');
    fprintf(fid, '  "problem": "%s",
', sm.problem);
    fprintf(fid, '  "method": "%s",
', sm.method);
    fprintf(fid, '  "reference": "%s",
', sm.reference);
    sk_str = strjoin(sm.skeleton, ' -> ');
    fprintf(fid, '  "skeleton": "%s",
', sk_str);
    fprintf(fid, '  "mc_expected_Z": %f,
', sm.mc_expected_Z);
    fprintf(fid, '  "mc_expected_M": %f,
', sm.mc_expected_M);
    fprintf(fid, '  "best_episode_Z": %d,
', sm.best_episode_Z);
    fprintf(fid, '  "best_episode_M": %d,
', sm.best_episode_M);
    fprintf(fid, '  "success_rate": %f,
', sm.success_rate);
    fprintf(fid, '  "elapsed_s": %f
', sm.elapsed_s);
    fprintf(fid, '}
');
    fclose(fid);
    fprintf('  JSON: task3_pbovi_result.json\n\nComplete!\n');
end

% ==================== Parameters ====================
function p = get_params()
    persistent pp;
    if isempty(pp)
        pp = [];
        pp.GRID = 30;
        pp.B = [1, 15]; pp.E = [30, 15];
        pp.S1 = [12, 16]; pp.S2 = [21, 16];
        pp.W1 = [6, 21]; pp.W2 = [15, 9]; pp.W3 = [24, 24];
        pp.CMOVE = [2,3,2; 8,4,3];
        pp.CIDLE = [1,1,1; 3,3,2];
        pp.CWORK = [5,4,3; 8,6,6];
        pp.PN = 0.8; pp.PS = 0.2;
        pp.PRICES = [2, 1, 2];
        pp.INIT_O = 100; pp.INIT_H = 150; pp.INIT_F = 100;
        pp.INIT_M = 750; pp.INIT_Z = 200;
        pp.MAX_LOAD = 400; pp.MAX_DAYS = 90;
        pp.EMOVE = pp.PN*pp.CMOVE(1,:) + pp.PS*pp.CMOVE(2,:);
        pp.EIDLE = pp.PN*pp.CIDLE(1,:) + pp.PS*pp.CIDLE(2,:);
        pp.EWORK = pp.PN*pp.CWORK(1,:) + pp.PS*pp.CWORK(2,:);
    end
    p = pp;
end

% ==================== Utilities ====================
function d = md(a, b)
    d = abs(a(1)-b(1)) + abs(a(2)-b(2));
end

function cr = consume_rate(wi, act)
    p = get_params();
    if strcmp(act, 'move')
        cr = p.CMOVE(wi+1, :);
    elseif strcmp(act, 'idle')
        cr = p.CIDLE(wi+1, :);
    elseif strcmp(act, 'work')
        cr = p.CWORK(wi+1, :);
    else
        cr = [0,0,0];
    end
end

function [nm, gain, mx] = at_wp(pos)
    p = get_params();
    nm = ''; gain = 0; mx = 0;
    if isequal(pos, p.W1), nm = 'W1'; gain = 20; mx = 4;
    elseif isequal(pos, p.W2), nm = 'W2'; gain = 15; mx = 5;
    elseif isequal(pos, p.W3), nm = 'W3'; gain = 28; mx = 3;
    end
end

function nm = at_sp(pos)
    p = get_params();
    nm = '';
    if isequal(pos, p.S1), nm = 'S1';
    elseif isequal(pos, p.S2), nm = 'S2';
    end
end

function ok = state_ok(s)
    p = get_params();
    ok = s.O >= 0 && s.H >= 0 && s.F >= 0 && s.M >= 0 && ...
         (s.O + s.H + s.F) <= p.MAX_LOAD;
end

function ns = apply_act(s, action, wi, work_gain, new_pos)
    if nargin < 4, work_gain = 0; end
    if nargin < 5, new_pos = []; end
    cr = consume_rate(wi, action);
    ns = s;
    ns.O = ns.O - cr(1); ns.H = ns.H - cr(2); ns.F = ns.F - cr(3);
    if strcmp(action, 'work')
        ns.Z = ns.Z + work_gain; ns.c = ns.c + 1;
    elseif strcmp(action, 'move')
        ns.x = new_pos(1); ns.y = new_pos(2); ns.c = 0;
    elseif strcmp(action, 'idle')
        ns.c = 0;
    end
    ns.day = ns.day + 1;
end

function ns = apply_buy_act(s, bO, bH, bF)
    p = get_params();
    ns = s;
    cost = bO * p.PRICES(1) + bH * p.PRICES(2) + bF * p.PRICES(3);
    if cost > ns.M, ns = []; return; end
    ns.O = ns.O + bO; ns.H = ns.H + bH; ns.F = ns.F + bF; ns.M = ns.M - cost;
end

function npos = mv_toward(s, tgt)
    dx = sign(tgt(1) - s.x);
    dy = 0;
    if dx == 0, dy = sign(tgt(2) - s.y); end
    if dx == 0 && dy == 0
        npos = [s.x, s.y];
    else
        npos = [s.x + dx, s.y + dy];
    end
end

function w = sw(p)
    w = double(rand() >= p.PN);
end

% ==================== State Features (15-dim) ====================
function feats = state_feats(p, s)
    dE = md([s.x, s.y], p.E);
    rem = (p.MAX_DAYS - s.day + 1) / p.MAX_DAYS;
    tp = dE / max(1, p.MAX_DAYS - s.day + 1);

    [~, wg, wm_cap] = at_wp([s.x, s.y]);
    at_w = double(~isempty(at_wp([s.x, s.y])));
    if at_w
        wp_rem = (wm_cap - s.c) / max(1, wm_cap);
        gf = wg * (wm_cap - s.c) / max(1, wm_cap);
    else
        wp_rem = 0; gf = 0;
    end
    at_s = double(~isempty(at_sp([s.x, s.y])));

    ld = s.O + s.H + s.F;
    at_e = double(isequal([s.x, s.y], p.E));
    feats = [1.0, dE/58.0, s.O/p.MAX_LOAD, s.H/p.MAX_LOAD, s.F/p.MAX_LOAD, ...
             s.M/750.0, rem, ld/p.MAX_LOAD, at_w, wp_rem, ...
             gf/30.0, s.c/5.0, at_s, tp, at_e];
end

% ==================== Skeleton Enumeration ====================
function [skels, nodePos] = enum_skeletons(p, mx_nodes, mx_cnt)
    nodePos = containers.Map({'B','E','S1','S2','W1','W2','W3'}, ...
        {p.B, p.E, p.S1, p.S2, p.W1, p.W2, p.W3});
    im = {'W1','W2','W3','S1','S2'};
    all_names = {'B','E','S1','S2','W1','W2','W3'};

    dist = containers.Map();
    for i = 1:length(all_names)
        for j = 1:length(all_names)
            key = [all_names{i}, '_', all_names{j}];
            dist(key) = md(nodePos(all_names{i}), nodePos(all_names{j}));
        end
    end

    global DFS_SKELS;
    DFS_SKELS = {};
    dfs('B', {'B'}, 0);
    skels = DFS_SKELS;
    clear global DFS_SKELS;

    function dfs(cur, path, tr)
        global DFS_SKELS;
        if length(DFS_SKELS) >= mx_cnt, return; end
        dE = dist([cur, '_E']);
        if tr + dE <= p.MAX_DAYS
            DFS_SKELS{end+1} = [path, {'E'}];
        end
        for n = 1:length(im)
            nxt = im{n};
            if strcmp(nxt, cur), continue; end
            if any(strcmp(cur, {'S1','S2'})) && any(strcmp(nxt, {'S1','S2'})), continue; end
            if length(path) > mx_nodes, continue; end
            d = dist([cur, '_', nxt]);
            if d == 0 || tr + d > p.MAX_DAYS, continue; end
            dfs(nxt, [path, {nxt}], tr + d);
        end
    end
end

function d = skel_dist(sk, nodePos)
    d = 0;
    for i = 1:length(sk)-1
        d = d + md(nodePos(sk{i}), nodePos(sk{i+1}));
    end
end

function uq = unique_skeletons(skels)
    uq = {};
    seen = containers.Map();
    for i = 1:length(skels)
        key = strjoin(skels{i}, '|');
        if ~isKey(seen, key)
            seen(key) = true;
            uq{end+1} = skels{i};
        end
    end
end

function sorted = sort_skels_by_dist(skels, nodePos)
    n = length(skels);
    dists = zeros(1, n);
    for i = 1:n, dists(i) = skel_dist(skels{i}, nodePos); end
    [~, idx] = sort(dists);
    sorted = skels(idx);
end

% ==================== Supply Computation ====================
function [bO, bH, bF] = comp_buy_supply(p, state, sk, seg, nodePos)
    no = 0; nh = 0; nf = 0;
    vo = 0; vh = 0; vf = 0;

    ns = length(sk);
    for j = seg+1:length(sk)
        if any(strcmp(sk{j}, {'S1','S2'})), ns = j; break; end
    end

    for j = seg:min(ns, length(sk))-1
        n1 = sk{j}; n2 = sk{min(j+1, length(sk))};
        d = md(nodePos(n1), nodePos(n2));
        no = no + p.EMOVE(1)*d; nh = nh + p.EMOVE(2)*d; nf = nf + p.EMOVE(3)*d;
        e2o = 0.8*4 + 0.2*64; vo = vo + (e2o - p.EMOVE(1)^2)*d;
        e2h = 0.8*9 + 0.2*16; vh = vh + (e2h - p.EMOVE(2)^2)*d;
        e2f = 0.8*4 + 0.2*9; vf = vf + (e2f - p.EMOVE(3)^2)*d;

        atW = false; wg_ = 0; mx_ = 0;
        if strcmp(n2, 'W1'), atW = true; wg_ = 20; mx_ = 4;
        elseif strcmp(n2, 'W2'), atW = true; wg_ = 15; mx_ = 5;
        elseif strcmp(n2, 'W3'), atW = true; wg_ = 28; mx_ = 3;
        end
        if atW
            ew = p.PN * mx_;
            no = no + p.EWORK(1)*ew; nh = nh + p.EWORK(2)*ew; nf = nf + p.EWORK(3)*ew;
            wv = mx_ * p.PN * p.PS;
            vo = vo + wv * (p.CWORK(1,1)-p.CWORK(2,1))^2;
            vh = vh + wv * (p.CWORK(1,2)-p.CWORK(2,2))^2;
            vf = vf + wv * (p.CWORK(1,3)-p.CWORK(2,3))^2;
        end
    end

    ec = no*2 + nh*1 + nf*2;
    cv = state.O*2 + state.H*1 + state.F*2;
    fn = max(0, ec - cv);
    sl = (state.M - fn)/max(1, fn);
    if fn <= 0, sl = 2.0; end
    z = 0.5 + 1.5 * min(1.0, max(0, sl));

    no = ceil(no + z * sqrt(max(0, vo)));
    nh = ceil(nh + z * sqrt(max(0, vh)));
    nf = ceil(nf + z * sqrt(max(0, vf)));

    bO = max(0, no - state.O);
    bH = max(0, nh - state.H);
    bF = max(0, nf - state.F);

    sp = p.MAX_LOAD - (state.O + state.H + state.F);
    tot = bO + bH + bF;
    if tot > sp && sp > 0
        sc = sp / tot; bO = floor(bO*sc); bH = floor(bH*sc); bF = floor(bF*sc);
    end
    cst = bO*2 + bH*1 + bF*2;
    if cst > state.M && state.M > 0
        sc = state.M / cst; bO = floor(bO*sc); bH = floor(bH*sc); bF = floor(bF*sc);
    end
end

% ==================== Base Simulator ====================
function [ok, Zf, Mf, log, svs] = sim_base(p, sk, nodePos)
    s = struct('x', p.B(1), 'y', p.B(2), 'O', p.INIT_O, 'H', p.INIT_H, ...
        'F', p.INIT_F, 'M', p.INIT_M, 'Z', p.INIT_Z, 'c', 0, 'day', 1);
    log = {}; svs = {}; si = 2;

    while s.day <= p.MAX_DAYS
        if isequal([s.x, s.y], p.E)
            for j = 1:length(svs), svs{j}.ret_Z = s.Z; svs{j}.ret_M = s.M; end
            ok = true; Zf = s.Z; Mf = s.M; return;
        end

        svs{end+1} = struct('state', s, 'ret_Z', 0, 'ret_M', 0);
        w = sw(p);
        sn = at_sp([s.x, s.y]);
        [wn, wg_val, wm_val] = at_wp([s.x, s.y]);

        wx = 'N'; if w == 1, wx = 'S'; end
        entry = struct('day', s.day, 'pos', [s.x, s.y], 'O', s.O, 'H', s.H, ...
            'F', s.F, 'M', s.M, 'Z', s.Z, 'c', s.c, 'weather', wx, ...
            'action', '', 'gain', 0, 'buy', [0,0,0], 'new_pos', [0,0]);
        log{end+1} = entry;

        % Supply check
        if ~isempty(sn) && si <= length(sk) && strcmp(sk{si}, sn)
            [bO, bH, bF] = comp_buy_supply(p, s, sk, si, nodePos);
            ns = apply_buy_act(s, bO, bH, bF);
            if isempty(ns), ok = false; Zf = 0; Mf = 0; return; end
            log{end}.action = sprintf('buy(%d,%d,%d)', bO, bH, bF);
            log{end}.buy = [bO, bH, bF];
            ns = apply_act(ns, 'idle', w);
            if ~state_ok(ns), ok = false; Zf = 0; Mf = 0; return; end
            s = ns; si = si + 1; continue;
        end

        % Work check
        if ~isempty(wn) && si <= length(sk) && strcmp(sk{si}, wn)
            if w == 0 && s.c < wm_val
                ns = apply_act(s, 'work', w, wg_val);
                log{end}.action = 'work'; log{end}.gain = wg_val;
            elseif s.c < wm_val && s.day < p.MAX_DAYS - 30
                ns = apply_act(s, 'idle', w);
                log{end}.action = 'idle_stay';
                ns.c = s.c;
            else
                ns = apply_act(s, 'idle', w);
                log{end}.action = 'idle_leave';
                si = si + 1;
            end
            if ~state_ok(ns), ok = false; Zf = 0; Mf = 0; return; end
            s = ns; continue;
        end

        % Movement
        if si > length(sk), ok = false; Zf = 0; Mf = 0; return; end
        tgt = nodePos(sk{si});
        if isequal([s.x, s.y], tgt), si = si + 1; continue; end
        npos = mv_toward(s, tgt);
        ns = apply_act(s, 'move', w, 0, npos);
        if ~state_ok(ns), ok = false; Zf = 0; Mf = 0; return; end
        log{end}.action = 'move'; log{end}.new_pos = npos;
        s = ns;
    end
    ok = false; Zf = 0; Mf = 0;
end

function [ok, Zf, Mf, svs] = sim_base_full(p, sk, nodePos)
    [ok, Zf, Mf, ~, svs] = sim_base(p, sk, nodePos);
end

% ==================== Linear Value Function ====================
function theta = fit_theta(samples)
    NF = 15;
    n = length(samples);
    X = zeros(n, NF);
    y = zeros(n, 1);
    for i = 1:n
        X(i,:) = samples{i}.feats;
        y(i) = samples{i}.ret;
    end
    theta = (X' * X + eye(NF)) \ (X' * y);
end

function v = predV(theta, s)
    p = get_params();
    if isempty(theta), v = 0;
    else, v = theta' * state_feats(p, s)'; end
end

% ==================== ADP Simulator ====================
function [ok, Zf, Mf, log, svs] = sim_adp(p, sk, nodePos, thZ, thM, ws)
    if nargin < 6, ws = []; end
    s = struct('x', p.B(1), 'y', p.B(2), 'O', p.INIT_O, 'H', p.INIT_H, ...
        'F', p.INIT_F, 'M', p.INIT_M, 'Z', p.INIT_Z, 'c', 0, 'day', 1);
    log = {}; svs = {}; si = 2;

    while s.day <= p.MAX_DAYS
        if isequal([s.x, s.y], p.E)
            for j = 1:length(svs), svs{j}.ret_Z = s.Z; svs{j}.ret_M = s.M; end
            ok = true; Zf = s.Z; Mf = s.M; return;
        end

        svs{end+1} = struct('state', s, 'ret_Z', 0, 'ret_M', 0);
        if ~isempty(ws), w = ws(s.day);
        else, w = sw(p); end
        sn = at_sp([s.x, s.y]);
        [wn, wg_val, wm_val] = at_wp([s.x, s.y]);

        wx = 'N'; if w == 1, wx = 'S'; end
        entry = struct('day', s.day, 'pos', [s.x, s.y], 'O', s.O, 'H', s.H, ...
            'F', s.F, 'M', s.M, 'Z', s.Z, 'c', s.c, 'weather', wx, ...
            'action', '', 'gain', 0, 'buy', [0,0,0], 'new_pos', [0,0]);
        log{end+1} = entry;

        % Supply
        if ~isempty(sn) && si <= length(sk) && strcmp(sk{si}, sn)
            [bO, bH, bF] = comp_buy_supply(p, s, sk, si, nodePos);
            ns = apply_buy_act(s, bO, bH, bF);
            if isempty(ns), ok = false; Zf = 0; Mf = 0; return; end
            log{end}.action = sprintf('buy(%d,%d,%d)', bO, bH, bF);
            log{end}.buy = [bO, bH, bF];
            ns = apply_act(ns, 'idle', w);
            if ~state_ok(ns), ok = false; Zf = 0; Mf = 0; return; end
            s = ns; si = si + 1; continue;
        end

        % Work - ADP decision
        if ~isempty(wn) && si <= length(sk) && strcmp(sk{si}, wn)
            ns_w_ok = false;
            if w == 0 && s.c < wm_val
                ns_w = apply_act(s, 'work', w, wg_val);
                if state_ok(ns_w), ns_w_ok = true; end
            end
            ns_i = apply_act(s, 'idle', w);
            ns_i_ok = state_ok(ns_i);
            if ns_w_ok && ns_i_ok
                pZw = predV(thZ, ns_w); pMw = predV(thM, ns_w);
                pZi = predV(thZ, ns_i); pMi = predV(thM, ns_i);
                if pZw > pZi || (pZw == pZi && pMw > pMi)
                    ns = ns_w; log{end}.action = 'work'; log{end}.gain = wg_val;
                else
                    ns = ns_i; log{end}.action = 'idle_leave'; si = si + 1;
                end
            elseif ns_i_ok
                ns = ns_i; log{end}.action = 'idle_leave'; si = si + 1;
            else
                ok = false; Zf = 0; Mf = 0; return;
            end
            s = ns; continue;
        end

        % Movement
        if si > length(sk), ok = false; Zf = 0; Mf = 0; return; end
        tgt = nodePos(sk{si});
        if isequal([s.x, s.y], tgt), si = si + 1; continue; end
        npos = mv_toward(s, tgt);
        ns = apply_act(s, 'move', w, 0, npos);
        if ~state_ok(ns), ok = false; Zf = 0; Mf = 0; return; end
        log{end}.action = 'move'; log{end}.new_pos = npos;
        s = ns;
    end
    ok = false; Zf = 0; Mf = 0;
end

function [ok, Zf, Mf, svs] = sim_adp_full(p, sk, nodePos, thZ, thM)
    [ok, Zf, Mf, ~, svs] = sim_adp(p, sk, nodePos, thZ, thM);
end

% ==================== Online Lookahead Tree ====================
function ba = tree_decide(p, thZ, thM, s, wi, wn, sk, si, nodePos, D)
    [~, wg_val, wm_val] = at_wp(nodePos(wn));

    acts = {};
    if wi == 0 && s.c < wm_val, acts{end+1} = 'work'; end
    acts{end+1} = 'idle';

    if length(acts) == 1, ba = acts{1}; return; end

    ba = ''; bv = -1e30;
    for a = 1:length(acts)
        v = q_est(p, thZ, thM, s, acts{a}, wi, wn, sk, si, nodePos, D, 0);
        if v > bv, bv = v; ba = acts{a}; end
    end
end

function v = q_est(p, thZ, thM, s, action, wi, wn, sk, si, nodePos, D, depth)
    [~, wg_val] = at_wp(nodePos(wn));

    if strcmp(action, 'work')
        ns = apply_act(s, 'work', wi, wg_val);
        nsi = si;
    else
        ns = apply_act(s, 'idle', wi);
        nsi = si + 1;
    end
    if ~state_ok(ns), v = -1e30; return; end
    if isequal([ns.x, ns.y], p.E), v = ns.Z * 100000 + ns.M; return; end
    if depth >= D
        v = predV(thZ, ns)*100000 + predV(thM, ns); return;
    end

    rn = rollout1(p, thZ, thM, ns, sk, nsi, nodePos, 0);
    if isnan(rn), vn = predV(thZ, ns)*100000 + predV(thM, ns);
    else, vn = rn; end

    rs = rollout1(p, thZ, thM, ns, sk, nsi, nodePos, 1);
    if isnan(rs), vs = predV(thZ, ns)*100000 + predV(thM, ns);
    else, vs = rs; end

    v = p.PN * vn + p.PS * vs;
end

function v = rollout1(p, thZ, thM, s, sk, si, nodePos, fw)
    if s.day >= p.MAX_DAYS || si > length(sk), v = NaN; return; end
    if isequal([s.x, s.y], p.E), v = s.Z * 100000 + s.M; return; end

    sn = at_sp([s.x, s.y]);
    [wn, wg_val, wm_val] = at_wp([s.x, s.y]);

    if ~isempty(sn)
        [bO, bH, bF] = comp_buy_supply(p, s, sk, si, nodePos);
        ns = apply_buy_act(s, bO, bH, bF);
        if isempty(ns), v = NaN; return; end
        ns = apply_act(ns, 'idle', fw);
        if ~state_ok(ns), v = NaN; return; end
        v = predV(thZ, ns)*100000 + predV(thM, ns); return;
    end

    if ~isempty(wn)
        if fw == 0 && s.c < wm_val
            ns = apply_act(s, 'work', fw, wg_val);
        else
            ns = apply_act(s, 'idle', fw);
        end
        if ~state_ok(ns), v = NaN; return; end
        v = predV(thZ, ns)*100000 + predV(thM, ns); return;
    end

    tgt = nodePos(sk{si});
    if isequal([s.x, s.y], tgt)
        v = predV(thZ, s)*100000 + predV(thM, s); return;
    end
    npos = mv_toward(s, tgt);
    ns = apply_act(s, 'move', fw, 0, npos);
    if ~state_ok(ns), v = NaN; return; end
    v = predV(thZ, ns)*100000 + predV(thM, ns);
end

% ==================== Rollout Refiner ====================
function [ok, Zf, Mf, log] = refine_sim(p, thZ, thM, sk, nodePos, D)
    s = struct('x', p.B(1), 'y', p.B(2), 'O', p.INIT_O, 'H', p.INIT_H, ...
        'F', p.INIT_F, 'M', p.INIT_M, 'Z', p.INIT_Z, 'c', 0, 'day', 1);
    log = {}; si = 2;

    while s.day <= p.MAX_DAYS
        if isequal([s.x, s.y], p.E)
            ok = true; Zf = s.Z; Mf = s.M; return;
        end
        w = sw(p);
        sn = at_sp([s.x, s.y]);
        [wn, wg_val] = at_wp([s.x, s.y]);

        wx = 'N'; if w == 1, wx = 'S'; end
        entry = struct('day', s.day, 'pos', [s.x, s.y], 'O', s.O, 'H', s.H, ...
            'F', s.F, 'M', s.M, 'Z', s.Z, 'c', s.c, 'weather', wx, ...
            'action', '', 'gain', 0, 'buy', [0,0,0], 'new_pos', [0,0]);
        log{end+1} = entry;

        % Supply
        if ~isempty(sn) && si <= length(sk) && strcmp(sk{si}, sn)
            [bO, bH, bF] = comp_buy_supply(p, s, sk, si, nodePos);
            ns = apply_buy_act(s, bO, bH, bF);
            if isempty(ns), ok = false; Zf = 0; Mf = 0; return; end
            log{end}.action = sprintf('buy(%d,%d,%d)', bO, bH, bF);
            log{end}.buy = [bO, bH, bF];
            ns = apply_act(ns, 'idle', w);
            if ~state_ok(ns), ok = false; Zf = 0; Mf = 0; return; end
            s = ns; si = si + 1; continue;
        end

        % Work point with online tree decision
        if ~isempty(wn) && si <= length(sk) && strcmp(sk{si}, wn)
            ba = tree_decide(p, thZ, thM, s, w, wn, sk, si, nodePos, D);
            if strcmp(ba, 'work')
                ns = apply_act(s, 'work', w, wg_val);
                log{end}.action = 'work'; log{end}.gain = wg_val;
            else
                ns = apply_act(s, 'idle', w);
                log{end}.action = 'idle_leave'; si = si + 1;
            end
            if ~state_ok(ns), ok = false; Zf = 0; Mf = 0; return; end
            s = ns; continue;
        end

        % Movement
        if si > length(sk), ok = false; Zf = 0; Mf = 0; return; end
        tgt = nodePos(sk{si});
        if isequal([s.x, s.y], tgt), si = si + 1; continue; end
        npos = mv_toward(s, tgt);
        ns = apply_act(s, 'move', w, 0, npos);
        if ~state_ok(ns), ok = false; Zf = 0; Mf = 0; return; end
        log{end}.action = 'move'; log{end}.new_pos = npos;
        s = ns;
    end
    ok = false; Zf = 0; Mf = 0;
end

% ==================== Helpers ====================
function sorted = sort_br(br)
    n = length(br);
    if n == 0, sorted = {}; return; end
    keys = zeros(n, 2);
    for i = 1:n, keys(i,:) = [br{i}.avgZ, br{i}.avgM]; end
    [~, idx] = sortrows(keys, [-1, -2]);
    sorted = br(idx);
end

function data = build_xls_data(p, result)
    lg = result.best_log;
    nrows = length(lg);
    data = cell(nrows + 1, 15);
    data(1,:) = {0, p.B(1), p.B(2), '-', 'Start', 'B', ...
                 p.INIT_O, p.INIT_H, p.INIT_F, p.INIT_M, p.INIT_Z, 0, 0, 0, 0};
    for i = 1:nrows
        e = lg{i};
        data(i+1,:) = {e.day, e.pos(1), e.pos(2), e.weather, e.action, ...
            point_name(e.pos), e.O, e.H, e.F, e.M, e.Z, e.c, ...
            e.buy(1), e.buy(2), e.buy(3)};
    end
end

function nm = point_name(pos)
    p = get_params();
    nm = '';
    if isequal(pos, p.W1), nm = 'W1';
    elseif isequal(pos, p.W2), nm = 'W2';
    elseif isequal(pos, p.W3), nm = 'W3';
    elseif isequal(pos, p.S1), nm = 'S1';
    elseif isequal(pos, p.S2), nm = 'S2';
    elseif isequal(pos, p.B), nm = 'B';
    elseif isequal(pos, p.E), nm = 'E';
    end
end
