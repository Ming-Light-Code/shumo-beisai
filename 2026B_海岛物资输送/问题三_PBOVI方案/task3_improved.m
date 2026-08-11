%% ==========================================================================
% Problem 3: Improved Solver
% Key improvements over PBOVI:
%   1. Dynamic re-planning at each waypoint (not fixed skeleton)
%   2. Smart bidirectional movement (not x-priority)
%   3. Risk-aware supply procurement with storm-streak modeling
%   4. Threshold-based work/leave decision using resource projection
%   5. Emergency re-routing to nearest supply point
% ==========================================================================

function task3_improved_main(seed, n_sim)
    if nargin < 1, seed = 42; end
    if nargin < 2, n_sim = 2000; end

    t0 = tic;
    rng(seed);
    p = init_params();
    D = precompute_dists(p);

    fprintf('=============================================================\n');
    fprintf('  Problem 3: Improved Solver\n');
    fprintf('  Method: Dynamic Re-planning + Risk-Aware Decision Making\n');
    fprintf('  MC simulations: %d\n', n_sim);
    fprintf('=============================================================\n\n');

    total_Z = 0; total_M = 0; successes = 0;
    best_Z = -1; best_M = -1; best_log = {};
    n_resource = 0; n_timeout = 0; n_other = 0;

    for k = 1:n_sim
        if mod(k, 200) == 0
            fprintf('  %d/%d (%.1fs, succ=%.1f%%)\n', ...
                k, n_sim, toc(t0), 100*successes/k);
        end

        [ok, Zf, Mf, lg, reason] = simulate_episode(p, D);
        if ok
            total_Z = total_Z + Zf;
            total_M = total_M + Mf;
            successes = successes + 1;
            if Zf > best_Z || (Zf == best_Z && Mf > best_M)
                best_Z = Zf; best_M = Mf; best_log = lg;
            end
        else
            if strcmp(reason, 'resource'), n_resource = n_resource + 1;
            elseif strcmp(reason, 'timeout'), n_timeout = n_timeout + 1;
            else, n_other = n_other + 1; end
        end
    end

    elapsed = toc(t0);
    avgZ = total_Z / max(1, successes);
    avgM = total_M / max(1, successes);
    rate = 100 * successes / n_sim;

    fprintf('\n  ==== RESULTS ====\n');
    fprintf('  Success: %d/%d (%.1f%%)\n', successes, n_sim, rate);
    fprintf('  MC E[Z] = %.1f,  MC E[M] = %.1f\n', avgZ, avgM);
    fprintf('  Best: Z = %d,  M = %d\n', best_Z, best_M);
    fprintf('  Failures: resource=%d, timeout=%d, other=%d\n', ...
        n_resource, n_timeout, n_other);
    fprintf('  Elapsed: %.1fs\n\n', elapsed);

    if ~isempty(best_log)
        T = build_table(p, best_log);
        writetable(T, 'result_improved.xlsx', 'Sheet', 'Improved');
        fprintf('  Exported: result_improved.xlsx\n');
    end

    fid = fopen('task3_improved_result.json', 'w');
    fprintf(fid, '{\n');
    fprintf(fid, '  "problem": "Task 3 (Improved)",\n');
    fprintf(fid, '  "method": "Dynamic Re-planning + Risk-Aware DP",\n');
    fprintf(fid, '  "mc_expected_Z": %.2f,\n', avgZ);
    fprintf(fid, '  "mc_expected_M": %.2f,\n', avgM);
    fprintf(fid, '  "best_Z": %d,\n', best_Z);
    fprintf(fid, '  "best_M": %d,\n', best_M);
    fprintf(fid, '  "success_rate": %.4f,\n', successes/n_sim);
    fprintf(fid, '  "elapsed_s": %.2f\n', elapsed);
    fprintf(fid, '}\n');
    fclose(fid);
    fprintf('  Exported: task3_improved_result.json\n');
end

%% ==================== Parameters ====================
function p = init_params()
    p.GRID = 30;
    p.B  = [1, 15];    p.E  = [30, 15];
    p.S1 = [12, 16];   p.S2 = [21, 16];
    p.W1 = [6, 21];    p.W2 = [15, 9];    p.W3 = [24, 24];

    p.CMOVE = [2,3,2; 8,4,3];
    p.CIDLE = [1,1,1; 3,3,2];
    p.CWORK = [5,4,3; 8,6,6];

    p.P_NORM  = 0.8;  p.P_STORM = 0.2;

    p.PRICES  = [2, 1, 2];
    p.INIT_O  = 100;  p.INIT_H = 150;  p.INIT_F = 100;
    p.INIT_M  = 750;  p.INIT_Z = 200;
    p.MAX_LOAD = 400; p.MAX_DAYS = 90;

    p.WP_GAIN = [20, 15, 28];
    p.WP_MAX  = [4,  5,  3];

    p.WP_NAMES = {'W1', 'W2', 'W3', 'S1', 'S2', 'B', 'E'};
    p.WP_POS   = containers.Map(p.WP_NAMES, ...
        {p.W1, p.W2, p.W3, p.S1, p.S2, p.B, p.E});
    p.WORK_NAMES = {'W1', 'W2', 'W3'};
    p.SUPPLY_NAMES = {'S1', 'S2'};

    p.SAFETY_FACTOR = 1.3;
    p.STORM_STREAK_PROB = 0.05;
end

%% ==================== Distance Precomputation ====================
function D = precompute_dists(p)
    names = p.WP_NAMES;
    n = length(names);
    D = containers.Map();
    for i = 1:n
        for j = 1:n
            key = [names{i},'_',names{j}];
            D(key) = md(p.WP_POS(names{i}), p.WP_POS(names{j}));
        end
    end
end

function d = md(a, b)
    d = abs(a(1)-b(1)) + abs(a(2)-b(2));
end

%% ==================== Main Simulation ====================
function [ok, Zf, Mf, log, reason] = simulate_episode(p, D)
    s = struct('x', p.B(1), 'y', p.B(2), ...
        'O', p.INIT_O, 'H', p.INIT_H, 'F', p.INIT_F, ...
        'M', p.INIT_M, 'Z', p.INIT_Z, 'c', 0, 'day', 1);
    log = {};
    completed = [false, false, false];
    target = [];

    while s.day <= p.MAX_DAYS
        if isequal([s.x, s.y], p.E)
            ok = true; Zf = s.Z; Mf = s.M; reason = ''; return;
        end

        w = sample_weather(p);

        wx_char = 'N'; if w == 1, wx_char = 'S'; end
        entry = struct('day', s.day, 'pos', [s.x, s.y], ...
            'O', s.O, 'H', s.H, 'F', s.F, 'M', s.M, 'Z', s.Z, ...
            'c', s.c, 'weather', wx_char, 'action', '', ...
            'gain', 0, 'buy', [0,0,0], 'new_pos', [0,0]);
        log{end+1} = entry;

        wp_name = which_waypoint(p, [s.x, s.y]);

        % At B (start), just plan route and continue to movement
        if strcmp(wp_name, 'B')
            [next_name, ~] = plan_route(p, D, s, completed);
            if isempty(next_name)
                ok = false; Zf = 0; Mf = 0; reason = 'timeout'; return;
            end
            target = p.WP_POS(next_name);
            entry.action = 'Start';
            log{end} = entry;
            % Fall through to movement
        elseif ~isempty(wp_name)
            [act, s, target, completed, entry] = ...
                handle_waypoint(p, D, s, w, wp_name, completed, entry);

            if isempty(s)
                ok = false; Zf = 0; Mf = 0; reason = 'resource'; return;
            end
            log{end} = entry;
            s.day = s.day + 1;
            if ~state_ok(p, s)
                ok = false; Zf = 0; Mf = 0; reason = 'resource'; return;
            end
            continue;
        end

        if isempty(target)
            [next_name, ~] = plan_route(p, D, s, completed);
            if isempty(next_name)
                ok = false; Zf = 0; Mf = 0; reason = 'timeout'; return;
            end
            target = p.WP_POS(next_name);
        end

        if is_emergency(p, s, target)
            [sp_name, sp_pos] = nearest_supply(p, s);
            if ~isempty(sp_name)
                target = sp_pos;
            end
        end

        [ns, new_pos] = move_toward(p, s, target, w);
        if isempty(ns)
            ok = false; Zf = 0; Mf = 0; reason = 'resource'; return;
        end

        entry.action = 'move';
        entry.new_pos = new_pos;
        log{end} = entry;
        s = ns;
        s.c = 0;
        s.day = s.day + 1;

        if ~state_ok(p, s)
            ok = false; Zf = 0; Mf = 0; reason = 'resource'; return;
        end
    end

    ok = false; Zf = 0; Mf = 0; reason = 'timeout';
end

%% ==================== Weather Sampling ====================
function w = sample_weather(p)
    w = double(rand() >= p.P_NORM);
end

%% ==================== Waypoint Identification ====================
function nm = which_waypoint(p, pos)
    nm = '';
    if isequal(pos, p.B),  nm = 'B';
    elseif isequal(pos, p.E),  nm = 'E';
    elseif isequal(pos, p.S1), nm = 'S1';
    elseif isequal(pos, p.S2), nm = 'S2';
    elseif isequal(pos, p.W1), nm = 'W1';
    elseif isequal(pos, p.W2), nm = 'W2';
    elseif isequal(pos, p.W3), nm = 'W3';
    end
end

function idx = workpoint_index(p, nm)
    idx = find(strcmp(p.WORK_NAMES, nm));
end

function tf = is_supply(p, nm)
    tf = any(strcmp(p.SUPPLY_NAMES, nm));
end

%% ==================== State Checking ====================
function ok = state_ok(p, s)
    ok = s.O >= 0 && s.H >= 0 && s.F >= 0 && s.M >= 0 && ...
        (s.O + s.H + s.F) <= p.MAX_LOAD;
end

%% ==================== Consumption ====================
function cr = consume_rate(p, action, wi)
    switch action
        case 'move',  cr = p.CMOVE(wi+1, :);
        case 'idle',  cr = p.CIDLE(wi+1, :);
        case 'work',  cr = p.CWORK(wi+1, :);
        otherwise,    cr = [0, 0, 0];
    end
end

function ns = apply_consumption(s, action, wi, p)
    cr = consume_rate(p, action, wi);
    ns = s;
    ns.O = ns.O - cr(1);
    ns.H = ns.H - cr(2);
    ns.F = ns.F - cr(3);
end

%% ==================== Smart Movement ====================
function [ns, new_pos] = move_toward(p, s, target, wi)
    dx = target(1) - s.x;
    dy = target(2) - s.y;

    if dx == 0 && dy == 0
        new_pos = [s.x, s.y];
    elseif abs(dx) >= abs(dy)
        new_pos = [s.x + sign(dx), s.y];
    else
        new_pos = [s.x, s.y + sign(dy)];
    end

    ns = apply_consumption(s, 'move', wi, p);
    ns.x = new_pos(1); ns.y = new_pos(2);
    if ~state_ok(p, ns), ns = []; end
end

%% ==================== Emergency Detection ====================
function tf = is_emergency(p, s, target)
    dist = md([s.x, s.y], target);
    wc_O = dist * p.CMOVE(2, 1);
    wc_H = dist * p.CMOVE(2, 2);
    wc_F = dist * p.CMOVE(2, 3);
    exp_O = dist * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
    exp_H = dist * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
    exp_F = dist * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));
    margin = 10;

    % Use expected consumption * safety, not worst-case
    if s.O < exp_O * 1.5 + margin || s.H < exp_H * 1.5 + margin || s.F < exp_F * 1.5 + margin
        tf = true; return;
    end

    dist_to_E = md([s.x, s.y], p.E);
    if s.day + dist_to_E > p.MAX_DAYS + 2
        tf = true; return;
    end

    tf = false;
end

function [name, pos] = nearest_supply(p, s)
    d1 = md([s.x, s.y], p.S1);
    d2 = md([s.x, s.y], p.S2);
    if d1 <= d2
        name = 'S1'; pos = p.S1;
    else
        name = 'S2'; pos = p.S2;
    end
end

%% ==================== Waypoint Handler ====================
function [act, s, target, completed, entry] = ...
        handle_waypoint(p, D, s, w, wp_name, completed, entry)

    act = ''; target = [];

    if strcmp(wp_name, 'B')
        [next_name, ~] = plan_route(p, D, s, completed);
        if isempty(next_name), s = []; return; end
        target = p.WP_POS(next_name);
        act = 'start';
        entry.action = 'Start';
        return;  % Don't increment day - let simulate_episode continue to movement
    end

    if is_supply(p, wp_name)
        [next_name, plan_seq] = plan_route(p, D, s, completed);
        if isempty(next_name)
            target = p.E;
        else
            target = p.WP_POS(next_name);
        end

        [s, buy] = purchase_supplies(p, D, s, plan_seq);
        if isempty(s), return; end

        ns = apply_consumption(s, 'idle', w, p);
        if ~state_ok(p, ns), s = []; return; end
        s = ns;

        act = 'buy';
        entry.action = sprintf('buy(%%d,%%d,%%d)', buy(1), buy(2), buy(3));
        entry.buy = buy;
        return;
    end

    if any(strcmp(p.WORK_NAMES, wp_name))
        idx = workpoint_index(p, wp_name);
        gain = p.WP_GAIN(idx); max_days = p.WP_MAX(idx);

        if completed(idx)
            [next_name, ~] = plan_route(p, D, s, completed);
            if isempty(next_name), s = []; return; end
            target = p.WP_POS(next_name);
            entry.action = 'idle_leave';
            ns = apply_consumption(s, 'idle', w, p);
            if ~state_ok(p, ns), s = []; return; end
            s = ns;
            return;
        end

        decision = work_decision(p, D, s, w, idx, completed);

        switch decision
            case 'work'
                ns = apply_consumption(s, 'work', w, p);
                if ~state_ok(p, ns), s = []; return; end
                ns.Z = ns.Z + gain;
                ns.c = ns.c + 1;
                s = ns;
                entry.action = 'work';
                entry.gain = gain;

                if s.c >= max_days
                    completed(idx) = true;
                    s.c = 0;
                    [next_name, ~] = plan_route(p, D, s, completed);
                    if isempty(next_name), s = []; return; end
                    target = p.WP_POS(next_name);
                end

            case 'idle_stay'
                ns = apply_consumption(s, 'idle', w, p);
                if ~state_ok(p, ns), s = []; return; end
                ns.c = s.c;
                s = ns;
                entry.action = 'idle_stay';

            case 'leave'
                s.c = 0;
                [next_name, ~] = plan_route(p, D, s, completed);
                if isempty(next_name), s = []; return; end
                target = p.WP_POS(next_name);
                ns = apply_consumption(s, 'idle', w, p);
                if ~state_ok(p, ns), s = []; return; end
                s = ns;
                entry.action = 'idle_leave';
        end
        return;
    end
end

%% ==================== Route Planning (Dynamic Re-planning) ====================
function [next_name, best_seq] = plan_route(p, D, s, completed)
    rem_work = {};
    rem_idx = [];
    for i = 1:3
        if ~completed(i)
            rem_work{end+1} = p.WORK_NAMES{i};
            rem_idx(end+1) = i;
        end
    end

    if isempty(rem_work)
        d_to_E = md([s.x, s.y], p.E);
        if s.day + d_to_E <= p.MAX_DAYS
            next_name = 'E'; best_seq = {'E'}; return;
        else
            next_name = []; best_seq = {}; return;
        end
    end

    best_score = -inf;
    best_seq = {};
    best_next = [];

    d_to_E = md([s.x, s.y], p.E);
    if s.day + d_to_E <= p.MAX_DAYS
        score = s.Z * 1000 + s.M;
        if score > best_score
            best_score = score;
            best_seq = {'E'};
            best_next = 'E';
        end
    end

    nw = length(rem_work);
    if nw <= 6
        perms_list = perms(1:nw);
    else
        perms_list = 1:nw;
    end

    for pi = 1:size(perms_list, 1)
        order = perms_list(pi, :);
        wp_seq_names = rem_work(order);

        [feasible, seq, est_Z] = build_feasible_seq(p, D, s, wp_seq_names, completed);
        if ~feasible, continue; end

        score = est_Z * 1000 + s.M;
        if score > best_score
            best_score = score;
            best_seq = seq;
            best_next = seq{1};
        end
    end

    if isempty(best_next)
        next_name = []; best_seq = {};
    else
        next_name = best_next;
    end
end

%% ==================== Supply Procurement (Risk-Aware) ====================
function [ns, buy] = purchase_supplies(p, D, s, plan_seq)
    if isempty(plan_seq) || strcmp(plan_seq{1}, 'E')
        d = md([s.x, s.y], p.E);
        need_O = d * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
        need_H = d * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
        need_F = d * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));
    else
        [need_O, need_H, need_F] = estimate_seq_needs(p, D, s, plan_seq);
    end

    k = 1;
    while p.P_STORM^k > p.STORM_STREAK_PROB && k < 10
        k = k + 1;
    end
    dmO = max(0, p.CMOVE(2,1) - (p.P_NORM*p.CMOVE(1,1) + p.P_STORM*p.CMOVE(2,1)));
    dmH = max(0, p.CMOVE(2,2) - (p.P_NORM*p.CMOVE(1,2) + p.P_STORM*p.CMOVE(2,2)));
    dmF = max(0, p.CMOVE(2,3) - (p.P_NORM*p.CMOVE(1,3) + p.P_STORM*p.CMOVE(2,3)));
    storm_extra_O = k * dmO;
    storm_extra_H = k * dmH;
    storm_extra_F = k * dmF;

    target_O = ceil(need_O * p.SAFETY_FACTOR + storm_extra_O);
    target_H = ceil(need_H * p.SAFETY_FACTOR + storm_extra_H);
    target_F = ceil(need_F * p.SAFETY_FACTOR + storm_extra_F);

    buy_O = max(0, target_O - s.O);
    buy_H = max(0, target_H - s.H);
    buy_F = max(0, target_F - s.F);

    spare = p.MAX_LOAD - (s.O + s.H + s.F);
    total_buy = buy_O + buy_H + buy_F;
    if total_buy > spare && spare > 0
        scale = spare / total_buy;
        buy_O = floor(buy_O * scale);
        buy_H = floor(buy_H * scale);
        buy_F = floor(buy_F * scale);
        total_buy = buy_O + buy_H + buy_F;
        rem = spare - total_buy;
        if rem > 0, buy_H = buy_H + rem; end
    end

    cost = buy_O*p.PRICES(1) + buy_H*p.PRICES(2) + buy_F*p.PRICES(3);
    if cost > s.M
        scale = s.M / cost;
        buy_O = floor(buy_O * scale);
        buy_H = floor(buy_H * scale);
        buy_F = floor(buy_F * scale);
        cost = buy_O*p.PRICES(1) + buy_H*p.PRICES(2) + buy_F*p.PRICES(3);
    end

    ns = s;
    ns.O = ns.O + buy_O;
    ns.H = ns.H + buy_H;
    ns.F = ns.F + buy_F;
    ns.M = ns.M - cost;

    if ~state_ok(p, ns), ns = []; end
    buy = [buy_O, buy_H, buy_F];
end

function [need_O, need_H, need_F] = estimate_seq_needs(p, D, s, plan_seq)
    cur_pos = [s.x, s.y];
    tot_O = 0; tot_H = 0; tot_F = 0;

    for i = 1:length(plan_seq)
        wn = plan_seq{i};
        wpos = p.WP_POS(wn);
        d = md(cur_pos, wpos);

        tot_O = tot_O + d * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
        tot_H = tot_H + d * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
        tot_F = tot_F + d * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));

        if strcmp(wn, 'E'), break;
        elseif is_supply(p, wn), break;
        else
            widx = workpoint_index(p, wn);
            wd = p.WP_MAX(widx);
            tot_O = tot_O + wd * (p.P_NORM * p.CWORK(1,1) + p.P_STORM * p.CWORK(2,1));
            tot_H = tot_H + wd * (p.P_NORM * p.CWORK(1,2) + p.P_STORM * p.CWORK(2,2));
            tot_F = tot_F + wd * (p.P_NORM * p.CWORK(1,3) + p.P_STORM * p.CWORK(2,3));
            tot_O = tot_O + (p.P_NORM * p.CIDLE(1,1) + p.P_STORM * p.CIDLE(2,1));
            tot_H = tot_H + (p.P_NORM * p.CIDLE(1,2) + p.P_STORM * p.CIDLE(2,2));
            tot_F = tot_F + (p.P_NORM * p.CIDLE(1,3) + p.P_STORM * p.CIDLE(2,3));
        end
        cur_pos = wpos;
    end

    need_O = tot_O; need_H = tot_H; need_F = tot_F;
end

%% ==================== Sequence Builder ====================
function [feasible, seq, est_Z] = build_feasible_seq(p, D, s, wp_seq_names, completed)
    cur_pos = [s.x, s.y];
    cur_day = s.day;
    cur_O = s.O; cur_H = s.H; cur_F = s.F; cur_M = s.M;

    seq = {};
    est_Z = s.Z;

    remaining = [false, false, false];
    for i = 1:3
        if ~completed(i), remaining(i) = true; end
    end

    for wi = 1:length(wp_seq_names)
        wn = wp_seq_names{wi};
        wpos = p.WP_POS(wn);
        widx = workpoint_index(p, wn);

        if ~remaining(widx), continue; end

        d = md(cur_pos, wpos);
        cur_day = cur_day + d;
        if cur_day > p.MAX_DAYS, feasible = false; return; end

        exp_O = d * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
        exp_H = d * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
        exp_F = d * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));
        cur_O = cur_O - exp_O; cur_H = cur_H - exp_H; cur_F = cur_F - exp_F;

        need_supply = (cur_O < -20) || (cur_H < -20) || (cur_F < -20);
        if need_supply
            [sp_name, sp_pos] = nearest_supply_pos(p, cur_pos);
            % Undo direct WP->WP travel
            cur_day = cur_day - d;
            cur_O = cur_O + exp_O;
            cur_H = cur_H + exp_H;
            cur_F = cur_F + exp_F;
            % Travel to supply point
            d_to_sp = md(cur_pos, sp_pos);
            cur_day = cur_day + d_to_sp + 1;
            cur_O = cur_O - d_to_sp * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
            cur_H = cur_H - d_to_sp * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
            cur_F = cur_F - d_to_sp * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));

            rem_dist = compute_remaining_dist(p, D, sp_pos, wp_seq_names(wi:end));
            idx2 = workpoint_index(p, wp_seq_names{wi});
            need_O_after = rem_dist * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1)) ...
                + p.WP_MAX(idx2) * (p.P_NORM * p.CWORK(1,1) + p.P_STORM * p.CWORK(2,1));
            need_H_after = rem_dist * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2)) ...
                + p.WP_MAX(idx2) * (p.P_NORM * p.CWORK(1,2) + p.P_STORM * p.CWORK(2,2));
            need_F_after = rem_dist * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3)) ...
                + p.WP_MAX(idx2) * (p.P_NORM * p.CWORK(1,3) + p.P_STORM * p.CWORK(2,3));
            need_O_after = need_O_after * p.SAFETY_FACTOR;
            need_H_after = need_H_after * p.SAFETY_FACTOR;
            need_F_after = need_F_after * p.SAFETY_FACTOR;

            buy_O = max(0, ceil(need_O_after - cur_O));
            buy_H = max(0, ceil(need_H_after - cur_H));
            buy_F = max(0, ceil(need_F_after - cur_F));

            cost = buy_O*p.PRICES(1) + buy_H*p.PRICES(2) + buy_F*p.PRICES(3);
            if cost > cur_M, feasible = false; return; end
            cur_M = cur_M - cost;
            cur_O = proj_O + buy_O; cur_H = proj_H + buy_H; cur_F = proj_F + buy_F;
            cur_day = cur_day_tmp;

            seq{end+1} = sp_name;
            cur_pos = sp_pos;

            d_sp_wp = md(sp_pos, wpos);
            cur_day = cur_day + d_sp_wp;
            cur_O = cur_O - d_sp_wp * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
            cur_H = cur_H - d_sp_wp * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
            cur_F = cur_F - d_sp_wp * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));
        end

        seq{end+1} = wn;
        cur_pos = wpos;

        work_days = p.WP_MAX(widx);
        cur_day = cur_day + work_days + 1;

        cur_O = cur_O - work_days * (p.P_NORM * p.CWORK(1,1) + p.P_STORM * p.CWORK(2,1));
        cur_H = cur_H - work_days * (p.P_NORM * p.CWORK(1,2) + p.P_STORM * p.CWORK(2,2));
        cur_F = cur_F - work_days * (p.P_NORM * p.CWORK(1,3) + p.P_STORM * p.CWORK(2,3));
        cur_O = cur_O - (p.P_NORM * p.CIDLE(1,1) + p.P_STORM * p.CIDLE(2,1));
        cur_H = cur_H - (p.P_NORM * p.CIDLE(1,2) + p.P_STORM * p.CIDLE(2,2));
        cur_F = cur_F - (p.P_NORM * p.CIDLE(1,3) + p.P_STORM * p.CIDLE(2,3));
        est_Z = est_Z + p.WP_GAIN(widx) * work_days;
        remaining(widx) = false;
    end

    d_final = md(cur_pos, p.E);
    cur_day = cur_day + d_final;
    seq{end+1} = 'E';

    if cur_day > p.MAX_DAYS + 5
        feasible = false; return;
    end

    feasible = true;
end

function [name, pos] = nearest_supply_pos(p, cur_pos)
    d1 = md(cur_pos, p.S1);
    d2 = md(cur_pos, p.S2);
    if d1 <= d2, name = 'S1'; pos = p.S1;
    else, name = 'S2'; pos = p.S2; end
end

function dist = compute_remaining_dist(p, D, pos, wp_seq)
    dist = 0; cur = pos;
    for i = 1:length(wp_seq)
        nxt = p.WP_POS(wp_seq{i});
        dist = dist + md(cur, nxt);
        cur = nxt;
    end
    dist = dist + md(cur, p.E);
end

%% ==================== Work Decision ====================
function decision = work_decision(p, D, s, w, widx, completed)
    quota = p.WP_MAX(widx) - s.c;
    if quota <= 0, decision = 'leave'; return; end

    cr = consume_rate(p, 'work', w);
    after_O = s.O - cr(1);
    after_H = s.H - cr(2);
    after_F = s.F - cr(3);

    if after_O < 0 || after_H < 0 || after_F < 0
        decision = 'leave'; return;
    end

    tmp_completed = completed;
    tmp_completed(widx) = true;

    [need_O, need_H, need_F, min_days] = ...
        estimate_remaining_needs(p, D, s, tmp_completed);

    remaining_days = p.MAX_DAYS - s.day;
    if remaining_days < min_days
        decision = 'leave'; return;
    end

    safe_O = need_O * p.SAFETY_FACTOR;
    safe_H = need_H * p.SAFETY_FACTOR;
    safe_F = need_F * p.SAFETY_FACTOR;

    if after_O >= safe_O && after_H >= safe_H && after_F >= safe_F
        decision = 'work'; return;
    end

    cr_idle = consume_rate(p, 'idle', w);
    idle_O = s.O - cr_idle(1);
    idle_H = s.H - cr_idle(2);
    idle_F = s.F - cr_idle(3);

    if idle_O >= safe_O && idle_H >= safe_H && idle_F >= safe_F && w == 1
        decision = 'idle_stay'; return;
    end

    decision = 'leave';
end

function [need_O, need_H, need_F, min_days] = ...
        estimate_remaining_needs(p, D, s, completed)
    rem_work = {};
    for i = 1:3
        if ~completed(i)
            rem_work{end+1} = p.WORK_NAMES{i};
        end
    end

    if isempty(rem_work)
        d = md([s.x, s.y], p.E);
        need_O = d * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
        need_H = d * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
        need_F = d * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));
        min_days = d;
        return;
    end

    nw = length(rem_work);
    best_days = inf;
    best_O = inf; best_H = inf; best_F = inf;

    perms_list = perms(1:nw);

    for pi = 1:size(perms_list, 1)
        order = perms_list(pi, :);
        cur = [s.x, s.y];
        days = 0; tot_O = 0; tot_H = 0; tot_F = 0;

        for wi = 1:nw
            wn = rem_work{order(wi)};
            wpos = p.WP_POS(wn);
            widx = workpoint_index(p, wn);

            d = md(cur, wpos);
            days = days + d;
            tot_O = tot_O + d * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
            tot_H = tot_H + d * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
            tot_F = tot_F + d * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));

            wd = p.WP_MAX(widx);
            days = days + wd + 1;
            tot_O = tot_O + wd * (p.P_NORM * p.CWORK(1,1) + p.P_STORM * p.CWORK(2,1));
            tot_H = tot_H + wd * (p.P_NORM * p.CWORK(1,2) + p.P_STORM * p.CWORK(2,2));
            tot_F = tot_F + wd * (p.P_NORM * p.CWORK(1,3) + p.P_STORM * p.CWORK(2,3));
            tot_O = tot_O + (p.P_NORM * p.CIDLE(1,1) + p.P_STORM * p.CIDLE(2,1));
            tot_H = tot_H + (p.P_NORM * p.CIDLE(1,2) + p.P_STORM * p.CIDLE(2,2));
            tot_F = tot_F + (p.P_NORM * p.CIDLE(1,3) + p.P_STORM * p.CIDLE(2,3));
            cur = wpos;
        end

        d_final = md(cur, p.E);
        days = days + d_final;
        tot_O = tot_O + d_final * (p.P_NORM * p.CMOVE(1,1) + p.P_STORM * p.CMOVE(2,1));
        tot_H = tot_H + d_final * (p.P_NORM * p.CMOVE(1,2) + p.P_STORM * p.CMOVE(2,2));
        tot_F = tot_F + d_final * (p.P_NORM * p.CMOVE(1,3) + p.P_STORM * p.CMOVE(2,3));

        if days < best_days
            best_days = days; best_O = tot_O; best_H = tot_H; best_F = tot_F;
        elseif days == best_days
            best_O = min(best_O, tot_O);
            best_H = min(best_H, tot_H);
            best_F = min(best_F, tot_F);
        end
    end

    need_O = best_O; need_H = best_H; need_F = best_F; min_days = best_days;
end

%% ==================== Output Table ====================
function T = build_table(p, log)
    n = length(log);
    data = cell(n+1, 15);
    data(1,:) = {0, p.B(1), p.B(2), '-', 'Start', 'B', ...
        p.INIT_O, p.INIT_H, p.INIT_F, p.INIT_M, p.INIT_Z, 0, 0, 0, 0};
    for i = 1:n
        e = log{i};
        data(i+1,:) = {e.day, e.pos(1), e.pos(2), e.weather, e.action, ...
            point_name(p, e.pos), e.O, e.H, e.F, e.M, e.Z, e.c, ...
            e.buy(1), e.buy(2), e.buy(3)};
    end
    T = cell2table(data(2:end,:), 'VariableNames', ...
        {'Day','PosX','PosY','Weather','Action','Point', ...
         'FuelO','WaterH','FoodF','MoneyM','TargetZ','c','BuyO','BuyH','BuyF'});
end

function nm = point_name(p, pos)
    nm = '';
    if isequal(pos, p.B),  nm = 'B';
    elseif isequal(pos, p.E),  nm = 'E';
    elseif isequal(pos, p.S1), nm = 'S1';
    elseif isequal(pos, p.S2), nm = 'S2';
    elseif isequal(pos, p.W1), nm = 'W1';
    elseif isequal(pos, p.W2), nm = 'W2';
    elseif isequal(pos, p.W3), nm = 'W3';
    end
end
