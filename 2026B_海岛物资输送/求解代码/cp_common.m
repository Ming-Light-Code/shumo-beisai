function varargout = cp_common(action, varargin)
% cp_common.m - Shared utility functions for CP-based island supply solvers
% Usage:
%   cp_common('max_work_with_park', mc, remaining) -> max_w
%   cp_common('enumerate_park_combs', n_seg, max_total) -> combs matrix
%   cp_common('simulate', pid, m, dist, wa, ww, tt, wdays, park_seg, cons, MAX_LOAD, init) -> [feasible, Z, M, sched]
%   cp_common('print_schedule', bP, bWD, bPS, dist, WM, WY, cons, MAX_DAYS, MAX_LOAD, all_xy, names, init) -> void
%
% init is a struct with fields: O, H, F, M, Z

switch action
    case 'max_work_with_park'
        varargout{1} = max_work_with_park_(varargin{:});
    case 'enumerate_park_combs'
        varargout{1} = enumerate_park_combs_(varargin{:});
    case 'simulate'
        [varargout{1}, varargout{2}, varargout{3}, varargout{4}] = simulate_(varargin{:});
    case 'print_schedule'
        print_schedule_(varargin{:});
    otherwise
        error('cp_common: unknown action "%s"', action);
end
end

% ===== max_work_with_park =====
function max_w = max_work_with_park_(mc, remaining)
% Compute maximum work days achievable in `remaining` days with max
% consecutive work `mc` and required park-reset days between sessions.
    best = 0;
    for k = 1:(remaining + 1)
        stay = k * mc + (k - 1);  % k sessions of mc work + (k-1) park days
        if stay > remaining, break; end
        best = k * mc;
        slack = remaining - stay;
        if slack >= 1
            best = max(best, k * mc + min(mc, slack - 1));
        end
    end
    max_w = max(best, min(mc, remaining));
end

% ===== enumerate_park_combs =====
function combs = enumerate_park_combs_(n_seg, max_total)
% Enumerate all ways to distribute <= max_total park days across n_seg segments.
% Each segment gets >= 0 park days, sum over all segments = any value 0..max_total.
% Returns: combs matrix of size (num_combs x n_seg)
    combs = zeros(0, n_seg);
    current = zeros(1, n_seg);
    rec_enumerate(1, max_total);

    function rec_enumerate(pos, remain)
        if pos == n_seg
            current(pos) = remain;
            combs(end+1, :) = current;  %#ok<AGROW>
            return;
        end
        for p = 0:remain
            current(pos) = p;
            rec_enumerate(pos + 1, remain - p);
        end
    end
end

% ===== simulate =====
function [feasible, Zf, Mf, sched] = simulate_(pid, m, dist_all, wa, ww, tt, wdays, park_seg, ...
    cons, MAX_LOAD, init)
% Simulate resource consumption along a path with work/park/supply decisions.
%
% Inputs:
%   pid     - path point indices (e.g. [1, 3, 7, 2] for B->W1->S2->E)
%   m       - number of intermediate segments (length(pid)-2)
%   dist_all- precomputed distance matrix
%   wa      - indices of work points in pid (positions in pid array)
%   ww      - work point type indices (1=W1, 2=W2, 3=W3)
%   tt      - total travel days (sum of distances)
%   wdays   - work days per work point
%   park_seg- park-at-sea days per segment (between movement legs)
%   cons    - consumption struct: MO, MH, MF, PO, PH, PF, WO, WH, WF, pO, pH, pF
%   MAX_LOAD- load capacity
%   init    - struct with O, H, F, M, Z (initial values)
%
% Outputs:
%   feasible- true if path is feasible
%   Zf, Mf  - final Z and M values
%   sched   - day-by-day schedule struct with fields: day, posX, posY, action, O, H, F, Z, M, detail

    if isempty(wdays), wdays = []; end
    if isempty(park_seg), park_seg = zeros(1, m+1); end

    % Allocate consumption arrays
    T_alloc = tt + sum(wdays) + sum(park_seg) + 100;
    cO = zeros(1, T_alloc); cH = zeros(1, T_alloc); cF = zeros(1, T_alloc);
    zG = zeros(1, T_alloc); isSup = false(1, T_alloc);

    % Build consumption schedule
    day = 0;
    for k = 1:(m+1)
        d = dist_all(pid(k), pid(k+1));

        % 1. Park at sea before moving (Task 2 strategy: reduce load before supply)
        for pd = 1:park_seg(k)
            day = day + 1;
            cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
        end

        % 2. Move to next point
        for dd = 1:d
            day = day + 1;
            cO(day) = cons.MO; cH(day) = cons.MH; cF(day) = cons.MF;
            if dd == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7  % S1 or S2
                    isSup(day) = true;
                end
            end
        end

        % 3. Work at destination (with park-reset between sessions)
        if ~isempty(wa)
            wk = find(wa == k+1, 1);
            if ~isempty(wk) && ~isempty(wdays) && wk <= length(wdays) && wdays(wk) > 0
                % Map ww index to WM and WY (support configurable arrays)
                WM_local = [4, 5, 3];  % default; overridden per-call if needed
                WY_local = [20, 15, 28];
                wi = ww(wk);
                mc = WM_local(wi);
                yld = WY_local(wi);
                rem_val = wdays(wk);
                while rem_val > 0
                    chunk = min(rem_val, mc);
                    for w = 1:chunk
                        day = day + 1;
                        cO(day) = cons.WO; cH(day) = cons.WH; cF(day) = cons.WF;
                        zG(day) = yld;
                    end
                    rem_val = rem_val - chunk;
                    if rem_val > 0
                        day = day + 1;
                        cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
                        zG(day) = 0;
                    end
                end
            end
        end
    end

    T_actual = day;

    % Initialize state
    O = init.O; H = init.H; F = init.F;
    M = init.M; Zf = init.Z;

    % Pre-allocate schedule recording
    sched_day   = zeros(1, T_actual);
    sched_posX  = zeros(1, T_actual);
    sched_posY  = zeros(1, T_actual);
    sched_O     = zeros(1, T_actual);
    sched_H     = zeros(1, T_actual);
    sched_F     = zeros(1, T_actual);
    sched_Z     = zeros(1, T_actual);
    sched_M     = zeros(1, T_actual);
    sched_act   = cell(1, T_actual);
    sched_det   = cell(1, T_actual);

    % Day-by-day simulation
    for t = 1:T_actual
        % Deduct consumption FIRST (critical: even at supply points)
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        Zf = Zf + zG(t);

        if O < 0 || H < 0 || F < 0
            feasible = false; Mf = 0;
            sched = make_sched_struct(sched_day, sched_posX, sched_posY, sched_act, sched_det, ...
                                      sched_O, sched_H, sched_F, sched_Z, sched_M, t-1);
            return;
        end

        if O + H + F > MAX_LOAD + 1e-9
            feasible = false; Mf = 0;
            sched = make_sched_struct(sched_day, sched_posX, sched_posY, sched_act, sched_det, ...
                                      sched_O, sched_H, sched_F, sched_Z, sched_M, t-1);
            return;
        end

        % Record state BEFORE possible supply (post-consumption state)
        sched_day(t)  = t;
        sched_O(t)    = O;
        sched_H(t)    = H;
        sched_F(t)    = F;
        sched_Z(t)    = Zf;
        sched_M(t)    = M;

        if isSup(t)
            % Find next supply day
            ns = T_actual + 1;
            for tt2 = (t+1):T_actual
                if isSup(tt2), ns = tt2; break; end
            end

            % Compute total resources needed until next supply
            nO = 0; nH = 0; nF = 0;
            for tt2 = (t+1):ns
                if tt2 > T_actual, break; end
                nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
            end

            sp = MAX_LOAD - (O + H + F);
            bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);

            if bO + bH + bF > sp
                feasible = false; Mf = 0;
                sched = make_sched_struct(sched_day, sched_posX, sched_posY, sched_act, sched_det, ...
                                          sched_O, sched_H, sched_F, sched_Z, sched_M, t);
                return;
            end

            if ns > T_actual && (O + bO < nO || H + bH < nH || F + bF < nF)
                feasible = false; Mf = 0;
                sched = make_sched_struct(sched_day, sched_posX, sched_posY, sched_act, sched_det, ...
                                          sched_O, sched_H, sched_F, sched_Z, sched_M, t);
                return;
            end

            cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
            if cost > M
                feasible = false; Mf = 0;
                sched = make_sched_struct(sched_day, sched_posX, sched_posY, sched_act, sched_det, ...
                                          sched_O, sched_H, sched_F, sched_Z, sched_M, t);
                return;
            end

            O = O + bO; H = H + bH; F = F + bF;
            M = M - cost;
            sched_act{t} = 'SUPPLY';
            sched_det{t} = sprintf('+O%d H%d F%d cost=%d', round(bO), round(bH), round(bF), round(cost));
        else
            sched_act{t} = '';
            sched_det{t} = '';
        end
    end

    feasible = true;
    Mf = M;
    sched = make_sched_struct(sched_day, sched_posX, sched_posY, sched_act, sched_det, ...
                              sched_O, sched_H, sched_F, sched_Z, sched_M, T_actual);
end

function s = make_sched_struct(day, px, py, act, det, O, H, F, Z, M, n)
    s = struct();
    if n > 0
        s.day   = day(1:n);
        s.posX  = px(1:n);
        s.posY  = py(1:n);
        s.action = {act{1:n}};
        s.detail = {det{1:n}};
        s.O = O(1:n);
        s.H = H(1:n);
        s.F = F(1:n);
        s.Z = Z(1:n);
        s.M = M(1:n);
    end
end

% ===== print_schedule =====
function print_schedule_(bP, bWD, bPS, dist, WM, WY, cons, MAX_DAYS, MAX_LOAD, all_xy, names, init)
% Print day-by-day schedule with resource tracking.
% bPS can be empty (Task 1: no park-at-sea) or an array of park days per segment.

    m = length(bP) - 2;
    if m < 0
        fprintf('  No feasible path found.\n');
        return;
    end
    if isempty(bPS), bPS = zeros(1, m+1); end

    % Rebuild wa/ww from path
    wa = []; ww = [];
    for i = 2:length(bP)
        if bP(i) >= 3 && bP(i) <= 5
            wa(end+1) = i;           %#ok<AGROW>
            ww(end+1) = bP(i) - 2;   %#ok<AGROW>
        end
    end

    % Compute travel days
    tt = 0;
    for k = 1:(m+1), tt = tt + dist(bP(k), bP(k+1)); end

    % Build consumption arrays (replay)
    T_max = tt + sum(bWD) + sum(bPS) + 100;
    cO = zeros(1, T_max); cH = zeros(1, T_max); cF = zeros(1, T_max);
    zG = zeros(1, T_max); isSup = false(1, T_max);

    day = 0;
    for k = 1:(m+1)
        d = dist(bP(k), bP(k+1));
        % Park at sea
        for pd = 1:bPS(k)
            day = day + 1;
            cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
        end
        % Move
        for dd = 1:d
            day = day + 1;
            cO(day) = cons.MO; cH(day) = cons.MH; cF(day) = cons.MF;
            if dd == d
                to_pt = bP(k+1);
                if to_pt == 6 || to_pt == 7, isSup(day) = true; end
            end
        end
        % Work
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && bWD(wk) > 0
            mc = WM(ww(wk)); yld = WY(ww(wk)); rem_val = bWD(wk);
            while rem_val > 0
                chunk = min(rem_val, mc);
                for w = 1:chunk
                    day = day + 1;
                    cO(day) = cons.WO; cH(day) = cons.WH; cF(day) = cons.WF;
                    zG(day) = yld;
                end
                rem_val = rem_val - chunk;
                if rem_val > 0
                    day = day + 1;
                    cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
                end
            end
        end
    end

    T_actual = day;

    % Header
    fprintf('\n===== DAY-BY-DAY SCHEDULE =====\n');
    fprintf('Day  | Pos (x,y)  | Action         |  O   H   F  Load |   Z     M\n');
    fprintf('-----|-------------|----------------|------------------|------------\n');

    O = init.O; H = init.H; F = init.F;
    M = init.M; Z = init.Z;
    day2 = 0;

    for k = 1:(m+1)
        fr = bP(k); to = bP(k+1); d = dist(fr, to);
        fr_xy = all_xy(fr, :); to_xy = all_xy(to, :);

        % 1. Park at sea
        for pd = 1:bPS(k)
            day2 = day2 + 1;
            O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
            fprintf('%4d | (%2d,%2d)     | park(at sea)   | %3d %3d %3d %4d | %4d %5d\n', ...
                day2, fr_xy(1), fr_xy(2), round(O), round(H), round(F), round(O+H+F), Z, round(M));
        end

        % 2. Move
        dx_total = to_xy(1) - fr_xy(1);
        dy_total = to_xy(2) - fr_xy(2);
        steps_x = abs(dx_total);
        steps_y = abs(dy_total);
        for dd = 1:d
            day2 = day2 + 1;
            % Manhattan movement: always orthogonal (x-first, then y)
            if dd <= steps_x
                x = fr_xy(1) + sign(dx_total) * dd;
                y = fr_xy(2);
            else
                x = to_xy(1);
                y = fr_xy(2) + sign(dy_total) * (dd - steps_x);
            end
            O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);

            if isSup(day2)
                ns = T_actual + 1;
                for tt2 = (day2+1):T_actual
                    if isSup(tt2), ns = tt2; break; end
                end
                nO = 0; nH = 0; nF = 0;
                for tt2 = (day2+1):ns
                    if tt2 > T_actual, break; end
                    nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
                end
                bO = max(0, nO - O); bH = max(0, nH - H); bF = max(0, nF - F);
                cost = bO * cons.pO + bH * cons.pH + bF * cons.pF;
                M = M - cost;
                O = O + bO; H = H + bH; F = F + bF;
                fprintf('%4d | (%2d,%2d)     | SUPPLY(%s)     | %3d %3d %3d %4d | %4d %5d  (+O%d H%d F%d)\n', ...
                    day2, x, y, names{to}, round(O), round(H), round(F), round(O+H+F), Z, round(M), round(bO), round(bH), round(bF));
            else
                fprintf('%4d | (%2d,%2d)     | move           | %3d %3d %3d %4d | %4d %5d\n', ...
                    day2, x, y, round(O), round(H), round(F), round(O+H+F), Z, round(M));
            end
        end

        % 3. Work
        wk = find(wa == k+1, 1);
        if ~isempty(wk) && bWD(wk) > 0
            mc = WM(ww(wk)); yld = WY(ww(wk)); rem_val = bWD(wk);
            pt_name = names{to};
            while rem_val > 0
                chunk = min(rem_val, mc);
                for w = 1:chunk
                    day2 = day2 + 1;
                    O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
                    Z = Z + zG(day2);
                    fprintf('%4d | (%2d,%2d)     | work(%s)       | %3d %3d %3d %4d | %4d %5d\n', ...
                        day2, to_xy(1), to_xy(2), pt_name, round(O), round(H), round(F), round(O+H+F), Z, round(M));
                end
                rem_val = rem_val - chunk;
                if rem_val > 0
                    day2 = day2 + 1;
                    O = O - cO(day2); H = H - cH(day2); F = F - cF(day2);
                    fprintf('%4d | (%2d,%2d)     | park(reset)    | %3d %3d %3d %4d | %4d %5d\n', ...
                        day2, to_xy(1), to_xy(2), round(O), round(H), round(F), round(O+H+F), Z, round(M));
                end
            end
        end
    end

    fprintf('-----|-------------|----------------|------------------|------------\n');
    fprintf('  Final at E: Z=%d M=%d Day=%d\n', Z, round(M), day2);
end
