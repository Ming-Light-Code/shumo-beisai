%% ========================================================================
%% solve_task2_online.m - Task 2: Online CP Decision Model
%% ========================================================================
%% 功能: 逐日观测天气 → CP搜索剩余最优路径 → 执行今日动作
%% 场景: 全雷暴极端情形 (w_t = 雷暴 ∀t)，退化为确定性CP
%% 框架: State S_t = (pos,O,H,F,M,Z,consec,w_t)
%%       Observe w_t -> CP search -> execute a_t -> S_{t+1}
%% 输出: 逐日决策表与最终结果
%% ========================================================================

function solve_task2_online()

%% ====== Parameters ======
MAX_DAYS = 30;
MAX_LOAD = 120;
INIT_O   = 35; INIT_H = 45; INIT_F = 30;
INIT_M   = 240; INIT_Z = 100;

all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
n_pts  = size(all_xy, 1);
names  = {'B', 'E', 'W1', 'W2', 'W3', 'S1', 'S2'};
WY     = [20, 15, 28];
WM     = [4, 5, 3];

% Manhattan distance matrix
distMap = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        distMap(i, j) = abs(all_xy(i,1) - all_xy(j,1)) + abs(all_xy(i,2) - all_xy(j,2));
    end
end

% Cost parameter table
cons = struct( ...
    'normal',    struct('MO',2,'MH',3,'MF',2,'PO',1,'PH',1,'PF',1,'WO',5,'WH',4,'WF',3,'pO',2,'pH',1,'pF',2), ...
    'thunder',   struct('MO',8,'MH',4,'MF',3,'PO',3,'PH',3,'PF',2,'WO',8,'WH',6,'WF',6,'pO',2,'pH',1,'pF',2));

% All-thunderstorm sequence
weatherSeq = ones(1, MAX_DAYS) * 2;

%% ====== Initialization ======
fprintf('========================================\n');
fprintf('  Task 2: Online CP Decision Model\n');
fprintf('========================================\n');
fprintf('Weather: 30 consecutive thunderstorm days\n');
fprintf('Model: State S_t = (pos, O,H,F, M,Z, consec, w_t)\n');
fprintf('       Observe w_t -> CP search -> execute a_t -> S_{t+1}\n\n');

curPt   = 1;
pos     = all_xy(1, :);
O = INIT_O; H = INIT_H; F = INIT_F;
M = INIT_M; Z = INIT_Z;
consec  = 0;
day     = 0;
arrived = false;

% CP planning state
planPath    = [];
planParks   = [];
planWorks   = [];
legIdx      = 1;
dayInLeg    = 0;
parkedInLeg = 0;

fprintf('--- Day-by-Day Decision ---\n');
fprintf('Day | Weather  | Action         | Pos (x,y)  |  O   H   F  Load |   Z     M\n');
fprintf('----|----------|----------------|------------|------------------|------------\n');

%% ====== Daily Decision Loop ======
while day < MAX_DAYS && ~arrived
    day = day + 1;
    w     = weatherSeq(day);
    wName = 'thunder';
    c     = cons.thunder;

    detailStr = '';

    % Replan if at a named POI
    if isempty(planPath) || (curPt ~= 1 && dayInLeg <= 0 && parkedInLeg <= 0)
        [planPath, planParks, planWorks, feasible] = ...
            cp_plan_from_state(curPt, day - 1, O, H, F, M, Z, consec, ...
                               c, MAX_DAYS, MAX_LOAD, distMap, all_xy, WY, WM, INIT_Z);
        legIdx      = 1;
        dayInLeg    = 0;
        parkedInLeg = 0;

        if ~feasible
            fprintf('%3d | %s    | NO FEASIBLE    | (%2d,%2d)     | %3d %3d %3d %4d | %4d %5d\n', ...
                day, wName, pos(1), pos(2), O, H, F, O+H+F, Z, M);
            break;
        end
    end

    if length(planPath) < 2
        break;
    end

    nextPt = planPath(legIdx + 1);

    % Decision: today's action
    if legIdx <= length(planParks) && parkedInLeg < planParks(legIdx)
        action = 'park(at sea)';
        O = O - c.PO; H = H - c.PH; F = F - c.PF;
        consec = 0;
        parkedInLeg = parkedInLeg + 1;

    elseif dayInLeg < distMap(curPt, nextPt) - 1
        action = 'move';
        O = O - c.MO; H = H - c.MH; F = F - c.MF;
        consec = 0;
        dayInLeg = dayInLeg + 1;
        pos = move_toward(pos, all_xy(nextPt, :));

    elseif dayInLeg == distMap(curPt, nextPt) - 1
        action = 'move';
        O = O - c.MO; H = H - c.MH; F = F - c.MF;
        consec = 0;
        dayInLeg = dayInLeg + 1;
        pos = all_xy(nextPt, :);
        curPt = nextPt;

        % Supply at supply point
        if curPt == 6 || curPt == 7
            remTravel = 0;
            remPark   = 0;
            for k = (legIdx + 1):(length(planPath) - 1)
                remTravel = remTravel + distMap(planPath(k), planPath(k+1));
                if k + 1 <= length(planParks)
                    remPark = remPark + planParks(k + 1);
                end
            end
            needO = remTravel * c.MO + remPark * c.PO;
            needH = remTravel * c.MH + remPark * c.PH;
            needF = remTravel * c.MF + remPark * c.PF;
            sp    = MAX_LOAD - (O + H + F);
            buyO  = max(0, needO - O);
            buyH  = max(0, needH - H);
            buyF  = max(0, needF - F);

            if buyO + buyH + buyF <= sp
                cost = buyO * c.pO + buyH * c.pH + buyF * c.pF;
                if cost <= M
                    O = O + buyO; H = H + buyH; F = F + buyF;
                    M = M - cost;
                    detailStr = sprintf('(+O=%d H=%d F=%d cost=%d)', buyO, buyH, buyF, cost);
                    action = sprintf('SUPPLY(%s)', names{curPt});
                end
            end
        end

        if curPt == 2
            action = 'ARRIVE at E';
            arrived = true;
        end

        legIdx = legIdx + 1;
        dayInLeg    = 0;
        parkedInLeg = 0;

    else
        % At destination: work or park
        wkIdx = find(curPt == [3, 4, 5], 1);
        if ~isempty(wkIdx) && ~isempty(planWorks) && sum(planWorks) > 0
            action = sprintf('work(%s)', names{curPt});
            O = O - c.WO; H = H - c.WH; F = F - c.WF;
            Z = Z + WY(wkIdx);
            consec = consec + 1;
        else
            action = 'park';
            O = O - c.PO; H = H - c.PH; F = F - c.PF;
            consec = 0;
        end
    end

    fprintf('%3d | %s    | %-14s | (%2d,%2d)     | %3d %3d %3d %4d | %4d %5d', ...
        day, wName, action, pos(1), pos(2), O, H, F, O+H+F, Z, M);
    if ~isempty(detailStr)
        fprintf('  %s', detailStr);
    end
    fprintf('\n');
end

fprintf('----|----------|----------------|------------|------------------|------------\n');

%% ====== Final Result ======
fprintf('\n================ FINAL RESULT (Extreme: 30-day Thunderstorm) ================\n');
if arrived
    fprintf('Status: Arrived at E on day %d\n', day);
else
    fprintf('Status: Did NOT reach E within %d days\n', MAX_DAYS);
end
fprintf('Z = %d  (target materials)\n', Z);
fprintf('M = %d  (remaining funds)\n', M);
fprintf('Load at E: O=%d H=%d F=%d Total=%d\n', O, H, F, O+H+F);
fprintf('\nDone.\n');

end

%% ========================================================================
%% CP Planning: Search optimal skeleton from current state
%% ========================================================================
function [bestPath, bestParks, bestWorks, feasible] = ...
    cp_plan_from_state(curPt, elapsed, O, H, F, M, Z, consec, ...
                        cons, MAX_DAYS, MAX_LOAD, dist, all_xy, WY, WM, INIT_Z)
    inter_idx = [3 4 5 6 7];

    bestZ    = -inf;
    bestMVal = -inf;
    bestPath  = [curPt, 2];
    bestParks = [];
    bestWorks = [];
    nodes = 0;

    [bestZ, bestMVal, bestPath, bestWorks, bestParks, nodes] = ...
        cp_search_online2([curPt], elapsed, [], [], bestZ, bestMVal, ...
                          bestPath, bestWorks, bestParks, ...
                          dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, ...
                          O, H, F, M, Z, cons, nodes, INIT_Z);

    feasible = (bestZ > -inf);
    if ~feasible
        bestPath  = [curPt, 2];
        bestParks = [];
        bestWorks = [];
    end
end

%% ========================================================================
%% CP Recursive Search (Online Version)
%% ========================================================================
function [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online2( ...
    path, tsf, wa, ww, bZ, bM, bP, bWD, bPS, ...
    dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, ...
    O_init, H_init, F_init, M_init, Z_init, cons, nodes, INIT_Z)

    nodes = nodes + 1;
    lp = path(end);

    % Upper-bound pruning
    if lp ~= 2
        dE  = dist(lp, 2);
        rem = MAX_DAYS - tsf;
        if rem < dE
            return;
        end
        max_w = max_work_with_park_online(3, rem - dE);
        ub    = Z_init + real(max_w) * 28;
        if ub <= bZ && bZ > -inf
            return;
        end
    end

    % Leaf evaluation
    dE = dist(lp, 2);
    if tsf + dE <= MAX_DAYS
        fp = [path, 2];
        m  = length(fp) - 2;
        tt = 0;
        for k = 1:(m + 1)
            tt = tt + dist(fp(k), fp(k+1));
        end

        rem_days = MAX_DAYS - tt;
        nw = length(wa);

        if nw == 0
            [bZ, bM, bP, bWD, bPS] = eval_leaf_online2( ...
                fp, m, tt, rem_days, dist, [], bZ, bM, bP, bWD, bPS, ...
                cons, MAX_LOAD, O_init, H_init, F_init, M_init, Z_init);
        else
            max_wk = zeros(1, nw);
            for j = 1:nw
                max_wk(j) = max_work_with_park_online(WM(ww(j)), rem_days);
            end
            sz = max_wk + 1;
            nc = prod(sz);

            for ci = 1:nc
                wd = zeros(1, nw);
                t2 = ci - 1;
                for j = nw:-1:1
                    wd(j) = mod(t2, sz(j));
                    t2 = floor(t2 / sz(j));
                end

                total_stay = 0;
                for j = 1:nw
                    if wd(j) > 0
                        total_stay = total_stay + wd(j) + max(0, ceil(wd(j) / WM(ww(j))) - 1);
                    end
                end
                if tt + total_stay > MAX_DAYS
                    continue;
                end

                park_rem = MAX_DAYS - tt - total_stay;
                [bZ, bM, bP, bWD, bPS] = eval_leaf_online2( ...
                    fp, m, tt, park_rem, dist, wd, bZ, bM, bP, bWD, bPS, ...
                    cons, MAX_LOAD, O_init, H_init, F_init, M_init, Z_init);
            end
        end
    end

    % Branch
    for ni = 1:5
        np = inter_idx(ni);
        if np == lp
            continue;
        end
        d = dist(lp, np);
        if tsf + d > MAX_DAYS
            continue;
        end
        dE2 = dist(np, 2);
        if tsf + d + dE2 > MAX_DAYS
            continue;
        end

        np2 = [path, np];
        nt  = tsf + d;
        nwa = wa;
        nww = ww;
        if np >= 3 && np <= 5
            nwa(end+1) = length(np2);
            nww(end+1) = np - 2;
        end

        [bZ, bM, bP, bWD, bPS, nodes] = cp_search_online2( ...
            np2, nt, nwa, nww, bZ, bM, bP, bWD, bPS, ...
            dist, inter_idx, WY, WM, MAX_DAYS, MAX_LOAD, ...
            O_init, H_init, F_init, M_init, Z_init, cons, nodes, INIT_Z);
    end
end

%% ========================================================================
%% Leaf Evaluation (Online)
%% ========================================================================
function [bZ, bM, bP, bWD, bPS] = eval_leaf_online2( ...
    fp, m, tt, park_rem, dist, wd_in, bZ, bM, bP, bWD, bPS, ...
    cons, MAX_LOAD, O_init, H_init, F_init, M_init, Z_init, WM, WY)

    n_seg      = m + 1;
    park_combs = enumerate_park_combs_online(n_seg + 1, park_rem);
    for ci = 1:size(park_combs, 1)
        ps = park_combs(ci, 1:n_seg);
        [ok, Z, M_out] = sim_online2(fp, m, tt, dist, wd_in, ps, ...
            cons, MAX_LOAD, O_init, H_init, F_init, M_init, Z_init);
        if ok && (Z > bZ || (Z == bZ && M_out > bM))
            bZ = Z; bM = M_out; bP = fp; bWD = wd_in; bPS = ps;
        end
    end
end

%% ========================================================================
%% Online Simulation
%% ========================================================================
function [feasible, Zf, Mf] = sim_online2(pid, m, tt, dist_all, wd, park_seg, WM, WY, ...
    cons, MAX_LOAD, O, H, F, M, Z)

    T   = tt + sum(wd) + sum(park_seg) + 100;
    cO  = zeros(1, T); cH = zeros(1, T); cF = zeros(1, T);
    zG  = zeros(1, T); isSup = false(1, T);
    day = 0;

    % Rebuild work-point index
    wa = []; ww = [];
    for i = 2:length(pid)
        if pid(i) >= 3 && pid(i) <= 5
            wa(end+1) = i;
            ww(end+1) = pid(i) - 2;
        end
    end

    for k = 1:(m + 1)
        % Park
        for pd = 1:park_seg(k)
            day = day + 1;
            cO(day) = cons.PO; cH(day) = cons.PH; cF(day) = cons.PF;
        end
        % Move
        d = dist_all(pid(k), pid(k+1));
        for dd = 1:d
            day = day + 1;
            cO(day) = cons.MO; cH(day) = cons.MH; cF(day) = cons.MF;
            if dd == d && (pid(k+1) == 6 || pid(k+1) == 7)
                isSup(day) = true;
            end
        end
        % Work
        wk = find(wa == k + 1, 1);
        if ~isempty(wk) && ~isempty(wd) && wd(wk) > 0
            rem_val = wd(wk);
            while rem_val > 0
                chunk = min(rem_val, wm_val);
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
    for t = 1:T_actual
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        Z = Z + zG(t);
        if O < 0 || H < 0 || F < 0
            feasible = false; Zf = Z; Mf = M; return;
        end
        if O + H + F > MAX_LOAD + 1e-9
            feasible = false; Zf = Z; Mf = M; return;
        end
        if isSup(t)
            ns = T_actual + 1;
            for tt2 = (t + 1):T_actual
                if isSup(tt2)
                    ns = tt2;
                    break;
                end
            end
            nO = 0; nH = 0; nF = 0;
            for tt2 = (t + 1):ns
                if tt2 > T_actual, break; end
                nO = nO + cO(tt2); nH = nH + cH(tt2); nF = nF + cF(tt2);
            end
            sp   = MAX_LOAD - (O + H + F);
            buyO = max(0, nO - O); buyH = max(0, nH - H); buyF = max(0, nF - F);
            if buyO + buyH + buyF > sp
                feasible = false; Zf = Z; Mf = M; return;
            end
            if ns > T_actual && (O + buyO < nO || H + buyH < nH || F + buyF < nF)
                feasible = false; Zf = Z; Mf = M; return;
            end
            cost = buyO * cons.pO + buyH * cons.pH + buyF * cons.pF;
            if cost > M
                feasible = false; Zf = Z; Mf = M; return;
            end
            O = O + buyO; H = H + buyH; F = F + buyF;
            M = M - cost;
        end
    end
    feasible = true; Zf = Z; Mf = M;
end

%% ========================================================================
%% Enumerate Parking Combinations
%% ========================================================================
function combs = enumerate_park_combs_online(n_seg, max_total)
    total   = nchoosek(max_total + n_seg, n_seg);
    combs   = zeros(total, n_seg);
    idx     = 0;
    current = zeros(1, n_seg);
    rec(1, max_total);

    function rec(pos, rem)
        if pos == n_seg
            current(pos) = rem;
            idx = idx + 1;
            combs(idx, :) = current;
            return;
        end
        for p = 0:rem
            current(pos) = p;
            rec(pos + 1, rem - p);
        end
    end
end

%% ========================================================================
%% Upper Bound Helper
%% ========================================================================
function max_w = max_work_with_park_online(mc, remaining)
    best = 0;
    for k = 1:(remaining + 1)
        stay = k * mc + (k - 1);
        if stay > remaining
            break;
        end
        best  = k * mc;
        slack = remaining - stay;
        if slack >= 1
            best = max(best, k * mc + min(mc, slack - 1));
        end
    end
    max_w = max(best, min(mc, remaining));
end

%% ========================================================================
%% Move One Step Toward Target
%% ========================================================================
function [newPos] = move_toward(fromPos, toPos)
    newPos = fromPos;
    if fromPos(1) < toPos(1)
        newPos(1) = fromPos(1) + 1;
    elseif fromPos(1) > toPos(1)
        newPos(1) = fromPos(1) - 1;
    elseif fromPos(2) < toPos(2)
        newPos(2) = fromPos(2) + 1;
    elseif fromPos(2) > toPos(2)
        newPos(2) = fromPos(2) - 1;
    end
end
