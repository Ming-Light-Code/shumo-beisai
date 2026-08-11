function milp_q3_stochastic(n_scenarios)
% ================================================================
% Q3 Approach 3: Stochastic Programming with Scenario Bundle
%
% Generates K weather scenarios, solves a large deterministic MILP
% where Day 1 decisions are shared across all scenarios
% (first-stage non-anticipativity).
%
% NOTE: This is a simplified DETERMINISTIC solver for a FIXED route.
% Full multi-scenario MILP across all skeletons would be too large.
% Instead, this evaluates a given route under multiple weather
% scenarios to find a robust resource allocation plan.
% ================================================================

if nargin < 1, n_scenarios = 20; end

run('..\共享工具\q3_params.m');

fprintf('========================================================\n');
fprintf('  Q3 Approach 3: Multi-Scenario Stochastic Program\n');
fprintf('  Scenarios: %d\n', n_scenarios);
fprintf('========================================================\n\n');

% ---- Generate weather scenarios ----
rng(42);
scenarios = rand(MAX_DAYS, n_scenarios) < P_STORM;  % 1=storm, 0=normal
fprintf('Generated %d weather scenarios.\n', n_scenarios);
fprintf('Average storm ratio: %.2f\n\n', mean(scenarios(:)));

% ---- Fixed candidate route: B->S1->W1->W2->S2->W3->E ----
pid = [1, 6, 3, 4, 7, 5, 2];
m = length(pid) - 2;
travel = zeros(1, m+1);
total_travel = 0;
for k = 1:(m+1)
    travel(k) = dist(pid(k), pid(k+1));
    total_travel = total_travel + travel(k);
end

% Identify work/supply segments
work_idx = zeros(1, m+1); work_which = [];
supp_idx = zeros(1, m+1);
n_work = 0; n_supply = 0;
for k = 1:(m+1)
    pt = pid(k+1);
    if pt >= 3 && pt <= 5
        n_work = n_work + 1;
        work_idx(k) = n_work;
        work_which(n_work) = pt - 2;
    end
    if pt == 6 || pt == 7
        n_supply = n_supply + 1;
        supp_idx(k) = n_supply;
    end
end

remain = MAX_DAYS - total_travel;

fprintf('Route: ');
for i = 1:length(pid)
    fprintf('%s', names{pid(i)});
    if i < length(pid), fprintf(' -> '); end
end
fprintf('\n');
fprintf('Travel: %d/%d days, Remain: %d for work/stop\n', total_travel, MAX_DAYS, remain);
fprintf('Work points: %d, Supply points: %d\n\n', n_work, n_supply);

% ---- Evaluate robust resource allocation ----
% Strategy: for each scenario, compute resource needs, then
% pick purchase amounts that cover all scenarios

min_buyO = zeros(1, n_supply);
min_buyH = zeros(1, n_supply);
min_buyF = zeros(1, n_supply);

buyO_all = zeros(n_supply, n_scenarios);
buyH_all = zeros(n_supply, n_scenarios);
buyF_all = zeros(n_supply, n_scenarios);

for sc = 1:n_scenarios
    ws = scenarios(:, sc);
    [~, bo, bh, bf] = scenario_optimal(pid, m, travel, work_idx, ...
        work_which, supp_idx, n_work, n_supply, total_travel, ws, ...
        MAX_DAYS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, LOAD_LIMIT, ...
        WY, WM, MAX_WY, CM_N, CS_N, CW_N, CM_T, CS_T, CW_T);
    if ~isempty(bo)
        buyO_all(:, sc) = bo;
        buyH_all(:, sc) = bh;
        buyF_all(:, sc) = bf;
    end
end

% Robust purchases: use 90th percentile across scenarios
for k = 1:n_supply
    valid_o = buyO_all(k, buyO_all(k,:) > 0);
    valid_h = buyH_all(k, buyH_all(k,:) > 0);
    valid_f = buyF_all(k, buyF_all(k,:) > 0);
    if ~isempty(valid_o)
        min_buyO(k) = prctile(valid_o, 90);
    end
    if ~isempty(valid_h)
        min_buyH(k) = prctile(valid_h, 90);
    end
    if ~isempty(valid_f)
        min_buyF(k) = prctile(valid_f, 90);
    end
end

fprintf('--- Robust Purchase Plan (90th percentile) ---\n');
sidx = 0;
for k = 1:(m+1)
    pt = pid(k+1);
    if pt == 6 || pt == 7
        sidx = sidx + 1;
        fprintf('  %s: O=%d H=%d F=%d\n', ...
            names{pt}, ceil(min_buyO(sidx)), ceil(min_buyH(sidx)), ceil(min_buyF(sidx)));
    end
end

% ---- Validate robust plan across all scenarios ----
fprintf('\n--- Validation ---\n');
success_count = 0;
Z_values = []; M_values = [];
w1_r = zeros(1, n_work); w2_r = zeros(1, n_work); b_r = zeros(1, n_work);
for j = 1:n_work
    w1_r(j) = min(WM(work_which(j)), ceil(remain / n_work));
    if w1_r(j) == WM(work_which(j)) && remain > w1_r(j) + 1
        b_r(j) = 1;
        w2_r(j) = min(WM(work_which(j)), remain - w1_r(j) - 1);
    end
end

buy_r = [ceil(min_buyO); ceil(min_buyH); ceil(min_buyF)]';

for sc = 1:n_scenarios
    ws = scenarios(:, sc);
    [ok, Z, M] = simulate_plan(pid, m, travel, work_idx, work_which, ...
        supp_idx, n_work, n_supply, total_travel, ws, ...
        w1_r, b_r, w2_r, buy_r, MAX_DAYS, INIT_O, INIT_H, INIT_F, ...
        INIT_M, INIT_Z, LOAD_LIMIT, WY, WM, ...
        CM_N, CS_N, CW_N, CM_T, CS_T, CW_T, PRICE_O, PRICE_H, PRICE_F);
    if ok
        success_count = success_count + 1;
        Z_values(end+1) = Z;
        M_values(end+1) = M;
    end
end

fprintf('Success rate: %.1f%% (%d/%d)\n', ...
    100*success_count/n_scenarios, success_count, n_scenarios);
if ~isempty(Z_values)
    fprintf('Z: mean=%.0f, std=%.0f, range=[%d,%d]\n', ...
        mean(Z_values), std(Z_values), min(Z_values), max(Z_values));
    fprintf('M: mean=%.0f, std=%.0f, range=[%d,%d]\n', ...
        mean(M_values), std(M_values), min(M_values), max(M_values));
end

end

% =================================================================
function [feasible, buyO, buyH, buyF] = scenario_optimal(pid, m, travel, ...
    work_idx, work_which, supp_idx, n_work, n_supply, total_travel, ...
    weather_seq, MAX_DAYS, INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z, ...
    LOAD_LIMIT, WY, WM, MAX_WY, CM_N, CS_N, CW_N, CM_T, CS_T, CW_T)
% Find feasible purchase amounts for a given weather scenario

    % Simple forward simulation:
    % 1. Allocate work days greedily
    remain = MAX_DAYS - total_travel;
    w1 = zeros(1, n_work); w2 = zeros(1, n_work); b = zeros(1, n_work);
    [~, order] = sort(WY(work_which), 'descend');
    for oi = 1:n_work
        j = order(oi);
        w1(j) = min(WM(work_which(j)), remain);
        remain = remain - w1(j);
        if remain > 0 && w1(j) == WM(work_which(j))
            b(j) = 1; remain = remain - 1;
            w2(j) = min(WM(work_which(j)), remain); remain = remain - w2(j);
        end
    end

    % 2. Simulate forward to find minimum purchases needed
    T = total_travel + sum(w1) + sum(b) + sum(w2);
    buyO = zeros(1, n_supply); buyH = zeros(1, n_supply); buyF = zeros(1, n_supply);

    % Build consumption schedule
    cO = zeros(1, T); cH = zeros(1, T); cF = zeros(1, T);
    zG = zeros(1, T); isSup = false(1, T);
    sup_day_to_k = zeros(1, T);

    day = 0; sidx = 0;
    for k = 1:(m+1)
        d = travel(k);
        for dd = 1:d
            day = day + 1;
            storm = (weather_seq(day) == 1);
            if storm
                cO(day) = CM_T(1); cH(day) = CM_T(2); cF(day) = CM_T(3);
            else
                cO(day) = CM_N(1); cH(day) = CM_N(2); cF(day) = CM_N(3);
            end
            if dd == d && supp_idx(k) > 0
                isSup(day) = true; sidx = sidx + 1;
                sup_day_to_k(day) = sidx;
            end
        end
        widx = work_idx(k);
        if widx > 0
            wh = work_which(widx);
            for ww = 1:w1(widx)
                day = day + 1;
                storm = (weather_seq(day) == 1);
                if storm
                    cO(day) = CW_T(1); cH(day) = CW_T(2); cF(day) = CW_T(3);
                else
                    cO(day) = CW_N(1); cH(day) = CW_N(2); cF(day) = CW_N(3);
                end
                zG(day) = WY(wh);
            end
            if b(widx) > 0
                day = day + 1;
                storm = (weather_seq(day) == 1);
                if storm
                    cO(day) = CS_T(1); cH(day) = CS_T(2); cF(day) = CS_T(3);
                else
                    cO(day) = CS_N(1); cH(day) = CS_N(2); cF(day) = CS_N(3);
                end
            end
            for ww = 1:w2(widx)
                day = day + 1;
                storm = (weather_seq(day) == 1);
                if storm
                    cO(day) = CW_T(1); cH(day) = CW_T(2); cF(day) = CW_T(3);
                else
                    cO(day) = CW_N(1); cH(day) = CW_N(2); cF(day) = CW_N(3);
                end
                zG(day) = WY(wh);
            end
        end
    end

    % Determine required purchases
    O = INIT_O; H = INIT_H; F = INIT_F; M = INIT_M;
    for t = 1:T
        O = O - cO(t); H = H - cH(t); F = F - cF(t);
        if O < 0, O = 0; end
        if H < 0, H = 0; end
        if F < 0, F = 0; end
        if isSup(t)
            needO = 0; needH = 0; needF = 0;
            for tt = t+1:min(T, t+30)
                needO = needO + cO(tt);
                needH = needH + cH(tt);
                needF = needF + cF(tt);
            end
            buyO_k = max(0, ceil(needO - O));
            buyH_k = max(0, ceil(needH - H));
            buyF_k = max(0, ceil(needF - F));
            sk = sup_day_to_k(t);
            if sk > 0 && sk <= n_supply
                buyO(sk) = buyO_k;
                buyH(sk) = buyH_k;
                buyF(sk) = buyF_k;
            end
            cost = buyO_k*2 + buyH_k*1 + buyF_k*2;
            if cost <= M
                O = O + buyO_k; H = H + buyH_k; F = F + buyF_k; M = M - cost;
            else
                feasible = false; return;
            end
        end
        if O + H + F > LOAD_LIMIT
            feasible = false; return;
        end
    end
    feasible = true;
end

% =================================================================
function [success, Z_final, M_final] = simulate_plan(pid, m, travel, ...
    work_idx, work_which, supp_idx, n_work, n_supply, total_travel, ...
    weather_seq, w1, b, w2, buy, MAX_DAYS, INIT_O, INIT_H, INIT_F, ...
    INIT_M, INIT_Z, LOAD_LIMIT, WY, WM, ...
    CM_N, CS_N, CW_N, CM_T, CS_T, CW_T, PRICE_O, PRICE_H, PRICE_F)
% Simulate a given plan (work and buy amounts) under a weather scenario

    T = total_travel + sum(w1) + sum(b) + sum(w2);
    O = INIT_O; H = INIT_H; F = INIT_F; M = INIT_M; Z = INIT_Z;
    day = 0; sidx = 0;

    for k = 1:(m+1)
        d = travel(k);
        for dd = 1:d
            day = day + 1;
            if day > length(weather_seq), break; end
            storm = (weather_seq(day) == 1);
            if storm
                O = O - CM_T(1); H = H - CM_T(2); F = F - CM_T(3);
            else
                O = O - CM_N(1); H = H - CM_N(2); F = F - CM_N(3);
            end
            if O < 0 || H < 0 || F < 0
                success = false; Z_final = Z; M_final = M; return;
            end
            if dd == d && supp_idx(k) > 0
                sidx = sidx + 1;
                if sidx <= size(buy, 1)
                    bO = buy(sidx, 1); bH = buy(sidx, 2); bF = buy(sidx, 3);
                    cost = bO*PRICE_O + bH*PRICE_H + bF*PRICE_F;
                    if M >= cost && O+bO+H+bH+F+bF <= LOAD_LIMIT
                        O = O + bO; H = H + bH; F = F + bF; M = M - cost;
                    else
                        success = false; Z_final = Z; M_final = M; return;
                    end
                end
            end
        end

        widx = work_idx(k);
        if widx > 0
            wh = work_which(widx);
            for ww = 1:w1(widx)
                day = day + 1;
                if day > length(weather_seq), break; end
                storm = (weather_seq(day) == 1);
                if storm
                    O = O - CW_T(1); H = H - CW_T(2); F = F - CW_T(3);
                else
                    O = O - CW_N(1); H = H - CW_N(2); F = F - CW_N(3);
                end
                Z = Z + WY(wh);
                if O < 0 || H < 0 || F < 0
                    success = false; Z_final = Z; M_final = M; return;
                end
            end
            if b(widx) > 0
                day = day + 1;
                if day > length(weather_seq), break; end
                storm = (weather_seq(day) == 1);
                if storm
                    O = O - CS_T(1); H = H - CS_T(2); F = F - CS_T(3);
                else
                    O = O - CS_N(1); H = H - CS_N(2); F = F - CS_N(3);
                end
            end
            for ww = 1:w2(widx)
                day = day + 1;
                if day > length(weather_seq), break; end
                storm = (weather_seq(day) == 1);
                if storm
                    O = O - CW_T(1); H = H - CW_T(2); F = F - CW_T(3);
                else
                    O = O - CW_N(1); H = H - CW_N(2); F = F - CW_N(3);
                end
                Z = Z + WY(wh);
                if O < 0 || H < 0 || F < 0
                    success = false; Z_final = Z; M_final = M; return;
                end
            end
        end
    end

    success = true; Z_final = Z; M_final = M;
end
