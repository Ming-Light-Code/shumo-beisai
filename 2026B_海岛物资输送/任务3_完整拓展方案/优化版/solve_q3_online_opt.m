function solve_q3_online_opt(weather_seq)
% =========================================================================
%  solve_q3_online_opt.m — 任务3 优化版在线随机决策模型
%  改进: P0差异化裕度 + P1预防性超购 + P2生存优先 + P4代码重构
% =========================================================================

cfg = cp_engine_opt('task3_config');
cons_exp = cp_engine_opt('get_cons', 'expected', cfg);
cons_N   = cp_engine_opt('get_cons', 'normal', cfg);
cons_T   = cp_engine_opt('get_cons', 'thunder', cfg);

if nargin < 1 || isempty(weather_seq)
    rng('shuffle');
    weather_seq = cp_engine_opt('gen_weather', 90, 0.8);
end

% 状态初始化
state.pt = 1; state.pos = cfg.all_xy(1,:);
state.O = cfg.init.O; state.H = cfg.init.H; state.F = cfg.init.F;
state.M = cfg.init.M; state.Z = cfg.init.Z;
state.consec = 0; state.day = 0;

plan_path = []; plan_parks = []; plan_works = [];
plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
wp_idx = 0; wd_done = 0;
replan_count = 0; survival_count = 0;

fprintf('========================================\n');
fprintf('  任务3 优化版在线随机决策\n');
fprintf('========================================\n');
fprintf('P0: O×%.2f H×%.2f F×%.2f | P1: 预防性超购10%% | P2: 生存优先\n', ...
    cons_exp.SAFETY_O, cons_exp.SAFETY_H, cons_exp.SAFETY_F);
fprintf('--- 逐日决策 ---\n');
fprintf('Day  | W | Action              | Pos         |  O   H   F Load |   Z     M\n');
fprintf('-----|---|---------------------|-------------|-----------------|----------\n');

while state.day < cfg.MAX_DAYS && state.pt ~= 2
    state.day = state.day + 1;
    w = weather_seq(state.day);
    if w == 'T', cons_act = cons_T; wname = 'T';
    else,        cons_act = cons_N; wname = 'N';
    end

    % ---- P2: 生存优先检查 ----
    dist_S1 = cfg.dist(state.pt, 6); dist_S2 = cfg.dist(state.pt, 7);
    nsd = min(dist_S1, dist_S2);
    survival_mode = (state.pt ~= 6) && (state.pt ~= 7) && ...
        ((state.O < 1.0 * nsd * cons_exp.MO) || ...
         (state.H < 1.0 * nsd * cons_exp.MH) || ...
         (state.F < 1.0 * nsd * cons_exp.MF));

    % ---- 重规划触发 ----
    at_named = (step_in_leg == 0 && parked_in_leg == 0);
    not_at_work = ~(state.pt >= 3 && state.pt <= 5);
    weather_changed = (state.day > 1 && w ~= weather_seq(max(1, state.day-1)));
    periodic = (mod(state.day, 3) == 1 && state.day > 1);

    need_replan = isempty(plan_path) || ...
        ((weather_changed || periodic) && at_named && not_at_work) || ...
        (survival_mode && at_named && not_at_work && ~isempty(plan_path) && any(plan_path(2:end-1) >= 3 & plan_path(2:end-1) <= 5));

    if need_replan
        replan_count = replan_count + 1;
        elapsed = state.day - 1;
        init_s = struct('O',state.O,'H',state.H,'F',state.F,'M',state.M,'Z',state.Z);

        if survival_mode && state.pt ~= 6 && state.pt ~= 7
            survival_count = survival_count + 1;
            [plan_path, plan_parks, plan_works, feasible] = ...
                cp_engine_opt('plan', state.pt, elapsed, cons_N, cfg, true);
        else
            [plan_path, plan_parks, plan_works, feasible] = ...
                cp_engine_opt('plan_scenario', state.pt, elapsed, cfg, false, init_s);
        end

        if ~feasible
            % 多层回退: 期望消耗 -> 正常消耗 -> 正常+生存模式
            [plan_path, plan_parks, plan_works, feasible] = ...
                cp_engine_opt('plan', state.pt, elapsed, cons_exp, cfg, false, init_s);
        end
        if ~feasible
            [plan_path, plan_parks, plan_works, feasible] = ...
                cp_engine_opt('plan', state.pt, elapsed, cons_N, cfg, survival_mode, init_s);
            if ~feasible
                fprintf('%4d | %s | NO PLAN             |             |                 | %5d %7.0f\n', ...
                    state.day, wname, state.Z, round(state.M));
                break;
            end
        end
        plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
        wp_idx = 0; wd_done = 0;
    end

    if length(plan_path) < 2, break; end
    next_pt = plan_path(plan_leg + 1);

    % ---- 动作决策 ----
    is_work_pt = (state.pt >= 3 && state.pt <= 5);
    wp_match = true;
    if is_work_pt && wp_idx < length(plan_works)
        wa_local = []; for ii = 2:length(plan_path)
            if plan_path(ii) >= 3 && plan_path(ii) <= 5, wa_local(end+1) = ii; end
        end
        if wp_idx + 1 <= length(wa_local)
            wp_match = (plan_path(wa_local(wp_idx+1)) == state.pt);
        end
    end
    need_work = is_work_pt && wp_match && wp_idx < length(plan_works) && wd_done < plan_works(wp_idx+1);

    act = ''; detail = ''; is_key = false;

    if need_work
        wk_type = state.pt - 2; wm_val = cfg.WM(wk_type); yld = cfg.WY(wk_type);
        if state.consec < wm_val
            act = sprintf('work(%s)', cfg.names{state.pt});
            state.O = state.O - cons_act.WO; state.H = state.H - cons_act.WH;
            state.F = state.F - cons_act.WF; state.Z = state.Z + yld;
            state.consec = state.consec + 1; wd_done = wd_done + 1;
            if wd_done >= plan_works(wp_idx+1), wp_idx = wp_idx + 1; wd_done = 0; end
            is_key = true;
        else
            act = 'park(reset)';
            state.O = state.O - cons_act.PO; state.H = state.H - cons_act.PH;
            state.F = state.F - cons_act.PF; state.consec = 0;
            is_key = true;
        end
    elseif plan_leg <= length(plan_parks) && parked_in_leg < plan_parks(plan_leg)
        act = 'park(at sea)';
        state.O = state.O - cons_act.PO; state.H = state.H - cons_act.PH;
        state.F = state.F - cons_act.PF; state.consec = 0;
        parked_in_leg = parked_in_leg + 1;
        is_key = true;
    elseif step_in_leg < cfg.dist(state.pt, next_pt)
        state.O = state.O - cons_act.MO; state.H = state.H - cons_act.MH;
        state.F = state.F - cons_act.MF; state.consec = 0;
        step_in_leg = step_in_leg + 1;
        fr = cfg.all_xy(state.pt, :); to = cfg.all_xy(next_pt, :);
        dx = to(1) - fr(1); dy = to(2) - fr(2);
        sx = abs(dx); sy = abs(dy);
        if step_in_leg <= sx
            state.pos(1) = fr(1) + sign(dx) * step_in_leg; state.pos(2) = fr(2);
        else
            state.pos(1) = to(1); state.pos(2) = fr(2) + sign(dy) * (step_in_leg - sx);
        end
        act = sprintf('move -> (%d,%d)', state.pos(1), state.pos(2));
        if step_in_leg >= cfg.dist(state.pt, next_pt)
            state.pt = next_pt; state.pos = cfg.all_xy(next_pt, :);
            step_in_leg = 0; parked_in_leg = 0;
            if state.pt >= 3 && state.pt <= 5
                wc = 0; for i = 2:plan_leg+1
                    if plan_path(i) >= 3 && plan_path(i) <= 5, wc = wc + 1; end
                end
                wp_idx = wc - 1; wd_done = 0;
            end
            if state.pt == 6 || state.pt == 7
                [needO, needH, needF] = cp_engine_opt('get_supply_needs', plan_path, plan_parks, plan_works, plan_leg, cons_exp, cfg);
                sp = cfg.MAX_LOAD - (state.O + state.H + state.F);
                bO = max(0, needO - state.O); bH = max(0, needH - state.H);
                bF = max(0, needF - state.F);
                if bO + bH + bF > sp + 1e-6
                    % 载重不足时按比例缩减至填满
                    scale = sp / (bO + bH + bF);
                    bO = bO * scale; bH = bH * scale; bF = bF * scale;
                end
                if bO + bH + bF <= sp + 1e-6
                    cost = bO * cons_act.pO + bH * cons_act.pH + bF * cons_act.pF;
                    if cost <= state.M + 1e-6
                        state.O = state.O + bO; state.H = state.H + bH;
                        state.F = state.F + bF; state.M = state.M - cost;
                        act = sprintf('SUPPLY(%s)', cfg.names{state.pt});
                        detail = sprintf('+O%.0f H%.0f F%.0f', bO, bH, bF);
                    else
                        % 预算不足时按比例缩减
                        scaleM = state.M / cost;
                        bO = bO * scaleM; bH = bH * scaleM; bF = bF * scaleM;
                        state.O = state.O + bO; state.H = state.H + bH;
                        state.F = state.F + bF; state.M = 0;
                        act = sprintf('SUPPLY(%s)-lim', cfg.names{state.pt});
                        detail = sprintf('+O%.0f H%.0f F%.0f (M exhausted)', bO, bH, bF);
                    end
                else, act = 'SUPPLY FAIL(ld)';
                end
                is_key = true;
            elseif state.pt == 2
                act = 'ARRIVE at E!'; is_key = true;
            end
            plan_leg = plan_leg + 1;
        end
    end

    if state.O < -1e-6 || state.H < -1e-6 || state.F < -1e-6
        act = 'EXHAUSTED!'; is_key = true;
        fprintf('%4d | %s | %-20s | (%2d,%2d)      | %3.0f %3.0f %3.0f %4.0f | %5d %7.0f  ***\n', ...
            state.day, wname, act, state.pos(1), state.pos(2), state.O, state.H, state.F, ...
            state.O+state.H+state.F, state.Z, round(state.M));
        break;
    end

    if is_key || need_replan
        tag = ''; if need_replan, tag = sprintf(' [R#%d]', replan_count); end
        if survival_mode, tag = [tag ' [SURV]']; end
        fprintf('%4d | %s | %-20s | (%2d,%2d)      | %3.0f %3.0f %3.0f %4.0f | %5d %7.0f%s', ...
            state.day, wname, act, state.pos(1), state.pos(2), state.O, state.H, state.F, ...
            state.O+state.H+state.F, state.Z, round(state.M), tag);
        if ~isempty(detail), fprintf('  %s', detail); end
        fprintf('\n');
    end
end

fprintf('-----|---|---------------------|-------------|-----------------|----------\n');
fprintf('\n===== 最终结果 =====\n');
if state.pt == 2
    fprintf('第 %d 天抵达E | Z=%d M=%.0f | 重规划:%d 生存模式:%d\n', ...
        state.day, state.Z, round(state.M), replan_count, survival_count);
elseif state.day >= cfg.MAX_DAYS
    fprintf('超时 | Z=%d M=%.0f\n', state.Z, round(state.M));
else
    fprintf('失败(day %d) | Z=%d M=%.0f\n', state.day, state.Z, round(state.M));
end
fprintf('\nDone.\n');
end
