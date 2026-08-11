function solve_q3_montecarlo_opt(N)
% =========================================================================
%  solve_q3_montecarlo_opt.m — 任务3 优化版蒙特卡洛验证
%  改进: P0-P2全部应用 + P4共享cp_engine_opt引擎
% =========================================================================

if nargin < 1 || isempty(N), N = 100; end

cfg = cp_engine_opt('task3_config');
cons_exp = cp_engine_opt('get_cons', 'expected', cfg);
cons_N   = cp_engine_opt('get_cons', 'normal', cfg);
cons_T   = cp_engine_opt('get_cons', 'thunder', cfg);

fprintf('========================================\n');
fprintf('  任务3 优化版蒙特卡洛 (N=%d)\n', N);
fprintf('========================================\n');
fprintf('P0: O×%.2f H×%.2f F×%.2f | P1: 预防性超购 | P2: 生存优先\n\n', ...
    cons_exp.SAFETY_O, cons_exp.SAFETY_H, cons_exp.SAFETY_F);

Z_results = NaN(1,N); M_results = NaN(1,N);
success = false(1,N); days_used = NaN(1,N);
fail_reason = cell(1,N);
survival_counts = zeros(1,N);

report_interval = max(1, floor(N/20));
tic;

for sim = 1:N
    wseq = cp_engine_opt('gen_weather', 90, 0.8);
    [Zf, Mf, arrived, days, reason, surv] = run_online_silent_opt(wseq);
    Z_results(sim) = Zf; M_results(sim) = Mf;
    success(sim) = arrived; days_used(sim) = days;
    fail_reason{sim} = reason; survival_counts(sim) = surv;

    if mod(sim, report_interval) == 0
        fprintf('  %d/%d (%.0f%%) | %.1fs | 成功率: %.1f%%\n', ...
            sim, N, 100*sim/N, toc, 100*sum(success(1:sim))/sim);
    end
end

total_time = toc; n_success = sum(success);
fprintf('\n完成。%.1fs\n\n', total_time);

fprintf('========================================\n');
fprintf('  统计结果\n');
fprintf('========================================\n');
fprintf('成功率: %d/%d (%.1f%%)\n', n_success, N, 100*n_success/N);
fprintf('平均生存模式触发: %.1f次/运行\n', mean(survival_counts));
fprintf('\n');

if n_success > 0
    Zs = Z_results(success); Ms = M_results(success); Ds = days_used(success);
    fprintf('--- 成功运行 (n=%d) ---\n', n_success);
    fprintf('%-16s %8s %8s %8s %8s\n', '指标', '均值', '标准差', '最小值', '最大值');
    fprintf('%-16s %8.1f %8.1f %8d %8d\n', 'Z', mean(Zs), std(Zs), min(Zs), max(Zs));
    fprintf('%-16s %8.2f %8.2f %8.2f %8.2f\n', 'M', mean(Ms), std(Ms), min(Ms), max(Ms));
    fprintf('%-16s %8.1f %8.1f %8d %8d\n', '天数', mean(Ds), std(Ds), min(Ds), max(Ds));
end

if N - n_success > 0
    fprintf('\n--- 失败原因 ---\n');
    fr = fail_reason(~success);
    [u, ~, ic] = unique(fr); cnts = accumarray(ic, 1);
    for i = 1:length(u), fprintf('  %s: %d\n', u{i}, cnts(i)); end
end

fprintf('\n分析完成。\n');
end

function [Zf, Mf, arrived, days, reason, surv_count] = run_online_silent_opt(wseq)
    cfg = cp_engine_opt('task3_config');
    cons_exp = cp_engine_opt('get_cons', 'expected', cfg);
    cons_N   = cp_engine_opt('get_cons', 'normal', cfg);
    cons_T   = cp_engine_opt('get_cons', 'thunder', cfg);

    state.pt = 1; state.pos = cfg.all_xy(1,:);
    state.O = cfg.init.O; state.H = cfg.init.H; state.F = cfg.init.F;
    state.M = cfg.init.M; state.Z = cfg.init.Z;
    state.consec = 0; state.day = 0;

    plan_path = []; plan_parks = []; plan_works = [];
    plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
    wp_idx = 0; wd_done = 0;
    reason = ''; surv_count = 0;

    while state.day < cfg.MAX_DAYS && state.pt ~= 2 && isempty(reason)
        state.day = state.day + 1;
        w = wseq(state.day);
        if w == 'T', cons_act = cons_T; else, cons_act = cons_N; end

        % P2: 生存优先
        dist_S1 = cfg.dist(state.pt, 6); dist_S2 = cfg.dist(state.pt, 7);
        nsd = min(dist_S1, dist_S2);
        survival_mode = (state.pt ~= 6) && (state.pt ~= 7) && ...
            ((state.O < 1.0 * nsd * cons_exp.MO) || ...
             (state.H < 1.0 * nsd * cons_exp.MH) || ...
             (state.F < 1.0 * nsd * cons_exp.MF));
        if survival_mode, surv_count = surv_count + 1; end

        at_named = (step_in_leg == 0 && parked_in_leg == 0);
        not_at_work = ~(state.pt >= 3 && state.pt <= 5);
        wc = (state.day > 1 && w ~= wseq(max(1,state.day-1)));
        periodic = (mod(state.day, 3) == 1 && state.day > 1);

        need_replan = isempty(plan_path) || ...
            ((wc || periodic) && at_named && not_at_work) || ...
            (survival_mode && at_named && not_at_work && ~isempty(plan_path) && ...
             any(plan_path(2:end-1) >= 3 & plan_path(2:end-1) <= 5));

        if need_replan
            elapsed = state.day - 1;
            init_s = struct('O',state.O,'H',state.H,'F',state.F,'M',state.M,'Z',state.Z);
            if survival_mode && state.pt ~= 6 && state.pt ~= 7
                [plan_path, plan_parks, plan_works, feasible] = ...
                    cp_engine_opt('plan', state.pt, elapsed, cons_N, cfg, true);
            else
                [plan_path, plan_parks, plan_works, feasible] = ...
                    cp_engine_opt('plan_scenario', state.pt, elapsed, cfg, false, init_s);
            end
            if ~feasible
                [plan_path, plan_parks, plan_works, feasible] = ...
                    cp_engine_opt('plan', state.pt, elapsed, cons_exp, cfg, false, init_s);
            end
            if ~feasible
                [plan_path, plan_parks, plan_works, feasible] = ...
                    cp_engine_opt('plan', state.pt, elapsed, cons_N, cfg, survival_mode, init_s);
                if ~feasible, reason = 'CP无可行方案'; break; end
            end
            plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
            wp_idx = 0; wd_done = 0;
        end

        if length(plan_path) < 2, reason = '空路径'; break; end
        next_pt = plan_path(plan_leg + 1);

        is_work_pt = (state.pt >= 3 && state.pt <= 5);
        wp_match = true;
        if is_work_pt && wp_idx < length(plan_works)
            wa_loc = []; for ii = 2:length(plan_path)
                if plan_path(ii)>=3 && plan_path(ii)<=5, wa_loc(end+1)=ii; end
            end
            if wp_idx+1 <= length(wa_loc), wp_match = (plan_path(wa_loc(wp_idx+1)) == state.pt); end
        end
        need_work = is_work_pt && wp_match && wp_idx < length(plan_works) && wd_done < plan_works(wp_idx+1);

        if need_work
            wk_type = state.pt - 2; wm_val = cfg.WM(wk_type); yld = cfg.WY(wk_type);
            if state.consec < wm_val
                state.O = state.O - cons_act.WO; state.H = state.H - cons_act.WH;
                state.F = state.F - cons_act.WF; state.Z = state.Z + yld;
                state.consec = state.consec + 1; wd_done = wd_done + 1;
                if wd_done >= plan_works(wp_idx+1), wp_idx = wp_idx + 1; wd_done = 0; end
            else
                state.O = state.O - cons_act.PO; state.H = state.H - cons_act.PH;
                state.F = state.F - cons_act.PF; state.consec = 0;
            end
        elseif plan_leg <= length(plan_parks) && parked_in_leg < plan_parks(plan_leg)
            state.O = state.O - cons_act.PO; state.H = state.H - cons_act.PH;
            state.F = state.F - cons_act.PF; state.consec = 0;
            parked_in_leg = parked_in_leg + 1;
        elseif step_in_leg < cfg.dist(state.pt, next_pt)
            state.O = state.O - cons_act.MO; state.H = state.H - cons_act.MH;
            state.F = state.F - cons_act.MF; state.consec = 0;
            step_in_leg = step_in_leg + 1;
            if step_in_leg >= cfg.dist(state.pt, next_pt)
                state.pt = next_pt; state.pos = cfg.all_xy(next_pt, :);
                step_in_leg = 0; parked_in_leg = 0;
                if state.pt >= 3 && state.pt <= 5
                    wc = 0; for i = 2:plan_leg+1
                        if plan_path(i)>=3 && plan_path(i)<=5, wc=wc+1; end
                    end
                    wp_idx = wc - 1; wd_done = 0;
                end
                if state.pt == 6 || state.pt == 7
                    [needO, needH, needF] = cp_engine_opt('get_supply_needs', plan_path, plan_parks, plan_works, plan_leg, cons_exp, cfg);
                    sp = cfg.MAX_LOAD - (state.O + state.H + state.F);
                    bO = max(0, needO-state.O); bH = max(0, needH-state.H); bF = max(0, needF-state.F);
                    if bO+bH+bF > sp+1e-6
                        scale = sp / (bO+bH+bF);
                        bO = bO*scale; bH = bH*scale; bF = bF*scale;
                    end
                    if bO+bH+bF <= sp+1e-6
                        cost = bO*cons_act.pO + bH*cons_act.pH + bF*cons_act.pF;
                        if cost <= state.M+1e-6
                            state.O=state.O+bO; state.H=state.H+bH; state.F=state.F+bF; state.M=state.M-cost;
                        else
                            scaleM = state.M / cost;
                            bO=bO*scaleM; bH=bH*scaleM; bF=bF*scaleM;
                            state.O=state.O+bO; state.H=state.H+bH; state.F=state.F+bF; state.M=0;
                        end
                    else, reason = '补给超载重'; break;
                    end
                end
                plan_leg = plan_leg + 1;
            end
        end

        if state.O < -1e-6 || state.H < -1e-6 || state.F < -1e-6
            reason = '资源耗尽'; break;
        end
        if state.O + state.H + state.F > cfg.MAX_LOAD + 1e-6
            reason = '超载'; break;
        end
    end

    if isempty(reason) && state.pt == 2, arrived = true; days = state.day;
    elseif isempty(reason) && state.day >= cfg.MAX_DAYS, reason = '超时'; arrived = false; days = cfg.MAX_DAYS;
    else, arrived = false; days = state.day;
    end

    Zf = state.Z; Mf = state.M;
    if ~arrived, Zf = 0; Mf = 0; end
end
