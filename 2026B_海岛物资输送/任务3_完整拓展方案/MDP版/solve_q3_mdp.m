function solve_q3_mdp(mode, N_or_weather)
% =========================================================================
%  solve_q3_mdp.m — 任务3 MDP蒙特卡洛Rollout求解器
%
%  用法:
%    solve_q3_mdp('offline')      离线CP搜索 (复用优化版引擎)
%    solve_q3_mdp('online')       在线MDP决策 (单次, K=10采样)
%    solve_q3_mdp('mc', N)        蒙特卡洛验证 (N次, K=10采样)
%    solve_q3_mdp('online_k', K)  在线MDP决策 (自定义K)
%  =========================================================================

addpath('C:\Users\ming\Desktop\任务3_完整拓展方案\优化版');

if nargin < 1, mode = 'online'; end
if nargin < 2
    if strcmp(mode, 'mc'), N_or_weather = 10; else, N_or_weather = []; end
end

cfg = cp_engine_opt('task3_config');

switch mode
    case 'offline'
        run_offline_cp(cfg);
    case 'online'
        mdp_online_run(cfg, 10, N_or_weather);
    case 'online_k'
        K = N_or_weather; if isempty(K), K = 10; end
        mdp_online_run(cfg, K, []);
    case 'mc'
        mdp_montecarlo(cfg, N_or_weather, 10);
    otherwise
        fprintf('Unknown mode: %s\n', mode);
end
end

% =====================================================================
function run_offline_cp(cfg)
    cons = cp_engine_opt('get_cons', 'expected', cfg);
    tic;
    [best_path, ~, best_works, feasible] = cp_engine_opt('plan', 1, 0, cons, cfg, false);
    elapsed = toc;
    fprintf('MDP版离线CP (复用优化引擎): Z求解完成, %.2fs\n', elapsed);
end

% =====================================================================
%  MDP在线决策主循环
% =====================================================================
function mdp_online_run(cfg, K, weather_seq)
    cons_exp = cp_engine_opt('get_cons', 'expected', cfg);
    cons_N   = cp_engine_opt('get_cons', 'normal', cfg);
    cons_T   = cp_engine_opt('get_cons', 'thunder', cfg);

    if isempty(weather_seq)
        rng('shuffle');
        weather_seq = cp_engine_opt('gen_weather', 90, 0.8);
    end

    state.pt = 1; state.pos = cfg.all_xy(1,:);
    state.O = cfg.init.O; state.H = cfg.init.H; state.F = cfg.init.F;
    state.M = cfg.init.M; state.Z = cfg.init.Z;
    state.consec = 0; state.day = 0;
    plan_path = []; plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
    wp_idx = 0; wd_done = 0; plan_works = []; plan_parks = [];
    decisions = 0;

    fprintf('========== MDP在线决策 (K=%d) ==========\n', K);
    fprintf('Day  | W | Action              |  O   H   F |   Z     M\n');
    fprintf('-----|---|---------------------|------------|----------\n');

    while state.day < cfg.MAX_DAYS && state.pt ~= 2
        state.day = state.day + 1;
        w = weather_seq(state.day);
        wname = 'N'; if w == 'T', wname = 'T'; end
        if w == 'T', cons_act = cons_T; else, cons_act = cons_N; end

        at_named = (step_in_leg == 0 && parked_in_leg == 0);
        not_at_work = ~(state.pt >= 3 && state.pt <= 5);
        wc = (state.day > 1 && w ~= weather_seq(max(1,state.day-1)));

        need_replan = isempty(plan_path) || ...
            ((wc || mod(state.day,5)==1) && at_named && not_at_work && state.day > 1);

        if need_replan
            decisions = decisions + 1;
            elapsed = state.day - 1;
            init_s = struct('O',state.O,'H',state.H,'F',state.F,'M',state.M,'Z',state.Z);

            % ---- MDP Rollout决策 ----
            [plan_path, plan_parks, plan_works] = mdp_rollout_decision(...
                state.pt, elapsed, init_s, cfg, cons_exp, cons_N, K, weather_seq(state.day:end));

            if isempty(plan_path)
                fprintf('%4d | %s | NO PLAN             |            | %5d %6.0f\n', ...
                    state.day, wname, state.Z, round(state.M));
                break;
            end
            plan_leg = 1; step_in_leg = 0; parked_in_leg = 0;
            wp_idx = 0; wd_done = 0;
        end

        if length(plan_path) < 2, break; end
        next_pt = plan_path(plan_leg + 1);

        % 动作决策 (与原版在线模型一致)
        is_work_pt = (state.pt >= 3 && state.pt <= 5);
        wp_match = true;
        if is_work_pt && wp_idx < length(plan_works)
            wa_loc = []; for ii=2:length(plan_path)
                if plan_path(ii)>=3 && plan_path(ii)<=5, wa_loc(end+1)=ii; end
            end
            if wp_idx+1 <= length(wa_loc)
                wp_match = (plan_path(wa_loc(wp_idx+1)) == state.pt);
            end
        end
        need_work = is_work_pt && wp_match && wp_idx<length(plan_works) && wd_done<plan_works(wp_idx+1);

        act = '';
        if need_work
            wk = state.pt - 2; wm = cfg.WM(wk); yld = cfg.WY(wk);
            if state.consec < wm
                act = sprintf('work(%s)', cfg.names{state.pt});
                state.O=state.O-cons_act.WO; state.H=state.H-cons_act.WH;
                state.F=state.F-cons_act.WF; state.Z=state.Z+yld;
                state.consec=state.consec+1; wd_done=wd_done+1;
                if wd_done>=plan_works(wp_idx+1), wp_idx=wp_idx+1; wd_done=0; end
            else
                act='park(reset)';
                state.O=state.O-cons_act.PO; state.H=state.H-cons_act.PH;
                state.F=state.F-cons_act.PF; state.consec=0;
            end
        elseif plan_leg<=length(plan_parks) && parked_in_leg<plan_parks(plan_leg)
            act='park(at sea)';
            state.O=state.O-cons_act.PO; state.H=state.H-cons_act.PH;
            state.F=state.F-cons_act.PF; state.consec=0; parked_in_leg=parked_in_leg+1;
        elseif step_in_leg < cfg.dist(state.pt, next_pt)
            state.O=state.O-cons_act.MO; state.H=state.H-cons_act.MH;
            state.F=state.F-cons_act.MF; state.consec=0; step_in_leg=step_in_leg+1;
            if step_in_leg >= cfg.dist(state.pt, next_pt)
                state.pt=next_pt; state.pos=cfg.all_xy(next_pt,:);
                step_in_leg=0; parked_in_leg=0;
                if state.pt>=3 && state.pt<=5
                    wc=0; for i=2:plan_leg+1
                        if plan_path(i)>=3&&plan_path(i)<=5, wc=wc+1; end
                    end; wp_idx=wc-1; wd_done=0;
                end
                if state.pt==6 || state.pt==7
                    rem_t=0; rem_p=0; workO=0; workH=0; workF=0;
                    wp_count=0;
                    for k=plan_leg:length(plan_path)-2
                        rem_t=rem_t+cfg.dist(plan_path(k+1),plan_path(k+2));
                        if k+1<=length(plan_parks), rem_p=rem_p+plan_parks(k+1); end
                        if plan_path(k+1)>=3 && plan_path(k+1)<=5
                            wp_count=wp_count+1;
                            if wp_count<=length(plan_works)
                                wd=plan_works(wp_count); wi=plan_path(k+1)-2;
                                np=max(0,ceil(wd/cfg.WM(wi))-1);
                                workO=workO+wd*cons_exp.WO+np*cons_exp.PO;
                                workH=workH+wd*cons_exp.WH+np*cons_exp.PH;
                                workF=workF+wd*cons_exp.WF+np*cons_exp.PF;
                            end
                        end
                    end
                    needO=rem_t*cons_exp.MO+rem_p*cons_exp.PO+workO;
                    needH=rem_t*cons_exp.MH+rem_p*cons_exp.PH+workH;
                    needF=rem_t*cons_exp.MF+rem_p*cons_exp.PF+workF;
                    sp=cfg.MAX_LOAD-(state.O+state.H+state.F);
                    bO=max(0,needO-state.O); bH=max(0,needH-state.H); bF=max(0,needF-state.F);
                    if bO+bH+bF>sp+1e-6
                        sc=sp/(bO+bH+bF); bO=bO*sc; bH=bH*sc; bF=bF*sc;
                    end
                    cost=bO*cons_act.pO+bH*cons_act.pH+bF*cons_act.pF;
                    if cost<=state.M+1e-6
                        state.O=state.O+bO; state.H=state.H+bH; state.F=state.F+bF; state.M=state.M-cost;
                        act=sprintf('SUPPLY(%s)',cfg.names{state.pt});
                    end
                elseif state.pt==2, act='ARRIVE!';
                end
                plan_leg=plan_leg+1;
            end
        end

        if state.O<-1e-6||state.H<-1e-6||state.F<-1e-6
            fprintf('%4d | %s | EXHAUSTED!           |%4.0f%4.0f%4.0f |%5d %6.0f\n',...
                state.day,wname,state.O,state.H,state.F,state.Z,round(state.M));
            break;
        end

        if need_replan || contains(act,'SUPPLY') || contains(act,'work') || contains(act,'ARRIVE')
            fprintf('%4d | %s | %-20s |%4.0f%4.0f%4.0f |%5d %6.0f',...
                state.day,wname,act,state.O,state.H,state.F,state.Z,round(state.M));
            if need_replan, fprintf(' [MDP#%d]',decisions); end
            fprintf('\n');
        end
    end

    fprintf('-----|---|---------------------|------------|----------\n');
    if state.pt==2
        fprintf('抵达E! Z=%d M=%.0f Day=%d MDP决策:%d\n',state.Z,round(state.M),state.day,decisions);
    else
        fprintf('失败 Day=%d Z=%d M=%.0f\n',state.day,state.Z,round(state.M));
    end
end

% =====================================================================
%  MDP Rollout决策核心
% =====================================================================
function [best_path, best_parks, best_works] = mdp_rollout_decision(...
    cur_pt, elapsed, init_s, cfg, cons_exp, cons_N, K, remaining_weather)

    candidates = cfg.inter_idx;  % {W1,W2,W3,S1,S2}
    best_avgZ = -inf; best_avgM = -inf;
    best_path = []; best_parks = []; best_works = [];

    rng_state = rng;  % 保存随机种子以保证公平比较
    for ci = 1:length(candidates)
        np = candidates(ci);
        d = cfg.dist(cur_pt, np);
        if elapsed + d + cfg.dist(np, 2) > cfg.MAX_DAYS, continue; end

        rng(rng_state);  % 每个候选动作使用相同随机序列
        totalZ = 0; totalM = 0; successes = 0;
        for k = 1:K
            rem_days = cfg.MAX_DAYS - elapsed;
            w_sim = cp_engine_opt('gen_weather', rem_days, 0.8);

            % 模拟: 移动到np, 然后CP搜索剩余路径
            [Zk, Mk, arrived] = simulate_one_trajectory(...
                cur_pt, np, elapsed, init_s, cfg, cons_exp, cons_N, w_sim);
            totalZ = totalZ + Zk;
            if arrived
                totalM = totalM + Mk; successes = successes + 1;
            end
        end

        avgZ = totalZ / K; avgM = totalM / K;
        if avgZ > best_avgZ || (abs(avgZ - best_avgZ) < 1e-6 && avgM > best_avgM)
            best_avgZ = avgZ; best_avgM = avgM;
            if successes > 0
                [best_path, best_parks, best_works] = ...
                    cp_engine_opt('plan', cur_pt, elapsed, cons_exp, cfg, false);
            else
                % 全失败时至少返回直赴E的兜底路径
                best_path = [cur_pt, 2]; best_parks = []; best_works = [];
            end
        end
    end
end

% =====================================================================
%  单条天气轨迹模拟
% =====================================================================
function [Zf, Mf, arrived] = simulate_one_trajectory(...
    cur_pt, np, elapsed, init_s, cfg, cons_exp, cons_N, w_sim)

    O = init_s.O; H = init_s.H; F = init_s.F;
    M = init_s.M; Z = init_s.Z;
    day_offset = 0;

    % 移动到目标点np
    d = cfg.dist(cur_pt, np);
    for dd = 1:d
        day_offset = day_offset + 1;
        if day_offset > length(w_sim)
            arrived = false; Zf = Z; Mf = M; return;
        end
        w = w_sim(day_offset);
        if w == 'T'
            O = O - 8; H = H - 4; F = F - 3;
        else
            O = O - 2; H = H - 3; F = F - 2;
        end
        if O < -1e-6 || H < -1e-6 || F < -1e-6
            arrived = false; Zf = 0; Mf = 0; return;
        end
        if O + H + F > cfg.MAX_LOAD + 1e-6
            arrived = false; Zf = 0; Mf = 0; return;
        end
    end

    % 到达np后, 用CP搜索剩余路径
    new_elapsed = elapsed + d;
    new_init = struct('O',O,'H',H,'F',F,'M',M,'Z',Z);
    [plan_path, plan_parks, plan_works, feasible] = ...
        cp_engine_opt('plan', np, new_elapsed, cons_exp, cfg, false);

    if ~feasible
        [plan_path, plan_parks, plan_works, feasible] = ...
            cp_engine_opt('plan', np, new_elapsed, cons_N, cfg, false);
    end
    if ~feasible
        arrived = false; Zf = Z; Mf = M; return;
    end

    % 重建作业点映射 (wa: plan_path中的作业点位置)
    wa_idx = []; for ii = 2:length(plan_path)
        if plan_path(ii)>=3 && plan_path(ii)<=5, wa_idx(end+1)=ii; end
    end
    wp_order = 0;

    % 如果np是作业点, 先在工作点np执行作业 (Bug修复)
    if np >= 3 && np <= 5 && ~isempty(wa_idx) && plan_path(wa_idx(1)) == np
        wp_order = 1;
        wd = plan_works(1); wi = np - 2;
        if wd > 0
            npark = max(0, ceil(wd / cfg.WM(wi)) - 1);
            for wday = 1:wd
                day_offset = day_offset + 1;
                if day_offset > length(w_sim), arrived = false; Zf = Z; Mf = M; return; end
                if w_sim(day_offset) == 'T', O=O-8; H=H-6; F=F-6;
                else, O=O-5; H=H-4; F=F-3; end
                Z = Z + cfg.WY(wi);
                if O<-1e-6||H<-1e-6||F<-1e-6, arrived=false; Zf=0; Mf=0; return; end
            end
            for pd = 1:npark
                day_offset = day_offset + 1;
                if day_offset > length(w_sim), arrived = false; Zf = Z; Mf = M; return; end
                if w_sim(day_offset) == 'T', O=O-3; H=H-3; F=F-2;
                else, O=O-1; H=H-1; F=F-1; end
                if O<-1e-6||H<-1e-6||F<-1e-6, arrived=false; Zf=0; Mf=0; return; end
            end
        end
    end

    % 沿CP路径继续移动+工作 (Bug修复: 按wa索引plan_works)
    pt = np;
    for leg = 1:(length(plan_path)-1)
        next_pt = plan_path(leg + 1);
        dd = cfg.dist(pt, next_pt);
        for s = 1:dd
            day_offset = day_offset + 1;
            if day_offset > length(w_sim), arrived = false; Zf = Z; Mf = M; return; end
            if w_sim(day_offset) == 'T', O=O-8; H=H-4; F=F-3;
            else, O=O-2; H=H-3; F=F-2; end
            if O<-1e-6||H<-1e-6||F<-1e-6, arrived=false; Zf=0; Mf=0; return; end
        end
        pt = next_pt;
        % 作业处理: 检查pt是否在wa_idx中
        wk_found = find(wa_idx == leg+1, 1);
        if ~isempty(wk_found) && wk_found > wp_order && wk_found <= length(plan_works)
            wd = plan_works(wk_found); wi = pt - 2;
            if wd > 0
                npark = max(0, ceil(wd / cfg.WM(wi)) - 1);
                for wday = 1:wd
                    day_offset = day_offset + 1;
                    if day_offset > length(w_sim), arrived = false; Zf = Z; Mf = M; return; end
                    if w_sim(day_offset) == 'T', O=O-8; H=H-6; F=F-6;
                    else, O=O-5; H=H-4; F=F-3; end
                    Z = Z + cfg.WY(wi);
                    if O<-1e-6||H<-1e-6||F<-1e-6, arrived=false; Zf=0; Mf=0; return; end
                end
                for pd = 1:npark
                    day_offset = day_offset + 1;
                    if day_offset > length(w_sim), arrived = false; Zf = Z; Mf = M; return; end
                    if w_sim(day_offset) == 'T', O=O-3; H=H-3; F=F-2;
                    else, O=O-1; H=H-1; F=F-1; end
                    if O<-1e-6||H<-1e-6||F<-1e-6, arrived=false; Zf=0; Mf=0; return; end
                end
            end
        end
    end

    arrived = true; Zf = Z; Mf = M;
end

% =====================================================================
%  蒙特卡洛验证
% =====================================================================
function mdp_montecarlo(cfg, N, K)
    if nargin < 3, K = 10; end
    fprintf('========== MDP MC验证 (N=%d, K=%d) ==========\n', N, K);

    Z_results = NaN(1,N); M_results = NaN(1,N);
    success = false(1,N); days_used = NaN(1,N);
    fail_reason = cell(1,N);

    tic;
    for sim = 1:N
        wseq = cp_engine_opt('gen_weather', 90, 0.8);
        [Zf, Mf, arrived, days, reason] = run_mdp_silent(cfg, wseq, K);
        Z_results(sim) = Zf; M_results(sim) = Mf;
        success(sim) = arrived; days_used(sim) = days;
        fail_reason{sim} = reason;

        if mod(sim, max(1,floor(N/10))) == 0
            fprintf('  %d/%d (%.0f%%) | %.1fs | 成功率: %.1f%%\n', ...
                sim, N, 100*sim/N, toc, 100*sum(success(1:sim))/sim);
        end
    end

    n_success = sum(success);
    fprintf('\n完成 %.1fs | 成功率: %d/%d (%.1f%%)\n', toc, n_success, N, 100*n_success/N);
    if n_success > 0
        Zs = Z_results(success); Ms = M_results(success);
        fprintf('Z: mean=%.0f std=%.0f min=%d max=%d\n', mean(Zs), std(Zs), min(Zs), max(Zs));
        fprintf('M: mean=%.0f std=%.0f min=%.0f max=%.0f\n', mean(Ms), std(Ms), min(Ms), max(Ms));
    end
end

function [Zf, Mf, arrived, days, reason] = run_mdp_silent(cfg, weather_seq, K)
    cons_exp = cp_engine_opt('get_cons', 'expected', cfg);
    cons_N = cp_engine_opt('get_cons', 'normal', cfg);
    cons_T = cp_engine_opt('get_cons', 'thunder', cfg);

    state.pt=1; state.O=cfg.init.O; state.H=cfg.init.H; state.F=cfg.init.F;
    state.M=cfg.init.M; state.Z=cfg.init.Z; state.consec=0; state.day=0;
    plan_path=[]; plan_leg=1; step_in_leg=0; parked_in_leg=0;
    wp_idx=0; wd_done=0; plan_works=[]; plan_parks=[];
    reason='';

    while state.day < cfg.MAX_DAYS && state.pt ~= 2 && isempty(reason)
        state.day = state.day + 1; w = weather_seq(state.day);
        if w=='T', cons_act=cons_T; else, cons_act=cons_N; end

        at_named = (step_in_leg==0 && parked_in_leg==0);
        not_at_work = ~(state.pt>=3 && state.pt<=5);
        wc = (state.day>1 && w~=weather_seq(max(1,state.day-1)));
        need_replan = isempty(plan_path) || ...
            ((wc || mod(state.day,5)==1) && at_named && not_at_work && state.day>1);

        if need_replan
            init_s=struct('O',state.O,'H',state.H,'F',state.F,'M',state.M,'Z',state.Z);
            [plan_path,plan_parks,plan_works] = mdp_rollout_decision(...
                state.pt, state.day-1, init_s, cfg, cons_exp, cons_N, K, weather_seq(state.day:end));
            if isempty(plan_path), reason='MDP无可行动作'; break; end
            plan_leg=1; step_in_leg=0; parked_in_leg=0; wp_idx=0; wd_done=0;
        end

        if length(plan_path)<2, reason='空路径'; break; end
        next_pt=plan_path(plan_leg+1);
        is_work_pt=(state.pt>=3 && state.pt<=5);
        wp_match=true;
        if is_work_pt && wp_idx<length(plan_works)
            wa_loc=[]; for ii=2:length(plan_path)
                if plan_path(ii)>=3&&plan_path(ii)<=5, wa_loc(end+1)=ii; end
            end
            if wp_idx+1<=length(wa_loc), wp_match=(plan_path(wa_loc(wp_idx+1))==state.pt); end
        end
        need_work=is_work_pt&&wp_match&&wp_idx<length(plan_works)&&wd_done<plan_works(wp_idx+1);

        if need_work
            wk=state.pt-2; wm=cfg.WM(wk); yld=cfg.WY(wk);
            if state.consec<wm
                state.O=state.O-cons_act.WO; state.H=state.H-cons_act.WH;
                state.F=state.F-cons_act.WF; state.Z=state.Z+yld;
                state.consec=state.consec+1; wd_done=wd_done+1;
                if wd_done>=plan_works(wp_idx+1), wp_idx=wp_idx+1; wd_done=0; end
            else
                state.O=state.O-cons_act.PO; state.H=state.H-cons_act.PH;
                state.F=state.F-cons_act.PF; state.consec=0;
            end
        elseif plan_leg<=length(plan_parks)&&parked_in_leg<plan_parks(plan_leg)
            state.O=state.O-cons_act.PO; state.H=state.H-cons_act.PH;
            state.F=state.F-cons_act.PF; state.consec=0; parked_in_leg=parked_in_leg+1;
        elseif step_in_leg<cfg.dist(state.pt,next_pt)
            state.O=state.O-cons_act.MO; state.H=state.H-cons_act.MH;
            state.F=state.F-cons_act.MF; state.consec=0; step_in_leg=step_in_leg+1;
            if step_in_leg>=cfg.dist(state.pt,next_pt)
                state.pt=next_pt; step_in_leg=0; parked_in_leg=0;
                if state.pt>=3&&state.pt<=5
                    wc=0; for i=2:plan_leg+1
                        if plan_path(i)>=3&&plan_path(i)<=5, wc=wc+1; end
                    end; wp_idx=wc-1; wd_done=0;
                end
                if state.pt==6||state.pt==7
                    rem_t=0; rem_p=0; workO=0; workH=0; workF=0; wp_count=0;
                    for k=plan_leg:length(plan_path)-2
                        rem_t=rem_t+cfg.dist(plan_path(k+1),plan_path(k+2));
                        if k+1<=length(plan_parks), rem_p=rem_p+plan_parks(k+1); end
                        if plan_path(k+1)>=3&&plan_path(k+1)<=5
                            wp_count=wp_count+1;
                            if wp_count<=length(plan_works)
                                wd=plan_works(wp_count); wi=plan_path(k+1)-2;
                                np=max(0,ceil(wd/cfg.WM(wi))-1);
                                workO=workO+wd*cons_exp.WO+np*cons_exp.PO;
                                workH=workH+wd*cons_exp.WH+np*cons_exp.PH;
                                workF=workF+wd*cons_exp.WF+np*cons_exp.PF;
                            end
                        end
                    end
                    needO=rem_t*cons_exp.MO+rem_p*cons_exp.PO+workO;
                    needH=rem_t*cons_exp.MH+rem_p*cons_exp.PH+workH;
                    needF=rem_t*cons_exp.MF+rem_p*cons_exp.PF+workF;
                    sp=cfg.MAX_LOAD-(state.O+state.H+state.F);
                    bO=max(0,needO-state.O); bH=max(0,needH-state.H); bF=max(0,needF-state.F);
                    if bO+bH+bF>sp+1e-6, sc=sp/(bO+bH+bF); bO=bO*sc; bH=bH*sc; bF=bF*sc; end
                    cost=bO*cons_act.pO+bH*cons_act.pH+bF*cons_act.pF;
                    if cost<=state.M+1e-6
                        state.O=state.O+bO; state.H=state.H+bH; state.F=state.F+bF; state.M=state.M-cost;
                    else, reason='补给资金不足'; break;
                    end
                end
                plan_leg=plan_leg+1;
            end
        end
        if state.O<-1e-6||state.H<-1e-6||state.F<-1e-6, reason='资源耗尽'; break; end
        if state.O+state.H+state.F>cfg.MAX_LOAD+1e-6, reason='超载'; break; end
    end

    if isempty(reason)&&state.pt==2, arrived=true; days=state.day;
    elseif isempty(reason)&&state.day>=cfg.MAX_DAYS, reason='超时'; arrived=false; days=cfg.MAX_DAYS;
    else, arrived=false; days=state.day;
    end
    Zf=state.Z; Mf=state.M;
    if ~arrived, Zf=0; Mf=0; end
end
