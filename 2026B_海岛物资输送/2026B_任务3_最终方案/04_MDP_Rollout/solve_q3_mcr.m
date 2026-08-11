function solve_q3_mcr(mode, N_or_weather)
% =========================================================================
%  solve_q3_mcr.m 鈥?浠诲姟3 MDP钂欑壒鍗℃礇Rollout姹傝В鍣?(v2.0 淇鐗?
%
%  淇: mdp_rollout_decision浣跨敤plan_from_candidate纭疄鎵ц鍊欓€夊姩浣?
%        simulate_one_trajectory鏇挎崲涓篶p_engine_opt.simulate_with_weather
%        addpath浣跨敤鐩稿璺緞 | 琛ョ粰璋冪敤浼犲叆completed_wp
%
%  鐢ㄦ硶:
%    solve_q3_mcr('offline')      绂荤嚎CP鎼滅储 (澶嶇敤浼樺寲鐗堝紩鎿?
%    solve_q3_mcr('online')       鍦ㄧ嚎MDP鍐崇瓥 (鍗曟, K=20閲囨牱)
%    solve_q3_mcr('mc', N)       钂欑壒鍗℃礇楠岃瘉 (N娆?, K=20閲囨牱)
%    solve_q3_mcr('online_k', K)  鍦ㄧ嚎MDP鍐崇瓥 (鑷畾涔塅)
%  =========================================================================

% 鑷姩娣诲姞褰撳墠鐩綍涓嬬殑浼樺寲鐗堣矾寰?this_dir = fileparts(mfilename('fullpath'));
opt_dir = fullfile(this_dir, '..', '02_浼樺寲鐗坃鏀硅繘CP');
if exist(opt_dir, 'dir'), addpath(opt_dir); end

if nargin < 1, mode = 'online'; end
if nargin < 2
    if strcmp(mode, 'mc'), N_or_weather = 20; else, N_or_weather = []; end
end

cfg = cp_engine_opt('task3_config');

switch mode
    case 'offline'
        run_offline_cp(cfg);
    case 'online'
        mdp_online_run(cfg, 20, N_or_weather);
    case 'online_k'
        K = N_or_weather; if isempty(K), K = 20; end
        mdp_online_run(cfg, K, []);
    case 'mc'
        mdp_montecarlo(cfg, N_or_weather, 20);
    otherwise
        fprintf('Unknown mode: %s\n', mode);
end
end

% =====================================================================
function run_offline_cp(cfg)
    cons = cp_engine_opt('get_cons', 'expected', cfg);
    tic;
    [~, ~, ~, feasible] = cp_engine_opt('plan', 1, 0, cons, cfg, false);
    elapsed = toc;
    fprintf('MDP鐗堢绾緾P (澶嶇敤浼樺寲寮曟搸): Z姹傝В瀹屾垚, %.2fs\n', elapsed);
end

% =====================================================================
%  MDP鍦ㄧ嚎鍐崇瓥涓诲惊鐜?% =====================================================================
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

    fprintf('========== MDP鍦ㄧ嚎鍐崇瓥 (K=%d) ==========\n', K);
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

            % ---- MDP Rollout鍐崇瓥 (淇: 杩斿洖閫変腑鍊欓€夌殑璁″垝) ----
            [plan_path, plan_parks, plan_works] = mdp_rollout_decision(...
                state.pt, elapsed, init_s, cfg, cons_exp, cons_N, cons_T, K);

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

        % 鍔ㄤ綔鍐崇瓥
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
                    [needO, needH, needF] = cp_engine_opt('get_supply_needs', ...
                        plan_path, plan_parks, plan_works, plan_leg, cons_exp, cfg, wp_idx);
                    sp=cfg.MAX_LOAD-(state.O+state.H+state.F);
                    bO=max(0,needO-state.O); bH=max(0,needH-state.H); bF=max(0,needF-state.F);
                    if bO+bH+bF>sp+1e-6
                        sc=sp/(bO+bH+bF); bO=bO*sc; bH=bH*sc; bF=bF*sc;
                    end
                    cost=bO*cons_act.pO+bH*cons_act.pH+bF*cons_act.pF;
                    if cost<=state.M+1e-6
                        state.O=state.O+bO; state.H=state.H+bH; state.F=state.F+bF; state.M=state.M-cost;
                        act=sprintf('SUPPLY(%s)',cfg.names{state.pt});
                    else
                        scM=min(1,state.M/cost);
                        bO=bO*scM; bH=bH*scM; bF=bF*scM;
                        state.O=state.O+bO; state.H=state.H+bH; state.F=state.F+bF; state.M=state.M-cost*scM;
                        act=sprintf('SUPPLY(%s)-lim',cfg.names{state.pt});
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
        fprintf('鎶佃揪E! Z=%d M=%.0f Day=%d MDP鍐崇瓥:%d\n',state.Z,round(state.M),state.day,decisions);
    else
        fprintf('澶辫触 Day=%d Z=%d M=%.0f\n',state.day,state.Z,round(state.M));
    end
end

% =====================================================================
%  MDP Rollout鍐崇瓥鏍稿績 (淇鐗?
%  瀵规瘡涓€欓€夌洰鏍嘗p, 浠巆ur_pt鈫抧p鐒跺悗CP鎼滅储鍓╀綑, 閲囨牱K鏉″ぉ姘旇建杩?,  杩斿洖鏈€浼榓vgZ鐨勫搴斿畬鏁磋鍒?(鍖呭惈cur_pt鈫抧p)
% =====================================================================
function [best_path, best_parks, best_works] = mdp_rollout_decision(...
    cur_pt, elapsed, init_s, cfg, cons_exp, cons_N, cons_T, K)

    candidates = cfg.inter_idx;
    best_avgZ = -inf; best_avgM = -inf;
    best_result = {};
    rng_state = rng;

    for ci = 1:length(candidates)
        np = candidates(ci);
        d = cfg.dist(cur_pt, np);
        if elapsed + d + cfg.dist(np, 2) > cfg.MAX_DAYS, continue; end

        % 浠?np 瑙勫垝鍓╀綑璺緞
        [rest_path, rest_parks, rest_works, ok] = ...
            [full_path_cand, full_parks_cand, full_works_cand, ok] = ...
            cp_engine_opt('plan_from_candidate', cur_pt, np, elapsed, cons_exp, cfg, init_s);
        if ~ok, continue; end

        full_path = full_path_cand; full_parks = full_parks_cand; full_works = full_works_cand;

        % K娆￠噰鏍疯瘎浼?        totalZ = 0; totalM = 0; successes = 0;
        rng(rng_state);
        for k = 1:K
            w_sim = cp_engine_opt('gen_weather', cfg.MAX_DAYS - elapsed, 0.8);
            [Zk, Mk, arrived] = cp_engine_opt('simulate_with_weather', ...
                full_path, full_parks, full_works, cur_pt, elapsed, init_s, cfg, cons_N, cons_T, w_sim);
            if arrived
                totalZ = totalZ + Zk; totalM = totalM + Mk; successes = successes + 1;
            end
        end

        avgZ = totalZ / K; avgM = 0;
        if successes > 0, avgM = totalM / successes; end

        if avgZ > best_avgZ || (abs(avgZ - best_avgZ) < 1e-6 && avgM > best_avgM)
            best_avgZ = avgZ; best_avgM = avgM;
            best_result = {full_path, full_parks, full_works};
        end
    end

    if isempty(best_result)
        % 鍏ㄥけ璐?: 杩斿洖鐩磋荡E鐨勫厹搴曡矾寰?        best_result = {[cur_pt, 2], zeros(1,1), []};
    end

    [best_path, best_parks, best_works] = best_result{:};
end

% =====================================================================
%  钂欑壒鍗℃礇楠岃瘉
% =====================================================================
function mdp_montecarlo(cfg, N, K)
    if nargin < 3, K = 20; end
    fprintf('========== MDP MC楠岃瘉 (N=%d, K=%d) ==========\n', N, K);

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
            fprintf('  %d/%d (%.0f%%) | %.1fs | 鎴愬姛鐜? %.1f%%\n', ...
                sim, N, 100*sim/N, toc, 100*sum(success(1:sim))/sim);
        end
    end

    n_success = sum(success);
    fprintf('\n瀹屾垚 %.1fs | 鎴愬姛鐜? %d/%d (%.1f%%)\n', toc, n_success, N, 100*n_success/N);
    if n_success > 0
        Zs = Z_results(success); Ms = M_results(success);
        fprintf('Z: mean=%.0f std=%.0f min=%d max=%d\n', mean(Zs), std(Zs), min(Zs), max(Zs));
        fprintf('M: mean=%.0f std=%.0f min=%.0f max=%.0f\n', mean(Ms), std(Ms), min(Ms), max(Ms));
    end
    if N - n_success > 0
        fprintf('\n--- 澶辫触鍘熷洜 ---\n');
        fr = fail_reason(~success);
        [u, ~, ic] = unique(fr); cnts = accumarray(ic, 1);
        for i = 1:length(u), fprintf('  %s: %d\n', u{i}, cnts(i)); end
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
                state.pt, state.day-1, init_s, cfg, cons_exp, cons_N, cons_T, K);
            if isempty(plan_path), reason='MDP鏃犲彲琛屽姩浣?'; break; end
            plan_leg=1; step_in_leg=0; parked_in_leg=0; wp_idx=0; wd_done=0;
        end

        if length(plan_path)<2, reason='绌鸿矾寰?'; break; end
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
                    [needO, needH, needF] = cp_engine_opt('get_supply_needs', ...
                        plan_path, plan_parks, plan_works, plan_leg, cons_exp, cfg, wp_idx);
                    sp=cfg.MAX_LOAD-(state.O+state.H+state.F);
                    bO=max(0,needO-state.O); bH=max(0,needH-state.H); bF=max(0,needF-state.F);
                    if bO+bH+bF>sp+1e-6, sc=sp/(bO+bH+bF); bO=bO*sc; bH=bH*sc; bF=bF*sc; end
                    cost=bO*cons_act.pO+bH*cons_act.pH+bF*cons_act.pF;
                    if cost<=state.M+1e-6
                        state.O=state.O+bO; state.H=state.H+bH; state.F=state.F+bF; state.M=state.M-cost;
                    else, reason='琛ョ粰璧勯噾涓嶈冻'; break;
                    end
                end
                plan_leg=plan_leg+1;
            end
        end
        if state.O<-1e-6||state.H<-1e-6||state.F<-1e-6, reason='璧勬簮鑰楀敖'; break; end
        if state.O+state.H+state.F>cfg.MAX_LOAD+1e-6, reason='瓒呰浇'; break; end
    end

    if isempty(reason)&&state.pt==2, arrived=true; days=state.day;
    elseif isempty(reason)&&state.day>=cfg.MAX_DAYS, reason='瓒呮椂'; arrived=false; days=cfg.MAX_DAYS;
    else, arrived=false; days=state.day;
    end
    Zf=state.Z; Mf=state.M;
    if ~arrived, Zf=0; Mf=0; end
end
