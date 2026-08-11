function solve_cg_q2()
% =========================================================================
% solve_cg_q2.m — 问题二：全雷暴极端天气下最优航行方案
% 基于列生成(Column Generation) + 停泊-采购优化策略
% 2026年东南大学大学生数学建模竞赛 B题
% =========================================================================
% 核心改进 (经审题修正):
%   1. 补给站到达后可先停泊若干天，消耗H/F腾出载重空间，再采购
%   2. 时间线: 到达→停泊(消耗资源)→采购(补齐余量)→下一段航行
%   3. 停泊天数通过枚举0~5天搜索最优解
%
% 正确最优解:
%   Z=100, M=116, 路径 B -> S1 -> E, 12天
%   策略: B处停泊1天 → B->S1(3天) → S1采购(补O:56,H:2,F:5) → S1->E(8天)
% =========================================================================

    B = [1, 5];  E = [10, 5];  MAX_DAYS = 30;
    CM = [8, 4, 3];  CW = [8, 6, 6];  CI = [3, 3, 2];
    WY = [20, 15, 28];  WM = [4, 5, 3];

    all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
    names  = {'B','E','W1','W2','W3','S1','S2'};

    dist = zeros(7);
    for i = 1:7
        for j = 1:7
            dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) ...
                      + abs(all_xy(i,2)-all_xy(j,2));
        end
    end

    inter_idx = [3 4 5 6 7];  n_inter = 5;
    max_seq = min(7, MAX_DAYS - dist(1,2));

    fprintf('========================================\n');
    fprintf('  问题二：全雷暴极端天气下最优航行方案\n');
    fprintf('  基于列生成 + 停泊-采购优化策略\n');
    fprintf('========================================\n');
    fprintf('雷暴: 移(%d,%d,%d) 作(%d,%d,%d) 停(%d,%d,%d)\n', ...
            CM(1),CM(2),CM(3), CW(1),CW(2),CW(3), CI(1),CI(2),CI(3));
    fprintf('连续上限: W1=%d W2=%d W3=%d  载重<=120\n', WM(1),WM(2),WM(3));
    fprintf('初始: O=35 H=45 F=30 M=240 Z=100\n\n');

    poolP = {};  poolW = {};  poolI = {};
    poolZ = [];  poolM = [];
    bZ = -inf;  bM = -inf;  bP = [1,2];  bW = [];  bI = [];
    bLog = struct();  nit = 0;  ncg = 0;

    %% 种子列: B -> S1 -> E
    pid0 = [1, 6, 2];  m0 = 1;
    tr0 = [dist(1,6), dist(6,2)];  tt0 = sum(tr0);
    [ok, Z0, M0, idle0, dlog0] = enhanced_gsim(pid0, m0, tr0, [], [], ...
                                               tt0, [], all_xy, names, WM);
    if ok
        poolP{1} = pid0;  poolW{1} = [];  poolI{1} = idle0;
        poolZ(1) = Z0;  poolM(1) = M0;
        bZ = Z0;  bM = M0;  bP = pid0;  bI = idle0;  bLog = dlog0;
    else
        fprintf('ERROR: B->S1->E 不可行!\n');  return;
    end

    fprintf('Iter |  pi(dual) |  New Z  |  New M  | Idle@Sup | Cols\n');
    fprintf('-----|-----------|---------|---------|----------|------\n');

    %% 定价迭代
    while true
        [pi, idx] = max(poolZ);
        if pi > bZ || (pi == bZ && poolM(idx) > bM)
            bZ = pi;  bM = poolM(idx);
            bP = poolP{idx};  bW = poolW{idx};  bI = poolI{idx};
        end
        found = false;  nit = nit + 1;

        for sl = 0:max_seq
            if found, break; end
            ns = n_inter^sl;
            for si = 1:ns
                if found, break; end
                s = zeros(1,sl);  t = si-1;
                for j = sl:-1:1, s(j) = mod(t,n_inter)+1; t = floor(t/n_inter); end
                pid = [1, inter_idx(s), 2];
                dup = false;
                for k = 2:length(pid)
                    if pid(k)==pid(k-1), dup = true; break; end
                end
                if dup, continue; end
                m = length(pid)-2;  tr = zeros(1,m+1);  tt = 0;
                for k = 1:m+1, tr(k)=dist(pid(k),pid(k+1)); tt = tt+tr(k); end
                if tt > MAX_DAYS, continue; end

                wa = [];  ww = [];
                for k = 2:m+1
                    pt = pid(k);
                    if pt>=3 && pt<=5, wa(end+1)=k; ww(end+1)=pt-2; end
                end
                nw = length(wa);

                if nw == 0
                    [ok, Z, M, idle, dlog] = enhanced_gsim(pid, m, tr, ...
                        wa, ww, tt, [], all_xy, names, WM);
                    if ok && Z > pi
                        poolP{end+1}=pid; poolW{end+1}=[]; poolI{end+1}=idle;
                        poolZ(end+1)=Z; poolM(end+1)=M; ncg=ncg+1;
                        if Z>bZ || (Z==bZ && M>bM)
                            bZ=Z; bM=M; bP=pid; bW=[]; bI=idle; bLog=dlog;
                        end
                        fprintf(' %3d | %9d | %7d | %7d | [%s] | %5d\n', ...
                            nit, pi, Z, M, mat2str(idle), length(poolZ));
                        found = true;
                    end
                else
                    avail = MAX_DAYS - tt;
                    sz = zeros(1,nw);
                    for jj = 1:nw
                        sz(jj) = max(WM(ww(jj))+1, min(WM(ww(jj))*3, avail)+1);
                    end
                    nc = prod(sz);
                    for ci = 1:nc
                        if found, break; end
                        wd = zeros(1,nw);  t2 = ci-1;
                        for j = nw:-1:1, wd(j)=mod(t2,sz(j)); t2=floor(t2/sz(j)); end
                        cal_days = 0;
                        for jj = 1:nw
                            cal_days = cal_days + work_cal_days(wd(jj), WM(ww(jj)));
                        end
                        if tt + cal_days > MAX_DAYS, continue; end
                        [ok, Z, M, idle, dlog] = enhanced_gsim(pid, m, tr, ...
                            wa, ww, tt, wd, all_xy, names, WM);
                        if ok && Z > pi
                            poolP{end+1}=pid; poolW{end+1}=wd; poolI{end+1}=idle;
                            poolZ(end+1)=Z; poolM(end+1)=M; ncg=ncg+1;
                            if Z>bZ || (Z==bZ && M>bM)
                                bZ=Z; bM=M; bP=pid; bW=wd; bI=idle; bLog=dlog;
                            end
                            fprintf(' %3d | %9d | %7d | %7d | [%s] | %5d\n', ...
                                nit, pi, Z, M, mat2str(idle), length(poolZ));
                            found = true;
                        end
                    end
                end
            end
        end
        if ~found, break; end
    end

    fprintf('-----|-----------|---------|---------|----------|------\n');

    %% 输出最优解
    fprintf('\n===== 最优解 (全雷暴极端天气) =====\n');
    fprintf('Z (目标物资) = %d\n', bZ);
    fprintf('M (剩余资金) = %d\n', bM);
    fprintf('路径: ');
    for i = 1:length(bP), fprintf('%s ', names{bP(i)}); end; fprintf('\n');

    tt = 0;
    for k = 1:length(bP)-1, tt = tt + dist(bP(k),bP(k+1)); end

    if ~isempty(bI) && any(bI > 0)
        fprintf('补给站停泊策略: ');
        sup_idx = 1;
        for k = 2:length(bP)
            pt = bP(k);
            if (pt==6||pt==7) && sup_idx<=length(bI) && bI(sup_idx)>0
                fprintf('%s:停%d天 ', names{pt}, bI(sup_idx));
                sup_idx = sup_idx + 1;
            end
        end
        fprintf('\n');
    end

    if ~isempty(bW) && any(bW > 0)
        wp_idx = 1;  wd_str = '';
        for k = 2:length(bP)
            pt = bP(k);
            if pt>=3 && pt<=5 && bW(wp_idx)>0
                W=bW(wp_idx); Mlim=WM(pt-2);
                if W<=Mlim
                    wd_str = [wd_str sprintf('%s:%dd ', names{pt}, W)];
                else
                    nb = ceil(W/Mlim);
                    wd_str = [wd_str sprintf('%s:%d(%dx%d+停) ', ...
                        names{pt}, W, nb-1, Mlim)];
                end
                wp_idx = wp_idx + 1;
            elseif pt>=3 && pt<=5
                wp_idx = wp_idx + 1;
            end
        end
        if ~isempty(wd_str), fprintf('作业: %s\n', wd_str); end
    else
        fprintf('作业: 无\n');
    end

    fprintf('旅行: %d天\n', tt);
    fprintf('生成列数: %d\n', ncg);
    fprintf('定价迭代: %d\n', nit);

    %% 每日航行日志
    [m_opt, tr_opt, wa_opt, ww_opt, tt_opt] = buildPathParams(bP, bW, dist);
    [~, ~, ~, ~, finalLog] = enhanced_gsim(bP, m_opt, tr_opt, wa_opt, ...
        ww_opt, tt_opt, bW, all_xy, names, WM);

    fprintf('\n===== 每日航行日志 (T=雷暴) =====\n');
    fprintf('Day | Pos    | Action                        |   O   H   F  Load |     M |     Z\n');
    fprintf('----|--------|-------------------------------|-------------------|-------|------\n');
    for d = 1:length(finalLog)
        fprintf('%3d | (%2d,%2d) | %-30s | %3d %3d %3d  %3d | %5d | %5d\n', ...
            finalLog(d).day, finalLog(d).x, finalLog(d).y, finalLog(d).action, ...
            finalLog(d).O, finalLog(d).H, finalLog(d).F, ...
            finalLog(d).O+finalLog(d).H+finalLog(d).F, ...
            finalLog(d).M, finalLog(d).Z);
    end

    fprintf('\n===== 采购记录 =====\n');
    for d = 1:length(finalLog)
        if contains(finalLog(d).action, 'Buy')
            fprintf('Day %2d: %s\n', finalLog(d).day, finalLog(d).action);
        end
    end
    fprintf('总天数: %d\n', length(finalLog));
    fprintf('\n===== 完成 =====\n');
end

%% ===== enhanced_gsim: 含补给站停泊优化的贪婪前向模拟 =====
function [f, Zf, Mf, best_idle, dailyLog] = enhanced_gsim(pid, m, travel, ...
        wa, ww, tt, wdays, all_xy, names, WM)

    % 收集补给站在路径中的段索引
    sup_segs = [];
    for k = 1:m+1
        to_pt = pid(k+1);
        if to_pt == 6 || to_pt == 7, sup_segs(end+1) = k; end
    end
    n_sup = length(sup_segs);

    max_idle_each = 5;
    best_final_M = -inf;
    best_idle = zeros(1, n_sup);
    best_Z = 0;
    best_log = [];

    % 枚举各补给站停泊天数组合
    total_combos = (max_idle_each + 1)^n_sup;
    for ci = 1:total_combos
        idle_vec = zeros(1, n_sup);  t = ci - 1;
        for j = n_sup:-1:1
            idle_vec(j) = mod(t, max_idle_each + 1);
            t = floor(t / (max_idle_each + 1));
        end

        % 检查天数上限
        total_idle = sum(idle_vec);
        total_work = 0;
        if ~isempty(wdays)
            for jj = 1:length(wdays)
                total_work = total_work + work_cal_days(wdays(jj), WM(ww(jj)));
            end
        end
        if tt + total_idle + total_work > 30, continue; end

        [ok, Z, M, dlog] = gsim_idle_first(pid, m, travel, wa, ww, tt, ...
            wdays, idle_vec, sup_segs, all_xy, names, WM);

        if ok
            if Z > best_Z || (Z == best_Z && M > best_final_M)
                best_Z = Z;  best_final_M = M;
                best_idle = idle_vec;  best_log = dlog;
            end
        end
    end

    if best_final_M > -inf
        f = true;  Zf = best_Z;  Mf = best_final_M;  dailyLog = best_log;
    else
        f = false;  Zf = 0;  Mf = 0;  dailyLog = [];
    end
end

%% ===== gsim_idle_first: 停泊先于采购的贪婪模拟 =====
function [f, Zf, Mf, dailyLog] = gsim_idle_first(pid, m, travel, wa, ww, ...
        tt, wdays, idle_vec, sup_segs, all_xy, names, WM)

    % 计算总天数
    T = tt + sum(idle_vec);
    if ~isempty(wdays)
        for jj = 1:length(wdays)
            T = T + work_cal_days(wdays(jj), WM(ww(jj)));
        end
    end

    cO = zeros(1, T);  cH = zeros(1, T);  cF = zeros(1, T);
    zG = zeros(1, T);
    isSup = false(1, T);
    actType = cell(1, T);
    posX = zeros(1, T);  posY = zeros(1, T);

    day = 0;
    curX = all_xy(pid(1), 1);  curY = all_xy(pid(1), 2);
    sup_cnt = 0;  % 已处理的补给站数

    for k = 1:m+1
        fromX = all_xy(pid(k), 1);  fromY = all_xy(pid(k), 2);
        toX   = all_xy(pid(k+1), 1);  toY   = all_xy(pid(k+1), 2);
        dx = toX - fromX;  dy = toY - fromY;
        stepsX = abs(dx);  stepsY = abs(dy);
        sx = sign(dx);  if sx == 0, sx = 0; end
        sy = sign(dy);  if sy == 0, sy = 0; end

        d = travel(k);
        for dd = 1:d
            day = day + 1;
            cO(day) = 8;  cH(day) = 4;  cF(day) = 3;
            if dd <= stepsX
                curX = curX + sx;
                if sx > 0,      actType{day} = '> E (T)';
                elseif sx < 0,  actType{day} = '< W (T)';
                end
            else
                curY = curY + sy;
                if sy > 0,      actType{day} = '^ N (T)';
                elseif sy < 0,  actType{day} = 'v S (T)';
                end
            end
            posX(day) = curX;  posY(day) = curY;
        end

        % 到达补给站后: 先停泊, 再标记采购日
        sup_idx = find(sup_segs == k, 1);
        if ~isempty(sup_idx)
            sup_cnt = sup_cnt + 1;
            n_idle = idle_vec(sup_cnt);

            % 停泊日
            for id = 1:n_idle
                day = day + 1;
                cO(day) = 3;  cH(day) = 3;  cF(day) = 2;
                zG(day) = 0;
                posX(day) = curX;  posY(day) = curY;
                actType{day} = sprintf('Idle %s (T)', names{pid(k+1)});
            end

            % 停泊后标记为采购日
            isSup(day) = true;
        end

        % 作业 (含停泊分块)
        if ~isempty(wa)
            wk = find(wa == k+1, 1);
            if ~isempty(wk) && wdays(wk) > 0
                W = wdays(wk);  Mlim = WM(ww(wk));
                yv = [20, 15, 28];  yv = yv(ww(wk));
                if W <= Mlim
                    for w = 1:W
                        day = day + 1;
                        cO(day) = 8;  cH(day) = 6;  cF(day) = 6;
                        zG(day) = yv;
                        posX(day) = curX;  posY(day) = curY;
                        actType{day} = sprintf('Work %s (T)', names{pid(k+1)});
                    end
                else
                    nblocks = ceil(W / Mlim);  remaining = W;
                    for blk = 1:nblocks
                        bs = min(Mlim, remaining);
                        for w = 1:bs
                            day = day + 1;
                            cO(day) = 8;  cH(day) = 6;  cF(day) = 6;
                            zG(day) = yv;
                            posX(day) = curX;  posY(day) = curY;
                            actType{day} = sprintf('Work %s (T)', names{pid(k+1)});
                        end
                        remaining = remaining - bs;
                        if remaining > 0
                            day = day + 1;
                            cO(day) = 3;  cH(day) = 3;  cF(day) = 2;
                            zG(day) = 0;
                            posX(day) = curX;  posY(day) = curY;
                            actType{day} = sprintf('Idle %s (T)', names{pid(k+1)});
                        end
                    end
                end
            end
        end
    end

    %% 前向模拟
    O = 35;  H = 45;  F = 30;  M = 240;  Zcur = 100;
    dailyLog = struct('day', num2cell(1:T), 'x', num2cell(posX), ...
        'y', num2cell(posY), 'action', actType, ...
        'O', [], 'H', [], 'F', [], 'M', [], 'Z', []);

    for t = 1:T
        O = O - cO(t);  H = H - cH(t);  F = F - cF(t);
        if O < 0 || H < 0 || F < 0
            f = false;  Zf = 0;  Mf = 0;  dailyLog = [];  return;
        end

        if isSup(t)
            % 找到下一补给站位置
            ns = T + 1;
            for tt2 = t+1:T
                if isSup(tt2), ns = tt2; break; end
            end
            % 计算到下一站/终点的资源需求
            nO=0; nH=0; nF=0;
            for tt2 = t+1:ns
                if tt2 > T, break; end
                nO = nO + cO(tt2);  nH = nH + cH(tt2);  nF = nF + cF(tt2);
            end

            sp = 120 - (O + H + F);
            bO = max(0, nO - O);  bH = max(0, nH - H);  bF = max(0, nF - F);

            if bO + bH + bF > sp
                f = false;  Zf = 0;  Mf = 0;  dailyLog = [];  return;
            end
            if ns > T && (O + bO < nO || H + bH < nH || F + bF < nF)
                f = false;  Zf = 0;  Mf = 0;  dailyLog = [];  return;
            end

            cost = bO * 2 + bH * 1 + bF * 2;
            if cost > M
                f = false;  Zf = 0;  Mf = 0;  dailyLog = [];  return;
            end

            O = O + bO;  H = H + bH;  F = F + bF;  M = M - cost;
            if bO + bH + bF > 0
                actType{t} = [actType{t} sprintf(' Buy O:%d H:%d F:%d', bO, bH, bF)];
            else
                actType{t} = [actType{t} ' NoBuy'];
            end
        end

        if O + H + F > 120
            f = false;  Zf = 0;  Mf = 0;  dailyLog = [];  return;
        end

        Zcur = Zcur + zG(t);
        dailyLog(t).action = actType{t};
        dailyLog(t).O = O;  dailyLog(t).H = H;  dailyLog(t).F = F;
        dailyLog(t).M = M;  dailyLog(t).Z = Zcur;
    end

    Zf = Zcur;  Mf = M;  f = true;
end

%% ===== 辅助函数 =====
function cal = work_cal_days(W, M)
    if W <= M, cal = W;
    else, cal = W + (ceil(W / M) - 1); end
end

function [m, tr, wa, ww, tt] = buildPathParams(pid, wd, dist)
    m = length(pid) - 2;  tr = zeros(1, m+1);  tt = 0;
    for k = 1:m+1, tr(k)=dist(pid(k),pid(k+1)); tt = tt+tr(k); end
    wa = [];  ww = [];
    for k = 2:m+1
        pt = pid(k);
        if pt>=3 && pt<=5, wa(end+1)=k; ww(end+1)=pt-2; end
    end
end
