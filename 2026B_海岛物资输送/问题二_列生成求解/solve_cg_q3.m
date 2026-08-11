function solve_cg_q3()
% =========================================================================
% solve_cg_q3.m - 问题三：天气随机先验已知下的最优航行规划
% 方法: 列生成(期望消耗率) + 蒙特卡洛验证 + 天气自适应策略
% 2026年东南大学大学生数学建模竞赛 B题
% =========================================================================
% 30x30网格, 90天, P(正常)=0.8, P(雷暴)=0.2
% B(1,15), E(30,15), S1(12,16), S2(21,16)
% W1(6,21), W2(15,9), W3(24,24)
% O0=100,H0=150,F0=100,M0=750,Z0=200, 载重上限=400
% =========================================================================

    %% 全局参数
    B=[1,15]; E=[30,15]; MAX_DAYS=90; LOAD=400;
    O0=100; H0=150; F0=100; M0=750; Z0=200;

    % 消耗率: 正常, 雷暴, 期望(0.8*N + 0.2*T)
    CM=[3.2,3.2,2.2]; CW=[5.6,4.4,3.6]; CI=[1.4,1.4,1.2];
    % 实际消耗率(用于蒙特卡洛)
    CM_N=[2,3,2]; CW_N=[5,4,3]; CI_N=[1,1,1];
    CM_T=[8,4,3]; CW_T=[8,6,6]; CI_T=[3,3,2];

    WY=[20,15,28]; WM=[4,5,3];  % 收益, 连续上限

    all_xy=[1 15;30 15;6 21;15 9;24 24;12 16;21 16];
    names={'B','E','W1','W2','W3','S1','S2'};

    dist=zeros(7);
    for i=1:7, for j=1:7
        dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));
    end; end

    inter_idx=[3 4 5 6 7]; n_inter=5;
    max_seq=min(6, floor((MAX_DAYS-15)/min(dist(dist>0))));

    fprintf('========================================\n');
    fprintf('  问题三：天气随机条件下最优航行规划\n');
    fprintf('  列生成 + 期望消耗率 + 蒙特卡洛验证\n');
    fprintf('========================================\n');
    fprintf('网格:30x30 天数:%d 载重:%d\n',MAX_DAYS,LOAD);
    fprintf('初始:O=%d H=%d F=%d M=%d Z=%d\n',O0,H0,F0,M0,Z0);
    fprintf('期望消耗:移(%.1f,%.1f,%.1f) 作(%.1f,%.1f,%.1f)\n',CM(1),CM(2),CM(3),CW(1),CW(2),CW(3));

    %% 阶段1: 列生成
    fprintf('\n--- 阶段1: 列生成(期望消耗率) ---\n');
    poolP={}; poolW={}; poolI={}; poolZ=[]; poolM=[];
    bZ=-inf; bM=-inf; bP=[1,2]; bW=[]; bI=[]; nit=0; ncg=0;

    pid_seed=[1,6,7,2]; m_seed=2;
    tr_seed=[dist(1,6),dist(6,7),dist(7,2)]; tt_seed=sum(tr_seed);

    fprintf('种子: B->S1->S2->E, 旅行=%d天\n',tt_seed);
    [ok,Z0,M0,idle0,~]=egsim(pid_seed,m_seed,tr_seed,[],[],tt_seed,[],all_xy,names,WM);
    if ok, poolP{1}=pid_seed;poolW{1}=[];poolI{1}=idle0;poolZ(1)=Z0;poolM(1)=M0;
        bZ=Z0;bM=M0;bP=pid_seed;bI=idle0;
        fprintf('种子可行: Z=%d M=%d\n',Z0,M0);
    else, fprintf('ERROR!\n'); return; end

    fprintf('Iter |    pi   | New Z  | New M  | Idle@Sup | Cols\n');
    fprintf('-----|---------|--------|--------|----------|------\n');

    max_iter=60; t0=tic;
    while nit<max_iter
        [pi,idx]=max(poolZ);
        if pi>bZ||(pi==bZ&&poolM(idx)>bM)
            bZ=pi;bM=poolM(idx);bP=poolP{idx};bW=poolW{idx};bI=poolI{idx};
        end
        found=false; nit=nit+1;

        for sl=0:max_seq
            if found, break; end
            ns=n_inter^sl; if ns>30000, ns=30000; end
            for si=1:ns
                if found, break; end
                s=zeros(1,sl); t=si-1;
                for j=sl:-1:1, s(j)=mod(t,n_inter)+1; t=floor(t/n_inter); end
                pid=[1,inter_idx(s),2];
                dup=false;
                for k=2:length(pid), if pid(k)==pid(k-1), dup=true; break; end; end
                if dup, continue; end
                m=length(pid)-2; tr=zeros(1,m+1); tt=0;
                for k=1:m+1, tr(k)=dist(pid(k),pid(k+1)); tt=tt+tr(k); end
                if tt>MAX_DAYS-5, continue; end

                wa=[]; ww=[];
                for k=2:m+1
                    pt=pid(k);
                    if pt>=3&&pt<=5, wa(end+1)=k; ww(end+1)=pt-2; end
                end
                nw=length(wa);

                if nw==0
                    [ok,Z,M,idle,~]=egsim(pid,m,tr,wa,ww,tt,[],all_xy,names,WM);
                    if ok&&Z>pi
                        poolP{end+1}=pid;poolW{end+1}=[];poolI{end+1}=idle;
                        poolZ(end+1)=Z;poolM(end+1)=M;ncg=ncg+1;
                        if Z>bZ||(Z==bZ&&M>bM), bZ=Z;bM=M;bP=pid;bW=[];bI=idle; end
                        fprintf(' %3d | %7d | %6d | %6d | [%s] | %4d\n',nit,pi,Z,M,mat2str(idle),length(poolZ));
                        found=true;
                    end
                else
                    avail=MAX_DAYS-tt;
                    sz=zeros(1,nw);
                    for jj=1:nw
                        sz(jj)=max(WM(ww(jj))+1, min(WM(ww(jj))*3, avail)+1);
                    end
                    nc=prod(sz); if nc>3000, continue; end
                    for ci=1:nc
                        if found, break; end
                        wd=zeros(1,nw); t2=ci-1;
                        for j=nw:-1:1, wd(j)=mod(t2,sz(j)); t2=floor(t2/sz(j)); end
                        cal=0;
                        for jj=1:nw, cal=cal+wcal(wd(jj),WM(ww(jj))); end
                        if tt+cal>MAX_DAYS, continue; end
                        [ok,Z,M,idle,~]=egsim(pid,m,tr,wa,ww,tt,wd,all_xy,names,WM);
                        if ok&&Z>pi
                            poolP{end+1}=pid;poolW{end+1}=wd;poolI{end+1}=idle;
                            poolZ(end+1)=Z;poolM(end+1)=M;ncg=ncg+1;
                            if Z>bZ||(Z==bZ&&M>bM), bZ=Z;bM=M;bP=pid;bW=wd;bI=idle; end
                            fprintf(' %3d | %7d | %6d | %6d | [%s] | %4d\n',nit,pi,Z,M,mat2str(idle),length(poolZ));
                            found=true;
                        end
                    end
                end
            end
        end
        if ~found, break; end
    end

    fprintf('-----|---------|--------|--------|----------|------\n');
    fprintf('列生成: %.1f秒, %d列, %d迭代\n\n',toc(t0),ncg,nit);

    %% 输出阶段1结果
    fprintf('===== 期望消耗率最优方案 =====\n');
    fprintf('Z=%d  M=%d\n',bZ,bM);
    fprintf('路径: '); for i=1:length(bP), fprintf('%s ',names{bP(i)}); end; fprintf('\n');
    tt=0; for k=1:length(bP)-1, tt=tt+dist(bP(k),bP(k+1)); end
    fprintf('旅行:%d天',tt);

    if ~isempty(bI)&&any(bI>0)
        fprintf('  停泊:');
        si=1;
        for k=2:length(bP)
            pt=bP(k);
            if (pt==6||pt==7)&&si<=length(bI)&&bI(si)>0
                fprintf(' %s:%dd',names{pt},bI(si)); si=si+1;
            end
        end
    end
    fprintf('\n');

    if ~isempty(bW)&&any(bW>0)
        wp=1; ws='';
        for k=2:length(bP)
            pt=bP(k);
            if pt>=3&&pt<=5&&bW(wp)>0
                W=bW(wp); M=WM(pt-2);
                if W<=M, ws=[ws sprintf('%s:%dd ',names{pt},W)];
                else, nb=ceil(W/M); ws=[ws sprintf('%s:%d(%dx%d+停) ',names{pt},W,nb-1,M)]; end
                wp=wp+1;
            elseif pt>=3&&pt<=5, wp=wp+1; end
        end
        if ~isempty(ws), fprintf('作业:%s\n',ws); end
    else, fprintf('作业:无\n'); end

    %% 每日日志
    [m_opt,tr_opt,wa_opt,ww_opt,tt_opt]=bp(bP,bW,dist);
    [~,~,~,~,finalLog]=egsim(bP,m_opt,tr_opt,wa_opt,ww_opt,tt_opt,bW,all_xy,names,WM);

    fprintf('\n===== 期望方案每日日志(前15天) =====\n');
    fprintf('Day| Pos     | Action              |  O  H  F Load|   M |  Z\n');
    for d=1:min(15,length(finalLog))
        fprintf('%3d|(%2d,%2d)  |%-21s|%3d%3d%3d %3d|%5d|%5d\n',...
            finalLog(d).day,finalLog(d).x,finalLog(d).y,finalLog(d).action,...
            finalLog(d).O,finalLog(d).H,finalLog(d).F,...
            finalLog(d).O+finalLog(d).H+finalLog(d).F,...
            finalLog(d).M,finalLog(d).Z);
    end
    if length(finalLog)>15
        fprintf('... (省略 %d-%d天)\n',16,length(finalLog)-3);
        for d=length(finalLog)-2:length(finalLog)
            fprintf('%3d|(%2d,%2d)  |%-21s|%3d%3d%3d %3d|%5d|%5d\n',...
                finalLog(d).day,finalLog(d).x,finalLog(d).y,finalLog(d).action,...
                finalLog(d).O,finalLog(d).H,finalLog(d).F,...
                finalLog(d).O+finalLog(d).H+finalLog(d).F,...
                finalLog(d).M,finalLog(d).Z);
        end
    end
    fprintf('总天数:%d\n',length(finalLog));

    %% 阶段2: 蒙特卡洛验证
    fprintf('\n--- 阶段2: 蒙特卡洛鲁棒性验证(1000次) ---\n');
    n_mc=1000; succ=0; Z_mc=zeros(1,n_mc); M_mc=zeros(1,n_mc);
    rng(42);
    for mc=1:n_mc
        weather=rand(1,200)<0.8;  % true=normal, false=thunderstorm
        [ok,Zf,Mf]=mc_sim(bP,bW,bI,weather,dist,all_xy,names);
        if ok, succ=succ+1; Z_mc(mc)=Zf; M_mc(mc)=Mf; end
    end
    fprintf('成功率:%.1f%%(%d/%d)\n',succ/n_mc*100,succ,n_mc);
    if succ>0
        fprintf('成功时Z:均值%.0f 最小%d 最大%d\n',mean(Z_mc(Z_mc>0)),min(Z_mc(Z_mc>0)),max(Z_mc(Z_mc>0)));
        fprintf('成功时M:均值%.0f 最小%d 最大%d\n',mean(M_mc(M_mc>0)),min(M_mc(M_mc>0)),max(M_mc(M_mc>0)));
    end

    %% 阶段3: 天气自适应策略
    fprintf('\n--- 阶段3: 天气自适应策略 ---\n');
    fprintf('策略:正常日按计划执行,雷暴日停泊避让,补给日无论天气均采购\n');
    weather_ad=rand(1,200)<0.8;
    [ok_ad,Z_ad,M_ad,log_ad]=adaptive_sim(bP,bW,bI,weather_ad,dist,all_xy,names);
    if ok_ad
        fprintf('自适应示例:Z=%d M=%d 天数=%d\n',Z_ad,M_ad,length(log_ad));
    else
        fprintf('自适应示例:不可行\n');
    end

    fprintf('\n===== 完成 =====\n');
end

%% ===== egsim: 期望消耗率贪婪模拟 =====
function [f,Zf,Mf,best_idle,dailyLog]=egsim(pid,m,travel,wa,ww,tt,wdays,all_xy,names,WM)
    CM=[3.2,3.2,2.2]; CW=[5.6,4.4,3.6]; CI=[1.4,1.4,1.2];
    LOAD=400; O0=100;H0=150;F0=100;M0=750;Z0=200; WY=[20,15,28];

    sup_segs=[];
    for k=1:m+1
        to_pt=pid(k+1);
        if to_pt==6||to_pt==7, sup_segs(end+1)=k; end
    end
    n_sup=length(sup_segs);

    max_idle=5; best_M=-inf; best_idle=zeros(1,n_sup); best_Z=0; best_log=[];

    total_combos=(max_idle+1)^n_sup;
    for ci=1:total_combos
        idle_vec=zeros(1,n_sup); t=ci-1;
        for j=n_sup:-1:1, idle_vec(j)=mod(t,max_idle+1); t=floor(t/(max_idle+1)); end
        tw=0;
        if ~isempty(wdays)
            for jj=1:length(wdays), tw=tw+wcal(wdays(jj),WM(ww(jj))); end
        end
        if tt+sum(idle_vec)+tw>90, continue; end
        [ok,Z,M,dlog]=gsim_q3(pid,m,travel,wa,ww,tt,wdays,idle_vec,sup_segs,all_xy,names,WM);
        if ok
            if Z>best_Z||(Z==best_Z&&M>best_M)
                best_Z=Z;best_M=M;best_idle=idle_vec;best_log=dlog;
            end
        end
    end
    if best_M>-inf, f=true;Zf=best_Z;Mf=best_M;dailyLog=best_log;
    else, f=false;Zf=0;Mf=0;dailyLog=[]; end
end

%% ===== gsim_q3: 期望消耗率详细模拟 =====
function [f,Zf,Mf,dailyLog]=gsim_q3(pid,m,travel,wa,ww,tt,wdays,idle_vec,sup_segs,all_xy,names,WM)
    CM=[3.2,3.2,2.2]; CW=[5.6,4.4,3.6]; CI=[1.4,1.4,1.2];
    LOAD=400; O0=100;H0=150;F0=100;M0=750;Z0=200; WY=[20,15,28];

    T=tt+sum(idle_vec);
    if ~isempty(wdays)
        for jj=1:length(wdays), T=T+wcal(wdays(jj),WM(ww(jj))); end
    end

    cO=zeros(1,T);cH=zeros(1,T);cF=zeros(1,T);zG=zeros(1,T);
    isSup=false(1,T);actType=cell(1,T);posX=zeros(1,T);posY=zeros(1,T);

    day=0;curX=all_xy(pid(1),1);curY=all_xy(pid(1),2);sup_cnt=0;

    for k=1:m+1
        fromX=all_xy(pid(k),1);fromY=all_xy(pid(k),2);
        toX=all_xy(pid(k+1),1);toY=all_xy(pid(k+1),2);
        dx=toX-fromX;dy=toY-fromY;stepsX=abs(dx);stepsY=abs(dy);
        sx=sign(dx);if sx==0,sx=0;end; sy=sign(dy);if sy==0,sy=0;end

        d=travel(k);
        for dd=1:d
            day=day+1;cO(day)=CM(1);cH(day)=CM(2);cF(day)=CM(3);
            if dd<=stepsX, curX=curX+sx;
                if sx>0, actType{day}='> E'; elseif sx<0, actType{day}='< W'; end
            else, curY=curY+sy;
                if sy>0, actType{day}='^ N'; elseif sy<0, actType{day}='v S'; end
            end
            posX(day)=curX;posY(day)=curY;
        end

        sup_idx=find(sup_segs==k,1);
        if ~isempty(sup_idx)
            sup_cnt=sup_cnt+1;n_idle=idle_vec(sup_cnt);
            for id=1:n_idle
                day=day+1;cO(day)=CI(1);cH(day)=CI(2);cF(day)=CI(3);
                zG(day)=0;posX(day)=curX;posY(day)=curY;
                actType{day}=sprintf('Idle %s',names{pid(k+1)});
            end
            isSup(day)=true;
        end

        if ~isempty(wa)
            wk=find(wa==k+1,1);
            if ~isempty(wk)&&wdays(wk)>0
                W=wdays(wk);Mlim=WM(ww(wk));yv=WY(ww(wk));
                if W<=Mlim
                    for w=1:W
                        day=day+1;cO(day)=CW(1);cH(day)=CW(2);cF(day)=CW(3);
                        zG(day)=yv;posX(day)=curX;posY(day)=curY;
                        actType{day}=sprintf('Work %s',names{pid(k+1)});
                    end
                else
                    nblocks=ceil(W/Mlim);remaining=W;
                    for blk=1:nblocks
                        bs=min(Mlim,remaining);
                        for w=1:bs
                            day=day+1;cO(day)=CW(1);cH(day)=CW(2);cF(day)=CW(3);
                            zG(day)=yv;posX(day)=curX;posY(day)=curY;
                            actType{day}=sprintf('Work %s',names{pid(k+1)});
                        end
                        remaining=remaining-bs;
                        if remaining>0
                            day=day+1;cO(day)=CI(1);cH(day)=CI(2);cF(day)=CI(3);
                            zG(day)=0;posX(day)=curX;posY(day)=curY;
                            actType{day}=sprintf('Idle %s',names{pid(k+1)});
                        end
                    end
                end
            end
        end
    end

    O=O0;H=H0;F=F0;M=M0;Zcur=Z0;
    dailyLog=struct('day',num2cell(1:T),'x',num2cell(posX),'y',num2cell(posY),...
        'action',actType,'O',[],'H',[],'F',[],'M',[],'Z',[]);

    for t=1:T
        O=O-cO(t);H=H-cH(t);F=F-cF(t);
        if O<0||H<0||F<0, f=false;Zf=0;Mf=0;dailyLog=[];return; end
        if isSup(t)
            ns=T+1;
            for tt2=t+1:T, if isSup(tt2), ns=tt2;break;end;end
            nO=0;nH=0;nF=0;
            for tt2=t+1:ns, if tt2>T, break; end
                nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);
            end
            sp=LOAD-(O+H+F);
            bO=max(0,ceil(nO-O));bH=max(0,ceil(nH-H));bF=max(0,ceil(nF-F));
            if bO+bH+bF>sp, f=false;Zf=0;Mf=0;dailyLog=[];return;end
            if ns>T&&(O+bO<nO||H+bH<nH||F+bF<nF), f=false;Zf=0;Mf=0;dailyLog=[];return;end
            cost=bO*2+bH*1+bF*2;
            if cost>M, f=false;Zf=0;Mf=0;dailyLog=[];return;end
            O=O+bO;H=H+bH;F=F+bF;M=M-cost;
            if bO+bH+bF>0
                actType{t}=[actType{t} sprintf(' Buy O:%d H:%d F:%d',bO,bH,bF)];
            end
        end
        if O+H+F>LOAD, f=false;Zf=0;Mf=0;dailyLog=[];return;end
        Zcur=Zcur+zG(t);
        dailyLog(t).action=actType{t};
        dailyLog(t).O=round(O);dailyLog(t).H=round(H);dailyLog(t).F=round(F);
        dailyLog(t).M=round(M);dailyLog(t).Z=round(Zcur);
    end
    Zf=round(Zcur);Mf=round(M);f=true;
end

%% ===== mc_sim: 蒙特卡洛单次模拟 =====
function [f,Zf,Mf]=mc_sim(pid,wdays,idle_vec,weather,dist,all_xy,names)
    CM_N=[2,3,2];CW_N=[5,4,3];CI_N=[1,1,1];
    CM_T=[8,4,3];CW_T=[8,6,6];CI_T=[3,3,2];
    LOAD=400;O0=100;H0=150;F0=100;M0=750;Z0=200; WY=[20,15,28];WM=[4,5,3];
    MAX_DAYS=90;

    m=length(pid)-2;
    travel=zeros(1,m+1);tt=0;
    for k=1:m+1, travel(k)=dist(pid(k),pid(k+1));tt=tt+travel(k);end
    wa=[];ww=[];
    for k=2:m+1, pt=pid(k);
        if pt>=3&&pt<=5, wa(end+1)=k;ww(end+1)=pt-2;end
    end

    sup_segs=[];
    for k=1:m+1, to_pt=pid(k+1);
        if to_pt==6||to_pt==7, sup_segs(end+1)=k;end
    end
    n_sup=length(sup_segs);
    if isempty(idle_vec), idle_vec=zeros(1,n_sup); end
    if isempty(wdays)&&~isempty(wa), wdays=zeros(1,length(wa)); end

    T=tt+sum(idle_vec);
    if ~isempty(wdays)
        for jj=1:length(wdays), T=T+wcal(wdays(jj),WM(ww(jj)));end
    end

    cO=zeros(1,T);cH=zeros(1,T);cF=zeros(1,T);zG=zeros(1,T);
    isSup=false(1,T);actType=cell(1,T);posX=zeros(1,T);posY=zeros(1,T);
    wDay=zeros(1,T);  % 标记每日真实天气:1=正常,0=雷暴

    day=0;curX=all_xy(pid(1),1);curY=all_xy(pid(1),2);sup_cnt=0;

    for k=1:m+1
        fromX=all_xy(pid(k),1);fromY=all_xy(pid(k),2);
        toX=all_xy(pid(k+1),1);toY=all_xy(pid(k+1),2);
        dx=toX-fromX;dy=toY-fromY;stepsX=abs(dx);stepsY=abs(dy);
        sx=sign(dx);if sx==0,sx=0;end;sy=sign(dy);if sy==0,sy=0;end

        d=travel(k);
        for dd=1:d
            day=day+1;wd=weather(day);
            if wd
                cO(day)=CM_N(1);cH(day)=CM_N(2);cF(day)=CM_N(3);
            else
                cO(day)=CM_T(1);cH(day)=CM_T(2);cF(day)=CM_T(3);
            end
            wDay(day)=wd;
            if dd<=stepsX, curX=curX+sx;
                if sx>0, actType{day}='> E';elseif sx<0, actType{day}='< W';end
            else, curY=curY+sy;
                if sy>0, actType{day}='^ N';elseif sy<0, actType{day}='v S';end
            end
            if ~wd, actType{day}=[actType{day} '(T)']; end
            posX(day)=curX;posY(day)=curY;
        end

        sup_idx=find(sup_segs==k,1);
        if ~isempty(sup_idx)
            sup_cnt=sup_cnt+1;n_idle=idle_vec(sup_cnt);
            for id=1:n_idle
                day=day+1;wd=weather(day);wDay(day)=wd;
                if wd
                    cO(day)=CI_N(1);cH(day)=CI_N(2);cF(day)=CI_N(3);
                else
                    cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);
                end
                zG(day)=0;posX(day)=curX;posY(day)=curY;
                actType{day}=sprintf('Idle %s',names{pid(k+1)});
                if ~wd, actType{day}=[actType{day} '(T)']; end
            end
            isSup(day)=true;
        end

        if ~isempty(wa)
            wk=find(wa==k+1,1);
            if ~isempty(wk)&&~isempty(wdays)&&wdays(wk)>0
                W=wdays(wk);Mlim=WM(ww(wk));yv=WY(ww(wk));
                if W<=Mlim
                    for w=1:W
                        day=day+1;wd=weather(day);wDay(day)=wd;
                        if wd
                            cO(day)=CW_N(1);cH(day)=CW_N(2);cF(day)=CW_N(3);
                        else
                            cO(day)=CW_T(1);cH(day)=CW_T(2);cF(day)=CW_T(3);
                        end
                        zG(day)=yv;posX(day)=curX;posY(day)=curY;
                        actType{day}=sprintf('Work %s',names{pid(k+1)});
                        if ~wd, actType{day}=[actType{day} '(T)']; end
                    end
                else
                    nblocks=ceil(W/Mlim);remaining=W;
                    for blk=1:nblocks
                        bs=min(Mlim,remaining);
                        for w=1:bs
                            day=day+1;wd=weather(day);wDay(day)=wd;
                            if wd
                                cO(day)=CW_N(1);cH(day)=CW_N(2);cF(day)=CW_N(3);
                            else
                                cO(day)=CW_T(1);cH(day)=CW_T(2);cF(day)=CW_T(3);
                            end
                            zG(day)=yv;posX(day)=curX;posY(day)=curY;
                            actType{day}=sprintf('Work %s',names{pid(k+1)});
                            if ~wd, actType{day}=[actType{day} '(T)']; end
                        end
                        remaining=remaining-bs;
                        if remaining>0
                            day=day+1;wd=weather(day);wDay(day)=wd;
                            if wd
                                cO(day)=CI_N(1);cH(day)=CI_N(2);cF(day)=CI_N(3);
                            else
                                cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);
                            end
                            zG(day)=0;posX(day)=curX;posY(day)=curY;
                            actType{day}=sprintf('Idle %s',names{pid(k+1)});
                            if ~wd, actType{day}=[actType{day} '(T)']; end
                        end
                    end
                end
            end
        end
    end

    O=O0;H=H0;F=F0;M=M0;Zcur=Z0;

    for t=1:T
        O=O-cO(t);H=H-cH(t);F=F-cF(t);
        if O<0||H<0||F<0, f=false;Zf=0;Mf=0;return;end
        if isSup(t)
            ns=T+1;
            for tt2=t+1:T, if isSup(tt2), ns=tt2;break;end;end
            nO=0;nH=0;nF=0;
            for tt2=t+1:ns, if tt2>T, break;end
                nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);
            end
            sp=LOAD-(O+H+F);
            bO=max(0,ceil(nO-O));bH=max(0,ceil(nH-H));bF=max(0,ceil(nF-F));
            if bO+bH+bF>sp, f=false;Zf=0;Mf=0;return;end
            if ns>T&&(O+bO<nO||H+bH<nH||F+bF<nF), f=false;Zf=0;Mf=0;return;end
            cost=bO*2+bH*1+bF*2;
            if cost>M, f=false;Zf=0;Mf=0;return;end
            O=O+bO;H=H+bH;F=F+bF;M=M-cost;
        end
        if O+H+F>LOAD, f=false;Zf=0;Mf=0;return;end
        Zcur=Zcur+zG(t);
    end
    Zf=round(Zcur);Mf=round(M);f=true;
end

%% ===== adaptive_sim: 自适应策略模拟 =====
function [f,Zf,Mf,dailyLog]=adaptive_sim(pid,wdays,idle_vec,weather,dist,all_xy,names)
    CM_N=[2,3,2];CW_N=[5,4,3];CI_N=[1,1,1];
    CM_T=[8,4,3];CW_T=[8,6,6];CI_T=[3,3,2];
    LOAD=400;O0=100;H0=150;F0=100;M0=750;Z0=200; WY=[20,15,28];WM=[4,5,3];

    m=length(pid)-2;
    travel=zeros(1,m+1);tt=0;
    for k=1:m+1, travel(k)=dist(pid(k),pid(k+1));tt=tt+travel(k);end
    wa=[];ww=[];
    for k=2:m+1, pt=pid(k);
        if pt>=3&&pt<=5, wa(end+1)=k;ww(end+1)=pt-2;end
    end

    sup_segs=[];
    for k=1:m+1, to_pt=pid(k+1);
        if to_pt==6||to_pt==7, sup_segs(end+1)=k;end
    end
    n_sup=length(sup_segs);
    if isempty(idle_vec), idle_vec=zeros(1,n_sup);end
    if isempty(wdays)&&~isempty(wa), wdays=zeros(1,length(wa));end

    T=tt+sum(idle_vec);
    if ~isempty(wdays)
        for jj=1:length(wdays), T=T+wcal(wdays(jj),WM(ww(jj)));end
    end

    % 自适应:雷暴日停泊替代移动/作业,雷暴日停泊延长计划天数
    T=T+20;  % 预留缓冲

    cO=zeros(1,T);cH=zeros(1,T);cF=zeros(1,T);zG=zeros(1,T);
    isSup=false(1,T);actType=cell(1,T);posX=zeros(1,T);posY=zeros(1,T);

    day=0;curX=all_xy(pid(1),1);curY=all_xy(pid(1),2);sup_cnt=0;
    w_ptr=1;  % 天气索引

    for k=1:m+1
        fromX=all_xy(pid(k),1);fromY=all_xy(pid(k),2);
        toX=all_xy(pid(k+1),1);toY=all_xy(pid(k+1),2);
        dx=toX-fromX;dy=toY-fromY;stepsX=abs(dx);stepsY=abs(dy);
        sx=sign(dx);if sx==0,sx=0;end;sy=sign(dy);if sy==0,sy=0;end

        d=travel(k);
        dd=0;consec_skip=0;  % 连续跳过(雷暴)计数
        while dd<d
            day=day+1; w_ptr=w_ptr+1;
            if day>T, f=false;Zf=0;Mf=0;dailyLog=[];return;end
            wd=weather(min(w_ptr,length(weather)));

            if wd  % 正常:执行移动
                cO(day)=CM_N(1);cH(day)=CM_N(2);cF(day)=CM_N(3);
                dd=dd+1;consec_skip=0;
                if dd<=stepsX, curX=curX+sx;
                    if sx>0, actType{day}='> E';elseif sx<0, actType{day}='< W';end
                else, curY=curY+sy;
                    if sy>0, actType{day}='^ N';elseif sy<0, actType{day}='v S';end
                end
            else  % 雷暴:停泊替代移动
                cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);
                actType{day}='Idle(T-skip)';
                consec_skip=consec_skip+1;
                if consec_skip>3  % 连续雷暴>3天,强制执行移动
                    dd=dd+1;consec_skip=0;
                    cO(day)=CM_T(1);cH(day)=CM_T(2);cF(day)=CM_T(3);
                    if dd<=stepsX, curX=curX+sx;
                        actType{day}='> E(T-force)';
                    else, curY=curY+sy;
                        actType{day}='^ N(T-force)';
                    end
                end
            end
            posX(day)=curX;posY(day)=curY;
        end

        % 补给站:到达后停泊+采购
        sup_idx=find(sup_segs==k,1);
        if ~isempty(sup_idx)
            sup_cnt=sup_cnt+1;n_idle=idle_vec(sup_cnt);
            for id=1:n_idle
                day=day+1;w_ptr=w_ptr+1;wd=weather(min(w_ptr,length(weather)));
                if wd
                    cO(day)=CI_N(1);cH(day)=CI_N(2);cF(day)=CI_N(3);
                else
                    cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);
                end
                zG(day)=0;posX(day)=curX;posY(day)=curY;
                actType{day}=sprintf('Idle %s',names{pid(k+1)});
                if ~wd, actType{day}=[actType{day} '(T)'];end
            end
            isSup(day)=true;
        end

        % 作业:雷暴日停泊替代作业
        if ~isempty(wa)
            wk=find(wa==k+1,1);
            if ~isempty(wk)&&~isempty(wdays)&&wdays(wk)>0
                W=wdays(wk);Mlim=WM(ww(wk));yv=WY(ww(wk));
                if W<=Mlim
                    w_done=0;consec_skip=0;
                    while w_done<W
                        day=day+1;w_ptr=w_ptr+1;wd=weather(min(w_ptr,length(weather)));
                        if wd  % 正常:作业
                            cO(day)=CW_N(1);cH(day)=CW_N(2);cF(day)=CW_N(3);
                            zG(day)=yv;w_done=w_done+1;consec_skip=0;
                            actType{day}=sprintf('Work %s',names{pid(k+1)});
                        else  % 雷暴:停泊
                            cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);
                            zG(day)=0;consec_skip=consec_skip+1;
                            actType{day}=sprintf('Idle %s(T-skip)',names{pid(k+1)});
                            if consec_skip>3  % 连续雷暴>3天，强制作业一天
                                cO(day)=CW_T(1);cH(day)=CW_T(2);cF(day)=CW_T(3);
                                zG(day)=yv;w_done=w_done+1;consec_skip=0;
                                actType{day}=sprintf('Work %s(T-force)',names{pid(k+1)});
                            end
                        end
                        posX(day)=curX;posY(day)=curY;
                    end
                else
                    nblocks=ceil(W/Mlim);remaining=W;
                    for blk=1:nblocks
                        bs=min(Mlim,remaining);w_done=0;consec_skip=0;
                        while w_done<bs
                            day=day+1;w_ptr=w_ptr+1;wd=weather(min(w_ptr,length(weather)));
                            if wd
                                cO(day)=CW_N(1);cH(day)=CW_N(2);cF(day)=CW_N(3);
                                zG(day)=yv;w_done=w_done+1;consec_skip=0;
                                actType{day}=sprintf('Work %s',names{pid(k+1)});
                            else
                                cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);
                                zG(day)=0;consec_skip=consec_skip+1;
                                actType{day}=sprintf('Idle %s(T-skip)',names{pid(k+1)});
                                if consec_skip>3
                                    cO(day)=CW_T(1);cH(day)=CW_T(2);cF(day)=CW_T(3);
                                    zG(day)=yv;w_done=w_done+1;consec_skip=0;
                                    actType{day}=sprintf('Work %s(T-force)',names{pid(k+1)});
                                end
                            end
                            posX(day)=curX;posY(day)=curY;
                        end
                        remaining=remaining-bs;
                        if remaining>0  % 块间停泊
                            day=day+1;w_ptr=w_ptr+1;wd=weather(min(w_ptr,length(weather)));
                            if wd
                                cO(day)=CI_N(1);cH(day)=CI_N(2);cF(day)=CI_N(3);
                            else
                                cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);
                            end
                            zG(day)=0;posX(day)=curX;posY(day)=curY;
                            actType{day}=sprintf('Idle %s',names{pid(k+1)});
                            if ~wd, actType{day}=[actType{day} '(T)'];end
                        end
                    end
                end
            end
        end
    end

    T_actual=day;
    O=O0;H=H0;F=F0;M=M0;Zcur=Z0;
    dailyLog=struct('day',num2cell(1:T_actual),'x',num2cell(posX(1:T_actual)),...
        'y',num2cell(posY(1:T_actual)),'action',{actType{1:T_actual}},...
        'O',[],'H',[],'F',[],'M',[],'Z',[]);

    for t=1:T_actual
        O=O-cO(t);H=H-cH(t);F=F-cF(t);
        if O<0||H<0||F<0, f=false;Zf=0;Mf=0;dailyLog=[];return;end
        if isSup(t)
            ns=T_actual+1;
            for tt2=t+1:T_actual, if isSup(tt2), ns=tt2;break;end;end
            nO=0;nH=0;nF=0;
            for tt2=t+1:ns, if tt2>T_actual, break;end
                nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);
            end
            sp=LOAD-(O+H+F);
            bO=max(0,ceil(nO-O));bH=max(0,ceil(nH-H));bF=max(0,ceil(nF-F));
            if bO+bH+bF>sp, f=false;Zf=0;Mf=0;dailyLog=[];return;end
            if ns>T_actual&&(O+bO<nO||H+bH<nH||F+bF<nF), f=false;Zf=0;Mf=0;dailyLog=[];return;end
            cost=bO*2+bH*1+bF*2;
            if cost>M, f=false;Zf=0;Mf=0;dailyLog=[];return;end
            O=O+bO;H=H+bH;F=F+bF;M=M-cost;
        end
        if O+H+F>LOAD, f=false;Zf=0;Mf=0;dailyLog=[];return;end
        Zcur=Zcur+zG(t);
        dailyLog(t).O=round(O);dailyLog(t).H=round(H);dailyLog(t).F=round(F);
        dailyLog(t).M=round(M);dailyLog(t).Z=round(Zcur);
    end
    Zf=round(Zcur);Mf=round(M);f=true;
end

%% 辅助
function cal=wcal(W,M)
    if W<=M, cal=W; else, cal=W+(ceil(W/M)-1); end
end

function [m,tr,wa,ww,tt]=bp(pid,wd,dist)
    m=length(pid)-2;tr=zeros(1,m+1);tt=0;
    for k=1:m+1, tr(k)=dist(pid(k),pid(k+1));tt=tt+tr(k);end
    wa=[];ww=[];
    for k=2:m+1, pt=pid(k);
        if pt>=3&&pt<=5, wa(end+1)=k;ww(end+1)=pt-2;end
    end
end
