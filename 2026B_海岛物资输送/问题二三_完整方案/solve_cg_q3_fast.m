function solve_cg_q3_fast()
% 快速聚焦求解: 仅枚举关键路径 B->S1->W1->S1->S2->W3->S2->E 的变体
% 大幅缩减枚举空间

    B=[1,15];E=[30,15];LOAD=400;MAX_DAYS=90;
    O0=100;H0=150;F0=100;M0=750;Z0=200;

    CM=[3.2,3.2,2.2];CW=[5.6,4.4,3.6];CI=[1.4,1.4,1.2];
    WY=[20,15,28];WM=[4,5,3];

    all_xy=[1 15;30 15;6 21;15 9;24 24;12 16;21 16];
    names={'B','E','W1','W2','W3','S1','S2'};
    dist=zeros(7);
    for i=1:7,for j=1:7
        dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));
    end;end

    % 聚焦枚举: 仅通过S1,S2的关键路径
    skeletons={
        [1,6,7,2],           % B,S1,S2,E
        [1,6,3,6,7,2],       % B,S1,W1,S1,S2,E
        [1,6,7,5,7,2],       % B,S1,S2,W3,S2,E
        [1,6,3,6,7,5,7,2],   % B,S1,W1,S1,S2,W3,S2,E
        [1,6,4,6,7,5,7,2],   % B,S1,W2,S1,S2,W3,S2,E
        [1,6,3,6,4,6,7,5,7,2],% B,S1,W1,S1,W2,S1,S2,W3,S2,E
        [1,6,7,4,7,5,7,2],   % B,S1,S2,W2,S2,W3,S2,E
        [1,3,6,7,5,7,2],     % B,W1,S1,S2,W3,S2,E
        [1,3,6,4,6,7,2],     % B,W1,S1,W2,S1,S2,E
        [1,5,7,2],           % B,W3,S2,E
        [1,4,6,7,2],         % B,W2,S1,S2,E
        [1,6,3,6,7,2],       % B,S1,W1,S1,S2,E (no W3)  
        [1,6,4,6,7,2],       % B,S1,W2,S1,S2,E
        [1,6,3,6,4,6,7,2],   % B,S1,W1,S1,W2,S1,S2,E
    };

    fprintf('===== 问题三: 聚焦列生成 =====\n');
    fprintf('初始:O=%d H=%d F=%d M=%d Z=%d\n',O0,H0,F0,M0,Z0);
    fprintf('枚举骨架数:%d\n\n',length(skeletons));

    poolP={};poolW={};poolI={};poolZ=[];poolM=[];
    bZ=-inf;bM=-inf;bP=[1,2];bW=[];bI=[];

    % 种子列
    pid0=skeletons{1};m0=length(pid0)-2;
    tr0=zeros(1,m0+1);tt0=0;
    for k=1:m0+1,tr0(k)=dist(pid0(k),pid0(k+1));tt0=tt0+tr0(k);end
    [ok,Z0,M0,idle0,~]=egsim_q3f(pid0,m0,tr0,[],[],tt0,[],all_xy,names,WM);
    if ok, poolP{1}=pid0;poolW{1}=[];poolI{1}=idle0;poolZ(1)=Z0;poolM(1)=M0;
        bZ=Z0;bM=M0;bP=pid0;bI=idle0;
        fprintf('种子:Z=%d M=%d\n',Z0,M0);
    end

    fprintf('Iter|  pi  | New Z| New M| Idle   |Cols\n');
    fprintf('----|------|------|------|--------|----\n');

    for nit=1:30
        [pi,idx]=max(poolZ);
        if pi>bZ||(pi==bZ&&poolM(idx)>bM)
            bZ=pi;bM=poolM(idx);bP=poolP{idx};bW=poolW{idx};bI=poolI{idx};
        end
        found=false;

        for si=1:length(skeletons)
            if found,break;end
            pid=skeletons{si};
            m=length(pid)-2;tr=zeros(1,m+1);tt=0;
            for k=1:m+1,tr(k)=dist(pid(k),pid(k+1));tt=tt+tr(k);end
            if tt>MAX_DAYS-3,continue;end

            wa=[];ww=[];
            for k=2:m+1
                pt=pid(k);
                if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end
            end
            nw=length(wa);

            if nw==0
                [ok,Z,M,idle,~]=egsim_q3f(pid,m,tr,wa,ww,tt,[],all_xy,names,WM);
                if ok&&Z>pi
                    poolP{end+1}=pid;poolW{end+1}=[];poolI{end+1}=idle;
                    poolZ(end+1)=Z;poolM(end+1)=M;
                    if Z>bZ||(Z==bZ&&M>bM),bZ=Z;bM=M;bP=pid;bW=[];bI=idle;end
                    fprintf('%3d |%5d |%5d |%5d |[%s]|%3d\n',nit,pi,Z,M,mat2str(idle),length(poolZ));
                    found=true;
                end
            else
                avail=MAX_DAYS-tt;
                sz=zeros(1,nw);
                for jj=1:nw
                    sz(jj)=max(WM(ww(jj))+1,min(WM(ww(jj))*3,avail)+1);
                end
                nc=prod(sz);if nc>5000,continue;end
                for ci=1:nc
                    if found,break;end
                    wd=zeros(1,nw);t2=ci-1;
                    for j=nw:-1:1,wd(j)=mod(t2,sz(j));t2=floor(t2/sz(j));end
                    cal=0;
                    for jj=1:nw,cal=cal+wcal_f(wd(jj),WM(ww(jj)));end
                    if tt+cal>MAX_DAYS,continue;end
                    [ok,Z,M,idle,~]=egsim_q3f(pid,m,tr,wa,ww,tt,wd,all_xy,names,WM);
                    if ok&&Z>pi
                        poolP{end+1}=pid;poolW{end+1}=wd;poolI{end+1}=idle;
                        poolZ(end+1)=Z;poolM(end+1)=M;
                        if Z>bZ||(Z==bZ&&M>bM),bZ=Z;bM=M;bP=pid;bW=wd;bI=idle;end
                        fprintf('%3d |%5d |%5d |%5d |[%s]|%3d\n',nit,pi,Z,M,mat2str(idle),length(poolZ));
                        found=true;
                    end
                end
            end
        end
        if ~found,break;end
    end

    fprintf('----|------|------|------|--------|----\n');
    fprintf('\n===== 最优方案(期望消耗率) =====\n');
    fprintf('Z=%d M=%d\n',bZ,bM);
    fprintf('路径:');for i=1:length(bP),fprintf(' %s',names{bP(i)});end;fprintf('\n');
    tt=0;for k=1:length(bP)-1,tt=tt+dist(bP(k),bP(k+1));end
    fprintf('旅行:%d天',tt);
    if ~isempty(bI)&&any(bI>0)
        si=1;fprintf(' 停泊:');
        for k=2:length(bP)
            pt=bP(k);
            if(pt==6||pt==7)&&si<=length(bI)&&bI(si)>0
                fprintf(' %s:%dd',names{pt},bI(si));si=si+1;
            end
        end
    end
    fprintf('\n');
    if ~isempty(bW)&&any(bW>0)
        wp=1;ws='';
        for k=2:length(bP)
            pt=bP(k);
            if pt>=3&&pt<=5&&bW(wp)>0
                W=bW(wp);M=WM(pt-2);
                if W<=M,ws=[ws sprintf('%s:%dd ',names{pt},W)];
                else,nb=ceil(W/M);ws=[ws sprintf('%s:%d(%dx%d+停) ',names{pt},W,nb-1,M)];end
                wp=wp+1;
            elseif pt>=3&&pt<=5,wp=wp+1;end
        end
        if ~isempty(ws),fprintf('作业:%s\n',ws);end
    end

    %% 蒙特卡洛
    fprintf('\n--- 蒙特卡洛(1000次,实际天气) ---\n');
    CM_N=[2,3,2];CW_N=[5,4,3];CI_N=[1,1,1];
    CM_T=[8,4,3];CW_T=[8,6,6];CI_T=[3,3,2];
    n_mc=1000;succ=0;Z_arr=zeros(1,n_mc);M_arr=zeros(1,n_mc);
    rng(42);
    for mc=1:n_mc
        weather=rand(1,300)<0.8;
        [ok,Zf,Mf]=mc_sim_fast(bP,bW,bI,weather,dist,all_xy,names,CM_N,CM_T,CW_N,CW_T,CI_N,CI_T);
        if ok,succ=succ+1;Z_arr(mc)=Zf;M_arr(mc)=Mf;end
    end
    fprintf('期望消耗率方案MC成功率:%.1f%%(%d/%d)\n',succ/n_mc*100,succ,n_mc);
    if succ>0
        fprintf('  Z:均值%.0f 范围[%d,%d]\n',mean(Z_arr(Z_arr>0)),min(Z_arr(Z_arr>0)),max(Z_arr(Z_arr>0)));
        fprintf('  M:均值%.0f 范围[%d,%d]\n',mean(M_arr(M_arr>0)),min(M_arr(M_arr>0)),max(M_arr(M_arr>0)));
    end

    %% 自适应策略MC
    fprintf('\n--- 自适应策略MC(%d次) ---\n',n_mc);
    succ_ad=0;Z_ad=zeros(1,n_mc);M_ad=zeros(1,n_mc);
    for mc=1:n_mc
        weather=rand(1,300)<0.8;
        [ok,Zf,Mf]=adaptive_sim_fast(bP,bW,bI,weather,dist,all_xy,names,CM_N,CM_T,CW_N,CW_T,CI_N,CI_T);
        if ok,succ_ad=succ_ad+1;Z_ad(mc)=Zf;M_ad(mc)=Mf;end
    end
    fprintf('自适应策略MC成功率:%.1f%%(%d/%d)\n',succ_ad/n_mc*100,succ_ad,n_mc);
    if succ_ad>0
        fprintf('  Z:均值%.0f 范围[%d,%d]\n',mean(Z_ad(Z_ad>0)),min(Z_ad(Z_ad>0)),max(Z_ad(Z_ad>0)));
        fprintf('  M:均值%.0f 范围[%d,%d]\n',mean(M_ad(M_ad>0)),min(M_ad(M_ad>0)),max(M_ad(M_ad>0)));
    end

    fprintf('\n===== 完成 =====\n');
end

%% egsim_q3f (期望消耗率,同前)
function [f,Zf,Mf,best_idle,dailyLog]=egsim_q3f(pid,m,travel,wa,ww,tt,wdays,all_xy,names,WM)
    CM=[3.2,3.2,2.2];CW=[5.6,4.4,3.6];CI=[1.4,1.4,1.2];
    LOAD=400;O0=100;H0=150;F0=100;M0=750;Z0=200;WY=[20,15,28];
    sup_segs=[];
    for k=1:m+1,to_pt=pid(k+1);if to_pt==6||to_pt==7,sup_segs(end+1)=k;end;end
    n_sup=length(sup_segs);max_idle=5;best_M=-inf;best_idle=zeros(1,n_sup);best_Z=0;best_log=[];
    total_combos=(max_idle+1)^n_sup;
    for ci=1:total_combos
        idle_vec=zeros(1,n_sup);t=ci-1;
        for j=n_sup:-1:1,idle_vec(j)=mod(t,max_idle+1);t=floor(t/(max_idle+1));end
        tw=0;if ~isempty(wdays),for jj=1:length(wdays),tw=tw+wcal_f(wdays(jj),WM(ww(jj)));end;end
        if tt+sum(idle_vec)+tw>90,continue;end
        [ok,Z,M,dlog]=gs_q3f(pid,m,travel,wa,ww,tt,wdays,idle_vec,sup_segs,all_xy,names);
        if ok,if Z>best_Z||(Z==best_Z&&M>best_M),best_Z=Z;best_M=M;best_idle=idle_vec;best_log=dlog;end;end
    end
    if best_M>-inf,f=true;Zf=best_Z;Mf=best_M;dailyLog=best_log;else,f=false;Zf=0;Mf=0;dailyLog=[];end
end

function [f,Zf,Mf,dailyLog]=gs_q3f(pid,m,travel,wa,ww,tt,wdays,idle_vec,sup_segs,all_xy,names)
    CM=[3.2,3.2,2.2];CW=[5.6,4.4,3.6];CI=[1.4,1.4,1.2];
    LOAD=400;O0=100;H0=150;F0=100;M0=750;Z0=200;WY=[20,15,28];WM=[4,5,3];
    T=tt+sum(idle_vec);
    if ~isempty(wdays),for jj=1:length(wdays),T=T+wcal_f(wdays(jj),WM(ww(jj)));end;end
    cO=zeros(1,T);cH=zeros(1,T);cF=zeros(1,T);zG=zeros(1,T);
    isSup=false(1,T);actType=cell(1,T);posX=zeros(1,T);posY=zeros(1,T);
    day=0;curX=all_xy(pid(1),1);curY=all_xy(pid(1),2);sup_cnt=0;
    for k=1:m+1
        fromX=all_xy(pid(k),1);fromY=all_xy(pid(k),2);
        toX=all_xy(pid(k+1),1);toY=all_xy(pid(k+1),2);
        dx=toX-fromX;dy=toY-fromY;stepsX=abs(dx);stepsY=abs(dy);
        sx=sign(dx);if sx==0,sx=0;end;sy=sign(dy);if sy==0,sy=0;end
        d=travel(k);
        for dd=1:d
            day=day+1;cO(day)=CM(1);cH(day)=CM(2);cF(day)=CM(3);
            if dd<=stepsX,curX=curX+sx;actType{day}='> E';
            else,curY=curY+sy;actType{day}='^ N';end
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
    dailyLog=struct('day',num2cell(1:T),'x',num2cell(posX),'y',num2cell(posY),'action',actType,'O',[],'H',[],'F',[],'M',[],'Z',[]);
    for t=1:T
        O=O-cO(t);H=H-cH(t);F=F-cF(t);
        if O<0||H<0||F<0,f=false;Zf=0;Mf=0;dailyLog=[];return;end
        if isSup(t)
            ns=T+1;for tt2=t+1:T,if isSup(tt2),ns=tt2;break;end;end
            nO=0;nH=0;nF=0;
            for tt2=t+1:ns,if tt2>T,break;end,nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);end
            sp=LOAD-(O+H+F);
            bO=max(0,ceil(nO-O));bH=max(0,ceil(nH-H));bF=max(0,ceil(nF-F));
            if bO+bH+bF>sp,f=false;Zf=0;Mf=0;dailyLog=[];return;end
            if ns>T&&(O+bO<nO||H+bH<nH||F+bF<nF),f=false;Zf=0;Mf=0;dailyLog=[];return;end
            cost=bO*2+bH*1+bF*2;
            if cost>M,f=false;Zf=0;Mf=0;dailyLog=[];return;end
            O=O+bO;H=H+bH;F=F+bF;M=M-cost;
        end
        if O+H+F>LOAD,f=false;Zf=0;Mf=0;dailyLog=[];return;end
        Zcur=Zcur+zG(t);
        dailyLog(t).O=round(O);dailyLog(t).H=round(H);dailyLog(t).F=round(F);
        dailyLog(t).M=round(M);dailyLog(t).Z=round(Zcur);
    end
    Zf=round(Zcur);Mf=round(M);f=true;
end

%% MC模拟(实际天气)
function [f,Zf,Mf]=mc_sim_fast(pid,wdays,idle_vec,weather,dist,all_xy,names,CM_N,CM_T,CW_N,CW_T,CI_N,CI_T)
    LOAD=400;O0=100;H0=150;F0=100;M0=750;Z0=200;WY=[20,15,28];WM=[4,5,3];MAX_DAYS=90;
    m=length(pid)-2;travel=zeros(1,m+1);tt=0;
    for k=1:m+1,travel(k)=dist(pid(k),pid(k+1));tt=tt+travel(k);end
    wa=[];ww=[];
    for k=2:m+1,pt=pid(k);if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end;end
    sup_segs=[];
    for k=1:m+1,to_pt=pid(k+1);if to_pt==6||to_pt==7,sup_segs(end+1)=k;end;end
    n_sup=length(sup_segs);
    if isempty(idle_vec),idle_vec=zeros(1,n_sup);end
    if isempty(wdays)&&~isempty(wa),wdays=zeros(1,length(wa));end

    T=tt+sum(idle_vec);
    if ~isempty(wdays),for jj=1:length(wdays),T=T+wcal_f(wdays(jj),WM(ww(jj)));end;end
    cO=zeros(1,T);cH=zeros(1,T);cF=zeros(1,T);zG=zeros(1,T);
    isSup=false(1,T);posX=zeros(1,T);posY=zeros(1,T);
    day=0;curX=all_xy(pid(1),1);curY=all_xy(pid(1),2);sup_cnt=0;
    for k=1:m+1
        fromX=all_xy(pid(k),1);fromY=all_xy(pid(k),2);
        toX=all_xy(pid(k+1),1);toY=all_xy(pid(k+1),2);
        dx=toX-fromX;dy=toY-fromY;stepsX=abs(dx);stepsY=abs(dy);
        sx=sign(dx);if sx==0,sx=0;end;sy=sign(dy);if sy==0,sy=0;end
        d=travel(k);
        for dd=1:d
            day=day+1;wd=weather(min(day,length(weather)));
            if wd,cO(day)=CM_N(1);cH(day)=CM_N(2);cF(day)=CM_N(3);
            else,cO(day)=CM_T(1);cH(day)=CM_T(2);cF(day)=CM_T(3);end
            if dd<=stepsX,curX=curX+sx;else,curY=curY+sy;end
            posX(day)=curX;posY(day)=curY;
        end
        sup_idx=find(sup_segs==k,1);
        if ~isempty(sup_idx)
            sup_cnt=sup_cnt+1;n_idle=idle_vec(sup_cnt);
            for id=1:n_idle
                day=day+1;wd=weather(min(day,length(weather)));
                if wd,cO(day)=CI_N(1);cH(day)=CI_N(2);cF(day)=CI_N(3);
                else,cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);end
                zG(day)=0;posX(day)=curX;posY(day)=curY;
            end
            isSup(day)=true;
        end
        if ~isempty(wa)
            wk=find(wa==k+1,1);
            if ~isempty(wk)&&~isempty(wdays)&&wdays(wk)>0
                W=wdays(wk);Mlim=WM(ww(wk));yv=WY(ww(wk));
                if W<=Mlim
                    for w=1:W
                        day=day+1;wd=weather(min(day,length(weather)));
                        if wd,cO(day)=CW_N(1);cH(day)=CW_N(2);cF(day)=CW_N(3);
                        else,cO(day)=CW_T(1);cH(day)=CW_T(2);cF(day)=CW_T(3);end
                        zG(day)=yv;posX(day)=curX;posY(day)=curY;
                    end
                else
                    nblocks=ceil(W/Mlim);remaining=W;
                    for blk=1:nblocks
                        bs=min(Mlim,remaining);
                        for w=1:bs
                            day=day+1;wd=weather(min(day,length(weather)));
                            if wd,cO(day)=CW_N(1);cH(day)=CW_N(2);cF(day)=CW_N(3);
                            else,cO(day)=CW_T(1);cH(day)=CW_T(2);cF(day)=CW_T(3);end
                            zG(day)=yv;posX(day)=curX;posY(day)=curY;
                        end
                        remaining=remaining-bs;
                        if remaining>0
                            day=day+1;wd=weather(min(day,length(weather)));
                            if wd,cO(day)=CI_N(1);cH(day)=CI_N(2);cF(day)=CI_N(3);
                            else,cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);end
                            zG(day)=0;posX(day)=curX;posY(day)=curY;
                        end
                    end
                end
            end
        end
    end
    O=O0;H=H0;F=F0;M=M0;Zcur=Z0;
    for t=1:T
        O=O-cO(t);H=H-cH(t);F=F-cF(t);
        if O<0||H<0||F<0,f=false;Zf=0;Mf=0;return;end
        if isSup(t)
            ns=T+1;for tt2=t+1:T,if isSup(tt2),ns=tt2;break;end;end
            nO=0;nH=0;nF=0;
            for tt2=t+1:ns,if tt2>T,break;end,nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);end
            sp=LOAD-(O+H+F);
            bO=max(0,ceil(nO-O));bH=max(0,ceil(nH-H));bF=max(0,ceil(nF-F));
            if bO+bH+bF>sp,f=false;Zf=0;Mf=0;return;end
            if ns>T&&(O+bO<nO||H+bH<nH||F+bF<nF),f=false;Zf=0;Mf=0;return;end
            cost=bO*2+bH*1+bF*2;
            if cost>M,f=false;Zf=0;Mf=0;return;end
            O=O+bO;H=H+bH;F=F+bF;M=M-cost;
        end
        if O+H+F>LOAD,f=false;Zf=0;Mf=0;return;end
        Zcur=Zcur+zG(t);
    end
    Zf=round(Zcur);Mf=round(M);f=true;
end

%% adaptive MC
function [f,Zf,Mf]=adaptive_sim_fast(pid,wdays,idle_vec,weather,dist,all_xy,names,CM_N,CM_T,CW_N,CW_T,CI_N,CI_T)
    LOAD=400;O0=100;H0=150;F0=100;M0=750;Z0=200;WY=[20,15,28];WM=[4,5,3];
    m=length(pid)-2;travel=zeros(1,m+1);tt=0;
    for k=1:m+1,travel(k)=dist(pid(k),pid(k+1));tt=tt+travel(k);end
    wa=[];ww=[];
    for k=2:m+1,pt=pid(k);if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end;end
    sup_segs=[];
    for k=1:m+1,to_pt=pid(k+1);if to_pt==6||to_pt==7,sup_segs(end+1)=k;end;end
    n_sup=length(sup_segs);
    if isempty(idle_vec),idle_vec=zeros(1,n_sup);end
    if isempty(wdays)&&~isempty(wa),wdays=zeros(1,length(wa));end

    T_budget=tt+sum(idle_vec)+40;  % 自适应预留缓冲
    if ~isempty(wdays),for jj=1:length(wdays),T_budget=T_budget+wcal_f(wdays(jj),WM(ww(jj)));end;end
    cO=zeros(1,T_budget);cH=zeros(1,T_budget);cF=zeros(1,T_budget);zG=zeros(1,T_budget);
    isSup=false(1,T_budget);posX=zeros(1,T_budget);posY=zeros(1,T_budget);
    day=0;curX=all_xy(pid(1),1);curY=all_xy(pid(1),2);sup_cnt=0;

    for k=1:m+1
        fromX=all_xy(pid(k),1);fromY=all_xy(pid(k),2);
        toX=all_xy(pid(k+1),1);toY=all_xy(pid(k+1),2);
        dx=toX-fromX;dy=toY-fromY;stepsX=abs(dx);stepsY=abs(dy);
        sx=sign(dx);if sx==0,sx=0;end;sy=sign(dy);if sy==0,sy=0;end
        d=travel(k);dd=0;consec=0;
        while dd<d
            day=day+1;if day>T_budget,f=false;Zf=0;Mf=0;return;end
            wd=weather(min(day,length(weather)));
            if wd||consec>3  % 正常日或连续雷暴>3天: 移动
                if wd,cO(day)=CM_N(1);cH(day)=CM_N(2);cF(day)=CM_N(3);
                else,cO(day)=CM_T(1);cH(day)=CM_T(2);cF(day)=CM_T(3);end
                dd=dd+1;consec=0;
                if dd<=stepsX,curX=curX+sx;else,curY=curY+sy;end
            else  % 雷暴: 停泊
                cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);consec=consec+1;
            end
            posX(day)=curX;posY(day)=curY;
        end
        sup_idx=find(sup_segs==k,1);
        if ~isempty(sup_idx)
            sup_cnt=sup_cnt+1;n_idle=idle_vec(sup_cnt);
            for id=1:n_idle
                day=day+1;wd=weather(min(day,length(weather)));
                if wd,cO(day)=CI_N(1);cH(day)=CI_N(2);cF(day)=CI_N(3);
                else,cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);end
                zG(day)=0;posX(day)=curX;posY(day)=curY;
            end
            isSup(day)=true;
        end
        if ~isempty(wa)
            wk=find(wa==k+1,1);
            if ~isempty(wk)&&~isempty(wdays)&&wdays(wk)>0
                W=wdays(wk);Mlim=WM(ww(wk));yv=WY(ww(wk));consec=0;
                if W<=Mlim,w_done=0;
                    while w_done<W
                        day=day+1;if day>T_budget,f=false;Zf=0;Mf=0;return;end
                        wd=weather(min(day,length(weather)));
                        if wd||consec>3
                            if wd,cO(day)=CW_N(1);cH(day)=CW_N(2);cF(day)=CW_N(3);
                            else,cO(day)=CW_T(1);cH(day)=CW_T(2);cF(day)=CW_T(3);end
                            zG(day)=yv;w_done=w_done+1;consec=0;
                        else
                            cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);consec=consec+1;
                        end
                        posX(day)=curX;posY(day)=curY;
                    end
                else
                    nblocks=ceil(W/Mlim);remaining=W;
                    for blk=1:nblocks
                        bs=min(Mlim,remaining);w_done=0;consec=0;
                        while w_done<bs
                            day=day+1;if day>T_budget,f=false;Zf=0;Mf=0;return;end
                            wd=weather(min(day,length(weather)));
                            if wd||consec>3
                                if wd,cO(day)=CW_N(1);cH(day)=CW_N(2);cF(day)=CW_N(3);
                                else,cO(day)=CW_T(1);cH(day)=CW_T(2);cF(day)=CW_T(3);end
                                zG(day)=yv;w_done=w_done+1;consec=0;
                            else
                                cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);consec=consec+1;
                            end
                            posX(day)=curX;posY(day)=curY;
                        end
                        remaining=remaining-bs;
                        if remaining>0
                            day=day+1;wd=weather(min(day,length(weather)));
                            if wd,cO(day)=CI_N(1);cH(day)=CI_N(2);cF(day)=CI_N(3);
                            else,cO(day)=CI_T(1);cH(day)=CI_T(2);cF(day)=CI_T(3);end
                            zG(day)=0;posX(day)=curX;posY(day)=curY;
                        end
                    end
                end
            end
        end
    end
    T_actual=day;O=O0;H=H0;F=F0;M=M0;Zcur=Z0;
    for t=1:T_actual
        O=O-cO(t);H=H-cH(t);F=F-cF(t);
        if O<0||H<0||F<0,f=false;Zf=0;Mf=0;return;end
        if isSup(t)
            ns=T_actual+1;for tt2=t+1:T_actual,if isSup(tt2),ns=tt2;break;end;end
            nO=0;nH=0;nF=0;
            for tt2=t+1:ns,if tt2>T_actual,break;end,nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);end
            sp=LOAD-(O+H+F);
            bO=max(0,ceil(nO-O));bH=max(0,ceil(nH-H));bF=max(0,ceil(nF-F));
            if bO+bH+bF>sp,f=false;Zf=0;Mf=0;return;end
            if ns>T_actual&&(O+bO<nO||H+bH<nH||F+bF<nF),f=false;Zf=0;Mf=0;return;end
            cost=bO*2+bH*1+bF*2;
            if cost>M,f=false;Zf=0;Mf=0;return;end
            O=O+bO;H=H+bH;F=F+bF;M=M-cost;
        end
        if O+H+F>LOAD,f=false;Zf=0;Mf=0;return;end
        Zcur=Zcur+zG(t);
    end
    Zf=round(Zcur);Mf=round(M);f=true;
end

function cal=wcal_f(W,M)
    if W<=M,cal=W;else,cal=W+(ceil(W/M)-1);end
end
