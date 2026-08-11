function verify_recovery()
% 验证: 问题二方法(含停泊-采购优化)在正常天气下能否回归问题一结果
% 预期: Z=328, 路径 B->W1->S2->W3->S2->E

    fprintf('========================================\n');
    fprintf('  验证: 问题二方法 -> 正常天气 -> 问题一\n');
    fprintf('========================================\n\n');

    B=[1,5];E=[10,5];MAX_DAYS=30;LOAD=120;
    O0=35;H0=45;F0=30;M0=240;Z0=100;

    % 正常天气消耗率
    CM=[2,3,2];CW=[5,4,3];CI=[1,1,1];WY=[20,15,28];WM=[4,5,3];

    all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];
    names={'B','E','W1','W2','W3','S1','S2'};
    dist=zeros(7);
    for i=1:7,for j=1:7
        dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));
    end;end

    inter_idx=[3 4 5 6 7];n_inter=5;max_seq=7;

    %% 使用问题二的 enhanced_gsim 框架 (含补给站停泊枚举)
    poolP={};poolW={};poolI={};poolZ=[];poolM=[];
    bZ=-inf;bM=-inf;bP=[1,2];bW=[];bI=[];

    % 种子: B->E
    pid0=[1,2];m0=0;tr0=dist(1,2);tt0=tr0;
    [ok,Z0,M0,idle0,~]=egsim_verify(pid0,m0,tr0,[],[],tt0,[],all_xy,names,WM,CM,CW,CI,LOAD,MAX_DAYS,O0,H0,F0,M0,Z0);
    if ok,poolP{1}=pid0;poolW{1}=[];poolI{1}=idle0;poolZ(1)=Z0;poolM(1)=M0;
        bZ=Z0;bM=M0;bP=pid0;bI=idle0;end

    fprintf('种子 B->E: Z=%d M=%d\n',Z0,M0);
    fprintf('Iter|  pi   | New Z | New M | Idle    |Cols\n');
    fprintf('----|-------|-------|-------|---------|----\n');

    nit=0;ncg=0;
    while nit<30
        [pi,idx]=max(poolZ);
        if pi>bZ||(pi==bZ&&poolM(idx)>bM)
            bZ=pi;bM=poolM(idx);bP=poolP{idx};bW=poolW{idx};bI=poolI{idx};
        end
        found=false;nit=nit+1;

        for sl=0:max_seq
            if found,break;end
            ns=n_inter^sl;
            for si=1:ns
                if found,break;end
                s=zeros(1,sl);t=si-1;
                for j=sl:-1:1,s(j)=mod(t,n_inter)+1;t=floor(t/n_inter);end
                pid=[1,inter_idx(s),2];
                dup=false;
                for k=2:length(pid),if pid(k)==pid(k-1),dup=true;break;end;end
                if dup,continue;end
                m=length(pid)-2;tr=zeros(1,m+1);tt=0;
                for k=1:m+1,tr(k)=dist(pid(k),pid(k+1));tt=tt+tr(k);end
                if tt>MAX_DAYS,continue;end

                wa=[];ww=[];
                for k=2:m+1,pt=pid(k);if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end;end
                nw=length(wa);

                if nw==0
                    [ok,Z,M,idle,~]=egsim_verify(pid,m,tr,wa,ww,tt,[],all_xy,names,WM,CM,CW,CI,LOAD,MAX_DAYS,O0,H0,F0,M0,Z0);
                    if ok&&Z>pi
                        poolP{end+1}=pid;poolW{end+1}=[];poolI{end+1}=idle;
                        poolZ(end+1)=Z;poolM(end+1)=M;ncg=ncg+1;
                        if Z>bZ||(Z==bZ&&M>bM),bZ=Z;bM=M;bP=pid;bW=[];bI=idle;end
                        fprintf('%3d |%6d |%6d |%6d |[%s]|%3d\n',nit,pi,Z,M,mat2str(idle),length(poolZ));
                        found=true;
                    end
                else
                    avail=MAX_DAYS-tt;sz=zeros(1,nw);
                    for jj=1:nw
                        sz(jj)=max(WM(ww(jj))+1,min(WM(ww(jj))*3,avail)+1);
                    end
                    nc=prod(sz);if nc>5000,continue;end
                    for ci=1:nc
                        if found,break;end
                        wd=zeros(1,nw);t2=ci-1;
                        for j=nw:-1:1,wd(j)=mod(t2,sz(j));t2=floor(t2/sz(j));end
                        cal=0;
                        for jj=1:nw,cal=cal+wcal_v(wd(jj),WM(ww(jj)));end
                        if tt+cal>MAX_DAYS,continue;end
                        [ok,Z,M,idle,~]=egsim_verify(pid,m,tr,wa,ww,tt,wd,all_xy,names,WM,CM,CW,CI,LOAD,MAX_DAYS,O0,H0,F0,M0,Z0);
                        if ok&&Z>pi
                            poolP{end+1}=pid;poolW{end+1}=wd;poolI{end+1}=idle;
                            poolZ(end+1)=Z;poolM(end+1)=M;ncg=ncg+1;
                            if Z>bZ||(Z==bZ&&M>bM),bZ=Z;bM=M;bP=pid;bW=wd;bI=idle;end
                            fprintf('%3d |%6d |%6d |%6d |[%s]|%3d\n',nit,pi,Z,M,mat2str(idle),length(poolZ));
                            found=true;
                        end
                    end
                end
            end
        end
        if ~found,break;end
    end

    fprintf('----|-------|-------|-------|---------|----\n\n');
    fprintf('===== 验证结果 =====\n');
    fprintf('最优 Z = %d\n',bZ);
    fprintf('最优 M = %d\n',bM);
    fprintf('路径: ');for i=1:length(bP),fprintf('%s ',names{bP(i)});end;fprintf('\n');
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
        if ~isempty(ws),fprintf('作业: %s\n',ws);end
    end

    if bZ==328
        fprintf('\n*** 验证通过: Z=328, 与问题一最优解一致! ***\n');
    else
        fprintf('\n*** 验证结果 Z=%d (问题一最优为328) ***\n',bZ);
    end
    fprintf('列数:%d 迭代:%d\n',ncg,nit);
end

function [f,Zf,Mf,best_idle,dailyLog]=egsim_verify(pid,m,travel,wa,ww,tt,wdays,all_xy,names,WM,CM,CW,CI,LOAD,MAX_DAYS,O0,H0,F0,M0,Z0)
    WY=[20,15,28];
    sup_segs=[];
    for k=1:m+1,to_pt=pid(k+1);if to_pt==6||to_pt==7,sup_segs(end+1)=k;end;end
    n_sup=length(sup_segs);max_idle=5;best_M=-inf;best_idle=zeros(1,n_sup);best_Z=0;best_log=[];
    total_combos=(max_idle+1)^n_sup;
    for ci=1:total_combos
        idle_vec=zeros(1,n_sup);t=ci-1;
        for j=n_sup:-1:1,idle_vec(j)=mod(t,max_idle+1);t=floor(t/(max_idle+1));end
        tw=0;if ~isempty(wdays),for jj=1:length(wdays),tw=tw+wcal_v(wdays(jj),WM(ww(jj)));end;end
        if tt+sum(idle_vec)+tw>MAX_DAYS,continue;end
        [ok,Z,M,dlog]=gs_verify(pid,m,travel,wa,ww,tt,wdays,idle_vec,sup_segs,all_xy,names,WM,CM,CW,CI,LOAD,MAX_DAYS,O0,H0,F0,M0,Z0);
        if ok,if Z>best_Z||(Z==best_Z&&M>best_M),best_Z=Z;best_M=M;best_idle=idle_vec;best_log=dlog;end;end
    end
    if best_M>-inf,f=true;Zf=best_Z;Mf=best_M;dailyLog=best_log;else,f=false;Zf=0;Mf=0;dailyLog=[];end
end

function [f,Zf,Mf,dailyLog]=gs_verify(pid,m,travel,wa,ww,tt,wdays,idle_vec,sup_segs,all_xy,names,WM,CM,CW,CI,LOAD,MAX_DAYS,O0,H0,F0,M0,Z0)
    WY=[20,15,28];
    T=tt+sum(idle_vec);
    if ~isempty(wdays),for jj=1:length(wdays),T=T+wcal_v(wdays(jj),WM(ww(jj)));end;end
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
            day=day+1;cO(day)=CM(1);cH(day)=CM(2);cF(day)=CM(3);
            if dd<=stepsX,curX=curX+sx;else,curY=curY+sy;end
            posX(day)=curX;posY(day)=curY;
        end
        sup_idx=find(sup_segs==k,1);
        if ~isempty(sup_idx)
            sup_cnt=sup_cnt+1;n_idle=idle_vec(sup_cnt);
            for id=1:n_idle
                day=day+1;cO(day)=CI(1);cH(day)=CI(2);cF(day)=CI(3);
                zG(day)=0;posX(day)=curX;posY(day)=curY;
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
                    end
                else
                    nblocks=ceil(W/Mlim);remaining=W;
                    for blk=1:nblocks
                        bs=min(Mlim,remaining);
                        for w=1:bs
                            day=day+1;cO(day)=CW(1);cH(day)=CW(2);cF(day)=CW(3);
                            zG(day)=yv;posX(day)=curX;posY(day)=curY;
                        end
                        remaining=remaining-bs;
                        if remaining>0
                            day=day+1;cO(day)=CI(1);cH(day)=CI(2);cF(day)=CI(3);
                            zG(day)=0;posX(day)=curX;posY(day)=curY;
                        end
                    end
                end
            end
        end
    end
    O=O0;H=H0;F=F0;M=M0;Zcur=Z0;
    dailyLog=struct('day',num2cell(1:T),'x',num2cell(posX),'y',num2cell(posY),'O',[],'H',[],'F',[],'M',[],'Z',[]);
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

function cal=wcal_v(W,M)
    if W<=M,cal=W;else,cal=W+(ceil(W/M)-1);end
end
