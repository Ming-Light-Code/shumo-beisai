function solve_cg()
% solve_cg.m - Column Generation for Task 1 (FIXED version)
% Bug fix: supply-day resource consumption now correctly deducted before purchase.
% Added: daily position tracking, full daily output log.

LOAD_LIMIT = 120;
B=[1,5]; E=[10,5]; CM=[2,3,2]; CW=[5,4,3]; WY=[20,15,28]; WM=[4,5,3];
O0=35; H0=45; F0=30; M0=240; Z0=100; MAX_DAYS=30;
all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];
dist=zeros(7); for i=1:7, for j=1:7, dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2)); end; end
inter_idx=[3 4 5 6 7]; n_inter=5; max_seq=min(7,MAX_DAYS-dist(1,2));
names={'B','E','W1','W2','W3','S1','S2'};

fprintf('========================================\n');
fprintf('  Column Generation for Task 1 (FIXED)\n');
fprintf('========================================\n\n');

poolP={}; poolW={}; poolZ=[]; poolM=[];
pid0=[1,2]; [ok,Z0,M0,~]=gsim(pid0,0,dist(1,2),[],[],dist(1,2),[],all_xy,names);
if ok, poolP{1}=pid0; poolW{1}=[]; poolZ(1)=Z0; poolM(1)=M0; end
bZ=-inf; bM=-inf; bP=[1,2]; bW=[]; bLog=struct(); nit=0; ncg=0;

fprintf('Iter |  pi(dual) |  New Z  |  New M  | Columns\n');
fprintf('-----|-----------|---------|---------|--------\n');

while true
    [pi,idx]=max(poolZ);
    if pi>bZ || (pi==bZ && poolM(idx)>bM)
        bZ=pi; bM=poolM(idx); bP=poolP{idx}; bW=poolW{idx};
    end
    found=false; nit=nit+1;
    for sl=0:max_seq
        if found, break; end
        ns=n_inter^sl;
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
            if tt>MAX_DAYS, continue; end
            wa=[]; ww=[];
            for k=2:m+1
                pt=pid(k);
                if pt>=3 && pt<=5, wa(end+1)=k; ww(end+1)=pt-2; end
            end
            nw=length(wa);
            if nw==0
                [ok,Z,M,dlog]=gsim(pid,m,tr,wa,ww,tt,[],all_xy,names);
                if ok && Z>pi
                    poolP{end+1}=pid; poolW{end+1}=[]; poolZ(end+1)=Z; poolM(end+1)=M; ncg=ncg+1;
                    if Z>bZ || (Z==bZ && M>bM), bZ=Z; bM=M; bP=pid; bW=[]; bLog=dlog; end
                    fprintf(' %3d | %9d | %7d | %7d | %6d\n',nit,pi,Z,M,length(poolZ));
                    found=true;
                end
            else
                sz=WM(ww)+1; nc=prod(sz);
                for ci=1:nc
                    if found, break; end
                    wd=zeros(1,nw); t2=ci-1;
                    for j=nw:-1:1, wd(j)=mod(t2,sz(j)); t2=floor(t2/sz(j)); end
                    if tt+sum(wd)>MAX_DAYS, continue; end
                    [ok,Z,M,dlog]=gsim(pid,m,tr,wa,ww,tt,wd,all_xy,names);
                    if ok && Z>pi
                        poolP{end+1}=pid; poolW{end+1}=wd; poolZ(end+1)=Z; poolM(end+1)=M; ncg=ncg+1;
                        if Z>bZ || (Z==bZ && M>bM), bZ=Z; bM=M; bP=pid; bW=wd; bLog=dlog; end
                        fprintf(' %3d | %9d | %7d | %7d | %6d\n',nit,pi,Z,M,length(poolZ));
                        found=true;
                    end
                end
            end
        end
    end
    if ~found, break; end
end

fprintf('-----|-----------|---------|---------|--------\n');

fprintf('\n===== OPTIMAL SOLUTION =====\n');
fprintf('Z = %d\n',bZ);
fprintf('M = %d\n',bM);
fprintf('Path: ');
for i=1:length(bP), fprintf('%s ',names{bP(i)}); end; fprintf('\n');
fprintf('Columns generated: %d\n',ncg);
fprintf('Pricing iterations: %d\n',nit);
tt=0; for k=1:length(bP)-1, tt=tt+dist(bP(k),bP(k+1)); end
fprintf('Travel: %d days, Work: %d days, Total: %d days\n',tt,sum(bW),tt+sum(bW));

[m_opt,tr_opt,wa_opt,ww_opt,tt_opt]=buildPathParams(bP,bW,dist);
[~,~,~,finalLog]=gsim(bP,m_opt,tr_opt,wa_opt,ww_opt,tt_opt,bW,all_xy,names);

fprintf('\n===== DAILY LOG =====\n');
fprintf('Day | Pos    | Action              |   O   H   F  Load |     M |     Z\n');
fprintf('----|--------|---------------------|-------------------|-------|------\n');
for d=1:length(finalLog)
    fprintf('%3d | (%2d,%2d) | %-20s | %3d %3d %3d  %3d | %5d | %5d\n',...
        finalLog(d).day, finalLog(d).x, finalLog(d).y, finalLog(d).action,...
        finalLog(d).O, finalLog(d).H, finalLog(d).F, finalLog(d).O+finalLog(d).H+finalLog(d).F,...
        finalLog(d).M, finalLog(d).Z);
end
fprintf('\nDone.\n');
end

function [m,tr,wa,ww,tt] = buildPathParams(pid,wd,dist)
    m=length(pid)-2;
    tr=zeros(1,m+1); tt=0;
    for k=1:m+1, tr(k)=dist(pid(k),pid(k+1)); tt=tt+tr(k); end
    wa=[]; ww=[];
    for k=2:m+1
        pt=pid(k);
        if pt>=3 && pt<=5, wa(end+1)=k; ww(end+1)=pt-2; end
    end
end

function [f,Zf,Mf,dailyLog] = gsim(pid,m,travel,wa,ww,tt,wdays,all_xy,names)
T=tt+sum(wdays);
cO=zeros(1,T); cH=zeros(1,T); cF=zeros(1,T);
zG=zeros(1,T); isSup=false(1,T);
actType=cell(1,T);

day=0;
curX=all_xy(pid(1),1); curY=all_xy(pid(1),2);
posX=zeros(1,T); posY=zeros(1,T);

for k=1:m+1
    fromX=all_xy(pid(k),1); fromY=all_xy(pid(k),2);
    toX=all_xy(pid(k+1),1); toY=all_xy(pid(k+1),2);
    dx=toX-fromX; dy=toY-fromY;
    stepsX=abs(dx); stepsY=abs(dy);
    sx=sign(dx); if sx==0, sx=0; end
    sy=sign(dy); if sy==0, sy=0; end

    d=travel(k);
    for dd=1:d
        day=day+1;
        cO(day)=2; cH(day)=3; cF(day)=2;
        if dd<=stepsX
            curX=curX+sx;
            if sx>0, actType{day}='> E'; elseif sx<0, actType{day}='< W'; end
        else
            curY=curY+sy;
            if sy>0, actType{day}='^ N'; elseif sy<0, actType{day}='v S'; end
        end
        posX(day)=curX; posY(day)=curY;
        if dd==d
            to_pt=pid(k+1);
            if to_pt==6||to_pt==7, isSup(day)=true; end
        end
    end

    if ~isempty(wa)
        wk=find(wa==k+1,1);
        if ~isempty(wk) && wdays(wk)>0
            yld=[20,15,28];
            for w=1:wdays(wk)
                day=day+1;
                cO(day)=5; cH(day)=4; cF(day)=3;
                zG(day)=yld(ww(wk));
                posX(day)=curX; posY(day)=curY;
                actType{day}=sprintf('Work %s',names{pid(k+1)});
            end
        end
    end
end

O=35; H=45; F=30; M=240; Zcur=100;

dailyLog=struct('day',num2cell(1:T),'x',num2cell(posX),'y',num2cell(posY),...
    'action',actType,'O',[],'H',[],'F',[],'M',[],'Z',[]);

for t=1:T
    O=O-cO(t); H=H-cH(t); F=F-cF(t);
    if O<0||H<0||F<0, f=false; Zf=0; Mf=0; dailyLog=[]; return; end

    if isSup(t)
        ns=T+1;
        for tt2=t+1:T, if isSup(tt2), ns=tt2; break; end; end
        nO=0; nH=0; nF=0;
        for tt2=t+1:ns
            if tt2>T, break; end
            nO=nO+cO(tt2); nH=nH+cH(tt2); nF=nF+cF(tt2);
        end
        sp=120-(O+H+F);
        bO=max(0,nO-O); bH=max(0,nH-H); bF=max(0,nF-F);
        if bO+bH+bF>sp, f=false; Zf=0; Mf=0; dailyLog=[]; return; end
        if ns>T && (O+bO<nO||H+bH<nH||F+bF<nF), f=false; Zf=0; Mf=0; dailyLog=[]; return; end
        cost=bO*2+bH*1+bF*2;
        if cost>M, f=false; Zf=0; Mf=0; dailyLog=[]; return; end
        O=O+bO; H=H+bH; F=F+bF; M=M-cost;
        if bO+bH+bF>0
            actType{t}=[actType{t} sprintf(' Buy O:%d H:%d F:%d',bO,bH,bF)];
        else
            actType{t}=[actType{t} ' NoBuy'];
        end
    end

    if O+H+F>120, f=false; Zf=0; Mf=0; dailyLog=[]; return; end

    Zcur=Zcur+zG(t);

    dailyLog(t).action=actType{t};
    dailyLog(t).O=O; dailyLog(t).H=H; dailyLog(t).F=F;
    dailyLog(t).M=M; dailyLog(t).Z=Zcur;
end

Zf=Zcur; Mf=M; f=true;
end
