function solve_cp()
% solve_cp.m - Constraint Programming for Task 1 (Revised)
% Fixes: supply-day bug, parking-at-workpoint, tighter bound

MAX_DAYS=30; MAX_LOAD=120;
all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];
dist=zeros(7); for i=1:7,for j=1:7,dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));end;end
inter_idx=[3 4 5 6 7]; names={'B','E','W1','W2','W3','S1','S2'};
WY=[20,15,28]; WM=[4,5,3];

fprintf('========================================\n');
fprintf('  CP for Task 1 (Revised: parking + bugfix)\n');
fprintf('========================================\n');

bZ=-inf; bM=-inf; bP=[1,2]; bW=0; bWD=[]; bSched=struct(); nodes=0;
[bZ,bM,bP,bW,bWD,bSched,nodes]=cp_search([1],0,[],[],bZ,bM,bP,bW,bWD,bSched,dist,inter_idx,WY,WM,nodes,0,MAX_DAYS,MAX_LOAD);

fprintf('\n===== OPTIMAL SOLUTION =====\n');
fprintf('Z = %d\n',bZ); fprintf('M = %d\n',bM');
pstr=''''; for i=1:length(bP), pstr=[pstr,'' '',names{bP(i)}]; end; fprintf('Path:%s\n',pstr);
tt=0; for k=1:length(bP)-1, tt=tt+dist(bP(k),bP(k+1)); end
fprintf('Travel: %d days | Work: %d days | Total: %d days\n',tt,sum(bWD),bW);
fprintf('Nodes explored: %d\n',nodes);
print_schedule(bP,bWD,bSched,dist,WM,WY,names,MAX_DAYS,MAX_LOAD,all_xy);
fprintf('\nDone.\n');
end

function [bZ,bM,bP,bW,bWD,bSched,nodes]=cp_search(path,tsf,wa,ww,bZ,bM,bP,bW,bWD,bSched,dist,inter_idx,WY,WM,nodes,depth,MAX_DAYS,MAX_LOAD)
    nodes=nodes+1; lp=path(end);
    if lp~=2
        dE=dist(lp,2); rem=MAX_DAYS-tsf;
        if rem<dE, return; end
        ub=100+max_work_with_park(3,rem-dE)*28;
        if ub<=bZ && bZ>-inf, return; end
    end
    dE=dist(lp,2);
    if tsf+dE<=MAX_DAYS
        fp=[path,2]; m=length(fp)-2; tt=0;
        for k=1:m+1, tt=tt+dist(fp(k),fp(k+1)); end
        rem=MAX_DAYS-tt; nw=length(wa);
        if nw==0
            [ok,Z,M,sched]=gsim(fp,m,dist,wa,ww,tt,[],WM,WY,MAX_LOAD);
            if ok && (Z>bZ || (Z==bZ && M>bM))
                bZ=Z; bM=M; bP=fp; bW=0; bWD=[]; bSched=sched;
            end
        else
            max_wk=zeros(1,nw);
            for j=1:nw, max_wk(j)=max_work_with_park(WM(ww(j)),rem); end
            sz=max_wk+1; nc=prod(sz);
            for ci=1:nc
                wd=zeros(1,nw); t2=ci-1;
                for j=nw:-1:1, wd(j)=mod(t2,sz(j)); t2=floor(t2/sz(j)); end
                total_stay=0;
                for j=1:nw
                    if wd(j)>0, total_stay=total_stay+wd(j)+max(0,ceil(wd(j)/WM(ww(j)))-1); end
                end
                if tt+total_stay>MAX_DAYS, continue; end
                [ok,Z,M,sched]=gsim(fp,m,dist,wa,ww,tt,wd,WM,WY,MAX_LOAD);
                if ok && (Z>bZ || (Z==bZ && M>bM))
                    bZ=Z; bM=M; bP=fp; bW=total_stay; bWD=wd; bSched=sched;
                end
            end
        end
    end
    for ni=1:5, np=inter_idx(ni);
        if np==lp, continue; end
        d=dist(lp,np); if tsf+d>MAX_DAYS, continue; end
        dE2=dist(np,2); if tsf+d+dE2>MAX_DAYS, continue; end
        np2=[path,np]; nt=tsf+d; nwa=wa; nww=ww;
        if np>=3 && np<=5, nwa(end+1)=length(np2); nww(end+1)=np-2; end
        [bZ,bM,bP,bW,bWD,bSched,nodes]=cp_search(np2,nt,nwa,nww,bZ,bM,bP,bW,bWD,bSched,dist,inter_idx,WY,WM,nodes,depth+1,MAX_DAYS,MAX_LOAD);
    end
end

function [f,Zf,Mf,sched]=gsim(pid,m,dist_all,wa,ww,tt,wdays,WM,WY,MAX_LOAD)
total_extra=0;
for j=1:length(wdays)
    if wdays(j)>0, total_extra=total_extra+max(0,ceil(wdays(j)/WM(ww(j)))-1); end
end
T=tt+sum(wdays)+total_extra;
cO=zeros(1,T); cH=zeros(1,T); cF=zeros(1,T); zG=zeros(1,T); isSup=false(1,T);
day=0;
for k=1:m+1
    d=dist_all(pid(k),pid(k+1));
    for dd=1:d
        day=day+1; cO(day)=2; cH(day)=3; cF(day)=2;
        if dd==d, to_pt=pid(k+1); if to_pt==6||to_pt==7, isSup(day)=true; end; end
    end
    if ~isempty(wa)
        wk=find(wa==k+1,1);
        if ~isempty(wk) && wdays(wk)>0
            mc=WM(ww(wk)); rem=wdays(wk); yld=WY(ww(wk));
            while rem>0
                chunk=min(rem,mc);
                for w=1:chunk, day=day+1; cO(day)=5; cH(day)=4; cF(day)=3; zG(day)=yld; end
                rem=rem-chunk;
                if rem>0, day=day+1; cO(day)=1; cH(day)=1; cF(day)=1; zG(day)=0; end
            end
        end
    end
end
O=35; H=45; F=30; M=240; Zf=100; f=false; Mf=0; sched=struct();
for t=1:T
    % FIXED: deduct consumption FIRST (even at supply points)
    O=O-cO(t); H=H-cH(t); F=F-cF(t); Zf=Zf+zG(t);
    if O<0||H<0||F<0, f=false; return; end
    if O+H+F>MAX_LOAD+1e-9, f=false; return; end
    if isSup(t)
        ns=T+1; for tt2=t+1:T, if isSup(tt2), ns=tt2; break; end; end
        nO=0; nH=0; nF=0; for tt2=t+1:ns, if tt2>T, break; end; nO=nO+cO(tt2); nH=nH+cH(tt2); nF=nF+cF(tt2); end
        sp=MAX_LOAD-(O+H+F); bO=max(0,nO-O); bH=max(0,nH-H); bF=max(0,nF-F);
        if bO+bH+bF>sp, f=false; return; end
        if ns>T && (O+bO<nO||H+bH<nH||F+bF<nF), f=false; return; end
        cost=bO*2+bH*1+bF*2; if cost>M, f=false; return; end
        O=O+bO; H=H+bH; F=F+bF; M=M-cost;
    end
end
f=true; Mf=M;
sched=struct();
end

function max_w=max_work_with_park(mc,remaining)
best=0;
for k=1:(remaining+1)
    stay=k*mc+(k-1);
    if stay>remaining, break; end
    best=k*mc;
    slack=remaining-stay;
    if slack>=1, best=max(best,k*mc+min(mc,slack-1)); end
end
max_w=max(best,min(mc,remaining));
end

function print_schedule(bP,bWD,sched,dist,WM,WY,names,MAX_DAYS,MAX_LOAD,all_xy)
% Rebuild wa/ww from bP, then call gsim for accurate daily state
wa=[]; ww=[];
for i=2:length(bP)
    if bP(i)>=3 && bP(i)<=5, wa(end+1)=i; ww(end+1)=bP(i)-2; end
end
m=length(bP)-2; tt=0;
for k=1:m+1, tt=tt+dist(bP(k),bP(k+1)); end

% Call gsim to get the consumption schedule, then replay with correct supply purchases
total_extra=0;
for j=1:length(wa)
    if bWD(j)>0, total_extra=total_extra+max(0,ceil(bWD(j)/WM(ww(j)))-1); end
end
T=tt+sum(bWD)+total_extra;
cO=zeros(1,T); cH=zeros(1,T); cF=zeros(1,T); zG=zeros(1,T); isSup=false(1,T);
day=0;
for k=1:m+1
    d=dist(bP(k),bP(k+1));
    for dd=1:d
        day=day+1; cO(day)=2; cH(day)=3; cF(day)=2;
        if dd==d, to_pt=bP(k+1); if to_pt==6||to_pt==7, isSup(day)=true; end; end
    end
    wk=find(wa==k+1,1);
    if ~isempty(wk) && bWD(wk)>0
        mc=WM(ww(wk)); rem=bWD(wk); yld=WY(ww(wk));
        while rem>0
            chunk=min(rem,mc);
            for w=1:chunk, day=day+1; cO(day)=5; cH(day)=4; cF(day)=3; zG(day)=yld; end
            rem=rem-chunk;
            if rem>0, day=day+1; cO(day)=1; cH(day)=1; cF(day)=1; zG(day)=0; end
        end
    end
end
fprintf('\n===== DAY-BY-DAY SCHEDULE =====\n');
fprintf('Day | Pos (x,y)  | Action      |  O   H   F  Load |   Z     M\n');
fprintf('----|-------------|-------------|------------------|------------\n');
O=35; H=45; F=30; M=240; Z=100; day2=0;
for k=1:m+1
    fr=bP(k); to=bP(k+1); d=dist(fr,to);
    for dd=1:d
        day2=day2+1;
        t=dd/d; x=round(all_xy(fr,1)+(all_xy(to,1)-all_xy(fr,1))*t);
        y=round(all_xy(fr,2)+(all_xy(to,2)-all_xy(fr,2))*t);
        O=O-2; H=H-3; F=F-2;
        if isSup(day2)
            ns=T+1; for tt2=day2+1:T, if isSup(tt2), ns=tt2; break; end; end
            nO=0; nH=0; nF=0; for tt2=day2+1:ns, if tt2>T,break;end; nO=nO+cO(tt2); nH=nH+cH(tt2); nF=nF+cF(tt2); end
            bO=max(0,nO-O); bH=max(0,nH-H); bF=max(0,nF-F);
            cost=bO*2+bH*1+bF*2; M=M-cost; O=O+bO; H=H+bH; F=F+bF;
            fprintf('%3d | (%2d,%2d)     | SUPPLY      | %3d %3d %3d %4d | %4d %5d  (+O%d H%d F%d)\n',day2,x,y,O,H,F,O+H+F,Z,M,bO,bH,bF);
        else
            fprintf('%3d | (%2d,%2d)     | move        | %3d %3d %3d %4d | %4d %5d\n',day2,x,y,O,H,F,O+H+F,Z,M);
        end
    end
    wk=find(wa==k+1,1);
    if ~isempty(wk) && bWD(wk)>0
        mc=WM(ww(wk)); rem=bWD(wk); yld=WY(ww(wk));
        while rem>0
            chunk=min(rem,mc);
            for w=1:chunk
                day2=day2+1; O=O-5; H=H-4; F=F-3; Z=Z+yld;
                fprintf('%3d | (%2d,%2d)     | work(%s)    | %3d %3d %3d %4d | %4d %5d\n',day2,all_xy(to,1),all_xy(to,2),names{to},O,H,F,O+H+F,Z,M);
            end
            rem=rem-chunk;
            if rem>0
                day2=day2+1; O=O-1; H=H-1; F=F-1;
                fprintf('%3d | (%2d,%2d)     | park(reset) | %3d %3d %3d %4d | %4d %5d\n',day2,all_xy(to,1),all_xy(to,2),O,H,F,O+H+F,Z,M);
            end
        end
    end
end
fprintf('----|-------------|-------------|------------------|------------\n');
fprintf('  Final at E: Z=%d M=%d Day=%d\n',Z,M,day2);
end
