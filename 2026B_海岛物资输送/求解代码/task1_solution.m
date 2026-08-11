function task1_solution()
% Five methods for Task 1

B=[1,5]; E=[10,5]; CM=[2,3,2]; CW=[5,4,3]; WY=[20,15,28]; WM=[4,5,3];
all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6]; inter_idx=[3 4 5 6 7];
dist=zeros(7); for i=1:7,for j=1:7,dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));end;end
max_seq=min(7,30-dist(1,2)); n_inter=5;
names={'B','E','W1','W2','W3','S1','S2'};

fprintf('========================================\n  Task 1: Five Methods\n========================================\n\n');

fprintf('--- 1. Enumeration+Greedy ---\n'); tic;
[Z1,M1,p1,w1]=m_enum(dist,all_xy,inter_idx,max_seq,WY,WM,CM,CW); t1=toc;
fprintf('  Z=%d M=%d %.2fs Path: ',Z1,M1,t1); for i=1:length(p1),fprintf('%s ',names{p1(i)});end;fprintf('\n');

fprintf('--- 2. DP ---\n'); tic;
[Z2,M2,p2,w2]=m_dp(dist,all_xy,inter_idx,max_seq,WY,WM,CM,CW); t2=toc;
fprintf('  Z=%d M=%d %.2fs Path: ',Z2,M2,t2); for i=1:length(p2),fprintf('%s ',names{p2(i)});end;fprintf('\n');

fprintf('--- 3. DE ---\n'); tic;
[Z3,M3,p3,w3]=m_de(dist,all_xy,inter_idx,WY,WM,CM,CW); t3=toc;
fprintf('  Z=%d M=%d %.2fs Path: ',Z3,M3,t3); for i=1:length(p3),fprintf('%s ',names{p3(i)});end;fprintf('\n');

fprintf('--- 4. Column Generation ---\n'); tic;
[Z4,M4,p4,w4]=m_cg(dist,all_xy,inter_idx,max_seq,WY,WM,CM,CW); t4=toc;
fprintf('  Z=%d M=%d %.2fs Path: ',Z4,M4,t4); for i=1:length(p4),fprintf('%s ',names{p4(i)});end;fprintf('\n');

fprintf('--- 5. Constraint Programming ---\n'); tic;
[Z5,M5,p5,w5]=m_cp(dist,all_xy,inter_idx,max_seq,WY,WM,CM,CW); t5=toc;
fprintf('  Z=%d M=%d %.2fs Path: ',Z5,M5,t5); for i=1:length(p5),fprintf('%s ',names{p5(i)});end;fprintf('\n');

fprintf('\n========================================\n            SUMMARY\n========================================\n');
fprintf('%-18s %6s %6s %8s\n','Method','Z','M','Time(s)');
fprintf('------------------------------------------\n');
fprintf('%-18s %6d %6d %8.2f\n','Enum+Greedy',Z1,M1,t1);
fprintf('%-18s %6d %6d %8.2f\n','DP',Z2,M2,t2);
fprintf('%-18s %6d %6d %8.2f\n','DE',Z3,M3,t3);
fprintf('%-18s %6d %6d %8.2f\n','ColGen',Z4,M4,t4);
fprintf('%-18s %6d %6d %8.2f\n','CP',Z5,M5,t5);
fprintf('------------------------------------------\n');
end

% ===== METHOD 1: ENUMERATION + GREEDY =====
function [bZ,bM,bP,bW]=m_enum(dist,all_xy,inter_idx,max_seq,WY,WM,CM,CW)
    bZ=-inf;bM=-inf;bP=[1,2];bW=[];
    for sl=0:max_seq,ns=5^sl;
        for si=1:ns,s=zeros(1,sl);t=si-1;
            for j=sl:-1:1,s(j)=mod(t,5)+1;t=floor(t/5);end
            pid=[1,inter_idx(s),2];dup=0;
            for k=2:length(pid),if pid(k)==pid(k-1),dup=1;break;end;end;if dup,continue;end
            m=length(pid)-2;tr=zeros(1,m+1);tt=0;
            for k=1:m+1,tr(k)=dist(pid(k),pid(k+1));tt=tt+tr(k);end;if tt>30,continue;end
            wa=[];ww=[];
            for k=2:m+1,pt=pid(k);if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end;end
            nw=length(wa);
            if nw==0
                [ok,Z,M]=gsim(pid,m,tr,wa,ww,tt,[],WY,CM,CW);
                if ok&&(Z>bZ||(Z==bZ&&M>bM)),bZ=Z;bM=M;bP=pid;end
            else
                sz=WM(ww)+1;nc=prod(sz);
                for ci=1:nc,wd=zeros(1,nw);t2=ci-1;
                    for j=nw:-1:1,wd(j)=mod(t2,sz(j));t2=floor(t2/sz(j));end
                    if tt+sum(wd)>30,continue;end
                    [ok,Z,M]=gsim(pid,m,tr,wa,ww,tt,wd,WY,CM,CW);
                    if ok&&(Z>bZ||(Z==bZ&&M>bM)),bZ=Z;bM=M;bP=pid;bW=wd;end
                end
            end
        end
    end
end

% ===== METHOD 2: DP =====
function [bZ,bM,bP,bW]=m_dp(dist,all_xy,inter_idx,max_seq,WY,WM,CM,CW)
    NM=32;dZ=-inf(NM,7);dM=-inf(NM,7);dZ(1,1)=100;dM(1,1)=240;
    bZ=-inf;bM=-inf;bP=[1,2];bW=[];
    for mask=1:NM
        for last=1:7,if dZ(mask,last)<0,continue;end
            dE=dist(last,2);if dE<=30
                [ok,Z,M]=gsim([last,2],0,dE,[],[],dE,[],WY,CM,CW);
                if ok&&(Z>bZ||(Z==bZ&&M>bM)),bZ=Z;bM=M;bP=[last,2];end
            end
            for nx=3:7,pb=nx-2;if bitand(mask,bitshift(1,pb-1)),continue;end
                nm=bitor(mask,bitshift(1,pb-1));d=dist(last,nx);
                [ok,Z,M]=gsim([last,nx],0,d,[],[],d,[],WY,CM,CW);
                if ok
                    if Z>dZ(nm,nx)||(Z==dZ(nm,nx)&&M>dM(nm,nx)),dZ(nm,nx)=Z;dM(nm,nx)=M;end
                end
            end
        end
    end
end

% ===== METHOD 3: DE =====
function [bZ,bM,bP,bW]=m_de(dist,all_xy,inter_idx,WY,WM,CM,CW)
    ng=10;ps=300;gn=150;F=0.8;CR=0.9;
    pop=zeros(ps,ng);fZ=zeros(ps,1);fM=zeros(ps,1);ff=false(ps,1);
    for i=1:ps,pop(i,:)=rchr(WM);[ff(i),fZ(i),fM(i)]=ed(pop(i,:),dist,all_xy,inter_idx,WY,WM,CM,CW);end
    bZ=-inf;bM=-inf;bP=[1,2];bW=[];bC=[];
    for gn=1:gn
        for i=1:ps
            c=randperm(ps,3);while any(c==i),c=randperm(ps,3);end
            a=c(1);b=c(2);c=c(3);donor=pop(a,:)+F*(pop(b,:)-pop(c,:));trial=pop(i,:);
            jr=randi(ng);for j=1:ng,if rand()<=CR||j==jr,trial(j)=donor(j);end;end
            trial=rpr(trial,WM);[ft,Zt,Mt]=ed(trial,dist,all_xy,inter_idx,WY,WM,CM,CW);
            if ft&&(~ff(i)||Zt>fZ(i)||(Zt==fZ(i)&&Mt>fM(i)))
                pop(i,:)=trial;ff(i)=1;fZ(i)=Zt;fM(i)=Mt;
            elseif ~ff(i)&&ft,pop(i,:)=trial;ff(i)=1;fZ(i)=Zt;fM(i)=Mt;end
            if ff(i)&&(fZ(i)>bZ||(fZ(i)==bZ&&fM(i)>bM)),bZ=fZ(i);bM=fM(i);bC=pop(i,:);end
        end
    end
    if ~isempty(bC),bP=dcd(bC,inter_idx,WM,dist);bW=wdcd(bC,inter_idx,WM,dist);end
end
function c=rchr(WM),c=zeros(1,10);c(1)=randi([0,6]);for i=1:6,c(1+i)=randi([1,5]);end;for wi=1:3,c(7+wi)=randi([0,WM(wi)]);end;end
function c=rpr(c,WM),c(1)=max(0,min(6,round(c(1))));for i=1:6,c(1+i)=max(1,min(5,round(c(1+i))));end;for wi=1:3,c(7+wi)=max(0,min(WM(wi),round(c(7+wi))));end;end
function [f,Z,M]=ed(c,dist,all_xy,inter_idx,WY,WM,CM,CW)
    sl=round(c(1));s=round(c(2:7));pid=[1];
    for i=1:sl,if s(i)>=1&&s(i)<=5,pid=[pid,inter_idx(s(i))];end;end;pid=[pid,2];
    cp=pid(1);for k=2:length(pid),if pid(k)~=cp(end),cp=[cp,pid(k)];end;end;pid=cp;
    m=length(pid)-2;tr=zeros(1,m+1);tt=0;
    for k=1:m+1,tr(k)=dist(pid(k),pid(k+1));tt=tt+tr(k);end;if tt>30,f=0;Z=0;M=0;return;end
    wa=[];ww=[];for k=2:m+1,pt=pid(k);if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end;end
    nw=length(wa);wr=round(c(8:10));
    if nw>0,wd=zeros(1,nw);for wi=1:nw,wh=ww(wi);wd(wi)=max(0,min(WM(wh),wr(wh)));end;else wd=[];end
    if tt+sum(wd)>30,f=0;Z=0;M=0;return;end
    [f,Z,M]=gsim(pid,m,tr,wa,ww,tt,wd,WY,CM,CW);
end
function p=dcd(c,inter_idx,WM,dist)
    sl=round(c(1));s=round(c(2:7));pid=[1];
    for i=1:sl,if s(i)>=1&&s(i)<=5,pid=[pid,inter_idx(s(i))];end;end;pid=[pid,2];
    cp=pid(1);for k=2:length(pid),if pid(k)~=cp(end),cp=[cp,pid(k)];end;end;p=cp;
end
function w=wdcd(c,inter_idx,WM,dist)
    p=dcd(c,inter_idx,WM,dist);m=length(p)-2;wa=[];ww=[];
    for k=2:m+1,pt=p(k);if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end;end
    nw=length(wa);wr=round(c(8:10));
    if nw>0,w=zeros(1,nw);for wi=1:nw,wh=ww(wi);w(wi)=max(0,min(WM(wh),wr(wh)));end;else w=[];end
end

% ===== METHOD 4: COLUMN GENERATION =====
function [bZ,bM,bP,bW]=m_cg(dist,all_xy,inter_idx,max_seq,WY,WM,CM,CW)
    poolP={};poolW={};poolZ=[];poolM=[];
    pid0=[1,2];[ok,Z0,M0]=gsim(pid0,0,dist(1,2),[],[],dist(1,2),[],WY,CM,CW);
    if ok,poolP{1}=pid0;poolW{1}=[];poolZ(1)=Z0;poolM(1)=M0;end
    bZ=-inf;bM=-inf;bP=[1,2];bW=[];nit=0;
    while true
        if isempty(poolZ),break;end
        [pi,idx]=max(poolZ);
        if pi>bZ||(pi==bZ&&poolM(idx)>bM),bZ=pi;bM=poolM(idx);bP=poolP{idx};bW=poolW{idx};end
        found=0;nit=nit+1;
        for sl=0:max_seq,if found,break;end;ns=5^sl;
            for si=1:ns,if found,break;end;s=zeros(1,sl);t=si-1;
                for j=sl:-1:1,s(j)=mod(t,5)+1;t=floor(t/5);end
                pid=[1,inter_idx(s),2];dup=0;
                for k=2:length(pid),if pid(k)==pid(k-1),dup=1;break;end;end;if dup,continue;end
                m=length(pid)-2;tr=zeros(1,m+1);tt=0;
                for k=1:m+1,tr(k)=dist(pid(k),pid(k+1));tt=tt+tr(k);end;if tt>30,continue;end
                wa=[];ww=[];for k=2:m+1,pt=pid(k);if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end;end
                nw=length(wa);
                if nw==0
                    [ok,Z,M]=gsim(pid,m,tr,wa,ww,tt,[],WY,CM,CW);
                    if ok&&Z>pi,poolP{end+1}=pid;poolW{end+1}=[];poolZ(end+1)=Z;poolM(end+1)=M;found=1;end
                else
                    sz=WM(ww)+1;nc=prod(sz);
                    for ci=1:nc,if found,break;end;wd=zeros(1,nw);t2=ci-1;
                        for j=nw:-1:1,wd(j)=mod(t2,sz(j));t2=floor(t2/sz(j));end
                        if tt+sum(wd)>30,continue;end
                        [ok,Z,M]=gsim(pid,m,tr,wa,ww,tt,wd,WY,CM,CW);
                        if ok&&Z>pi,poolP{end+1}=pid;poolW{end+1}=wd;poolZ(end+1)=Z;poolM(end+1)=M;found=1;end
                    end
                end
            end
        end
        if ~found,break;end
    end
    fprintf('  CG: %d cols, %d pricing iters\n',length(poolZ),nit);
end

% ===== METHOD 5: CONSTRAINT PROGRAMMING =====
function [bZ,bM,bP,bW]=m_cp(dist,all_xy,inter_idx,max_seq,WY,WM,CM,CW)
    bZ=-inf;bM=-inf;bP=[1,2];bW=[];nodes=0;
    [bZ,bM,bP,bW,nodes]=cp_search([1],0,[],[],bZ,bM,bP,bW,dist,inter_idx,WY,WM,CM,CW,nodes,30);
    fprintf('  CP: %d nodes\n',nodes);
end

function [bZ,bM,bP,bW,nodes]=cp_search(path,tsf,wa,ww,bZ,bM,bP,bW,dist,inter_idx,WY,WM,CM,CW,nodes,md)
    nodes=nodes+1;lp=path(end);
    if lp~=2,dE=dist(lp,2);rem=md-tsf;if rem<dE,return;end
        if 100+(rem-dE)*28<=bZ&&bZ>-inf,return;end
    end
    dE=dist(lp,2);
    if tsf+dE<=md
        fp=[path,2];m=length(fp)-2;tr=zeros(1,m+1);tt=0;
        for k=1:m+1,tr(k)=dist(fp(k),fp(k+1));tt=tt+tr(k);end
        nw=length(wa);
        if nw==0
            [ok,Z,M]=gsim(fp,m,tr,[],[],tt,[],WY,CM,CW);
            if ok&&(Z>bZ||(Z==bZ&&M>bM)),bZ=Z;bM=M;bP=fp;bW=[];end
        else
            sz=WM(ww)+1;nc=prod(sz);
            for ci=1:nc,wd=zeros(1,nw);t2=ci-1;
                for j=nw:-1:1,wd(j)=mod(t2,sz(j));t2=floor(t2/sz(j));end
                if tt+sum(wd)>md,continue;end
                [ok,Z,M]=gsim(fp,m,tr,wa,ww,tt,wd,WY,CM,CW);
                if ok&&(Z>bZ||(Z==bZ&&M>bM)),bZ=Z;bM=M;bP=fp;bW=wd;end
            end
        end
    end
    for ni=1:5,np=inter_idx(ni);
        if np==lp,continue;end  % skip adjacent duplicates only
        d=dist(lp,np);if tsf+d>md,continue;end
        dE2=dist(np,2);if tsf+d+dE2>md,continue;end
        np2=[path,np];nt=tsf+d;nwa=wa;nww=ww;
        if np>=3&&np<=5,nwa(end+1)=length(np2);nww(end+1)=np-2;end
        [bZ,bM,bP,bW,nodes]=cp_search(np2,nt,nwa,nww,bZ,bM,bP,bW,dist,inter_idx,WY,WM,CM,CW,nodes,md);
    end
end

% ===== SHARED GREEDY SIMULATION =====
function [feasible,Zf,Mf]=gsim(pid,m,travel,work_at,work_wh,tt,wdays,WY,CM,CW)
    T=tt+sum(wdays);cO=zeros(1,T);cH=zeros(1,T);cF=zeros(1,T);
    zG=zeros(1,T);isSup=false(1,T);
    day=0;
    for k=1:m+1
        d=travel(k);
        for dd=1:d
            day=day+1;cO(day)=CM(1);cH(day)=CM(2);cF(day)=CM(3);
            if dd==d,to_pt=pid(k+1);if to_pt==6||to_pt==7,isSup(day)=true;end;end
        end
        if ~isempty(work_at),wk=find(work_at==k+1,1);
            if ~isempty(wk)&&wdays(wk)>0
                for w=1:wdays(wk),day=day+1;cO(day)=CW(1);cH(day)=CW(2);cF(day)=CW(3);zG(day)=WY(work_wh(wk));end
            end
        end
    end
    O=35;H=45;F=30;M=240;
    for t=1:T
        if isSup(t)
            ns=T+1;for tt2=t+1:T,if isSup(tt2),ns=tt2;break;end;end
            nO=0;nH=0;nF=0;
            for tt2=t+1:ns,if tt2>T,break;end;nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);end
            sp=120-(O+H+F);bO=max(0,nO-O);bH=max(0,nH-H);bF=max(0,nF-F);
            if bO+bH+bF>sp,feasible=false;Zf=0;Mf=0;return;end
            if ns>T&&(O+bO<nO||H+bH<nH||F+bF<nF),feasible=false;Zf=0;Mf=0;return;end
            cost=bO*2+bH*1+bF*2;if cost>M,feasible=false;Zf=0;Mf=0;return;end
            O=O+bO;H=H+bH;F=F+bF;M=M-cost;
        else
            O=O-cO(t);H=H-cH(t);F=F-cF(t);
        end
        if O<0||H<0||F<0||M<0,feasible=false;Zf=0;Mf=0;return;end
        if O+H+F>120,feasible=false;Zf=0;Mf=0;return;end
    end
    Zf=100+sum(zG);Mf=M;feasible=true;
end
