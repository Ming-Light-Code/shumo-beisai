import os, sys
# Use environment to find desktop
desktop = os.path.join(os.environ['USERPROFILE'], 'Desktop')

# ===== COLUMN GENERATION CODE =====
cg_code = r"""function solve_cg()
% solve_cg.m - Column Generation for Task 1
% 2026 SEU Math Modeling Competition, Problem B

B=[1,5];E=[10,5];CM=[2,3,2];CW=[5,4,3];WY=[20,15,28];WM=[4,5,3];
O0=35;H0=45;F0=30;M0=240;Z0=100;MAX_DAYS=30;LOAD_LIMIT=120;
all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];
dist=zeros(7);for i=1:7,for j=1:7,dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));end;end
inter_idx=[3 4 5 6 7];n_inter=5;max_seq=min(7,MAX_DAYS-dist(1,2));
names={'B','E','W1','W2','W3','S1','S2'};

fprintf('========================================\n');
fprintf('  Column Generation for Task 1\n');
fprintf('========================================\n\n');
fprintf('Theory:\n');
fprintf('  Master Problem: max sum_r Z(r)*a_r  s.t. sum_r a_r = 1\n');
fprintf('  Dual variable: pi = Z_current (single convexity constraint)\n');
fprintf('  Pricing: find column r with reduced cost Z(r)-pi > 0\n');
fprintf('  Stop when no improving column exists -> optimal.\n\n');

poolP={};poolW={};poolZ=[];poolM=[];
pid0=[1,2];[ok,Z0,M0]=gsim(pid0,0,dist(1,2),[],[],dist(1,2),[]);
if ok,poolP{1}=pid0;poolW{1}=[];poolZ(1)=Z0;poolM(1)=M0;end
bZ=-inf;bM=-inf;bP=[1,2];bW=[];nit=0;ncg=0;

fprintf('Iter |  pi(dual) |  New Z  |  New M  | Columns\n');
fprintf('-----|-----------|---------|---------|--------\n');

while true
    [pi,idx]=max(poolZ);
    if pi>bZ||(pi==bZ&&poolM(idx)>bM),bZ=pi;bM=poolM(idx);bP=poolP{idx};bW=poolW{idx};end
    found=0;nit=nit+1;
    for sl=0:max_seq,if found,break;end;ns=n_inter^sl;
        for si=1:ns,if found,break;end
            s=zeros(1,sl);t=si-1;for j=sl:-1:1,s(j)=mod(t,n_inter)+1;t=floor(t/n_inter);end
            pid=[1,inter_idx(s),2];dup=0;
            for k=2:length(pid),if pid(k)==pid(k-1),dup=1;break;end;end;if dup,continue;end
            m=length(pid)-2;tr=zeros(1,m+1);tt=0;
            for k=1:m+1,tr(k)=dist(pid(k),pid(k+1));tt=tt+tr(k);end;if tt>MAX_DAYS,continue;end
            wa=[];ww=[];for k=2:m+1,pt=pid(k);if pt>=3&&pt<=5,wa(end+1)=k;ww(end+1)=pt-2;end;end
            nw=length(wa);
            if nw==0
                [ok,Z,M]=gsim(pid,m,tr,wa,ww,tt,[]);
                if ok&&Z>pi,poolP{end+1}=pid;poolW{end+1}=[];poolZ(end+1)=Z;poolM(end+1)=M;ncg=ncg+1;
                    fprintf(' %3d | %9d | %7d | %7d | %6d\n',nit,pi,Z,M,length(poolZ));found=1;end
            else
                sz=WM(ww)+1;nc=prod(sz);
                for ci=1:nc,if found,break;end;wd=zeros(1,nw);t2=ci-1;
                    for j=nw:-1:1,wd(j)=mod(t2,sz(j));t2=floor(t2/sz(j));end
                    if tt+sum(wd)>MAX_DAYS,continue;end
                    [ok,Z,M]=gsim(pid,m,tr,wa,ww,tt,wd);
                    if ok&&Z>pi,poolP{end+1}=pid;poolW{end+1}=wd;poolZ(end+1)=Z;poolM(end+1)=M;ncg=ncg+1;
                        fprintf(' %3d | %9d | %7d | %7d | %6d\n',nit,pi,Z,M,length(poolZ));found=1;end
                end
            end
        end
    end
    if ~found,break;end
end

fprintf('-----|-----------|---------|---------|--------\n');
fprintf('\n===== OPTIMAL SOLUTION =====\n');
fprintf('Z = %d\n',bZ);fprintf('M = %d\n',bM);
fprintf('Path: ');for i=1:length(bP),fprintf('%s ',names{bP(i)});end;fprintf('\n');
fprintf('Columns generated: %d\n',ncg);
fprintf('Pricing iterations: %d\n',nit);
tt=0;for k=1:length(bP)-1,tt=tt+dist(bP(k),bP(k+1));end
fprintf('Travel: %d days, Work: %d days, Total: %d days\n',tt,sum(bW),tt+sum(bW));
fprintf('\nDone.\n');
end

function [f,Zf,Mf]=gsim(pid,m,travel,wa,ww,tt,wdays)
T=tt+sum(wdays);cO=zeros(1,T);cH=zeros(1,T);cF=zeros(1,T);zG=zeros(1,T);isSup=false(1,T);
day=0;
for k=1:m+1
    d=travel(k);
    for dd=1:d
        day=day+1;cO(day)=2;cH(day)=3;cF(day)=2;
        if dd==d,to_pt=pid(k+1);if to_pt==6||to_pt==7,isSup(day)=true;end;end
    end
    if ~isempty(wa),wk=find(wa==k+1,1);
        if ~isempty(wk)&&wdays(wk)>0
            yld=[20,15,28];for w=1:wdays(wk),day=day+1;cO(day)=5;cH(day)=4;cF(day)=3;zG(day)=yld(ww(wk));end
        end
    end
end
O=35;H=45;F=30;M=240;
for t=1:T
    if isSup(t)
        ns=T+1;for tt2=t+1:T,if isSup(tt2),ns=tt2;break;end;end
        nO=0;nH=0;nF=0;for tt2=t+1:ns,if tt2>T,break;end;nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);end
        sp=120-(O+H+F);bO=max(0,nO-O);bH=max(0,nH-H);bF=max(0,nF-F);
        if bO+bH+bF>sp,f=false;Zf=0;Mf=0;return;end
        if ns>T&&(O+bO<nO||H+bH<nH||F+bF<nF),f=false;Zf=0;Mf=0;return;end
        cost=bO*2+bH*1+bF*2;if cost>M,f=false;Zf=0;Mf=0;return;end
        O=O+bO;H=H+bH;F=F+bF;M=M-cost;
    else
        O=O-cO(t);H=H-cH(t);F=F-cF(t);
    end
    if O<0||H<0||F<0||M<0,f=false;Zf=0;Mf=0;return;end
    if O+H+F>120,f=false;Zf=0;Mf=0;return;end
end
Zf=100+sum(zG);Mf=M;f=true;
end
"""

# ===== CONSTRAINT PROGRAMMING CODE =====
cp_code = r"""function solve_cp()
% solve_cp.m - Constraint Programming for Task 1
% 2026 SEU Math Modeling Competition, Problem B

B=[1,5];E=[10,5];CM=[2,3,2];CW=[5,4,3];WY=[20,15,28];WM=[4,5,3];MAX_DAYS=30;
all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];
dist=zeros(7);for i=1:7,for j=1:7,dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));end;end
inter_idx=[3 4 5 6 7];names={'B','E','W1','W2','W3','S1','S2'};

fprintf('========================================\n');
fprintf('  Constraint Programming for Task 1\n');
fprintf('========================================\n\n');
fprintf('CP Mechanisms:\n');
fprintf('  1. Domain Reduction: work days bounded by max_consec\n');
fprintf('  2. Forward Checking: verify reachability to E\n');
fprintf('  3. Bound Pruning: upper bound on achievable Z\n');
fprintf('  4. Backtracking: on failure, try next value\n\n');

fprintf('Search tree trace (first 20 nodes):\n');
fprintf('  Node | Depth |   Action  | UpperBound | BestZ\n');
fprintf('  -----|-------|-----------|------------|------\n');

bZ=-inf;bM=-inf;bP=[1,2];bW=[];nodes=0;
[bZ,bM,bP,bW,nodes]=cp_search([1],0,[],[],bZ,bM,bP,bW,dist,inter_idx,WY,WM,CM,CW,nodes,0,20);

fprintf('  -----|-------|-----------|------------|------\n');
fprintf('\n===== OPTIMAL SOLUTION =====\n');
fprintf('Z = %d\n',bZ);fprintf('M = %d\n',bM);
fprintf('Path: ');for i=1:length(bP),fprintf('%s ',names{bP(i)});end;fprintf('\n');
fprintf('Nodes explored: %d\n',nodes);
tt=0;for k=1:length(bP)-1,tt=tt+dist(bP(k),bP(k+1));end
fprintf('Travel: %d days, Work: %d days, Total: %d days\n',tt,sum(bW),tt+sum(bW));
fprintf('\nDone.\n');
end

function [bZ,bM,bP,bW,nodes]=cp_search(path,tsf,wa,ww,bZ,bM,bP,bW,dist,inter_idx,WY,WM,CM,CW,nodes,depth,max_trace)
    nodes=nodes+1;lp=path(end);
    if lp~=2
        dE=dist(lp,2);rem=30-tsf;if rem<dE,return;end
        ub=100+(rem-dE)*28;
        if nodes<=max_trace,fprintf('  %4d | %5d | %9s | %10d | %5d\n',nodes,depth,'branch',ub,bZ);end
        if ub<=bZ&&bZ>-inf,return;end
    end
    dE=dist(lp,2);
    if tsf+dE<=30
        fp=[path,2];m=length(fp)-2;tr=zeros(1,m+1);tt=0;
        for k=1:m+1,tr(k)=dist(fp(k),fp(k+1));tt=tt+tr(k);end
        nw=length(wa);
        if nw==0
            [ok,Z,M]=gsim(fp,m,tr,[],[],tt,[]);
            if ok&&(Z>bZ||(Z==bZ&&M>bM)),bZ=Z;bM=M;bP=fp;bW=[];
                if nodes<=max_trace,fprintf('  %4d | %5d | %9s | %10d | %5d *** NEW BEST\n',nodes,depth,'leaf',0,bZ);end
            end
        else
            sz=WM(ww)+1;nc=prod(sz);
            for ci=1:nc,wd=zeros(1,nw);t2=ci-1;
                for j=nw:-1:1,wd(j)=mod(t2,sz(j));t2=floor(t2/sz(j));end
                if tt+sum(wd)>30,continue;end
                [ok,Z,M]=gsim(fp,m,tr,wa,ww,tt,wd);
                if ok&&(Z>bZ||(Z==bZ&&M>bM)),bZ=Z;bM=M;bP=fp;bW=wd;
                    if nodes<=max_trace,fprintf('  %4d | %5d | %9s | %10d | %5d *** NEW BEST\n',nodes,depth,'leaf',0,bZ);end
                end
            end
        end
    end
    for ni=1:5,np=inter_idx(ni);
        if np==lp,continue;end
        d=dist(lp,np);if tsf+d>30,continue;end
        dE2=dist(np,2);if tsf+d+dE2>30,continue;end
        np2=[path,np];nt=tsf+d;nwa=wa;nww=ww;
        if np>=3&&np<=5,nwa(end+1)=length(np2);nww(end+1)=np-2;end
        [bZ,bM,bP,bW,nodes]=cp_search(np2,nt,nwa,nww,bZ,bM,bP,bW,dist,inter_idx,WY,WM,CM,CW,nodes,depth+1,max_trace);
    end
end

function [f,Zf,Mf]=gsim(pid,m,travel,wa,ww,tt,wdays)
T=tt+sum(wdays);cO=zeros(1,T);cH=zeros(1,T);cF=zeros(1,T);zG=zeros(1,T);isSup=false(1,T);
day=0;
for k=1:m+1
    d=travel(k);
    for dd=1:d
        day=day+1;cO(day)=2;cH(day)=3;cF(day)=2;
        if dd==d,to_pt=pid(k+1);if to_pt==6||to_pt==7,isSup(day)=true;end;end
    end
    if ~isempty(wa),wk=find(wa==k+1,1);
        if ~isempty(wk)&&wdays(wk)>0
            yld=[20,15,28];for w=1:wdays(wk),day=day+1;cO(day)=5;cH(day)=4;cF(day)=3;zG(day)=yld(ww(wk));end
        end
    end
end
O=35;H=45;F=30;M=240;
for t=1:T
    if isSup(t)
        ns=T+1;for tt2=t+1:T,if isSup(tt2),ns=tt2;break;end;end
        nO=0;nH=0;nF=0;for tt2=t+1:ns,if tt2>T,break;end;nO=nO+cO(tt2);nH=nH+cH(tt2);nF=nF+cF(tt2);end
        sp=120-(O+H+F);bO=max(0,nO-O);bH=max(0,nH-H);bF=max(0,nF-F);
        if bO+bH+bF>sp,f=false;Zf=0;Mf=0;return;end
        if ns>T&&(O+bO<nO||H+bH<nH||F+bF<nF),f=false;Zf=0;Mf=0;return;end
        cost=bO*2+bH*1+bF*2;if cost>M,f=false;Zf=0;Mf=0;return;end
        O=O+bO;H=H+bH;F=F+bF;M=M-cost;
    else
        O=O-cO(t);H=H-cH(t);F=F-cF(t);
    end
    if O<0||H<0||F<0||M<0,f=false;Zf=0;Mf=0;return;end
    if O+H+F>120,f=false;Zf=0;Mf=0;return;end
end
Zf=100+sum(zG);Mf=M;f=true;
end
"""

# Write files using binary-safe paths
cg_dir = os.path.join(desktop, '\u5217\u751f\u6210')  # Unicode escapes for Chinese
cp_dir = os.path.join(desktop, '\u7ea6\u675f\u89c4\u5212')

# Actually, let me just use the actual Chinese chars - they work in Python 3 with proper encoding
# The earlier issue was with the OSError which might be from something else

cg_dir = os.path.join(desktop, '列生成')
cp_dir = os.path.join(desktop, '约束规划')

try:
    os.makedirs(cg_dir, exist_ok=True)
    os.makedirs(cp_dir, exist_ok=True)
except:
    pass

cg_path = os.path.join(cg_dir, 'solve_cg.m')
cp_path = os.path.join(cp_dir, 'solve_cp.m')

with open(cg_path, 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(cg_code)
print(f'Written solve_cg.m ({len(cg_code)} chars)')

with open(cp_path, 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(cp_code)
print(f'Written solve_cp.m ({len(cp_code)} chars)')

print('Done.')