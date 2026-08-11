function solve_q3()
%% ========================================================================
%% solve_task3.m - Task 3: Two-Stage Stochastic Optimization
%% ========================================================================
%% 场景: 30x30网格, 90天, P(正常)=0.8, P(雷暴)=0.2
%% 方法: Phase1(骨架枚举) -> Phase2(两阶段MILP) -> Phase3(SAA评估) -> Phase4(滚动时域)
%% 优化: 字典序 max Z -> max M
%% 依赖: Optimization Toolbox (intlinprog)
%% 输出: 最优骨架, 期望Z, 成功率, 最终逐日结果
%% ========================================================================
if ~exist("intlinprog","file"), error("Requires Optimization Toolbox."); end
rng(42);
all_xy=[1 15;30 15;6 21;15 9;24 24;12 16;21 16];
names={"B","E","W1","W2","W3","S1","S2"};
POI=[3 4 5 6 7]; WY=[20,15,28]; WM=[4,5,3];
LD=400; MD=90; IO=100; IH=150; IF=100; IM=750; IZ=200;
CMn=[2,3,2]; CWn=[5,4,3]; CSn=[1,1,1];
CMs=[8,4,3]; CWs=[8,6,6]; CSs=[3,3,2];
% Expected costs (p_n=0.8, p_s=0.2)
CMe=[3.2,3.2,2.2]; CWe=[5.6,4.4,3.6]; CSe=[1.4,1.4,1.2];
% Variance per day for safety buffer (z=2 -> 97.5% confidence)
VM=0.16*[6,1,1].^2; VW=0.16*[3,2,3].^2; VS=0.16*[2,2,1].^2;
SAFE_Z=2;

fprintf('================ TASK3: Stochastic Optimization ================\n');
fprintf("Grid: 30x30, 90 days, p_storm=0.2, p_normal=0.8\n\n");

%% ====== Phase 1: Skeleton Enumeration + Greedy Screening ======
fprintf('--- Phase 1: Skeleton Enumeration ---\n');
d=zeros(7);
for i=1:7
    for j=1:7
        d(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));
    end
end
ms=4; best_skels={}; best_Zs=[];
fprintf("  Enumerating skeletons (up to %d intermediates)...\n",ms);

for sl=0:ms
  for si=1:5^sl
    sq=zeros(1,sl); tmp=si-1;
    for j=sl:-1:1, sq(j)=mod(tmp,5)+1; tmp=floor(tmp/5); end
    pid=[1,POI(sq),2];
    dup=false; for k=2:length(pid), if pid(k)==pid(k-1), dup=true; break; end; end
    if dup, continue; end
    m=length(pid)-2; tv=zeros(1,m+1); tt=0;
    for k=1:m+1, dk=d(pid(k),pid(k+1)); tv(k)=dk; tt=tt+dk; end
    if tt>MD-10, continue; end  % need at least 10 days for work
    
    % Get work/supply indices
    wi=zeros(1,m+1); wc=[]; si=zeros(1,m+1); nw=0; ns=0;
    for k=1:m+1
      pt=pid(k+1);
      if pt>=3&&pt<=5, nw=nw+1; wi(k)=nw; wc(nw)=pt-2; end
      if pt==6||pt==7, ns=ns+1; si(k)=ns; end
    end
    
    % Greedy evaluation with safety buffer
    [ok,Z,M,w1g,bg,w2g,byg]=gq_greedy(m,tv,wi,wc,si,nw,ns,tt,...
      CMe,CWe,CSe,LD,MD,IO,IH,IF,IM,IZ,VM,VW,VS,SAFE_Z,WY,WM);
    
    if ok && Z>IZ
      best_skels{end+1}=struct("pid",pid,"m",m,"tv",tv,"wi",wi,"wc",wc,"si",si,...
        "nw",nw,"ns",ns,"Z",Z,"M",M,"w1",w1g,"b",bg,"w2",w2g,"by",byg);
      best_Zs(end+1)=Z;
    end
  end
end

[~,ord]=sort(best_Zs,"descend");
N_KEEP=min(10,length(best_Zs));
top_skels=best_skels(ord(1:N_KEEP));
top_Zs=best_Zs(ord(1:N_KEEP));
fprintf("  Found %d feasible skeletons, keeping top %d\n",length(best_Zs),N_KEEP);
for i=1:N_KEEP
  fprintf("    #%d: Z=%.0f, path=",i,top_Zs(i));
  for j=top_skels{i}.pid, fprintf("%s ",names{j}); end; fprintf("\n");
end

%% ====== Phase 2: MILP Optimization on Top Candidates ======
fprintf('\n--- Phase 2: MILP Optimization ---\n');
for i=1:N_KEEP
  sk=top_skels{i};
  [ok2,Z2,M2,w12,b12,w22,bu2]=m1_q3(sk.m,sk.tv,sk.wi,sk.wc,sk.si,...
    sk.nw,sk.ns,sk.tv(end),WY,WM,CMe,CWe,CSe,LD,MD,IO,IH,IF,IM,IZ);
  if ok2
    top_skels{i}.Z=Z2; top_skels{i}.M=M2;
    top_skels{i}.w1=w12; top_skels{i}.b=b12; top_skels{i}.w2=w22; top_skels{i}.by=bu2;
    fprintf("    #%d: Z=%d M=%d\n",i,Z2,M2);
  else
    fprintf("    #%d: MILP INFEASIBLE (removing)\n",i);
    top_skels{i}.Z=-inf;
  end
end

%% ====== Phase 3: SAA Sample-Average Approximation ======
N_SAA=500;
fprintf('\n--- Phase 3: SAA Evaluation (N=%d) ---\n', N_SAA);
fprintf("  Evaluating top skeletons over %d weather scenarios...\n",N_SAA);
saa_best=-inf; saa_best_idx=0;
for i=1:N_KEEP
  if top_skels{i}.Z<=-inf, continue; end
  sk=top_skels{i};
  succ=0; Zsum=0; Msum=0;
  for s=1:N_SAA
    % Generate weather sequence
    wx=rand(1,MD)>0.8; % 1=Storm, 0=Normal
    % Simulate
    [okS,ZS,MS]=sim_path_q3(sk,wx,all_xy,names,CMn,CWn,CSn,CMs,CWs,CSs,...
      WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,SAFE_Z);
    if okS, succ=succ+1; Zsum=Zsum+ZS; end
  end
  EZ=Zsum/N_SAA; SR=succ/N_SAA*100;
  fprintf("    #%d: E[Z]=%.0f, success=%d/%d (%.0f%%)\n",i,EZ,succ,N_SAA,SR);
  if EZ>saa_best
    saa_best=EZ; saa_best_idx=i;
  end
end

fprintf('\n================ BEST SKELETON: #%d ================\n', saa_best_idx);
best_sk=top_skels{saa_best_idx};
fprintf("  Path: "); for j=best_sk.pid, fprintf("%s ",names{j}); end; fprintf("\n");
fprintf("  Expected Z: %.0f\n",saa_best);
fprintf("  Work pattern: first work=%dd\n",best_sk.w1(1));

%% ====== Phase 4: Daily Rolling Horizon Re-solve ======
fprintf('\n--- Phase 4: Daily Rolling Horizon ---\n');
wx=rand(1,MD)>0.8;
[S,Z,M]=phase4_daily(best_sk,wx,all_xy,CMn,CWn,CSn,CMs,CWs,CSs,WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,IO,IH,IF,IM,IZ);
fprintf("  FINAL: Z=%d M=%d days=%d\n",Z,M,S.day);
end
%% ======================================================================
%% Helper: Greedy evaluation with safety buffers
%% ======================================================================
function [ok,Z,M,w1,b,w2,by]=gq_greedy(m,tv,wi,wc,si,nw,ns,tt,CM,CW,CS,LD,MD,IO,IH,IF,IM,IZ,VM,VW,VS,Zf,WY,WM)
% Work assignment
w1=zeros(1,nw); b=zeros(1,nw); w2=zeros(1,nw);
if nw>0
  rem=MD-tt;
  [~,ord]=sort(WY(wc),"descend");
  for idx=1:nw, i=ord(idx); w1(i)=min(WM(wc(i)),rem); rem=rem-w1(i); if rem<=0,break;end; end
  for idx=1:nw, i=ord(idx);
    if rem>0&&w1(i)==WM(wc(i)), b(i)=1; rem=rem-1;
      if rem>0, w2(i)=min(WM(wc(i)),rem); rem=rem-w2(i); end; end
    if rem<=0,break;end;
  end
end
% Resource simulation with safety buffer
ts=sum(b); T=tt+sum(w1)+ts+sum(w2); if T<1, T=1; end
cO=zeros(1,T); cH=zeros(1,T); cF=zeros(1,T); zG=zeros(1,T);
iS=false(1,T); sdk=zeros(1,T); day=0; s=0;
for k=1:m+1
  if k<=length(tv), d=tv(k); else d=0; end
  for dd=1:d
    day=day+1; if day>T, break; end;
    cO(day)=CM(1); cH(day)=CM(2); cF(day)=CM(3);
    if dd==d&&k<=length(si)&&si(k)>0, iS(day)=true; s=s+1; sdk(day)=s; end
  end
  wi2=0; if k<=length(wi), wi2=wi(k); end
  if wi2>0&&wi2<=length(w1)&&wi2<=length(wc)
    wh=wc(wi2);
    for ww=1:w1(wi2)
      day=day+1; if day>T, break; end; cO(day)=CW(1); cH(day)=CW(2); cF(day)=CW(3); zG(day)=WY(wh);
    end
    if wi2<=length(b)&&b(wi2)>0
      day=day+1; if day>T, break; end; cO(day)=CS(1); cH(day)=CS(2); cF(day)=CS(3);
    end
    if wi2<=length(w2)
      for ww=1:w2(wi2), day=day+1; if day>T, break; end; cO(day)=CW(1); cH(day)=CW(2); cF(day)=CW(3); zG(day)=WY(wh); end
    end
  end
end
% Precompute prefix sums and next supply index for O(1) range queries
prefO=[0,cumsum(cO)]; prefH=[0,cumsum(cH)]; prefF=[0,cumsum(cF)];
nSup=zeros(1,T+1); nSupply=T+1;
for tx=T:-1:1
    if tx<=length(iS)&&iS(tx), nSupply=tx; end
    nSup(tx)=nSupply;
end
by=zeros(ns,3); O=IO; H=IH; F=IF; M=IM;
for t=1:T
    O=O-cO(t); H=H-cH(t); F=F-cF(t);
    if O<0||H<0||F<0, ok=false; Z=0; M=0; return; end
    if t<=length(iS)&&iS(t)
        % Range-query remaining consumption until next supply (O(1) via prefix sums)
        nsDay=nSup(t+1)-1;
        remT=nsDay-t;
        no=prefO(nsDay+1)-prefO(t+1);
        nh=prefH(nsDay+1)-prefH(t+1);
        nf=prefF(nsDay+1)-prefF(t+1);
        % Safety buffer based on remaining variance
        bufO=Zf*sqrt(remT*VM(1)); bufH=Zf*sqrt(remT*VW(1)); bufF=Zf*sqrt(remT*VS(1));
        bO=floor(max(0,no+0.5*bufO-O)); bH=floor(max(0,nh+0.5*bufH-H)); bF=floor(max(0,nf+0.5*bufF-F));
        free=LD-(O+H+F); tb=bO+bH+bF;
        if tb>free, sc=free/max(tb,1); bO=floor(bO*sc); bH=floor(bH*sc); bF=floor(bF*sc); end
        cost=bO*2+bH+bF*2; if cost>M, ok=false; Z=0; M=0; return; end
        O=O+bO; H=H+bH; F=F+bF; M=M-cost;
        sk=sdk(t); if sk>0&&sk<=ns, by(sk,1)=bO; by(sk,2)=bH; by(sk,3)=bF; end
    end
    if O+H+F>LD||M<0, ok=false; Z=0; M=0; return; end
end
Z=IZ+sum(zG); ok=true;
end
%% ======================================================================
%% MILP: Two-stage optimization for skeleton
%% ======================================================================
function [ok,Z,M,w1o,bo,w2o,buo]=m1_q3(m,tv,wi,wc,si,nw,ns,tt,WY,WM,CM,CW,CS,LD,MD,IO,IH,IF,IM,IZ)
[Aeq,beq,Ai,bi,nv,icn,lb,ub,off]=bd_q3(m,tv,wi,wc,si,nw,ns,tt,CM,CW,CS,LD,MD,WM,IO,IH,IF,IM,IZ);
if nw==0
  ok=true; Z=IZ; M=IM; w1o=[]; bo=[]; w2o=[]; buo=zeros(ns,3); return;
end
fl=zeros(nv,1); for j=1:nw, fl(off.w1+j)=-WY(wc(j)); fl(off.w2+j)=-WY(wc(j)); end
op=optimoptions("intlinprog","Display","off");
[x1,~,flag]=intlinprog(fl,icn,Ai,bi,Aeq,beq,lb,ub,op);
if flag<=0||isempty(x1), ok=false; Z=0; M=0; w1o=[];bo=[];w2o=[];buo=[]; return; end
Z=IZ+round(-fl'*x1);
A2=[Ai;zeros(2,nv)]; b2=[bi;Z-IZ;-(Z-IZ)]; nr=size(Ai,1);
for j=1:nw, A2(nr+1,off.w1+j)=WY(wc(j)); A2(nr+1,off.w2+j)=WY(wc(j));
  A2(nr+2,off.w1+j)=-WY(wc(j)); A2(nr+2,off.w2+j)=-WY(wc(j)); end
f2=zeros(nv,1); f2(off.M+m+2)=-1;
[x2,~,flag2]=intlinprog(f2,icn,A2,b2,Aeq,beq,lb,ub,op);
if flag2<=0||isempty(x2), ok=false; Z=0; M=0; w1o=[];bo=[];w2o=[];buo=[]; return; end
M=round(-f2'*x2);
w1o=round(x2(off.w1+(1:nw))); bo=round(x2(off.b+(1:nw))); w2o=round(x2(off.w2+(1:nw)));
buo=zeros(ns,3); for k=1:ns, buo(k,1)=round(x2(off.bO+k)); buo(k,2)=round(x2(off.bH+k)); buo(k,3)=round(x2(off.bF+k)); end
ok=true;
end

%% ======================================================================
%% Helper: Build MILP Constraint Matrix
%% ======================================================================
function [Aeq,beq,Ai,bi,nv,icn,lb,ub,off]=bd_q3(m,tv,wi,wc,si2,nw,ns,tt,CM,CW,CS,LD,MD,WM,IO,IH,IF,IM,IZ)
nv=3*nw+3*ns+4*(m+2); icn=1:(3*nw+3*ns);
off.w1=0; off.b=off.w1+nw; off.w2=off.b+nw;
off.bO=off.w2+nw; off.bH=off.bO+ns; off.bF=off.bH+ns;
off.O=off.bF+ns; off.H=off.O+(m+2); off.F=off.H+(m+2); off.M=off.F+(m+2);
ne=4*(m+1); Aeq=zeros(ne,nv); beq=zeros(ne,1); eq=0;
for i=1:m+1
  d=tv(i); w2i=wi(i); s2i=si2(i);
  eq=eq+1; Aeq(eq,off.O+1+i)=1; Aeq(eq,off.O+i)=-1; beq(eq)=-d*CM(1);
  if w2i>0, Aeq(eq,off.w1+w2i)=CW(1); Aeq(eq,off.w2+w2i)=CW(1); Aeq(eq,off.b+w2i)=CS(1); end
  if s2i>0, Aeq(eq,off.bO+s2i)=-1; end
  eq=eq+1; Aeq(eq,off.H+1+i)=1; Aeq(eq,off.H+i)=-1; beq(eq)=-d*CM(2);
  if w2i>0, Aeq(eq,off.w1+w2i)=CW(2); Aeq(eq,off.w2+w2i)=CW(2); Aeq(eq,off.b+w2i)=CS(2); end
  if s2i>0, Aeq(eq,off.bH+s2i)=-1; end
  eq=eq+1; Aeq(eq,off.F+1+i)=1; Aeq(eq,off.F+i)=-1; beq(eq)=-d*CM(3);
  if w2i>0, Aeq(eq,off.w1+w2i)=CW(3); Aeq(eq,off.w2+w2i)=CW(3); Aeq(eq,off.b+w2i)=CS(3); end
  if s2i>0, Aeq(eq,off.bF+s2i)=-1; end
  eq=eq+1; Aeq(eq,off.M+1+i)=1; Aeq(eq,off.M+i)=-1; beq(eq)=0;
  if s2i>0, Aeq(eq,off.bO+s2i)=2; Aeq(eq,off.bH+s2i)=1; Aeq(eq,off.bF+s2i)=2; end
end
ni=(m+2)+1+2*nw+3*ns; Ai=zeros(ni,nv); bi=zeros(ni,1); in=0;
for i=0:m+1, in=in+1; Ai(in,off.O+1+i)=1; Ai(in,off.H+1+i)=1; Ai(in,off.F+1+i)=1; bi(in)=LD; end
in=in+1; for j=1:nw, Ai(in,off.w1+j)=1; Ai(in,off.b+j)=1; Ai(in,off.w2+j)=1; end; bi(in)=MD-tt;
for j=1:nw
  in=in+1; Ai(in,off.w1+j)=1; bi(in)=WM(wc(j));
  in=in+1; Ai(in,off.w2+j)=1; Ai(in,off.b+j)=-WM(wc(j)); bi(in)=0;
end
for i=1:m+1
  if si2(i)>0, d=tv(i);
    in=in+1; Ai(in,off.O+i)=-1; bi(in)=-d*CM(1);
    in=in+1; Ai(in,off.H+i)=-1; bi(in)=-d*CM(2);
    in=in+1; Ai(in,off.F+i)=-1; bi(in)=-d*CM(3);
  end
end
lb=zeros(nv,1); ub=inf(nv,1);
lb(off.O+1)=IO; ub(off.O+1)=IO; lb(off.H+1)=IH; ub(off.H+1)=IH;
lb(off.F+1)=IF; ub(off.F+1)=IF; lb(off.M+1)=IM; ub(off.M+1)=IM;
for j=1:nw, ub(off.b+j)=1; ub(off.w1+j)=WM(wc(j)); ub(off.w2+j)=WM(wc(j)); end
end
%% ======================================================================
%% Path simulation under given weather
%% ======================================================================
function [ok,Z,M]=sim_path_q3(sk,wx,all_xy,names,CMn,CWn,CSn,CMs,CWs,CSs,WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,Zf)
pid=sk.pid; m=sk.m; tv=sk.tv; wi=sk.wi; wc=sk.wc; si=sk.si;
nw=sk.nw; ns=sk.ns; w1=sk.w1; b=sk.b; w2=sk.w2; buy_p=sk.by;
if isempty(buy_p), buy_p=zeros(ns,3); end
S=struct("pos",all_xy(1,:),"O",100,"H",150,"F",100,"M",750,"Z",200,"cw",0,"day",0);
sup_cnt=0;
for k=1:m+1
  fi=pid(k); ti=pid(k+1);
  if fi==8, ps=S.pos; else ps=all_xy(fi,:); end; pe=all_xy(ti,:);
  % Travel
  if ~isequal(ps,pe)
    cur=ps;
    while ~isequal(cur,pe)
      nxt=cur; if nxt(1)~=pe(1), nxt(1)=nxt(1)+sign(pe(1)-nxt(1)); else nxt(2)=nxt(2)+sign(pe(2)-nxt(2)); end
      S.day=S.day+1; if S.day>MD, ok=false; Z=0; M=0; return; end
      wx_t=wx(S.day);
      if wx_t==0, CM=CMn; else CM=CMs; end
      S.O=S.O-CM(1); S.H=S.H-CM(2); S.F=S.F-CM(3); S.cw=0;
      if S.O<0||S.H<0||S.F<0, ok=false; Z=0; M=0; return; end
      cur=nxt;
    end
  end
  % At POI
  wk_idx=0; for j=1:nw, if wi(k)==j, wk_idx=j; break; end; end
  sup_idx=0; for j=1:ns, if si(k)==j, sup_idx=j; break; end; end
  
  if wk_idx>0 && wk_idx<=length(w1)
    % Work
    for ww=1:w1(wk_idx)
      S.day=S.day+1; if S.day>MD, ok=false; Z=0; M=0; return; end
      wx_t=wx(S.day);
      if wx_t==0, CW=CWn; else CW=CWs; end
      S.O=S.O-CW(1); S.H=S.H-CW(2); S.F=S.F-CW(3);
      wh=wc(wk_idx); S.Z=S.Z+WY(wh); S.cw=S.cw+1;
      if S.O<0||S.H<0||S.F<0, ok=false; Z=0; M=0; return; end
    end
    % Moor
    if wk_idx<=length(b)&&b(wk_idx)>0
      S.day=S.day+1; if S.day>MD, ok=false; Z=0; M=0; return; end
      wx_t=wx(S.day);
      if wx_t==0, CS=CSn; else CS=CSs; end
      S.O=S.O-CS(1); S.H=S.H-CS(2); S.F=S.F-CS(3); S.cw=0;
      if S.O<0||S.H<0||S.F<0, ok=false; Z=0; M=0; return; end
    end
    % Work2
    if wk_idx<=length(w2)
      for ww=1:w2(wk_idx)
        S.day=S.day+1; if S.day>MD, ok=false; Z=0; M=0; return; end
        wx_t=wx(S.day);
        if wx_t==0, CW=CWn; else CW=CWs; end
        S.O=S.O-CW(1); S.H=S.H-CW(2); S.F=S.F-CW(3);
        wh=wc(wk_idx); S.Z=S.Z+WY(wh); S.cw=S.cw+1;
        if S.O<0||S.H<0||S.F<0, ok=false; Z=0; M=0; return; end
      end
    end
  end
  
  if sup_idx>0
    % Supply: compute expected consumption for remaining path
    rem_days=0;
    for kk=k+1:m+1
      dk=tv(kk); rem_days=rem_days+dk;
      wi2=0; if kk<=length(wi), wi2=wi(kk); end
      if wi2>0&&wi2<=length(w1), rem_days=rem_days+w1(wi2); end
      if wi2>0&&wi2<=length(b)&&b(wi2)>0, rem_days=rem_days+1; end
      if wi2>0&&wi2<=length(w2), rem_days=rem_days+w2(wi2); end
    end
    Tr=MD-S.day;
    rem_days=min(rem_days,Tr);
    % Expected consumption
    eO=rem_days*(0.7*CMe(1)+0.3*CWe(1)); eH=rem_days*(0.7*CMe(2)+0.3*CWe(2)); eF=rem_days*(0.7*CMe(3)+0.3*CWe(3)); % approx
    bufO=Zf*sqrt(rem_days*VM(1)); bufH=Zf*sqrt(rem_days*VW(1)); bufF=Zf*sqrt(rem_days*VS(1));
    buyO=max(0,eO+0.5*bufO-S.O); buyH=max(0,eH+0.5*bufH-S.H); buyF=max(0,eF+0.5*bufF-S.F);
    free=LD-(S.O+S.H+S.F); tb=buyO+buyH+buyF;
    if tb>free, sc=free/max(tb,1); buyO=floor(buyO*sc); buyH=floor(buyH*sc); buyF=floor(buyF*sc); end
    cost=buyO*2+buyH+buyF*2;
    if cost<=S.M
      S.O=S.O+buyO; S.H=S.H+buyH; S.F=S.F+buyF; S.M=S.M-cost;
    end
    % Idle cost
    S.day=S.day+1; if S.day>MD, ok=false; Z=0; M=0; return; end
    wx_t=wx(S.day);
    if wx_t==0, CS=CSn; else CS=CSs; end
    S.O=S.O-CS(1); S.H=S.H-CS(2); S.F=S.F-CS(3); S.cw=0;
    if S.O<0||S.H<0||S.F<0||S.M<0, ok=false; Z=0; M=0; return; end
  end
end
Z=S.Z; M=S.M; ok=true;
end
%% ======================================================================
%% Single step execution
%% ======================================================================
function [S,aname,ok]=step_q3(S,act,w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD)
if w==0, CM=CMn; CW=CWn; CS=CSn; else CM=CMs; CW=CWs; CS=CSs; end
ok=true;
switch act.type
  case "move"
    S.pos=act.t; S.O=S.O-CM(1); S.H=S.H-CM(2); S.F=S.F-CM(3); S.cw=0;
    aname="Mv";
  case "work"
    wh=0; for wk=1:3, if isequal(S.pos,all_xy(wk+2,:)), wh=wk; break; end; end
    if wh==0, aname="Wk?"; else
      S.O=S.O-CW(1); S.H=S.H-CW(2); S.F=S.F-CW(3); S.Z=S.Z+WY(wh); S.cw=S.cw+1;
      aname=sprintf("Wk%d",wh);
    end
  case "moor"
    S.O=S.O-CS(1); S.H=S.H-CS(2); S.F=S.F-CS(3); S.cw=0; aname="Moor";
  case "supply"
    S.O=S.O-CS(1); S.H=S.H-CS(2); S.F=S.F-CS(3);
    if act.buyO>0||act.buyH>0||act.buyF>0
      S.O=S.O+act.buyO; S.H=S.H+act.buyH; S.F=S.F+act.buyF; S.M=S.M-act.cost;
    end
    S.cw=0; aname="Sup";
  otherwise
    aname="???";
end
S.day=S.day+1;
if S.O<0||S.H<0||S.F<0||S.M<0||S.O+S.H+S.F>LD, ok=false; end
end
%% ===== Phase 4: Daily Rolling Horizon =====
function [S,Z,M]=phase4_daily(sk,wx,all_xy,CMn,CWn,CSn,CMs,CWs,CSs,WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,IO,IH,IF,IM,IZ)
S=struct("pos",all_xy(1,:),"O",IO,"H",IH,"F",IF,"M",IM,"Z",IZ,"cw",0,"day",0);
pid=sk.pid; tv=sk.tv; wi=sk.wi; wc=sk.wc; si=sk.si;
nw=sk.nw; ns=sk.ns; w1=sk.w1; b=sk.b; w2=sk.w2;
di=0;
for k=1:length(pid)-1
    fi=pid(k); ti=pid(k+1);
    if fi==8
        ps=S.pos;
    else
        ps=all_xy(fi,:);
    end
    pe=all_xy(ti,:);
    % Travel to next POI
    if ~isequal(ps,pe)
        cur=ps;
        while ~isequal(cur,pe) && S.day<MD
            di=di+1;
            w=wx(min(di,length(wx)));
            nxt=cur;
            if nxt(1)~=pe(1)
                nxt(1)=nxt(1)+sign(pe(1)-nxt(1));
            else
                nxt(2)=nxt(2)+sign(pe(2)-nxt(2));
            end
            [S,~,ok]=step_q3(S,struct("type","move","t",nxt,"buyO",0,"buyH",0,"buyF",0,"cost",0),...
                w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
            if ~ok
                Z=S.Z; M=S.M; return;
            end
            cur=nxt;
        end
    end
    % Find work/supply indices at current POI
    wk_idx=0;
    for j=1:nw
        if wi(k)==j, wk_idx=j; break; end
    end
    sup_idx=0;
    for j=1:ns
        if si(k)==j, sup_idx=j; break; end
    end
    % Work: only if enough O for work + travel
    if wk_idx>0 && wk_idx<=length(w1)
        for ww=1:w1(wk_idx)
            if S.day>=MD, break; end
            r=abs(S.pos(1)-30)+abs(S.pos(2)-15);
            if S.O > CWe(1)+1.5*CMe(1)*r
                di=di+1;
                w=wx(min(di,length(wx)));
                [S,~,ok]=step_q3(S,struct("type","work","t",pe,"buyO",0,"buyH",0,"buyF",0,"cost",0),...
                    w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
                if ~ok
                    Z=S.Z; M=S.M; return;
                end
            end
        end
        if wk_idx<=length(b) && b(wk_idx)>0 && S.day<MD
            di=di+1;
            w=wx(min(di,length(wx)));
            [S,~,ok]=step_q3(S,struct("type","moor","t",pe,"buyO",0,"buyH",0,"buyF",0,"cost",0),...
                w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
            if ~ok
                Z=S.Z; M=S.M; return;
            end
        end
        if wk_idx<=length(w2)
            for ww=1:w2(wk_idx)
                if S.day>=MD, break; end
                r=abs(S.pos(1)-30)+abs(S.pos(2)-15);
                if S.O > CWe(1)+1.5*CMe(1)*r
                    di=di+1;
                    w=wx(min(di,length(wx)));
                    [S,~,ok]=step_q3(S,struct("type","work","t",pe,"buyO",0,"buyH",0,"buyF",0,"cost",0),...
                        w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
                    if ~ok
                        Z=S.Z; M=S.M; return;
                    end
                end
            end
        end
    end
    % Supply: skeleton-based remaining needs
    if sup_idx>0 && S.day<MD
        rd=0; rw=0;
        for kk=k:length(tv)
            rd=rd+tv(kk);
            wi2=0;
            if kk<=length(wi), wi2=wi(kk); end
            if wi2>0 && wi2<=length(w1)
                rw=rw+w1(wi2)+b(wi2)+w2(wi2);
            end
        end
        eO=rd*CMe(1)+rw*CWe(1);
        eH=rd*CMe(2)+rw*CWe(2);
        eF=rd*CMe(3)+rw*CWe(3);
        bO=floor(max(0,eO+0.5*sqrt((rd+rw)*VM(1))-S.O));
        bH=floor(max(0,eH+0.5*sqrt((rd+rw)*VW(1))-S.H));
        bF=floor(max(0,eF+0.5*sqrt((rd+rw)*VS(1))-S.F));
        free=LD-(S.O+S.H+S.F);
        tb=bO+bH+bF;
        if tb>free
            sc=free/max(tb,1);
            bO=floor(bO*sc); bH=floor(bH*sc); bF=floor(bF*sc);
        end
        cost=bO*2+bH+bF*2;
        if cost>S.M
            sc=max(0,S.M)/max(cost,1);
            bO=floor(bO*sc); bH=floor(bH*sc); bF=floor(bF*sc);
            cost=bO*2+bH+bF*2;
        end
        di=di+1;
        w=wx(min(di,length(wx)));
        [S,~,ok]=step_q3(S,struct("type","supply","t",pe,"buyO",bO,"buyH",bH,"buyF",bF,"cost",cost),...
            w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
        if ~ok
            Z=S.Z; M=S.M; return;
        end
    end
    if isequal(S.pos,all_xy(2,:))
        break;
    end
end
Z=S.Z; M=S.M;
end
