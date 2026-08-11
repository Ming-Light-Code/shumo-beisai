function solve_q3_new()
if ~exist("intlinprog","file"), error("Need Optimization Toolbox."); end
rng(2026); tic;
fprintf("=== TASK 3: CCASR ===\n");
%% ---- Data ----
all_xy=[1 15;30 15;6 21;15 9;24 24;12 16;21 16];
names={"B","E","W1","W2","W3","S1","S2"};
WY=[20,15,28]; WM=[4,5,3];
LD=400; MD=90; IO=100; IH=150; IF=100; IM=750; IZ=200;
CMn=[2,3,2]; CWn=[5,4,3]; CSn=[1,1,1];
CMs=[8,4,3]; CWs=[8,6,6]; CSs=[3,3,2];
CMe=0.8*CMn+0.2*CMs; CWe=0.8*CWn+0.2*CWs; CSe=0.8*CSn+0.2*CSs;
VM=0.16*(CMs-CMn).^2; VW=0.16*(CWs-CWn).^2; VS=0.16*(CSs-CSn).^2;
ZA=1.645; PR=[2,1,2];
nP=7; d=zeros(nP);
for i=1:nP,for j=1:nP,d(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));end;end

function cand = gen_policies(d, MD)
bp = {[6 7 5],[6 5],[7 5],[5],[3 6 5],[3 6 7 5],[6 4 7 5],[4 7 5],[3 6 4 7 5],[6 3 5],[7 4 5]};
rp = {[5 7 5],[6 5 7 5],[7 5 7 5],[6 7 5 7 5],[3 6 5 7 5]};
np = {[6 3],[7 4],[6 3 7 4],[3 6 4 7]};
ap = [bp, rp, np]; cand = {};
for p = 1:length(ap)
    seq = ap{p}; td = d(1,seq(1));
    for i = 1:length(seq)-1, td = td + d(seq(i),seq(i+1)); end
    td = td + d(seq(end),2); mw = 0;
    for j = 1:length(seq)
        if seq(j)>=3&&seq(j)<=5, mw=mw+1; elseif seq(j)==6||seq(j)==7, mw=mw+1; end
    end
    if td + mw <= MD, cand{end+1}=seq; end
end
uq = {};
for i = 1:length(cand)
    dup = false;
    for j = 1:length(uq), if isequal(cand{i},uq{j}), dup=true; break; end; end
    if ~dup, uq{end+1}=cand{i}; end
end
cand = uq;
end

function [ok, Z, M, wi, bi] = milp_seq(seq, d, CMe, CWe, CSe, VM, VW, VS, ZA, LD, MD, IO, IH, IF, IM, IZ, WY, WM, PR)
ok=false; Z=0; M=0; wi=[]; bi=[];
n=length(seq); ns=n+1; fs=[1,seq,2]; tv=zeros(1,ns);
for i=1:ns, tv(i)=d(fs(i),fs(i+1)); end
wix=zeros(1,n); six=zeros(1,n); nw=0; ns2=0; wt=[];
for i=1:n
    pt=seq(i);
    if pt>=3&&pt<=5, nw=nw+1; wix(i)=nw; wt(nw)=pt-2;
    elseif pt==6||pt==7, ns2=ns2+1; six(i)=ns2; end
end
if nw==0
    tt=sum(tv); if tt>MD, return; end
    eO=tt*CMe(1); eH=tt*CMe(2); eF=tt*CMe(3);
    sO=ZA*sqrt(tt*VM(1)); sH=ZA*sqrt(tt*VM(2)); sF=ZA*sqrt(tt*VM(3));
    nO=max(0,eO+sO-IO); nH=max(0,eH+sH-IH); nF=max(0,eF+sF-IF);
    cs=nO*PR(1)+nH*PR(2)+nF*PR(3);
    if cs>IM||IO+nO+IH+nH+IF+nF>LD, return; end
    ok=true; Z=IZ; M=IM-cs; bi=zeros(ns2,3); return;
end
nv=3*nw+3*ns2+4*(ns+1);
o1=0; ob=o1+nw; o2=ob+nw; oO=o2+nw; oH=oO+ns2; oF=oH+ns2;
oR=oF+ns2; oRR=oR+(ns+1); oRRF=oRR+(ns+1); oRM=oRRF+(ns+1);
iv=1:(3*nw+3*ns2);
lb=zeros(nv,1); ub=inf(nv,1);
for j=1:nw, ub(o1+j)=WM(wt(j)); ub(ob+j)=1; ub(o2+j)=WM(wt(j)); end
lb(oR+1)=IO; ub(oR+1)=IO; lb(oRR+1)=IH; ub(oRR+1)=IH;
lb(oRRF+1)=IF; ub(oRRF+1)=IF; lb(oRM+1)=IM; ub(oRM+1)=IM;
neq=4*ns; Aeq=zeros(neq,nv); beq=zeros(neq,1);
for sg=1:ns
    dd=tv(sg); r=(sg-1)*4;
    Aeq(r+1,oR+sg+1)=1; Aeq(r+1,oR+sg)=-1; beq(r+1)=-dd*CMe(1);
    Aeq(r+2,oRR+sg+1)=1; Aeq(r+2,oRR+sg)=-1; beq(r+2)=-dd*CMe(2);
    Aeq(r+3,oRRF+sg+1)=1; Aeq(r+3,oRRF+sg)=-1; beq(r+3)=-dd*CMe(3);
    Aeq(r+4,oRM+sg+1)=1; Aeq(r+4,oRM+sg)=-1; beq(r+4)=0;
    wi2=wix(min(sg,n)); si=six(min(sg,n));
    if wi2>0
        Aeq(r+1,o1+wi2)=CWe(1); Aeq(r+1,ob+wi2)=CSe(1); Aeq(r+1,o2+wi2)=CWe(1);
        Aeq(r+2,o1+wi2)=CWe(2); Aeq(r+2,ob+wi2)=CSe(2); Aeq(r+2,o2+wi2)=CWe(2);
        Aeq(r+3,o1+wi2)=CWe(3); Aeq(r+3,ob+wi2)=CSe(3); Aeq(r+3,o2+wi2)=CWe(3);
    end
    if si>0
        Aeq(r+1,oO+si)=-1; Aeq(r+2,oH+si)=-1; Aeq(r+3,oF+si)=-1;
        Aeq(r+4,oO+si)=PR(1); Aeq(r+4,oH+si)=PR(2); Aeq(r+4,oF+si)=PR(3);
    end
end
ni=(ns+1)+1+2*nw+3; Ai=zeros(ni,nv); bi2=zeros(ni,1); inn=0;
for sg=0:ns
    inn=inn+1; Ai(inn,oR+sg+1)=1; Ai(inn,oRR+sg+1)=1; Ai(inn,oRRF+sg+1)=1; bi2(inn)=LD;
end
inn=inn+1;
for j=1:nw, Ai(inn,o1+j)=1; Ai(inn,ob+j)=1; Ai(inn,o2+j)=1; end; bi2(inn)=MD-sum(tv);
for j=1:nw, inn=inn+1; Ai(inn,o1+j)=1; bi2(inn)=WM(wt(j)); end
for j=1:nw, inn=inn+1; Ai(inn,o2+j)=1; Ai(inn,ob+j)=WM(wt(j)); bi2(inn)=WM(wt(j)); end
inn=inn+1; Ai(inn,oR+ns+1)=-1; bi2(inn)=0;
inn=inn+1; Ai(inn,oRR+ns+1)=-1; bi2(inn)=0;
inn=inn+1; Ai(inn,oRRF+ns+1)=-1; bi2(inn)=0;
lb(oR+ns+1)=0; lb(oRR+ns+1)=0; lb(oRRF+ns+1)=0; lb(oRM+ns+1)=0;
f=zeros(nv,1);
for j=1:nw, f(o1+j)=-WY(wt(j)); f(o2+j)=-WY(wt(j)); end
op=optimoptions('intlinprog','Display','off');
[x1,~,fl1]=intlinprog(f,iv,Ai,bi2,Aeq,beq,lb,ub,op);
if fl1<=0||isempty(x1), return; end
Zv=IZ+round(-f'*x1);
A2=[Ai;zeros(2,nv)]; b2=[bi2;Zv-IZ;-(Zv-IZ)]; nr2=size(Ai,1);
for j=1:nw
    A2(nr2+1,o1+j)=WY(wt(j)); A2(nr2+1,o2+j)=WY(wt(j));
    A2(nr2+2,o1+j)=-WY(wt(j)); A2(nr2+2,o2+j)=-WY(wt(j));
end
fM=zeros(nv,1); fM(oRM+ns+1)=-1;
[x2,~,fl2]=intlinprog(fM,iv,A2,b2,Aeq,beq,lb,ub,op);
if fl2<=0||isempty(x2), return; end
ok=true; Z=Zv; M=round(-fM'*x2);
wi=zeros(nw,3); for j=1:nw, wi(j,:)=[round(x2(o1+j)),round(x2(ob+j)),round(x2(o2+j))]; end
bi=zeros(ns2,3); for k=1:ns2, bi(k,:)=[round(x2(oO+k)),round(x2(oH+k)),round(x2(oF+k))]; end
end

function td = total_days(seq, d, wi)
fs=[1,seq,2]; tv=0;
for i=1:length(fs)-1, tv=tv+d(fs(i),fs(i+1)); end
tw=sum(wi(:,1))+sum(wi(:,2))+sum(wi(:,3));
ns=sum(seq==6|seq==7); td=tv+tw+ns;
end

function [EZ, EM, sr] = mc_eval(pol, NM, all_xy, d, CMn, CWn, CSn, CMs, CWs, CSs, CMe, CWe, CSe, VM, VW, VS, ZA, LD, MD, WY, WM, PR, IZ)
su=0; Zs=0; Ms=0;
for s=1:NM
    wx=rand(1,MD)<0.2;
    [okk,Zf,Mf]=sim_one(pol,wx,all_xy,d,CMn,CWn,CSn,CMs,CWs,CSs,CMe,CWe,CSe,VM,VW,VS,ZA,LD,MD,WY,WM,PR,IZ);
    if okk, su=su+1; Zs=Zs+Zf; Ms=Ms+Mf; end
end
EZ=Zs/NM; EM=Ms/max(su,1); sr=su/NM;
end

function [okk, Zf, Mf] = sim_one(pol, wx, all_xy, d, CMn, CWn, CSn, CMs, CWs, CSs, CMe, CWe, CSe, VM, VW, VS, ZA, LD, MD, WY, WM, PR, IZ)
seq=pol.seq; wi=pol.wi; n=length(seq); fs=[1,seq,2];
S.O=100; S.H=150; S.F=100; S.M=750; S.Z=200;
S.p=all_xy(1,:); S.day=0; S.cw=0;
wic=0;
for sg=1:(n+1)
    ti=fs(sg+1); tp=all_xy(ti,:); cur=S.p;
    while ~isequal(cur,tp)
        S.day=S.day+1;
        if S.day>MD||S.day>length(wx), okk=false;Zf=0;Mf=0;return; end
        is_st=wx(S.day);
        if is_st, CMt=CMs; CSt=CSs; else, CMt=CMn; CSt=CSn; end
        if is_st&&(S.O<CMt(1)*3||S.H<CMt(2)*2||S.F<CMt(3)*2)
            S.O=S.O-CSt(1); S.H=S.H-CSt(2); S.F=S.F-CSt(3); S.cw=0;
        else
            nxt=cur;
            if nxt(1)~=tp(1), nxt(1)=nxt(1)+sign(tp(1)-nxt(1));
            else, nxt(2)=nxt(2)+sign(tp(2)-nxt(2)); end
            S.O=S.O-CMt(1); S.H=S.H-CMt(2); S.F=S.F-CMt(3);
            cur=nxt; S.p=cur; S.cw=0;
        end
        if S.O<0||S.H<0||S.F<0, okk=false;Zf=0;Mf=0;return; end
        if S.O+S.H+S.F>LD, okk=false;Zf=0;Mf=0;return; end
    end
    S.p=tp; pt=ti;
    if pt>=3&&pt<=5
        wic=wic+1; wh=pt-2;
        if wic<=size(wi,1), w1=wi(wic,1); b=wi(wic,2); w2=wi(wic,3);
        else, w1=0;b=0;w2=0; end
        for ww=1:w1
            S.day=S.day+1;
            if S.day>MD||S.day>length(wx), okk=false;Zf=0;Mf=0;return; end
            is_st=wx(S.day);
            if is_st, CWt=CWs; CSt=CSs; else, CWt=CWn; CSt=CSn; end
            rd=abs(S.p(1)-30)+abs(S.p(2)-15);
            if S.O>=CWt(1)+CMe(1)*rd*1.2&&S.H>=CWt(2)*1.5&&S.F>=CWt(3)*1.5
                S.O=S.O-CWt(1); S.H=S.H-CWt(2); S.F=S.F-CWt(3); S.Z=S.Z+WY(wh); S.cw=S.cw+1;
            else
                S.O=S.O-CSt(1); S.H=S.H-CSt(2); S.F=S.F-CSt(3); S.cw=0;
            end
            if S.O<0||S.H<0||S.F<0, okk=false;Zf=0;Mf=0;return; end
        end
        if b>0&&S.day<MD
            S.day=S.day+1;
            if S.day>MD||S.day>length(wx), okk=false;Zf=0;Mf=0;return; end
            is_st=wx(S.day);
            if is_st, CSt=CSs; else, CSt=CSn; end
            S.O=S.O-CSt(1); S.H=S.H-CSt(2); S.F=S.F-CSt(3); S.cw=0;
            if S.O<0||S.H<0||S.F<0, okk=false;Zf=0;Mf=0;return; end
        end
        for ww=1:w2
            S.day=S.day+1;
            if S.day>MD||S.day>length(wx), okk=false;Zf=0;Mf=0;return; end
            is_st=wx(S.day);
            if is_st, CWt=CWs; CSt=CSs; else, CWt=CWn; CSt=CSn; end
            rd=abs(S.p(1)-30)+abs(S.p(2)-15);
            if S.O>=CWt(1)+CMe(1)*rd*1.2&&S.H>=CWt(2)*1.5&&S.F>=CWt(3)*1.5
                S.O=S.O-CWt(1); S.H=S.H-CWt(2); S.F=S.F-CWt(3); S.Z=S.Z+WY(wh); S.cw=S.cw+1;
            else
                S.O=S.O-CSt(1); S.H=S.H-CSt(2); S.F=S.F-CSt(3); S.cw=0;
            end
            if S.O<0||S.H<0||S.F<0, okk=false;Zf=0;Mf=0;return; end
        end
    end
    if (pt==6||pt==7)&&S.day<MD
        [bO,bH,bF]=adap_sup(S,seq,sg,d,fs,wi,wic,CMe,CWe,CSe,VM,VW,VS,ZA,LD,MD,PR);
        S.day=S.day+1;
        if S.day>MD||S.day>length(wx), okk=false;Zf=0;Mf=0;return; end
        is_st=wx(S.day);
        if is_st, CSt=CSs; else, CSt=CSn; end
        cs=bO*PR(1)+bH*PR(2)+bF*PR(3);
        if cs>S.M, sc=S.M/max(cs,1); bO=floor(bO*sc); bH=floor(bH*sc); bF=floor(bF*sc); cs=bO*PR(1)+bH*PR(2)+bF*PR(3); end
        S.O=S.O-CSt(1)+bO; S.H=S.H-CSt(2)+bH; S.F=S.F-CSt(3)+bF; S.M=S.M-cs; S.cw=0;
        if S.O<0||S.H<0||S.F<0||S.M<0, okk=false;Zf=0;Mf=0;return; end
        if S.O+S.H+S.F>LD, okk=false;Zf=0;Mf=0;return; end
    end
    if ti==2, break; end
end
okk=true; Zf=S.Z; Mf=S.M;
end

function [bO,bH,bF] = adap_sup(S,seq,csg,d,fs,wi,wic,CMe,CWe,CSe,VM,VW,VS,ZA,LD,MD,PR)
n=length(seq);
rt=0; for k=csg+1:length(fs)-1, rt=rt+d(fs(k),fs(k+1)); end
rw=0; for j=wic+1:size(wi,1), rw=rw+wi(j,1)+wi(j,2)+wi(j,3); end
rs=0; for k=csg+1:n, if seq(k)==6||seq(k)==7, rs=rs+1; end; end
eO=rt*CMe(1)+rw*CWe(1)+rs*CSe(1);
eH=rt*CMe(2)+rw*CWe(2)+rs*CSe(2);
eF=rt*CMe(3)+rw*CWe(3)+rs*CSe(3);
vO=rt*VM(1)+rw*VW(1)+rs*VS(1);
vH=rt*VM(2)+rw*VW(2)+rs*VS(2);
vF=rt*VM(3)+rw*VW(3)+rs*VS(3);
sO=ZA*sqrt(max(0,vO)); sH=ZA*sqrt(max(0,vH)); sF=ZA*sqrt(max(0,vF));
nO=max(0,eO+sO-S.O); nH=max(0,eH+sH-S.H); nF=max(0,eF+sF-S.F);
fr=LD-(S.O+S.H+S.F); tn=nO+nH+nF;
if tn>fr&&fr>0, sc=fr/tn; nO=nO*sc; nH=nH*sc; nF=nF*sc; end
bO=floor(nO); bH=floor(nH); bF=floor(nF);
end

function [fs2, lg] = online_exec(pol, all_xy, d, CMn, CWn, CSn, CMs, CWs, CSs, CMe, CWe, CSe, VM, VW, VS, ZA, LD, MD, IO, IH, IF, IM, IZ, WY, WM, PR)
wx=rand(1,MD)<0.2;
S.O=IO; S.H=IH; S.F=IF; S.M=IM; S.Z=IZ; S.p=all_xy(1,:); S.day=0; S.cw=0;
lg.mv=0; lg.wk=0; lg.mr=0; lg.sp=0; lg.st=0; lg.cost=0;
seq=pol.seq; wi=pol.wi; n=length(seq); fs=[1,seq,2]; wic=0;
for sg=1:(n+1)
    ti=fs(sg+1); tp=all_xy(ti,:); cur=S.p;
    while ~isequal(cur,tp)&&S.day<MD
        S.day=S.day+1; is_st=wx(S.day);
        if is_st, lg.st=lg.st+1; CMt=CMs; CSt=CSs; else, CMt=CMn; CSt=CSn; end
        if is_st&&(S.O<CMt(1)*3||S.H<CMt(2)*2||S.F<CMt(3)*2)
            S.O=S.O-CSt(1); S.H=S.H-CSt(2); S.F=S.F-CSt(3); lg.mr=lg.mr+1; S.cw=0;
        else
            nxt=cur;
            if nxt(1)~=tp(1), nxt(1)=nxt(1)+sign(tp(1)-nxt(1));
            else, nxt(2)=nxt(2)+sign(tp(2)-nxt(2)); end
            S.O=S.O-CMt(1); S.H=S.H-CMt(2); S.F=S.F-CMt(3);
            lg.mv=lg.mv+1; cur=nxt; S.p=cur; S.cw=0;
        end
        if S.O<0||S.H<0||S.F<0, fprintf('FAILED day %d\n',S.day); fs2=S; return; end
    end
    S.p=tp; pt=ti;
    if pt>=3&&pt<=5
        wic=wic+1; wh=pt-2;
        if wic<=size(wi,1), w1=wi(wic,1); b=wi(wic,2); w2=wi(wic,3); else, w1=0;b=0;w2=0; end
        for ww=1:w1
            if S.day>=MD, break; end
            S.day=S.day+1; is_st=wx(S.day);
            if is_st, lg.st=lg.st+1; CWt=CWs; CSt=CSs; else, CWt=CWn; CSt=CSn; end
            rd=abs(S.p(1)-30)+abs(S.p(2)-15);
            if S.O>=CWt(1)+CMe(1)*rd*1.2&&S.H>=CWt(2)*1.5&&S.F>=CWt(3)*1.5
                S.O=S.O-CWt(1); S.H=S.H-CWt(2); S.F=S.F-CWt(3); S.Z=S.Z+WY(wh); S.cw=S.cw+1; lg.wk=lg.wk+1;
            else
                S.O=S.O-CSt(1); S.H=S.H-CSt(2); S.F=S.F-CSt(3); S.cw=0; lg.mr=lg.mr+1;
            end
            if S.O<0||S.H<0||S.F<0, fprintf('FAILED work day %d\n',S.day); fs2=S; return; end
        end
        if b>0&&S.day<MD
            S.day=S.day+1; is_st=wx(S.day);
            if is_st, lg.st=lg.st+1; CSt=CSs; else, CSt=CSn; end
            S.O=S.O-CSt(1); S.H=S.H-CSt(2); S.F=S.F-CSt(3); S.cw=0; lg.mr=lg.mr+1;
            if S.O<0||S.H<0||S.F<0, fs2=S; return; end
        end
        for ww=1:w2
            if S.day>=MD, break; end
            S.day=S.day+1; is_st=wx(S.day);
            if is_st, lg.st=lg.st+1; CWt=CWs; CSt=CSs; else, CWt=CWn; CSt=CSn; end
            rd=abs(S.p(1)-30)+abs(S.p(2)-15);
            if S.O>=CWt(1)+CMe(1)*rd*1.2&&S.H>=CWt(2)*1.5&&S.F>=CWt(3)*1.5
                S.O=S.O-CWt(1); S.H=S.H-CWt(2); S.F=S.F-CWt(3); S.Z=S.Z+WY(wh); S.cw=S.cw+1; lg.wk=lg.wk+1;
            else
                S.O=S.O-CSt(1); S.H=S.H-CSt(2); S.F=S.F-CSt(3); S.cw=0; lg.mr=lg.mr+1;
            end
            if S.O<0||S.H<0||S.F<0, fs2=S; return; end
        end
    end
    if (pt==6||pt==7)&&S.day<MD
        [bO,bH,bF]=adap_sup(S,seq,sg,d,fs,wi,wic,CMe,CWe,CSe,VM,VW,VS,ZA,LD,MD,PR);
        S.day=S.day+1; is_st=wx(S.day);
        if is_st, lg.st=lg.st+1; CSt=CSs; else, CSt=CSn; end
        cs=bO*PR(1)+bH*PR(2)+bF*PR(3);
        if cs>S.M, sc=S.M/max(cs,1); bO=floor(bO*sc); bH=floor(bH*sc); bF=floor(bF*sc); cs=bO*PR(1)+bH*PR(2)+bF*PR(3); end
        S.O=S.O-CSt(1)+bO; S.H=S.H-CSt(2)+bH; S.F=S.F-CSt(3)+bF; S.M=S.M-cs; lg.cost=lg.cost+cs; lg.sp=lg.sp+1; S.cw=0;
        if S.O<0||S.H<0||S.F<0||S.M<0, fprintf('FAILED supply day %d\n',S.day); fs2=S; return; end
    end
    if ti==2, break; end
end
fs2=S;
end
