function [best_skel, best_Z, best_M, best_succ] = mdp_task3_final()
% =========================================================================
% MDP Task 3 - MATLAB Final Version
% Fixes: orthogonal movement, no supply idle, adaptive z-buy, stay-on-storm, weighted-Z
% =========================================================================

all_xy = [1 15; 30 15; 12 16; 21 16; 6 21; 15 9; 24 24];
names = {'B','E','S1','S2','W1','W2','W3'};
work_info = [5 20 4; 6 15 5; 7 28 3];
supply_idx = [3 4];

INIT_O=100; INIT_H=150; INIT_F=100; INIT_M=750; INIT_Z=200;
MAX_LOAD=400; MAX_DAYS=90;
PRICE=[2 1 2];
CM_N=[2 3 2]; CM_S=[8 4 3];
CI_N=[1 1 1]; CI_S=[3 3 2];
CW_N=[5 4 3]; CW_S=[8 6 6];
P_NORMAL=0.8; P_STORM=0.2;
EXP_MOVE=P_NORMAL*CM_N+P_STORM*CM_S;
EXP_IDLE=P_NORMAL*CI_N+P_STORM*CI_S;
EXP_WORK=P_NORMAL*CW_N+P_STORM*CW_S;

n_pts=size(all_xy,1); dist=zeros(n_pts);
for i=1:n_pts, for j=1:n_pts
    dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));
end, end

fprintf('Phase 1: Skeleton enumeration...\n');
intermediate=[3 4 5 6 7];
skeletons={}; max_sk=5000;

    function dfs(current,path,total_travel)
        if length(skeletons)>=max_sk, return; end
        dE=dist(current,2);
        if total_travel+dE<=MAX_DAYS, skeletons{end+1}=[path 2]; end
        for nxt=intermediate
            if nxt==current, continue; end
            if ismember(current,supply_idx)&&ismember(nxt,supply_idx), continue; end
            if length(path)>8, continue; end
            d=dist(current,nxt);
            if d==0||total_travel+d>MAX_DAYS, continue; end
            dfs(nxt,[path nxt],total_travel+d);
        end
    end

dfs(1,[1],0);
fprintf('  Generated %d skeletons\n',length(skeletons));

skel_strs=cellfun(@(v) sprintf('%d,',v),skeletons,'UniformOutput',false);
[~,ia]=unique(skel_strs); skeletons=skeletons(ia);
fprintf('  Unique: %d\n',length(skeletons));

has_work=false(1,length(skeletons));
for i=1:length(skeletons), s=skeletons{i};
    has_work(i)=any(ismember(s(2:end-1),work_info(:,1)'));
end
skeletons_work=skeletons(has_work);
all_cand=[skeletons_work, {[1 2]}];
fprintf('  With work: %d, Total: %d\n\n',length(skeletons_work),length(all_cand));

fprintf('Phase 2: Monte Carlo evaluation...\n');
N_SIM=500;
results=struct('skel',{},'avg_Z',{},'avg_M',{},'succ_rate',{},'best_Z',{},'best_M',{},'succ_count',{},'best_log',{});
tic;
for idx=1:length(all_cand)
    if mod(idx,50)==0, fprintf('  Progress: %d/%d (%.1fs)\n',idx,length(all_cand),toc); end
    skel=all_cand{idx};
    travel=0; for i=1:length(skel)-1, travel=travel+dist(skel(i),skel(i+1)); end
    if travel>80, continue; end
    [avg_Z,avg_M,succ,bestZ,bestM,bestLog]=eval_skel(skel,N_SIM);
    if succ>0
        results(end+1)=struct('skel',skel,'avg_Z',avg_Z,'avg_M',avg_M,...
            'succ_rate',succ/N_SIM,'best_Z',bestZ,'best_M',bestM,...
            'succ_count',succ,'best_log',{bestLog});
    end
end
if ~isempty(results)
    w=[results.succ_rate].*[results.avg_Z]; [w_sorted,o]=sort(w,'descend'); results=results(o);
end
fprintf('\n  Feasible: %d, Time: %.1fs\n\n',length(results),toc);

fprintf('Phase 3: Top results\n');
TOP_K=min(5,length(results));
for rank=1:TOP_K
    r=results(rank);
    skel_str=strjoin(names(r.skel),' -> ');
    fprintf('  #%d: %s\n',rank,skel_str);
    fprintf('       Z=%.1f M=%.1f succ=%d/%d (%.1f%%) best_Z=%d best_M=%d\n',...
        r.avg_Z,r.avg_M,r.succ_count,N_SIM,r.succ_rate*100,r.best_Z,r.best_M);
end

best_skel=results(1).skel; best_Z=results(1).avg_Z;
best_M=results(1).avg_M; best_succ=results(1).succ_rate;
fprintf('\n  Best: %s\n',strjoin(names(best_skel),' -> '));
fprintf('  E[Z]=%.1f E[M]=%.1f succ=%.1f%%\n\n',best_Z,best_M,best_succ*100);

if ~isempty(results(1).best_log)
    log=results(1).best_log; n_days=length(log);
    hdrs={'day','posX','posY','weather','action','node','O','H','F','M','Z','c','buyO','buyH','buyF'};
    data=cell(n_days+2,15); data(1,:)=hdrs;
    data{2,1}=0;data{2,2}=all_xy(1,1);data{2,3}=all_xy(1,2);
    data{2,4}='-';data{2,5}='init';data{2,6}='B';
    data{2,7}=INIT_O;data{2,8}=INIT_H;data{2,9}=INIT_F;
    data{2,10}=INIT_M;data{2,11}=INIT_Z;data{2,12}=0;
    data{2,13}=0;data{2,14}=0;data{2,15}=0;
    for i=1:n_days
        e=log{i}; data{i+2,1}=e.day; data{i+2,2}=e.pos(1); data{i+2,3}=e.pos(2);
        data{i+2,4}=e.weather; data{i+2,5}=e.action; data{i+2,6}=e.node;
        data{i+2,7}=e.O; data{i+2,8}=e.H; data{i+2,9}=e.F; data{i+2,10}=e.M;
        data{i+2,11}=e.Z; data{i+2,12}=e.c; data{i+2,13}=e.buy_O;
        data{i+2,14}=e.buy_H; data{i+2,15}=e.buy_F;
    end
    writecell(data,'result.xlsx');
    fprintf('  Schedule saved to result.xlsx\n');
end
fprintf('\nDone!\n');

    function [aZ,aM,s,bZ,bM,bL]=eval_skel(skel,ns)
        tZ=0;tM=0;s=0;bZ=-1;bM=-1;bL={};
        for sim=1:ns
            [Zf,Mf,log]=sim_one(skel);
            if ~isempty(log) && Zf > 0
                tZ=tZ+Zf;tM=tM+Mf;s=s+1;
                if Zf>bZ||(Zf==bZ&&Mf>bM), bZ=Zf;bM=Mf;bL=log; end
            end
        end
        if s>0, aZ=tZ/s;aM=tM/s; else aZ=0;aM=0; end
    end

    function [Zf,Mf,L]=sim_one(skel)
        x=all_xy(1,1);y=all_xy(1,2);
        O=INIT_O;H=INIT_H;F=INIT_F;M=INIT_M;Z=INIT_Z;c=0;day=1;
        L={};si=2;
        while day<=MAX_DAYS
            if x==all_xy(2,1)&&y==all_xy(2,2), Zf=Z;Mf=M;return; end
            tgt=skel(si);tx=all_xy(tgt,1);ty=all_xy(tgt,2);
            if x==tx&&y==ty
                w=rand()<P_NORMAL;wi=1+~w;
                if ismember(tgt,work_info(:,1)')
                    wr=work_info(work_info(:,1)==tgt,:);gain=wr(2);mc=wr(3);
                    rd=0;for j=si:length(skel)-1,rd=rd+dist(skel(j),skel(j+1));end
                    urg=(MAX_DAYS-day)/max(1,rd);
                    cw=w&&c<mc;sw=~w&&c<mc&&urg<1.5;
                    if cw||sw
                        cr=cr_work(w);O=O-cr(1);H=H-cr(2);F=F-cr(3);Z=Z+gain;c=c+1;day=day+1;
                        if ~ok(),Zf=0;Mf=0;return;end
                        L{end+1}=mk(day-1,[x y],w,'work',names{tgt},O+cr(1),H+cr(2),F+cr(3),M,Z-gain,c-1);
                    elseif c<mc
                        cr=cr_idle(w);O=O-cr(1);H=H-cr(2);F=F-cr(3);day=day+1;
                        if ~ok(),Zf=0;Mf=0;return;end
                        L{end+1}=mk(day-1,[x y],w,'idle',names{tgt},O+cr(1),H+cr(2),F+cr(3),M,Z,c);
                    else
                        cr=cr_idle(w);O=O-cr(1);H=H-cr(2);F=F-cr(3);c=0;day=day+1;si=si+1;
                        if ~ok()||si>length(skel),Zf=0;Mf=0;return;end
                    end
                elseif ismember(tgt,supply_idx)
                    [bo,bh,bf]=comp_buy(O,H,F,M,skel,si);
                    cost=bo*2+bh*1+bf*2;if cost>M,Zf=0;Mf=0;return;end
                    O=O+bo;H=H+bh;F=F+bf;M=M-cost;si=si+1;
                    if ~ok()||si>length(skel),Zf=0;Mf=0;return;end
                else, si=si+1;if si>length(skel),Zf=0;Mf=0;return;end;continue;
                end
            else
                w=rand()<P_NORMAL;cr=cr_move(w);
                dx=sign(tx-x);dy=0;if dx==0,dy=sign(ty-y);end
                if dx==0&&dy==0,si=si+1;continue;end
                O=O-cr(1);H=H-cr(2);F=F-cr(3);x=x+dx;y=y+dy;c=0;day=day+1;
                if ~ok(),Zf=0;Mf=0;return;end
                L{end+1}=mk(day-1,[x-dx y-dy],w,'move','',O+cr(1),H+cr(2),F+cr(3),M,Z,0);
            end
        end
        Zf=0;Mf=0;
        function r=ok(), r=~(O<0||H<0||F<0||M<0||O+H+F>MAX_LOAD); end
    end

    function [bo,bh,bf]=comp_buy(O,H,F,M,skel,si)
        no=0;nh=0;nf=0;vo=0;vh=0;vf=0;
        ni=length(skel);for j=si+1:length(skel),if ismember(skel(j),supply_idx),ni=j;break;end,end
        for j=si:ni-1
            n1=skel(j);n2=skel(j+1);d=dist(n1,n2);
            no=no+EXP_MOVE(1)*d;nh=nh+EXP_MOVE(2)*d;nf=nf+EXP_MOVE(3)*d;
            e2o=P_NORMAL*CM_N(1)^2+P_STORM*CM_S(1)^2;vo=vo+(e2o-EXP_MOVE(1)^2)*d;
            e2h=P_NORMAL*CM_N(2)^2+P_STORM*CM_S(2)^2;vh=vh+(e2h-EXP_MOVE(2)^2)*d;
            e2f=P_NORMAL*CM_N(3)^2+P_STORM*CM_S(3)^2;vf=vf+(e2f-EXP_MOVE(3)^2)*d;
            if ismember(n2,work_info(:,1)')
                wr=work_info(work_info(:,1)==n2,:);mw=wr(3);ew=P_NORMAL*mw;
                no=no+EXP_WORK(1)*ew;nh=nh+EXP_WORK(2)*ew;nf=nf+EXP_WORK(3)*ew;
                wv=mw*P_NORMAL*P_STORM;vo=vo+wv*(CW_N(1)-CW_S(1))^2;
                vh=vh+wv*(CW_N(2)-CW_S(2))^2;vf=vf+wv*(CW_N(3)-CW_S(3))^2;
            end
        end
        ec=no*2+nh*1+nf*2;cv=O*2+H*1+F*2;fn=max(0,ec-cv);
        sl=(M-fn)/max(1,fn);if fn==0,sl=2;end;z=0.5+1.5*min(1,max(0,sl));
        no=ceil(no+z*sqrt(max(0,vo)));nh=ceil(nh+z*sqrt(max(0,vh)));nf=ceil(nf+z*sqrt(max(0,vf)));
        bo=max(0,no-O);bh=max(0,nh-H);bf=max(0,nf-F);
        sp=MAX_LOAD-(O+H+F);tb=bo+bh+bf;if tb>sp&&tb>0,scl=sp/tb;bo=floor(bo*scl);bh=floor(bh*scl);bf=floor(bf*scl);end
        cst=bo*2+bh*1+bf*2;if cst>M&&cst>0,scl=M/cst;bo=floor(bo*scl);bh=floor(bh*scl);bf=floor(bf*scl);end
    end
end

function cr=cr_move(w), if w,cr=[2 3 2];else,cr=[8 4 3];end,end
function cr=cr_idle(w), if w,cr=[1 1 1];else,cr=[3 3 2];end,end
function cr=cr_work(w), if w,cr=[5 4 3];else,cr=[8 6 6];end,end

function e=mk(d,pos,w,act,node,O,H,F,M,Z,c,bo,bh,bf,np)
    if nargin<12,bo=0;end;if nargin<13,bh=0;end;if nargin<14,bf=0;end;if nargin<15,np=[0 0];end
    e=struct('day',d,'pos',pos,'weather',w,'action',act,'node',node,...
        'O',O,'H',H,'F',F,'M',M,'Z',Z,'c',c,'buy_O',bo,'buy_H',bh,'buy_F',bf,'new_pos',np);
end
