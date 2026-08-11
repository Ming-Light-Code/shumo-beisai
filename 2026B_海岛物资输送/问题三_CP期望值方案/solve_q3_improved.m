%% ==========================================================================
% 2026B Problem 3: Improved Stochastic Optimization Solver (MATLAB)
% Method: CP + Statistical Safety Margins + Monte Carlo SAA + Adaptive RHR
% ==========================================================================
% Key improvements over the original CP expected-value solver:
%   1. Statistical (z-score) safety margins replace ad-hoc 1.25/1.05 multipliers
%   2. Monte Carlo SAA (500 scenarios) to evaluate candidates under real weather
%   3. Online adaptive rolling-horizon replanning at each POI
%   4. Full Monte Carlo validation (1000 scenarios) of the adaptive strategy
%   5. Reports success probability, expected Z, and risk-return tradeoffs
% ==========================================================================

function solve_q3_improved()
    t0 = tic;
    fprintf('=============================================================\n');
    fprintf(' Task 3: Improved Stochastic Optimization Solver\n');
    fprintf('=============================================================\n');

    %% ==================== Parameters ====================
    MAX_DAYS = 90;  MAX_LOAD = 400;
    INIT_O = 100;   INIT_H = 150;  INIT_F = 100;
    INIT_M = 750;   INIT_Z = 200;
    PRICE_O = 2;    PRICE_H = 1;   PRICE_F = 2;
    PN = 0.8;       PS = 0.2;
    Z_ALPHA = 1.645;   % z-score for 95% one-sided confidence
    N_MC_CANDIDATE = 500;   % Monte Carlo scenarios per candidate
    N_MC_ADAPTIVE  = 1000;  % Monte Carlo scenarios for adaptive validation
    MAX_CNT = 50000;

    %% ==================== Expected Values and Variances ====================
    e = struct('MO',PN*2+PS*8, 'MH',PN*3+PS*4, 'MF',PN*2+PS*3, ...
               'PO',PN*1+PS*3, 'PH',PN*1+PS*3, 'PF',PN*1+PS*2, ...
               'WO',PN*5+PS*8, 'WH',PN*4+PS*6, 'WF',PN*3+PS*6);
    v = struct('MO',PN*(2-e.MO)^2+PS*(8-e.MO)^2, ...
               'MH',PN*(3-e.MH)^2+PS*(4-e.MH)^2, ...
               'MF',PN*(2-e.MF)^2+PS*(3-e.MF)^2, ...
               'WO',PN*(5-e.WO)^2+PS*(8-e.WO)^2, ...
               'WH',PN*(4-e.WH)^2+PS*(6-e.WH)^2, ...
               'WF',PN*(3-e.WF)^2+PS*(6-e.WF)^2, ...
               'PO',PN*(1-e.PO)^2+PS*(3-e.PO)^2, ...
               'PH',PN*(1-e.PH)^2+PS*(3-e.PH)^2, ...
               'PF',PN*(1-e.PF)^2+PS*(2-e.PF)^2);

    fprintf('  Grid: 30x30 | Days: %d | Load: %d\n',MAX_DAYS,MAX_LOAD);
    fprintf('  P(N)=%.1f P(S)=%.1f | z_alpha=%.3f (%.0f%%%% conf) | MC_cand=%d MC_adapt=%d\n',...
        PN,PS,Z_ALPHA,0.5*erfc(-Z_ALPHA/sqrt(2))*100,N_MC_CANDIDATE,N_MC_ADAPTIVE);

    %% ==================== Coordinate and POI Setup ====================
    XY.B=[1 15]; XY.E=[30 15]; XY.W1=[6 21]; XY.W2=[15 9];
    XY.W3=[24 24]; XY.S1=[12 16]; XY.S2=[21 16];
    AN = {'B','E','W1','W2','W3','S1','S2'};
    WI.W1=[20 4]; WI.W2=[15 5]; WI.W3=[28 3];
    IN = {'W1','W2','W3','S1','S2'};

    D=zeros(7);
    for i=1:7,for j=1:7,D(i,j)=md(XY.(AN{i}),XY.(AN{j}));end,end
    distFn=@(a,b)D(find(strcmp(AN,a)),find(strcmp(AN,b)));

    %% ==================== Phase 1: CP Candidate Generation ====================
    fprintf('\n[Phase 1] CP skeleton enumeration + statistical safety margins...\n');
    skels=enum_skels(distFn,IN,MAX_DAYS,MAX_CNT);
    fprintf('  Raw skeletons: %d\n',length(skels));

    uq={}; seen=containers.Map();
    for i=1:length(skels)
        key=strjoin(skels{i},'|');
        if ~isKey(seen,key),seen(key)=true;uq{end+1}=skels{i};end
    end
    nUq=length(uq); tdists=zeros(1,nUq);
    for i=1:nUq,tdists(i)=skeldist(uq{i},distFn);end
    [~,order]=sort(tdists); uq=uq(order);
    hw={};
    for i=1:nUq,if haswork(uq{i},WI),hw{end+1}=uq{i};end,end
    fprintf('  Unique: %d  With work: %d\n',nUq,length(hw));

    bestCandidates={}; explored=0; pruned=0;
    MAX_CANDIDATES = 30;

    for idx=1:length(hw)
        sk=hw{idx};
        wps={};
        for i=1:length(sk)
            if isfield(WI,sk{i}),wps{end+1}=sk{i};end
        end
        if isempty(wps),continue;end

        td=0;
        for i=1:length(sk)-1,td=td+distFn(sk{i},sk{i+1});end
        nSup=sum(cellfun(@(x)any(strcmp(x,{'S1','S2'})),sk));
        rem=MAX_DAYS-td-nSup;

        if length(bestCandidates)>=MAX_CANDIDATES
            worstZ=min(cellfun(@(x)x{3},bestCandidates));
            ubZ=INIT_Z+mwp(3,rem)*28;
            if ubZ<=worstZ,pruned=pruned+1;continue;end
        end

        wpUniq=unique(wps);
        ranges=cell(1,length(wpUniq));
        for wi=1:length(wpUniq)
            nm=wpUniq{wi}; wm=WI.(nm); wm=wm(2);
            mw=mwp(wm,rem);
            rng=0:mw; ranges{wi}=rng;
        end
        totalCombos=1;
        for wi=1:length(ranges),totalCombos=totalCombos*length(ranges{wi});end
        if totalCombos>100000
            for wi=1:length(ranges)
                if length(ranges{wi})>8
                    step=max(1,floor(length(ranges{wi})/6));
                    ranges{wi}=ranges{wi}(1:step:end);
                end
            end
            totalCombos=1;
            for wi=1:length(ranges),totalCombos=totalCombos*length(ranges{wi});end
        end
        if totalCombos>200000,continue;end

        combos=cp(ranges);
        for ci=1:size(combos,1)
            wd=containers.Map();
            for wi=1:length(wpUniq),wd(wpUniq{wi})=combos(ci,wi);end
            if sum(combos(ci,:))==0,continue;end

            tpk=0;
            for wi=1:length(wpUniq)
                nm=wpUniq{wi}; wm=WI.(nm); wm=wm(2); wdv=wd(nm);
                if wdv>0,tpk=tpk+pdfw(wdv,wm);end
            end
            if td+sum(combos(ci,:))+tpk+nSup>MAX_DAYS,continue;end

            explored=explored+1;
            [ok,Zf,Mf,~,~]=simul_deterministic(sk,wd,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
                INIT_O,INIT_H,INIT_F,INIT_M,INIT_Z,e,v,Z_ALPHA,PRICE_O,PRICE_H,PRICE_F);

            if ok
                bestCandidates{end+1}={sk,wd,Zf,Mf};
                if length(bestCandidates)>MAX_CANDIDATES
                    [~,si]=sort(cellfun(@(x)x{3},bestCandidates),'descend');
                    bestCandidates=bestCandidates(si(1:MAX_CANDIDATES));
                end
            end
        end
        if mod(idx,100)==0
            fprintf('  [%d/%d] candidates=%d explored=%d\n',...
                idx,length(hw),length(bestCandidates),explored);
        end
    end
    fprintf('  Phase 1 done: %d candidates, explored=%d, pruned=%d (%.1fs)\n',...
        length(bestCandidates),explored,pruned,toc(t0));

    %% ==================== Phase 2: Monte Carlo SAA Evaluation ====================
    fprintf('\n[Phase 2] Monte Carlo SAA evaluation (%d scenarios each)...\n',N_MC_CANDIDATE);

    mcResults = cell(length(bestCandidates),1);
    for ci=1:length(bestCandidates)
        sk=bestCandidates{ci}{1}; wd=bestCandidates{ci}{2};
        Zdet=bestCandidates{ci}{3}; Mdet=bestCandidates{ci}{4};

        try
            rng(2026+ci);
        catch e2
            fprintf('rng call FAILED: %s\n', e2.message);
        end
        successes=0; Zvals=zeros(N_MC_CANDIDATE,1); Mvals=zeros(N_MC_CANDIDATE,1);
        for s=1:N_MC_CANDIDATE
            weather=rand(MAX_DAYS,1)<PS;
            [ok,Zf,Mf,~]=simul_stochastic_fixed(sk,wd,weather,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
                INIT_O,INIT_H,INIT_F,INIT_M,INIT_Z,e,v,Z_ALPHA,PRICE_O,PRICE_H,PRICE_F);
            if ok,successes=successes+1;Zvals(s)=Zf;Mvals(s)=Mf;
            else,Zvals(s)=0;Mvals(s)=0;end
        end
        successRate=successes/N_MC_CANDIDATE;
        expectedZ=mean(Zvals);
        Z_std=std(Zvals(Zvals>0)); if successes==0,Z_std=0;end
        M_avg=mean(Mvals(Mvals>0)); if successes==0,M_avg=0;end

        mcResults{ci}=struct('sk',{sk},'wd',{wd},...
            'Zdet',Zdet,'Mdet',Mdet,...
            'successRate',successRate,'expectedZ',expectedZ,...
            'Z_std',Z_std,'M_avg',M_avg,'successes',successes);

        pathStr=strjoin(sk,'->');
        fprintf('  [%2d] %s | Zdet=%d Mdet=%d | succ=%.1f%%%% E[Z]=%.0f\n',...
            ci,pathStr,Zdet,Mdet,successRate*100,expectedZ);
    end

    [bestExpZ,bestIdx]=max(cellfun(@(x)x.expectedZ,mcResults));
    bestResult=mcResults{bestIdx};
    fprintf('\n  Selected best: %s | E[Z]=%.0f success=%.1f%%%%\n',...
        strjoin(bestResult.sk,'->'),bestResult.expectedZ,bestResult.successRate*100);

    if bestResult.successRate<0.30
        fprintf('  WARNING: Best candidate success rate < 30%%. Consider more conservative plan.\n');
    end

    %% ==================== Phase 3: Online Adaptive Demo ====================
    fprintf('\n[Phase 3] Online adaptive rolling-horizon execution...\n');
    rng(2026);
    demoWeather=rand(MAX_DAYS,1)<PS;

    [adaptOK,adaptZ,adaptM,adaptLog,adaptDays]=adaptive_execute(...
        bestResult.sk,bestResult.wd,demoWeather,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
        INIT_O,INIT_H,INIT_F,INIT_M,INIT_Z,e,v,Z_ALPHA,PRICE_O,PRICE_H,PRICE_F);

    nStorms=sum(demoWeather);
    fprintf('  Demo weather: %d storms / %d days\n',nStorms,MAX_DAYS);
    fprintf('  Adaptive result: OK=%d Z=%d M=%d Days=%d/%d\n',...
        adaptOK,adaptZ,adaptM,adaptDays,MAX_DAYS);

    %% ==================== Phase 4: MC Validation of Adaptive Strategy ====================
    fprintf('\n[Phase 4] MC validation of adaptive strategy (%d scenarios)...\n',N_MC_ADAPTIVE);

    rng(2026);
    adaptSuccesses=0; adaptZvals=zeros(N_MC_ADAPTIVE,1);
    adaptMvals=zeros(N_MC_ADAPTIVE,1); adaptDaysVals=zeros(N_MC_ADAPTIVE,1);

    for s=1:N_MC_ADAPTIVE
        weather=rand(MAX_DAYS,1)<PS;
        [ok,Zf,Mf,~,days]=adaptive_execute(...
            bestResult.sk,bestResult.wd,weather,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
            INIT_O,INIT_H,INIT_F,INIT_M,INIT_Z,e,v,Z_ALPHA,PRICE_O,PRICE_H,PRICE_F);
        if ok
            adaptSuccesses=adaptSuccesses+1;
            adaptZvals(s)=Zf;adaptMvals(s)=Mf;adaptDaysVals(s)=days;
        else
            adaptZvals(s)=0;adaptMvals(s)=0;adaptDaysVals(s)=MAX_DAYS+1;
        end
        if mod(s,200)==0
            fprintf('  [%d/%d] success=%.1f%%%%\n',s,N_MC_ADAPTIVE,adaptSuccesses/s*100);
        end
    end

    adaptSuccRate=adaptSuccesses/N_MC_ADAPTIVE;
    adaptExpZ=mean(adaptZvals);
    adaptStdZ=std(adaptZvals(adaptZvals>0)); if adaptSuccesses==0,adaptStdZ=0;end
    adaptAvgM=mean(adaptMvals(adaptMvals>0)); if adaptSuccesses==0,adaptAvgM=0;end
    adaptAvgDays=mean(adaptDaysVals(adaptDaysVals<=MAX_DAYS));
    if adaptSuccesses==0,adaptAvgDays=0;end

    fprintf('\n  Adaptive strategy validation:\n');
    fprintf('    Success rate:  %.1f%%%% (%d/%d)\n',adaptSuccRate*100,adaptSuccesses,N_MC_ADAPTIVE);
    fprintf('    Expected Z:    %.0f\n',adaptExpZ);
    fprintf('    Z std (success): %.0f\n',adaptStdZ);
    fprintf('    Avg M (success): %.0f\n',adaptAvgM);
    fprintf('    Avg Days (success): %.0f\n',adaptAvgDays);

    %% ==================== Comparison Summary ====================
    fprintf('\n=============================================================\n');
    fprintf('  COMPARISON SUMMARY\n');
    fprintf('=============================================================\n');
    fprintf('  %-40s %8s %8s %10s %10s\n','Method','Z','M','Success%','E[Z]');
    fprintf('  %-40s %8s %8s %10s %10s\n','----','---','---','------','----');
    fprintf('  %-40s %8s %8s %10s %10s\n','Original CP (expected-value)','596','14','UNKNOWN','UNKNOWN');
    fprintf('  %-40s %8d %8d %9.1f %10.0f\n',...
        ['Fixed plan: ' strjoin(bestResult.sk,'->')],...
        bestResult.Zdet,bestResult.Mdet,bestResult.successRate*100,bestResult.expectedZ);
    fprintf('  %-40s %8s %8s %9.1f %10.0f\n',...
        ['Adaptive: ' strjoin(bestResult.sk,'->')],...
        'varies','varies',adaptSuccRate*100,adaptExpZ);

    fprintf('\n  Total time: %.1fs\n',toc(t0));

    %% ==================== Export Results ====================
    fprintf('\n[Export] Writing results...\n');
    write_results_improved(bestResult,mcResults,adaptSuccRate,adaptExpZ,adaptStdZ,adaptAvgM,...
        N_MC_CANDIDATE,N_MC_ADAPTIVE,Z_ALPHA,XY,WI,adaptLog,adaptAvgDays);
    fprintf('  -> result_improved.xlsx\n');
    fprintf('  -> result_improved.json\n');
    fprintf('\nDone.\n');
end

%% ==================== Helper: Manhattan Distance ====================
function d=md(a,b),d=abs(a(1)-b(1))+abs(a(2)-b(2));end
function d=skeldist(sk,fn),d=0;for i=1:length(sk)-1,d=d+fn(sk{i},sk{i+1});end,end
function h=haswork(sk,WI),h=false;for i=1:length(sk),if isfield(WI,sk{i}),h=true;return;end,end,end

%% ==================== Max Work with Park ====================
function best=mwp(wm,R)
    if R<=0||wm<=0,best=0;return;end
    best=min(wm,R);
    for k=1:R
        need=k*wm+(k-1);
        if need<=R,best=max(best,k*wm+min(wm,R-need));
        else,break;end
    end
end

%% ==================== Park Days from Work Days ====================
function pk=pdfw(wd,wm)
    if wd<=0,pk=0;return;end
    pk=ceil(wd/wm)-1;
end

%% ==================== Skeleton Enumeration ====================
function skels=enum_skels(distFn,IN,MAX_DAYS,MAX_CNT)
    skels={};
    function dfs(path,td)
        if length(skels)>=MAX_CNT,return;end
        cur=path{end};dE=distFn(cur,'E');
        if td+dE<=MAX_DAYS,skels{end+1}=[path,{'E'}];end
        if length(path)-1>=8,return;end
        for ni=1:length(IN)
            nxt=IN{ni};if strcmp(nxt,cur),continue;end
            d=distFn(cur,nxt);
            if d==0||td+d+distFn(nxt,'E')>MAX_DAYS,continue;end
            dfs([path,{nxt}],td+d);
        end
    end
    dfs({'B'},0);
end

%% ==================== Cartesian Product ====================
function C=cp(ranges)
    n=length(ranges);lens=zeros(1,n);
    for i=1:n,lens(i)=length(ranges{i});end
    total=prod(lens);C=zeros(total,n);
    for i=1:total
        t=i-1;
        for j=n:-1:1
            rng=ranges{j};C(i,j)=rng(mod(t,lens(j))+1);t=floor(t/lens(j));
        end
    end
end

%% ==================== Statistical Safety-Margin Supply ====================
% Computes purchase amounts using expected consumption + z*sigma safety buffer.
% Replaces the original ad-hoc (1.25/1.05/1.05) multipliers with
% quantile-based margins derived from consumption variances.
function [bo,bh,bf,cost]=supply_statistical(sk,seg,wd,XY,WI,distFn,MAX_LOAD,O,H,F,M,...
        e,v,z_alpha,PO,PH,PF)
    ns=length(sk);
    for j=seg+2:length(sk)
        if any(strcmp(sk{j},{'S1','S2','E'})),ns=j;break;end
    end

    eo=0;eh=0;ef=0;
    vo=0;vh=0;vf=0;

    % Travel consumption (expected + variance)
    for j=seg+1:min(ns,length(sk))-1
        a=sk{j};b=sk{j+1};d=distFn(a,b);
        eo=eo+e.MO*d;eh=eh+e.MH*d;ef=ef+e.MF*d;
        vo=vo+v.MO*d;vh=vh+v.MH*d;vf=vf+v.MF*d;
    end

    % Work + park consumption
    for j=seg+1:min(ns,length(sk))
        nm=sk{j};
        if isfield(WI,nm)
            wdv=0;if isKey(wd,nm),wdv=wd(nm);end
            if wdv>0
                wm=WI.(nm);wm=wm(2);npk=pdfw(wdv,wm);
                eo=eo+e.WO*wdv+e.PO*npk;
                eh=eh+e.WH*wdv+e.PH*npk;
                ef=ef+e.WF*wdv+e.PF*npk;
                vo=vo+v.WO*wdv+v.PO*npk;
                vh=vh+v.WH*wdv+v.PH*npk;
                vf=vf+v.WF*wdv+v.PF*npk;
            end
        end
    end

    % One day for the supply action itself
    eo=eo+e.PO;eh=eh+e.PH;ef=ef+e.PF;
    vo=vo+v.PO;vh=vh+v.PH;vf=vf+v.PF;

    % Statistical safety buffer: z_alpha * sqrt(variance)
    bufO=z_alpha*sqrt(max(0,vo));
    bufH=z_alpha*sqrt(max(0,vh));
    bufF=z_alpha*sqrt(max(0,vf));

    needO=eo+bufO;needH=eh+bufH;needF=ef+bufF;

    bo=max(0,ceil(needO-O));bh=max(0,ceil(needH-H));bf=max(0,ceil(needF-F));

    % Load constraint
    sp=MAX_LOAD-(O+H+F);tot=bo+bh+bf;
    if tot>sp&&sp>0
        sc=sp/tot;bo=floor(bo*sc);bh=floor(bh*sc);bf=floor(bf*sc);
    end

    % Budget constraint
    costRaw=bo*PO+bh*PH+bf*PF;
    if costRaw>M&&M>0
        sc=M/costRaw;bo=floor(bo*sc);bh=floor(bh*sc);bf=floor(bf*sc);
    end
    cost=bo*PO+bh*PH+bf*PF;
end

%% ==================== Deterministic Simulation (for CP candidate generation) ====================
% Uses expected-value consumption. Used only for quickly filtering candidates.
function [ok,Zf,Mf,log,nDays]=simul_deterministic(sk,wd,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
        IO,IH,IF,IM,IZ,e,v,z_alpha,PO,PH,PF)
    O=IO;H=IH;F=IF;M=IM;Z=IZ;day=1;
    log={struct('day',0,'x',1,'y',15,'weather','-','action','Start',...
        'point','B','O',O,'H',H,'F',F,'M',M,'Z',Z,'c',0,...
        'buyO',0,'buyH',0,'buyF',0)};

    for seg=1:length(sk)-1
        cur=sk{seg};nxt=sk{seg+1};d=distFn(cur,nxt);
        cx=XY.(cur);cxx=cx(1);cxy=cx(2);
        tx=XY.(nxt);txx=tx(1);txy=tx(2);

        for st=1:d
            if day>MAX_DAYS,ok=false;Zf=Z;Mf=M;nDays=day;return;end
            log{end+1}=struct('day',day,'x',cxx,'y',cxy,'weather','N',...
                'action','move','point','','O',max(0,O),'H',max(0,H),'F',max(0,F),...
                'M',M,'Z',Z,'c',0,'buyO',0,'buyH',0,'buyF',0);
            O=O-e.MO;H=H-e.MH;F=F-e.MF;
            if O<0||H<0||F<0,ok=false;Zf=0;Mf=0;nDays=day;return;end
            if cxx~=txx,cxx=cxx+sign(txx-cxx);elseif cxy~=txy,cxy=cxy+sign(txy-cxy);end
            day=day+1;
        end

        if any(strcmp(nxt,{'S1','S2'}))
            [bo,bh,bf,cst]=supply_statistical(sk,seg,wd,XY,WI,distFn,MAX_LOAD,O,H,F,M,...
                e,v,z_alpha,PO,PH,PF);
            if cst<=M,O=O+bo;H=H+bh;F=F+bf;M=M-cst;
                actStr=sprintf('buy(O=%d,H=%d,F=%d)',bo,bh,bf);
            else,actStr='buy(0,0,0)';end
            O=O-e.PO;H=H-e.PH;F=F-e.PF;
            log{end+1}=struct('day',day,'x',cxx,'y',cxy,'weather','N',...
                'action',actStr,'point',nxt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                'M',M,'Z',Z,'c',0,'buyO',bo,'buyH',bh,'buyF',bf);
            if O<0||H<0||F<0||M<0,ok=false;Zf=Z;Mf=M;nDays=day;return;end
            day=day+1;
        end

        if isfield(WI,nxt)
            wdv=0;if isKey(wd,nxt),wdv=wd(nxt);end
            if wdv>0
                gain=WI.(nxt);gain=gain(1);wm=WI.(nxt);wm=wm(2);
                consec=0;done=0;
                while done<wdv
                    if day>MAX_DAYS,ok=false;Zf=Z;Mf=M;nDays=day;return;end
                    if consec<wm
                        log{end+1}=struct('day',day,'x',cxx,'y',cxy,'weather','N',...
                            'action','work','point',nxt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                            'M',M,'Z',Z,'c',consec,'buyO',0,'buyH',0,'buyF',0);
                        O=O-e.WO;H=H-e.WH;F=F-e.WF;Z=Z+gain;consec=consec+1;done=done+1;
                    else
                        log{end+1}=struct('day',day,'x',cxx,'y',cxy,'weather','N',...
                            'action','park(reset)','point',nxt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                            'M',M,'Z',Z,'c',consec,'buyO',0,'buyH',0,'buyF',0);
                        O=O-e.PO;H=H-e.PH;F=F-e.PF;consec=0;
                    end
                    if O<0||H<0||F<0,ok=false;Zf=0;Mf=0;nDays=day;return;end
                    day=day+1;
                end
            end
        end

        if strcmp(nxt,'E'),ok=true;Zf=Z;Mf=M;nDays=day;return;end
    end
    ok=false;Zf=Z;Mf=M;nDays=day;
end

%% ==================== Stochastic Simulation (Fixed Plan, for Monte Carlo) ====================
% Simulates a fixed plan under ACTUAL random weather.
% Uses statistical safety margins for purchases (same amounts as planned).
% This is the core of Phase 2 SAA evaluation.
function [ok,Zf,Mf,nDays]=simul_stochastic_fixed(sk,wd,weather,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
        IO,IH,IF,IM,IZ,e,v,z_alpha,PO,PH,PF)
    O=IO;H=IH;F=IF;M=IM;Z=IZ;day=1;

    for seg=1:length(sk)-1
        cur=sk{seg};nxt=sk{seg+1};d=distFn(cur,nxt);
        cx=XY.(cur);cxx=cx(1);cxy=cx(2);
        tx=XY.(nxt);txx=tx(1);txy=tx(2);

        % Travel with ACTUAL weather draws
        for st=1:d
            if day>MAX_DAYS,ok=false;Zf=Z;Mf=M;nDays=day;return;end
            isStorm=weather(day);
            if isStorm
                O=O-8;H=H-4;F=F-3;
            else
                O=O-2;H=H-3;F=F-2;
            end
            if O<0||H<0||F<0,ok=false;Zf=0;Mf=0;nDays=day;return;end
            if cxx~=txx,cxx=cxx+sign(txx-cxx);elseif cxy~=txy,cxy=cxy+sign(txy-cxy);end
            day=day+1;
        end

        % Supply: use statistical margins for purchase
        if any(strcmp(nxt,{'S1','S2'}))
            [bo,bh,bf,cst]=supply_statistical(sk,seg,wd,XY,WI,distFn,MAX_LOAD,O,H,F,M,...
                e,v,z_alpha,PO,PH,PF);
            if cst<=M,O=O+bo;H=H+bh;F=F+bf;M=M-cst;end
            % Supply day consumption (actual weather)
            if day>MAX_DAYS,ok=false;Zf=Z;Mf=M;nDays=day;return;end
            isStorm=weather(day);
            if isStorm,O=O-3;H=H-3;F=F-2;
            else,O=O-1;H=H-1;F=F-1;end
            if O<0||H<0||F<0||M<0,ok=false;Zf=Z;Mf=M;nDays=day;return;end
            day=day+1;
        end

        % Work point with actual weather
        if isfield(WI,nxt)
            wdv=0;if isKey(wd,nxt),wdv=wd(nxt);end
            if wdv>0
                gain=WI.(nxt);gain=gain(1);wm=WI.(nxt);wm=wm(2);
                consec=0;done=0;
                while done<wdv
                    if day>MAX_DAYS,ok=false;Zf=Z;Mf=M;nDays=day;return;end
                    isStorm=weather(day);
                    if consec<wm
                        if isStorm,O=O-8;H=H-6;F=F-6;
                        else,O=O-5;H=H-4;F=F-3;end
                        Z=Z+gain;consec=consec+1;done=done+1;
                    else
                        if isStorm,O=O-3;H=H-3;F=F-2;
                        else,O=O-1;H=H-1;F=F-1;end
                        consec=0;
                    end
                    if O<0||H<0||F<0,ok=false;Zf=0;Mf=0;nDays=day;return;end
                    day=day+1;
                end
            end
        end

        if strcmp(nxt,'E'),ok=true;Zf=Z;Mf=M;nDays=day;return;end
    end
    ok=false;Zf=Z;Mf=M;nDays=day;
end

%% ==================== Online Adaptive Rolling-Horizon Execution ====================
% Executes the plan adaptively: at each supply/work point, re-optimizes
% remaining path and resource allocation based on current state.
% Uses statistical safety margins for purchases.
% Weather is observed daily; consumption uses ACTUAL weather.
function [ok,Zf,Mf,log,finalDay]=adaptive_execute(sk_init,wd_init,weather,XY,WI,distFn,...
        MAX_DAYS,MAX_LOAD,IO,IH,IF,IM,IZ,e,v,z_alpha,PO,PH,PF)
    O=IO;H=IH;F=IF;M=IM;Z=IZ;day=1;
    curPt='B';curX=XY.B(1);curY=XY.B(2);

    % Deep-copy the initial work-day plan
    sk=sk_init;
    wd=containers.Map();
    wpKeys=keys(wd_init);
    for i=1:length(wpKeys),wd(wpKeys{i})=wd_init(wpKeys{i});end

    log={struct('day',0,'x',curX,'y',curY,'weather','-','action','Start',...
        'point','B','O',O,'H',H,'F',F,'M',M,'Z',Z,'c',0,...
        'buyO',0,'buyH',0,'buyF',0)};

    skIdx=1; subStep=0;
    workDone=0; workConsec=0;

    while day<=MAX_DAYS && ~strcmp(curPt,'E')
        isStorm=weather(day);
        wLabel='N';if isStorm,wLabel='S';end

        % --- Adaptive Replanning at POIs ---
        if subStep==0 && any(strcmp(curPt,{'B','S1','S2','W1','W2','W3'}))
            needReplan = any(strcmp(curPt,{'S1','S2'})) || ...
                         (isfield(WI,curPt) && skIdx>1);

            if needReplan
                remDays=MAX_DAYS-day+1;
                % Build remaining path
                remSk={curPt};
                for i=(skIdx+1):length(sk),remSk{end+1}=sk{i};end
                if ~strcmp(remSk{end},'E'),remSk{end+1}='E';end

                % Travel from current to E
                remTravel=0;
                for i=1:length(remSk)-1
                    remTravel=remTravel+distFn(remSk{i},remSk{i+1});
                end
                nSupRem=sum(cellfun(@(x)any(strcmp(x,{'S1','S2'})),remSk(2:end-1)));
                avail=remDays-remTravel-nSupRem;

                % Re-assess work feasibility for remaining work points
                for i=2:length(remSk)
                    nm=remSk{i};
                    if isfield(WI,nm) && isKey(wd,nm)
                        planned=wd(nm);
                        wm=WI.(nm);wm=wm(2);
                        maxFeasible=mwp(wm,avail);
                        if planned>maxFeasible
                            wd(nm)=maxFeasible;
                        end
                    end
                end
            end
        end

        % --- Execute Action ---
        nxt=sk{skIdx+1};
        dSeg=distFn(curPt,nxt);

        if subStep<dSeg
            % Travel
            if isStorm,O=O-8;H=H-4;F=F-3;
            else,O=O-2;H=H-3;F=F-2;end
            if O<0||H<0||F<0
                log{end+1}=struct('day',day,'x',curX,'y',curY,'weather',wLabel,...
                    'action','FAIL(O)','point',curPt,'O',0,'H',max(0,H),'F',max(0,F),...
                    'M',M,'Z',Z,'c',0,'buyO',0,'buyH',0,'buyF',0);
                ok=false;Zf=0;Mf=0;finalDay=day;return;
            end
            tx=XY.(nxt);txx=tx(1);txy=tx(2);
            if curX~=txx,curX=curX+sign(txx-curX);
            elseif curY~=txy,curY=curY+sign(txy-curY);end
            subStep=subStep+1;
            log{end+1}=struct('day',day,'x',curX,'y',curY,'weather',wLabel,...
                'action','move','point',curPt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                'M',M,'Z',Z,'c',0,'buyO',0,'buyH',0,'buyF',0);
            if subStep>=dSeg,curPt=nxt;skIdx=skIdx+1;subStep=0;workDone=0;workConsec=0;end
            day=day+1;

        elseif any(strcmp(curPt,{'S1','S2'}))
            [bo,bh,bf,cst]=supply_statistical(sk,skIdx-1,wd,XY,WI,distFn,MAX_LOAD,O,H,F,M,...
                e,v,z_alpha,PO,PH,PF);
            if cst<=M,O=O+bo;H=H+bh;F=F+bf;M=M-cst;
                actStr=sprintf('buy(O=%d,H=%d,F=%d)',bo,bh,bf);
            else,actStr='buy(0,0,0)';bo=0;bh=0;bf=0;end
            if isStorm,O=O-3;H=H-3;F=F-2;
            else,O=O-1;H=H-1;F=F-1;end
            if O<0||H<0||F<0
                ok=false;Zf=Z;Mf=M;finalDay=day;
                log{end+1}=struct('day',day,'x',curX,'y',curY,'weather',wLabel,...
                    'action','FAIL(supply)','point',curPt,'O',0,'H',max(0,H),'F',max(0,F),...
                    'M',M,'Z',Z,'c',0,'buyO',bo,'buyH',bh,'buyF',bf);
                return;
            end
            log{end+1}=struct('day',day,'x',curX,'y',curY,'weather',wLabel,...
                'action',actStr,'point',curPt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                'M',M,'Z',Z,'c',0,'buyO',bo,'buyH',bh,'buyF',bf);
            day=day+1;

        elseif isfield(WI,curPt)
            wdv=0;if isKey(wd,curPt),wdv=wd(curPt);end
            if wdv>0 && workDone<wdv
                wm=WI.(curPt);wm=wm(2);gain=WI.(curPt);gain=gain(1);
                if workConsec<wm
                    if isStorm,O=O-8;H=H-6;F=F-6;
                    else,O=O-5;H=H-4;F=F-3;end
                    Z=Z+gain;workConsec=workConsec+1;workDone=workDone+1;
                    log{end+1}=struct('day',day,'x',curX,'y',curY,'weather',wLabel,...
                        'action','work','point',curPt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                        'M',M,'Z',Z,'c',workConsec,'buyO',0,'buyH',0,'buyF',0);
                else
                    if isStorm,O=O-3;H=H-3;F=F-2;
                    else,O=O-1;H=H-1;F=F-1;end
                    workConsec=0;
                    log{end+1}=struct('day',day,'x',curX,'y',curY,'weather',wLabel,...
                        'action','park(reset)','point',curPt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                        'M',M,'Z',Z,'c',0,'buyO',0,'buyH',0,'buyF',0);
                end
                if O<0||H<0||F<0,ok=false;Zf=0;Mf=0;finalDay=day;return;end
                day=day+1;
            else
                skIdx=skIdx+1;subStep=0;workDone=0;workConsec=0;
            end

        elseif strcmp(curPt,'E')
            ok=true;Zf=Z;Mf=M;finalDay=day;return;
        else
            skIdx=skIdx+1;subStep=0;
        end
    end

    if strcmp(curPt,'E'),ok=true;Zf=Z;Mf=M;finalDay=day;
    else,ok=false;Zf=Z;Mf=M;finalDay=day;end
end

%% ==================== Export Results ====================
function write_results_improved(bestResult,mcResults,adaptSuccRate,adaptExpZ,...
        adaptStdZ,adaptAvgM,N_MC,N_MC_ADAPT,Z_ALPHA,XY,WI,adaptLog,adaptAvgDays)
    % --- Excel: daily schedule from adaptive demo ---
    nRows=length(adaptLog);
    data=cell(nRows+1,15);
    hd={'Day','PosX','PosY','Weather','Action','Point',...
        'FuelO','WaterH','FoodF','MoneyM','TargetZ',...
        'c','BuyO','BuyH','BuyF'};
    for c=1:15,data{1,c}=hd{c};end
    for r=1:nRows
        e=adaptLog{r};data{r+1,1}=e.day;data{r+1,2}=e.x;data{r+1,3}=e.y;
        data{r+1,4}=e.weather;data{r+1,5}=e.action;
        if isempty(e.point),data{r+1,6}='';else,data{r+1,6}=e.point;end
        data{r+1,7}=round(e.O,1);data{r+1,8}=round(e.H,1);data{r+1,9}=round(e.F,1);
        data{r+1,10}=round(e.M,1);data{r+1,11}=e.Z;data{r+1,12}=e.c;
        data{r+1,13}=e.buyO;data{r+1,14}=e.buyH;data{r+1,15}=e.buyF;
    end
    T=cell2table(data(2:end,:),'VariableNames',data(1,:));
    writetable(T,'result_improved.xlsx','Sheet','AdaptiveDemo');

    % --- Excel: candidate comparison ---
    compData=cell(length(mcResults)+1,8);
    compData(1,:)={'Path','Z_det','M_det','SuccRate','ExpZ','Z_std','M_avg','Succes'};
    for ci=1:length(mcResults)
        r=mcResults{ci};
        compData{ci+1,1}=strjoin(r.sk,'->');
        compData{ci+1,2}=r.Zdet;compData{ci+1,3}=r.Mdet;
        compData{ci+1,4}=r.successRate;compData{ci+1,5}=r.expectedZ;
        compData{ci+1,6}=r.Z_std;compData{ci+1,7}=r.M_avg;
        compData{ci+1,8}=r.successes;
    end
    T2=cell2table(compData(2:end,:),'VariableNames',compData(1,:));
    writetable(T2,'result_improved.xlsx','Sheet','CandidateComparison');

    % --- Excel: Summary ---
    sumData={'Metric','Value';
        'Best Path',strjoin(bestResult.sk,'->');
        'Deterministic Z',bestResult.Zdet;
        'Deterministic M',bestResult.Mdet;
        'Fixed Plan Success Rate',sprintf('%.1f%%',bestResult.successRate*100);
        'Fixed Plan Expected Z',bestResult.expectedZ;
        'Fixed Plan Z std (success only)',bestResult.Z_std;
        'Fixed Plan Avg M (success only)',bestResult.M_avg;
        'Adaptive Success Rate',sprintf('%.1f%%',adaptSuccRate*100);
        'Adaptive Expected Z',adaptExpZ;
        'Adaptive Z std (success only)',adaptStdZ;
        'Adaptive Avg M (success only)',adaptAvgM;
        'Adaptive Avg Days (success only)',adaptAvgDays;
        'z_alpha',Z_ALPHA;
        'Confidence Level',sprintf('%.0f%%',0.5*erfc(-Z_ALPHA/sqrt(2))*100);
        'MC Scenarios (candidate eval)',N_MC;
        'MC Scenarios (adaptive validation)',N_MC_ADAPT};
    T3=cell2table(sumData(2:end,:),'VariableNames',sumData(1,:));
    writetable(T3,'result_improved.xlsx','Sheet','Summary');

    % --- JSON ---
    sm=struct();
    sm.problem='Task 3 (Improved: Statistical Safety + MC + Adaptive RHR)';
    sm.method='CP + Statistical Safety Margins + Monte Carlo SAA + Adaptive Rolling Horizon';
    sm.best_path=strjoin(bestResult.sk,'->');
    sm.Z_deterministic=bestResult.Zdet;
    sm.M_deterministic=bestResult.Mdet;
    sm.fixed_success_rate=bestResult.successRate;
    sm.fixed_expected_Z=bestResult.expectedZ;
    sm.adaptive_success_rate=adaptSuccRate;
    sm.adaptive_expected_Z=adaptExpZ;
    sm.adaptive_Z_std=adaptStdZ;
    sm.adaptive_avg_M=adaptAvgM;
    sm.adaptive_avg_days=adaptAvgDays;
    sm.z_alpha=Z_ALPHA;
    sm.confidence_level=0.5*erfc(-Z_ALPHA/sqrt(2));
    sm.MC_scenarios_candidate=N_MC;
    sm.MC_scenarios_adaptive=N_MC_ADAPT;
    wdStruct=struct();
    wpKeys=keys(bestResult.wd);
    for i=1:length(wpKeys),nm=wpKeys{i};wdStruct.(nm)=bestResult.wd(nm);end
    sm.work_days=wdStruct;

    fid=fopen('result_improved.json','w');
    fprintf(fid,'%s\n',jsonencode(sm,'PrettyPrint',true));
    fclose(fid);
end
