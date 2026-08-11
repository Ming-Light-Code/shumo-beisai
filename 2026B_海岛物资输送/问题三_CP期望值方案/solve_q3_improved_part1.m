%% ==========================================================================
% 2026B Problem 3: Improved Stochastic Optimization Solver (MATLAB)
% Method: CP + Statistical Safety Margins + Monte Carlo SAA + Adaptive RHR
% ==========================================================================
% Improvements over the original CP expected-value solver:
%   1. Statistical (z-score) safety margins replace ad-hoc multipliers
%   2. Monte Carlo SAA evaluates candidate plans under real weather
%   3. Online adaptive rolling-horizon replanning at each POI
%   4. Full Monte Carlo validation of the adaptive strategy
%   5. Reports success probability, expected Z, and risk-return tradeoffs
% ==========================================================================

function solve_q3_improved()
    t0 = tic;
    fprintf(''=============================================================\n'');
    fprintf('' Task 3: Improved Stochastic Optimization Solver\n'');
    fprintf(''=============================================================\n'');

    %% ==================== Parameters ====================
    MAX_DAYS = 90;  MAX_LOAD = 400;
    INIT_O = 100;   INIT_H = 150;  INIT_F = 100;
    INIT_M = 750;   INIT_Z = 200;
    PRICE_O = 2;    PRICE_H = 1;   PRICE_F = 2;
    PN = 0.8;       PS = 0.2;
    Z_ALPHA = 1.645;   % z-score for 95%% one-sided confidence
    N_MC_CANDIDATE = 500;   % Monte Carlo scenarios per candidate
    N_MC_ADAPTIVE  = 1000;  % Monte Carlo scenarios for adaptive validation
    MAX_CNT = 50000;

    %% ==================== Expected Values & Variances ====================
    e = struct(''MO'',PN*2+PS*8, ''MH'',PN*3+PS*4, ''MF'',PN*2+PS*3, ...
               ''PO'',PN*1+PS*3, ''PH'',PN*1+PS*3, ''PF'',PN*1+PS*2, ...
               ''WO'',PN*5+PS*8, ''WH'',PN*4+PS*6, ''WF'',PN*3+PS*6);
    v = struct(''MO'',PN*(2-e.MO)^2+PS*(8-e.MO)^2, ...
               ''MH'',PN*(3-e.MH)^2+PS*(4-e.MH)^2, ...
               ''MF'',PN*(2-e.MF)^2+PS*(3-e.MF)^2, ...
               ''WO'',PN*(5-e.WO)^2+PS*(8-e.WO)^2, ...
               ''WH'',PN*(4-e.WH)^2+PS*(6-e.WH)^2, ...
               ''WF'',PN*(3-e.WF)^2+PS*(6-e.WF)^2, ...
               ''PO'',PN*(1-e.PO)^2+PS*(3-e.PO)^2, ...
               ''PH'',PN*(1-e.PH)^2+PS*(3-e.PH)^2, ...
               ''PF'',PN*(1-e.PF)^2+PS*(2-e.PF)^2);

    %% ==================== Coordinate & POI Setup ====================
    XY.B=[1 15]; XY.E=[30 15]; XY.W1=[6 21]; XY.W2=[15 9];
    XY.W3=[24 24]; XY.S1=[12 16]; XY.S2=[21 16];
    AN = {''B'',''E'',''W1'',''W2'',''W3'',''S1'',''S2''};
    WI.W1=[20 4]; WI.W2=[15 5]; WI.W3=[28 3];
    IN = {''W1'',''W2'',''W3'',''S1'',''S2''};

    D=zeros(7);
    for i=1:7,for j=1:7,D(i,j)=md(XY.(AN{i}),XY.(AN{j}));end,end
    distFn=@(a,b)D(find(strcmp(AN,a)),find(strcmp(AN,b)));

    fprintf(''  Grid: 30x30 | Days: %%d | Load: %%d\n'',MAX_DAYS,MAX_LOAD);
    fprintf(''  P(N)=%%.1f P(S)=%%.1f | z_alpha=%%.3f | MC_cand=%%d MC_adapt=%%d\n'',...
        PN,PS,Z_ALPHA,N_MC_CANDIDATE,N_MC_ADAPTIVE);
    fprintf(''  Statistical safety margins (%%.0f%%%% one-sided confidence)\n'',...
        normcdf(Z_ALPHA)*100);

    %% ==================== Phase 1: CP Candidate Generation ====================
    fprintf(''\n[Phase 1] CP skeleton enumeration + statistical safety margins...\n'');
    skels=enum_skels(distFn,IN,MAX_DAYS,MAX_CNT);
    fprintf(''  Raw skeletons: %%d\n'',length(skels));

    uq={}; seen=containers.Map();
    for i=1:length(skels)
        key=strjoin(skels{i},''|'');
        if ~isKey(seen,key),seen(key)=true;uq{end+1}=skels{i};end
    end
    nUq=length(uq); tdists=zeros(1,nUq);
    for i=1:nUq,tdists(i)=skeldist(uq{i},distFn);end
    [~,order]=sort(tdists); uq=uq(order);
    hw={};
    for i=1:nUq,if haswork(uq{i},WI),hw{end+1}=uq{i};end,end
    fprintf(''  Unique: %%d  With work: %%d\n'',nUq,length(hw));

    bestCandidates={};
    explored=0; pruned=0;
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
        nSup=sum(cellfun(@(x)any(strcmp(x,{''S1'',''S2''})),sk));
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
                    [~,si]=sort(cellfun(@(x)x{3},bestCandidates),''descend'');
                    bestCandidates=bestCandidates(si(1:MAX_CANDIDATES));
                end
            end
        end
        if mod(idx,100)==0
            fprintf(''  [%%d/%%d] candidates=%%d explored=%%d\n'',...
                idx,length(hw),length(bestCandidates),explored);
        end
    end
    fprintf(''  Phase 1 done: %%d candidates, explored=%%d, pruned=%%d (%%.1fs)\n'',...
        length(bestCandidates),explored,pruned,toc(t0));

    %% ==================== Phase 2: Monte Carlo SAA Evaluation ====================
    fprintf(''\n[Phase 2] Monte Carlo SAA evaluation (%%d scenarios each)...\n'',N_MC_CANDIDATE);

    mcResults = cell(length(bestCandidates),1);
    for ci=1:length(bestCandidates)
        sk=bestCandidates{ci}{1}; wd=bestCandidates{ci}{2};
        Zdet=bestCandidates{ci}{3}; Mdet=bestCandidates{ci}{4};

        rng(2026+ci);
        successes=0; Zvals=zeros(N_MC_CANDIDATE,1); Mvals=zeros(N_MC_CANDIDATE,1);
        parfor s=1:N_MC_CANDIDATE
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

        mcResults{ci}=struct(''sk'',{sk},''wd'',{wd},...
            ''Zdet'',Zdet,''Mdet'',Mdet,...
            ''successRate'',successRate,''expectedZ'',expectedZ,...
            ''Z_std'',Z_std,''M_avg'',M_avg,''successes'',successes);

        pathStr=strjoin(sk,''->'');
        fprintf(''  [%%2d] %%s | Zdet=%%d Mdet=%%d | succ=%%.1f%%%% E[Z]=%%.0f\n'',...
            ci,pathStr,Zdet,Mdet,successRate*100,expectedZ);
    end

    [bestExpZ,bestIdx]=max(cellfun(@(x)x.expectedZ,mcResults));
    bestResult=mcResults{bestIdx};
    fprintf(''\n  Selected best: %%s | E[Z]=%%.0f success=%%.1f%%%%\n'',...
        strjoin(bestResult.sk,''->''),bestResult.expectedZ,bestResult.successRate*100);

    %% ==================== Phase 3: Online Adaptive Execution ====================
    fprintf(''\n[Phase 3] Online adaptive rolling-horizon execution...\n'');
    rng(2026);
    demoWeather=rand(MAX_DAYS,1)<PS;

    [adaptOK,adaptZ,adaptM,adaptLog,adaptDays]=adaptive_execute(...
        bestResult.sk,bestResult.wd,demoWeather,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
        INIT_O,INIT_H,INIT_F,INIT_M,INIT_Z,e,v,Z_ALPHA,PRICE_O,PRICE_H,PRICE_F);

    nStorms=sum(demoWeather);
    fprintf(''  Demo weather: %%d storms / %%d days\n'',nStorms,MAX_DAYS);
    fprintf(''  Adaptive result: OK=%%d Z=%%d M=%%d Days=%%d/%%d\n'',...
        adaptOK,adaptZ,adaptM,adaptDays,MAX_DAYS);

    %% ==================== Phase 4: Monte Carlo Validation of Adaptive Strategy ====================
    fprintf(''\n[Phase 4] Monte Carlo validation of adaptive strategy (%%d scenarios)...\n'',N_MC_ADAPTIVE);

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
            fprintf(''  [%%d/%%d] success=%%.1f%%%%\n'',s,N_MC_ADAPTIVE,adaptSuccesses/s*100);
        end
    end

    adaptSuccRate=adaptSuccesses/N_MC_ADAPTIVE;
    adaptExpZ=mean(adaptZvals);
    adaptStdZ=std(adaptZvals(adaptZvals>0)); if adaptSuccesses==0,adaptStdZ=0;end
    adaptAvgM=mean(adaptMvals(adaptMvals>0)); if adaptSuccesses==0,adaptAvgM=0;end

    fprintf(''\n  Adaptive strategy validation:\n'');
    fprintf(''    Success rate:  %%.1f%%%% (%%d/%%d)\n'',adaptSuccRate*100,adaptSuccesses,N_MC_ADAPTIVE);
    fprintf(''    Expected Z:    %%.0f\n'',adaptExpZ);
    fprintf(''    Z std (success): %%.0f\n'',adaptStdZ);
    fprintf(''    Avg M (success): %%.0f\n'',adaptAvgM);

    %% ==================== Comparison Summary ====================
    fprintf(''\n=============================================================\n'');
    fprintf(''  COMPARISON SUMMARY\n'');
    fprintf(''=============================================================\n'');
    fprintf(''  %%-35s %%8s %%8s %%10s %%10s\n'',''Method'',''Z'',''M'',''Success%%'',''E[Z]'');
    fprintf(''  %%-35s %%8s %%8s %%10s %%10s\n'',''---'',''---'',''---'',''------'',''----'');
    fprintf(''  %%-35s %%8s %%8s %%10s %%10s\n'',''Original CP (expected-value)'',''596'',''14'',''UNKNOWN'',''UNKNOWN'');
    fprintf(''  %%-35s %%8d %%8d %%9.1f%%%% %%10.0f\n'',...
        [''Fixed plan: '' strjoin(bestResult.sk,''->'')],...
        bestResult.Zdet,bestResult.Mdet,bestResult.successRate*100,bestResult.expectedZ);
    fprintf(''  %%-35s %%8s %%8s %%9.1f%%%% %%10.0f\n'',...
        [''Adaptive: '' strjoin(bestResult.sk,''->'')],...
        ''varies'',''varies'',adaptSuccRate*100,adaptExpZ);

    fprintf(''\n  Total time: %%.1fs\n'',toc(t0));

    %% ==================== Export Results ====================
    fprintf(''\n[Export] Writing results...\n'');
    write_results_improved(bestResult,mcResults,adaptSuccRate,adaptExpZ,adaptStdZ,adaptAvgM,...
        N_MC_CANDIDATE,N_MC_ADAPTIVE,Z_ALPHA,XY,WI,adaptLog);
    fprintf(''  -> result_improved.xlsx\n'');
    fprintf(''  -> result_improved.json\n'');
    fprintf(''\nDone.\n'');
end
