%% ==========================================================================
% 2026B Problem 3: CP Expectation-Value Solver (MATLAB)
% Method: Constraint Programming + Expected Consumption + Safety Margins
% Strategy: DFS skeleton enum + work-day enum + park-reset + UB pruning
% ==========================================================================

function solve_q3_cp_expected()
    t0 = tic;
    fprintf('=============================================================\n');
    fprintf(' Task 3: CP Expectation-Value Solver (MATLAB)\n');
    fprintf('=============================================================\n');

    MAX_DAYS = 90; MAX_LOAD = 400;
    INIT_O = 100; INIT_H = 150; INIT_F = 100;
    INIT_M = 750; INIT_Z = 200;
    PN = 0.8; PS = 0.2;
    PRICE_O = 2; PRICE_H = 1; PRICE_F = 2;
    SAFETY_O = 1.25; SAFETY_H = 1.05; SAFETY_F = 1.05;
    MAX_CNT = 50000;

    e = struct('MO',PN*2+PS*8,'MH',PN*3+PS*4,'MF',PN*2+PS*3,...
        'PO',PN*1+PS*3,'PH',PN*1+PS*3,'PF',PN*1+PS*2,...
        'WO',PN*5+PS*8,'WH',PN*4+PS*6,'WF',PN*3+PS*6);
    pl = struct('MO',e.MO*SAFETY_O,'MH',e.MH*SAFETY_H,'MF',e.MF*SAFETY_F,...
        'PO',e.PO*SAFETY_O,'PH',e.PH*SAFETY_H,'PF',e.PF*SAFETY_F,...
        'WO',e.WO*SAFETY_O,'WH',e.WH*SAFETY_H,'WF',e.WF*SAFETY_F);

    XY.B=[1 15]; XY.E=[30 15]; XY.W1=[6 21]; XY.W2=[15 9];
    XY.W3=[24 24]; XY.S1=[12 16]; XY.S2=[21 16];
    AN = {'B','E','W1','W2','W3','S1','S2'};
    WI.W1=[20 4]; WI.W2=[15 5]; WI.W3=[28 3];
    IN = {'W1','W2','W3','S1','S2'};

    D=zeros(7);
    for i=1:7,for j=1:7,D(i,j)=md(XY.(AN{i}),XY.(AN{j}));end,end
    distFn=@(a,b)D(find(strcmp(AN,a)),find(strcmp(AN,b)));

    fprintf('  Grid: 30x30 | Days: %d | Load: %d\n',MAX_DAYS,MAX_LOAD);
    fprintf('  P(N)=%.1f P(S)=%.1f | Safety: O=%.2f H=%.2f F=%.2f\n',...
        PN,PS,SAFETY_O,SAFETY_H,SAFETY_F);

    fprintf('\n[Phase 1] Skeleton enumeration...\n');
    skels=enum_skels(distFn,IN,MAX_DAYS,MAX_CNT);
    fprintf('  Generated %d candidates\n',length(skels));

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

    fprintf('\n[Phase 2] CP search (with UB pruning)...\n');
    bestZ=-1; bestM=-1; bestSk={}; bestWd=containers.Map(); bestLog={};
    explored=0; pruned=0;

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

        ubZ=INIT_Z+mwp(3,rem)*28;
        if ubZ<=bestZ
            pruned=pruned+1;
            if mod(idx,300)==0
                fprintf('  [%d/%d] best:Z=%d M=%d pruned=%d explored=%d\n',...
                    idx,length(hw),bestZ,bestM,pruned,explored);
            end
            continue;
        end

        wpUniq=unique(wps);
        ranges=cell(1,length(wpUniq));
        for wi=1:length(wpUniq)
            nm=wpUniq{wi}; wm=WI.(nm); wm=wm(2);
            mw=mwp(wm,rem);
            if mw<=6,rng=0:mw;
            elseif mw<=20,rng=0:mw;
            else,rng=0:2:mw;end
            ranges{wi}=rng;
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
            [ok,Zf,Mf,log]=simul(sk,wd,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
                INIT_O,INIT_H,INIT_F,INIT_M,INIT_Z,e,pl,...
                PRICE_O,PRICE_H,PRICE_F);
            if ok&&(Zf>bestZ||(Zf==bestZ&&Mf>bestM))
                bestZ=Zf;bestM=Mf;bestSk=sk;bestWd=wd;bestLog=log;
            end
        end
        if mod(idx,300)==0
            fprintf('  [%d/%d] best:Z=%d M=%d pruned=%d explored=%d\n',...
                idx,length(hw),bestZ,bestM,pruned,explored);
        end
    end
    fprintf('\n  Total: explored=%d  pruned=%d\n',explored,pruned);

    fprintf('\n============================================================\n');
    fprintf('  OPTIMAL SOLUTION\n');
    fprintf('============================================================\n');
    fprintf('  Path: %s\n',strjoin(bestSk,' -> '));
    fprintf('  Z = %d\n',int32(bestZ));
    fprintf('  M = %d\n',int32(bestM));
    wpKeys=keys(bestWd);fprintf('  Work: ');
    for i=1:length(wpKeys),fprintf('%s=%d  ',wpKeys{i},bestWd(wpKeys{i}));end
    fprintf('\n');
    nDays=sum(cellfun(@(x)x.day>0,bestLog));
    fprintf('  Days: %d/%d\n',nDays,MAX_DAYS);
    fprintf('  Time: %.1fs\n',toc(t0));

    fprintf('\n[Phase 3] Exporting Excel...\n');
    writexlsx(bestSk,bestWd,bestZ,bestM,bestLog,XY);
    fprintf('  result.xlsx  result_q3_cp_expected.json\n');
    fprintf('Done.\n');
end

function d=md(a,b),d=abs(a(1)-b(1))+abs(a(2)-b(2));end
function d=skeldist(sk,fn),d=0;for i=1:length(sk)-1,d=d+fn(sk{i},sk{i+1});end,end
function h=haswork(sk,WI),h=false;for i=1:length(sk),if isfield(WI,sk{i}),h=true;return;end,end,end

function best=mwp(wm,R)
    if R<=0||wm<=0,best=0;return;end
    best=min(wm,R);
    for k=1:R
        need=k*wm+(k-1);
        if need<=R,best=max(best,k*wm+min(wm,R-need));
        else,break;end
    end
end

function pk=pdfw(wd,wm)
    if wd<=0,pk=0;return;end
    pk=ceil(wd/wm)-1;
end

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

function [bo,bh,bf]=supply(sk,seg,wd,XY,WI,distFn,MAX_LOAD,O,H,F,M,pl,PO,PH,PF)
    ns=length(sk);
    for j=seg+2:length(sk)
        if any(strcmp(sk{j},{'S1','S2','E'})),ns=j;break;end
    end
    no=0;nh=0;nf=0;
    for j=seg+1:min(ns,length(sk))-1
        a=sk{j};b=sk{j+1};d=distFn(a,b);
        no=no+pl.MO*d;nh=nh+pl.MH*d;nf=nf+pl.MF*d;
    end
    for j=seg+1:min(ns,length(sk))
        nm=sk{j};
        if isfield(WI,nm)
            wdv=0;if isKey(wd,nm),wdv=wd(nm);end
            if wdv>0
                wm=WI.(nm);wm=wm(2);npk=pdfw(wdv,wm);
                no=no+pl.WO*wdv+pl.PO*npk;
                nh=nh+pl.WH*wdv+pl.PH*npk;
                nf=nf+pl.WF*wdv+pl.PF*npk;
            end
        end
    end
    no=no+pl.PO;nh=nh+pl.PH;nf=nf+pl.PF;
    bo=max(0,ceil(no-O));bh=max(0,ceil(nh-H));bf=max(0,ceil(nf-F));
    sp=MAX_LOAD-(O+H+F);tot=bo+bh+bf;
    if tot>sp&&sp>0,sc=sp/tot;bo=floor(bo*sc);bh=floor(bh*sc);bf=floor(bf*sc);end
    cst=bo*PO+bh*PH+bf*PF;
    if cst>M&&M>0,sc=M/cst;bo=floor(bo*sc);bh=floor(bh*sc);bf=floor(bf*sc);end
end

function [ok,Zf,Mf,log]=simul(sk,wd,XY,WI,distFn,MAX_DAYS,MAX_LOAD,...
        IO,IH,IF,IM,IZ,e,pl,PO,PH,PF)
    O=IO;H=IH;F=IF;M=IM;Z=IZ;day=1;log={};
    log{end+1}=struct('day',0,'x',1,'y',15,'weather','-','action','Start',...
        'point','B','O',O,'H',H,'F',F,'M',M,'Z',Z,'c',0,...
        'buyO',0,'buyH',0,'buyF',0);
    for seg=1:length(sk)-1
        cur=sk{seg};nxt=sk{seg+1};d=distFn(cur,nxt);
        cx=XY.(cur);cxx=cx(1);cxy=cx(2);
        tx=XY.(nxt);txx=tx(1);txy=tx(2);
        for st=1:d
            if day>MAX_DAYS,ok=false;Zf=Z;Mf=M;return;end
            pt='';
            if st==1&&any(strcmp(cur,{'B','W1','W2','W3','S1','S2','E'})),pt=cur;end
            log{end+1}=struct('day',day,'x',cxx,'y',cxy,'weather','N',...
                'action','move','point',pt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                'M',M,'Z',Z,'c',0,'buyO',0,'buyH',0,'buyF',0);
            O=O-e.MO;H=H-e.MH;F=F-e.MF;
            if O<0||H<0||F<0,ok=false;Zf=0;Mf=0;return;end
            if cxx~=txx,cxx=cxx+sign(txx-cxx);elseif cxy~=txy,cxy=cxy+sign(txy-cxy);end
            day=day+1;
        end
        if any(strcmp(nxt,{'S1','S2'}))
            [bo,bh,bf]=supply(sk,seg,wd,XY,WI,distFn,MAX_LOAD,O,H,F,M,pl,PO,PH,PF);
            cst=bo*PO+bh*PH+bf*PF;
            if cst<=M,O=O+bo;H=H+bh;F=F+bf;M=M-cst;
                actStr=sprintf('buy(%d,%d,%d)',bo,bh,bf);
            else,actStr='buy(0,0,0)';end
            O=O-e.PO;H=H-e.PH;F=F-e.PF;
            log{end+1}=struct('day',day,'x',cxx,'y',cxy,'weather','N',...
                'action',actStr,'point',nxt,'O',max(0,O),'H',max(0,H),'F',max(0,F),...
                'M',M,'Z',Z,'c',0,'buyO',bo,'buyH',bh,'buyF',bf);
            if O<0||H<0||F<0||M<0,ok=false;Zf=Z;Mf=M;return;end
            day=day+1;
        end
        if isfield(WI,nxt)
            wdv=0;if isKey(wd,nxt),wdv=wd(nxt);end
            if wdv>0
                gain=WI.(nxt);gain=gain(1);wm=WI.(nxt);wm=wm(2);
                consec=0;done=0;
                while done<wdv
                    if day>MAX_DAYS,ok=false;Zf=Z;Mf=M;return;end
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
                    if O<0||H<0||F<0,ok=false;Zf=0;Mf=0;return;end
                    day=day+1;
                end
            end
        end
        if strcmp(nxt,'E'),ok=true;Zf=Z;Mf=M;return;end
    end
    ok=false;Zf=Z;Mf=M;
end

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

function writexlsx(sk,wd,Z,M,log,XY)
    nRows=length(log);data=cell(nRows+1,15);
    hd={'Day','PosX','PosY','Weather','Action','Point',...
        'FuelO','WaterH','FoodF','MoneyM','TargetZ',...
        'c','BuyO','BuyH','BuyF'};
    for c=1:15,data{1,c}=hd{c};end
    for r=1:nRows
        e=log{r};data{r+1,1}=e.day;data{r+1,2}=e.x;data{r+1,3}=e.y;
        data{r+1,4}=e.weather;data{r+1,5}=e.action;
        if isempty(e.point),data{r+1,6}='';else,data{r+1,6}=e.point;end
        data{r+1,7}=round(e.O,1);data{r+1,8}=round(e.H,1);data{r+1,9}=round(e.F,1);
        data{r+1,10}=round(e.M,1);data{r+1,11}=e.Z;data{r+1,12}=e.c;
        data{r+1,13}=e.buyO;data{r+1,14}=e.buyH;data{r+1,15}=e.buyF;
    end
    T=cell2table(data(2:end,:),'VariableNames',data(1,:));
    writetable(T,'result.xlsx','Sheet','CP_Expected');
    fprintf('  Rows: %d (Day 0 .. Day %d)\n',nRows-1,nRows-2);

    sm.problem='Task 3 (CP Expected-Value)';
    sm.method='Constraint Programming + Expected Consumption + Safety Margins';
    sm.path=strjoin(sk,' -> ');
    sm.Z=Z;sm.M=M;
    wpKeys=keys(wd);wdStruct=struct();
    for i=1:length(wpKeys),nm=wpKeys{i};wdStruct.(nm)=wd(nm);end
    sm.work_days=wdStruct;
    fid=fopen('result_q3_cp_expected.json','w');
    fprintf(fid,'%s\n',jsonencode(sm,'PrettyPrint',true));
    fclose(fid);
end
