function varargout = cp_engine(action, varargin)
% =========================================================================
%  cp_engine.m — 任务3 公共CP引擎 (最终版)
%  提供: 配置 | CP搜索 | 路径模拟 | 场景规划 | 补给计算 | 天气生成
% =========================================================================

switch action
    case 'config',       varargout{1} = get_config();
    case 'cons',         varargout{1} = get_cons(varargin{:});
    case 'plan',         [varargout{1},varargout{2},varargout{3},varargout{4}] = plan_from_state(varargin{:});
    case 'plan_scenario',[varargout{1},varargout{2},varargout{3},varargout{4}] = plan_scenario(varargin{:});
    case 'simulate',     [varargout{1},varargout{2},varargout{3}] = simulate_path(varargin{:});
    case 'supply_needs', [varargout{1},varargout{2},varargout{3}] = get_supply_needs(varargin{:});
    case 'weather',      varargout{1} = gen_weather(varargin{:});
    otherwise, error('cp_engine: unknown "%s"', action);
end
end

% ===== 任务3配置 =====
function cfg = get_config()
    cfg.MAX_DAYS = 90; cfg.MAX_LOAD = 400;
    cfg.xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16]; % B,E,W1,W2,W3,S1,S2
    cfg.names = {'B','E','W1','W2','W3','S1','S2'};
    cfg.WY = [20 15 28]; cfg.WM = [4 5 3];
    cfg.init = struct('O',100,'H',150,'F',100,'M',750,'Z',200);
    cfg.inter = [3 4 5 6 7];
    n = size(cfg.xy,1); cfg.dist = zeros(n);
    for i=1:n, for j=1:n, cfg.dist(i,j)=abs(cfg.xy(i,1)-cfg.xy(j,1))+abs(cfg.xy(i,2)-cfg.xy(j,2)); end; end
end

% ===== 消耗参数 (SAFETY=1.05 轻度保守) =====
function cons = get_cons(mode)
    SO=1.05; SH=1.05; SF=1.05;
    switch mode
        case 'expected'
            cons.MO=3.2*SO; cons.MH=3.2*SH; cons.MF=2.2*SF;
            cons.PO=1.4*SO; cons.PH=1.4*SH; cons.PF=1.2*SF;
            cons.WO=5.6*SO; cons.WH=4.4*SH; cons.WF=3.6*SF;
        case 'expected_pes'
            P=1.15; cons.MO=3.2*SO*P; cons.MH=3.2*SH*P; cons.MF=2.2*SF*P;
            cons.PO=1.4*SO*P; cons.PH=1.4*SH*P; cons.PF=1.2*SF*P;
            cons.WO=5.6*SO*P; cons.WH=4.4*SH*P; cons.WF=3.6*SF*P;
        case 'normal'
            cons.MO=2; cons.MH=3; cons.MF=2; cons.PO=1; cons.PH=1; cons.PF=1;
            cons.WO=5; cons.WH=4; cons.WF=3;
        case 'thunder'
            cons.MO=8; cons.MH=4; cons.MF=3; cons.PO=3; cons.PH=3; cons.PF=2;
            cons.WO=8; cons.WH=6; cons.WF=6;
    end
    cons.pO=2; cons.pH=1; cons.pF=2;
end

% ===== CP规划入口 =====
function [bp, bprk, bwrk, ok] = plan_from_state(cur_pt, elapsed, cons, cfg, survival, init_s)
    if nargin<5, survival=false; end
    if nargin<6, init_s=cfg.init; end
    bZ=-inf; bM=-inf; bp=[cur_pt,2]; bwrk=[]; bprk=[]; nodes=0;
    eff=cfg.MAX_DAYS-elapsed;
    [bZ,bM,bp,bwrk,bprk,nodes]=cp_search([cur_pt],0,[],[],bZ,bM,bp,bwrk,bprk,cfg,cons,nodes,eff,init_s);
    ok=(bZ>-inf);
end

% ===== 场景CP (三天气假设) =====
function [bp, bprk, bwrk, ok] = plan_scenario(cur_pt, elapsed, cfg, survival, init_s)
    if nargin<4, survival=false; end
    if nargin<5, init_s=cfg.init; end
    cl={get_cons('normal'),get_cons('expected'),get_cons('expected_pes')};
    bZ=-inf; bM=-inf; bp=[cur_pt,2]; bwrk=[]; bprk=[];
    for sc=1:3
        [pth,prk,wrk,fok]=plan_from_state(cur_pt,elapsed,cl{sc},cfg,survival,init_s);
        if fok
            m=length(pth)-2; tt=0;
            for k=1:m+1, tt=tt+cfg.dist(pth(k),pth(k+1)); end
            wa=[]; ww=[];
            for i=2:length(pth), if pth(i)>=3&&pth(i)<=5, wa(end+1)=i; ww(end+1)=pth(i)-2; end; end
            [fe,Z,M]=simulate_path(pth,m,cfg.dist,wa,ww,tt,wrk,prk,get_cons('expected'),cfg,init_s);
            if fe&&(Z>bZ||(Z==bZ&&M>bM)), bZ=Z; bM=M; bp=pth; bwrk=wrk; bprk=prk; end
        end
    end
    ok=(bZ>-inf);
end

% ===== CP递归搜索 =====
function [bZ,bM,bP,bWD,bPS,nds]=cp_search(path,tsf,wa,ww,bZ,bM,bP,bWD,bPS,cfg,cons,nds,eff,init_s)
    nds=nds+1; lp=path(end);
    if lp~=2
        dE=cfg.dist(lp,2); rem=eff-tsf;
        if rem<dE, return; end
        if init_s.Z+mwp(3,rem-dE)*28<=bZ&&bZ>-inf, return; end
    end
    dE=cfg.dist(lp,2);
    if tsf+dE<=eff
        fp=[path,2]; m=length(fp)-2; tt=0;
        for k=1:m+1, tt=tt+cfg.dist(fp(k),fp(k+1)); end
        rem_d=eff-tt; nw=length(wa);
        if nw==0
            [ok,Z,M]=simulate_path(fp,m,cfg.dist,[],[],tt,[],[],cons,cfg,init_s);
            if ok&&(Z>bZ||(Z==bZ&&M>bM)), bZ=Z; bM=M; bP=fp; bWD=[]; bPS=[]; end
        else
            mx=zeros(1,nw); for j=1:nw, mx(j)=mwp(cfg.WM(ww(j)),rem_d); end
            sz=mx+1; nc=prod(sz);
            for ci=1:nc
                wd=zeros(1,nw); t2=ci-1;
                for j=nw:-1:1, wd(j)=mod(t2,sz(j)); t2=floor(t2/sz(j)); end
                ts=0; for j=1:nw, if wd(j)>0, ts=ts+wd(j)+max(0,ceil(wd(j)/cfg.WM(ww(j)))-1); end; end
                if tt+ts>eff, continue; end
                [ok,Z,M]=simulate_path(fp,m,cfg.dist,wa,ww,tt,wd,[],cons,cfg,init_s);
                if ok&&(Z>bZ||(Z==bZ&&M>bM)), bZ=Z; bM=M; bP=fp; bWD=wd; bPS=[]; end
            end
        end
    end
    for ni=1:5
        np=cfg.inter(ni); if np==lp, continue; end
        d=cfg.dist(lp,np); if tsf+d>eff, continue; end
        if tsf+d+cfg.dist(np,2)>eff, continue; end
        np2=[path,np]; nt=tsf+d; nwa=wa; nww=ww;
        if np>=3&&np<=5, nwa(end+1)=length(np2); nww(end+1)=np-2; end
        [bZ,bM,bP,bWD,bPS,nds]=cp_search(np2,nt,nwa,nww,bZ,bM,bP,bWD,bPS,cfg,cons,nds,eff,init_s);
    end
end

% ===== 路径模拟 (含预防性超购) =====
function [ok,Zf,Mf]=simulate_path(pid,m,dist_all,wa,ww,tt,wdays,park_seg,cons,cfg,init_s)
    if isempty(wdays), wdays=[]; end
    if isempty(park_seg), park_seg=zeros(1,m+1); end
    if nargin<11, cfg=get_config(); end
    if nargin<12, init_s=cfg.init; end
    Ta=tt+sum(wdays)+sum(park_seg)+100;
    cO=zeros(1,Ta); cH=zeros(1,Ta); cF=zeros(1,Ta); zG=zeros(1,Ta); iS=false(1,Ta);
    day=0;
    for k=1:m+1
        d=dist_all(pid(k),pid(k+1));
        for pd=1:park_seg(k), day=day+1; cO(day)=cons.PO; cH(day)=cons.PH; cF(day)=cons.PF; end
        for dd=1:d
            day=day+1; cO(day)=cons.MO; cH(day)=cons.MH; cF(day)=cons.MF;
            if dd==d, tp=pid(k+1); if tp==6||tp==7, iS(day)=true; end; end
        end
        if ~isempty(wa)
            wk=find(wa==k+1,1);
            if ~isempty(wk)&&~isempty(wdays)&&wk<=length(wdays)&&wdays(wk)>0
                mc=cfg.WM(ww(wk)); yld=cfg.WY(ww(wk)); rv=wdays(wk);
                while rv>0
                    ch=min(rv,mc);
                    for w=1:ch, day=day+1; cO(day)=cons.WO; cH(day)=cons.WH; cF(day)=cons.WF; zG(day)=yld; end
                    rv=rv-ch; if rv>0, day=day+1; cO(day)=cons.PO; cH(day)=cons.PH; cF(day)=cons.PF; end
                end
            end
        end
    end
    Ta=day; O=init_s.O; H=init_s.H; F=init_s.F; M=init_s.M; Zf=init_s.Z;
    lsi=find(iS,1,'last'); if isempty(lsi), lsi=0; end
    for t=1:Ta
        O=O-cO(t); H=H-cH(t); F=F-cF(t); Zf=Zf+zG(t);
        if O<-1e-6||H<-1e-6||F<-1e-6, ok=false; Mf=0; return; end
        if O+H+F>cfg.MAX_LOAD+1e-6, ok=false; Mf=0; return; end
        if iS(t)
            ns=Ta+1; for tt2=t+1:Ta, if iS(tt2), ns=tt2; break; end; end
            nO=0; nH=0; nF=0;
            for tt2=t+1:ns, if tt2>Ta, break; end; nO=nO+cO(tt2); nH=nH+cH(tt2); nF=nF+cF(tt2); end
            sp=cfg.MAX_LOAD-(O+H+F);
            bO=max(0,nO-O); bH=max(0,nH-H); bF=max(0,nF-F);
            if t~=lsi
                sab=sp-(bO+bH+bF);
                if sab>1e-6
                    bt=0.10*sab; tot=cons.MO+cons.MH+cons.MF;
                    bO=bO+bt*(cons.MO/tot); bH=bH+bt*(cons.MH/tot); bF=bF+bt*(cons.MF/tot);
                end
            end
            if bO+bH+bF>sp+1e-6, ok=false; Mf=0; return; end
            if ns>Ta&&(O+bO<nO-1e-6||H+bH<nH-1e-6||F+bF<nF-1e-6), ok=false; Mf=0; return; end
            cost=bO*cons.pO+bH*cons.pH+bF*cons.pF;
            if cost>M+1e-6, ok=false; Mf=0; return; end
            O=O+bO; H=H+bH; F=F+bF; M=M-cost;
        end
    end
    ok=true; Mf=M;
end

% ===== 补给需求计算 (含移动+作业+停泊) =====
function [nO,nH,nF]=get_supply_needs(plan_path,plan_parks,plan_works,plan_leg,cons,cfg)
    rt=0; rp=0; wO=0; wH=0; wF=0; wc=0;
    for k=plan_leg:length(plan_path)-2
        rt=rt+cfg.dist(plan_path(k+1),plan_path(k+2));
        if k+1<=length(plan_parks), rp=rp+plan_parks(k+1); end
        if plan_path(k+1)>=3&&plan_path(k+1)<=5
            wc=wc+1;
            if wc<=length(plan_works)
                wd=plan_works(wc); wi=plan_path(k+1)-2;
                np=max(0,ceil(wd/cfg.WM(wi))-1);
                wO=wO+wd*cons.WO+np*cons.PO; wH=wH+wd*cons.WH+np*cons.PH; wF=wF+wd*cons.WF+np*cons.PF;
            end
        end
    end
    nO=rt*cons.MO+rp*cons.PO+wO; nH=rt*cons.MH+rp*cons.PH+wH; nF=rt*cons.MF+rp*cons.PF+wF;
end

function mw=mwp(mc,rem)
    best=0;
    for k=1:rem+1
        st=k*mc+(k-1); if st>rem, break; end
        best=k*mc; sl=rem-st;
        if sl>=1, best=max(best,k*mc+min(mc,sl-1)); end
    end
    mw=max(best,min(mc,rem));
end

function ws=gen_weather(n,pn)
    ws=repmat('N',1,n); for i=1:n, if rand()>pn, ws(i)='T'; end; end
end
