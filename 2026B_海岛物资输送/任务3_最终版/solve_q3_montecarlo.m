function solve_q3_montecarlo(N)
% =========================================================================
%  solve_q3_montecarlo.m — 任务3 蒙特卡洛验证 (最终版)
%  共享 cp_engine, 成功率: 60% (N=50)
% =========================================================================

if nargin<1||isempty(N), N=100; end
cfg=cp_engine('config');
ce=cp_engine('cons','expected');

fprintf('========================================\n');
fprintf('  任务3 MC验证 (N=%d) 最终版\n',N);
fprintf('========================================\n');
fprintf('SAFETY×1.05 | 三资源生存 | 场景CP | 预算自适应\n\n');

Zr=NaN(1,N); Mr=NaN(1,N); ok=false(1,N); dy=NaN(1,N); fr=cell(1,N); sc=zeros(1,N);
ri=max(1,floor(N/20)); tic;

for sim=1:N
    ws=cp_engine('weather',90,0.8);
    [Zf,Mf,arr,days,reason,surv]=run_silent(ws);
    Zr(sim)=Zf; Mr(sim)=Mf; ok(sim)=arr; dy(sim)=days; fr{sim}=reason; sc(sim)=surv;
    if mod(sim,ri)==0, fprintf(' %d/%d (%.0f%%) | %.1fs | %.1f%%\n',sim,N,100*sim/N,toc,100*sum(ok(1:sim))/sim); end
end
tt=toc; ns=sum(ok);
fprintf('\nDone. %.1fs\n\n',tt);

fprintf('========================================\n  结果\n========================================\n');
fprintf('成功率: %d/%d (%.1f%%)\n',ns,N,100*ns/N);
fprintf('平均生存触发: %.1f/run\n',mean(sc));
if ns>0
    Zs=Zr(ok); Ms=Mr(ok); Ds=dy(ok);
    fprintf('--- 成功 (n=%d) ---\n',ns);
    fprintf('%-12s %8s %8s %8s %8s\n','','均值','std','min','max');
    fprintf('%-12s %8.1f %8.1f %8d %8d\n','Z',mean(Zs),std(Zs),min(Zs),max(Zs));
    fprintf('%-12s %8.2f %8.2f %8.2f %8.2f\n','M',mean(Ms),std(Ms),min(Ms),max(Ms));
    fprintf('%-12s %8.1f %8.1f %8d %8d\n','Days',mean(Ds),std(Ds),min(Ds),max(Ds));
end
if N-ns>0
    fprintf('\n--- 失败 ---\n'); [u,~,ic]=unique(fr(~ok)); cnt=accumarray(ic,1);
    for i=1:length(u), fprintf(' %s: %d\n',u{i},cnt(i)); end
end
fprintf('\nDone.\n');
end

function [Zf,Mf,arr,days,reason,surv]=run_silent(wseq)
    cfg=cp_engine('config'); ce=cp_engine('cons','expected');
    cN=cp_engine('cons','normal'); cT=cp_engine('cons','thunder');
    st.pt=1; st.pos=cfg.xy(1,:); st.O=cfg.init.O; st.H=cfg.init.H; st.F=cfg.init.F;
    st.M=cfg.init.M; st.Z=cfg.init.Z; st.consec=0; st.day=0;
    pp=[]; pprk=[]; pwrk=[]; pl=1; sil=0; pil=0; wpi=0; wdd=0;
    reason=''; surv=0;

    while st.day<cfg.MAX_DAYS&&st.pt~=2&&isempty(reason)
        st.day=st.day+1; w=wseq(st.day);
        if w=='T', ca=cT; else, ca=cN; end
        dS1=cfg.dist(st.pt,6); dS2=cfg.dist(st.pt,7); nsd=min(dS1,dS2);
        sv=(st.pt~=6)&&(st.pt~=7)&&((st.O<1.0*nsd*ce.MO)||(st.H<1.0*nsd*ce.MH)||(st.F<1.0*nsd*ce.MF));
        if sv, surv=surv+1; end
        an=(sil==0&&pil==0); naw=~(st.pt>=3&&st.pt<=5);
        wc=(st.day>1&&w~=wseq(max(1,st.day-1))); per=(mod(st.day,3)==1&&st.day>1);
        nr=isempty(pp)||((wc||per)&&an&&naw)||(sv&&an&&naw&&~isempty(pp)&&any(pp(2:end-1)>=3&pp(2:end-1)<=5));

        if nr
            el=st.day-1; is=struct('O',st.O,'H',st.H,'F',st.F,'M',st.M,'Z',st.Z);
            if sv&&st.pt~=6&&st.pt~=7
                [pp,pprk,pwrk,fok]=cp_engine('plan',st.pt,el,cN,cfg,true,is);
            else
                [pp,pprk,pwrk,fok]=cp_engine('plan_scenario',st.pt,el,cfg,false,is);
            end
            if ~fok, [pp,pprk,pwrk,fok]=cp_engine('plan',st.pt,el,ce,cfg,false,is); end
            if ~fok, [pp,pprk,pwrk,fok]=cp_engine('plan',st.pt,el,cN,cfg,sv,is);
                if ~fok, reason='NO PLAN'; break; end
            end
            pl=1; sil=0; pil=0; wpi=0; wdd=0;
        end
        if length(pp)<2, reason='EMPTY'; break; end
        np=pp(pl+1);
        iwp=(st.pt>=3&&st.pt<=5); wpm=true;
        if iwp&&wpi<length(pwrk)
            wal=[]; for ii=2:length(pp), if pp(ii)>=3&&pp(ii)<=5, wal(end+1)=ii; end; end
            if wpi+1<=length(wal), wpm=(pp(wal(wpi+1))==st.pt); end
        end
        nw=iwp&&wpm&&wpi<length(pwrk)&&wdd<pwrk(wpi+1);
        if nw
            wt=st.pt-2; wm=cfg.WM(wt); yl=cfg.WY(wt);
            if st.consec<wm
                st.O=st.O-ca.WO; st.H=st.H-ca.WH; st.F=st.F-ca.WF; st.Z=st.Z+yl;
                st.consec=st.consec+1; wdd=wdd+1;
                if wdd>=pwrk(wpi+1), wpi=wpi+1; wdd=0; end
            else, st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF; st.consec=0; end
        elseif pl<=length(pprk)&&pil<pprk(pl)
            st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF; st.consec=0; pil=pil+1;
        elseif sil<cfg.dist(st.pt,np)
            st.O=st.O-ca.MO; st.H=st.H-ca.MH; st.F=st.F-ca.MF; st.consec=0; sil=sil+1;
            if sil>=cfg.dist(st.pt,np)
                st.pt=np; st.pos=cfg.xy(np,:); sil=0; pil=0;
                if st.pt>=3&&st.pt<=5
                    wc=0; for i=2:pl+1, if pp(i)>=3&&pp(i)<=5, wc=wc+1; end; end
                    wpi=wc-1; wdd=0;
                end
                if st.pt==6||st.pt==7
                    [nO,nH,nF]=cp_engine('supply_needs',pp,pprk,pwrk,pl,ce,cfg);
                    sp=cfg.MAX_LOAD-(st.O+st.H+st.F);
                    bO=max(0,nO-st.O); bH=max(0,nH-st.H); bF=max(0,nF-st.F);
                    if bO+bH+bF>sp+1e-6, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
                    cost=bO*ca.pO+bH*ca.pH+bF*ca.pF;
                    if cost<=st.M+1e-6
                        st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=st.M-cost;
                    else
                        sclM=st.M/cost; bO=bO*sclM; bH=bH*sclM; bF=bF*sclM;
                        st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=0;
                    end
                end
                pl=pl+1;
            end
        end
        if st.O<-1e-6||st.H<-1e-6||st.F<-1e-6, reason='RESOURCE'; break; end
        if st.O+st.H+st.F>cfg.MAX_LOAD+1e-6, reason='OVERLOAD'; break; end
    end
    if isempty(reason)&&st.pt==2, arr=true; days=st.day;
    elseif isempty(reason)&&st.day>=cfg.MAX_DAYS, reason='TIMEOUT'; arr=false; days=cfg.MAX_DAYS;
    else, arr=false; days=st.day; end
    Zf=st.Z; Mf=st.M; if ~arr, Zf=0; Mf=0; end
end
