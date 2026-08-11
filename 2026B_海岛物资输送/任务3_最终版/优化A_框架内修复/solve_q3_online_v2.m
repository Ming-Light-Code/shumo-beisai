function solve_q3_online_v2(weather_seq)
% SOLVE_Q3_ONLINE_V2  Task3 Online Decision v2 (Weather-Adaptive)
% Improvements over v1:
%   1. PARK during thunder instead of moving (saves O=5/day)
%   2. Thunder-aware survival check (uses thunder consumption)
%   3. Event-driven re-planning (consec thunder, resource threshold)
%   4. Safe-mode fallback: skip work, head directly to E when resources critical
%   5. Thunder-safe supply needs blending

cfg = cp_engine_v2('config');
cons_exp = cp_engine_v2('cons', 'expected');
cons_N   = cp_engine_v2('cons', 'normal');
cons_T   = cp_engine_v2('cons', 'thunder');

if nargin < 1 || isempty(weather_seq)
    rng('shuffle'); weather_seq = cp_engine_v2('weather', 90, 0.8);
end

st.pt=1; st.pos=cfg.xy(1,:);
st.O=cfg.init.O; st.H=cfg.init.H; st.F=cfg.init.F;
st.M=cfg.init.M; st.Z=cfg.init.Z; st.consec=0; st.day=0;
pp=[]; pprk=[]; pwrk=[]; pl=1; sil=0; pil=0; wpi=0; wdd=0;
rc=0; sc=0; pausethunder=0; safemode=false;

fprintf('========================================\n');
fprintf('  Task3 Online v2 (Weather-Adaptive)\n');
fprintf('========================================\n');
fprintf('Features: Thunder-Park | Event Replan | Safe-Mode | Supply-Blend\n');
fprintf('Day  | W | Action              | Pos        |  O   H   F Load|   Z    M\n');
fprintf('-----|---|---------------------|------------|----------------|---------\n');

while st.day<cfg.MAX_DAYS && st.pt~=2
    st.day=st.day+1; w=weather_seq(st.day);
    if w=='T', ca=cons_T; wn='T'; else, ca=cons_N; wn='N'; end

    % ==== Thunder-aware survival check ====
    dS1=cfg.dist(st.pt,6); dS2=cfg.dist(st.pt,7); nsd=min(dS1,dS2);
    % Use THUNDER consumption when weather is thunder
    if w=='T'
        surv=(st.pt~=6)&&(st.pt~=7)&&...
            ((st.O<1.0*nsd*cons_T.MO)||(st.H<1.0*nsd*cons_T.MH)||(st.F<1.0*nsd*cons_T.MF));
    else
        surv=(st.pt~=6)&&(st.pt~=7)&&...
            ((st.O<1.0*nsd*cons_N.MO)||(st.H<1.0*nsd*cons_N.MH)||(st.F<1.0*nsd*cons_N.MF));
    end

    % ==== Safe-mode check: can we reach E with current resources? ====
    dE=cfg.dist(st.pt,2);
    if ~safemode && (st.O<dE*cons_T.MO || st.H<dE*cons_T.MH || st.F<dE*cons_T.MF)
        safemode=true;
    end

    % ==== Event-driven re-planning triggers ====
    an=(sil==0&&pil==0); naw=~(st.pt>=3&&st.pt<=5);
    wc=(st.day>1&&w~=weather_seq(max(1,st.day-1)));
    consec_thunder=0;
    for ck=max(1,st.day-2):st.day
        if weather_seq(ck)=='T', consec_thunder=consec_thunder+1; end
    end
    thunder_alert=(consec_thunder>=3);
    nr=isempty(pp)||(wc&&an&&naw)||thunder_alert||...
       (surv&&an&&naw&&~isempty(pp)&&any(pp(2:end-1)>=3&pp(2:end-1)<=5));

    if nr
        rc=rc+1; el=st.day-1;
        is=struct('O',st.O,'H',st.H,'F',st.F,'M',st.M,'Z',st.Z);
        if safemode
            pp=cp_engine_v2('path_to_E',st.pt,el,cfg);
            pprk=[]; pwrk=[]; fok=~isempty(pp);
        elseif surv&&st.pt~=6&&st.pt~=7
            sc=sc+1;
            [pp,pprk,pwrk,fok]=cp_engine_v2('plan',st.pt,el,cons_T,cfg,true,is);
        else
            [pp,pprk,pwrk,fok]=cp_engine_v2('plan_scenario',st.pt,el,cfg,false,is);
        end
        if ~fok
            [pp,pprk,pwrk,fok]=cp_engine_v2('plan',st.pt,el,cons_exp,cfg,false,is);
        end
        if ~fok
            [pp,pprk,pwrk,fok]=cp_engine_v2('plan',st.pt,el,cons_N,cfg,surv,is);
            if ~fok
                % Last resort: head directly to E
                pp=cp_engine_v2('path_to_E',st.pt,el,cfg);
                pprk=[]; pwrk=[]; fok=~isempty(pp);
                safemode=true;
                if ~fok, fprintf('%4d | %s | NO PLAN            |            |                | %5d %6.0f\n',st.day,wn,st.Z,round(st.M)); break; end
            end
        end
        pl=1; sil=0; pil=0; wpi=0; wdd=0;
    end

    if length(pp)<2, break; end
    np=pp(pl+1);

    % ==== Weather-adaptive action decision ====
    iwp=(st.pt>=3&&st.pt<=5); wpm=true;
    if iwp&&wpi<length(pwrk)
        wal=[]; for ii=2:length(pp), if pp(ii)>=3&&pp(ii)<=5, wal(end+1)=ii; end; end
        if wpi+1<=length(wal), wpm=(pp(wal(wpi+1))==st.pt); end
    end
    nw=iwp&&wpm&&wpi<length(pwrk)&&wdd<pwrk(wpi+1);

    % KEY IMPROVEMENT: Park during thunder instead of moving
    thunder_park = false;
    if w=='T' && ~nw && ~(st.pt==6||st.pt==7) && sil<cfg.dist(st.pt,np) && pl<=length(pprk)&&pil>=pprk(pl)
        thunder_park = true;
        pausethunder = pausethunder + 1;
    end

    act=''; det=''; ik=false;
    if nw
        wt=st.pt-2; wm=cfg.WM(wt); yl=cfg.WY(wt);
        if st.consec<wm
            act=sprintf('work(%s)',cfg.names{st.pt});
            st.O=st.O-ca.WO; st.H=st.H-ca.WH; st.F=st.F-ca.WF; st.Z=st.Z+yl;
            st.consec=st.consec+1; wdd=wdd+1;
            if wdd>=pwrk(wpi+1), wpi=wpi+1; wdd=0; end
            ik=true;
        else
            act='park(reset)';
            st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF; st.consec=0; ik=true;
        end
    elseif thunder_park
        act='PARK(thunder)';
        st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF; st.consec=0; ik=true;
    elseif pl<=length(pprk)&&pil<pprk(pl)
        act='park(at sea)';
        st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF; st.consec=0; pil=pil+1; ik=true;
    elseif sil<cfg.dist(st.pt,np)
        st.O=st.O-ca.MO; st.H=st.H-ca.MH; st.F=st.F-ca.MF; st.consec=0; sil=sil+1;
        fr=cfg.xy(st.pt,:); to=cfg.xy(np,:); dx=to(1)-fr(1); dy=to(2)-fr(2);
        sx=abs(dx); sy=abs(dy);
        if sil<=sx, st.pos(1)=fr(1)+sign(dx)*sil; st.pos(2)=fr(2);
        else, st.pos(1)=to(1); st.pos(2)=fr(2)+sign(dy)*(sil-sx); end
        act=sprintf('move->(%d,%d)',st.pos(1),st.pos(2));
        if sil>=cfg.dist(st.pt,np)
            st.pt=np; st.pos=cfg.xy(np,:); sil=0; pil=0;
            if st.pt>=3&&st.pt<=5
                wc=0; for i=2:pl+1, if pp(i)>=3&&pp(i)<=5, wc=wc+1; end; end
                wpi=wc-1; wdd=0;
            end
            if st.pt==6||st.pt==7
                % Use thunder-safe supply needs
                [nO,nH,nF]=cp_engine_v2('supply_needs_safe',pp,pprk,pwrk,pl,cons_exp,cfg);
                sp=cfg.MAX_LOAD-(st.O+st.H+st.F);
                bO=max(0,nO-st.O); bH=max(0,nH-st.H); bF=max(0,nF-st.F);
                if bO+bH+bF>sp+1e-6, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
                cost=bO*ca.pO+bH*ca.pH+bF*ca.pF;
                if cost<=st.M+1e-6
                    st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=st.M-cost;
                    act=sprintf('SUPPLY(%s)',cfg.names{st.pt}); det=sprintf('+O%.0f H%.0f F%.0f',bO,bH,bF);
                else
                    sclM=st.M/cost; bO=bO*sclM; bH=bH*sclM; bF=bF*sclM;
                    st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=0;
                    act=sprintf('SUPPLY(%s)-lim',cfg.names{st.pt}); det=sprintf('+O%.0f H%.0f F%.0f(M=0)',bO,bH,bF);
                end
                ik=true;
            elseif st.pt==2, act='ARRIVE!'; ik=true; end
            pl=pl+1;
        end
    end

    if st.O<-1e-6||st.H<-1e-6||st.F<-1e-6
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f%5.0f|%5d%6.0f ***\n',st.day,wn,'EXHAUSTED!',st.pos(1),st.pos(2),st.O,st.H,st.F,st.O+st.H+st.F,st.Z,round(st.M));
        break;
    end

    if ik||nr
        tg='';
        if nr, tg=sprintf('[R#%d]',rc); end
        if surv, tg=[tg '[S]']; end
        if safemode, tg=[tg '[SAFE]']; end
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f%5.0f|%5d%6.0f%s',st.day,wn,act,st.pos(1),st.pos(2),st.O,st.H,st.F,st.O+st.H+st.F,st.Z,round(st.M),tg);
        if ~isempty(det), fprintf(' %s',det); end; fprintf('\n');
    end
end

fprintf('-----|---|---------------------|------------|----------------|---------\n');
fprintf('\n===== Result =====\n');
if st.pt==2
    fprintf('Day %d Arrived at E | Z=%d M=%.0f | Replans:%d Survival:%d ThunderParks:%d SafeMode:%d\n',...
        st.day,st.Z,round(st.M),rc,sc,pausethunder,safemode);
elseif st.day>=cfg.MAX_DAYS
    fprintf('TIMEOUT | Z=%d M=%.0f\n',st.Z,round(st.M));
else
    fprintf('FAILED(Day %d) | Z=%d M=%.0f\n',st.day,st.Z,round(st.M));
end
fprintf('Done.\n');
end
