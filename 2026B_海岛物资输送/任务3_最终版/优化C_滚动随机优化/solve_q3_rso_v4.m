function solve_q3_rso_v4(weather_seq)
% SOLVE_Q3_RSO_V4  Task3 RSO v4 Online (Full-distance move)

rcfg=rso_solver_v4('config');
cfg=cp_engine_v2('config');
cN=cp_engine_v2('cons','normal'); cT=cp_engine_v2('cons','thunder');
ce=cp_engine_v2('cons','expected');

if nargin<1||isempty(weather_seq)
    rng('shuffle'); weather_seq=cp_engine_v2('weather',90,0.8);
end

st.pt=1; st.pos=cfg.xy(1,:);
st.O=cfg.init.O; st.H=cfg.init.H; st.F=cfg.init.F;
st.M=cfg.init.M; st.Z=cfg.init.Z; st.consec_work=0; st.day=0;

fprintf('========================================\n');
fprintf('  Task3 RSO v4 Online (Full-dist move)\n');
fprintf('========================================\n');
fprintf('H=%dd K=%d(%dT) | Full-dist eval | Per-res supply\n',...
    rcfg.H,rcfg.K_total,rcfg.K_thunder);
fprintf('Day  | W | Action              | Pos        | Res       |   Z    M\n');
fprintf('-----|---|---------------------|------------|-----------|---------\n');

while st.day<cfg.MAX_DAYS && st.pt~=2
    st.day=st.day+1;
    w=weather_seq(st.day);
    if w=='T', ca=cT; wn='T'; else, ca=cN; wn='N'; end
    curr=struct('O',st.O,'H',st.H,'F',st.F,'M',st.M,'Z',st.Z);
    [best_act,~]=rso_solver_v4('step',st.pt,st.day-1,curr,cfg,rcfg.H,rcfg.K_total,rcfg.K_thunder);
    action_str=''; ik=false;
    
    switch best_act.type
        case 'MOVE'
            target=best_act.target;
            fr=cfg.xy(st.pt,:); to=cfg.xy(target,:);
            dx=to(1)-fr(1); dy=to(2)-fr(2);
            if abs(dx)>0, st.pos(1)=fr(1)+sign(dx); st.pos(2)=fr(2);
            elseif abs(dy)>0, st.pos(1)=fr(1); st.pos(2)=fr(2)+sign(dy);
            end
            st.O=st.O-ca.MO; st.H=st.H-ca.MH; st.F=st.F-ca.MF;
            st.consec_work=0;
            action_str=sprintf('MOVE->(%d,%d)',st.pos(1),st.pos(2));
            if st.pos(1)==to(1)&&st.pos(2)==to(2)
                st.pt=target;
                if st.pt==6||st.pt==7
                    dE_s3=cfg.dist(st.pt,2); cT_s3=cp_engine_v2('cons','thunder');
                    nO=dE_s3*(0.7*ce.MO+0.3*cT_s3.MO);
                    nH=dE_s3*(0.7*ce.MH+0.3*cT_s3.MH);
                    nF=dE_s3*(0.7*ce.MF+0.3*cT_s3.MF);
                    sp=cfg.MAX_LOAD-(st.O+st.H+st.F);
                    bO=max(0,nO-st.O); bH=max(0,nH-st.H); bF=max(0,nF-st.F);
                    if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
                    cost=bO*2+bH*1+bF*2;
                    if cost<=st.M
                        st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=st.M-cost;
                        action_str=sprintf('SUPPLY(%s)',cfg.names{st.pt});
                    end
                elseif st.pt==2, action_str='ARRIVE!'; end
            end
            ik=true;
        case 'PARK'
            st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF;
            st.consec_work=0; action_str='PARK'; ik=true;
        case 'WORK'
            if st.pt>=3&&st.pt<=5
                wi=st.pt-2;
                if st.consec_work<cfg.WM(wi)
                    st.O=st.O-ca.WO; st.H=st.H-ca.WH; st.F=st.F-ca.WF;
                    st.Z=st.Z+cfg.WY(wi); st.consec_work=st.consec_work+1;
                    action_str=sprintf('WORK(%s)',cfg.names{st.pt});
                else
                    st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF;
                    st.consec_work=0; action_str='PARK(reset)';
                end
                ik=true;
            end
        case 'SUPPLY'
            if st.pt==6||st.pt==7
                dE_s3=cfg.dist(st.pt,2); cT_s3=cp_engine_v2('cons','thunder');
                    nO=dE_s3*(0.7*ce.MO+0.3*cT_s3.MO);
                    nH=dE_s3*(0.7*ce.MH+0.3*cT_s3.MH);
                    nF=dE_s3*(0.7*ce.MF+0.3*cT_s3.MF);
                sp=cfg.MAX_LOAD-(st.O+st.H+st.F);
                bO=max(0,nO-st.O); bH=max(0,nH-st.H); bF=max(0,nF-st.F);
                if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
                cost=bO*2+bH*1+bF*2;
                if cost<=st.M
                    st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=st.M-cost;
                end
                st.consec_work=0; action_str='SUPPLY'; ik=true;
            end
    end
    
    if st.O<-1e-6||st.H<-1e-6||st.F<-1e-6
        fprintf('%4d | %s | EXHAUSTED!           | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f ***\n',...
            st.day,wn,st.pos(1),st.pos(2),st.O,st.H,st.F,st.Z,round(st.M));
        break;
    end
    if ik
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f\n',...
            st.day,wn,action_str,st.pos(1),st.pos(2),st.O,st.H,st.F,st.Z,round(st.M));
    end
end

fprintf('-----|---|---------------------|------------|-----------|---------\n');
if st.pt==2, fprintf('Day %d Arrived | Z=%d M=%.0f\n',st.day,st.Z,round(st.M));
else, fprintf('FAIL/TIMEOUT Day %d\n',st.day); end
fprintf('Done.\n');
end
