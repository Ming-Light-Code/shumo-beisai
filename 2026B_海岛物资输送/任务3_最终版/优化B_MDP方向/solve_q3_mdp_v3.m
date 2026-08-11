function solve_q3_mdp_v3(weather_seq)
% SOLVE_Q3_MDP_V3  Task3 MDP v3 Online (Corrected move model)

cfg=cp_engine_v2('config'); cfg_mdp=mdp_solver_v4('config');
cN=cp_engine_v2('cons','normal'); cT=cp_engine_v2('cons','thunder');
ce=cp_engine_v2('cons','expected');
V=mdp_solver_v4('solve');

if nargin<1||isempty(weather_seq)
    rng('shuffle'); weather_seq=cp_engine_v2('weather',90,0.8);
end

st.pt=1; st.pos=cfg.xy(1,:);
st.O=cfg.init.O; st.H=cfg.init.H; st.F=cfg.init.F;
st.M=cfg.init.M; st.Z=cfg.init.Z; st.consec_work=0; st.day=0;
supplied_today=false;

fprintf('========================================\n');
fprintf('  Task3 MDP v3 Online (Corrected)\n');
fprintf('========================================\n');
fprintf('Full-dist move | Consistent quant | Dist-aware init\n');
fprintf('Day  | W | Action              | Pos        | Res       |   Z    M\n');
fprintf('-----|---|---------------------|------------|-----------|---------\n');

while st.day<cfg.MAX_DAYS && st.pt~=2
    st.day=st.day+1; supplied_today=false;
    w=weather_seq(st.day);
    if w=='T', ca=cT; w_idx=2; wn='T';
    else, ca=cN; w_idx=1; wn='N';
    end
    
    r=get_res_level(st.O,st.H,st.F,cfg_mdp);
    if st.day<=30, d=1; elseif st.day<=60, d=2; else, d=3; end
    c=min(5,st.consec_work+1);
    
    [act,target,~]=mdp_solver_v4('policy',st.pt,w_idx,r,d,c,V,cfg_mdp);
    action_str=''; ik=false;
    
    switch act
        case 1  % MOVE
            if target~=st.pt
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
                    action_str=sprintf('ARR(%s)',cfg.names{st.pt});
                    if st.pt==6||st.pt==7
                        do_supply();
                    elseif st.pt==2, action_str='ARRIVE!'; end
                end
                ik=true;
            end
        case 2  % PARK
            st.O=st.O-ca.PO; st.H=st.H-ca.PH; st.F=st.F-ca.PF;
            st.consec_work=0; action_str='PARK'; ik=true;
        case 3  % WORK
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
        case 4  % SUPPLY (only if not already supplied today)
            if (st.pt==6||st.pt==7) && ~supplied_today
                do_supply(); ik=true;
            end
    end
    
    if st.O<-1e-6||st.H<-1e-6||st.F<-1e-6
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f ***\n',...
            st.day,wn,'EXHAUSTED!',st.pos(1),st.pos(2),st.O,st.H,st.F,st.Z,round(st.M));
        break;
    end
    if ik
        fprintf('%4d | %s | %-20s | (%2d,%2d)     |%3.0f%4.0f%4.0f|%5d%6.0f\n',...
            st.day,wn,action_str,st.pos(1),st.pos(2),st.O,st.H,st.F,st.Z,round(st.M));
    end
end

fprintf('-----|---|---------------------|------------|-----------|---------\n');
fprintf('\n===== Result =====\n');
if st.pt==2, fprintf('Day %d Arrived | Z=%d M=%.0f\n',st.day,st.Z,round(st.M));
else, fprintf('FAIL/TIMEOUT Day %d | Z=%d\n',st.day,st.Z); end
fprintf('Done.\n');

    function do_supply()
        dE_s=cfg.dist(st.pt,2); cT_s=cp_engine_v2('cons','thunder');
        nO=dE_s*(0.7*ce.MO+0.3*cT_s.MO);
        nH=dE_s*(0.7*ce.MH+0.3*cT_s.MH);
        nF=dE_s*(0.7*ce.MF+0.3*cT_s.MF);
        sp=cfg.MAX_LOAD-(st.O+st.H+st.F);
        bO=max(0,nO-st.O); bH=max(0,nH-st.H); bF=max(0,nF-st.F);
        if bO+bH+bF>sp, scl=sp/(bO+bH+bF); bO=bO*scl; bH=bH*scl; bF=bF*scl; end
        cost=bO*ca.pO+bH*ca.pH+bF*ca.pF;
        if cost<=st.M
            st.O=st.O+bO; st.H=st.H+bH; st.F=st.F+bF; st.M=st.M-cost;
            action_str=sprintf('SUPPLY(%s)',cfg.names{st.pt});
        end
        supplied_today=true; st.consec_work=0;
    end
end

function r=get_res_level(O,H,F,cfg_mdp)
    if O<cfg_mdp.thresh(1)||H<cfg_mdp.thresh(1)||F<cfg_mdp.thresh_F(1), r=1;
    elseif O<cfg_mdp.thresh(2)||H<cfg_mdp.thresh(2)||F<cfg_mdp.thresh_F(2), r=2;
    elseif O<cfg_mdp.thresh(3)||H<cfg_mdp.thresh(3)||F<cfg_mdp.thresh_F(3), r=3;
    elseif O<cfg_mdp.thresh(4)||H<cfg_mdp.thresh(4)||F<cfg_mdp.thresh_F(4), r=4;
    else, r=5;
    end
end
