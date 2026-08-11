function result = sim_q3(cfg, V, weather_seq)
% SIM_Q3 v2: Targeted supply + E-only safe work + supply return fallback

if nargin < 3 || isempty(weather_seq)
    weather_seq = rand(1, cfg.T_MAX) < cfg.pN;
end
result = struct();

pos=cfg.xy(cfg.N_B,:); node_at=cfg.N_B; target=0;
O=cfg.init.O; H=cfg.init.H; F=cfg.init.F;
M=cfg.init.M; Z=cfg.init.Z;
cw=0; day=0; arrived=false; fail_reason=''; supplied_here=false;

% Conservative rates (thunder-biased for safety)
ce_mv = 0.5*cfg.cn(1,:) + 0.5*cfg.ct(1,:);
ce_wk = 0.5*cfg.cn(3,:) + 0.5*cfg.ct(3,:);
ce_pk = 0.5*cfg.cn(2,:) + 0.5*cfg.ct(2,:);
SAFETY = 1.3;
SAFETY_EXIT = 1.0;  % relaxed for return-to-supply exit

MAXLOG=cfg.T_MAX+500;
L=struct();
L.Day=zeros(MAXLOG,1);L.Weather=cell(MAXLOG,1);
L.PosX=zeros(MAXLOG,1);L.PosY=zeros(MAXLOG,1);
L.Node=cell(MAXLOG,1);L.Action=cell(MAXLOG,1);L.Detail=cell(MAXLOG,1);
L.O=zeros(MAXLOG,1);L.H=zeros(MAXLOG,1);L.F=zeros(MAXLOG,1);
L.M=zeros(MAXLOG,1);L.Z=zeros(MAXLOG,1);
L.Load=zeros(MAXLOG,1);L.CW=zeros(MAXLOG,1);
nlog=0;

while day<cfg.T_MAX && ~arrived
    day=day+1;
    is_norm=weather_seq(day);
    if is_norm, w_idx=1;wn='N';cc=cfg.cn;
    else,w_idx=2;wn='T';cc=cfg.ct;end

    [node_now,at_nd]=which_node(pos,cfg);

    if at_nd
        if node_now==cfg.N_E
            arrived=true;write_log('ARR','E');

        elseif ismember(node_now,cfg.N_W)
            wi=node_now-2;maxC=cfg.W_maxC(wi);yld=cfg.W_yield(wi);
            dE=cfg.distE(node_now);

            % Resources needed for E trip (conservative)
            resE_O=dE*ce_mv(1)*SAFETY;
            resE_H=dE*ce_mv(2)*SAFETY;
            resE_F=dE*ce_mv(3)*SAFETY;

            % Can work if resources allow reaching E after
            can_work=(O>resE_O && H>resE_H && F>resE_F) && ...
                     (day+dE<cfg.T_MAX-2) && (cw<maxC);

            if can_work
                O=O-cc(3,1);H=H-cc(3,2);F=F-cc(3,3);Z=Z+yld;cw=cw+1;
                write_log('WORK',cfg.names{node_now});
            elseif cw>=maxC
                O=O-cc(2,1);H=H-cc(2,2);F=F-cc(2,3);cw=0;
                write_log('PARK','(reset)');
            else
                % Must leave: E or supply
                canE=(O>resE_O && H>resE_H && F>resE_F) && (day+dE<cfg.T_MAX-1);
                [ns,~]=nearest_supply(node_now,cfg);
                canS=false;
                if ns>0
                    dS=cfg.dist(node_now,ns);
                    resS_O=dS*ce_mv(1)*SAFETY_EXIT;
                    resS_H=dS*ce_mv(2)*SAFETY_EXIT;
                    resS_F=dS*ce_mv(3)*SAFETY_EXIT;
                    canS=(O>resS_O && H>resS_H && F>resS_F) && ...
                         (day+dS+cfg.dist(ns,cfg.N_E)<cfg.T_MAX-2);
                end
                if canE
                    target=cfg.N_E;node_at=0;
                    [pos,O,H,F]=step_toward(pos,cfg.xy(cfg.N_E,:),O,H,F,cc);
                    cw=0;supplied_here=false;
                    write_log('MOVE','to E');
                    if isequal(pos,cfg.xy(cfg.N_E,:)),node_at=cfg.N_E;arrived=true;end
                elseif canS
                    target=ns;node_at=0;
                    [pos,O,H,F]=step_toward(pos,cfg.xy(ns,:),O,H,F,cc);
                    cw=0;supplied_here=false;
                    write_log('MOVE',['to ' cfg.names{ns}]);
                    if isequal(pos,cfg.xy(ns,:)),node_at=ns;end
                else
                    O=O-cc(2,1);H=H-cc(2,2);F=F-cc(2,3);cw=0;
                    write_log('PARK','');
                end
            end

        elseif ismember(node_now,cfg.N_S) && ~supplied_here
            % ==== TARGETED SUPPLY: buy exact needs for next work cycle ====
            % Find best work point and compute round-trip needs
            best_wp=0;best_val=-inf;best_d_to=0;best_d_back=0;
            for wi=1:3
                wp=cfg.N_W(wi);
                d_to=cfg.dist(node_now,wp);
                ns_wp=nearest_supply_simple(wp,cfg);
                d_back=cfg.dist(wp,ns_wp);
                if day+1+d_to+d_back+cfg.dist(ns_wp,cfg.N_E)>=cfg.T_MAX,continue;end
                rem=cfg.T_MAX-day-1-d_to-d_back-cfg.dist(ns_wp,cfg.N_E);
                wd=rem*cfg.W_maxC(wi)/(cfg.W_maxC(wi)+1);
                val=wd*cfg.W_yield(wi);
                if val>best_val,best_val=val;best_wp=wp;best_d_to=d_to;best_d_back=d_back;end
            end

            if best_wp>0
                wi_wp=best_wp-2;
                % Travel to work point
                nmO=best_d_to*ce_mv(1)*SAFETY;
                nmH=best_d_to*ce_mv(2)*SAFETY;
                nmF=best_d_to*ce_mv(3)*SAFETY;
                % One work cycle (maxC work + park)
                nwO=cfg.W_maxC(wi_wp)*ce_wk(1)+ce_pk(1);
                nwH=cfg.W_maxC(wi_wp)*ce_wk(2)+ce_pk(2);
                nwF=cfg.W_maxC(wi_wp)*ce_wk(3)+ce_pk(3);
                % Return to supply
                nrO=best_d_back*ce_mv(1)*SAFETY_EXIT;
                nrH=best_d_back*ce_mv(2)*SAFETY_EXIT;
                nrF=best_d_back*ce_mv(3)*SAFETY_EXIT;
                need_O=nmO+nwO+nrO; need_H=nmH+nwH+nrH; need_F=nmF+nwF+nrF;
            else
                dE=cfg.distE(node_now);
                need_O=dE*ce_mv(1)*SAFETY;
                need_H=dE*ce_mv(2)*SAFETY;
                need_F=dE*ce_mv(3)*SAFETY;
            end

            buy_O=max(0,need_O-O);buy_H=max(0,need_H-H);buy_F=max(0,need_F-F);

            % Load capacity
            spare=cfg.LOAD_MAX-(O+H+F);
            if buy_O+buy_H+buy_F>spare
                scl=spare/(buy_O+buy_H+buy_F);
                buy_O=buy_O*scl;buy_H=buy_H*scl;buy_F=buy_F*scl;
            end
            cost=buy_O*cfg.price(1)+buy_H*cfg.price(2)+buy_F*cfg.price(3);
            if cost>M,scl=M/cost;buy_O=buy_O*scl;buy_H=buy_H*scl;buy_F=buy_F*scl;cost=M;end

            O=O+buy_O;H=H+buy_H;F=F+buy_F;M=M-cost;
            O=O-cc(2,1);H=H-cc(2,2);F=F-cc(2,3);
            cw=0;supplied_here=true;
            dtl=sprintf('+O%d+H%d+F%d M-%.0f Mrem=%d',round(buy_O),round(buy_H),round(buy_F),cost,round(M));
            write_log('SUPPLY',dtl);

        else
            if node_now==cfg.N_B
                rl=res_level(O,H,F,cfg);cs=min(cfg.mdp.nC,cw+1);
                [act,tgt]=mdp_q3('policy',cfg,V,node_now,w_idx,rl,day,cs);
            else
                best_wp=0;best_val=-inf;
                for wi=1:3
                    wp=cfg.N_W(wi);d_to=cfg.dist(node_now,wp);
                    d_wpE=cfg.dist(wp,cfg.N_E);
                    if day+d_to+d_wpE>=cfg.T_MAX,continue;end
                    rem=cfg.T_MAX-day-d_to-d_wpE;
                    wd=rem*cfg.W_maxC(wi)/(cfg.W_maxC(wi)+1);
                    val=wd*cfg.W_yield(wi);
                    if val>best_val,best_val=val;best_wp=wp;end
                end
                tgt=best_wp;if tgt==0,tgt=cfg.N_E;end
                act=1;
            end
            if act==1&&tgt~=node_now
                target=tgt;node_at=0;
                [pos,O,H,F]=step_toward(pos,cfg.xy(tgt,:),O,H,F,cc);
                cw=0;supplied_here=false;
                write_log('MOVE',['to ' cfg.names{tgt}]);
                if isequal(pos,cfg.xy(tgt,:)),node_at=tgt;target=0;
                    if tgt==cfg.N_E,arrived=true;end
                end
            else
                O=O-cc(2,1);H=H-cc(2,2);F=F-cc(2,3);cw=0;
                write_log('PARK','');
            end
        end
    else
        [pos,O,H,F]=step_toward(pos,cfg.xy(target,:),O,H,F,cc);
        cw=0;
        if isequal(pos,cfg.xy(target,:))
            node_at=target;supplied_here=false;
            write_log('ARR',cfg.names{target});
            if target==cfg.N_E,arrived=true;end
        else
            write_log('TRANSIT',['to ' cfg.names{target}]);
        end
    end

    if O<0||H<0||F<0,fail_reason='RESOURCE';break;end
    if O+H+F>cfg.LOAD_MAX+1e-9,fail_reason='OVERLOAD';break;end
    if M<-1e-9,fail_reason='FUNDS';break;end
end

if ~arrived&&isempty(fail_reason)&&day>=cfg.T_MAX,fail_reason='TIMEOUT';end

L.Day=L.Day(1:nlog);L.Weather=L.Weather(1:nlog);
L.PosX=L.PosX(1:nlog);L.PosY=L.PosY(1:nlog);
L.Node=L.Node(1:nlog);L.Action=L.Action(1:nlog);L.Detail=L.Detail(1:nlog);
L.O=L.O(1:nlog);L.H=L.H(1:nlog);L.F=L.F(1:nlog);
L.M=L.M(1:nlog);L.Z=L.Z(1:nlog);
L.Load=L.Load(1:nlog);L.CW=L.CW(1:nlog);

result.arrived=arrived;result.day=day;
result.Z=Z;result.M=M;result.reason=fail_reason;
result.log=L;

    function write_log(act,dtl)
        nlog=nlog+1;L.Day(nlog)=day;L.Weather{nlog}=wn;
        L.PosX(nlog)=pos(1);L.PosY(nlog)=pos(2);
        if node_at>0,L.Node{nlog}=cfg.names{node_at};
        elseif target>0,L.Node{nlog}=['~' cfg.names{target}];
        else,L.Node{nlog}='?';end
        L.Action{nlog}=act;L.Detail{nlog}=dtl;
        L.O(nlog)=round(O);L.H(nlog)=round(H);L.F(nlog)=round(F);
        L.M(nlog)=round(M);L.Z(nlog)=round(Z);
        L.Load(nlog)=round(O+H+F);L.CW(nlog)=cw;
    end
end

function [node_id,found]=which_node(pos,cfg)
found=false;node_id=0;
for i=1:cfg.nN
    if isequal(pos,cfg.xy(i,:)),node_id=i;found=true;return;end
end
end

function [npos,O,H,F]=step_toward(pos,target,O,H,F,cc)
dx=target(1)-pos(1);dy=target(2)-pos(2);
if abs(dx)>=abs(dy),npos=[pos(1)+sign(dx),pos(2)];
else,npos=[pos(1),pos(2)+sign(dy)];end
O=O-cc(1,1);H=H-cc(1,2);F=F-cc(1,3);
end

function r=res_level(O,H,F,cfg)
for lvl=cfg.mdp.nR:-1:2
    th=cfg.mdp.thresh;
    if O>=th(1,lvl-1)&&H>=th(2,lvl-1)&&F>=th(3,lvl-1),r=lvl;return;end
end
r=1;
end

function [ns,found]=nearest_supply(node,cfg)
found=false;ns=0;best=inf;
for s=cfg.N_S
    d=cfg.dist(node,s);
    if d<best,best=d;ns=s;found=true;end
end
end

function ns=nearest_supply_simple(node,cfg)
[ns,~]=nearest_supply(node,cfg);
if ns==0,ns=cfg.N_S(1);end
end
