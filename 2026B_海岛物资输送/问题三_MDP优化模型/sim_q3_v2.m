function result = sim_q3_v2(cfg, weather_seq)
if nargin<2||isempty(weather_seq), weather_seq=rand(1,cfg.T_MAX)<cfg.pN; end
pos=cfg.xy(cfg.N_B,:); target=cfg.N_S(2);
O=cfg.init.O; H=cfg.init.H; F=cfg.init.F; M=cfg.init.M; Z=cfg.init.Z;
cw=0; day=0; arrived=false; fail_reason=''; supplied=false;
N_planned=0; work_done=0; go_to_E=false; plan_target=cfg.N_W(3);
MAXLOG=cfg.T_MAX+500; L=struct();
L.Day=zeros(MAXLOG,1); L.Weather=cell(MAXLOG,1);
L.PosX=zeros(MAXLOG,1); L.PosY=zeros(MAXLOG,1); L.Node=cell(MAXLOG,1);
L.Action=cell(MAXLOG,1); L.Detail=cell(MAXLOG,1);
L.O=zeros(MAXLOG,1); L.H=zeros(MAXLOG,1); L.F=zeros(MAXLOG,1);
L.M=zeros(MAXLOG,1); L.Z=zeros(MAXLOG,1); L.Load=zeros(MAXLOG,1); L.CW=zeros(MAXLOG,1);
nlog=0;
while day<=cfg.T_MAX&&~arrived
    day=day+1; if day>cfg.T_MAX, break; end
    is_n=weather_seq(day); if is_n, wn='N'; cc=cfg.cn; else wn='T'; cc=cfg.ct; end
    [nd,~]=which_node(pos,cfg);
    if nd>0
        if nd==cfg.N_E, arrived=true; write_log('ARR','E');
        elseif nd==cfg.N_B
            target=cfg.N_S(2); supplied=false; N_planned=0; work_done=0;
            [pos,O,H,F]=step_toward(pos,cfg.xy(target,:),O,H,F,cc); cw=0;
            write_log('MOVE',['to ' cfg.names{target}]);
        elseif ismember(nd,cfg.N_W)
            wi=nd-2; mxC=cfg.W_maxC(wi); yd=cfg.W_yield(wi);
            dE=cfg.distE(nd); dS=nsp_dist(nd,cfg);
            tO=dE*cfg.ct(1,1)+cfg.E_BUFFER; tH=dE*cfg.ct(1,2)+cfg.E_BUFFER; tF=dE*cfg.ct(1,3)+cfg.E_BUFFER;
            rSO=dS*cfg.ce_move(1)*1.05; rSH=dS*cfg.ce_move(2)*1.05; rSF=dS*cfg.ce_move(3)*1.05;
            must=(day+dE>cfg.T_MAX)||(work_done>=N_planned&&N_planned>0);
            if must
                if go_to_E
                    if O>tO&&H>tH&&F>tF, target=cfg.N_E;
                    elseif O>rSO&&H>rSH&&F>rSF&&dS>0, target=nsp_node(nd,cfg); supplied=false; N_planned=0; work_done=0;
                    else target=cfg.N_E; end
                else target=plan_target; supplied=false; N_planned=0; work_done=0; end
                [pos,O,H,F]=step_toward(pos,cfg.xy(target,:),O,H,F,cc); cw=0;
                write_log('MOVE',['to ' cfg.names{target}]);
            elseif cw>=mxC
                O=O-cc(2,1); H=H-cc(2,2); F=F-cc(2,3); cw=0; write_log('PARK','(reset)');
            else
                aO=O-cc(3,1); aH=H-cc(3,2); aF=F-cc(3,3);
                if aO>tO&&aH>tH&&aF>tF
                    O=aO; H=aH; F=aF; Z=Z+yd; cw=cw+1; work_done=work_done+1;
                    write_log('WORK',cfg.names{nd});
                elseif O>rSO&&H>rSH&&F>rSF&&dS>0
                    target=nsp_node(nd,cfg); [pos,O,H,F]=step_toward(pos,cfg.xy(target,:),O,H,F,cc);
                    cw=0; supplied=false; N_planned=0; work_done=0;
                    write_log('MOVE',['to ' cfg.names{target}]);
                else target=cfg.N_E; [pos,O,H,F]=step_toward(pos,cfg.xy(target,:),O,H,F,cc); cw=0; write_log('MOVE','to E (emerg)'); end
            end
        elseif ismember(nd,cfg.N_S)
            if ~supplied
                [N_planned,bO,bH,bF,go_to_E,plan_target]=plan_supply(O,H,F,M,day,cfg,nd);
                O=O+bO; H=H+bH; F=F+bF; M=M-(bO*cfg.price(1)+bH*cfg.price(2)+bF*cfg.price(3));
                O=O-cc(2,1); H=H-cc(2,2); F=F-cc(2,3); cw=0; supplied=true; work_done=0;
                target=cfg.N_W(3);
                dtl=sprintf('+O%d+H%d+F%d M-%.0f rem=%.0f N=%d toE=%d',round(bO),round(bH),round(bF),bO*cfg.price(1)+bH*cfg.price(2)+bF*cfg.price(3),round(M),N_planned,go_to_E);
                write_log('SUPPLY',dtl);
            else
                if go_to_E, target=cfg.N_E; end
                if target==nd||target<1, target=cfg.N_W(3); end
                [pos,O,H,F]=step_toward(pos,cfg.xy(target,:),O,H,F,cc); cw=0;
                write_log('MOVE',['to ' cfg.names{target}]);
            end
        end
    else
        [pos,O,H,F]=step_toward(pos,cfg.xy(target,:),O,H,F,cc); cw=0;
        if isequal(pos,cfg.xy(target,:))
            nd=target; if target==cfg.N_E, arrived=true; end
            supplied=false; write_log('ARR',cfg.names{target});
        else write_log('TRANSIT',['to ' cfg.names{target}]); end
    end
    if O<0||H<0||F<0, fail_reason='RESOURCE'; break; end
    if O+H+F>cfg.LOAD_MAX+1e-9, fail_reason='OVERLOAD'; break; end
    if M<-1e-9, fail_reason='FUNDS'; break; end
end
if ~arrived&&isempty(fail_reason), fail_reason='TIMEOUT'; end
flds=fieldnames(L); for i=1:numel(flds), L.(flds{i})=L.(flds{i})(1:nlog); end
result.arrived=arrived; result.day=day; result.Z=Z; result.M=M; result.reason=fail_reason; result.log=L;
    function write_log(act,dtl)
        nlog=nlog+1; L.Day(nlog)=day; L.Weather{nlog}=wn;
        L.PosX(nlog)=pos(1); L.PosY(nlog)=pos(2);
        if nd>0, L.Node{nlog}=cfg.names{nd};
        elseif target>0, L.Node{nlog}=['~' cfg.names{target}]; else L.Node{nlog}='?'; end
        L.Action{nlog}=act; L.Detail{nlog}=dtl;
        L.O(nlog)=round(O); L.H(nlog)=round(H); L.F(nlog)=round(F);
        L.M(nlog)=round(M); L.Z(nlog)=round(Z); L.Load(nlog)=round(O+H+F); L.CW(nlog)=cw;
    end
end
function [id,f]=which_node(pos,cfg)
    f=false; id=0; for i=1:cfg.nN, if isequal(pos,cfg.xy(i,:)), id=i; f=true; return; end, end
end
function [np,O,H,F]=step_toward(pos,tgt,O,H,F,cc)
    dx=tgt(1)-pos(1); dy=tgt(2)-pos(2);
    if abs(dx)>=abs(dy), np=[pos(1)+sign(dx),pos(2)]; else np=[pos(1),pos(2)+sign(dy)]; end
    O=O-cc(1,1); H=H-cc(1,2); F=F-cc(1,3);
end
function d=nsp_dist(node,cfg)
    d=inf; for s=cfg.N_S, ds=cfg.dist(node,s); if ds<d, d=ds; end, end; if isinf(d), d=0; end
end
function ns=nsp_node(node,cfg)
    ns=0; best=inf; for s=cfg.N_S, ds=cfg.dist(node,s); if ds<best, best=ds; ns=s; end, end
end

function [N,buy_O,buy_H,buy_F,go_to_E,plan_target] = plan_supply(O,H,F,M,day,cfg,node)
    d_sw=cfg.dist(node,cfg.N_W(3)); d_se=cfg.distE(node);
    cycO=3*cfg.ce_work(1)+cfg.ce_park(1); cycH=3*cfg.ce_work(2)+cfg.ce_park(2); cycF=3*cfg.ce_work(3)+cfg.ce_park(3);
    rt_days=d_sw+d_sw+2; ft_days=d_sw+d_se+2;
    can_rt=(day+rt_days+ft_days<=cfg.T_MAX);
    if can_rt
        transit=d_sw+d_sw; e_safety_O=0; e_safety_H=0; e_safety_F=0;
        E_rsv_O=0; E_rsv_H=d_se*cfg.ct(1,2)+cfg.E_BUFFER; E_rsv_F=d_se*cfg.ct(1,3)+cfg.E_BUFFER;
        plan_target=cfg.N_S(2); go_to_E=false;
    else
        transit=d_sw+d_se;
        e_safety_O=d_se*cfg.ct(1,1)+cfg.E_BUFFER; e_safety_H=d_se*cfg.ct(1,2)+cfg.E_BUFFER; e_safety_F=d_se*cfg.ct(1,3)+cfg.E_BUFFER;
        E_rsv_O=0; E_rsv_H=0; E_rsv_F=0; go_to_E=true; plan_target=cfg.N_E;
    end
    for N=6:-1:0
        % O: safety only on variable part (transit+work), not on thunder-E
        need_O=(transit*cfg.ce_move(1)+N*cycO)*cfg.PLAN_SAFETY+e_safety_O+E_rsv_O;
        need_H=(transit*cfg.ce_move(2)+N*cycH)*1.03+e_safety_H+E_rsv_H;
        need_F=(transit*cfg.ce_move(3)+N*cycF)*1.03+e_safety_F+E_rsv_F;
        bO=max(0,need_O-O); bH=max(0,need_H-H); bF=max(0,need_F-F);
        nL=(O+bO)+(H+bH)+(F+bF); c=bO*cfg.price(1)+bH*cfg.price(2)+bF*cfg.price(3);
        if nL<=cfg.LOAD_MAX&&c<=M-cfg.M_RESERVE&&day+transit+N*4<=cfg.T_MAX
            buy_O=bO; buy_H=bH; buy_F=bF; return;
        end
    end
    % Fallback: E-only thunder-safe
    need_O=d_se*cfg.ct(1,1)+cfg.E_BUFFER*2; need_H=d_se*cfg.ct(1,2)+cfg.E_BUFFER*2; need_F=d_se*cfg.ct(1,3)+cfg.E_BUFFER*2;
    bO=max(0,need_O-O); bH=max(0,need_H-H); bF=max(0,need_F-F);
    nL=(O+bO)+(H+bH)+(F+bF); c=bO*cfg.price(1)+bH*cfg.price(2)+bF*cfg.price(3);
    if nL>cfg.LOAD_MAX||c>M, s=min(cfg.LOAD_MAX/nL,(M-cfg.M_RESERVE)/max(c,1)); bO=bO*s; bH=bH*s; bF=bF*s; end
    N=0; buy_O=bO; buy_H=bH; buy_F=bF; go_to_E=true; plan_target=cfg.N_E;
end
