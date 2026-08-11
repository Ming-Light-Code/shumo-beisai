function varargout = mdp_q3(action, varargin)
% MDP_Q3  Markov Decision Process Engine (backward induction, 90 days)

switch action
    case 'train'
        cfg = varargin{1};
        varargout{1} = train_BI(cfg);
    case 'policy'
        [cfg, V, node, w, r, day, c] = varargin{:};
        [act, tgt, ~] = best_action(cfg, V, node, w, r, day, c);
        varargout{1} = act;
        varargout{2} = tgt;
    otherwise
        error('mdp_q3: unknown action "%s"', action);
end
end

function V = train_BI(cfg)
% Backward induction: day=90 down to 1
nN=cfg.nN; nW=cfg.mdp.nW; nR=cfg.mdp.nR; nD=cfg.mdp.nD; nC=cfg.mdp.nC;
fprintf('MDP: %d states, backward induction...\n', nN*nW*nR*nD*nC);

V = zeros(nN, nW, nR, nD, nC);
ARRIVAL_BONUS = 0;  % Z tracked through work yields

for day = nD:-1:1
    for node = 1:nN
        for w = 1:nW
            for r = 1:nR
                for c = 1:nC
                    if node == cfg.N_E
                        % At E: arrival bonus (task complete)
                        V(node,w,r,day,c) = ARRIVAL_BONUS;
                    elseif day + cfg.distE(node) > cfg.T_MAX
                        % Cannot reach E in time
                        V(node,w,r,day,c) = -1e6;
                    else
                        best = -inf;
                        % MOVE to each node
                        for tgt = 1:nN
                            if tgt == node, continue; end
                            v = ev_move(cfg, V, node, tgt, w, r, day, c);
                            if v > best, best = v; end
                        end
                        % PARK
                        v = ev_park(cfg, V, node, w, r, day, c);
                        if v > best, best = v; end
                        % WORK
                        if ismember(node, cfg.N_W)
                            wi = node - 2;
                            if c <= cfg.W_maxC(wi)
                                v = ev_work(cfg, V, node, w, r, day, c);
                                if v > best, best = v; end
                            end
                        end
                        % SUPPLY
                        if ismember(node, cfg.N_S)
                            v = ev_supply(cfg, V, node, w, r, day, c);
                            if v > best, best = v; end
                        end
                        V(node,w,r,day,c) = best;
                    end
                end
            end
        end
    end
    if mod(day, 10) == 0
        fprintf('  day %3d processed\n', day);
    end
end
fprintf('  Backward induction complete.\n');
end

function [act,tgt,val] = best_action(cfg, V, node, w, r, day, c)
act=2; tgt=node; val=-inf;
for t=1:cfg.nN
    if t==node, continue; end
    v=ev_move(cfg,V,node,t,w,r,day,c);
    if v>val, val=v; act=1; tgt=t; end
end
v=ev_park(cfg,V,node,w,r,day,c);
if v>val, val=v; act=2; tgt=node; end
if ismember(node,cfg.N_W)
    wi=node-2;
    if c<=cfg.W_maxC(wi)
        v=ev_work(cfg,V,node,w,r,day,c);
        if v>val, val=v; act=3; tgt=node; end
    end
end
if ismember(node,cfg.N_S)
    v=ev_supply(cfg,V,node,w,r,day,c);
    if v>val, val=v; act=4; tgt=node; end
end
end

function val=ev_move(cfg,V,src,tgt,w,r,day,~)
dist=cfg.dist(src,tgt);
new_day=day+dist;
if new_day>cfg.T_MAX, val=-inf; return; end
expO=dist*cfg.ce_move(1); expH=dist*cfg.ce_move(2); expF=dist*cfg.ce_move(3);
r_new=consume_level(r,expO,expH,expF,cfg);
if r_new==0, val=-inf; return; end
vN=V(tgt,1,r_new,new_day,1); vT=V(tgt,2,r_new,new_day,1);
val=cfg.pN*vN+cfg.pT*vT - 0.01;
end

function val=ev_park(cfg,V,node,w,r,day,~)
new_day=day+1;
if new_day>cfg.T_MAX, val=-inf; return; end
if w==1, dO=cfg.cn(2,1); dH=cfg.cn(2,2); dF=cfg.cn(2,3);
else, dO=cfg.ct(2,1); dH=cfg.ct(2,2); dF=cfg.ct(2,3); end
r_new=consume_level(r,dO,dH,dF,cfg);
if r_new==0, val=-inf; return; end
vN=V(node,1,r_new,new_day,1); vT=V(node,2,r_new,new_day,1);
val=cfg.pN*vN+cfg.pT*vT - 0.01;
end

function val=ev_work(cfg,V,node,w,r,day,c)
wi=node-2;
new_day=day+1;
if new_day>cfg.T_MAX, val=-inf; return; end
if w==1, dO=cfg.cn(3,1); dH=cfg.cn(3,2); dF=cfg.cn(3,3);
else, dO=cfg.ct(3,1); dH=cfg.ct(3,2); dF=cfg.ct(3,3); end
r_new=consume_level(r,dO,dH,dF,cfg);
if r_new==0, val=-inf; return; end
c_new=min(cfg.mdp.nC,c+1);
vN=V(node,1,r_new,new_day,c_new); vT=V(node,2,r_new,new_day,c_new);
val=cfg.W_yield(wi)+cfg.pN*vN+cfg.pT*vT;
end

function val=ev_supply(cfg,V,node,w,r,day,~)
new_day=day+1;
if new_day>cfg.T_MAX, val=-inf; return; end
if w==1, dO=cfg.cn(2,1); dH=cfg.cn(2,2); dF=cfg.cn(2,3);
else, dO=cfg.ct(2,1); dH=cfg.ct(2,2); dF=cfg.ct(2,3); end
r_after=min(cfg.mdp.nR,r+3);
cost_penalty=(r_after-r)*40*0.005;
r_net=consume_level(r_after,dO,dH,dF,cfg);
if r_net==0, val=-inf; return; end
vN=V(node,1,r_net,new_day,1); vT=V(node,2,r_net,new_day,1);
val=cfg.pN*vN+cfg.pT*vT-cost_penalty;
end

function r2=consume_level(r,dO,dH,dF,cfg)
if r<1||r>cfg.mdp.nR, r2=0; return; end
remO=cfg.mdp.midpt(1,r)-dO;
remH=cfg.mdp.midpt(2,r)-dH;
remF=cfg.mdp.midpt(3,r)-dF;
if remO<0||remH<0||remF<0, r2=0; return; end
r2=quantize(remO,remH,remF,cfg);
end

function r=quantize(O,H,F,cfg)
for lvl=cfg.mdp.nR:-1:2
    if O>=cfg.mdp.thresh(1,lvl-1)&&H>=cfg.mdp.thresh(2,lvl-1)&&F>=cfg.mdp.thresh(3,lvl-1)
        r=lvl; return;
    end
end
r=1;
end
