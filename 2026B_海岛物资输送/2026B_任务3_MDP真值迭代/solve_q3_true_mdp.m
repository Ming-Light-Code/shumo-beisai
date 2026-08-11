function solve_q3_true_mdp()
% 真正MDP值迭代求解器 — 后向Bellman方程
% 状态: (pt, oLv, hLv, fLv, mLv, d), 161,280状态
cfg = get_config();
fprintf('=== True MDP Value Iteration ===\n');
fprintf('States: 7x4x4x4x4x90 = %d\n\n', 7*4*4*4*4*90);

tic; [V, pol_np, pol_wd] = value_iteration(cfg);
fprintf('VI done: %.1fs\n', toc);

[Zd, Md, pd] = simulate_det(cfg, pol_np, pol_wd);
fprintf('Deterministic: Z=%d path=%s\n', Zd, pd);

fprintf('MC (N=30)... '); tic;
succ=0; Zs=[];
for s=1:30
    [Zf,~,ok] = simulate_stoch(cfg, pol_np, pol_wd);
    if ok, succ=succ+1; Zs(end+1)=Zf; end
end
fprintf('%.1fs | Success: %d/30 (%.0f%%)\n', toc, succ, 100*succ/30);
if succ>0, fprintf('Z: mean=%.0f min=%d max=%d\n', mean(Zs), min(Zs), max(Zs)); end
fprintf('Done.\n');
end

function cfg = get_config()
cfg.T=90; cfg.L=400;
cfg.xy=[1 15;30 15;6 21;15 9;24 24;12 16;21 16];
cfg.nm={'B','E','W1','W2','W3','S1','S2'};
cfg.wy=[20 15 28]; cfg.wm=[4 5 3];
cfg.iO=100; cfg.iH=150; cfg.iF=100; cfg.iM=750; cfg.iZ=200;
n=size(cfg.xy,1); cfg.d=zeros(n);
for i=1:n,for j=1:n,cfg.d(i,j)=abs(cfg.xy(i,1)-cfg.xy(j,1))+abs(cfg.xy(i,2)-cfg.xy(j,2));end,end
cfg.Ob=[0 50 120 250 400]; cfg.Hb=[0 60 140 280 400];
cfg.Fb=[0 40 100 200 400]; cfg.Mb=[0 150 400 750];
cfg.nO=4;cfg.nH=4;cfg.nF=4;cfg.nM=4;
cfg.wo=[0 3 6 9 12];
end

function [V, pol_np, pol_wd] = value_iteration(cfg)
V=-inf(cfg.nO,cfg.nH,cfg.nF,cfg.nM,7,cfg.T+1);
pol_np=zeros(cfg.nO,cfg.nH,cfg.nF,cfg.nM,7,cfg.T);
pol_wd=zeros(cfg.nO,cfg.nH,cfg.nF,cfg.nM,7,cfg.T);
for o=1:cfg.nO,for h=1:cfg.nH,for f=1:cfg.nF,for m=1:cfg.nM
    V(o,h,f,m,2,:)=200+1e-6*cfg.Mb(m);
end,end,end,end

% Pre-compute binomial cache
bc=cell(50,1);
for d=1:50, bc{d}=arrayfun(@(k)nchoosek(d,k),0:d); end

for day=cfg.T:-1:1
    if mod(day,30)==0, fprintf(' d=%d...\n',day); end
    for pt=[1 3 4 5 6 7]
        for o=1:cfg.nO,for h=1:cfg.nH,for f=1:cfg.nF,for m=1:cfg.nM
            ov=bm(cfg.Ob,o); hv=bm(cfg.Hb,h); fv=bm(cfg.Fb,f);
            if ov+hv+fv>cfg.L, continue; end
            bestV=-inf; bestN=0; bestW=0;
            for np=[2 3 4 5 6 7]
                if np==pt, continue; end
                dm=cfg.d(pt,np);
                if day+dm>cfg.T+1, continue; end
                wo=[0]; if np>=3&&np<=5, wo=cfg.wo; end
                for wi=1:length(wo)
                    wd=wo(wi); npk=0;
                    if wd>0, npk=max(0,ceil(wd/cfg.wm(np-2))-1); end
                    if day+dm+wd+npk>cfg.T+1, continue; end
                    % Compute expected value of moving to np
                    mv=cfg.Mb(m); ev=0;
                    for k=0:dm
                        p=bc{dm}(k+1)*0.2^k*0.8^(dm-k);
                        if p<1e-8, continue; end
                        oa=ov-(dm-k)*2-k*8;
                        ha=hv-(dm-k)*3-k*4;
                        fa=fv-(dm-k)*2-k*3;
                        if oa<0||ha<0||fa<0, continue; end
                        nd=day+dm;
                        if np==6||np==7
                            sp=cfg.L-(oa+ha+fa);
                            if sp>0
                                bo=sp*.25;bh=sp*.5;bf=sp-bo-bh;
                                cst=bo*2+bh+bf*2;
                                if cst<=mv, oa=oa+bo;ha=ha+bh;fa=fa+bf;mv=mv-cst; end
                            end
                        end
                        zg=0;
                        if wd>0&&np>=3&&np<=5
                            wi2=np-2;
                            oa=oa-wd*5-npk*1; ha=ha-wd*4-npk*1; fa=fa-wd*3-npk*1;
                            zg=wd*cfg.wy(wi2); nd=nd+wd+npk;
                        end
                        if oa<0||ha<0||fa<0, continue; end
                        no=dsc(oa,cfg.Ob); nh=dsc(ha,cfg.Hb);
                        nf=dsc(fa,cfg.Fb); nm=dsc(mv,cfg.Mb);
                        if nd>cfg.T, continue; end
                        vn=V(no,nh,nf,nm,np,nd);
                        if vn>-inf, ev=ev+p*(zg+vn); end
                    end
                    if ev>bestV, bestV=ev; bestN=np; bestW=wd; end
                end
            end
            V(o,h,f,m,pt,day)=bestV;
            pol_np(o,h,f,m,pt,day)=bestN;
            pol_wd(o,h,f,m,pt,day)=bestW;
        end,end,end,end
    end
end
end

function [Zf,Mf,ps]=simulate_det(cfg,pol_np,pol_wd)
pt=1;d=1;O=cfg.iO;H=cfg.iH;F=cfg.iF;M=cfg.iM;Z=cfg.iZ;
pp=[1];
while pt~=2&&d<=cfg.T
    o=dsc(O,cfg.Ob);h=dsc(H,cfg.Hb);f=dsc(F,cfg.Fb);m=dsc(M,cfg.Mb);
    np=pol_np(o,h,f,m,pt,d); wd=pol_wd(o,h,f,m,pt,d);
    if np==0, break; end
    dm=cfg.d(pt,np);
    O=O-dm*3.2;H=H-dm*3.2;F=F-dm*2.2; d=d+dm;
    pt=np; pp(end+1)=pt;
    if pt==6||pt==7
        sp=cfg.L-(O+H+F);
        if sp>0, O=O+sp*.25;H=H+sp*.5;F=F+sp-sp*.25-sp*.5; end
    end
    if wd>0&&pt>=3&&pt<=5
        wi=pt-2; npk=max(0,ceil(wd/cfg.wm(wi))-1);
        O=O-wd*5.6-npk*1.4; H=H-wd*4.4-npk*1.4; F=F-wd*3.6-npk*1.2;
        Z=Z+wd*cfg.wy(wi); d=d+wd+npk;
    end
    if O<0||H<0||F<0, break; end
end
Zf=Z;Mf=M;ps=strjoin(cfg.nm(pp),' -> ');
end

function [Zf,Mf,ok]=simulate_stoch(cfg,pol_np,pol_wd)
pt=1;d=1;O=cfg.iO;H=cfg.iH;F=cfg.iF;M=cfg.iM;Z=cfg.iZ;
while pt~=2&&d<=cfg.T
    o=dsc(O,cfg.Ob);h=dsc(H,cfg.Hb);f=dsc(F,cfg.Fb);m=dsc(M,cfg.Mb);
    np=pol_np(o,h,f,m,pt,d); wd=pol_wd(o,h,f,m,pt,d);
    if np==0, ok=false;Zf=Z;Mf=M;return; end
    dm=cfg.d(pt,np);
    for s=1:dm
        if rand<0.8, O=O-2;H=H-3;F=F-2; else O=O-8;H=H-4;F=F-3; end
        if O<0||H<0||F<0, ok=false;Zf=0;Mf=0;return; end
    end
    d=d+dm;pt=np;
    if pt==6||pt==7
        sp=cfg.L-(O+H+F);
        if sp>0
            bo=sp*.25;bh=sp*.5;bf=sp-bo-bh;cst=bo*2+bh+bf*2;
            if cst<=M,O=O+bo;H=H+bh;F=F+bf;M=M-cst;end
        end
    end
    if wd>0&&pt>=3&&pt<=5
        wi=pt-2;npk=max(0,ceil(wd/cfg.wm(wi))-1);
        for s=1:wd
            if rand<0.8, O=O-5;H=H-4;F=F-3; else O=O-8;H=H-6;F=F-6; end
            Z=Z+cfg.wy(wi); if O<0||H<0||F<0, ok=false;Zf=0;Mf=0;return; end
        end
        for s=1:npk
            if rand<0.8, O=O-1;H=H-1;F=F-1; else O=O-3;H=H-3;F=F-2; end
            if O<0||H<0||F<0, ok=false;Zf=0;Mf=0;return; end
        end
        d=d+wd+npk;
    end
end
ok=(pt==2);Zf=Z;Mf=M; if~ok,Zf=0;Mf=0;end
end

function l=dsc(v,b),for i=1:length(b)-1,if v<=b(i+1),l=i;return;end,end,l=length(b)-1;end
function m=bm(b,l),m=(b(l)+b(min(l+1,length(b))))/2;end
