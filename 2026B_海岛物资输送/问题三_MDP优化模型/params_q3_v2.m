function cfg = params_q3_v2()
cfg.xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
cfg.names = {'B','E','W1','W2','W3','S1','S2'};
cfg.nN = size(cfg.xy, 1); cfg.N_B = 1; cfg.N_E = 2; cfg.N_W = [3 4 5]; cfg.N_S = [6 7];
cfg.dist = zeros(cfg.nN);
for i=1:cfg.nN,for j=1:cfg.nN,cfg.dist(i,j)=abs(cfg.xy(i,1)-cfg.xy(j,1))+abs(cfg.xy(i,2)-cfg.xy(j,2));end,end
cfg.distE = cfg.dist(:, cfg.N_E)';
cfg.W_yield=[20 15 28]; cfg.W_maxC=[4 5 3];
cfg.T_MAX=90; cfg.LOAD_MAX=400;
cfg.pN=0.8; cfg.pT=0.2;
cfg.init=struct('O',100,'H',150,'F',100,'M',750,'Z',200);
cfg.cn=[2 3 2;1 1 1;5 4 3]; cfg.ct=[8 4 3;3 3 2;8 6 6];
cfg.ce_move=cfg.pN*cfg.cn(1,:)+cfg.pT*cfg.ct(1,:);
cfg.ce_park=cfg.pN*cfg.cn(2,:)+cfg.pT*cfg.ct(2,:);
cfg.ce_work=cfg.pN*cfg.cn(3,:)+cfg.pT*cfg.ct(3,:);
cfg.price=[2 1 2];
cfg.PLAN_SAFETY=1.10; cfg.M_RESERVE=100; cfg.E_BUFFER=6;
end
