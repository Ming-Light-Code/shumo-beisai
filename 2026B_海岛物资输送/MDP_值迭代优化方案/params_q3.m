function cfg = params_q3()
% PARAMS_Q3  Problem 3 Configuration
% 30x30 grid, 90 days, weather p=0.8/0.2, 7 nodes

% ── Grid & Node Coordinates ──────────────────────────────────────────
cfg.GRID = 30;
cfg.xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
cfg.names = {'B','E','W1','W2','W3','S1','S2'};
cfg.nN = size(cfg.xy, 1);
cfg.N_B = 1;  cfg.N_E = 2;
cfg.N_W = [3 4 5];
cfg.N_S = [6 7];

% Manhattan distances between all node pairs
cfg.dist = zeros(cfg.nN);
for i = 1:cfg.nN
    for j = 1:cfg.nN
        cfg.dist(i,j) = abs(cfg.xy(i,1)-cfg.xy(j,1)) + abs(cfg.xy(i,2)-cfg.xy(j,2));
    end
end
cfg.distE = cfg.dist(:, cfg.N_E)';  % distance from each node to E

% ── Work Points ──────────────────────────────────────────────────────
cfg.W_yield = [20 15 28];     % Z gain per work day
cfg.W_maxC  = [4 5 3];        % max consecutive work days

% ── Task Constraints ──────────────────────────────────────────────────
cfg.T_MAX   = 90;
cfg.LOAD_MAX = 400;

% ── Weather ──────────────────────────────────────────────────────────
cfg.pN = 0.8;   % P(normal)
cfg.pT = 0.2;   % P(thunder)

% ── Initial State ────────────────────────────────────────────────────
cfg.init = struct('O',100,'H',150,'F',100,'M',750,'Z',200);

% ── Consumption Tables ───────────────────────────────────────────────
%    [move_O move_H move_F; park_O park_H park_F; work_O work_H work_F]
cfg.cn = [2 3 2; 1 1 1; 5 4 3];   % normal weather
cfg.ct = [8 4 3; 3 3 2; 8 6 6];   % thunder weather

% Expected consumption = pN*normal + pT*thunder
cfg.ce_move = cfg.pN*cfg.cn(1,:) + cfg.pT*cfg.ct(1,:);
cfg.ce_park = cfg.pN*cfg.cn(2,:) + cfg.pT*cfg.ct(2,:);
cfg.ce_work = cfg.pN*cfg.cn(3,:) + cfg.pT*cfg.ct(3,:);

% ── Supply Prices ────────────────────────────────────────────────────
cfg.price = [2 1 2];  % O, H, F

% ── MDP State Discretization ─────────────────────────────────────────
% Resource thresholds: 5 levels (1=critical .. 5=full)
cfg.mdp = struct();
cfg.mdp.thresh = [40 80 120 160 200 240 280 320 360;   % O
                   40 80 120 160 200 240 280 320 360;   % H
                   40 80 120 160 200 240 280 320 360];  % F
cfg.mdp.midpt  = [20  60 100 140 180 220 260 300 340 380;   % O midpoints
                   20  60 100 140 180 220 260 300 340 380;   % H midpoints
                   20  60 100 140 180 220 260 300 340 380];  % F midpoints
cfg.mdp.nR = 10;     % resource levels
cfg.mdp.nW = 2;     % weather states
cfg.mdp.nD = 90;     % time stages: 1-30, 31-60, 61-90
cfg.mdp.nC = 5;     % consec work: 0,1,2,3,4+
cfg.mdp.gamma   = 1.0;
cfg.mdp.maxIter = 500;
cfg.mdp.tol     = 1e-5;

% ── MDP Supply Action Costs (approximate) ────────────────────────────
% Expected purchase amounts and costs per resource level increment
cfg.mdp.supply_cost_per_level = [80 80 80 80];  % rough cost per +1 level
end
