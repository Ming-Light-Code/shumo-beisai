% ================================================================
% Q3 Shared Parameters
% Grid: 30x30, B(1,15), E(30,15)
% Supply: S1(12,16), S2(21,16)
% Operation: W1(6,21), W2(15,9), W3(24,24)
% ================================================================

%% ---- Physical coordinates ----
all_xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
names  = {'B','E','W1','W2','W3','S1','S2'};
n_pts = 7;

%% ---- Distances (Manhattan) ----
dist = zeros(n_pts);
for i = 1:n_pts
    for j = 1:n_pts
        dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
    end
end

%% ---- Intermediate point indices ----
inter_idx = [3 4 5 6 7];

%% ---- Resource parameters ----
INIT_O = 100; INIT_H = 150; INIT_F = 100;
INIT_M = 750; INIT_Z = 200;
LOAD_LIMIT = 400;
MAX_DAYS = 90;

%% ---- Weather probabilities ----
P_NORMAL = 0.8; P_STORM = 0.2;

%% ---- Normal weather consumption ----
CM_N = [2, 3, 2]; CS_N = [1, 1, 1]; CW_N = [5, 4, 3];

%% ---- Thunderstorm consumption ----
CM_T = [8, 4, 3]; CS_T = [3, 3, 2]; CW_T = [8, 6, 6];

%% ---- Expected consumption ----
CM_EXP = P_NORMAL * CM_N + P_STORM * CM_T;
CS_EXP = P_NORMAL * CS_N + P_STORM * CS_T;
CW_EXP = P_NORMAL * CW_N + P_STORM * CW_T;

%% ---- Supply prices ----
PRICE_O = 2; PRICE_H = 1; PRICE_F = 2;

%% ---- Operation yields and max continuous days ----
WY = [20, 15, 28]; WM = [4, 5, 3]; MAX_WY = max(WY);

fprintf('=== Q3 Parameters Loaded ===\n');
fprintf('Grid: 30x30, B(%%d,%%d) -> E(%%d,%%d)\n', all_xy(1,1), all_xy(1,2), all_xy(2,1), all_xy(2,2));
fprintf('S1(%%d,%%d) S2(%%d,%%d)\n', all_xy(6,1), all_xy(6,2), all_xy(7,1), all_xy(7,2));
fprintf('W1(%%d,%%d) W2(%%d,%%d) W3(%%d,%%d)\n', all_xy(3,1), all_xy(3,2), all_xy(4,1), all_xy(4,2), all_xy(5,1), all_xy(5,2));
fprintf('Init: O=%%d H=%%d F=%%d M=%%d Z=%%d\n', INIT_O, INIT_H, INIT_F, INIT_M, INIT_Z);
fprintf('Load=%%d Days=%%d\n', LOAD_LIMIT, MAX_DAYS);
fprintf('Exp Move: O=%%5.1f H=%%5.1f F=%%5.1f\n', CM_EXP(1), CM_EXP(2), CM_EXP(3));
fprintf('Exp Work: O=%%5.1f H=%%5.1f F=%%5.1f\n', CW_EXP(1), CW_EXP(2), CW_EXP(3));
fprintf('Exp Stop: O=%%5.1f H=%%5.1f F=%%5.1f\n\n', CS_EXP(1), CS_EXP(2), CS_EXP(3));
