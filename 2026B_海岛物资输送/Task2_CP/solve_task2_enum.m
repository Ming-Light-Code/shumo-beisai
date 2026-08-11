%% ========================================================================
%% solve_task2_enum.m - Task 2: Simple Enumeration for Thunderstorm Extreme
%% ========================================================================
%% 功能: 枚举 B→S1→E 的最优停泊策略，解决全雷暴下的载重瓶颈
%% 场景: 30天全雷暴，10×10网格
%% 雷暴消耗: Move(O=8,H=4,F=3) | Park(O=3,H=3,F=2) | Work(O=8,H=6,F=6)
%% 输出: 最优停泊天数与逐日航线
%% ========================================================================

function solve_task2_enum()

%% ====== Parameters ======
MAX_DAYS    = 30;
MAX_LOAD    = 120;
all_xy      = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];

% Storm cost parameters
costMove    = [8, 4, 3];
costPark    = [3, 3, 2];
costWork    = [8, 6, 6];

% Supply prices
priceO = 2; priceH = 1; priceF = 2;

% Key distances
distBS1 = 3;
distS1E = 8;
distBS2 = 7;

%% ====== Connectivity Analysis ======
fprintf('=========================================\n');
fprintf('  Task 2: 30-Day Thunderstorm Extreme Case\n');
fprintf('=========================================\n');
fprintf('Connectivity: B->S1 = %d cells (OK)\n', distBS1);
fprintf('              B->S2 = %d cells (O-need=%d > 35, NO)\n', distBS2, distBS2 * costMove(1));
fprintf('\n');

%% ====== Enumerate B->S1->E with Park Strategy ======
bestM = -inf;
bestC = 0;
bestP = 0;

fprintf('Enumerating B->S1->E (c cells + p park days):\n');
for c = 3:10
    O_base = 35 - costMove(1) * c;
    H_base = 45 - costMove(2) * c;
    F_base = 30 - costMove(3) * c;
    for p = 0:30
        O = O_base - costPark(1) * p;
        H = H_base - costPark(2) * p;
        F = F_base - costPark(3) * p;

        if O < 0 || H < 0 || F < 0
            break;
        end

        loadNow = O + H + F;
        space   = MAX_LOAD - loadNow;

        needO = max(0, costMove(1) * distS1E - O);
        needH = max(0, costMove(2) * distS1E - H);
        needF = max(0, costMove(3) * distS1E - F);
        need  = needO + needH + needF;

        if need <= space
            costSupply = priceO * needO + priceH * needH + priceF * needF;
            M = 240 - costSupply;
            fprintf('  c=%d p=%d: M=%d\n', c, p, M);
            if M > bestM
                bestM = M;
                bestC = c;
                bestP = p;
            end
        end
    end
end

%% ====== Optimal Solution Summary ======
fprintf('\n================ OPTIMAL SOLUTION ================\n');
fprintf('Z = 100  (no work feasible under thunderstorm)\n');
fprintf('M = %d\n', bestM);
fprintf('Route: B (%d cells + %d parks) -> S1 -> E (8 cells)\n', bestC, bestP);
totalDays = bestC + bestP + distS1E;
fprintf('Total: %d days (Travel=%d, Park=%d)\n', totalDays, bestC + distS1E, bestP);
fprintf('\n');

%% ====== Day-by-Day Schedule ======
fprintf('================ DAY-BY-DAY SCHEDULE ================\n');
fprintf('Day | Pos (x,y)  | Action       |  O   H   F  Load |   Z     M\n');
fprintf('----|-------------|--------------|------------------|------------\n');

% Replay with optimal parameters
O = 35; H = 45; F = 30;
M = 240; Z = 100;
day = 0;
pos = [1, 5];

% B -> S1 (with intermediate steps)
for i = 1:bestC
    if i == 1
        pos = [2, 5];
    elseif i == 2
        pos = [3, 5];
    else
        pos = [3, 4];
    end
    day = day + 1;
    O = O - costMove(1);
    H = H - costMove(2);
    F = F - costMove(3);
    fprintf('%3d | (%2d,%2d)     | move         | %3d %3d %3d %4d | %4d %5d\n', ...
        day, pos(1), pos(2), O, H, F, O + H + F, Z, M);
end

% Park at sea before S1
for pi = 1:bestP
    day = day + 1;
    O = O - costPark(1);
    H = H - costPark(2);
    F = F - costPark(3);
    fprintf('%3d | (%2d,%2d)     | park(at sea) | %3d %3d %3d %4d | %4d %5d\n', ...
        day, pos(1), pos(2), O, H, F, O + H + F, Z, M);
end

% Supply at S1
needO = max(0, costMove(1) * distS1E - O);
needH = max(0, costMove(2) * distS1E - H);
needF = max(0, costMove(3) * distS1E - F);
costSupply = priceO * needO + priceH * needH + priceF * needF;
M = M - costSupply;
O = O + needO;
H = H + needH;
F = F + needF;
fprintf('%3d | (%2d,%2d)     | SUPPLY(S1)   | %3d %3d %3d %4d | %4d %5d  (+O=%d H=%d F=%d)\n', ...
    day, 3, 4, O, H, F, O + H + F, Z, M, needO, needH, needF);

% S1 -> E
pathToE = {[4,4], [5,4], [6,4], [7,4], [8,4], [9,4], [9,5], [10,5]};
for i = 1:distS1E
    pt = pathToE{i};
    day = day + 1;
    O = O - costMove(1);
    H = H - costMove(2);
    F = F - costMove(3);
    fprintf('%3d | (%2d,%2d)     | move         | %3d %3d %3d %4d | %4d %5d\n', ...
        day, pt(1), pt(2), O, H, F, O + H + F, Z, M);
end

fprintf('----|-------------|--------------|------------------|------------\n');
fprintf('  Final at E: Z=%d M=%d Day=%d\n', Z, M, day);
fprintf('\nDone.\n');

end
