function solve_q3_full(weather_seq)
% SOLVE_Q3_FULL  Complete Problem 3 solver with result.xls output
%   solve_q3_full           -- run with random weather
%   solve_q3_full(wseq)     -- run with fixed weather sequence

fprintf('========================================\n');
fprintf('  Problem 3: MDP + Grid Navigation\n');
fprintf('  30x30 Grid | 90 Days | p(N)=0.8\n');
fprintf('========================================\n\n');

% ---- 1. Configuration ----
cfg = params_q3();

% ---- 2. MDP Training ----
fprintf('--- MDP Training ---\n');
tic;
V = mdp_q3('train', cfg);
fprintf('  Training time: %.1fs\n\n', toc);

% ---- 3. Weather Generation ----
if nargin < 1 || isempty(weather_seq)
    rng(42);  % reproducible
    weather_seq = rand(1, cfg.T_MAX) < cfg.pN;
end

% ---- 4. Online Simulation ----
fprintf('--- Online Simulation ---\n');
tic;
result = sim_q3(cfg, V, weather_seq);
fprintf('  Simulation time: %.1fs\n\n', toc);

% ---- 5. Results ----
fprintf('========================================\n');
fprintf('  Results\n');
fprintf('========================================\n');
if result.arrived
    fprintf('  STATUS:  ARRIVED at E\n');
    fprintf('  Day:     %d / %d\n', result.day, cfg.T_MAX);
    fprintf('  Z:       %d\n', result.Z);
    fprintf('  M:       %.0f\n', result.M);
else
    fprintf('  STATUS:  FAILED (%s)\n', result.reason);
    fprintf('  Day:     %d / %d\n', result.day, cfg.T_MAX);
    fprintf('  Z (partial): %d\n', result.Z);
end

% ---- 6. Export to result.xls ----
fprintf('\n--- Exporting result.xls ---\n');
log = result.log;
n = length(log.Day);

% Build table
Day   = log.Day;
Wthr  = log.Weather;
PosX  = log.PosX;
PosY  = log.PosY;
Node  = log.Node;
Action= log.Action;
Detail= log.Detail;
O_out = log.O;
H_out = log.H;
F_out = log.F;
M_out = log.M;
Z_out = log.Z;
Load  = log.Load;

T = table(Day, Wthr, PosX, PosY, Node, Action, Detail, ...
          O_out, H_out, F_out, M_out, Z_out, Load, ...
          'VariableNames', {'Day','Weather','PosX','PosY','Node', ...
          'Action','Detail','O','H','F','M','Z','Load'});

outpath = fullfile(pwd, 'result.xls');
writetable(T, outpath, 'Sheet', 'DailyLog');
fprintf('  Daily log written to result.xls (%d rows)\n', n);

% Summary sheet
Summary = table({cfg.names{cfg.N_B}}, {cfg.names{cfg.N_E}}, ...
    cfg.T_MAX, cfg.pN, result.arrived, result.day, result.Z, round(result.M), ...
    {result.reason}, ...
    'VariableNames', {'Start','End','MaxDays','P_Normal','Arrived', ...
    'ArrivalDay','FinalZ','FinalM','FailReason'});
writetable(Summary, outpath, 'Sheet', 'Summary', 'WriteMode', 'overwritesheet');
fprintf('  Summary written to result.xls\n');

fprintf('\nDone. See result.xls for full output.\n');
end
