function solve_q3_v2(weather_seq)
% SOLVE_Q3_V2  Complete Problem 3 solver (refactored).

fprintf('========================================\n');
fprintf('  Problem 3: Optimized W3-Centric Policy\n');
fprintf('  30x30 Grid | 90 Days | p(N)=0.8\n');
fprintf('========================================\n\n');

cfg = params_q3_v2();

if nargin < 1 || isempty(weather_seq)
    rng(42);
    weather_seq = rand(1, cfg.T_MAX) < cfg.pN;
end

fprintf('--- Running Simulation ---\n');
tic;
result = sim_q3_v2(cfg, weather_seq);
fprintf('  Time: %.1fs\n\n', toc);

fprintf('========================================\n');
fprintf('  Results\n');
fprintf('========================================\n');
if result.arrived
    fprintf('  STATUS:  ARRIVED at E\n');
    fprintf('  Day:     %d / %d\n', result.day, cfg.T_MAX);
    fprintf('  Z:       %d  (initial 200, net +%d)\n', result.Z, result.Z - 200);
    fprintf('  M:       %.0f  (initial 750)\n', result.M);
else
    fprintf('  STATUS:  FAILED (%s)\n', result.reason);
    fprintf('  Day:     %d / %d\n', result.day, cfg.T_MAX);
    fprintf('  Z (partial): %d\n', result.Z);
end

fprintf('\n--- Exporting result.xls ---\n');
log = result.log;
n = length(log.Day);

Day    = log.Day;
Wthr   = log.Weather;
PosX   = log.PosX;
PosY   = log.PosY;
Node   = log.Node;
Action = log.Action;
Detail = log.Detail;
O_out  = log.O;
H_out  = log.H;
F_out  = log.F;
M_out  = log.M;
Z_out  = log.Z;
Load   = log.Load;

T = table(Day, Wthr, PosX, PosY, Node, Action, Detail, ...
          O_out, H_out, F_out, M_out, Z_out, Load, ...
          'VariableNames', {'Day','Weather','PosX','PosY','Node', ...
          'Action','Detail','O','H','F','M','Z','Load'});

outpath = fullfile(pwd, 'result_v2.xls');
writetable(T, outpath, 'Sheet', 'DailyLog');
fprintf('  Daily log: %d rows -> result_v2.xls\n', n);

Summary = table({cfg.names{cfg.N_B}}, {cfg.names{cfg.N_E}}, ...
    cfg.T_MAX, cfg.pN, result.arrived, result.day, result.Z, round(result.M), ...
    {result.reason}, ...
    'VariableNames', {'Start','End','MaxDays','P_Normal','Arrived', ...
    'ArrivalDay','FinalZ','FinalM','FailReason'});
writetable(Summary, outpath, 'Sheet', 'Summary', 'WriteMode', 'overwritesheet');
fprintf('  Summary -> result_v2.xls\n');

fprintf('\nDone.\n');
end
