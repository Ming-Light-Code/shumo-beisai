% run_task1.m - Run MILP and DP methods for Task 1
% 2026 SEU Math Modeling Competition, Problem B

clear; clc;
fprintf('========================================\n');
fprintf('  Task 1: MILP and DP Methods\n');
fprintf('========================================\n\n');

% Load common parameters
common_params;

% Results storage
results = struct();

% ===== Method 1: MILP =====
fprintf('\n>>> Method 1: MILP (Mixed Integer Linear Programming)\n');
fprintf('-----------------------------------------------------\n');
tic;
[Z1, M1, sol1] = solve_milp();
t1 = toc;
results(1).name = 'MILP';
results(1).Z = Z1;
results(1).M = M1;
results(1).time = t1;

% ===== Method 2: DP (Dynamic Programming) =====
fprintf('\n>>> Method 2: DP (Dynamic Programming)\n');
fprintf('-----------------------------------------------------\n');
tic;
[Z2, M2, sol2] = solve_dp();
t2 = toc;
results(2).name = 'DP';
results(2).Z = Z2;
results(2).M = M2;
results(2).time = t2;

% ===== Summary =====
fprintf('\n========================================\n');
fprintf('            RESULTS SUMMARY\n');
fprintf('========================================\n');
fprintf('%-12s %8s %8s %10s\n', 'Method', 'Z', 'M', 'Time(s)');
fprintf('------------------------------------------\n');
for i = 1:2
    fprintf('%-12s %8d %8d %10.2f\n', ...
        results(i).name, results(i).Z, results(i).M, results(i).time);
end
fprintf('------------------------------------------\n');

% Display optimal solution
fprintf('\n===== OPTIMAL SOLUTION =====\n');
if ~isempty(sol1)
    fprintf('Target物资 Z = %d\n', sol1.Z);
    fprintf('Remaining M  = %d\n', sol1.M);
    if isfield(sol1, 'path')
        names = {'B','E','W1','W2','W3','S1','S2'};
        fprintf('Path: ');
        for i = 1:length(sol1.path)
            fprintf('%s ', names{sol1.path(i)});
        end
        fprintf('\n');
    end
end

fprintf('\nDone.\n');
