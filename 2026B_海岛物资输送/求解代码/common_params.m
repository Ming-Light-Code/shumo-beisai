% common_params.m - Shared parameters for Task 1
% 2026 SEU Math Modeling Competition, Problem B

% ========== Global declarations ==========
global B E S W O0 H0 F0 M0 Z0 LOAD_LIMIT MAX_DAYS PRICE
global CONSUME_MOVE CONSUME_STAY CONSUME_WORK WORK_YIELD WORK_MAX_CONSEC
global INTERMEDIATE_PTS N_INTERMEDIATE manhattan

% ========== Grid and Points ==========
B = [1, 5];
E = [10, 5];
S = {[3, 4], [7, 6]};
W = {[2, 7], [5, 3], [8, 8]};

% ========== Initial Resources ==========
O0 = 35;
H0 = 45;
F0 = 30;
M0 = 240;
Z0 = 100;

% ========== Constraint Parameters ==========
LOAD_LIMIT = 120;
MAX_DAYS = 30;
PRICE = [2, 1, 2];
CONSUME_MOVE = [2, 3, 2];
CONSUME_STAY = [1, 1, 1];
CONSUME_WORK = [5, 4, 3];

% ========== Work Parameters ==========
WORK_YIELD = [20, 15, 28];
WORK_MAX_CONSEC = [4, 5, 3];

% ========== Intermediate Points Enumeration ==========
INTERMEDIATE_PTS = [
    2, 7, 1, 1;
    5, 3, 1, 2;
    8, 8, 1, 3;
    3, 4, 2, 1;
    7, 6, 2, 2;
];

N_INTERMEDIATE = size(INTERMEDIATE_PTS, 1);

% ========== Helper ==========
manhattan = @(a, b) abs(a(1)-b(1)) + abs(a(2)-b(2));
