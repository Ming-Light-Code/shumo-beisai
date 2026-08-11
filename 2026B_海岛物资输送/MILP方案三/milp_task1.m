function [best_Z, best_M, best_path, best_wdays] = milp_task1()
% 方案三：MILP 完整实现
% 骨架全枚举（5^k） + 事件驱动 MILP 求解各骨架
% 依赖：MATLAB Optimization Toolbox (intlinprog)

CM = [2, 3, 2];   CW = [5, 4, 3];
WY = [20, 15, 28]; WM = [4, 5, 3];

all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
dist = zeros(7);
for i = 1:7, for j = 1:7
    dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
end; end

inter_idx = [3 4 5 6 7];
max_seq = min(7, 30 - dist(1,2));

best_Z = -inf; best_M = -inf; best_path = []; best_wdays = [];

for seq_len = 0:max_seq
    n_seqs = 5^seq_len;
    for si = 1:n_seqs
        seq = zeros(1, seq_len); tmp = si - 1;
        for j = seq_len:-1:1
            seq(j) = mod(tmp, 5) + 1; tmp = floor(tmp / 5);
        end
        pid = [1, inter_idx(seq), 2];
        has_dup = false;
        for k = 2:length(pid)
            if pid(k) == pid(k-1), has_dup = true; break; end
        end
        if has_dup, continue; end

        m = length(pid) - 2;
        travel = zeros(1,m+1); total_travel = 0;
        for k = 1:(m+1)
            travel(k) = dist(pid(k), pid(k+1));
            total_travel = total_travel + travel(k);
        end
        if total_travel > 30, continue; end

        work_idx = zeros(1,m+1); work_which = [];
        supp_idx = zeros(1,m+1);
        n_work = 0; n_supply = 0;
        for k = 1:(m+1)
            pt = pid(k+1);
            if pt >= 3 && pt <= 5
                n_work = n_work + 1;
                work_idx(k) = n_work;
                work_which(n_work) = pt - 2;
            end
            if pt == 6 || pt == 7
                n_supply = n_supply + 1;
                supp_idx(k) = n_supply;
            end
        end

        [feasible, Z, M, w_opt] = milp_skeleton(...
            m, travel, work_idx, work_which, supp_idx, ...
            n_work, n_supply, total_travel, WY, WM, CM, CW);

        if feasible && (Z > best_Z || (Z == best_Z && M > best_M))
            best_Z = Z; best_M = M;
            best_path = pid; best_wdays = w_opt;
        end
    end
end

fprintf('MILP 最优解: Z=%d, M=%d\n', best_Z, best_M);
if ~isempty(best_path)
    names = {'B','E','W1','W2','W3','S1','S2'};
    fprintf('路径: ');
    fprintf('%s ', names{best_path});
    fprintf('\n');
    if ~isempty(best_wdays)
        fprintf('工作天数: '); fprintf('%d ', best_wdays); fprintf('\n');
    end
end
end

% ========== 事件驱动 MILP 求解器 ==========
function [feasible, Z_opt, M_opt, w_opt] = milp_skeleton(...
    m, travel, work_idx, work_which, supp_idx, ...
    n_work, n_supply, total_travel, WY, WM, CM, CW)

% 变量布局:
%   w(1:n_work)           [整数]
%   buyO(1:n_supply)      [连续]
%   buyH(1:n_supply)      [连续]
%   buyF(1:n_supply)      [连续]
%   O(0:m+1) H(0:m+1) F(0:m+1) M(0:m+1)   [连续]

n_vars = n_work + 3*n_supply + 4*(m+2);
intvars = 1:n_work;

off_w  = 0;
off_bO = n_work;
off_bH = n_work + n_supply;
off_bF = n_work + 2*n_supply;
off_O  = n_work + 3*n_supply;
off_H  = off_O + (m+2);
off_F  = off_H + (m+2);
off_M  = off_F + (m+2);

% ---- 等式约束: 4*(m+1) 个 ----
n_eq = 4*(m+1);
Aeq = zeros(n_eq, n_vars); beq = zeros(n_eq, 1);
eq = 0;

for i = 1:(m+1)
    d = travel(i);
    widx = work_idx(i);
    sidx = supp_idx(i);

    % O 平衡: O_i - O_{i-1} + w*CW_O - buyO = -d*CM_O
    eq = eq + 1;
    Aeq(eq, off_O+1+i) = 1;   Aeq(eq, off_O+i) = -1;
    beq(eq) = -d * CM(1);
    if widx > 0, Aeq(eq, off_w + widx) = CW(1); end
    if sidx > 0, Aeq(eq, off_bO + sidx) = -1; end

    % H 平衡: H_i - H_{i-1} + w*CW_H - buyH = -d*CM_H
    eq = eq + 1;
    Aeq(eq, off_H+1+i) = 1;   Aeq(eq, off_H+i) = -1;
    beq(eq) = -d * CM(2);
    if widx > 0, Aeq(eq, off_w + widx) = CW(2); end
    if sidx > 0, Aeq(eq, off_bH + sidx) = -1; end

    % F 平衡: F_i - F_{i-1} + w*CW_F - buyF = -d*CM_F
    eq = eq + 1;
    Aeq(eq, off_F+1+i) = 1;   Aeq(eq, off_F+i) = -1;
    beq(eq) = -d * CM(3);
    if widx > 0, Aeq(eq, off_w + widx) = CW(3); end
    if sidx > 0, Aeq(eq, off_bF + sidx) = -1; end

    % M 平衡: M_i - M_{i-1} + 2*buyO + buyH + 2*buyF = 0
    eq = eq + 1;
    Aeq(eq, off_M+1+i) = 1;   Aeq(eq, off_M+i) = -1;
    beq(eq) = 0;
    if sidx > 0
        Aeq(eq, off_bO + sidx) = 2;
        Aeq(eq, off_bH + sidx) = 1;
        Aeq(eq, off_bF + sidx) = 2;
    end
end

% ---- 不等式约束 ----
n_ineq = (m+2) + 1 + n_work + 3*n_supply;
if n_ineq > 0
    A = zeros(n_ineq, n_vars); b = zeros(n_ineq, 1);
else
    A = []; b = [];
end
ineq = 0;

% 载重: O_i + H_i + F_i ≤ 120, i=0..m+1
for i = 0:(m+1)
    ineq = ineq + 1;
    A(ineq, off_O+1+i) = 1;
    A(ineq, off_H+1+i) = 1;
    A(ineq, off_F+1+i) = 1;
    b(ineq) = 120;
end

% 总天数: Σw ≤ 30 - total_travel
ineq = ineq + 1;
for j = 1:n_work, A(ineq, off_w+j) = 1; end
b(ineq) = 30 - total_travel;

% 工作上限: w_j ≤ WM(work_which(j))
for j = 1:n_work
    ineq = ineq + 1;
    A(ineq, off_w+j) = 1;
    b(ineq) = WM(work_which(j));
end

% 补给站到达前资源非负: O_{i-1} ≥ d*CM (否则段内会耗尽)
for i = 1:(m+1)
    if supp_idx(i) > 0
        d = travel(i);
        ineq = ineq+1; A(ineq, off_O+i) = -1; b(ineq) = -d*CM(1);
        ineq = ineq+1; A(ineq, off_H+i) = -1; b(ineq) = -d*CM(2);
        ineq = ineq+1; A(ineq, off_F+i) = -1; b(ineq) = -d*CM(3);
    end
end

% ---- 边界 ----
lb = zeros(n_vars, 1); ub = inf(n_vars, 1);
lb(off_O+1) = 35;  ub(off_O+1) = 35;
lb(off_H+1) = 45;  ub(off_H+1) = 45;
lb(off_F+1) = 30;  ub(off_F+1) = 30;
lb(off_M+1) = 240; ub(off_M+1) = 240;

opts = optimoptions('intlinprog', 'Display', 'off');

% ---- 阶段 1: max Z ----
f1 = zeros(n_vars, 1);
for j = 1:n_work, f1(off_w+j) = -WY(work_which(j)); end

if isempty(intvars)
    [x1, fval1, flag] = linprog(f1, A, b, Aeq, beq, lb, ub, opts);
else
    [x1, fval1, flag] = intlinprog(f1, intvars, A, b, Aeq, beq, lb, ub, opts);
end
if flag <= 0 || isempty(x1)
    feasible = false; Z_opt = 0; M_opt = 0; w_opt = []; return;
end
Z_opt = 100 + round(-fval1);

% ---- 阶段 2: 固定 Z, max M ----
if n_work > 0
    A2 = [A; zeros(2, n_vars)];
    b2 = [b; Z_opt - 100; -(Z_opt - 100)];
    nr = size(A, 1);
    for j = 1:n_work
        A2(nr+1, off_w+j) =  WY(work_which(j));
        A2(nr+2, off_w+j) = -WY(work_which(j));
    end
else
    A2 = A; b2 = b;
end

f2 = zeros(n_vars, 1);
f2(off_M + m + 2) = -1;  % M_{m+1} 在 off_M + 1 + (m+1) = off_M + m + 2

if isempty(intvars)
    [x2, fval2, flag2] = linprog(f2, A2, b2, Aeq, beq, lb, ub, opts);
else
    [x2, fval2, flag2] = intlinprog(f2, intvars, A2, b2, Aeq, beq, lb, ub, opts);
end
if flag2 <= 0 || isempty(x2)
    feasible = false; Z_opt = 0; M_opt = 0; w_opt = []; return;
end

M_opt = round(-fval2);
w_opt = round(x2(off_w + (1:n_work)));
feasible = true;
end
