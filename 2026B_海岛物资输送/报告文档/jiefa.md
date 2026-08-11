# 三种方法详细讲解与代码

---

## 一、枚举 + 贪婪前向模拟

### 数学建模

将问题建模为三层嵌套优化：

**外层（路径骨架）**：设中间点集合 $\mathcal{P} = \{W_1, W_2, W_3, S_1, S_2\}$，路径骨架为序列 $P = [B, p_1, p_2, \ldots, p_k, E]$，其中 $p_i \in \mathcal{P}$ 且 $p_i \neq p_{i+1}$（去除相邻重复）。$k \in [0, K_{\max}]$。

**中层（工作天数）**：对路径中每个作业点 $w_j$（按出现顺序编号 $j=1,\ldots,n_w$），决策工作天数 $d_j \in [0, D_j^{\max}]$，其中 $D_j^{\max}$ 为该作业点的最大连续作业天数。

**内层（贪婪采购）**：给定骨架和工作天数，日程完全确定。在补给日 $t$，设当前资源为 $(O_t, H_t, F_t)$，下一个补给日为 $t_{\text{next}}$，需求为：

$$\text{need} = \sum_{\tau=t+1}^{t_{\text{next}}} \mathbf{c}(\tau)$$

其中 $\mathbf{c}(\tau) \in \{(2,3,2), (5,4,3)\}$ 为第 $\tau$ 天的消耗向量。采购量：

$$\mathbf{buy} = \max(\mathbf{0}, \mathbf{need} - \mathbf{current})$$

约束：
- 载重：$\sum\text{buy} \le 120 - (O_t + H_t + F_t)$
- 资金：$2 \cdot \text{buy}_O + 1 \cdot \text{buy}_H + 2 \cdot \text{buy}_F \le M_t$

这个策略等价于求解微型 LP（3 变量、4 约束），在恒定消耗速率下得到最优采购量。

### 算法伪代码

```
输入：地图、初始资源、约束参数
输出：最优 (Z, M, 路径, 工作天数)

best_Z ← -∞, best_M ← -∞
for 每条路径骨架 P (B→中间点序列→E, 无相邻重复):
    total_travel ← Σ相邻点间曼哈顿距离
    if total_travel > 30: continue
    
    识别 P 中的作业点序列 work_at[1..n_w]
    
    for 每组工作天数 d[1..n_w] ∈ [0, max_consec]:
        if total_travel + Σd > 30: continue
        
        (feasible, Z, M) ← 贪婪前向模拟(P, work_at, d)
        if feasible:
            if Z > best_Z or (Z == best_Z and M > best_M):
                更新最优解

返回最优解
```

### 贪婪前向模拟子过程

```
输入：(路径 P, 工作天数 d)
输出：(feasible, Z, M)

构建逐日日程表: 每格填充消耗(cO,cH,cF)、收益(zGain)、是否补给日(isSup)

O←35, H←45, F←30, M←240  // 初始资源

for t = 1 to T:           // 逐日推进
    if isSup[t]:
        找到下一个补给日 nextSup
        计算 need = Σ_{τ=t+1}^{nextSup} 消耗
        buy = max(0, need - (O,H,F))
        检查: buy总量 ≤ 120 - (O+H+F)
        检查: 若 nextSup 是终点(>T), 必须满足 need
        检查: cost ≤ M
        (O,H,F) += buy
        M -= cost
    else:
        (O,H,F) -= 当日消耗
    
    检查: O≥0, H≥0, F≥0, M≥0, O+H+F≤120

Z = 100 + ΣzGain
M = M_final
```

### MATLAB 代码

```matlab
function task1_solution()
% 桌面文件: C:\Users\ming\Desktop\task1_solution.m
% 运行: 在MATLAB中直接输入 task1_solution

% ========== 参数 ==========
B = [1, 5];  E = [10, 5];
CM = [2, 3, 2];   CW = [5, 4, 3];   % 消耗率
WY = [20, 15, 28];  WM = [4, 5, 3];   % 作业收益/上限

% 7个点位坐标: B=1, E=2, W1=3, W2=4, W3=5, S1=6, S2=7
all_xy = [1 5; 10 5; 2 7; 5 3; 8 8; 3 4; 7 6];
dist = zeros(7);
for i = 1:7, for j = 1:7
    dist(i,j) = abs(all_xy(i,1)-all_xy(j,1)) + abs(all_xy(i,2)-all_xy(j,2));
end; end

inter_idx = [3 4 5 6 7];  % W1,W2,W3,S1,S2
max_seq = min(7, 30 - dist(1,2));

best_Z = -inf; best_M = -inf; best_path = []; best_wd = [];

% ========== 外层: 枚举路径骨架 ==========
for seq_len = 0:max_seq
    n_seqs = 5^seq_len;
    for si = 1:n_seqs
        seq = zeros(1, seq_len); tmp = si - 1;
        for j = seq_len:-1:1
            seq(j) = mod(tmp, 5) + 1; tmp = floor(tmp / 5);
        end
        
        pid = [1, inter_idx(seq), 2];        % B -> 中间点 -> E
        
        % 去除相邻重复点 (如 W3→W3)
        has_dup = false;
        for k = 2:length(pid)
            if pid(k) == pid(k-1), has_dup = true; break; end
        end
        if has_dup, continue; end
        
        m = length(pid) - 2;                  % 中间点数量
        
        % 计算旅行距离
        travel = zeros(1,m+1); total_travel = 0;
        for k = 1:(m+1)
            travel(k) = dist(pid(k), pid(k+1));
            total_travel = total_travel + travel(k);
        end
        if total_travel > 30, continue; end
        
        % 识别作业点
        work_at = []; work_wh = [];
        for k = 2:(m+1)
            pt = pid(k);
            if pt >= 3 && pt <= 5
                work_at(end+1) = k; work_wh(end+1) = pt-2;
            end
        end
        n_work = length(work_at);
        
        % ========== 中层: 枚举工作天数 ==========
        if n_work == 0
            [ok,Z,M] = greedy_sim(pid,m,travel,work_at,work_wh,total_travel,[],WY,CM,CW);
            if ok && (Z>best_Z||(Z==best_Z&&M>best_M))
                best_Z=Z; best_M=M; best_path=pid;
            end
        else
            sizes = WM(work_wh) + 1;
            n_combos = prod(sizes);
            for ci = 1:n_combos
                wdays = zeros(1,n_work); tmp2 = ci - 1;
                for j = n_work:-1:1
                    wdays(j) = mod(tmp2, sizes(j));
                    tmp2 = floor(tmp2 / sizes(j));
                end
                if total_travel + sum(wdays) > 30, continue; end
                
                % ========== 内层: 贪婪模拟 ==========
                [ok,Z,M] = greedy_sim(pid,m,travel,work_at,work_wh,...
                    total_travel,wdays,WY,CM,CW);
                if ok && (Z>best_Z||(Z==best_Z&&M>best_M))
                    best_Z=Z; best_M=M; best_path=pid; best_wd=wdays;
                end
            end
        end
    end
end

% 输出结果
fprintf('Z=%d, M=%d\n', best_Z, best_M);
% ... (路径输出略)
end

% ========== 贪婪前向模拟 ==========
function [feasible, Z_final, M_final] = greedy_sim(pid, m, travel, ...
    work_at, work_wh, total_travel, wdays, WY, CM, CW)

    T = total_travel + sum(wdays);       % 总天数
    cO = zeros(1,T); cH = zeros(1,T); cF = zeros(1,T);
    zG = zeros(1,T); isSup = false(1,T);
    
    % --- 构建逐日日程表 ---
    day = 0;
    for k = 1:(m+1)
        d = travel(k);
        for dd = 1:d
            day = day + 1;
            cO(day)=CM(1); cH(day)=CM(2); cF(day)=CM(3);  % 移动消耗
            if dd == d
                to_pt = pid(k+1);
                if to_pt == 6 || to_pt == 7   % S1 或 S2
                    isSup(day) = true;         % 标记补给日
                end
            end
        end
        % 作业日
        wk = find(work_at == k+1, 1);
        if ~isempty(wk) && wdays(wk) > 0
            for w = 1:wdays(wk)
                day = day + 1;
                cO(day)=CW(1); cH(day)=CW(2); cF(day)=CW(3);
                zG(day) = WY(work_wh(wk));
            end
        end
    end
    
    % --- 逐日推进 ---
    O=35; H=45; F=30; M=240;
    
    for t = 1:T
        if isSup(t)
            % 找到下一个补给日
            nextSup = T + 1;
            for tt = t+1:T
                if isSup(tt), nextSup = tt; break; end
            end
            
            % 计算未来需求 (t+1 到 nextSup，含 nextSup 到达消耗)
            needO = 0; needH = 0; needF = 0;
            for tt = t+1:nextSup
                if tt > T, break; end
                needO = needO + cO(tt);
                needH = needH + cH(tt);
                needF = needF + cF(tt);
            end
            
            space = 120 - (O + H + F);
            buyO = max(0, needO - O);
            buyH = max(0, needH - H);
            buyF = max(0, needF - F);
            
            % 可行性检查
            if buyO+buyH+buyF > space
                feasible=false; Z_final=0; M_final=0; return;
            end
            if nextSup > T  % 这是最后一个补给站
                if O+buyO < needO || H+buyH < needH || F+buyF < needF
                    feasible=false; Z_final=0; M_final=0; return;
                end
            end
            
            cost = buyO*2 + buyH*1 + buyF*2;
            if cost > M
                feasible=false; Z_final=0; M_final=0; return;
            end
            
            O=O+buyO; H=H+buyH; F=F+buyF; M=M-cost;
        else
            O=O-cO(t); H=H-cH(t); F=F-cF(t);
        end
        
        if O<0||H<0||F<0||M<0, feasible=false; Z_final=0; M_final=0; return; end
        if O+H+F>120, feasible=false; Z_final=0; M_final=0; return; end
    end
    
    Z_final = 100 + sum(zG);
    M_final = M;
    feasible = true;
end
```

### 关键设计决策

1. **去除相邻重复点**：`W3→W3` 的距离为 0，会绕过连续作业上限约束（允许在 W3 连续作业 6 天而只计为两次 3 天），必须过滤。

2. **need 计算包含下一个补给日的消耗**（`nextSup` 而非 `nextSup-1`）：因为补给日的到达消耗由**前一个补给站的购买**覆盖，补给日本身不应再次扣除。

3. **贪婪购买的全局最优性**：在恒定消耗速率下，多买资源只会占用载重和资金而无收益。买"刚好够"即为最优。

---

## 二、DP（动态规划，单段转移版）

### 数学建模

将问题定义为在**有向图**上的最优路径问题。节点为 7 个特殊点位 $\{B, E, W_1, W_2, W_3, S_1, S_2\}$。状态为 $(\text{mask}, \text{last})$：

- $\text{mask} \in [0, 2^5)$：已访问中间点的位掩码（bit 0→W1, bit 1→W2, bit 2→W3, bit 3→S1, bit 4→S2）
- $\text{last} \in \{1,\ldots,7\}$：当前所在点

状态转移：

$$\text{new\_mask} = \text{mask} \ |\ (1 \ll (next-2)), \quad next \notin \text{visited}$$

每段转移 `last→next` 的收益由贪婪模拟评估。

### 算法伪代码

```
初始化 DP 表:
  dp_Z[1][1] = 100    (mask=0, 在B点)
  dp_M[1][1] = 240
  其他 = -∞

for mask = 1 to 2^5:
    for last = 1 to 7:
        if dp_Z[mask][last] < 0: continue
        
        % 尝试直接去终点
        [feas, Z, M] ← 评估 last→E 段
        if feas: 更新全局最优
        
        % 尝试去每个未访问的中间点
        for next = W1, W2, W3, S1, S2:
            if next 已在 mask 中: continue
            new_mask = mask | (1 << (next-2))
            [feas, Z, M] ← 评估 last→next 段
            if feas:
                更新 dp_Z[new_mask][next], dp_M[new_mask][next]

返回全局最优
```

### MATLAB 代码

```matlab
function [best_Z, best_M, best_path, best_wd] = method_dp(dist, all_xy, ...
    inter_idx, n_inter, max_seq, WY, WM, CM, CW)
    % 单段转移 DP（当前简化版）

    N_MASKS = 2^5;
    dp_Z = -inf(N_MASKS, 7);  dp_M = -inf(N_MASKS, 7);
    dp_Z(1,1) = 100;  dp_M(1,1) = 240;   % 初始状态：在B
    
    best_Z = -inf;  best_M = -inf;  best_path = [1,2];
    
    for mask = 1:N_MASKS
        for last = 1:7
            if dp_Z(mask,last) < 0, continue; end
            
            % 转移1: 直接去终点 E
            dE = dist(last, 2);
            if dE <= 30
                pid = [last, 2];
                [ok, Z, M] = greedy_sim(pid, 0, dE, [], [], dE, [], WY, CM, CW);
                if ok && (Z>best_Z || (Z==best_Z && M>best_M))
                    best_Z = Z;  best_M = M;  best_path = [last, 2];
                end
            end
            
            % 转移2: 去未访问的中间点
            for nxt = 3:7   % W1,W2,W3,S1,S2
                pt_bit = nxt - 2;
                if bitand(mask, bitshift(1, pt_bit-1)), continue; end
                
                new_mask = bitor(mask, bitshift(1, pt_bit-1));
                d = dist(last, nxt);
                pid = [last, nxt];
                
                [ok, Z, M] = greedy_sim(pid, 0, d, [], [], d, [], WY, CM, CW);
                if ok
                    if Z > dp_Z(new_mask,nxt) || ...
                       (Z == dp_Z(new_mask,nxt) && M > dp_M(new_mask,nxt))
                        dp_Z(new_mask,nxt) = Z;
                        dp_M(new_mask,nxt) = M;
                    end
                end
            end
        end
    end
end
```

### 当前局限与改进方向

当前实现只做**一步转移**（`last→next`），没有递归累积路径。结果只有 B→E 的平凡解（Z=100）。

**改进为多段 DP**：将状态转移改为递归调用，累积多段路径的收益。伪代码：

```
function dfs(mask, last, path_so_far, Z_so_far, M_so_far):
    % 尝试去 E
    [feas, Z, M] ← 评估 last→E 段（使用 path_so_far 的累积资源）
    if feas: 更新全局最优
    
    % 尝试去每个未访问点
    for next in 未访问点:
        [feas, Z, M] ← 评估 last→next 段
        if feas:
            dfs(new_mask, next, path+[next], Z, M)
```

状态空间为 $2^5 \times 7 = 224$，完全可行。但需注意资源状态是路径累积的，不能简单用 dp 表存单一 (Z,M)——需要存 Pareto 前沿或保留完整的 (O,H,F,M,Z) 向量。

---

## 三、MILP（混合整数线性规划）

### 数学建模

**决策变量**（对给定骨架）：

- $w_j \in \mathbb{Z}_{\ge 0}$：第 $j$ 个作业点的工作天数，$w_j \le D_j^{\max}$
- $\text{buy}_{O,k}, \text{buy}_{H,k}, \text{buy}_{F,k} \in \mathbb{R}_{\ge 0}$：第 $k$ 个补给站的采购量
- $O_i, H_i, F_i, M_i$：第 $i$ 个停靠点后的资源状态

**事件转移方程**（从停靠点 $i-1$ 到 $i$）：

$$\begin{aligned}
O_i &= O_{i-1} - d_i \cdot \text{CM}_O - w_i \cdot \text{CW}_O + \text{buy}_{O,i} \\
H_i &= H_{i-1} - d_i \cdot \text{CM}_H - w_i \cdot \text{CW}_H + \text{buy}_{H,i} \\
F_i &= F_{i-1} - d_i \cdot \text{CM}_F - w_i \cdot \text{CW}_F + \text{buy}_{F,i} \\
M_i &= M_{i-1} - (2 \cdot \text{buy}_{O,i} + 1 \cdot \text{buy}_{H,i} + 2 \cdot \text{buy}_{F,i})
\end{aligned}$$

其中 $d_i$ 为两点间曼哈顿距离（常量），$w_i$ 仅在作业点非零。

**约束**：

$$\begin{aligned}
O_i, H_i, F_i, M_i &\ge 0 \quad \forall i \\
O_i + H_i + F_i &\le 120 \quad \forall i \\
\sum d_i + \sum w_j &\le 30 \\
0 \le w_j &\le D_j^{\max}
\end{aligned}$$

**两阶段字典序优化**：

- 阶段 1：$\max Z = Z_0 + \sum w_j \cdot \text{yield}_j$
- 阶段 2：固定 $Z = Z_{\max}$，$\max M_{m+1}$

### MATLAB 代码（核心 MILP 构建函数）

```matlab
function [feasible, Z_opt, M_opt, w_opt] = milp_skeleton(m, travel, ...
    work_at, work_which, supply_at, n_work, n_supply, total_travel)

    % 变量布局:
    % w(1:n_work) [整数] | buy_O(1:n_supply) | buy_H | buy_F |
    % O(0:m+1) | H(0:m+1) | F(0:m+1) | M(0:m+1)
    
    n_vars = n_work + 3*n_supply + 4*(m+2);
    intvars = 1:n_work;
    
    % 变量偏移量
    off_w = 0;  off_buyO = n_work;  off_buyH = n_work + n_supply;
    off_buyF = n_work + 2*n_supply;
    off_O = n_work + 3*n_supply;  off_H = off_O + (m+2);
    off_F = off_H + (m+2);  off_M = off_F + (m+2);
    
    % ---- 等式约束 (4 × (m+1) 个) ----
    Aeq = zeros(4*(m+1), n_vars);  beq = zeros(4*(m+1), 1);
    eq_row = 0;
    
    for i = 1:(m+1)
        d = travel(i);  to_pt = i + 1;
        
        % O 平衡: O_i - O_{i-1} + w_i*CW_O - buy_O_i = -d*CM_O
        eq_row = eq_row + 1;
        Aeq(eq_row, off_O+i) = 1;  Aeq(eq_row, off_O+i-1) = -1;
        beq(eq_row) = -d * 2;
        if work_map(to_pt) > 0
            Aeq(eq_row, off_w + work_map(to_pt)) = 5;  % CW_O
        end
        if supply_map(to_pt) > 0
            Aeq(eq_row, off_buyO + supply_map(to_pt)) = -1;
        end
        % (H 和 F 类似，M 含价格项)
        % ...
    end
    
    % ---- 不等式约束 ----
    % 非负: -O_i ≤ 0, -H_i ≤ 0, -F_i ≤ 0, -M_i ≤ 0
    % 载重: O_i + H_i + F_i ≤ 120
    % 总天数: Σw_i ≤ 30 - total_travel
    % 工作上限: w_i ≤ WM
    
    % ---- 边界 ----
    lb = zeros(n_vars, 1);  ub = inf(n_vars, 1);
    lb(off_O+1) = 35;  ub(off_O+1) = 35;  % O_0 = 35
    % (H_0, F_0, M_0 同理)
    
    % ---- 阶段1: max Z ----
    f1 = zeros(n_vars, 1);
    for w = 1:n_work
        f1(off_w + w) = -WY(work_which(w));  % min -Z = max Z
    end
    [x1, fval1, exitflag] = intlinprog(f1, intvars, A, b, Aeq, beq, lb, ub);
    Z_opt = 100 + round(-fval1);  % Z0 = 100
    
    % ---- 阶段2: 固定 Z, max M ----
    % 添加约束: Σ w_i * yield_i = Z_opt - 100
    A2 = [A; zeros(2, n_vars)];
    b2 = [b; Z_opt - 100; -(Z_opt - 100)];
    f2 = zeros(n_vars, 1);
    f2(off_M + m + 1) = -1;  % min -M = max M
    
    [x2, fval2, exitflag] = intlinprog(f2, intvars, A2, b2, Aeq, beq, lb, ub);
    M_opt = round(-fval2);
end
```

### 事件驱动建模的优势

相比逐日 LP（需要 $7T$ 个变量，$T \le 30$），事件驱动 MILP 只需要 $n_w + 3n_s + 4(m+2)$ 个变量。对于 5 个中间点的骨架，约 40 个变量，求解极快（骨架枚举约 20,000 次，每次 MILP 在毫秒级）。

### 缺点

- 依赖 Optimization Toolbox（`intlinprog`）
- 骨架枚举未覆盖 W3→S2→W3 模式（可能被约束判定不可行），导致 Z=264 而非 288

---

