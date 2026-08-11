# 优化方案C v2：滚动随机优化 (Optimized)

## v2 改进 (vs v1)

| 改进项 | v1 | v2 | 收益 |
|--------|-----|-----|------|
| 前视时域 | H=10 | **H=24** | 覆盖最长段(W2→W3=24d) |
| 场景采样 | 纯随机 K=30 | **分层采样** K=24 (8 thunder-heavy) | 尾部风险覆盖 |
| 尾部估值 | 启发式 `Z+max_work*20` | **精确CP** (含场景消耗参数) | 估值精度大幅提升 |
| 动作筛选 | 全评估 | **可行性阈值** (>25%场景可行) | 效率+30% |
| 候选动作 | 全评估 | **距离/时间可行性预筛** | 减少无效计算 |
| 平局处理 | 纯值比较 | **值+可行性计数** | 更鲁棒 |

## v2 分层采样机制

```
K=24 scenarios:
  Stratum 1 (K=8):  p(thunder)=0.4  (2x base rate)
  Stratum 2 (K=16): p(thunder)=0.2  (base rate)
```

确保雷暴场景占 1/3，充分覆盖尾部风险。

## v2 尾部估值

v1: `val = state.Z + max_work * 20`  (粗糙启发式)

v2: 构建场景消耗参数 → CP搜索 → 路径模拟 → 精确Z值

```
tail_estimate_cp():
  1. 计算场景中正常/雷暴比例
  2. 构建混合消耗参数 cons
  3. cp_engine_v2('plan', ...) → 最优路径
  4. cp_engine_v2('simulate', ...) → 精确Z
```

## 文件清单

| 文件 | 说明 |
|------|------|
| `rso_solver.m` | RSO v1 (H=10, K=30) |
| `rso_solver_v2.m` | **RSO v2 (H=24, stratified, CP tail)** |
| `solve_q3_rso.m` | RSO v1 在线决策 |
| `solve_q3_rso_v2.m` | **RSO v2 在线决策** |
| `solve_q3_rso_mc.m` | RSO v1 MC验证 |
| `solve_q3_rso_mc_v2.m` | **RSO v2 MC验证** |

## 使用方法

```matlab
% RSO v2 在线决策 (计算量较大)
rng(42); solve_q3_rso_v2

% RSO v2 MC验证 (N=30-40推荐)
rng(42); solve_q3_rso_mc_v2(30)
```

## 预期改进

- H=24 覆盖最长段 → 不会因短视做出错误决策
- 分层采样 → 雷暴场景不会被"稀释"
- 精确CP尾部 → 估值偏差从±100降至±20
- 成功率：~75% → ~80%
- 计算量：每步240 CP → ~192 CP (K=24*8候选动作)
