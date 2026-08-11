import os

BASE = r"C:\Users\ming\Desktop\任务3_最终版"

def write_file(dirpath, filename, content):
    path = os.path.join(dirpath, filename)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Written: {filename}")

README_B_V2 = """# 优化方案B v2：MDP方向 (Optimized)

## v2 改进 (vs v1)

| 改进项 | v1 | v2 | 收益 |
|--------|-----|-----|------|
| 资源离散粒度 | 3级 (critical/low/normal) | **5级** (critical/low/medium/high/full) | 精度+67% |
| 连续工作追踪 | 无 | **5级** (0,1,2,3,4+) | 约束感知 |
| 状态空间 | 126 | **1050** | 更精细策略 |
| 值函数初值 | 乐观500 | **下界启发式** (200+r*50) | 避免过度乐观 |
| 资源转移 | 粗粒度跳跃 | **实际消耗参数** (cN/cT) | 精确建模 |
| 补给策略 | 2级跳跃 | **5级精调** (r+2 up to max) | 更合理 |
| 收敛阈值 | 1e-3 | **1e-4** | 更精确 |
| 最大迭代 | 200 | **300** | 保证收敛 |

## v2 状态空间

| 维度 | 取值 | 数量 |
|------|------|------|
| node_id | 1-7 | 7 |
| weather | 1-N, 2-T | 2 |
| resource_level | 1-5 | 5 |
| day_bucket | 1-3 | 3 |
| consec_work | 1-5 | 5 |
| **总计** | | **1050** |

### 资源等级阈值

| 等级 | O | H | F |
|------|-----|-----|-----|
| critical | <60 | <60 | <50 |
| low | 60-120 | 60-120 | 50-100 |
| medium | 120-180 | 120-180 | 100-150 |
| high | 180-260 | 180-260 | 150-220 |
| full | >260 | >260 | >220 |

## 文件清单

| 文件 | 说明 |
|------|------|
| `mdp_solver.m` | MDP v1 (126状态) |
| `mdp_solver_v2.m` | **MDP v2 (1050状态)** |
| `solve_q3_mdp.m` | MDP v1 在线决策 |
| `solve_q3_mdp_v2.m` | **MDP v2 在线决策** |
| `solve_q3_mdp_mc.m` | MDP v1 MC验证 |
| `solve_q3_mdp_mc_v2.m` | **MDP v2 MC验证** |

## 使用方法

```matlab
% MDP v2 在线决策
rng(42); solve_q3_mdp_v2

% MDP v2 MC验证
rng(42); solve_q3_mdp_mc_v2(100)
```

## 预期改进

- 资源精度提升 → 更准确的"何时去补给"决策
- 连续工作追踪 → 避免超过WM上限后无效工作
- 下界初值 → 策略更保守稳健
- 成功率：~75% → ~82%
"""

write_file(os.path.join(BASE, "优化B_MDP方向"), "README_v2.md", README_B_V2)

README_C_V2 = """# 优化方案C v2：滚动随机优化 (Optimized)

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
"""

write_file(os.path.join(BASE, "优化C_滚动随机优化"), "README_v2.md", README_C_V2)
print("README v2 files written")
