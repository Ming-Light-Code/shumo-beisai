# 2026B 海岛物资输送 — 任务3 完整方案 (v2.0 修复版)

## 文件夹结构
```
2026B_任务3_完整方案/
├── README.md                          # 本文件
├── 01_原版_期望值CP/                   # 初始版本 (已废弃)
├── 02_优化版_改进CP/                   # ★ 推荐使用 (v2.0)
└── 03_MCRollout_蒙特卡洛策略/          # 研究用途
```

## 快速使用
```matlab
% 离线CP搜索
cd(''02_优化版_改进CP''); solve_q3_cp_opt
% 在线随机决策 (单次)
rng(42); solve_q3_online_opt
% 蒙特卡洛验证 (N=500)
rng(12345); solve_q3_montecarlo_opt(500)
% MDP Rollout
solve_q3_mcr(''online'')
```

## 三版对比
| 维度 | 原版 | 优化版(v2.0)★ | MCRollout(v2.0) |
|------|:---:|:---:|:---:|
| 离线最优Z | 480 | 576 | 557 (复用) |
| 路径 | B→W1→W3→S2→W3→S2→E | B→W1→S1→S2→W3→S2→E | 动态决策 |
| 安全裕度 | O/H/F 均1.10 | O×1.25, H/F×1.05 | 继承优化版 |

## v2.0 修复要点
| 优先级 | 问题 | 修复 |
|:---:|------|------|
| P0 | get_supply_needs索引错位 | completed_wp参数跳过已完成工作点 |
| P0 | MDP Rollout决策未执行 | plan_from_candidate + simulate_with_weather |
| P1 | plan_scenario评分丢弃 | 三场景加权 + 悲观回退 + 直赴E比较 |
| P1 | simulate轨迹无补给 | 统一使用cp_engine_opt.simulate_with_weather |
| P1 | addpath硬编码 | fileparts(mfilename)相对路径 |
| P2 | 文档/代码参数同步 | SAFETY_O=1.25, 生存阈值1.5×, 重规划每5天 |
| P3 | MC N=100→500 | 提升统计可靠性 |

*整理日期: 2026-07-20 | v2.0 修复版*
