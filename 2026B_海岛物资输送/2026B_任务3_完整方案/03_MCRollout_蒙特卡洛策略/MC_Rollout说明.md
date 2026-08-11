<!-- 任务3 MDP Rollout方案说明 (v2.0 修复版) -->

# 任务3 MDP Rollout方案说明 (v2.0)

## 修复内容
| 修复项 | 原问题 | v2.0 修复 |
|------|------|------|
| Rollout决策 | avgZ排序后重新从cur_pt做CP，忽略选定候选 | **plan_from_candidate(cur_pt, target_pt) 确保计划执行选定动作** |
| 轨迹模拟 | simulate_one_trajectory无补给、硬编码消耗 | **替换为 cp_engine_opt('simulate_with_weather') 含完整补给** |
| addpath | 硬编码绝对路径 | **fileparts(mfilename)相对路径** |
| 补给需求 | 未传completed_wp | **传入 wp_idx 正确索引** |
| 重规划周期 | 每5天(文档) | **统一为每5天** |

## MDP Rollout 决策流程 (修复后)
```
对每个候选 np ∈ {W1,W2,W3,S1,S2}:
  1. plan_from_candidate(cur_pt, np): 强制下一站=np, CP规划剩余
  2. simulate_with_weather(full_plan, K条天气): K次随机天气模拟
  3. 计算 avgZ = totalZ / K
选择 avgZ 最大的 np*, 返回对应完整计划 (cur_pt→np*→...)
```

## 使用方法
```matlab
solve_q3_mcr('offline')       % 离线CP (复用优化版)
solve_q3_mcr('online')        % 在线MDP决策 (K=20)
solve_q3_mcr('online_k', 50)  % 在线MDP (自定义K=50)
solve_q3_mcr('mc', 100)       % MC验证 (N=100, K=20)
```

*修复日期: 2026-07-20*
