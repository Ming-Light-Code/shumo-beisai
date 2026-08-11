import os, sys
desktop = os.path.join(os.environ['USERPROFILE'], 'Desktop')

cg_doc = """# 列生成方法求解海岛物资输送问题

## 一、算法概述

列生成（Column Generation）是一种用于求解大规模线性/整数规划的分解算法。
其核心思想是：当问题的决策变量（"列"）数量极其庞大时，不一次性生成所有列，
而是在求解过程中根据需要动态生成能改进目标函数的列。

对于本题，每一"列"代表一条完整的航行方案——包含路径序列、每日工作天数、
各补给站的采购量。所有可行方案构成集合 R。主问题从 R 中恰好选择一条路径。
"""

cp_doc = """# 约束规划方法求解海岛物资输送问题

## 一、算法概述

约束规划（Constraint Programming, CP）是一种用于求解组合优化问题的编程范式。
与数学规划（LP/MILP）依赖线性松弛和目标函数梯度不同，CP 通过约束传播 +
系统搜索在离散决策变量的有限值域上寻找可行解（并优化目标）。
"""

# Try writing with explicit Unicode paths
for folder_name, doc_name, content in [
    ('\u5217\u751f\u6210', '\u5217\u751f\u6210\u65b9\u6cd5\u8bf4\u660e.md', cg_doc),
    ('\u7ea6\u675f\u89c4\u5212', '\u7ea6\u675f\u89c4\u5212\u65b9\u6cd5\u8bf4\u660e.md', cp_doc)
]:
    folder_path = os.path.join(desktop, folder_name)
    os.makedirs(folder_path, exist_ok=True)
    file_path = os.path.join(folder_path, doc_name)
    with open(file_path, 'w', encoding='utf-8', newline='\r\n') as f:
        f.write(content)
    print(f'Written: {file_path}')

print('Done.')