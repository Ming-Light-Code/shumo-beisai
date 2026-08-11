# -*- coding: utf-8 -*-
filepath = r"C:\Users\ming\Downloads\正文 (9).tex"
with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

Q = "\""  # ASCII double quote

# ============================================================
# FIX TRIM 4: 问题四-有效性 (18→10 lines)
# ============================================================
old4 = (
    "\subsubsection{模型与方法的有效性}\n"
    "\n"
    "本文提出的" + Q + "宏节点+MPC+联合MILP" + Q + "框架在处理双船协同随机动态规划问题上展现了多方面的有效性：\n"
    "\\begin{enumerate}\n"
    "    \\item \\textbf{维度压缩}：宏节点将变量规模从 $\\mathcal O(900\\times90\\times2)$ 压缩至"
    " $\\mathcal O(20\\times15\\times2)\\approx600$，使 MILP 可在 5 秒内求解；\n"
    "    \\item \\textbf{信息利用}：MPC 框架充分利用了" + Q + "每日可观测当日天气" + Q + "的增量信息，"
    "通过每日重优化及时修正因实际天气偏离期望导致的资源偏差；\n"
    "    \\item \\textbf{最优性保障}：字典序两阶段优化严格保证了" + Q + "先 $Z$ 后 $M$" + Q + "的优先级，"
    "不会出现加权和方法的帕累托偏离；\n"
    "    \\item \\textbf{可靠性量化}：独立蒙特卡洛验证和 Wilson 置信区间提供了统计上可量化的可靠性评估，"
    "弥补了机会约束近似无法给出解析保证的不足；\n"
    "    \\item \\textbf{策略可解释性}：五种策略的递进对比清晰地揭示了合作机制"
    "（交换、分工、集中）各自对团队总收益的边际贡献。\n"
    "\\end{enumerate}"
)
new4 = (
    "\subsubsection{模型与方法的有效性}\n"
    "\n"
    + Q + "宏节点+MPC+联合MILP" + Q + "框架的有效性体现在：\n"
    "\\begin{enumerate}\n"
    "    \\item \\textbf{维度压缩}：宏节点将变量规模从 $\\mathcal O(900\\times90\\times2)$ 压缩至"
    " $\\mathcal O(20\\times15\\times2)\\approx600$，使 MILP 可在 $5$ 秒内求解；\n"
    "    \\item \\textbf{信息利用}：MPC 每日重优化修正实际天气偏离期望造成的资源偏差；\n"
    "    \\item \\textbf{最优性保障}：字典序两阶段严格保证" + Q + "先 $Z$ 后 $M$" + Q + "的优先级，"
    "避免加权和的帕累托偏离；\n"
    "    \\item \\textbf{可靠性量化}：蒙特卡洛验证与 Wilson 置信区间提供统计可靠性评估；\n"
    "    \\item \\textbf{策略可解释性}：五种策略递进对比揭示合作机制对团队收益的边际贡献。\n"
    "\\end{enumerate}"
)
if old4 in content:
    content = content.replace(old4, new4)
    print("Fix 4: OK")
else:
    print("Fix 4: FAILED - old text not found")
    idx = content.find("模型与方法的有效性")
    if idx >= 0:
        print("  Found at", idx)
        print("  Context:", repr(content[idx:idx+80]))

# ============================================================
# FIX TRIM 5: 问题三-SAA (34→18 lines)
# ============================================================
old5 = (
    "\subsubsection{理论最优的" + Q + "方差陷阱" + Q + "}\n"
    "\n"
    "阶段二的结果表明，理论 $Z$ 最高的骨架（如 $B\\to S_1\\to W_2\\to S_2\\to W_3\\to E$，理论 $Z=518$）"
    "往往需要极长的作业与移动时间。在 90 天的尺度下，一旦遭遇连续雷暴，其资源消耗将远超正常天气，"
    "导致任务失败。单纯追求理论 $Z$ 会掉入" + Q + "高风险陷阱" + Q + "。\n"
    "\n"
    "\subsubsection{样本平均近似（SAA）评估}\n"
    "\n"
    "为量化风险，我们引入\\textbf{样本平均近似（Sample Average Approximation, SAA）}。"
    "对阶段二选出的 Top-10 骨架，分别生成 $N_{\\mathrm{SAA}}=500$ 条独立随机天气序列"
    " $\\boldsymbol{\\omega}^{(\\ell)}$，其中每日雷暴概率为 $p_{\\mathrm T}=0.2$。"
    "在第 $\\ell$ 条序列下，按期望消耗执行该骨架，记录其实际达成的 $Z^{(\\ell)}$"
    "（若中途资源耗尽则 $Z^{(\\ell)}=0$）。\n"
    "\n"
    "定义骨架的\\textbf{期望收益}为：\n"
    "\\begin{equation}\n"
    "\\widehat{\\mathbb{E}}[Z]=\\frac{1}{N_{\\mathrm{SAA}}}\\sum_{\\ell=1}^{N_{\\mathrm{SAA}}}Z^{(\\ell)}.\n"
    "\\end{equation}\n"
    "\n"
    "\subsubsection{Z\u2014\u2014成功率权衡分析}"
)
new5 = (
    "\subsubsection{Z\u2014\u2014成功率风险权衡与 SAA 评估}\n"
    "\n"
    "理论 $Z$ 最高的骨架（$B\\to S_1\\to W_2\\to S_2\\to W_3\\to E$，理论 $Z=518$）"
    "在连续雷暴下资源消耗远超正常天气，掉入" + Q + "高风险陷阱" + Q + "。"
    "为此引入\\textbf{样本平均近似}（SAA）：对 Top-10 骨架各生成 $N_{\\mathrm{SAA}}=500$ 条"
    "独立随机天气序列（$p_{\\mathrm T}=0.2$），按期望消耗执行并记录实际达成的 $Z^{(\\ell)}$"
    "（中途资源耗尽则 $Z^{(\\ell)}=0$），以\n"
    "\\begin{equation}\n"
    "\\widehat{\\mathbb{E}}[Z]=\\frac{1}{N_{\\mathrm{SAA}}}\\sum_{\\ell=1}^{N_{\\mathrm{SAA}}}Z^{(\\ell)}\n"
    "\\end{equation}\n"
    "评估期望收益。"
)
if old5 in content:
    content = content.replace(old5, new5)
    print("Fix 5: OK")
else:
    print("Fix 5: FAILED - old text not found")
    idx = content.find("理论最优的" + Q + "方差陷阱" + Q)
    if idx >= 0:
        print("  Found at", idx)
    else:
        idx = content.find("方差陷阱")
        if idx >= 0:
            print("  Found '方差陷阱' at", idx)
            print("  Context:", repr(content[idx-20:idx+60]))

with open(filepath, 'w', encoding='utf-8', newline='') as f:
    f.write(content)
print("\nFixes applied. File saved.")
