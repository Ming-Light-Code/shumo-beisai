filepath = r"C:\Users\ming\Downloads\正文 (9).tex"
with open(filepath, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find start: \section{符号说明}
start = None
for i, line in enumerate(lines):
    if r'\section{符号说明}' in line:
        start = i
        break

# tbl_start is the actual table content start (after \section + blank line)
tbl_start = start + 2  # skip section line and blank line

# Find end of table: close brace of \end{center} that ends the table
# The table has \end{tabular}\n\end{center} at the end
for i in range(tbl_start, len(lines)):
    if r'\end{center}' in line and i > tbl_start + 5:
        # Check if the preceding lines around i contain a \end{tabular}
        # Count \end{center} occurrences - we want the first one after the table starts
        end = i
        break

tbl_end = end + 1  # include \end{center} line

# Build three tables
table1 = '''\\begin{center}
\\begin{tabular}{>{\\centering\\arraybackslash}p{0.18\\textwidth}p{0.72\\textwidth}}
\\hline
符号 & 含义 \\\\ \\hline
$B,E$ & 起始岛和终点交付岛（表中后续坐标均按相应任务地图取值） \\\\
$S_1,S_2$ & 两个海上补给平台 \\\\
$W_1,W_2,W_3$ & 三个海上作业点 \\\\
$d(p,q)$ & 点 $p$ 与 $q$ 间的曼哈顿距离 \\\\
$p_t$ & 船舶在第 $t$ 个决策时刻的位置 \\\\
$O_t,H_t,F_t$ & 第 $t$ 个决策时刻的燃油、淡水、食物余量 \\\\
$M_t,Z_t$ & 第 $t$ 个决策时刻的剩余资金、目标物资存量 \\\\
$c_t$ & 第 $t$ 个决策时刻的连续作业天数 \\\\
$\\mathbf{s}_t$ & 第 $t$ 个决策时刻的完整系统状态 \\\\
$\\omega_t$ & 第 $t$ 天观测到的天气，$\\omega_t\\in\\{\\mathrm{N},\\mathrm{T}\\}$ \\\\
$\\ell_t$ & 截至决策时刻 $t$ 已使用的天数 \\\\
$p_{\\mathrm N},p_{\\mathrm T}$ & 正常、雷暴天气概率（任务3/4中分别为 $0.8,0.2$） \\\\
$L_{\\max}$ & 载重上限（任务1/2为 $120$，任务3/4为 $400$） \\\\
$T_{\\max}$ & 任务期限（任务1/2为 $30$ 天，任务3/4为 $90$ 天） \\\\
\\hline
\\end{tabular}
\\end{center}'''

    table2 = '''\\begin{center}
\\begin{tabular}{>{\\centering\\arraybackslash}p{0.18\\textwidth}p{0.72\\textwidth}}
\\hline
符号 & 含义 \\\\ \\hline
$c_r^\\chi(\\omega)$ & 天气 $\\omega$ 下资源 $r$ 执行动作类型 $\\chi$ 的日消耗，$r\\in\\{O,H,F\\}$，$\\chi\\in\\{\\mathrm{mv},\\mathrm{pk},\\mathrm{wk}\\}$ \\\\
$\\mathbf c^\\chi(\\omega)$ & 三类自持资源的消耗向量 $(c_O^\\chi(\\omega),c_H^\\chi(\\omega),c_F^\\chi(\\omega))$ \\\\
$\\pi_O,\\pi_H,\\pi_F$ & 补给采购单价（2,\\,1,\\,2） \\\\
$Y_j$ & 作业点 $W_j$ 的日收益（20,\\,15,\\,28） \\\\
$W_j^{\\max}$ & $W_j$ 的单次最大连续作业天数（4,\\,5,\\,3） \\\\
\\hline
\\end{tabular}
\\end{center}'''

    table3 = '''\\begin{center}
\\begin{tabular}{>{\\centering\\arraybackslash}p{0.18\\textwidth}p{0.72\\textwidth}}
\\hline
符号 & 含义 \\\\ \\hline
$\\mathcal{R}=(p_0,\\dots,p_{m+1})$ & 路径骨架，$p_0=B$，$p_{m+1}=E$ \\\\
$w_{1i},w_{2i}$ & 路径中第 $i$ 个作业点停靠段内的两段作业天数 \\\\
$\\rho_i$ & 第 $i$ 个停靠段是否通过停泊重置连续作业计数 \\\\
$q_i^r$ & 第 $i$ 个停靠段采购资源 $r$ 的数量 \\\\
\\hline
\\end{tabular}
\\end{center}'''

new_block = table1 + '\n\n' + table2 + '\n\n' + table3 + '\n'

del lines[tbl_start:tbl_end]
lines.insert(tbl_start, new_block)

with open(filepath, 'w', encoding='utf-8', newline='') as f:
    f.writelines(lines)
print(f"Table split OK. Lines {tbl_start+1}-{tbl_end} replaced. Total: {len(lines)}")
