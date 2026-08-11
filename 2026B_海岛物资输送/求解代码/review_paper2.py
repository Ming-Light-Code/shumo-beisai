import re, os, glob
fpath = glob.glob(r'C:\Users\ming\Downloads\*.tex')[0]
with open(fpath, 'r', encoding='utf-8') as f:
    content = f.read()

issues = []

# 1. placeholder keywords
if '关键词1' in content:
    issues.append(('[CRITICAL] 关键词占位符', '摘要末尾 \\keywords 仍为占位符"关键词1/2/3"，需替换为实际关键词'))

# 2. missing bib entries
cites = re.findall(r'\\cite\{([^}]+)\}', content)
bib_keys = re.findall(r'\\bibitem\{([^}]+)\}', content)
for c in cites:
    if c not in bib_keys:
        issues.append(('[CRITICAL] 引用缺失', f'正文引用 \\cite{{{c}}} 在参考文献列表中无对应条目'))

# 3. Redundant expressions
if '基于对于' in content:
    issues.append(('[表述] 冗余', '"基于对于" 两词连用冗余，建议改为 "基于" 或 "针对"'))
if '建立起了' in content:
    issues.append(('[表述] 冗余', '"建立起了" 建议简化为 "建立了"'))
if '进而保证' in content:
    issues.append(('[表述] 冗余', '"进而保证" 建议改为 "以确保"'))

# 4. Chinese enumeration commas
if '物资剩余，交易资金，天气状态' in content:
    issues.append(('[标点] 顿号/逗号', '摘要中列举项之间应使用顿号(、)而非逗号(，): "物资剩余，交易资金，天气状态" → "物资剩余、交易资金、天气状态"'))

# 5. Hardcoded table number
if '表 8' in content and 'ref' not in content[content.find('表 8')-50:content.find('表 8')]:
    issues.append(('[LaTeX] 硬编码编号', '"表 8" 应改用 \\ref{{tab:strategies_comparison}} 自动引用，避免编号漂移'))

# 6. Check \ref{sec:q4_mc}
if r'\label{sec:q4_mc}' in content and r'\ref{sec:q4_mc}' in content:
    pass  # OK

# Check for unreferenced figures
fig_labels = re.findall(r'\\label\{fig:([^}]+)\}', content)
for lbl in fig_labels:
    if not re.search(rf'\\ref\{{fig:{re.escape(lbl)}\}}', content):
        issues.append(('[LaTeX] 图片标签未引用', f'fig:{lbl} 定义了label但正文中未引用'))

# Check fig:problem1_flow ref
for ref in re.findall(r'\\ref\{([^}]+)\}', content):
    pass  # we have these

# 7. Check for end-of-sentence period in non-table Chinese text
lines = content.split('\n')
for i, line in enumerate(lines, 1):
    stripped = line.strip()
    # Skip LaTeX commands, tables, comments, figure paths
    if not stripped or stripped.startswith('\\') or stripped.startswith('%') or stripped.startswith('}'):
        continue
    if '\\' in stripped: continue
    if stripped.startswith('(') and stripped.endswith(')'): continue
    # Check if it's Chinese and doesn't end with period
    if re.search(r'[\u4e00-\u9fff]', stripped) and len(stripped) > 30:
        if not re.search(r'[。！？…—]$', stripped) and not re.search(r'[:：]$', stripped):
            if not any(x in stripped for x in ['\\', '{', '}', '&', '\\hline']):
                issues.append(('[标点] 句末可能缺句号', f'第{i}行: "{stripped[-50:]}"'))

# Filter out false positives: most table-related lines, LaTeX internal
issues = [x for x in issues if '\\' not in x[1][:3] or 'cite' not in x[1]]

# Deduplicate
seen = set()
unique = []
for item in issues:
    key = (item[0], item[1][:60])
    if key not in seen:
        seen.add(key)
        unique.append(item)
issues = unique

print(f'=== 论文审查结果 (共 {len(issues)} 个问题) ===')
print()
for i, (cat, detail) in enumerate(issues, 1):
    print(f'{i}. {cat}')
    print(f'   {detail}')
    print()

print('=== 参考文献统计 ===')
print(f'正文引用: {cites}')
print(f'文献条目: {bib_keys}')
print(f'缺失: {[c for c in cites if c not in bib_keys]}')
