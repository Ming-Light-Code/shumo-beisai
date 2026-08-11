import os

# Fix rso_solver_v4.m
path = r'C:\Users\ming\Desktop\任务3_最终版\优化C_滚动随机优化\rso_solver_v4.m'
with open(path, 'r', encoding='utf-8') as f:
    content = f.read()

old1 = "[nO,nH,nF]=cp_engine_v2('supply_needs_safe',[cur_pt,2],[],[],1,ce,cfg);"
new1 = "dE_sup=cfg.dist(cur_pt,2); cT_sup=cp_engine_v2('cons','thunder');\n    nO=dE_sup*(0.7*ce.MO+0.3*cT_sup.MO);\n    nH=dE_sup*(0.7*ce.MH+0.3*cT_sup.MH);\n    nF=dE_sup*(0.7*ce.MF+0.3*cT_sup.MF);"

old2 = "[nO,nH,nF]=cp_engine_v2('supply_needs_safe',[sim_pt,2],[],[],1,ce,cfg);"
new2 = "dE_sup2=cfg.dist(sim_pt,2); cT_sup2=cp_engine_v2('cons','thunder');\n                nO=dE_sup2*(0.7*ce.MO+0.3*cT_sup2.MO);\n                nH=dE_sup2*(0.7*ce.MH+0.3*cT_sup2.MH);\n                nF=dE_sup2*(0.7*ce.MF+0.3*cT_sup2.MF);"

c1 = content.count(old1)
c2 = content.count(old2)
print(f'rso_solver_v4.m: old1={c1}, old2={c2}')
content = content.replace(old1, new1)
content = content.replace(old2, new2)

with open(path, 'w', encoding='utf-8') as f:
    f.write(content)
print('rso_solver_v4.m fixed')

# Fix solve_q3_rso_v4.m
path2 = r'C:\Users\ming\Desktop\任务3_最终版\优化C_滚动随机优化\solve_q3_rso_v4.m'
with open(path2, 'r', encoding='utf-8') as f:
    content2 = f.read()

old3 = "[nO,nH,nF]=cp_engine_v2('supply_needs_safe',[st.pt,2],[],[],1,ce,cfg);"
new3 = "dE_s3=cfg.dist(st.pt,2); cT_s3=cp_engine_v2('cons','thunder');\n                    nO=dE_s3*(0.7*ce.MO+0.3*cT_s3.MO);\n                    nH=dE_s3*(0.7*ce.MH+0.3*cT_s3.MH);\n                    nF=dE_s3*(0.7*ce.MF+0.3*cT_s3.MF);"

c3 = content2.count(old3)
print(f'solve_q3_rso_v4.m: old3 occurrences={c3}')
content2 = content2.replace(old3, new3)

with open(path2, 'w', encoding='utf-8') as f:
    f.write(content2)
print('solve_q3_rso_v4.m fixed')

# Verify fixes
for p, label in [(path, 'rso'), (path2, 'rso_online')]:
    with open(p, 'r', encoding='utf-8') as f:
        c = f.read()
    remaining = c.count("supply_needs_safe")
    print(f'{label}: {remaining} remaining supply_needs_safe calls')
