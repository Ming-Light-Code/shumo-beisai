import os

# Fix solve_q3_mdp_v3.m 
path = r'C:\Users\ming\Desktop\任务3_最终版\优化B_MDP方向\solve_q3_mdp_v3.m'
with open(path, 'r', encoding='utf-8') as f:
    content = f.read()

old = "[nO,nH,nF]=cp_engine_v2('supply_needs_safe',[st.pt,2],[],[],1,ce,cfg);"
new = "dE_s=cfg.dist(st.pt,2); cT_s=cp_engine_v2('cons','thunder');\n        nO=dE_s*(0.7*ce.MO+0.3*cT_s.MO);\n        nH=dE_s*(0.7*ce.MH+0.3*cT_s.MH);\n        nF=dE_s*(0.7*ce.MF+0.3*cT_s.MF);"

count = content.count(old)
print(f'solve_q3_mdp_v3.m: occurrences={count}')
content = content.replace(old, new)

with open(path, 'w', encoding='utf-8') as f:
    f.write(content)

# Verify
with open(path, 'r', encoding='utf-8') as f:
    c = f.read()
print(f'Remaining: {c.count("supply_needs_safe")} supply_needs_safe calls')
print('Done')
