# Final verification
import os

files = [
    r'C:\Users\ming\Desktop\任务3_最终版\优化C_滚动随机优化\rso_solver_v4.m',
    r'C:\Users\ming\Desktop\任务3_最终版\优化C_滚动随机优化\solve_q3_rso_v4.m',
    r'C:\Users\ming\Desktop\任务3_最终版\优化B_MDP方向\solve_q3_mdp_v3.m',
]

all_ok = True
for p in files:
    with open(p, 'r', encoding='utf-8') as f:
        content = f.read()
    remaining = content.count("supply_needs_safe")
    direct_calcs = content.count("dE_s") + content.count("dE_sup")
    status = "OK" if remaining == 0 and direct_calcs >= 1 else "ISSUE"
    if status != "OK": all_ok = False
    fname = os.path.basename(p)
    print(f'{fname}: supply_needs_safe={remaining}, direct_calcs={direct_calcs} -> {status}')

# Sample the fix in rso_solver_v4
with open(files[0], 'r', encoding='utf-8') as f:
    lines = f.readlines()
print()
print('=== Sample fix in rso_solver_v4.m (sim_supply_then_tail) ===')
for i, line in enumerate(lines):
    if 'dE_sup' in line or 'cT_sup' in line:
        for j in range(max(0,i-1), min(len(lines), i+7)):
            print(f'  {j+1}: {lines[j].rstrip()}')
        break

print()
print('All fixes verified: ' + ('PASS' if all_ok else 'FAIL'))
