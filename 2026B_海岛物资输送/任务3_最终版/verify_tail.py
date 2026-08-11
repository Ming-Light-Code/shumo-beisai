# Verify tail_simulate supply routing logic
with open(r'C:\Users\ming\Desktop\任务3_最终版\优化C_滚动随机优化\rso_solver_v4.m', 'r', encoding='utf-8') as f:
    content = f.read()

# Find and print the MOVE section
idx = content.find('4. MOVE: toward E, or toward nearest supply')
if idx >= 0:
    print(content[idx:idx+500])
else:
    print('NOT FOUND')
    print('Looking for supply routing...')
    idx2 = content.find('need_supply')
    if idx2 >= 0:
        print(f'Found need_supply at {idx2}')
        print(content[idx2-50:idx2+300])
