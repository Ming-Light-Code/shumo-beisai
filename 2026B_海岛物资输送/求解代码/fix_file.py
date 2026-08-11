import os
src = r'C:\Users\ming\Desktop\数模备赛\task3_improved.m'
dst = r'C:\tmp\task3_improved.m'

# Read with utf-8-sig to strip BOM
with open(src, 'r', encoding='utf-8-sig') as f:
    content = f.read()

os.makedirs(r'C:\tmp', exist_ok=True)
with open(dst, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)

print(f'Written {len(content)} chars to {dst}')
print(f'First 80 chars: {content[:80]}')
