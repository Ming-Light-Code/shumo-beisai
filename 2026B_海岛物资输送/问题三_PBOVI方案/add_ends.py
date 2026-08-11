import re

with open(r'C:\Users\ming\Desktop\问题三_PBOVI方案\task3_improved.m','r',encoding='utf-8-sig') as f:
    content = f.read()

lines = content.split('\n')
out = []
i = 0
added = 0
while i < len(lines):
    l = lines[i]
    s = l.strip()
    if s.startswith('function '):
        out.append(l)
        i += 1
        fn_indent = len(l) - len(l.lstrip())
        fn_end = len(lines)
        for j in range(i, len(lines)):
            js = lines[j].strip()
            if js.startswith('function ') or js.startswith('%%'):
                fn_end = j
                break
        # Get last non-empty line of function body
        last_line_idx = fn_end - 1
        while last_line_idx >= i and lines[last_line_idx].strip() == '':
            last_line_idx -= 1
        if last_line_idx >= i:
            last_s = lines[last_line_idx].strip()
            # Check if last line is 'end' at function indentation
            last_indent = len(lines[last_line_idx]) - len(lines[last_line_idx].lstrip())
            if last_s != 'end' or last_indent != fn_indent:
                # Add function body and an end
                for k in range(i, fn_end):
                    out.append(lines[k])
                # Insert 'end' before next function/section with proper indentation
                indent_str = ' ' * fn_indent
                out.append(indent_str + 'end')
                added += 1
                i = fn_end
            else:
                for k in range(i, fn_end):
                    out.append(lines[k])
                i = fn_end
    else:
        out.append(l)
        i += 1

with open(r'C:\Users\ming\Desktop\问题三_PBOVI方案\task3_improved.m','w',encoding='utf-8',newline='\n') as f:
    f.write('\n'.join(out))
print(f'Added {added} end statements')
