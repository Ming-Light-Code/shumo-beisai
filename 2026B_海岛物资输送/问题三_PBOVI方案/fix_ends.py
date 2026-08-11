import re

with open(r'C:\Users\ming\Desktop\问题三_PBOVI方案\task3_improved.m', 'r') as f:
    lines = f.readlines()

# Find all function start lines and check if they have end
out = []
i = 0
fixes = 0
while i < len(lines):
    line = lines[i]
    stripped = line.strip()
    # Check if this line is a function declaration
    is_fn = stripped.startswith('function ')
    if is_fn:
        fn_start_idx = i
        fn_body_start = i + 1
        # Find the end of this function: next function or next %% or EOF
        fn_end_candidate = len(lines)
        for j in range(i+1, len(lines)):
            s = lines[j].strip()
            if s.startswith('function ') or s.startswith('%%'):
                fn_end_candidate = j
                break
        # Check if the line before fn_end_candidate is 'end'
        last_line = lines[fn_end_candidate - 1].strip()
        needs_end = True
        if last_line == 'end':
            needs_end = False
            fn_end_candidate = fn_end_candidate
        else:
            # Check for single-line function pattern
            fn_line = lines[i].strip()
            # Single-line: function d = md(a, b)
            # followed by body and no end
            # Already determined needs_end
            pass
        
        if needs_end:
            # Insert 'end' before the next function/section
            out.extend(lines[i:fn_end_candidate])
            out.append('end\n')
            i = fn_end_candidate
            fixes += 1
        else:
            out.extend(lines[i:fn_end_candidate])
            i = fn_end_candidate
    else:
        out.append(line)
        i += 1

with open(r'C:\Users\ming\Desktop\问题三_PBOVI方案\task3_improved.m', 'w', newline='\n') as f:
    f.writelines(out)
print(f'Added {fixes} end statements')
