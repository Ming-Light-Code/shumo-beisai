import re

with open(r'C:\Users\ming\Desktop\数模备赛\task3_improved.m', 'r') as f:
    lines = f.readlines()

# Strategy: Remove ALL function-level 'end' keywords
# Function-level ends are lines where stripped content is exactly 'end'
# and they are preceded by a blank line or the last line of function body
# But NOT preceded by an if/for/while/switch that's pending closure

# Simpler approach: find all function defs, track their end, remove standalone 'end' at function level
out_lines = []
i = 0
removed = 0
while i < len(lines):
    line = lines[i]
    stripped = line.strip()
    # Check if this is a function declaration
    if stripped.startswith('function '):
        out_lines.append(line)
        i += 1
        fn_indent = len(line) - len(line.lstrip())
        # Continue through function body
        body_start = i
        while i < len(lines):
            s = lines[i].strip()
            if s.startswith('function ') or s.startswith('%%'):
                break
            # Check if this is a function-level end
            if s == 'end':
                cur_indent = len(lines[i]) - len(lines[i].lstrip())
                if cur_indent == fn_indent:
                    # This is the function's closing end - skip it
                    removed += 1
                    i += 1
                    continue
            out_lines.append(lines[i])
            i += 1
    else:
        out_lines.append(line)
        i += 1

with open(r'C:\Users\ming\Desktop\问题三_PBOVI方案\task3_improved.m', 'w', newline='\n') as f:
    f.writelines(out_lines)
print(f'Removed {removed} function-level end statements. Lines: {len(lines)} -> {len(out_lines)}')
