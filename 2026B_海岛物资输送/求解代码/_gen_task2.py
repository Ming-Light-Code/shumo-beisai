import sys
sys.stdout.reconfigure(encoding="utf-8")

with open(r"C:\Users\ming\Desktop\数模代码\task1_solution.m", "r", encoding="utf-8") as f:
    content = f.read()

# Replace header
content = content.replace(
    "function task1_solution()",
    "function task2_solution()"
)
content = content.replace(
    "% task1_solution.m - Task 1: Enum+Greedy + DP + DE comparison",
    "% task2_solution.m - Task 2: Extreme thunderstorm (all 30 days thunderstorm)"
)
content = content.replace(
    "% Fixed: consume-then-buy in greedy_sim, proper max_seq, daily schedule output",
    "% Thunderstorm rates: Move O=8,H=4,F=3; Idle O=3,H=3,F=2; Work O=8,H=6,F=6"
)

# Replace consumption rates
content = content.replace(
    "CM = [2, 3, 2];   CW = [5, 4, 3];",
    "CM = [8, 4, 3];   CW = [8, 6, 6];"
)

# Replace idle consumption in greedy_sim (hardcoded as 1,1,1)
content = content.replace(
    "cO(day) = 1; cH(day) = 1; cF(day) = 1;",
    "cO(day) = 3; cH(day) = 3; cF(day) = 2;"
)

# Also check for any remaining "= 1; cH(day) = 1; cF(day) = 1" patterns
# (The above replace should catch all instances)

# Replace "Task 1" in print statements
content = content.replace(
    "Task 1: Three Methods Comparison",
    "Task 2: Thunderstorm Extreme Case"
)
content = content.replace(
    "Method 1: Enumeration + Greedy (MILP equivalent)",
    "Method 1: Enumeration + Greedy (Thunderstorm)"
)

with open(r"C:\Users\ming\Desktop\数模代码\task2_solution.m", "w", encoding="utf-8") as f:
    f.write(content)

print(f"Written: {len(content)} chars")

# Verify key changes
for check in ["CM = [8, 4, 3]", "CW = [8, 6, 6]", "cO(day) = 3; cH(day) = 3; cF(day) = 2", "task2_solution"]:
    count = content.count(check)
    print(f"  '{check}': {count} occurrences")
