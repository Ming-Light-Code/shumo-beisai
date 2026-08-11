# MILP v6 (Stop-Work Strategy) - Changelog

## New Feature: Stop-Work Strategy

### Problem
The original model enforces w_j <= WM_j per work visit (max consecutive
work days). But the problem only limits *consecutive* work days. Inserting
a stop day (O=1,H=1,F=1) resets the counter, enabling a second work block.

### Model Change
Each work visit now has THREE decision variables:
- w1_j: work days in first block (0..WM_j)
- b_j:  binary, whether to insert a stop day (0 or 1)
- w2_j: work days in second block (0..WM_j, constrained: w2_j <= b_j * WM_j)

Total work at visit: w1_j + w2_j
Total stop at visit: b_j
Total days at visit: w1_j + b_j + w2_j
Z gained: (w1_j + w2_j) * WY_j

### Impact Example (Task 1)
Path B -> S1 -> W3 -> S2 -> E:
- Without stop: max 3 work days at W3 = 84 Z
- With stop: 3 + stop(1) + 3 = 168 Z (for 7 total days at W3)

### New/Modified Functions (7 total)
| Function | Change |
|----------|--------|
| milp_task1_v6 | Updated w1/b/w2 tracking |
| greedy_work_assign_sw | Two-pass: fill w1 blocks, then w2 blocks with stops |
| greedy_quick_sw | Accepts w1,b,w2 arrays, builds stop days in schedule |
| milp_full_sw | 3*n_work work vars + b_j constraints |
| milp_stage2_sw | Stage-2 only with stop-work vars |
| build_milp_sw | Updated constraints for stop-work model |
| print_daily_schedule_sw | Shows Stop@Wx actions in output |

### Variable Layout (MILP)
Before: w_j (n_work) + buy (3*n_supply) + state (4*(m+2))
After:  w1_j + b_j + w2_j (3*n_work) + buy (3*n_supply) + state (4*(m+2))
intvars increased by 2*n_work per work visit.
