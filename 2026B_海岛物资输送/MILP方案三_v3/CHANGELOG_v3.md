# MILP v3 (Turbo) - Changelog vs v2

## New Optimizations

### O1. Greedy Pre-filter (safe)
Before calling MILP, run greedy simulation with wdays=0. If infeasible, skip.
Reasoning: work days only add resource consumption; if infeasible with 0 work
days, definitely infeasible with any positive work days.
Impact: eliminates MILP calls for skeletons that are structurally infeasible.

### O2. n_work==0 -> Greedy Only (safe)
Skeletons with no work points have no integer variables to optimize.
The greedy purchase strategy is proven optimal for deterministic schedules.
No MILP call needed; greedy gives exact (Z, M).
Impact: eliminates MILP calls for work-free paths (~40% of filtered skeletons).

### O3. Upper Bound Pruning (safe)
Compute Z_upper = INIT_Z + min(remain_days*28, sum(WM_j*WY_j)).
If Z_upper <= best_Z, the skeleton cannot beat the current best, skip.
Impact: increasingly effective as better solutions are found.

### O4. Skeleton Ordering
Process skeletons by seq_len ascending (0,1,2,...,max_seq).
Finds simple solutions early, establishing high best_Z for O3 pruning.
Impact: amplifies O3 effectiveness.

## Performance Metrics (added to output)
- Greedy-only count: skeletons solved without MILP
- MILP calls: actual intlinprog invocations
- UB-skipped: skeletons pruned by upper bound check
- Prefilter-skip: skeletons pruned by greedy pre-filter
- MILP saved %: (1 - milp_calls/(total - greedy_only)) * 100

## Code Changes
- Added greedy_quick() function (~60 lines): fast greedy forward simulation
- Main loop: 3 early-exit branches before MILP call
- Output: additional statistics section showing optimization effectiveness
