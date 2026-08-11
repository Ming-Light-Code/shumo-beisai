# MILP v4 (Turbo+) - Changelog vs v3

## Bug Fixes from v3 Analysis

### P1: No-Solution Guard (FIXED)
If no feasible solution exists, v3 would attempt to print daily schedule
with empty arrays, causing a MATLAB runtime error.
v4 checks if best_Z <= 0 before printing and returns early with a
clear message.

### P2: Greedy Solutions Now Include Purchase Amounts (FIXED)
In v3, 
_work==0 solutions set est_buy=[], losing purchase
information in the daily schedule output.
v4: greedy_quick now returns uy_all (n_supply x 3 matrix) recording
purchase amounts at each supply station. All solutions, greedy or MILP,
now show complete purchase details.

### P3: Accurate MILP-Saved Percentage (FIXED)
v3 used count (all skeletons including pre-filtered) as denominator,
underestimating the true MILP savings rate.
v4 tracks possible_milp (skeletons passing all pre-filters) and
computes savings against the theoretical maximum of 2 MILP calls per
eligible skeleton.

## New Optimizations

### O5: Greedy Max-Work Stage-1 Skip
Before running full MILP, v4 runs greedy with maximum feasible work days
(assigned greedily by yield: W3 > W1 > W2). If greedy achieves the
theoretical Z_upper (meaning Z is already optimal):
- Skip MILP stage 1 entirely (1 intlinprog call saved)
- Run only stage 2 with Z fixed at Z_upper

New helper: greedy_work_assign() assigns work days greedily by yield.

### O6: Separate Stage-1/Stage-2 Tracking
v4 reports Stage-1 calls, Stage-2 calls, and Stage-1 saved
separately, giving granular visibility into where MILP time is spent.

### Code Quality: DRY Constraint Construction
Milp constraint matrices (Aeq, beq, A, b) are now built by a shared
uild_milp_matrices() function, eliminating duplicate code between
milp_full and milp_stage2.

## Function Summary (7 total)
| Function | Purpose |
|----------|---------|
| milp_task1_v4 | Main driver with all optimizations |
| greedy_work_assign | Greedy work-day assignment by yield |
| greedy_quick | Fast greedy simulation (returns buy amounts) |
| milp_full | Full two-stage MILP |
| milp_stage2 | Stage-2-only MILP (Z pre-known) |
| build_milp_matrices | Shared constraint construction |
| print_daily_schedule_milp | Daily schedule output |
