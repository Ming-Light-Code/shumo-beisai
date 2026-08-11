import numpy as np
import sys
sys.path.insert(0, r'C:\Users\ming\Desktop\数模备赛')
from q4_utils import *
from q4_solvers import *

# ============================================================
# MAIN: Execute all three solution tiers
# ============================================================
print('='*70)
print('Q4 MULTI-BEAM SURVEY LINE OPTIMIZATION')
print('Sea area: 5NM (NS) x 4NM (EW)')
print('Beam angle: 120 degrees')
print('='*70)

depth, y_coords, x_coords = load_data()
print(f'Depth grid: {depth.shape}')
print(f'Depth range: {np.min(depth):.1f} - {np.max(depth):.1f} m')
print()

# Determine directions to test
# EW: lines run east-west, spaced along Y (north-south)
# NS: lines run north-south, spaced along X (east-west)
directions = ['EW', 'NS']

results = {}

for direction in directions:
    print(f'\\n{\"=\"*60}')
    print(f'DIRECTION: {direction}')
    if direction == 'EW':
        print(f'  Survey lines run East-West (4 NM each)')
        print(f'  Spaced along North-South (5 NM range)')
    else:
        print(f'  Survey lines run North-South (5 NM each)')
        print(f'  Spaced along East-West (4 NM range)')
    print(f'{\"=\"*60}')
    
    results[direction] = {}
    
    # ---- TIER 2: GREEDY ----
    print(f'\\n--- TIER 2: GREEDY ALGORITHM ---')
    greedy_lines = solve_greedy(depth, y_coords, x_coords, direction)
    g_cov, g_leak, g_exceed, g_len = compute_metrics(depth, y_coords, x_coords, greedy_lines, direction, nx=150)
    print(f'  Lines: {len(greedy_lines)}, Total length: {g_len:.2f} NM')
    print(f'  Coverage: {g_cov:.2f}%, Leakage: {g_leak:.2f}%')
    print(f'  Overlap exceed 20%: {g_exceed:.4f} NM')
    results[direction]['greedy'] = {
        'n_lines': len(greedy_lines), 'total_len': g_len,
        'coverage_pct': g_cov, 'leakage_pct': g_leak,
        'exceed_len': g_exceed, 'lines': greedy_lines
    }
    
    # ---- TIER 1: DAG-DP ----
    print(f'\\n--- TIER 1: DAG-DP (GOLD) ---')
    dp_lines = solve_dag_dp(depth, y_coords, x_coords, direction, alpha=0.5)
    d_cov, d_leak, d_exceed, d_len = compute_metrics(depth, y_coords, x_coords, dp_lines, direction, nx=150)
    print(f'  Lines: {len(dp_lines)}, Total length: {d_len:.2f} NM')
    print(f'  Coverage: {d_cov:.2f}%, Leakage: {d_leak:.2f}%')
    print(f'  Overlap exceed 20%: {d_exceed:.4f} NM')
    results[direction]['dp'] = {
        'n_lines': len(dp_lines), 'total_len': d_len,
        'coverage_pct': d_cov, 'leakage_pct': d_leak,
        'exceed_len': d_exceed, 'lines': dp_lines
    }
    
    # ---- TIER 1: SA refinement ----
    print(f'\\n--- TIER 1: SA REFINEMENT (GOLD) ---')
    if len(dp_lines) > 0:
        sa_lines = sa_refine(depth, y_coords, x_coords, dp_lines, direction, iterations=1000)
    else:
        sa_lines = sa_refine(depth, y_coords, x_coords, greedy_lines, direction, iterations=1000)
    s_cov, s_leak, s_exceed, s_len = compute_metrics(depth, y_coords, x_coords, sa_lines, direction, nx=150)
    print(f'  Lines: {len(sa_lines)}, Total length: {s_len:.2f} NM')
    print(f'  Coverage: {s_cov:.2f}%, Leakage: {s_leak:.2f}%')
    print(f'  Overlap exceed 20%: {s_exceed:.4f} NM')
    results[direction]['sa'] = {
        'n_lines': len(sa_lines), 'total_len': s_len,
        'coverage_pct': s_cov, 'leakage_pct': s_leak,
        'exceed_len': s_exceed, 'lines': sa_lines
    }
    
    # ---- TIER 3: Multi-lambda DP ----
    print(f'\\n--- TIER 3: MULTI-LAMBDA DP (PLATINUM) ---')
    dp_multi = solve_dp_multi_lambda(depth, y_coords, x_coords, direction,
                                     alpha_values=[0.1, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0])
    pareto_points = []
    for alpha, info in dp_multi.items():
        print(f'  alpha={alpha:.1f}: {info[\"n_lines\"]} lines, leak={info[\"leakage_pct\"]:.2f}%, exceed={info[\"exceed_len\"]:.3f} NM, len={info[\"total_len\"]:.2f} NM')
        pareto_points.append(info)
    results[direction]['multi_dp'] = pareto_points
    
    # ---- TIER 3: NSGA-II ----
    print(f'\\n--- TIER 3: NSGA-II (PLATINUM) ---')
    nsga_results = nsga2_optimize(depth, y_coords, x_coords, direction, pop_size=30, generations=60)
    if nsga_results:
        # Find best by leakage
        best_nsga = min(nsga_results, key=lambda x: x['leakage_pct'])
        print(f'  Best (lowest leakage): {best_nsga[\"n_lines\"]} lines, leak={best_nsga[\"leakage_pct\"]:.2f}%, exceed={best_nsga[\"exceed_len\"]:.3f} NM, len={best_nsga[\"total_len\"]:.2f} NM')
        results[direction]['nsga_best'] = best_nsga
        results[direction]['nsga_all'] = nsga_results


# ============================================================
# FINAL COMPARISON
# ============================================================
print(f'\\n\\n{\"=\"*70}')
print(f'FINAL COMPARISON - ALL METHODS')
print(f'{\"=\"*70}')

for direction in ['EW', 'NS']:
    print(f'\\n--- DIRECTION: {direction} ---')
    print(f'{\"Method\":<20} {\"#Lines\":>8} {\"Length(NM)\":>12} {\"Cover%\":>10} {\"Leak%\":>10} {\"Exceed(NM)\":>12}')
    print(f'{\"-\"*20} {\"-\"*8} {\"-\"*12} {\"-\"*10} {\"-\"*10} {\"-\"*12}')
    
    for method_key, method_name in [('greedy','Greedy (Silver)'), ('dp','DAG-DP (Gold)'), 
                                      ('sa','SA+DP (Gold)')]:
        if method_key in results[direction]:
            r = results[direction][method_key]
            print(f'{method_name:<20} {r[\"n_lines\"]:>8} {r[\"total_len\"]:>12.2f} {r[\"coverage_pct\"]:>10.2f} {r[\"leakage_pct\"]:>10.2f} {r[\"exceed_len\"]:>12.4f}')
    
    if 'nsga_best' in results[direction]:
        r = results[direction]['nsga_best']
        print(f'{\"NSGA-II (Platinum)\":<20} {r[\"n_lines\"]:>8} {r[\"total_len\"]:>12.2f} {r[\"coverage_pct\"]:>10.2f} {r[\"leakage_pct\"]:>10.2f} {r[\"exceed_len\"]:>12.4f}')

print(f'\\n\\nEXECUTION COMPLETE.')
