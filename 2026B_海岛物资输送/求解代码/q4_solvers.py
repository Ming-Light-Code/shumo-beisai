import numpy as np
import sys
sys.path.insert(0, r'C:\Users\ming\Desktop\数模备赛')
from q4_utils import *

# ============================================================
# COVERAGE METRICS
# ============================================================
def compute_metrics(depth, y_coords, x_coords, line_positions, direction='EW', nx=400):
    if len(line_positions) == 0:
        return 0.0, 100.0, 0.0, 0.0
    
    if direction == 'EW':
        line_len = SEA_X_NM
        x_eval = np.linspace(0, SEA_X_NM, nx)
        dx = SEA_X_NM / nx
        total_area = SEA_X_NM * SEA_Y_NM
    else:
        line_len = SEA_Y_NM
        x_eval = np.linspace(0, SEA_Y_NM, nx)
        dx = SEA_Y_NM / nx
        total_area = SEA_X_NM * SEA_Y_NM

    covered_area_nm2 = 0.0
    overlap_exceed_len = 0.0

    for xi in range(nx):
        x = x_eval[xi]
        intervals = []
        for yk in line_positions:
            if direction == 'EW':
                w = bilinear_interp(depth, y_coords, x_coords, yk, x) * TAN60 / NM_TO_M
            else:
                w = bilinear_interp(depth, y_coords, x_coords, x, yk) * TAN60 / NM_TO_M
            if w > 0:
                intervals.append((yk - w, yk + w))
        
        if not intervals:
            continue
        
        intervals.sort(key=lambda v: v[0])
        merged = [intervals[0]]
        for lo, hi in intervals[1:]:
            if lo <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
            else:
                merged.append((lo, hi))
        
        union_len = sum(hi - lo for lo, hi in merged)
        covered_area_nm2 += union_len * dx

    # Overlap check for adjacent lines
    m = len(line_positions)
    for k in range(m - 1):
        yk = line_positions[k]
        yj = line_positions[k + 1]
        for xi in range(nx):
            x = x_eval[xi]
            if direction == 'EW':
                wk = bilinear_interp(depth, y_coords, x_coords, yk, x) * TAN60 / NM_TO_M
                wj = bilinear_interp(depth, y_coords, x_coords, yj, x) * TAN60 / NM_TO_M
            else:
                wk = bilinear_interp(depth, y_coords, x_coords, x, yk) * TAN60 / NM_TO_M
                wj = bilinear_interp(depth, y_coords, x_coords, x, yj) * TAN60 / NM_TO_M
            if wk <= 0 or wj <= 0:
                continue
            overlap_val = (yk + wk) - (yj - wj)
            if overlap_val > 0:
                total_w = wk + wj
                rate = overlap_val / total_w if total_w > 0 else 0
                if rate > OVERLAP_MAX:
                    overlap_exceed_len += (overlap_val - OVERLAP_MAX * total_w) * dx

    leakage_pct = max(0, 100.0 * (total_area - covered_area_nm2) / total_area)
    coverage_pct = 100.0 * covered_area_nm2 / total_area
    total_line_len = len(line_positions) * line_len

    return coverage_pct, leakage_pct, overlap_exceed_len, total_line_len


# ============================================================
# TIER 2 (SILVER): GREEDY
# ============================================================
def solve_greedy(depth, y_coords, x_coords, direction='EW'):
    if direction == 'EW':
        x_eval = np.linspace(0, SEA_X_NM, 150)
        max_pos = SEA_Y_NM
        line_len = SEA_X_NM
    else:
        x_eval = np.linspace(0, SEA_Y_NM, 200)
        max_pos = SEA_X_NM
        line_len = SEA_Y_NM
    
    lines = []
    current_y = 0.0
    step = 0.005

    # First line: find position where strip covers y=0
    for pos in np.arange(step, max_pos, step):
        w = get_strip_half_widths(depth, y_coords, x_coords, pos, x_eval)
        w_min = np.min(w)
        if pos - w_min <= 0:
            lines.append(pos)
            current_y = pos + w_min
            break

    # Subsequent lines
    MAX_ITER = 500
    for _ in range(MAX_ITER):
        if current_y >= max_pos:
            break
        best_pos = None
        best_gap = 999.0
        for pos in np.arange(current_y + step, min(max_pos + step, current_y + 0.5), step):
            w = get_strip_half_widths(depth, y_coords, x_coords, pos, x_eval)
            w_min = np.min(w)
            # Can this line cover from current_y?
            if pos - w_min <= current_y:
                gap = current_y - (pos - w_min)  # how much it overshoots (<= 0 means gap)
                if gap < best_gap and pos - w_min >= current_y - 0.02:
                    best_gap = gap
                    best_pos = pos
        
        if best_pos is None:
            # No position found - force next line
            last_w = get_strip_half_widths(depth, y_coords, x_coords, lines[-1], x_eval)
            current_y = lines[-1] + np.min(last_w)
            continue
        
        lines.append(best_pos)
        w = get_strip_half_widths(depth, y_coords, x_coords, best_pos, x_eval)
        current_y = best_pos + np.min(w)

    # Ensure last line covers boundary
    if lines:
        for pos in np.arange(max_pos, lines[-1], -step):
            w = get_strip_half_widths(depth, y_coords, x_coords, pos, x_eval)
            if pos + np.min(w) >= max_pos and pos not in lines:
                lines.append(pos)
                break

    lines = sorted(set(lines))
    return np.array(lines)


# ============================================================
# TIER 1 (GOLD): DAG-DP
# ============================================================
def solve_dag_dp(depth, y_coords, x_coords, direction='EW', alpha=0.5):
    if direction == 'EW':
        x_eval = np.linspace(0, SEA_X_NM, 100)
        max_pos = SEA_Y_NM
        line_len = SEA_X_NM
    else:
        x_eval = np.linspace(0, SEA_Y_NM, 100)
        max_pos = SEA_X_NM
        line_len = SEA_Y_NM
    
    # Candidate positions: 0.01 NM spacing
    step = 0.01
    candidates = np.arange(step, max_pos, step)
    N = len(candidates)
    
    # Precompute min strip half-width for each candidate
    min_w = np.zeros(N)
    for i, y in enumerate(candidates):
        w = get_strip_half_widths(depth, y_coords, x_coords, y, x_eval)
        min_w[i] = np.min(w)
    
    # Edge validity check: for edge (i,j), d = candidates[j] - candidates[i]
    # No gap condition: d <= min_w[i] + min_w[j]
    # (Conservative: guarantees no gaps)
    
    INF = 1e10
    dp = np.full(N, INF)
    prev = np.full(N, -1, dtype=int)
    
    # Initialize: edges from virtual start (y=0)
    for i in range(N):
        if candidates[i] - min_w[i] <= 0:  # covers y=0
            # Compute excess overlap cost with boundary (approximate)
            cost = line_len + alpha * max(0, candidates[i] - min_w[i])
            dp[i] = cost
    
    # DP transitions
    for j in range(N):
        for i in range(j):
            if dp[i] >= INF:
                continue
            d = candidates[j] - candidates[i]
            # No-gap condition
            if d > min_w[i] + min_w[j]:
                continue
            # Edge cost: line length + overlap penalty
            overlap = min_w[i] + min_w[j] - d
            excess = max(0, overlap - OVERLAP_MAX * (min_w[i] + min_w[j]))
            cost_ij = line_len + alpha * excess
            if dp[i] + cost_ij < dp[j]:
                dp[j] = dp[i] + cost_ij
                prev[j] = i
    
    # Find best endpoint: line that covers the north boundary
    best_end = -1
    best_cost = INF
    for i in range(N):
        if candidates[i] + min_w[i] >= max_pos and dp[i] < best_cost:
            best_cost = dp[i]
            best_end = i
    
    # Reconstruct path
    if best_end < 0:
        return np.array([])
    
    path = []
    cur = best_end
    while cur >= 0:
        path.append(candidates[cur])
        cur = prev[cur]
    path.reverse()
    
    return np.array(path)


# ============================================================
# TIER 1 (GOLD): Simulated Annealing refinement
# ============================================================
def sa_refine(depth, y_coords, x_coords, lines_init, direction='EW', iterations=2000):
    def cost_fn(lines_arr):
        if len(lines_arr) < 2:
            return 1e10
        cov, leak, exceed, length = compute_metrics(depth, y_coords, x_coords, lines_arr, direction, nx=150)
        # Penalize leakage heavily
        return leak * 500 + exceed * 5 + length
    
    lines = np.array(sorted(lines_init))
    best_lines = lines.copy()
    best_cost = cost_fn(best_lines)
    current_cost = best_cost
    
    T0 = 0.5
    T_end = 0.001
    cooling_rate = (T_end / T0) ** (1.0 / iterations)
    T = T0
    
    np.random.seed(42)
    
    for it in range(iterations):
        new_lines = lines.copy()
        op = np.random.randint(0, 3)
        if op == 0 and len(new_lines) > 1:
            # Shift one line
            idx = np.random.randint(0, len(new_lines))
            delta = np.random.normal(0, 0.02)
            new_lines[idx] += delta
            new_lines = np.clip(new_lines, 0.01, (SEA_Y_NM if direction=='EW' else SEA_X_NM) - 0.01)
            new_lines = np.sort(new_lines)
        elif op == 1:
            # Remove a line
            if len(new_lines) > 1:
                idx = np.random.randint(0, len(new_lines))
                new_lines = np.delete(new_lines, idx)
        elif op == 2:
            # Add a line
            pos = np.random.uniform(0.01, (SEA_Y_NM if direction=='EW' else SEA_X_NM) - 0.01)
            new_lines = np.sort(np.append(new_lines, pos))
        
        new_cost = cost_fn(new_lines)
        delta = new_cost - current_cost
        
        if delta < 0 or np.random.random() < np.exp(-delta / T):
            lines = new_lines
            current_cost = new_cost
            if current_cost < best_cost:
                best_cost = current_cost
                best_lines = lines.copy()
        
        T *= cooling_rate
    
    return best_lines


# ============================================================
# TIER 3 (PLATINUM): Multi-lambda DP + Pareto
# ============================================================
def solve_dp_multi_lambda(depth, y_coords, x_coords, direction='EW', alpha_values=None):
    if alpha_values is None:
        alpha_values = [0.1, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0]
    
    results = {}
    for alpha in alpha_values:
        lines = solve_dag_dp(depth, y_coords, x_coords, direction, alpha)
        if len(lines) > 0:
            cov, leak, exceed, length = compute_metrics(depth, y_coords, x_coords, lines, direction, nx=150)
            results[alpha] = {
                'lines': lines,
                'n_lines': len(lines),
                'coverage_pct': cov,
                'leakage_pct': leak,
                'exceed_len': exceed,
                'total_len': length
            }
    return results


# ============================================================
# TIER 3 (PLATINUM): NSGA-II for multi-objective
# ============================================================
def nsga2_optimize(depth, y_coords, x_coords, direction='EW', pop_size=50, generations=100):
    max_pos = SEA_Y_NM if direction == 'EW' else SEA_X_NM
    
    def random_solution():
        n = np.random.randint(5, 40)
        pos = np.sort(np.random.uniform(0.01, max_pos - 0.01, n))
        return pos
    
    def evaluate(ind):
        if len(ind) == 0:
            return (0, 100, 1e10, 0)
        return compute_metrics(depth, y_coords, x_coords, ind, direction, nx=100)

    np.random.seed(123)
    population = [random_solution() for _ in range(pop_size)]
    
    for gen in range(generations):
        # Evaluate all
        fitness = [evaluate(ind) for ind in population]
        # Objectives: min(leakage), min(exceed_len), min(total_len)
        objectives = np.array([(f[1], f[2], f[3]) for f in fitness])
        
        # Non-dominated sorting (simplified)
        n = len(population)
        fronts = []
        dominated_count = np.zeros(n, dtype=int)
        dominates_list = [[] for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                a, b = objectives[i], objectives[j]
                # i dominates j?
                if (a[0] <= b[0] and a[1] <= b[1] and a[2] <= b[2]) and \
                   (a[0] < b[0] or a[1] < b[1] or a[2] < b[2]):
                    dominates_list[i].append(j)
                elif (b[0] <= a[0] and b[1] <= a[1] and b[2] <= a[2]) and \
                     (b[0] < a[0] or b[1] < a[1] or b[2] < a[2]):
                    dominated_count[i] += 1
        
        front1 = [i for i in range(n) if dominated_count[i] == 0]
        fronts.append(front1)
        
        # Crowding distance for front1
        crowd = np.zeros(n)
        for front in fronts:
            if len(front) <= 2:
                for idx in front:
                    crowd[idx] = 1.0
                continue
            for obj_idx in range(3):
                sorted_idx = sorted(front, key=lambda x: objectives[x][obj_idx])
                crowd[sorted_idx[0]] = crowd[sorted_idx[-1]] = 1e10
                obj_range = objectives[sorted_idx[-1]][obj_idx] - objectives[sorted_idx[0]][obj_idx]
                if obj_range == 0:
                    continue
                for k in range(1, len(sorted_idx) - 1):
                    crowd[sorted_idx[k]] += (objectives[sorted_idx[k+1]][obj_idx] - objectives[sorted_idx[k-1]][obj_idx]) / obj_range
        
        # Selection for next generation
        if gen < generations - 1:
            new_pop = []
            for _ in range(pop_size // 2):
                # Tournament selection
                t1, t2 = np.random.choice(front1, 2, replace=False)
                p1_idx = t1 if crowd[t1] >= crowd[t2] else t2
                t3, t4 = np.random.choice(front1, 2, replace=False)
                p2_idx = t3 if crowd[t3] >= crowd[t4] else t4
                
                # Crossover
                p1, p2 = population[p1_idx], population[p2_idx]
                if len(p1) >= 2 and len(p2) >= 2:
                    # Average the positions (simplified crossover)
                    child1 = np.sort(np.concatenate([p1[:len(p1)//2], p2[len(p2)//2:]]))
                    child2 = np.sort(np.concatenate([p2[:len(p2)//2], p1[len(p1)//2:]]))
                else:
                    child1, child2 = p1.copy(), p2.copy()
                
                # Mutation
                if np.random.random() < 0.3:
                    idx = np.random.randint(0, max(1, len(child1)))
                    child1 = np.sort(np.append(np.delete(child1, idx % len(child1)), 
                                     np.random.uniform(0.01, max_pos - 0.01)))
                if np.random.random() < 0.3:
                    idx = np.random.randint(0, max(1, len(child2)))
                    child2 = np.sort(np.append(np.delete(child2, idx % len(child2)), 
                                     np.random.uniform(0.01, max_pos - 0.01)))
                
                new_pop.extend([child1, child2])
            population = new_pop[:pop_size]
    
    # Final evaluation of all solutions
    all_results = []
    for ind in population:
        if len(ind) > 0:
            cov, leak, exceed, length = evaluate(ind)
            all_results.append({
                'lines': ind,
                'n_lines': len(ind),
                'coverage_pct': cov,
                'leakage_pct': leak,
                'exceed_len': exceed,
                'total_len': length
            })
    
    return all_results

print("All solvers loaded successfully.")
