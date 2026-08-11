
import numpy as np
import warnings
warnings.filterwarnings('ignore')

NM_TO_M = 1852.0
TAN60 = np.sqrt(3)
SEA_Y_NM = 5.0
SEA_X_NM = 4.0
GRID_RES = 0.02
OVERLAP_MAX = 0.20

def load_data():
    depth = np.load(r'C:\Users\ming\Desktop\数模备赛\depth_data.npy')
    yc = np.arange(depth.shape[0]) * GRID_RES
    xc = np.arange(depth.shape[1]) * GRID_RES
    return depth, yc, xc

def bilinear(depth, yc, xc, yq, xq):
    i = int(np.clip(np.searchsorted(yc, yq) - 1, 0, len(yc) - 2))
    j = int(np.clip(np.searchsorted(xc, xq) - 1, 0, len(xc) - 2))
    y0, y1 = float(yc[i]), float(yc[i+1])
    x0, x1 = float(xc[j]), float(xc[j+1])
    wy = (yq - y0) / (y1 - y0) if y1 != y0 else 0.0
    wx = (xq - x0) / (x1 - x0) if x1 != x0 else 0.0
    a = float(depth[i,j]); b = float(depth[i,j+1])
    c = float(depth[i+1,j]); d = float(depth[i+1,j+1])
    return (1-wy)*(1-wx)*a + (1-wy)*wx*b + wy*(1-wx)*c + wy*wx*d

def strip_hw(depth, yc, xc, yq, xvals):
    return np.array([bilinear(depth, yc, xc, yq, float(x)) for x in xvals]) * TAN60 / NM_TO_M

# ----- COVERAGE METRICS -----
def metrics(depth, yc, xc, lines, direction='EW', nx=120):
    if len(lines) == 0:
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
    covered = 0.0
    exceed_len = 0.0
    for xi in range(nx):
        x = x_eval[xi]
        intervals = []
        for yk in lines:
            if direction == 'EW':
                w = bilinear(depth, yc, xc, yk, x) * TAN60 / NM_TO_M
            else:
                w = bilinear(depth, yc, xc, x, yk) * TAN60 / NM_TO_M
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
        covered += sum(hi - lo for lo, hi in merged) * dx
    m = len(lines)
    for k in range(m - 1):
        yk = lines[k]; yj = lines[k+1]
        for xi in range(nx):
            x = x_eval[xi]
            if direction == 'EW':
                wk = bilinear(depth, yc, xc, yk, x) * TAN60 / NM_TO_M
                wj = bilinear(depth, yc, xc, yj, x) * TAN60 / NM_TO_M
            else:
                wk = bilinear(depth, yc, xc, x, yk) * TAN60 / NM_TO_M
                wj = bilinear(depth, yc, xc, x, yj) * TAN60 / NM_TO_M
            if wk <= 0 or wj <= 0:
                continue
            ov = (yk + wk) - (yj - wj)
            if ov > 0:
                tw = wk + wj
                rate = ov / tw if tw > 0 else 0
                if rate > OVERLAP_MAX:
                    exceed_len += (ov - OVERLAP_MAX * tw) * dx
    leak_pct = max(0, 100.0 * (total_area - covered) / total_area)
    cov_pct = 100.0 * covered / total_area
    total_len = len(lines) * line_len
    return cov_pct, leak_pct, exceed_len, total_len

# ----- GREEDY -----
def solve_greedy(depth, yc, xc, direction='EW'):
    if direction == 'EW':
        x_eval = np.linspace(0, SEA_X_NM, 120)
        max_pos = SEA_Y_NM
    else:
        x_eval = np.linspace(0, SEA_Y_NM, 120)
        max_pos = SEA_X_NM
    lines = []
    current = 0.0
    step = 0.005
    # First line
    for pos in np.arange(step, max_pos, step):
        w = strip_hw(depth, yc, xc, pos, x_eval)
        if pos - np.min(w) <= 0:
            lines.append(pos)
            current = pos + np.min(w)
            break
    # Subsequent lines
    for _ in range(500):
        if current >= max_pos:
            break
        best = None
        best_gap = 999.0
        for pos in np.arange(current + step, min(max_pos + step, current + 0.3), step):
            w = strip_hw(depth, yc, xc, pos, x_eval)
            wmin = np.min(w)
            if pos - wmin <= current and pos - wmin >= current - 0.03:
                gap = current - (pos - wmin)
                if gap < best_gap:
                    best_gap = gap
                    best = pos
        if best is None:
            if lines:
                w = strip_hw(depth, yc, xc, lines[-1], x_eval)
                current = lines[-1] + np.min(w)
            continue
        lines.append(best)
        w = strip_hw(depth, yc, xc, best, x_eval)
        current = best + np.min(w)
    # Ensure boundary coverage
    if lines:
        for pos in np.arange(max_pos, lines[-1], -step):
            w = strip_hw(depth, yc, xc, pos, x_eval)
            if pos + np.min(w) >= max_pos and pos not in lines:
                lines.append(pos)
                break
    return np.array(sorted(set(lines)))

# ----- DAG-DP -----
def solve_dp(depth, yc, xc, direction='EW', alpha=0.5):
    if direction == 'EW':
        x_eval = np.linspace(0, SEA_X_NM, 80)
        max_pos = SEA_Y_NM
        line_len = SEA_X_NM
    else:
        x_eval = np.linspace(0, SEA_Y_NM, 80)
        max_pos = SEA_X_NM
        line_len = SEA_Y_NM
    step = 0.01
    cand = np.arange(step, max_pos, step)
    N = len(cand)
    min_w = np.zeros(N)
    for i, y in enumerate(cand):
        w = strip_hw(depth, yc, xc, y, x_eval)
        min_w[i] = np.min(w)
    INF = 1e10
    dp = np.full(N, INF)
    prev = np.full(N, -1, dtype=int)
    for i in range(N):
        if cand[i] - min_w[i] <= 0:
            dp[i] = line_len + alpha * max(0, cand[i] - min_w[i])
    for j in range(N):
        for i in range(j):
            if dp[i] >= INF:
                continue
            d = cand[j] - cand[i]
            if d > min_w[i] + min_w[j]:
                continue
            ov = min_w[i] + min_w[j] - d
            exc = max(0, ov - OVERLAP_MAX * (min_w[i] + min_w[j]))
            c = line_len + alpha * exc
            if dp[i] + c < dp[j]:
                dp[j] = dp[i] + c
                prev[j] = i
    best_end = -1
    best_cost = INF
    for i in range(N):
        if cand[i] + min_w[i] >= max_pos and dp[i] < best_cost:
            best_cost = dp[i]
            best_end = i
    if best_end < 0:
        return np.array([])
    path = []
    cur = best_end
    while cur >= 0:
        path.append(cand[cur])
        cur = prev[cur]
    path.reverse()
    return np.array(path)

# ----- SIMULATED ANNEALING -----
def solve_sa(depth, yc, xc, init_lines, direction='EW', iters=1500):
    def cost(ls):
        if len(ls) < 2:
            return 1e10
        _, leak, exc, length = metrics(depth, yc, xc, ls, direction, nx=80)
        return leak * 500 + exc * 5 + length
    lines = np.array(sorted(init_lines))
    best_l = lines.copy()
    best_c = cost(best_l)
    cur_c = best_c
    T0 = 0.5; Te = 0.001
    cr = (Te / T0) ** (1.0 / iters)
    T = T0
    np.random.seed(42)
    max_pos = SEA_Y_NM if direction == 'EW' else SEA_X_NM
    for _ in range(iters):
        nl = lines.copy()
        op = np.random.randint(0, 3)
        if op == 0 and len(nl) > 1:
            idx = np.random.randint(0, len(nl))
            nl[idx] += np.random.normal(0, 0.015)
            nl = np.clip(nl, 0.01, max_pos - 0.01)
            nl = np.sort(nl)
        elif op == 1 and len(nl) > 1:
            nl = np.delete(nl, np.random.randint(0, len(nl)))
        elif op == 2:
            nl = np.sort(np.append(nl, np.random.uniform(0.01, max_pos - 0.01)))
        nc = cost(nl)
        delta = nc - cur_c
        if delta < 0 or np.random.random() < np.exp(-delta / max(T, 1e-8)):
            lines = nl; cur_c = nc
            if cur_c < best_c:
                best_c = cur_c; best_l = lines.copy()
        T *= cr
    return best_l

# ----- MULTI-LAMBDA DP -----
def solve_multi_dp(depth, yc, xc, direction='EW', alphas=None):
    if alphas is None:
        alphas = [0.1, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0]
    res = {}
    for a in alphas:
        ls = solve_dp(depth, yc, xc, direction, a)
        if len(ls) > 0:
            cv, lk, ex, ln = metrics(depth, yc, xc, ls, direction, nx=80)
            res[a] = {'lines': ls, 'n': len(ls), 'cov': cv, 'leak': lk, 'exceed': ex, 'len': ln}
    return res

# ----- NSGA-II -----
def solve_nsga2(depth, yc, xc, direction='EW', pop=40, gens=80):
    max_pos = SEA_Y_NM if direction == 'EW' else SEA_X_NM
    def random_sol():
        n = np.random.randint(5, 35)
        return np.sort(np.random.uniform(0.01, max_pos - 0.01, n))
    def evaluate(ind):
        if len(ind) == 0:
            return (0, 100, 1e10, 0)
        return metrics(depth, yc, xc, ind, direction, nx=60)
    np.random.seed(123)
    population = [random_sol() for _ in range(pop)]
    for gen in range(gens):
        fit = [evaluate(ind) for ind in population]
        objs = np.array([(f[1], f[2], f[3]) for f in fit])
        n = len(population)
        dom_count = np.zeros(n, dtype=int)
        dom_list = [[] for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                a, b = objs[i], objs[j]
                if (a[0] <= b[0] and a[1] <= b[1] and a[2] <= b[2]) and \
                   (a[0] < b[0] or a[1] < b[1] or a[2] < b[2]):
                    dom_list[i].append(j)
                elif (b[0] <= a[0] and b[1] <= a[1] and b[2] <= a[2]) and \
                     (b[0] < a[0] or b[1] < a[1] or b[2] < a[2]):
                    dom_count[i] += 1
        front1 = [i for i in range(n) if dom_count[i] == 0]
        crowd = np.zeros(n)
        if len(front1) > 2:
            for oi in range(3):
                si = sorted(front1, key=lambda x: objs[x][oi])
                crowd[si[0]] = crowd[si[-1]] = 1e10
                rng = objs[si[-1]][oi] - objs[si[0]][oi]
                if rng > 0:
                    for k in range(1, len(si) - 1):
                        crowd[si[k]] += (objs[si[k+1]][oi] - objs[si[k-1]][oi]) / rng
        if gen < gens - 1:
            new_pop = []
            for _ in range(pop // 2):
                pool = front1 if len(front1) >= 2 else list(range(n))
                t = np.random.choice(pool, min(4, len(pool)), replace=False)
                p1 = t[0] if crowd[t[0]] >= crowd[t[1]] else t[1]
                p2 = t[2] if len(t)>3 and crowd[t[2]] >= crowd[t[3]] else (t[2] if len(t)>2 else t[0])
                c1 = population[p1].copy(); c2 = population[p2].copy()
                if np.random.random() < 0.3:
                    c1 = np.sort(np.append(np.delete(c1, np.random.randint(0, max(1,len(c1))) % max(1,len(c1))),
                                   np.random.uniform(0.01, max_pos-0.01)))
                if np.random.random() < 0.3:
                    c2 = np.sort(np.append(np.delete(c2, np.random.randint(0, max(1,len(c2))) % max(1,len(c2))),
                                   np.random.uniform(0.01, max_pos-0.01)))
                new_pop.extend([c1, c2])
            population = new_pop[:pop]
    results = []
    for ind in population:
        if len(ind) > 0:
            cv, lk, ex, ln = evaluate(ind)
            results.append({'lines': ind, 'n': len(ind), 'cov': cv, 'leak': lk, 'exceed': ex, 'len': ln})
    return results

print('All functions defined.')
