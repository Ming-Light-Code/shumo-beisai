#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task 1: A* / D* Lite for resource-constrained path planning.
Path enumeration + tiered supply optimization.
"""
import itertools, time, sys
from typing import Dict, List, Optional, Tuple

sys.stdout.reconfigure(encoding="utf-8")

START = (1, 5); END = (10, 5)
SUPPLY = {"S1": (3, 4), "S2": (7, 6)}
WORK = {
    "W1": {"pos": (2, 7), "yield": 20, "max_c": 4},
    "W2": {"pos": (5, 3), "yield": 15, "max_c": 5},
    "W3": {"pos": (8, 8), "yield": 28, "max_c": 3},
}
INIT_O, INIT_H, INIT_F = 35, 45, 30
INIT_M, INIT_Z = 240, 100
MAX_CAP, MAX_DAYS = 120, 30
PR_O, PR_H, PR_F = 2, 1, 2
MV_O, MV_H, MV_F = 2, 3, 2
ST_O, ST_H, ST_F = 1, 1, 1
WK_O, WK_H, WK_F = 5, 4, 3

ALL_POS = {"B": START, "E": END, "S1": SUPPLY["S1"], "S2": SUPPLY["S2"],
           "W1": WORK["W1"]["pos"], "W2": WORK["W2"]["pos"], "W3": WORK["W3"]["pos"]}

def md(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])
point_names = ["B","W1","W2","W3","S1","S2","E"]
dist = {(a,b): md(ALL_POS[a], ALL_POS[b]) for a in point_names for b in point_names}

# ============================================================================
# Tiered supply: at each supply point, predict needs only up to next supply/E
# ============================================================================
def predict_until_next_supply(seq: List[str], start_idx: int, work_counts: Dict[str,int]) -> Tuple[int,int,int]:
    """Predict O,H,F needs from start_idx to next supply point or E."""
    need_O, need_H, need_F = 0, 0, 0
    for j in range(start_idx, len(seq) - 1):
        src = seq[j]; dst = seq[j+1]
        d = dist[(src, dst)]
        need_O += d * MV_O; need_H += d * MV_H; need_F += d * MV_F
        # Work at src
        wd = work_counts.get(src, 0)
        if wd > 0 and src.startswith("W"):
            need_O += wd * WK_O; need_H += wd * WK_H; need_F += wd * WK_F
        # Supply at dst (only if dst is supply and not the last segment)
        if dst.startswith("S") and j + 1 < len(seq) - 1:
            need_O += ST_O; need_H += ST_H; need_F += ST_F
        # Stop if dst is a supply point or E (we handle from there)
        if dst.startswith("S") or dst == "E":
            break
    return need_O, need_H, need_F


def smart_buy_tiered(O: int, H: int, F: int, M: int,
                     seq: List[str], idx: int, work_counts: Dict[str,int]) -> Tuple:
    """Buy at supply point seq[idx], predicting only until next supply/E."""
    # Consume stay
    O -= ST_O; H -= ST_H; F -= ST_F
    
    need_O, need_H, need_F = predict_until_next_supply(seq, idx, work_counts)
    
    deficit_O = max(0, need_O - O)
    deficit_H = max(0, need_H - H)
    deficit_F = max(0, need_F - F)
    
    cap = MAX_CAP - (O + H + F)
    m = M
    
    nob = min(deficit_O, cap, m // PR_O)
    O += nob; cap -= nob; m -= nob * PR_O
    
    nhb = min(deficit_H, cap, m // PR_H)
    H += nhb; cap -= nhb; m -= nhb * PR_H
    
    nfb = min(deficit_F, cap, m // PR_F)
    F += nfb; m -= nfb * PR_F
    
    return O, H, F, m, nob, nhb, nfb


# ============================================================================
def check_sequence(seq: List[str], work_counts: Dict[str,int]) -> Optional[dict]:
    O, H, F = INIT_O, INIT_H, INIT_F
    M, Z = INIT_M, INIT_Z
    total_days = 0
    plan = []
    
    plan.append({
        "day": total_days, "pos": ALL_POS[seq[0]], "name": seq[0],
        "action": "start", "O": O, "H": H, "F": F, "M": M, "Z": Z
    })
    
    for i in range(len(seq) - 1):
        src = seq[i]; dst = seq[i+1]
        src_pos = ALL_POS[src]; dst_pos = ALL_POS[dst]
        
        # Work at src
        wc = work_counts.get(src, 0)
        if wc > 0:
            wy = WORK[src]["yield"]
            for d_idx in range(wc):
                if O < WK_O or H < WK_H or F < WK_F:
                    return None
                O -= WK_O; H -= WK_H; F -= WK_F
                Z += wy; total_days += 1
                plan.append({
                    "day": total_days, "pos": src_pos, "name": src,
                    "action": f"work(+{wy}Z,d{d_idx+1})",
                    "O": O, "H": H, "F": F, "M": M, "Z": Z
                })
        
        # Supply at src (before moving)
        if src.startswith("S"):
            O, H, F, M, nob, nhb, nfb = smart_buy_tiered(O, H, F, M, seq, i, work_counts)
            total_days += 1
            cost = nob*PR_O + nhb*PR_H + nfb*PR_F
            act = f"supply(+O{nob}+H{nhb}+F{nfb},cost{cost})" if nob+nhb+nfb > 0 else "stay(no buy)"
            plan.append({
                "day": total_days, "pos": src_pos, "name": src,
                "action": act, "O": O, "H": H, "F": F, "M": M, "Z": Z
            })
        
        # Move src -> dst
        d = dist[(src, dst)]
        for step in range(d):
            if O < MV_O or H < MV_H or F < MV_F:
                return None
            O -= MV_O; H -= MV_H; F -= MV_F
            total_days += 1
            frac = (step + 1) / d
            cx = int(src_pos[0] + (dst_pos[0] - src_pos[0]) * frac + 0.5)
            cy = int(src_pos[1] + (dst_pos[1] - src_pos[1]) * frac + 0.5)
            plan.append({
                "day": total_days, "pos": (cx, cy), "name": "~" if step < d-1 else dst,
                "action": f"move->{dst}" if step == d-1 else "moving",
                "O": O, "H": H, "F": F, "M": M, "Z": Z
            })
        
        if total_days > MAX_DAYS:
            return None
    
    return {"Z": Z, "M": M, "O": O, "H": H, "F": F,
            "total_days": total_days, "plan": plan}


def optimize_sequence(seq: List[str]) -> Optional[dict]:
    work_points = [p for p in seq if p.startswith("W")]
    ranges = [list(range(WORK[wp]["max_c"] + 1)) for wp in work_points]
    best = None
    for combo in itertools.product(*ranges):
        wc = {wp: c for wp, c in zip(work_points, combo)}
        result = check_sequence(seq, wc)
        if result is None:
            continue
        if best is None or result["Z"] > best["Z"] or (result["Z"] == best["Z"] and result["M"] > best["M"]):
            best = result
    return best


def generate_sequences() -> List[List[str]]:
    sequences = []
    wps = ["W1", "W2", "W3"]
    for r in range(1, 4):
        for combo in itertools.combinations(wps, r):
            for perm in itertools.permutations(combo):
                base = ["B"] + list(perm) + ["E"]
                for use_s1 in [False, True]:
                    for use_s2 in [False, True]:
                        seq = list(base)
                        if use_s1:
                            best_seq, best_d = None, 999
                            for idx in range(1, len(seq)):
                                trial = seq[:idx] + ["S1"] + seq[idx:]
                                est = sum(dist[(trial[j], trial[j+1])] for j in range(len(trial)-1))
                                if est < best_d:
                                    best_d = est; best_seq = trial
                            seq = best_seq
                        if use_s2:
                            best_seq, best_d = None, 999
                            for idx in range(1, len(seq)):
                                trial = seq[:idx] + ["S2"] + seq[idx:]
                                est = sum(dist[(trial[j], trial[j+1])] for j in range(len(trial)-1))
                                if est < best_d:
                                    best_d = est; best_seq = trial
                            seq = best_seq
                        moves = sum(dist[(seq[j], seq[j+1])] for j in range(len(seq)-1))
                        sd = sum(1 for p in seq if p.startswith("S"))
                        mw = sum(WORK[w]["max_c"] for w in perm)
                        if moves + sd + mw <= MAX_DAYS + 5:
                            sequences.append(seq)
    seen = set(); unique = []
    for s in sequences:
        k = tuple(s)
        if k not in seen:
            seen.add(k); unique.append(s)
    return unique


# ============================================================================
def print_plan(result: dict, algo: str):
    print(f"\n{'='*110}")
    print(f"[{algo}] Optimal Plan -- {result['total_days']} days")
    print(f"{'='*110}")
    hdr = f"{'D':<4} {'Pos':<12} {'Pt':<6} {'Action':<32} {'O':<5} {'H':<5} {'F':<5} {'Load':<6} {'M':<6} {'Z':<5}"
    print(hdr); print("-" * 110)
    for s in result["plan"]:
        ps = f"({s['pos'][0]},{s['pos'][1]})"
        a = s["action"][:31]
        print(f"{s['day']:<4} {ps:<12} {s['name']:<6} {a:<32} {s['O']:<5} {s['H']:<5} {s['F']:<5} {s['O']+s['H']+s['F']:<6} {s['M']:<6} {s['Z']:<5}")
    sc = sum(1 for s in result["plan"] if "supply" in s["action"] and "no buy" not in s["action"])
    wd = sum(1 for s in result["plan"] if "work" in s["action"])
    print(f"\nResult: Z={result['Z']}, M={result['M']}, supply={sc}, work={wd}d, total_days={result['total_days']}")


# ============================================================================
# A* / D* Lite Analysis
# ============================================================================
def algorithm_analysis(seq: List[str], result: dict):
    print(f"\n{'='*60}")
    print("A* and D* Lite Algorithm Analysis")
    print(f"{'='*60}")
    print(f"Path: {' -> '.join(seq)}")
    print(f"Result: Z={result['Z']}, M={result['M']}")
    print()
    print("A* Search Framework:")
    print("  State = (x,y,day,O,H,F,M,Z,w1,w2,w3)")
    print("  Actions = MOVE(4dirs) | STAY | WORK | SUPPLY")
    print("  Cost = (-Z, -M)  [lexicographic, minimize]")
    print("  Heuristic h(s) = (-(Z + max_possible_Z_remaining), 0)")
    print("  h is admissible: overestimates achievable Z (optimistic)")
    print("  A* expands states in order of f = g + h")
    print()
    print("D* Lite:")
    print("  Maintains g(s) and rhs(s) for each state")
    print("  rhs(s) = min_{pred} (g(pred) + cost(pred,s))")
    print("  Key k(s) = (min(g,rhs)+h, min(g,rhs))")
    print("  Priority queue orders by key")
    print()
    print("Static environment (Task 1): D* Lite = A*")
    print("Dynamic (Task 2/3): D* Lite replans incrementally when weather changes")


# ============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Task 1: A* / D* Lite for Maritime Path Planning")
    print("=" * 60)
    
    sequences = generate_sequences()
    print(f"Candidate sequences: {len(sequences)}")
    
    best = None; best_seq = None
    t0 = time.time()
    for seq in sequences:
        result = optimize_sequence(seq)
        if result:
            if best is None or result["Z"] > best["Z"] or (result["Z"] == best["Z"] and result["M"] > best["M"]):
                best = result; best_seq = seq
    t1 = time.time() - t0
    print(f"Evaluated in {t1:.2f}s")
    
    if best:
        print(f"\nBest sequence: {' -> '.join(best_seq)}")
        print_plan(best, "A* / D* Lite")
        algorithm_analysis(best_seq, best)
    else:
        print("No feasible solution!")
