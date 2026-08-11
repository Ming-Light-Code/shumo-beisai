import numpy as np
import time
import sys
sys.path.insert(0, r"C:\Users\ming\Desktop\数模备赛\task3_mdp")
from compressed_dp import p, solve_compressed_dp
from scenario_mpc import run_simulation, monte_carlo_validation, plot_route, save_results

print("=" * 60)
print("Task 3: Compressed DP + Scenario Tree MPC")
print("=" * 60)

print("\n[Step 1] Solving compressed DP...")
t0 = time.time()
V = solve_compressed_dp("dp_value.npy")
print(f"DP solved in {time.time()-t0:.1f}s")

print("\n[Step 2] Running MPC simulation...")
np.random.seed(2026)
weather = ["storm" if np.random.random() < p.p_storm else "normal" for _ in range(p.T)]
t0 = time.time()
result = run_simulation(weather, V, verbose=True)
print(f"Simulation in {time.time()-t0:.1f}s")

if result["feasible"]:
    print(f"\n=== Result ===")
    print(f"Arrival: day {result['arrival_day']}, Z={result['final_Z']}, M={result['final_M']}")
    plot_route(result["route"])
else:
    print(f"Failed at day {result.get('failure_day','?')}")

print("\n[Step 3] Monte Carlo validation...")
t0 = time.time()
mc = monte_carlo_validation(V, n=300, seed=42)
print(f"MC in {time.time()-t0:.1f}s")

print("\n[Step 4] Saving...")
save_results(result, mc)

print("\nDone!")
