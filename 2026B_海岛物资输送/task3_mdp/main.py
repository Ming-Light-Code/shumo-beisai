
# main.py - ??? MDP+???MPC ???
import numpy as np
import time
import sys
sys.path.insert(0, '.')

from compressed_dp import p, solve_compressed_dp
from simulation import run_simulation, monte_carlo_validation, plot_route, save_results

def main():
    print('=' * 60)
    print('Task 3: Compressed DP + Scenario Tree MPC')
    print('=' * 60)
    
    # Step 1: Solve compressed DP (offline)
    print('\n[Step 1] Solving compressed DP...')
    t0 = time.time()
    V = solve_compressed_dp('dp_value.npy')
    print(f'DP solved in {time.time() - t0:.1f}s')
    
    # Step 2: Run simulation with random weather (initial seed)
    print('\n[Step 2] Running MPC simulation...')
    np.random.seed(2026)
    weather = ['storm' if np.random.random() < p.p_storm else 'normal' for _ in range(p.T)]
    
    t0 = time.time()
    result = run_simulation(weather, V, verbose=True)
    print(f'Simulation completed in {time.time() - t0:.1f}s')
    
    if result['feasible']:
        print(f'\n=== Best Result ===')
        print(f'Arrival Day: {result[\"arrival_day\"]}')
        print(f'Final Z: {result[\"final_Z\"]}')
        print(f'Final M: {result[\"final_M\"]}')
        
        # Plot route
        plot_route(result['route'], 'task3_route_mdp.png')
    else:
        print(f'\nSimulation failed at day {result.get(\"failure_day\", \"?\")}')
    
    # Step 3: Monte Carlo validation
    print('\n[Step 3] Monte Carlo validation...')
    t0 = time.time()
    mc_result = monte_carlo_validation(V, n_samples=500, seed=42)
    print(f'MC completed in {time.time() - t0:.1f}s')
    
    # Step 4: Save results
    print('\n[Step 4] Saving results...')
    save_results(result, mc_result, 'task3_result_mdp.xlsx')
    
    print('\n' + '=' * 60)
    print('Done!')
    print('=' * 60)
    return result, mc_result

if __name__ == '__main__':
    main()
