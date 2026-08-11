
# simulation.py - ????????????
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import time
from compressed_dp import p, solve_compressed_dp
from scenario_mpc import MPCController, compute_consumption, is_work_point, is_supply_point, get_adjacent_positions

def run_simulation(weather_sequence, V, verbose=True):
    controller = MPCController(V)
    state = {
        'position': p.B,
        'O': p.init_O, 'H': p.init_H, 'F': p.init_F,
        'M': p.init_M, 'Z': p.init_Z,
        'consecutive_work': 0, 'work_point': 0,
    }
    daily_records = []
    route = [p.B]
    arrival_day = 0
    
    for day in range(1, p.T + 1):
        weather = weather_sequence[day - 1]
        if state['position'] == p.E and not arrival_day:
            arrival_day = day
        if state['position'] == p.E and arrival_day and day > arrival_day:
            break
        
        action, action_data = controller.decide_action(state, weather, day)
        if action is None:
            return {'feasible': False, 'failure_day': day, 'daily_records': daily_records}
        
        record = {
            'Day': day, 'Weather': weather,
            'StartX': state['position'][0], 'StartY': state['position'][1],
            'Action': '', 'EndX': state['position'][0], 'EndY': state['position'][1],
            'BuyO': 0, 'BuyH': 0, 'BuyF': 0, 'Gain': 0,
            'O': state['O'], 'H': state['H'], 'F': state['F'],
            'M': state['M'], 'Z': state['Z'],
        }
        
        if action == 'move':
            new_pos = action_data['target']
            cons = action_data['details']['consumption']
            record['Action'] = f'Move({new_pos[0]},{new_pos[1]})'
            record['EndX'], record['EndY'] = new_pos
            state['position'] = new_pos
            state['O'] -= cons[0]; state['H'] -= cons[1]; state['F'] -= cons[2]
            state['consecutive_work'] = 0; state['work_point'] = 0
            wp_new = is_work_point(new_pos)
            if wp_new is not None: state['work_point'] = wp_new + 1
            route.append(new_pos)
            if new_pos == p.E: arrival_day = day
        elif action == 'idle':
            cons = action_data['details']['consumption']
            record['Action'] = 'Stay'
            state['O'] -= cons[0]; state['H'] -= cons[1]; state['F'] -= cons[2]
            state['consecutive_work'] = 0; state['work_point'] = 0
        elif action == 'work':
            cons = action_data['details']['consumption']
            gain = action_data['details']['gain']
            record['Action'] = f'Work(+{gain})'
            record['Gain'] = gain
            state['O'] -= cons[0]; state['H'] -= cons[1]; state['F'] -= cons[2]
            state['Z'] += gain
            state['consecutive_work'] = action_data['details']['new_wc']
        elif action == 'buy':
            buy = action_data['details']['buy']
            cost = action_data['details']['cost']
            record['Action'] = f'Buy({buy[0]},{buy[1]},{buy[2]})'
            record['BuyO'], record['BuyH'], record['BuyF'] = buy
            state['O'] += buy[0]; state['H'] += buy[1]; state['F'] += buy[2]
            state['M'] -= cost
        
        record['O'], record['H'], record['F'] = state['O'], state['H'], state['F']
        record['M'], record['Z'] = state['M'], state['Z']
        daily_records.append(record)
        
        if state['O'] < 0 or state['H'] < 0 or state['F'] < 0 or state['M'] < 0:
            return {'feasible': False, 'failure_day': day, 'daily_records': daily_records}
        if state['O'] + state['H'] + state['F'] > p.capacity:
            return {'feasible': False, 'failure_day': day, 'daily_records': daily_records}
        
        if verbose and day % 15 == 0:
            print(f"  Day {day}: pos={state['position']}, Z={state['Z']}, M={state['M']}, O={state['O']}")
    
    return {
        'feasible': True if arrival_day > 0 else False,
        'arrival_day': arrival_day if arrival_day > 0 else p.T,
        'final_Z': state['Z'],
        'final_M': state['M'],
        'daily_records': daily_records,
        'route': route
    }

def monte_carlo_validation(V, n_samples=1000, seed=42):
    print(f'\n?????? ({n_samples} samples)...')
    np.random.seed(seed)
    successes = 0
    z_values = []
    m_values = []
    arrival_days = []
    
    for i in range(n_samples):
        weather = ['storm' if np.random.random() < p.p_storm else 'normal' for _ in range(p.T)]
        result = run_simulation(weather, V, verbose=False)
        if result['feasible']:
            successes += 1
            z_values.append(result['final_Z'])
            m_values.append(result['final_M'])
            arrival_days.append(result['arrival_day'])
        
        if (i + 1) % 200 == 0:
            print(f"  {i+1}/{n_samples}, success rate: {successes/(i+1):.3f}")
    
    success_rate = successes / n_samples
    # Wilson 95% confidence lower bound
    z = 1.96
    n = n_samples
    p_hat = success_rate
    denominator = 1 + z**2 / n
    center = (p_hat + z**2 / (2*n)) / denominator
    margin = z * np.sqrt(p_hat * (1-p_hat) / n + z**2 / (4*n**2)) / denominator
    lower95 = max(0, center - margin)
    
    print(f'\n=== Monte Carlo Results ===')
    print(f'Success rate: {success_rate:.4f} ({successes}/{n_samples})')
    print(f'95% Wilson lower bound: {lower95:.4f}')
    if z_values:
        print(f'Avg Z: {np.mean(z_values):.1f} +/- {np.std(z_values):.1f}')
        print(f'Avg M: {np.mean(m_values):.1f} +/- {np.std(m_values):.1f}')
        print(f'Avg arrival day: {np.mean(arrival_days):.1f} +/- {np.std(arrival_days):.1f}')
    
    return {
        'samples': n_samples, 'successes': successes,
        'success_rate': success_rate, 'lower95': lower95,
        'z_mean': np.mean(z_values) if z_values else 0,
        'z_std': np.std(z_values) if z_values else 0,
        'm_mean': np.mean(m_values) if m_values else 0,
        'm_std': np.std(m_values) if m_values else 0,
        'arrival_mean': np.mean(arrival_days) if arrival_days else 0,
        'arrival_std': np.std(arrival_days) if arrival_days else 0,
    }

def plot_route(route, output_path='task3_route_mdp.png'):
    fig, ax = plt.subplots(figsize=(12, 8))
    route_arr = np.array(route)
    ax.plot(route_arr[:, 0], route_arr[:, 1], '-o', linewidth=1.5, markersize=3, color='steelblue')
    ax.plot(p.B[0], p.B[1], 'go', markersize=12, label='B (Start)')
    ax.plot(p.E[0], p.E[1], 'ro', markersize=12, label='E (End)')
    for i, s in enumerate(p.S):
        ax.plot(s[0], s[1], 'bs', markersize=10, label=f'S{i+1}' if i == 0 else '')
    for i, w in enumerate(p.W):
        ax.plot(w[0], w[1], 'm^', markersize=10, label=f'W{i+1}' if i == 0 else '')
    ax.set_xlim(0.5, 30.5)
    ax.set_ylim(0.5, 30.5)
    ax.set_xlabel('X'); ax.set_ylabel('Y')
    ax.set_title('Task 3: MDP+Scenario Tree Route')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    ax.set_aspect('equal')
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f'Route plot saved to {output_path}')

def save_results(result, mc_result, output_path='task3_result_mdp.xlsx'):
    if not result['feasible']:
        print('No feasible solution; result not saved.')
        return
    df = pd.DataFrame(result['daily_records'])
    cols_order = ['Day', 'Weather', 'StartX', 'StartY', 'Action', 'EndX', 'EndY',
                  'BuyO', 'BuyH', 'BuyF', 'Gain', 'O', 'H', 'F', 'M', 'Z']
    df = df[[c for c in cols_order if c in df.columns]]
    with pd.ExcelWriter(output_path, engine='openpyxl') as writer:
        df.to_excel(writer, sheet_name='DailyPlan', index=False)
        summary = pd.DataFrame({
            'Metric': ['Arrival Day', 'Final Z', 'Final M', 'MC Success Rate', 'MC 95% Lower'],
            'Value': [result['arrival_day'], result['final_Z'], result['final_M'],
                      f"{mc_result['success_rate']:.4f}", f"{mc_result['lower95']:.4f}"]
        })
        summary.to_excel(writer, sheet_name='Summary', index=False)
    print(f'Results saved to {output_path}')
