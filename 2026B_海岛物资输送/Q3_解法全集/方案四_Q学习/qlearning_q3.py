#!/usr/bin/env python3
"""Q3 Approach 4: Q-Learning with Function Approximation.

Since the state space is enormous, we use feature-based Q-learning
with a linear function approximator. The weather distribution provides
a natural simulator for iterative policy improvement.

State features (23 dims): normalized position, distances to all special
points, resource levels, time remaining, binary at-point flags.
Actions (7): Move(R/L/U/D), Stop, Work, Supply.
"""
import numpy as np
import os, random

ALL_XY = {'B':(1,15),'E':(30,15),'W1':(6,21),'W2':(15,9),'W3':(24,24),
          'S1':(12,16),'S2':(21,16)}
INIT_O,INIT_H,INIT_F = 100,150,100
INIT_M,INIT_Z = 750,200
LOAD_LIMIT,MAX_DAYS = 400,90
P_NORMAL,P_STORM = 0.8,0.2
CM_N = np.array([2,3,2]); CS_N = np.array([1,1,1]); CW_N = np.array([5,4,3])
CM_T = np.array([8,4,3]); CS_T = np.array([3,3,2]); CW_T = np.array([8,6,6])
WY = [20,15,28]; WM = [4,5,3]

def manhattan(p1,p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

class ShipEnv:
    """Environment for Q-Learning with 7 discrete actions."""
    def __init__(self):
        self.reset()
    def reset(self):
        self.x,self.y = 1,15
        self.O,self.H,self.F = INIT_O,INIT_H,INIT_F
        self.M,self.Z = INIT_M,INIT_Z
        self.day = 0; self.consec_work = 0; self.cur_work = None
        self.at_E = False
        return self._get_state()
    def _get_state(self):
        x_n = self.x/30.0; y_n = self.y/30.0
        d_E = manhattan((self.x,self.y),(30,15))/30.0
        d_W1 = manhattan((self.x,self.y),(6,21))/30.0
        d_W2 = manhattan((self.x,self.y),(15,9))/30.0
        d_W3 = manhattan((self.x,self.y),(24,24))/30.0
        d_S1 = manhattan((self.x,self.y),(12,16))/30.0
        d_S2 = manhattan((self.x,self.y),(21,16))/30.0
        O_n = self.O/400.0; H_n = self.H/400.0; F_n = self.F/400.0
        M_n = self.M/750.0; Z_n = self.Z/500.0
        day_n = self.day/90.0; time_left = (90-self.day)/90.0
        load_n = (self.O+self.H+self.F)/400.0
        aW1 = 1.0 if (self.x,self.y) == (6,21) else 0.0
        aW2 = 1.0 if (self.x,self.y) == (15,9) else 0.0
        aW3 = 1.0 if (self.x,self.y) == (24,24) else 0.0
        aS1 = 1.0 if (self.x,self.y) == (12,16) else 0.0
        aS2 = 1.0 if (self.x,self.y) == (21,16) else 0.0
        return np.array([x_n,y_n,d_E,d_W1,d_W2,d_W3,d_S1,d_S2,
                         O_n,H_n,F_n,M_n,Z_n,day_n,time_left,load_n,
                         aW1,aW2,aW3,aS1,aS2,1.0])
    def step(self, action):
        storm = np.random.random() < P_STORM
        cm,cs,cw = (CM_T,CS_T,CW_T) if storm else (CM_N,CS_N,CW_N)
        reward = 0.0
        if action == 0:
            if self.x < 30: self.x += 1
            self.O -= cm[0]; self.H -= cm[1]; self.F -= cm[2]
            self.consec_work = 0; self.cur_work = None
        elif action == 1:
            if self.x > 1: self.x -= 1
            self.O -= cm[0]; self.H -= cm[1]; self.F -= cm[2]
            self.consec_work = 0; self.cur_work = None
        elif action == 2:
            if self.y < 30: self.y += 1
            self.O -= cm[0]; self.H -= cm[1]; self.F -= cm[2]
            self.consec_work = 0; self.cur_work = None
        elif action == 3:
            if self.y > 1: self.y -= 1
            self.O -= cm[0]; self.H -= cm[1]; self.F -= cm[2]
            self.consec_work = 0; self.cur_work = None
        elif action == 4:
            self.O -= cs[0]; self.H -= cs[1]; self.F -= cs[2]
            self.consec_work = 0; self.cur_work = None
        elif action == 5:
            worked = False
            for wi,(wx,wy) in enumerate([(6,21),(15,9),(24,24)]):
                if self.x == wx and self.y == wy and self.consec_work < WM[wi]:
                    self.Z += WY[wi]; self.consec_work += 1
                    self.cur_work = wi; reward += 5.0; worked = True; break
            self.O -= cw[0]; self.H -= cw[1]; self.F -= cw[2]
        elif action == 6:
            d = manhattan((self.x,self.y),(30,15))
            e_move = np.array([3.2,3.2,2.2]); need = e_move * d
            bo = max(0, int(need[0] - max(0, self.O)))
            bh = max(0, int(need[1] - max(0, self.H)))
            bf = max(0, int(need[2] - max(0, self.F)))
            cost = bo*2 + bh + bf*2
            if self.M >= cost and self.O+bo+self.H+bh+self.F+bf <= LOAD_LIMIT:
                self.O += bo; self.H += bh; self.F += bf; self.M -= cost
            self.O -= cs[0]; self.H -= cs[1]; self.F -= cs[2]
        self.day += 1
        done = False
        if self.x == 30 and self.y == 15:
            self.at_E = True; done = True
            reward += 50.0 + (self.Z/100.0) + (self.M/100.0)
        elif self.day >= MAX_DAYS:
            done = True; reward -= 20.0
        elif self.O < 0 or self.H < 0 or self.F < 0 or self.M < 0:
            done = True; reward -= 50.0
        if self.O + self.H + self.F > LOAD_LIMIT:
            done = True; reward -= 30.0
        reward -= 0.01
        return self._get_state(), reward, done

class LinearQAgent:
    """Linear function approximation Q-Learning."""
    def __init__(self, state_dim=22, n_actions=7, lr=0.005, gamma=0.95, epsilon=0.25):
        self.W = np.zeros((n_actions, state_dim))
        self.n_actions = n_actions
        self.lr = lr; self.gamma = gamma; self.epsilon = epsilon
    def get_q(self, state, action):
        return np.dot(self.W[action], state)
    def act(self, state, training=True):
        if training and np.random.random() < self.epsilon:
            return np.random.randint(self.n_actions)
        return int(np.argmax([self.get_q(state, a) for a in range(self.n_actions)]))
    def update(self, state, action, reward, next_state, done):
        q = self.get_q(state, action)
        if done:
            target = reward
        else:
            target = reward + self.gamma * max(self.get_q(next_state, a) for a in range(self.n_actions))
        self.W[action] += self.lr * (target - q) * state

def train_agent(episodes=2000):
    env = ShipEnv(); agent = LinearQAgent()
    rewards_hist = []
    for ep in range(episodes):
        state = env.reset(); total_r = 0.0
        for _ in range(MAX_DAYS + 10):
            action = agent.act(state, training=True)
            next_state, reward, done = env.step(action)
            agent.update(state, action, reward, next_state, done)
            total_r += reward; state = next_state
            if done: break
        rewards_hist.append(total_r)
        if (ep+1) % 200 == 0:
            avg_r = np.mean(rewards_hist[-100:])
            succ = 0
            for _ in range(20):
                s = env.reset()
                for __ in range(MAX_DAYS+10):
                    a = agent.act(s, training=False)
                    ns, r, d = env.step(a)
                    if env.at_E: succ += 1
                    if d: break
                    s = ns
            print(f"Ep {ep+1:5d}/{episodes}: avg_R={avg_r:.1f} eval_ok={succ}/20 eps={agent.epsilon:.3f}")
        if ep > 400:
            agent.epsilon = max(0.05, agent.epsilon * 0.998)
    return agent, rewards_hist

def test_agent(agent, n_tests=100):
    env = ShipEnv(); success = 0; Zv = []; Mv = []; best_Z = -1; best_log = None
    for _ in range(n_tests):
        s = env.reset(); log = []
        for __ in range(MAX_DAYS + 10):
            a = agent.act(s, training=False)
            ns, r, done = env.step(a)
            log.append({'day':env.day,'x':env.x,'y':env.y,'O':env.O,
                        'H':env.H,'F':env.F,'M':env.M,'Z':env.Z,'act':a})
            if done:
                if env.at_E:
                    success += 1; Zv.append(env.Z); Mv.append(env.M)
                    if env.Z > best_Z:
                        best_Z = env.Z; best_log = log
                break
            s = ns
    return {
        'success_rate': success/n_tests, 'successes': success,
        'Z_mean': np.mean(Zv) if Zv else 0,
        'Z_std': np.std(Zv) if Zv else 0,
        'Z_max': np.max(Zv) if Zv else 0,
        'M_mean': np.mean(Mv) if Mv else 0,
    }, best_log

def main():
    print("=" * 60)
    print("  Q3 Approach 4: Q-Learning with Function Approximation")
    print("=" * 60)
    print("7 actions: R,L,U,D,Stop,Work,Supply")
    print("23 state features, linear approximator")
    print()
    print("Training 2000 episodes...")
    agent, rewards = train_agent(2000)
    print()
    print("Testing on 100 episodes...")
    results, best_log = test_agent(agent, 100)
    print()
    print("--- Test Results ---")
    print(f"Success rate: {results['success_rate']:.1%}")
    print(f"Z: mean={results['Z_mean']:.0f} std={results['Z_std']:.0f} max={results['Z_max']}")
    print(f"M: mean={results['M_mean']:.0f}")
    if best_log:
        am = {0:'R',1:'L',2:'U',3:'D',4:'Stop',5:'Work',6:'Supply'}
        print()
        print("--- Best trajectory (first 15 steps) ---")
        for e in best_log[:15]:
            print(f"  D{e['day']:2d}: ({e['x']:2d},{e['y']:2d}) O={e['O']:.0f} H={e['H']:.0f} F={e['F']:.0f} M={e['M']:.0f} Z={e['Z']:.0f} | {am[e['act']]}")

if __name__ == '__main__':
    main()
