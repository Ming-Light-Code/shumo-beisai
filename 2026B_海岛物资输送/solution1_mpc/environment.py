"""
问题3 共享仿真环境
30x30 网格, 天气随机, 资源消耗, 补给, 作业
"""
from dataclasses import dataclass, field
from typing import Tuple, List, Optional, Dict
import random

GRID_SIZE = 30
START = (1, 15)
END = (30, 15)
SUPPLY = {"S1": (12, 16), "S2": (21, 16)}
WORK = {"W1": (6, 21), "W2": (15, 9), "W3": (24, 24)}
WORK_YIELD = {"W1": 20, "W2": 15, "W3": 28}
WORK_MAX_CONSECUTIVE = {"W1": 4, "W2": 5, "W3": 3}

INIT_O, INIT_H, INIT_F = 100, 150, 100
INIT_M = 750
INIT_Z = 200
LOAD_CAP = 400
TIME_LIMIT = 90

PRICE_O, PRICE_H, PRICE_F = 2, 1, 2
P_NORMAL = 0.8
P_STORM = 0.2

CONSUME = {
    "normal": {"move": (2,3,2), "stay": (1,1,1), "work": (5,4,3)},
    "storm":  {"move": (8,4,3), "stay": (3,3,2), "work": (8,6,6)},
}

@dataclass
class ShipState:
    x: int; y: int; O: int; H: int; F: int; M: int; Z: int; day: int
    work_days: Dict[str,int] = field(default_factory=lambda: {"W1":0,"W2":0,"W3":0})
    terminated: bool = False
    @property
    def pos(self): return (self.x, self.y)
    @property
    def load(self): return self.O + self.H + self.F
    def copy(self):
        return ShipState(x=self.x,y=self.y,O=self.O,H=self.H,F=self.F,
                         M=self.M,Z=self.Z,day=self.day,
                         work_days=dict(self.work_days),terminated=self.terminated)

class Environment:
    def __init__(self, seed=42):
        self.rng = random.Random(seed)
        self.state = ShipState(x=START[0],y=START[1],O=INIT_O,H=INIT_H,F=INIT_F,
                               M=INIT_M,Z=INIT_Z,day=1)
        self.weather_history = []
        self.log = []

    def reset(self, seed=None):
        if seed is not None:
            self.rng = random.Random(seed)
        self.state = ShipState(x=START[0],y=START[1],O=INIT_O,H=INIT_H,F=INIT_F,
                               M=INIT_M,Z=INIT_Z,day=1)
        self.weather_history = []
        self.log = []
        return self.state.copy()

    def sample_weather(self):
        return "normal" if self.rng.random() < P_NORMAL else "storm"

    def get_pos_type(self, x, y):
        pos = (x, y)
        if pos == START: return "start"
        if pos == END: return "end"
        if pos in SUPPLY.values(): return "supply"
        if pos in WORK.values(): return "work"
        return "plain"

    def get_work_name(self, x, y):
        pos = (x, y)
        for name, wp in WORK.items():
            if pos == wp: return name
        return None

    def get_supply_name(self, x, y):
        pos = (x, y)
        for name, sp in SUPPLY.items():
            if pos == sp: return name
        return None

    def legal_actions(self, state):
        actions = []
        x, y = state.x, state.y
        moves = [(-1,0,"up"),(1,0,"down"),(0,-1,"left"),(0,1,"right")]
        for dx, dy, name in moves:
            nx, ny = x+dx, y+dy
            if 1 <= nx <= GRID_SIZE and 1 <= ny <= GRID_SIZE:
                actions.append("move_"+name)
        actions.append("stay")
        wn = self.get_work_name(x, y)
        if wn and state.work_days[wn] < WORK_MAX_CONSECUTIVE[wn]:
            actions.append("work")
        if self.get_supply_name(x, y):
            actions.append("replenish")
        return actions

    def step(self, state, action, weather, replenish=None):
        ns = state.copy()
        # replenish
        if action == "replenish" and replenish:
            bo, bh, bf = replenish
            cost = bo*PRICE_O + bh*PRICE_H + bf*PRICE_F
            if cost > ns.M or ns.load+bo+bh+bf > LOAD_CAP:
                return state.copy(), -10000, True
            ns.O += bo; ns.H += bh; ns.F += bf; ns.M -= cost
        # consume
        consume = CONSUME[weather]
        if action.startswith("move_"):
            cO, cH, cF = consume["move"]
            dirs = {"move_up":(-1,0),"move_down":(1,0),"move_left":(0,-1),"move_right":(0,1)}
            dx, dy = dirs[action]
            ns.x += dx; ns.y += dy
            for wn in WORK:
                if (ns.x, ns.y) != WORK[wn]:
                    ns.work_days[wn] = 0
        elif action == "work":
            cO, cH, cF = consume["work"]
            wn = self.get_work_name(ns.x, ns.y)
            if wn:
                ns.work_days[wn] += 1
                ns.Z += WORK_YIELD[wn]
        else:
            cO, cH, cF = consume["stay"]
        ns.O -= cO; ns.H -= cH; ns.F -= cF
        ns.day += 1
        # legality
        if ns.O < 0 or ns.H < 0 or ns.F < 0 or ns.M < 0:
            state_copy = state.copy()
            state_copy.terminated = True
            return state_copy, -10000, True
        if ns.load > LOAD_CAP:
            state_copy = state.copy()
            state_copy.terminated = True
            return state_copy, -10000, True
        # terminal
        if (ns.x, ns.y) == END:
            ns.terminated = True
            return ns, ns.Z + ns.M*1e-4, True
        if ns.day > TIME_LIMIT:
            ns.terminated = True
            return ns, -5000, True
        return ns, 0.0, False

    def expected_consume(self, action):
        if action.startswith("move_"): key = "move"
        elif action == "work": key = "work"
        else: key = "stay"
        n, s = CONSUME["normal"][key], CONSUME["storm"][key]
        return (P_NORMAL*n[0]+P_STORM*s[0], P_NORMAL*n[1]+P_STORM*s[1], P_NORMAL*n[2]+P_STORM*s[2])

    def manhattan(self, p1, p2):
        return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

    def simulate_full(self, policy, seed=42):
        state = self.reset(seed)
        log = []
        while not state.terminated and state.day <= TIME_LIMIT:
            weather = self.sample_weather()
            action, replenish = policy(state, weather)
            log.append({"day":state.day,"pos":state.pos,"weather":weather,
                        "action":action,"replenish":replenish,
                        "O":state.O,"H":state.H,"F":state.F,"M":state.M,"Z":state.Z})
            state, reward, done = self.step(state, action, weather, replenish)
            if done:
                break
        return {"success":state.terminated and (state.x,state.y)==END,
                "final_Z":state.Z,"final_M":state.M,"final_day":state.day,
                "final_O":state.O,"final_H":state.H,"final_F":state.F,"log":log}
