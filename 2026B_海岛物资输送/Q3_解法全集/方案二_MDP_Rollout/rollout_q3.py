#!/usr/bin/env python3
"""
Q3 Approach 2: MDP + Rollout Strategy.

Formulates the problem as a finite-horizon MDP. Uses a base policy
(from Approach 1's MILP result) and improves it via rollout:
at each state, simulate N trajectories for each candidate action
using the base policy for the remainder, and pick the action
with the best expected Z (ties broken by M).
"""
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '方案一_骨架枚举MILP'))
from simulate_q3 import *

@dataclass
class State:
    x:int;y:int;O:int;H:int;F:int;M:int;Z:int
    day:int;consec_work:int;cur_work:Optional[int]

    def key(self):
        return (self.x,self.y,self.O,self.H,self.F,self.M,self.Z,
                self.day,self.consec_work,self.cur_work or -1)

class ShipFull:
    def __init__(self,s:State=None):
        if s:
            self.x,self.y=s.x,s.y
            self.O,self.H,self.F=s.O,s.H,s.F
            self.M,self.Z=s.M,s.Z
            self.day=s.day
            self.consec_work=s.consec_work
            self.cur_work=s.cur_work
        else:
            self.x,self.y=ALL_XY['B']
            self.O,self.H,self.F=INIT_O,INIT_H,INIT_F
            self.M,self.Z=INIT_M,INIT_Z
            self.day=0;self.consec_work=0;self.cur_work=None
    @property
    def load(self): return self.O+self.H+self.F
    @property
    def pos(self): return (self.x,self.y)
    def alive(self): return self.O>=0 and self.H>=0 and self.F>=0 and self.M>=0
    def to_state(self):
        return State(self.x,self.y,self.O,self.H,self.F,self.M,self.Z,
                     self.day,self.consec_work,self.cur_work)

def get_actions(ship:ShipFull):
    """Return list of legal actions: (name, new_ship) pairs."""
    actions=[]
    # Move actions
    for dx,dy,dn in [(1,0,'R'),(-1,0,'L'),(0,1,'U'),(0,-1,'D')]:
        nx,ny=ship.x+dx,ship.y+dy
        if 1<=nx<=30 and 1<=ny<=30:
            ns=ShipFull(ship.to_state());ns.x=nx;ns.y=ny
            ns.consec_work=0;ns.cur_work=None
            actions.append((f'Move{dn}',ns))
    # Stay
    ns_stay=ShipFull(ship.to_state())
    ns_stay.consec_work=0;ns_stay.cur_work=None
    actions.append(('Stay',ns_stay))
    # Work (if at work point)
    for wi,(wx,wy) in enumerate([ALL_XY['W1'],ALL_XY['W2'],ALL_XY['W3']]):
        if ship.pos==(wx,wy) and ship.consec_work<WM[wi]:
            ns_w=ShipFull(ship.to_state())
            ns_w.Z+=WY[wi];ns_w.consec_work+=1;ns_w.cur_work=wi
            actions.append((f'Work@W{wi+1}',ns_w))
    # Supply (at supply point)
    if ship.pos in [ALL_XY['S1'],ALL_XY['S2']]:
        actions.append(('Supply',ship))  # Simplified
    return actions

def apply_weather(ship:ShipFull,action_name:str,storm:bool):
    """Apply consumption based on action and weather."""
    cm,cs,cw=(CM_T,CS_T,CW_T) if storm else (CM_N,CS_N,CW_N)
    if action_name.startswith('Move'):
        ship.O-=cm[0];ship.H-=cm[1];ship.F-=cm[2]
    elif action_name=='Stay':
        ship.O-=cs[0];ship.H-=cs[1];ship.F-=cs[2]
    elif action_name.startswith('Work'):
        ship.O-=cw[0];ship.H-=cw[1];ship.F-=cw[2]
    else:
        ship.O-=cs[0];ship.H-=cs[1];ship.F-=cs[2]
    ship.day+=1

def base_policy(ship:ShipFull,plan:MacroPlan,storm:bool,si:int,next_tgt:str,
                w1r:int,str_rem:int,w2r:int,bought:bool):
    """Base policy from Approach 1 macro plan."""
    at_target=(ship.pos==ALL_XY.get(next_tgt,(-1,-1)))
    if not at_target:
        safe_load=0.3*LOAD_LIMIT
        if storm and ship.load<safe_load:
            return ('Stay',si,next_tgt,w1r,str_rem,w2r,bought)
        tp=ALL_XY[next_tgt]
        dx=np.sign(tp[0]-ship.x);dy=np.sign(tp[1]-ship.y)
        if abs(tp[0]-ship.x)>=abs(tp[1]-ship.y) and dx!=0:
            dn='R' if dx>0 else 'L'
        elif dy!=0:
            dn='U' if dy>0 else 'D'
        else:
            dn='R' if dx>0 else 'L'
        return (f'Move{dn}',si,next_tgt,w1r,str_rem,w2r,bought)
    seg=plan.segments[si]
    if next_tgt in ['S1','S2']:
        if not bought:
            return ('Supply',si,next_tgt,w1r,str_rem,w2r,True)
        else:
            si2=si+1
            if si2<len(plan.segments):
                return ('Depart',si2,plan.segments[si2].target,
                        plan.segments[si2].w1,plan.segments[si2].stop,
                        plan.segments[si2].w2,False)
            else:
                return ('Stay',si,next_tgt,w1r,str_rem,w2r,bought)
    elif next_tgt in ['W1','W2','W3']:
        wi=['W1','W2','W3'].index(next_tgt)
        if w1r>0:
            return (f'Work@W{wi+1}',si,next_tgt,w1r-1,str_rem,w2r,bought)
        elif str_rem>0:
            return ('Stay',si,next_tgt,w1r,str_rem-1,w2r,bought)
        elif w2r>0:
            return (f'Work@W{wi+1}',si,next_tgt,w1r,str_rem,w2r-1,bought)
        else:
            si2=si+1
            if si2<len(plan.segments):
                return ('Depart',si2,plan.segments[si2].target,
                        plan.segments[si2].w1,plan.segments[si2].stop,
                        plan.segments[si2].w2,False)
            else:
                return ('Stay',si,next_tgt,w1r,str_rem,w2r,bought)
    elif next_tgt=='E':
        return ('Arrive',si,next_tgt,w1r,str_rem,w2r,bought)
    return ('Stay',si,next_tgt,w1r,str_rem,w2r,bought)

def rollout_eval(ship:ShipFull,plan:MacroPlan,action_name:str,
                 si:int,next_tgt:str,w1r:int,str_rem:int,w2r:int,
                 bought:bool,n_rollouts:int=20):
    """Evaluate an action via rollout."""
    Z_total=0.0;M_total=0.0;success_count=0
    for _ in range(n_rollouts):
        ns=ShipFull(ship.to_state())
        storm=np.random.random()<P_STORM
        apply_weather(ns,action_name,storm)
        if not ns.alive():
            continue
        # Check arrival
        if action_name=='Arrive' and ns.pos==ALL_XY['E']:
            Z_total+=ns.Z;M_total+=ns.M;success_count+=1
            continue
        # Simulate remaining with base policy
        si2,nt2,w1r2,str2,w2r2,bt2=si,next_tgt,w1r,str_rem,w2r,bought
        if action_name=='Depart':
            si2=si+1
            if si2<len(plan.segments):
                nt2=plan.segments[si2].target
                w1r2=plan.segments[si2].w1
                str2=plan.segments[si2].stop
                w2r2=plan.segments[si2].w2
                bt2=False
        elif action_name.startswith('Work') and w1r>0:
            w1r2-=1
        elif action_name.startswith('Work') and w2r>0:
            w2r2-=1
        elif action_name=='Stay' and str_rem>0:
            str2-=1
        elif action_name=='Supply':
            bt2=True
            bo=plan.segments[si].buy_O;bh=plan.segments[si].buy_H;bf=plan.segments[si].buy_F
            cost=bo*2+bh+bf*2
            if ns.M>=cost:
                ns.O+=bo;ns.H+=bh;ns.F+=bf;ns.M-=cost
        # Continue rollout
        for _ in range(MAX_DAYS-ns.day):
            if ns.pos==ALL_XY['E']:
                Z_total+=ns.Z;M_total+=ns.M;success_count+=1
                break
            storm=np.random.random()<P_STORM
            act,si2,nt2,w1r2,str2,w2r2,bt2=base_policy(
                ns,plan,storm,si2,nt2,w1r2,str2,w2r2,bt2)
            if act=='Arrive':
                Z_total+=ns.Z;M_total+=ns.M;success_count+=1
                break
            ns_after=ShipFull(ns.to_state())
            if act.startswith('Move'):
                d={'R':(1,0),'L':(-1,0),'U':(0,1),'D':(0,-1)}[act[-1]]
                ns_after.x+=d[0];ns_after.y+=d[1]
                ns_after.consec_work=0;ns_after.cur_work=None
            elif act.startswith('Work'):
                wi=int(act[-1])-1
                ns_after.Z+=WY[wi];ns_after.consec_work+=1;ns_after.cur_work=wi
            elif act=='Stay':
                ns_after.consec_work=0;ns_after.cur_work=None
            apply_weather(ns_after,act,storm)
            if not ns_after.alive():
                break
            ns=ns_after
    if success_count==0:
        return -1e9,-1e9
    return Z_total/success_count,M_total/success_count

def run_rollout(plan:MacroPlan,n_rollouts:int=20):
    """Execute rollout policy from start to finish."""
    np.random.seed(42)
    ship=ShipFull()
    si=0;nt=plan.segments[0].target
    w1r=plan.segments[0].w1;str_rem=plan.segments[0].stop
    w2r=plan.segments[0].w2;bought=False
    log=[]
    for _ in range(MAX_DAYS):
        if ship.pos==ALL_XY['E']:
            break
        actions=get_actions(ship)
        # Filter to reasonable actions
        best_a=None;best_z=-1e9;best_m=-1e9
        for a_name,a_ship in actions:
            z,m=rollout_eval(ship,plan,a_name,si,nt,w1r,str_rem,w2r,bought,n_rollouts)
            if z>best_z or (z==best_z and m>best_m):
                best_z,best_m=z,m;best_a=a_name
        if best_a is None:
            best_a='Stay'
        storm=np.random.random()<P_STORM
        apply_weather(ship,best_a,storm)
        log.append({'day':ship.day,'pos':ship.pos,'O':ship.O,'H':ship.H,
                    'F':ship.F,'M':ship.M,'Z':ship.Z,'action':best_a})
        # Update tracking variables
        if best_a=='Depart':
            si+=1
            if si<len(plan.segments):
                nt=plan.segments[si].target
                w1r=plan.segments[si].w1;str_rem=plan.segments[si].stop
                w2r=plan.segments[si].w2;bought=False
        elif best_a.startswith('Work') and w1r>0:w1r-=1
        elif best_a.startswith('Work') and w2r>0:w2r-=1
        elif best_a=='Stay' and str_rem>0:str_rem-=1
        elif best_a=='Supply':
            bought=True
            seg=plan.segments[si]
            bo,bh,bf=seg.buy_O,seg.buy_H,seg.buy_F
            cost=bo*2+bh+bf*2
            if ship.M>=cost:
                ship.O+=bo;ship.H+=bh;ship.F+=bf;ship.M-=cost
        if not ship.alive():
            print(f"Died at day {ship.day}")
            break
    success=(ship.pos==ALL_XY['E'])
    return success,ship.Z,ship.M,ship.day,log

def main():
    print("="*60)
    print("  Q3 Approach 2: MDP + Rollout Strategy")
    print("="*60)
    print("Using a base policy from a candidate macro plan,")
    print("and improving decisions via rollout simulation.")
    print(f"P(normal)={P_NORMAL}, P(storm)={P_STORM}")
    print()
    # Build a candidate plan (should be loaded from MILP output)
    pid=[1,6,3,4,7,5,2]
    w1=[4,5,3];b=[1,1,1];w2=[4,5,3]
    buy=np.array([[50,80,50],[30,40,30]])
    plan=build_plan_from_milp(pid,w1,b,w2,buy)
    print(f"Base plan route: {' -> '.join(plan.route)}")
    print("Running rollout with 20 rollouts per decision...")
    success,Z,M,days,log=run_rollout(plan,n_rollouts=20)
    print(f"\nRollout result: success={success}, Z={Z}, M={M}, days={days}")
    if log:
        print("\n--- Rollout decisions (first 10) ---")
        for e in log[:10]:
            print(f"  D{e['day']:2d}: {e['pos']} O={e['O']} H={e['H']} "
                  f"F={e['F']} M={e['M']} Z={e['Z']} | {e['action']}")

if __name__=='__main__':
    main()
