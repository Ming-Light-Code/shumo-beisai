#!/usr/bin/env python3
"""
Q3 Monte Carlo Simulation with Weather-Adaptive Strategy.

Given a macroscopic plan from the expected-consumption MILP,
this simulator samples random weather sequences and executes
an online adaptive policy.
"""
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

ALL_XY = {'B':(1,15),'E':(30,15),'W1':(6,21),'W2':(15,9),'W3':(24,24),'S1':(12,16),'S2':(21,16)}
NAME_IDX = {'B':1,'E':2,'W1':3,'W2':4,'W3':5,'S1':6,'S2':7}
IDX_NAME = {v:k for k,v in NAME_IDX.items()}
INIT_O,INIT_H,INIT_F=100,150,100
INIT_M,INIT_Z=750,200
LOAD_LIMIT,MAX_DAYS=400,90
P_NORMAL,P_STORM=0.8,0.2
CM_N=np.array([2,3,2]);CS_N=np.array([1,1,1]);CW_N=np.array([5,4,3])
CM_T=np.array([8,4,3]);CS_T=np.array([3,3,2]);CW_T=np.array([8,6,6])
PRICE=np.array([2,1,2])
WY=[20,15,28];WM=[4,5,3]

def manhattan(p1,p2): return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

@dataclass
class PlanSegment:
    target:str;travel_dist:int=0;w1:int=0;stop:int=0;w2:int=0
    buy_O:int=0;buy_H:int=0;buy_F:int=0

@dataclass
class MacroPlan:
    route:List[str];segments:List[PlanSegment]

def build_plan_from_milp(pid,w1,b,w2,buy):
    xys=[(1,15),(30,15),(6,21),(15,9),(24,24),(12,16),(21,16)]
    route=[IDX_NAME[p] for p in pid];m=len(pid)-2
    segs=[];sidx=0;widx=0
    for k in range(m+1):
        s=PlanSegment(target=IDX_NAME[pid[k+1]])
        s.travel_dist=manhattan(xys[pid[k]-1],xys[pid[k+1]-1])
        pt=pid[k+1]
        if pt in [6,7]:
            if sidx<buy.shape[0]:
                s.buy_O,s.buy_H,s.buy_F=int(buy[sidx,0]),int(buy[sidx,1]),int(buy[sidx,2])
            sidx+=1
        elif pt in [3,4,5]:
            if widx<len(w1):
                s.w1=int(w1[widx]);s.stop=int(b[widx]) if widx<len(b) else 0
                s.w2=int(w2[widx]) if widx<len(w2) else 0
            widx+=1
        segs.append(s)
    return MacroPlan(route=route,segments=segs)

class Ship:
    def __init__(self):
        self.x,self.y=ALL_XY['B']
        self.O,self.H,self.F=INIT_O,INIT_H,INIT_F
        self.M,self.Z=INIT_M,INIT_Z
        self.day=0;self.consec_work=0;self.cur_work=None;self.at_E=False
    @property
    def load(self): return self.O+self.H+self.F
    @property
    def pos(self): return (self.x,self.y)
    def consume(self,c):
        self.O-=c[0];self.H-=c[1];self.F-=c[2]
    def alive(self):
        return self.O>=0 and self.H>=0 and self.F>=0 and self.M>=0

def shortest_path(start,end):
    path=[];cx,cy=start;tx,ty=end
    while cx!=tx or cy!=ty:
        if cx<tx:cx+=1
        elif cx>tx:cx-=1
        elif cy<ty:cy+=1
        else:cy-=1
        path.append((cx,cy))
    return path

def simulate_one(plan,weather_seq,safety=0.3):
    s=Ship();si=0;nt=plan.segments[si].target
    w1r=plan.segments[si].w1;w2r=plan.segments[si].w2
    str=plan.segments[si].stop;bought=False
    log=[];safe_load=safety*LOAD_LIMIT
    while s.day<MAX_DAYS and si<len(plan.segments):
        s.day+=1;storm=(weather_seq[s.day-1]==1)
        cm,cs,cw=(CM_T,CS_T,CW_T) if storm else (CM_N,CS_N,CW_N)
        action=""
        at=(s.pos==ALL_XY[nt])
        if at:
            seg=plan.segments[si]
            if nt in ['S1','S2']:
                if not bought:
                    bo,bh,bf=seg.buy_O,seg.buy_H,seg.buy_F
                    cost=bo*2+bh*1+bf*2
                    if s.M>=cost and s.O+bo+s.H+bh+s.F+bf<=LOAD_LIMIT:
                        s.O+=bo;s.H+=bh;s.F+=bf;s.M-=cost
                        action=f"Supply({bo},{bh},{bf})"
                    bought=True
                else:
                    si+=1;bought=False
                    if si<len(plan.segments):
                        nt=plan.segments[si].target
                        w1r=plan.segments[si].w1;str=plan.segments[si].stop
                        w2r=plan.segments[si].w2
                    action="Depart";continue
            elif nt in ['W1','W2','W3']:
                wi=['W1','W2','W3'].index(nt);maxc=WM[wi]
                sw=False
                if w1r>0:sw=True
                elif str>0:sw=False
                elif w2r>0:sw=True
                else:
                    si+=1
                    if si<len(plan.segments):
                        nt=plan.segments[si].target
                        w1r=plan.segments[si].w1;str=plan.segments[si].stop
                        w2r=plan.segments[si].w2
                    s.consec_work=0;s.cur_work=None
                    action="Depart";continue
                if storm and s.load<safe_load and sw:
                    s.consume(cs);action="StormStop"
                elif sw and s.consec_work<maxc:
                    s.consume(cw);s.Z+=WY[wi];s.consec_work+=1;s.cur_work=wi
                    action=f"Work@{nt}"
                    if w1r>0:w1r-=1
                    else:w2r-=1
                elif not sw:
                    s.consume(cs);s.consec_work=0;s.cur_work=None
                    if str>0:str-=1
                    action=f"Stop@{nt}"
                else:
                    s.consume(cs);s.consec_work=0;s.cur_work=None
                    action="ForceStop"
            elif nt=='E':
                s.at_E=True;action="Arrive@E";break
        else:
            if storm and s.load<safe_load:
                s.consume(cs);action="StormStop"
            else:
                tp=ALL_XY[nt];dx=np.sign(tp[0]-s.x);dy=np.sign(tp[1]-s.y)
                if abs(tp[0]-s.x)>=abs(tp[1]-s.y) and dx!=0:s.x+=dx
                elif dy!=0:s.y+=dy
                else:s.x+=dx
                s.consume(cm);s.consec_work=0;s.cur_work=None
                action=f"Move({s.x},{s.y})"
        if not s.alive():
            log.append({'day':s.day,'x':s.x,'y':s.y,'O':s.O,'H':s.H,'F':s.F,
                'M':s.M,'Z':s.Z,'action':action,'storm':storm})
            return False,s.Z,s.M,s.day,log
        log.append({'day':s.day,'x':s.x,'y':s.y,'O':s.O,'H':s.H,'F':s.F,
            'M':s.M,'Z':s.Z,'action':action,'storm':storm})
    if not s.at_E:
        d2e=manhattan(s.pos,ALL_XY['E']);rem=MAX_DAYS-s.day
        if d2e<=rem:
            path=shortest_path(s.pos,ALL_XY['E'])
            for sp in path:
                s.day+=1
                if s.day>MAX_DAYS:break
                storm=(weather_seq[s.day-1]==1) if s.day<=len(weather_seq) else False
                s.consume(CM_T if storm else CM_N)
                s.x,s.y=sp;s.consec_work=0
                if not s.alive():break
            if s.pos==ALL_XY['E']:s.at_E=True
    return s.at_E,s.Z,s.M,s.day,log

def run_monte_carlo(plan,n_sims=1000,safety=0.3):
    np.random.seed(42)
    Zv,Mv,Dv=[],[],[]
    succ=0;all_logs=[]
    for i in range(n_sims):
        ws=(np.random.random(MAX_DAYS)>P_NORMAL).astype(int)
        ok,z,m,d,log=simulate_one(plan,ws,safety)
        if ok:
            succ+=1;Zv.append(z);Mv.append(m);Dv.append(d)
            if i<5:all_logs.append(log)
    return {
        'n_sims':n_sims,'success_rate':succ/n_sims,'successes':succ,
        'Z_mean':np.mean(Zv) if Zv else 0,'Z_std':np.std(Zv) if Zv else 0,
        'Z_min':np.min(Zv) if Zv else 0,'Z_max':np.max(Zv) if Zv else 0,
        'M_mean':np.mean(Mv) if Mv else 0,'M_std':np.std(Mv) if Mv else 0,
        'days_mean':np.mean(Dv) if Dv else 0,'days_std':np.std(Dv) if Dv else 0,
    },all_logs

def main():
    print("="*60)
    print("  Q3 Monte Carlo Weather Simulation")
    print("="*60)
    print(f"P(normal)={P_NORMAL}, P(storm)={P_STORM}")
    print(f"Load={LOAD_LIMIT}, MaxDays={MAX_DAYS}")
    print(f"Init: O={INIT_O} H={INIT_H} F={INIT_F} M={INIT_M} Z={INIT_Z}")
    # Sample candidate plan: B->S1->W1->W2->S2->W3->E
    pid=[1,6,3,4,7,5,2]
    w1=[4,5,3];b=[1,1,1];w2=[4,5,3]
    buy=np.array([[50,80,50],[30,40,30]])
    plan=build_plan_from_milp(pid,w1,b,w2,buy)
    print(f"\nRoute: {' -> '.join(plan.route)}")
    for i,seg in enumerate(plan.segments):
        print(f"  Seg{i+1}: ->{seg.target} d={seg.travel_dist} "
              f"w1={seg.w1} stop={seg.stop} w2={seg.w2} "
              f"buy=({seg.buy_O},{seg.buy_H},{seg.buy_F})")
    print(f"\nRunning 1000 sims...")
    results,logs=run_monte_carlo(plan,1000)
    print(f"\n--- Monte Carlo Results ---")
    print(f"Success: {results['success_rate']:.1%} ({results['successes']}/{results['n_sims']})")
    print(f"Z: mean={results['Z_mean']:.0f} std={results['Z_std']:.0f} [{results['Z_min']},{results['Z_max']}]")
    print(f"M: mean={results['M_mean']:.0f} std={results['M_std']:.0f}")
    print(f"Days: mean={results['days_mean']:.1f} std={results['days_std']:.1f}")
    if logs:
        print(f"\n--- Sample log (run 1, first 15 days) ---")
        for e in logs[0][:15]:
            sm="[STORM]" if e['storm'] else ""
            print(f"  D{e['day']:2d}: ({e['x']:2d},{e['y']:2d}) "
                  f"O={e['O']:3d} H={e['H']:3d} F={e['F']:3d} "
                  f"M={e['M']:3d} Z={e['Z']:3d} | {e['action']} {sm}")

if __name__=='__main__':
    main()
