import os
os.chdir(r'C:\Users\ming\Desktop\数模备赛\task3_mdp')

code = '''import numpy as np
from compressed_dp import p, map_to_compressed

def get_adj(pos):
    x,y=pos; adj=[]
    for dx,dy in [(0,1),(0,-1),(1,0),(-1,0)]:
        nx,ny=x+dx,y+dy
        if 1<=nx<=30 and 1<=ny<=30: adj.append((nx,ny))
    return adj

def is_work(pos):
    for i,w in enumerate(p.W):
        if pos==w: return i
    return None

def is_supply(pos):
    for i,s in enumerate(p.S):
        if pos==s: return i
    return None

def consumption(action,weather):
    if weather=='normal': pref='normal'
    else: pref='storm'
    if action=='move': return tuple(getattr(p,f'{pref}_move'))
    elif action=='idle': return tuple(getattr(p,f'{pref}_idle'))
    elif action=='work': return tuple(getattr(p,f'{pref}_work'))
    return (0,0,0)

def eval_state(pos,O,H,F,M,consec,wp,tr,V):
    if tr<=0: return M if pos==p.E else -1e9
    v,_=map_to_compressed(pos,O,H,F,M,consec,wp,tr,V)
    return v if v is not None else -1e9

def optimal_buy(O,H,F,M,V,tr,pos):
    best_v=-1e9; best_b=(0,0,0); cap=p.capacity
    rem=cap-(O+H+F)
    buys=[(0,0,0)]
    if rem>0 and M>0:
        for frac in [0.3,0.6,1.0]:
            bo=int(frac*rem*0.35); bh=int(frac*rem*0.30); bf=int(frac*rem*0.35)
            bo=min(bo,rem); bh=min(bh,rem-bo); bf=min(bf,rem-bo-bh)
            cost=bo*p.price[0]+bh*p.price[1]+bf*p.price[2]
            if cost<=M: buys.append((bo,bh,bf))
        for rsrc,cost_pu in [(0,p.price[0]),(1,p.price[1]),(2,p.price[2])]:
            for frac in [0.5,1.0]:
                amt=int(frac*min(rem,M//cost_pu))
                if amt>0:
                    bb=[0,0,0]; bb[rsrc]=amt
                    if amt*cost_pu<=M: buys.append(tuple(bb))
    buys=list(set(buys))
    for bo,bh,bf in buys:
        if bo==0 and bh==0 and bf==0: continue
        cost=bo*p.price[0]+bh*p.price[1]+bf*p.price[2]
        if cost>M: continue
        v=eval_state(pos,O+bo,H+bh,F+bf,M-cost,0,0,tr,V)
        if v>best_v: best_v=v; best_b=(bo,bh,bf)
    return best_b,best_v

class MPCController:
    def __init__(self,V): self.V=V; self.ZS=100000
    def decide(self,state,weather,day):
        pos=state['position']; O,H,F=state['O'],state['H'],state['F']
        M,Z=state['M'],state['Z']; wc,wp=state['consecutive_work'],state['work_point']
        tr=p.T-day+1; cands=[]
        for nx,ny in get_adj(pos):
            cO,cH,cF=consumption('move',weather)
            nO,nH,nF=O-cO,H-cH,F-cF
            if nO<0 or nH<0 or nF<0: continue
            if nO+nH+nF>p.capacity: continue
            if (nx,ny)==p.E: value=Z*self.ZS+M
            else:
                nwp_val=is_work((nx,ny)); nwp=nwp_val+1 if nwp_val is not None else 0
                ev=0.0
                for nw,prob in [('normal',p.p_normal),('storm',p.p_storm)]:
                    v=eval_state((nx,ny),nO,nH,nF,M,0,nwp,tr-1,self.V)
                    if v>-1e8: ev+=prob*v
                if ev<=-1e8: continue
                value=Z*self.ZS+ev
            cands.append({'action':'move','target':(nx,ny),'value':value,'details':{'cons':(cO,cH,cF)}})
        cO,cH,cF=consumption('idle',weather)
        nO,nH,nF=O-cO,H-cH,F-cF
        if nO>=0 and nH>=0 and nF>=0 and nO+nH+nF<=p.capacity:
            ev=0.0
            for nw,prob in [('normal',p.p_normal),('storm',p.p_storm)]:
                v=eval_state(pos,nO,nH,nF,M,0,0,tr-1,self.V)
                if v>-1e8: ev+=prob*v
            if ev>-1e8:
                cands.append({'action':'idle','target':pos,'value':Z*self.ZS+ev,'details':{'cons':(cO,cH,cF)}})
        wp_val=is_work(pos)
        if wp_val is not None and wc<p.max_work[wp_val]:
            cO,cH,cF=consumption('work',weather)
            nO,nH,nF=O-cO,H-cH,F-cF
            if nO>=0 and nH>=0 and nF>=0 and nO+nH+nF<=p.capacity:
                nZ=Z+p.reward[wp_val]; nwc=wc+1
                ev=0.0
                for nw,prob in [('normal',p.p_normal),('storm',p.p_storm)]:
                    v=eval_state(pos,nO,nH,nF,M,nwc,wp_val+1,tr-1,self.V)
                    if v>-1e8: ev+=prob*v
                if ev>-1e8:
                    cands.append({'action':'work','target':pos,'value':nZ*self.ZS+ev,
                        'details':{'gain':p.reward[wp_val],'cons':(cO,cH,cF),'nwc':nwc}})
        sp_val=is_supply(pos)
        if sp_val is not None:
            buys,buy_v=optimal_buy(O,H,F,M,self.V,tr,pos)
            bo,bh,bf=buys
            if bo>0 or bh>0 or bf>0:
                cost=bo*p.price[0]+bh*p.price[1]+bf*p.price[2]
                nO,nH,nF=O+bo,H+bh,F+bf; nM=M-cost
                ev=0.0
                for nw,prob in [('normal',p.p_normal),('storm',p.p_storm)]:
                    v=eval_state(pos,nO,nH,nF,nM,0,0,tr-1,self.V)
                    if v>-1e8: ev+=prob*v
                if ev>-1e8:
                    value=Z*self.ZS+ev-cost*1e-6
                    cands.append({'action':'buy','target':pos,'value':value,'details':{'buy':buys,'cost':cost}})
        if not cands: return None,None
        best=max(cands,key=lambda c:c['value'])
        return best['action'],best

def run_simulation(weather_seq,V,verbose=True):
    ctrl=MPCController(V)
    S={'position':p.B,'O':p.init_O,'H':p.init_H,'F':p.init_F,'M':p.init_M,'Z':p.init_Z,'consecutive_work':0,'work_point':0}
    recs=[]; route=[p.B]; arr_day=0
    for day in range(1,p.T+1):
        w=weather_seq[day-1]
        if S['position']==p.E and not arr_day: arr_day=day
        if S['position']==p.E and arr_day and day>arr_day: break
        act,ad=ctrl.decide(S,w,day)
        if act is None: return {'feasible':False,'failure_day':day,'records':recs}
        rec={'Day':day,'Weather':w,'StartX':S['position'][0],'StartY':S['position'][1],
            'Action':'','EndX':S['position'][0],'EndY':S['position'][1],
            'BuyO':0,'BuyH':0,'BuyF':0,'Gain':0,
            'O':S['O'],'H':S['H'],'F':S['F'],'M':S['M'],'Z':S['Z']}
        if act=='move':
            np2=ad['target']; cons=ad['details']['cons']
            rec['Action']=f'Move({np2[0]},{np2[1]})'; rec['EndX'],rec['EndY']=np2
            S['position']=np2; S['O']-=cons[0]; S['H']-=cons[1]; S['F']-=cons[2]
            S['consecutive_work']=0; S['work_point']=0
            wpn=is_work(np2)
            if wpn is not None: S['work_point']=wpn+1
            route.append(np2)
            if np2==p.E: arr_day=day
        elif act=='idle':
            cons=ad['details']['cons']; rec['Action']='Stay'
            S['O']-=cons[0]; S['H']-=cons[1]; S['F']-=cons[2]
            S['consecutive_work']=0; S['work_point']=0
        elif act=='work':
            cons=ad['details']['cons']; gain=ad['details']['gain']
            rec['Action']=f'Work(+{gain})'; rec['Gain']=gain
            S['O']-=cons[0]; S['H']-=cons[1]; S['F']-=cons[2]
            S['Z']+=gain; S['consecutive_work']=ad['details']['nwc']
        elif act=='buy':
            buy=ad['details']['buy']; cost=ad['details']['cost']
            rec['Action']=f'Buy({buy[0]},{buy[1]},{buy[2]})'
            rec['BuyO'],rec['BuyH'],rec['BuyF']=buy
            S['O']+=buy[0]; S['H']+=buy[1]; S['F']+=buy[2]; S['M']-=cost
        rec['O'],rec['H'],rec['F']=S['O'],S['H'],S['F']
        rec['M'],rec['Z']=S['M'],S['Z']
        recs.append(rec)
        if S['O']<0 or S['H']<0 or S['F']<0 or S['M']<0:
            return {'feasible':False,'failure_day':day,'records':recs}
        if S['O']+S['H']+S['F']>p.capacity:
            return {'feasible':False,'failure_day':day,'records':recs}
        if verbose and day%15==0:
            print(f'  Day{day}: pos={S["position"]}, Z={S["Z"]}, M={S["M"]}, O={S["O"]}')
    return {'feasible':True if arr_day>0 else False,'arrival_day':arr_day if arr_day>0 else p.T,
        'final_Z':S['Z'],'final_M':S['M'],'records':recs,'route':route}
'''

with open('scenario_mpc.py','w',encoding='utf-8') as f:
    f.write(code)
print('OK')
