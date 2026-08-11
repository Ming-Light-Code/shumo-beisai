"""
True Stochastic Scenario Tree MPC
At each day, build a 3-deep scenario tree with weather branching (normal/storm).
Evaluate all paths, pick action with max expected value.
"""
import numpy as np
from math import comb

class P:
    T=90;capacity=400
    B=(1,15);E=(30,15);S=[(12,16),(21,16)];W=[(6,21),(15,9),(24,24)]
    reward=[20,15,28];max_work=[4,5,3];price=[2,1,2]
    nm=[2,3,2];ni=[1,1,1];nw=[5,4,3]
    sm=[8,4,3];si=[3,3,2];sw=[8,6,6]
    pn=0.8;ps=0.2
    iO=100;iH=150;iF=100;iM=750;iZ=200

def md(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])

def is_wp(pos):
    for i,w in enumerate(P.W):
        if pos==w: return i
    return None

def is_sp(pos):
    for i,s in enumerate(P.S):
        if pos==s: return i
    return None

def neighbors(pos):
    x,y=pos;a=[]
    for dx,dy in [(0,1),(0,-1),(1,0),(-1,0)]:
        nx,ny=x+dx,y+dy
        if 1<=nx<=30 and 1<=ny<=30: a.append((nx,ny))
    return a

def cons(action,weather):
    if weather=='normal':
        if action=='m': return tuple(P.nm)
        if action=='i': return tuple(P.ni)
        if action=='w': return tuple(P.nw)
    else:
        if action=='m': return tuple(P.sm)
        if action=='i': return tuple(P.si)
        if action=='w': return tuple(P.sw)
    return (0,0,0)

ZS=100000

def leaf_value(pos,O,H,F,M,Z,wc,wp,tr):
    """Heuristic value at a leaf node (after tree depth)."""
    if pos==P.E: return Z*ZS+M
    if tr<=0 or O<0 or H<0 or F<0 or M<0: return -1e9
    if O+H+F>P.capacity: return -1e9
    
    dE=md(pos,P.E)
    expO=dE*(P.pn*P.nm[0]+P.ps*P.sm[0])
    expH=dE*(P.pn*P.nm[1]+P.ps*P.sm[1])
    expF=dE*(P.pn*P.nm[2]+P.ps*P.sm[2])
    
    best=Z*ZS+M if (O>=expO and H>=expH and F>=expF and dE<=tr) else -1e9
    
    # Check work point bonuses (position-dependent)
    for wi in range(3):
        dw=md(pos,P.W[wi]);dWE=md(P.W[wi],P.E)
        td=dw+P.max_work[wi]+dWE
        if td>tr: continue
        eOw=dw*(P.pn*P.nm[0]+P.ps*P.sm[0])+P.max_work[wi]*(P.pn*P.nw[0]+P.ps*P.sw[0])+dWE*(P.pn*P.nm[0]+P.ps*P.sm[0])
        eHw=dw*(P.pn*P.nm[1]+P.ps*P.sm[1])+P.max_work[wi]*(P.pn*P.nw[1]+P.ps*P.sw[1])+dWE*(P.pn*P.nm[1]+P.ps*P.sm[1])
        eFw=dw*(P.pn*P.nm[2]+P.ps*P.sm[2])+P.max_work[wi]*(P.pn*P.nw[2]+P.ps*P.sw[2])+dWE*(P.pn*P.nm[2]+P.ps*P.sm[2])
        if O>=eOw and H>=eHw and F>=eFw:
            bonus=P.max_work[wi]*P.reward[wi]
            df=1.0/(1.0+dw/5.0)
            v=(Z+bonus*df)*ZS+M
            if v>best: best=v
    
    # Supply-then-work bonus
    for si in range(2):
        dS=md(pos,P.S[si])
        if dS>=tr: continue
        eOs=dS*(P.pn*P.nm[0]+P.ps*P.sm[0])
        eHs=dS*(P.pn*P.nm[1]+P.ps*P.sm[1])
        eFs=dS*(P.pn*P.nm[2]+P.ps*P.sm[2])
        if O<eOs or H<eHs or F<eFs: continue
        oa=O-eOs;ha=H-eHs;fa=F-eFs
        rem=P.capacity-(oa+ha+fa)
        if rem>0 and M>0:
            mb=min(rem*0.35,150)
            oa+=mb*0.35;ha+=mb*0.30;fa+=mb*0.35
        for wi in range(3):
            dSW=md(P.S[si],P.W[wi]);dWE2=md(P.W[wi],P.E)
            td2=dS+1+dSW+P.max_work[wi]+dWE2
            if td2>tr: continue
            eOw2=dSW*(P.pn*P.nm[0]+P.ps*P.sm[0])+P.max_work[wi]*(P.pn*P.nw[0]+P.ps*P.sw[0])+dWE2*(P.pn*P.nm[0]+P.ps*P.sm[0])
            eHw2=dSW*(P.pn*P.nm[1]+P.ps*P.sm[1])+P.max_work[wi]*(P.pn*P.nw[1]+P.ps*P.sw[1])+dWE2*(P.pn*P.nm[1]+P.ps*P.sm[1])
            eFw2=dSW*(P.pn*P.nm[2]+P.ps*P.sm[2])+P.max_work[wi]*(P.pn*P.nw[2]+P.ps*P.sw[2])+dWE2*(P.pn*P.nm[2]+P.ps*P.sm[2])
            if oa>=eOw2 and ha>=eHw2 and fa>=eFw2:
                bonus=P.max_work[wi]*P.reward[wi]
                df2=1.0/(1.0+dS/5.0+dSW/10.0)
                v=(Z+bonus*df2)*ZS+max(0,M-50)
                if v>best: best=v
    return best


def tree_eval(pos,O,H,F,M,Z,wc,wp,tr,depth):
    """Recursive scenario tree: branch on weather, select best action."""
    if pos==P.E: return Z*ZS+M
    if tr<=0 or O<0 or H<0 or F<0 or M<0: return -1e9
    if O+H+F>P.capacity: return -1e9
    if depth<=0: return leaf_value(pos,O,H,F,M,Z,wc,wp,tr)
    
    best_val=-1e9
    for nx,ny in neighbors(pos):
        # Two weather branches
        ev=0.0
        for w,pr in [('normal',P.pn),('storm',P.ps)]:
            cO,cH,cF=cons('m',w)
            nO,nH,nF=O-cO,H-cH,F-cF
            if nO<0 or nH<0 or nF<0: continue
            if nO+nH+nF>P.capacity: continue
            if (nx,ny)==P.E:
                ev+=pr*(Z*ZS+M)
            else:
                nwp=is_wp((nx,ny));nwpid=nwp+1 if nwp is not None else 0
                v=tree_eval((nx,ny),nO,nH,nF,M,Z,0,nwpid,tr-1,depth-1)
                if v>-1e8: ev+=pr*v
        if ev>best_val: best_val=ev
    
    # Idle
    ev=0.0
    for w,pr in [('normal',P.pn),('storm',P.ps)]:
        cO,cH,cF=cons('i',w)
        nO,nH,nF=O-cO,H-cH,F-cF
        if nO>=0 and nH>=0 and nF>=0 and nO+nH+nF<=P.capacity:
            v=tree_eval(pos,nO,nH,nF,M,Z,0,0,tr-1,depth-1)
            if v>-1e8: ev+=pr*v
    best_val=max(best_val,ev)
    
    # Work
    wpi=is_wp(pos)
    if wpi is not None and wc<P.max_work[wpi]:
        ev=0.0
        for w,pr in [('normal',P.pn),('storm',P.ps)]:
            cO,cH,cF=cons('w',w)
            nO,nH,nF=O-cO,H-cH,F-cF
            if nO>=0 and nH>=0 and nF>=0 and nO+nH+nF<=P.capacity:
                nZ=Z+P.reward[wpi];nwc=wc+1
                v=tree_eval(pos,nO,nH,nF,M,nZ,nwc,wpi+1,tr-1,depth-1)
                if v>-1e8: ev+=pr*v
        best_val=max(best_val,ev)
    
    return best_val


class TreeMPC2:
    def decide(self,state,weather,day):
        pos=state['position'];O,H,F=state['O'],state['H'],state['F']
        M,Z=state['M'],state['Z'];wc,wp=state['consecutive_work'],state['work_point']
        tr=P.T-day+1;cands=[]
        
        for nx,ny in neighbors(pos):
            cO,cH,cF=cons('m',weather)
            nO,nH,nF=O-cO,H-cH,F-cF
            if nO<0 or nH<0 or nF<0: continue
            if nO+nH+nF>P.capacity: continue
            if (nx,ny)==P.E:
                val=Z*ZS+M
            else:
                nwp=is_wp((nx,ny));nwpid=nwp+1 if nwp is not None else 0
                val=tree_eval((nx,ny),nO,nH,nF,M,Z,0,nwpid,tr-1,2)
                val+=1.0/(1+md((nx,ny),P.E))*0.1
            cands.append({'action':'move','target':(nx,ny),'value':val,'c':(cO,cH,cF)})
        
        cO,cH,cF=cons('i',weather)
        nO,nH,nF=O-cO,H-cH,F-cF
        if nO>=0 and nH>=0 and nF>=0 and nO+nH+nF<=P.capacity:
            val=tree_eval(pos,nO,nH,nF,M,Z,0,0,tr-1,2)
            cands.append({'action':'idle','target':pos,'value':val,'c':(cO,cH,cF)})
        
        wpi=is_wp(pos)
        if wpi is not None and wc<P.max_work[wpi]:
            cO,cH,cF=cons('w',weather)
            nO,nH,nF=O-cO,H-cH,F-cF
            if nO>=0 and nH>=0 and nF>=0 and nO+nH+nF<=P.capacity:
                nZ=Z+P.reward[wpi];nwc=wc+1
                val=tree_eval(pos,nO,nH,nF,M,nZ,nwc,wpi+1,tr-1,2)
                cands.append({'action':'work','target':pos,'value':val,'c':(cO,cH,cF),'g':P.reward[wpi],'nwc':nwc})
        
        spi=is_sp(pos)
        if spi is not None:
            rem=P.capacity-(O+H+F);buys=[(0,0,0)]
            if rem>0 and M>0:
                for frac in [0.4,0.7,1.0]:
                    bo=int(frac*rem*0.35);bh=int(frac*rem*0.3);bf=int(frac*rem*0.35)
                    bo=min(bo,rem);bh=min(bh,rem-bo);bf=min(bf,rem-bo-bh)
                    cost=bo*P.price[0]+bh*P.price[1]+bf*P.price[2]
                    if cost<=M: buys.append((bo,bh,bf))
            buys=list(set(buys))
            for bo,bh,bf in buys:
                if bo==0 and bh==0 and bf==0: continue
                cost=bo*P.price[0]+bh*P.price[1]+bf*P.price[2]
                if cost>M: continue
                nO,nH,nF=O+bo,H+bh,F+bf;nM=M-cost
                val=tree_eval(pos,nO,nH,nF,nM,Z,0,0,tr,2)-cost*1e-6
                cands.append({'action':'buy','target':pos,'value':val,'b':(bo,bh,bf),'cost':cost})
        
        if not cands: return None,None
        best=max(cands,key=lambda c:c['value'])
        return best['action'],best


def run_sim(weather_seq,verbose=True):
    ctrl=TreeMPC2()
    S={'position':P.B,'O':P.iO,'H':P.iH,'F':P.iF,'M':P.iM,'Z':P.iZ,'consecutive_work':0,'work_point':0}
    recs=[];route=[P.B];arr=0
    for day in range(1,P.T+1):
        w=weather_seq[day-1]
        if S['position']==P.E and not arr: arr=day
        if S['position']==P.E and arr and day>arr: break
        act,ad=ctrl.decide(S,w,day)
        if act is None: return {'feasible':False,'day':day,'recs':recs}
        rec={'Day':day,'Weather':w,'SX':S['position'][0],'SY':S['position'][1],
            'A':'','EX':S['position'][0],'EY':S['position'][1],
            'BO':0,'BH':0,'BF':0,'G':0,'O':S['O'],'H':S['H'],'F':S['F'],'M':S['M'],'Z':S['Z']}
        if act=='move':
            np2=ad['target'];cO,cH,cF=ad['c']
            rec['A']='M(%d,%d)'%np2;rec['EX'],rec['EY']=np2
            S['position']=np2;S['O']-=cO;S['H']-=cH;S['F']-=cF
            S['consecutive_work']=0;S['work_point']=0
            wpn=is_wp(np2)
            if wpn is not None: S['work_point']=wpn+1
            route.append(np2)
            if np2==P.E: arr=day
        elif act=='idle':
            cO,cH,cF=ad['c'];rec['A']='Stay'
            S['O']-=cO;S['H']-=cH;S['F']-=cF
            S['consecutive_work']=0;S['work_point']=0
        elif act=='work':
            cO,cH,cF=ad['c'];rec['A']='W(+%d)'%ad['g'];rec['G']=ad['g']
            S['O']-=cO;S['H']-=cH;S['F']-=cF
            S['Z']+=ad['g'];S['consecutive_work']=ad['nwc']
        elif act=='buy':
            bo,bh,bf=ad['b'];cost=ad['cost']
            rec['A']='B(%d,%d,%d)'%(bo,bh,bf);rec['BO'],rec['BH'],rec['BF']=bo,bh,bf
            S['O']+=bo;S['H']+=bh;S['F']+=bf;S['M']-=cost
        rec['O'],rec['H'],rec['F']=S['O'],S['H'],S['F']
        rec['M'],rec['Z']=S['M'],S['Z'];recs.append(rec)
        if S['O']<0 or S['H']<0 or S['F']<0 or S['M']<0:
            return {'feasible':False,'day':day,'recs':recs}
        if S['O']+S['H']+S['F']>P.capacity:
            return {'feasible':False,'day':day,'recs':recs}
        if verbose and day%15==0:
            print('  D%d: %s Z=%d M=%d O=%d'%(day,str(S['position']),S['Z'],S['M'],S['O']))
    return {'feasible':True,'arr':arr if arr else P.T,'Z':S['Z'],'M':S['M'],'recs':recs,'route':route}


if __name__=='__main__':
    import time
    print('='*60)
    print('Stochastic Tree MPC')
    print('='*60)
    np.random.seed(2026)
    w=['storm' if np.random.random()<P.ps else 'normal' for _ in range(P.T)]
    t0=time.time()
    r=run_sim(w,verbose=True)
    print('Sim in %.1fs'%(time.time()-t0))
    if r['feasible']:
        print('\nArrival day %d, Z=%d, M=%d'%(r['arr'],r['Z'],r['M']))
        # MC
        print('MC (50 runs)...')
        succ=0;zs=[];ms=[]
        for i in range(50):
            w2=['storm' if np.random.random()<P.ps else 'normal' for _ in range(P.T)]
            r2=run_sim(w2,verbose=False)
            if r2['feasible']:
                succ+=1;zs.append(r2['Z']);ms.append(r2['M'])
        rate=succ/50
        print('Rate: %.3f (%d/50)'%(rate,succ))
        if zs: print('Z: %.0f+-%.0f, M: %.0f+-%.0f'%(np.mean(zs),np.std(zs),np.mean(ms),np.std(ms)))
    else:
        print('Failed day %d'%r.get('day','?'))
    print('Done!')
