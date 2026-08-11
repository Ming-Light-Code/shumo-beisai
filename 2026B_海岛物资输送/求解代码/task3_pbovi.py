
"""
Problem 3: PBOVI (Point-Based Online Value Iteration) Solver
Reference: [1] "Dynamic Uncertain Environment Agent Sequential Decision Methods" by Wu Bo
Method: MDP (Sec 2.1) + Online Forward Tree (Sec 3.3) + API Value Function Approximation
"""
import numpy as np, random, copy, time, json
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict

# === Problem Parameters ===
GRID = 30; B = (1,15); E = (30,15); S1 = (12,16); S2 = (21,16)
W1 = (6,21); W2 = (15,9); W3 = (24,24)
WORK_INFO = {"W1":(W1,20,4),"W2":(W2,15,5),"W3":(W3,28,3)}
SUPPLY = {"S1":S1,"S2":S2}
ALL_NODES = {"B":B,"E":E,"S1":S1,"S2":S2,"W1":W1,"W2":W2,"W3":W3}
INIT_O,INIT_H,INIT_F=100,150,100; INIT_M,INIT_Z=750,200; MAX_LOAD=400; MAX_DAYS=90; PRICES=(2,1,2)
CMOVE=[(2,3,2),(8,4,3)]; CIDLE=[(1,1,1),(3,3,2)]; CWORK=[(5,4,3),(8,6,6)]
PN,PS=0.8,0.2
EMOVE=tuple(PN*CMOVE[0][i]+PS*CMOVE[1][i] for i in range(3))
EIDLE=tuple(PN*CIDLE[0][i]+PS*CIDLE[1][i] for i in range(3))
EWORK=tuple(PN*CWORK[0][i]+PS*CWORK[1][i] for i in range(3))

def md(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])
def cns(wi,act):
    if act=="move": return CMOVE[wi]
    if act=="idle": return CIDLE[wi]
    if act=="work": return CWORK[wi]
    return (0,0,0)

@dataclass
class State:
    x:int;y:int;O:int;H:int;F:int;M:int;Z:int;c:int=0;day:int=1
    def clone(self): return copy.deepcopy(self)
    def ok(self): return self.O>=0 and self.H>=0 and self.F>=0 and self.M>=0 and self.O+self.H+self.F<=MAX_LOAD
    @property
    def pos(self): return (self.x,self.y)
    @pos.setter
    def pos(self,v): self.x,self.y=v
    def load(self): return self.O+self.H+self.F

def at_work(s):
    for nm,(pos,gain,mx) in WORK_INFO.items():
        if s.pos==pos: return nm,gain,mx
    return None,0,0

def at_supply(s):
    for nm,pos in SUPPLY.items():
        if s.pos==pos: return nm
    return None

def apply_action(s,action,wi,work_gain=0,new_pos=None):
    ns=s.clone(); cr=cns(wi,action)
    ns.O-=cr[0];ns.H-=cr[1];ns.F-=cr[2]
    if action=="work": ns.Z+=work_gain;ns.c+=1
    elif action=="move": ns.pos=new_pos;ns.c=0
    elif action=="idle": ns.c=0
    ns.day+=1
    return ns if ns.ok() else None

def apply_buy(s,bO,bH,bF):
    ns=s.clone(); cost=bO*2+bH*1+bF*2
    if cost>ns.M: return None
    ns.O+=bO;ns.H+=bH;ns.F+=bF;ns.M-=cost
    return ns if ns.ok() else None

def sw(rng): return 0 if rng.random()<PN else 1

def enum_skels(mx_nodes=10,mx_cnt=10000):
    nodes=ALL_NODES; im=["W1","W2","W3","S1","S2"]
    dist={}
    for n1 in nodes:
        for n2 in nodes: dist[(n1,n2)]=md(nodes[n1],nodes[n2])
    skels=[]
    def dfs(cur,path,tr):
        if len(skels)>=mx_cnt: return
        dE=dist[(cur,"E")]
        if tr+dE<=MAX_DAYS: skels.append(tuple(path+["E"]))
        for nxt in im:
            if nxt==cur: continue
            if cur in ("S1","S2") and nxt in ("S1","S2"): continue
            if len(path)>mx_nodes: continue
            d=dist[(cur,nxt)]
            if d==0 or tr+d>MAX_DAYS: continue
            dfs(nxt,path+[nxt],tr+d)
    dfs("B",["B"],0)
    return skels,nodes

def skel_dist(sk,np_):
    t=0
    for i in range(len(sk)-1): t+=md(np_[sk[i]],np_[sk[i+1]])
    return t

def comp_buy(state,sk,seg,np_):
    no,nh,nf=0.,0.,0.; vo,vh,vf=0.,0.,0.
    ns=len(sk)
    for j in range(seg+1,len(sk)):
        if sk[j] in ("S1","S2"): ns=j;break
    for j in range(seg,min(ns,len(sk)-1)):
        n1=sk[j];n2=sk[min(j+1,len(sk)-1)]
        d=md(np_[n1],np_[n2])
        no+=EMOVE[0]*d;nh+=EMOVE[1]*d;nf+=EMOVE[2]*d
        e2o=0.8*4+0.2*64;vo+=(e2o-EMOVE[0]**2)*d
        e2h=0.8*9+0.2*16;vh+=(e2h-EMOVE[1]**2)*d
        e2f=0.8*4+0.2*9;vf+=(e2f-EMOVE[2]**2)*d
        if n2 in WORK_INFO:
            mx=WORK_INFO[n2][2];ew=PN*mx
            no+=EWORK[0]*ew;nh+=EWORK[1]*ew;nf+=EWORK[2]*ew
            wv=mx*PN*PS
            vo+=wv*(CWORK[0][0]-CWORK[1][0])**2
            vh+=wv*(CWORK[0][1]-CWORK[1][1])**2
            vf+=wv*(CWORK[0][2]-CWORK[1][2])**2
    ec=no*2+nh*1+nf*2;cv=state.O*2+state.H*1+state.F*2
    fn=max(0,ec-cv)
    sl=(state.M-fn)/max(1,fn) if fn>0 else 2.0
    z=0.5+1.5*min(1.,max(0,sl))
    no=int(np.ceil(no+z*np.sqrt(max(0,vo))))
    nh=int(np.ceil(nh+z*np.sqrt(max(0,vh))))
    nf=int(np.ceil(nf+z*np.sqrt(max(0,vf))))
    bo=max(0,no-state.O);bh=max(0,nh-state.H);bf=max(0,nf-state.F)
    sp=MAX_LOAD-state.load();tot=bo+bh+bf
    if tot>sp>0: sc=sp/tot;bo=int(bo*sc);bh=int(bh*sc);bf=int(bf*sc)
    cst=bo*2+bh*1+bf*2
    if cst>state.M>0: sc=state.M/cst;bo=int(bo*sc);bh=int(bh*sc);bf=int(bf*sc)
    return bo,bh,bf

def mv_toward(s,tgt):
    dx=np.sign(tgt[0]-s.x) or 0; dy=0
    if dx==0: dy=np.sign(tgt[1]-s.y) or 0
    if dx==0 and dy==0: return s.pos
    return (s.x+dx,s.y+dy)

NF=15
def state_feats(s):
    dE=md(s.pos,E);rem=(MAX_DAYS-s.day+1)/MAX_DAYS;tp_=dE/max(1,MAX_DAYS-s.day+1)
    wnm,wg,wm_cap=at_work(s)
    at_w=1.0 if wnm else 0.0;wp_rem=(wm_cap-s.c)/max(1,wm_cap) if wnm else 0.0
    at_s=1.0 if at_supply(s) else 0.0;gf=wg*(wm_cap-s.c)/max(1,wm_cap) if wnm else 0.0
    return np.array([1.0,dE/58.0,s.O/MAX_LOAD,s.H/MAX_LOAD,s.F/MAX_LOAD,s.M/750.0,
        rem,s.load()/MAX_LOAD,at_w,wp_rem,gf/30.0,s.c/5.0,at_s,tp_,1.0 if s.pos==E else 0.0],dtype=np.float64)

def sim_base(sk,np_,rng):
    s=State(x=B[0],y=B[1],O=INIT_O,H=INIT_H,F=INIT_F,M=INIT_M,Z=INIT_Z,c=0,day=1)
    log,svs,si=[],[],1
    while s.day<=MAX_DAYS:
        if s.pos==E:
            for sv in svs: sv["ret_Z"]=s.Z;sv["ret_M"]=s.M
            return s.Z,s.M,log,svs
        svs.append({"state":s.clone(),"ret_Z":0,"ret_M":0})
        w=sw(rng);sn=at_supply(s);wn,wg,wm=at_work(s)
        entry={"day":s.day,"pos":(s.x,s.y),"O":s.O,"H":s.H,"F":s.F,"M":s.M,"Z":s.Z,"c":s.c,"weather":"N" if w==0 else "S"}
        log.append(entry)
        if sn and si<len(sk)-1 and sk[si]==sn:
            bo,bh,bf=comp_buy(s,sk,si,np_)
            ns=apply_buy(s,bo,bh,bf)
            if ns is None: return None
            entry["action"]=f"buy({bo},{bh},{bf})";entry["buy"]=(bo,bh,bf)
            ns=apply_action(ns,"idle",w)
            if ns is None: return None
            s=ns;si+=1;continue
        if wn and sk[si]==wn:
            if w==0 and s.c<wm:
                ns=apply_action(s,"work",w,work_gain=wg);entry["action"]="work";entry["gain"]=wg
            elif s.c<wm and s.day<MAX_DAYS-30:
                ns=apply_action(s,"idle",w);entry["action"]="idle_stay";entry["gain"]=0
                if ns is not None: ns.c=s.c
            else:
                ns=apply_action(s,"idle",w);entry["action"]="idle_leave";entry["gain"]=0;si+=1
            entry["buy"]=None
            if ns is None: return None
            s=ns;continue
        if si>=len(sk): return None
        tgt=np_[sk[si]]
        if s.pos==tgt: si+=1;continue
        npos=mv_toward(s,tgt);ns=apply_action(s,"move",w,new_pos=npos)
        if ns is None: return None
        entry["action"]="move";entry["new_pos"]=npos;entry["gain"]=0;entry["buy"]=None
        s=ns
    return None

def fit_theta(samples):
    X=np.array([state_feats(s) for s,_ in samples])
    y=np.array([r for _,r in samples],dtype=np.float64)
    I=np.eye(NF); return np.linalg.solve(X.T@X+1.0*I,X.T@y)

def sim_adp(sk,np_,thZ,thM,rng,ws=None):
    s=State(x=B[0],y=B[1],O=INIT_O,H=INIT_H,F=INIT_F,M=INIT_M,Z=INIT_Z,c=0,day=1)
    log,svs,si=[],[],1
    def pZ(st): return 0.0 if thZ is None else thZ@state_feats(st)
    def pM(st): return 0.0 if thM is None else thM@state_feats(st)
    while s.day<=MAX_DAYS:
        if s.pos==E:
            for sv in svs: sv["ret_Z"]=s.Z;sv["ret_M"]=s.M
            return s.Z,s.M,log,svs
        svs.append({"state":s.clone(),"ret_Z":0,"ret_M":0})
        w=ws[s.day-1] if ws is not None else sw(rng)
        sn=at_supply(s);wn,wg,wm=at_work(s)
        entry={"day":s.day,"pos":(s.x,s.y),"O":s.O,"H":s.H,"F":s.F,"M":s.M,"Z":s.Z,"c":s.c,"weather":"N" if w==0 else "S"}
        log.append(entry)
        if sn and si<len(sk)-1 and sk[si]==sn:
            bo,bh,bf=comp_buy(s,sk,si,np_)
            ns=apply_buy(s,bo,bh,bf)
            if ns is None: return None
            entry["action"]=f"buy({bo},{bh},{bf})";entry["buy"]=(bo,bh,bf)
            ns=apply_action(ns,"idle",w)
            if ns is None: return None
            s=ns;si+=1;continue
        if wn and sk[si]==wn:
            ns_w=apply_action(s,"work",w,work_gain=wg) if w==0 and s.c<wm else None
            ns_i=apply_action(s,"idle",w)
            if ns_i is not None and s.c<wm and s.day<MAX_DAYS-30 and not (w==0 and s.c<wm): ns_i.c=s.c
            if ns_w and ns_i:
                pZw,pMw=pZ(ns_w),pM(ns_w); pZi,pMi=pZ(ns_i),pM(ns_i)
                if pZw>pZi or (pZw==pZi and pMw>pMi): ns=ns_w;entry["action"]="work";entry["gain"]=wg
                else: ns=ns_i;entry["action"]="idle_leave";entry["gain"]=0;si+=1
            elif ns_i: ns=ns_i;entry["action"]="idle_leave";entry["gain"]=0;si+=1
            else: return None
            entry["buy"]=None;s=ns;continue
        if si>=len(sk): return None
        tgt=np_[sk[si]]
        if s.pos==tgt: si+=1;continue
        npos=mv_toward(s,tgt);ns=apply_action(s,"move",w,new_pos=npos)
        if ns is None: return None
        entry["action"]="move";entry["new_pos"]=npos;entry["gain"]=0;entry["buy"]=None
        s=ns
    return None

class OnlineTree:
    """Point-based online forward tree (Ref [1] Sec 3.3 PBOVI).
    AND-OR tree: AND=action choice, OR=weather outcome.
    Branch-and-bound with value function leaf evaluation."""
    def __init__(self,thZ,thM,sk,np_,rng,D=3):
        self.thZ=thZ;self.thM=thM;self.sk=sk;self.np_=np_;self.rng=rng;self.D=D
    def pV(self,s):
        if self.thZ is None or self.thM is None: return s.Z*100000.+s.M
        return (self.thZ@state_feats(s))*100000.+(self.thM@state_feats(s))
    def decide(self,s,wi,wn,sk,si):
        wg,wm=WORK_INFO[wn][1],WORK_INFO[wn][2]
        acts=[]
        if wi==0 and s.c<wm: acts.append("work")
        acts.append("idle")
        if len(acts)==1: return acts[0]
        ba,bv=None,-1e30
        for a in acts:
            v=self._qe(s,a,wi,wn,sk,si,0)
            if v>bv: bv=v;ba=a
        return ba
    def _qe(self,s,a,wi,wn,sk,si,d):
        wg=WORK_INFO[wn][1]
        if a=="work": ns=apply_action(s,"work",wi,work_gain=wg);nsi=si
        else: ns=apply_action(s,"idle",wi);nsi=si+1
        if ns is None: return -1e30
        if ns.pos==E: return ns.Z*100000.+ns.M
        if d>=self.D: return self.pV(ns)
        rn=self._ro(ns,sk,nsi,0);vn=rn if rn is not None else self.pV(ns)
        rs=self._ro(ns,sk,nsi,1);vs_=rs if rs is not None else self.pV(ns)
        return PN*vn+PS*vs_
    def _ro(self,s,sk,si,fw):
        if s.day>=MAX_DAYS or si>=len(sk): return None
        s=s.clone();w=fw
        if s.pos==E: return s.Z*100000.+s.M
        sn=at_supply(s);wn,wg,wm=at_work(s)
        if sn:
            bo,bh,bf=comp_buy(s,sk,si,self.np_)
            s=apply_buy(s,bo,bh,bf)
            if s is None: return None
            s=apply_action(s,"idle",w)
            if s is None: return None
            return self.pV(s)
        tn = sk[si] if si < len(sk) else None
        if wn and tn==wn:
            if w==0 and s.c<wm: s=apply_action(s,"work",w,work_gain=wg)
            elif s.c<wm:
                s2=apply_action(s,"idle",w)
                if s2 is not None: s=s2;s.c=s.c
                else: return None
            else: s=apply_action(s,"idle",w)
            if s is None: return None
            return self.pV(s)
        tgt=self.np_[sk[si]]
        if s.pos==tgt: return self.pV(s)
        npos=mv_toward(s,tgt);s=apply_action(s,"move",w,new_pos=npos)
        if s is None: return None
        return self.pV(s)

class Refiner:
    """Wrap OnlineTree into full-episode simulator with online decisions at work points."""
    def __init__(self,thZ,thM,sk,np_,rng,D=4):
        self.tree=OnlineTree(thZ,thM,sk,np_,rng,D);self.np_=np_;self.sk=sk;self.rng=rng
    def sim(self):
        sk=self.sk;np_=self.np_;rng=self.rng
        s=State(x=B[0],y=B[1],O=INIT_O,H=INIT_H,F=INIT_F,M=INIT_M,Z=INIT_Z,c=0,day=1)
        log=[];si=1
        while s.day<=MAX_DAYS:
            if s.pos==E: return s.Z,s.M,log
            w=sw(rng);sn=at_supply(s);wn,wg,wm=at_work(s)
            entry={"day":s.day,"pos":(s.x,s.y),"O":s.O,"H":s.H,"F":s.F,"M":s.M,"Z":s.Z,"c":s.c,"weather":"N" if w==0 else "S","action":"","gain":0,"buy":None,"new_pos":None}
            log.append(entry)
            if sn and si<len(sk)-1 and sk[si]==sn:
                bo,bh,bf=comp_buy(s,sk,si,np_)
                ns=apply_buy(s,bo,bh,bf)
                if ns is None: return None
                entry["action"]=f"buy({bo},{bh},{bf})";entry["buy"]=(bo,bh,bf)
                ns=apply_action(ns,"idle",w)
                if ns is None: return None
                s=ns;si+=1;continue
            if wn and sk[si]==wn:
                a=self.tree.decide(s,w,wn,sk,si)
                if a=="work": ns=apply_action(s,"work",w,work_gain=wg);entry["action"]="work";entry["gain"]=wg
                else: ns=apply_action(s,"idle",w);entry["action"]="idle_leave";si+=1
                if ns is None: return None
                s=ns;continue
            if si>=len(sk): return None
            tgt=np_[sk[si]]
            if s.pos==tgt: si+=1;continue
            npos=mv_toward(s,tgt);ns=apply_action(s,"move",w,new_pos=npos)
            if ns is None: return None
            entry["action"]="move";entry["new_pos"]=npos;s=ns
        return None

class Solver:
    """Two-stage PBOVI solver:
    Stage 1: Approximate Policy Iteration (train value functions via MC rollouts)
    Stage 2: Online tree refinement on best skeleton (Ref [1] Sec 3.3)"""
    def __init__(self,seed=42): self.seed=seed;self.rb=random.Random(seed)
    def solve(self,nsb=200,nsa=150,nsf=2000):
        t0=time.time()
        print("="*65)
        print(" Problem 3: PBOVI Solver")
        print(" Reference: Wu Bo PhD Thesis Ch2.1 MDP, Ch3.3 PBOVI")
        print("="*65)
        print("\n[Phase 1] Skeleton enumeration")
        skels,np_=enum_skels(10,10000)
        print(f"  Generated {len(skels)} candidates")
        uq=list(set(skels));uq.sort(key=lambda s:skel_dist(s,np_))
        hw=[s for s in uq if any(n in WORK_INFO for n in s[1:-1])]
        cand=hw+[("B","E")]
        print(f"  Dedup: {len(uq)}, with work: {len(hw)}, total: {len(cand)}")
        print(f"\n[Phase 2] Base policy MC ({nsb} runs each)")
        br=[]
        for i,sk in enumerate(cand):
            if skel_dist(sk,np_)>80: continue
            if i%100==0: print(f"    {i}/{len(cand)} ({time.time()-t0:.1f}s)")
            tZ,tM,sc=0,0,0
            for _ in range(nsb):
                rng=random.Random(self.rb.randint(0,2**31-1))
                r=sim_base(sk,np_,rng)
                if r: Zf,Mf,_,_=r;tZ+=Zf;tM+=Mf;sc+=1
            if sc>=nsb*0.1: br.append((sk,tZ/sc,tM/sc,sc))
        br.sort(key=lambda x:(x[1],x[2]),reverse=True)
        print(f"  Feasible: {len(br)}")
        for i,(sk,aZ,aM,sc) in enumerate(br[:5]):
            print(f"    #{i+1}: {' -> '.join(sk)}  Z={aZ:.1f} M={aM:.1f} ({sc}/{nsb})")
        print(f"\n[Phase 3] API Iter1: fit value functions")
        sZ,sM=[],[]
        for i,sk in enumerate(cand[:min(500,len(cand))]):
            if skel_dist(sk,np_)>80: continue
            for _ in range(nsb):
                rng=random.Random(self.rb.randint(0,2**31-1))
                r=sim_base(sk,np_,rng)
                if r:
                    Zf,Mf,_,svs=r
                    for sv in svs: sZ.append((sv["state"],sv["ret_Z"]));sM.append((sv["state"],sv["ret_M"]))
            if i%100==0: print(f"    {i} skeletons, {len(sZ)} samples")
        print(f"  Total: {len(sZ)} state-return pairs")
        if len(sZ)>50000:
            idx=np.random.choice(len(sZ),50000,replace=False)
            sZ=[sZ[i] for i in idx];sM=[sM[i] for i in idx]
        thZ=fit_theta(sZ);thM=fit_theta(sM)
        print(f"  theta_Z range: [{thZ.min():.1f}, {thZ.max():.1f}]")
        print(f"  theta_M range: [{thM.min():.1f}, {thM.max():.1f}]")
        print(f"\n[Phase 4] ADP eval (top 50, {nsa} MC)")
        tsk=[sk for sk,_,_,_ in br[:50]]+[("B","E")]
        bsk,bZ,bM,bL=None,-1,-1,None
        for sk in tsk:
            tZ,tM,sc=0,0,0;lZ,lM,lL=-1,-1,None
            for _ in range(nsa):
                rng=random.Random(self.rb.randint(0,2**31-1))
                r=sim_adp(sk,np_,thZ,thM,rng)
                if r:
                    Zf,Mf,lg,_=r;tZ+=Zf;tM+=Mf;sc+=1
                    if Zf>lZ or (Zf==lZ and Mf>lM): lZ,lM,lL=Zf,Mf,lg
            if sc>=nsa*0.15:
                aZ,aM=tZ/sc,tM/sc
                if aZ>bZ or (aZ==bZ and aM>bM): bZ,bM,bsk,bL=aZ,aM,sk,lL
        print(f"  Best skeleton: {' -> '.join(bsk) if bsk else 'None'}")
        print(f"  Expected Z={bZ:.1f}, M={bM:.1f}")
        print(f"\n[Phase 5] API Iter2: on-policy sampling & refit")
        sZ2,sM2=[],[]
        for i,sk in enumerate(cand[:min(300,len(cand))]):
            if skel_dist(sk,np_)>80: continue
            for _ in range(30):
                rng=random.Random(self.rb.randint(0,2**31-1))
                r=sim_adp(sk,np_,thZ,thM,rng)
                if r:
                    Zf,Mf,lg,svs=r
                    for sv in svs: sZ2.append((sv["state"],sv["ret_Z"]));sM2.append((sv["state"],sv["ret_M"]))
        print(f"  Added {len(sZ2)} on-policy samples")
        allZ=sZ+sZ2;allM=sM+sM2
        if len(allZ)>80000:
            idx=np.random.choice(len(allZ),80000,replace=False)
            allZ=[allZ[i] for i in idx];allM=[allM[i] for i in idx]
        thZ2=fit_theta(allZ);thM2=fit_theta(allM)
        print(f"  Refitted theta_Z2: [{thZ2.min():.1f}, {thZ2.max():.1f}]")
        print(f"\n[Phase 6] Online tree refinement (Paper Sec 3.3)")
        rz,rm,sc=0,0,0
        for _ in range(nsa):
            rng=random.Random(self.rb.randint(0,2**31-1))
            rf=Refiner(thZ2,thM2,bsk,np_,rng,4)
            r=rf.sim()
            if r:
                Zf,Mf,lg=r;rz+=Zf;rm+=Mf;sc+=1
                if Zf>bZ or (Zf==bZ and Mf>bM): bZ,bM,bL=Zf,Mf,lg
        if sc>0: print(f"  Refined E[Z]={rz/sc:.1f}, E[M]={rm/sc:.1f} ({sc}/{nsa})")
        print(f"\n[Phase 7] MC validation ({nsf} runs)")
        tZ,tM,sc=0,0,0;lZb,lMb,lLb=-1,-1,None
        for _ in range(nsf):
            rng=random.Random(self.rb.randint(0,2**31-1))
            rf=Refiner(thZ2,thM2,bsk,np_,rng,4)
            r=rf.sim()
            if r:
                Zf,Mf,lg=r;tZ+=Zf;tM+=Mf;sc+=1
                if Zf>lZb or (Zf==lZb and Mf>lMb): lZb,lMb,lLb=Zf,Mf,lg
        mZ=tZ/max(1,sc);mM=tM/max(1,sc)
        print(f"  Success: {sc}/{nsf} ({100*sc/nsf:.1f}%)")
        print(f"  MC Expected: Z={mZ:.1f}, M={mM:.1f}")
        print(f"  Best episode: Z={lZb}, M={lMb}")
        print(f"\n[Phase 8] Deterministic replay (seed=42)")
        dr=random.Random(42)
        wd=[0 if dr.random()<PN else 1 for _ in range(MAX_DAYS)]
        rf=Refiner(thZ2,thM2,bsk,np_,dr,4)
        r=rf.sim()
        if r:
            dZ,dM,dl=r;print(f"  Sample weather: Z={dZ}, M={dM}")
            if dl and lLb is not None and len(dl)>len(lLb): lLb,lZb,lMb=dl,dZ,dM
        else:
            print(f"  Deterministic replay failed, using best MC log")
        elapsed=time.time()-t0
        print(f"\n  Total time: {elapsed:.1f}s")
        if lLb is None:
            # Generate simple B->E fallback trace
            sf=State(x=B[0],y=B[1],O=INIT_O,H=INIT_H,F=INIT_F,M=INIT_M,Z=INIT_Z,c=0,day=1)
            lLb=[]
            while sf.day<=MAX_DAYS and sf.pos!=E:
                rng2=random.Random(42+sf.day)
                wf=0 if rng2.random()<PN else 1
                npos=mv_toward(sf,E)
                entry={"day":sf.day,"pos":(sf.x,sf.y),"O":sf.O,"H":sf.H,"F":sf.F,"M":sf.M,"Z":sf.Z,"c":sf.c,"weather":"N" if wf==0 else "S","action":"move","gain":0,"buy":None,"new_pos":npos}
                lLb.append(entry)
                sf=apply_action(sf,"move",wf,new_pos=npos)
                if sf is None: break
        return {"skeleton":bsk,"mc_Z":mZ,"mc_M":mM,"best_Z":lZb,"best_M":lMb,"best_log":lLb,"success_rate":sc/nsf if nsf>0 else 0,"elapsed_s":elapsed}

def gen_xls(result,out="result.xlsx"):
    try: import openpyxl
    except: import subprocess;subprocess.run(["pip","install","openpyxl"],check=True);import openpyxl
    wb=openpyxl.Workbook();ws=wb.active;ws.title="PBOVI"
    hd=["Day","PosX","PosY","Weather","Action","Point","FuelO","WaterH","FoodF","MoneyM","TargetZ","c","BuyO","BuyH","BuyF"]
    for c,h in enumerate(hd,1): ws.cell(row=1,column=c,value=h)
    iv=[0,B[0],B[1],"-","Start","B",INIT_O,INIT_H,INIT_F,INIT_M,INIT_Z,0,0,0,0]
    for c,v in enumerate(iv,1): ws.cell(row=2,column=c,value=v)
    lg=result.get("best_log",[])
    for i,e in enumerate(lg,3):
        ws.cell(row=i,column=1,value=e["day"]);ws.cell(row=i,column=2,value=e["pos"][0]);ws.cell(row=i,column=3,value=e["pos"][1])
        ws.cell(row=i,column=4,value=e["weather"]);ws.cell(row=i,column=5,value=e["action"])
        wp="";ps=e["pos"]
        for nm,(cr,_,_) in WORK_INFO.items():
            if ps==cr: wp=nm;break
        if ps==S1: wp="S1"
        elif ps==S2: wp="S2"
        elif ps==B: wp="B"
        elif ps==E: wp="E"
        ws.cell(row=i,column=6,value=wp);ws.cell(row=i,column=7,value=e["O"]);ws.cell(row=i,column=8,value=e["H"])
        ws.cell(row=i,column=9,value=e["F"]);ws.cell(row=i,column=10,value=e["M"]);ws.cell(row=i,column=11,value=e["Z"]);ws.cell(row=i,column=12,value=e["c"])
        bu=e.get("buy");ws.cell(row=i,column=13,value=bu[0] if bu else 0);ws.cell(row=i,column=14,value=bu[1] if bu else 0);ws.cell(row=i,column=15,value=bu[2] if bu else 0)
    wb.save(out);print(f"\n  Excel output: {out}")

if __name__=="__main__":
    print("="*65);print("  Problem 3 - PBOVI (Point-Based Online Value Iteration)")
    print("  Ref: Wu Bo PhD Thesis, Ch2.1 MDP, Ch3.3 PBOVI, Ch2.3 Belief Compression")
    print("="*65)
    solver=Solver(seed=42);result=solver.solve(200,150,2000)
    base=r"C:\Users\ming\Desktop\数模备赛"
    if result.get("best_log"): gen_xls(result,f"{base}\\result.xlsx")
    sm={"problem":"Task 3 (PBOVI)","method":"Point-Based Online Value Iteration","reference":"Wu Bo PhD Thesis Ch2.1, Ch3.3","skeleton":list(result["skeleton"]) if result["skeleton"] else [],"mc_expected_Z":result["mc_Z"],"mc_expected_M":result["mc_M"],"best_episode_Z":result["best_Z"],"best_episode_M":result["best_M"],"success_rate":result["success_rate"],"elapsed_s":result["elapsed_s"]}
    jp=f"{base}\\task3_pbovi_result.json"
    with open(jp,"w",encoding="utf-8") as f: json.dump(sm,f,ensure_ascii=False,indent=2)
    print(f"  JSON: {jp}\nComplete!")
