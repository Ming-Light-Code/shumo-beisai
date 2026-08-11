# -*- coding: utf-8 -*-
"""
ADP Solver for Problem 3 - True Approximate Policy Iteration (2 iterations)
Separate V_Z(s) and V_M(s) with lexicographic decision rule
"""
import numpy as np, random, copy, time, json
from collections import defaultdict
from dataclasses import dataclass

GRID=30; B=(1,15); E=(30,15); S1=(12,16); S2=(21,16)
W1=(6,21); W2=(15,9); W3=(24,24)
WORK_INFO={"W1":(W1,20,4),"W2":(W2,15,5),"W3":(W3,28,3)}
SUPPLY={"S1":S1,"S2":S2}
ALL_NODES={"B":B,"E":E,"S1":S1,"S2":S2,"W1":W1,"W2":W2,"W3":W3}
INIT_O,INIT_H,INIT_F=100,150,100; INIT_M=750; INIT_Z=200
MAX_LOAD=400; MAX_DAYS=90; PRICES=(2,1,2)
CMOVE=[(2,3,2),(8,4,3)]; CIDLE=[(1,1,1),(3,3,2)]; CWORK=[(5,4,3),(8,6,6)]
PN=0.8; PS=0.2
EMOVE=tuple(PN*CMOVE[0][i]+PS*CMOVE[1][i] for i in range(3))
EIDLE=tuple(PN*CIDLE[0][i]+PS*CIDLE[1][i] for i in range(3))
EWORK=tuple(PN*CWORK[0][i]+PS*CWORK[1][i] for i in range(3))
def md(p1,p2): return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])
def cns(wi,at):
    if at=="move": return CMOVE[wi]
    if at=="idle": return CIDLE[wi]
    if at=="work": return CWORK[wi]
    return (0,0,0)

@dataclass
class St:
    x:int;y:int;O:int;H:int;F:int;M:int;Z:int;c:int=0;day:int=1
    @property
    def pos(self): return (self.x,self.y)
    @pos.setter
    def pos(self,v): self.x,self.y=v
    def clone(self): return copy.deepcopy(self)
    def ok(self):
        if self.O<0 or self.H<0 or self.F<0 or self.M<0: return False
        if self.O+self.H+self.F>MAX_LOAD: return False
        return True
    def ld(self): return self.O+self.H+self.F

def act(state,action,weather,work_gain=0,new_pos=None):
    s=state.clone(); cr=cns(weather,action)
    s.O-=cr[0];s.H-=cr[1];s.F-=cr[2]
    if action=="work": s.Z+=work_gain;s.c+=1
    elif action=="move": s.pos=new_pos;s.c=0
    elif action=="idle": s.c=0
    s.day+=1; return s if s.ok() else None

def buy_act(state,bO,bH,bF):
    s=state.clone(); cost=bO*2+bH*1+bF*2
    if cost>s.M: return None
    s.O+=bO;s.H+=bH;s.F+=bF;s.M-=cost
    return s if s.ok() else None

def at_work(s):
    for nm,(pos,gain,mx) in WORK_INFO.items():
        if s.pos==pos: return nm,gain,mx
    return None,0,0

def at_supply(s):
    for nm,pos in SUPPLY.items():
        if s.pos==pos: return nm
    return None

def sw(): return 0 if random.random()<PN else 1

def compute_buy(state,skeleton,seg_idx,node_pos):
    nO,nH,nF=0.0,0.0,0.0;vO,vH,vF=0.0,0.0,0.0
    ns=len(skeleton)
    for j in range(seg_idx+1,len(skeleton)):
        if skeleton[j] in ["S1","S2"]: ns=j;break
    for j in range(seg_idx,min(ns,len(skeleton)-1)):
        n1=skeleton[j];n2=skeleton[min(j+1,len(skeleton)-1)]
        d=md(node_pos[n1],node_pos[n2])
        nO+=EMOVE[0]*d;nH+=EMOVE[1]*d;nF+=EMOVE[2]*d
        e2o=0.8*4+0.2*64;vO+=(e2o-EMOVE[0]**2)*d
        e2h=0.8*9+0.2*16;vH+=(e2h-EMOVE[1]**2)*d
        e2f=0.8*4+0.2*9;vF+=(e2f-EMOVE[2]**2)*d
        if n2 in WORK_INFO:
            ew=PN*WORK_INFO[n2][2]
            nO+=EWORK[0]*ew;nH+=EWORK[1]*ew;nF+=EWORK[2]*ew
            mx=WORK_INFO[n2][2];wv=mx*PN*PS
            vO+=wv*(CWORK[0][0]-CWORK[1][0])**2
            vH+=wv*(CWORK[0][1]-CWORK[1][1])**2
            vF+=wv*(CWORK[0][2]-CWORK[1][2])**2
    ec=nO*2+nH*1+nF*2;cv=state.O*2+state.H*1+state.F*2
    fn=max(0,ec-cv)
    sl=(state.M-fn)/max(1,fn) if fn>0 else 2.0
    z=0.5+1.5*min(1.0,max(0,sl))
    nO=int(np.ceil(nO+z*np.sqrt(max(0,vO))))
    nH=int(np.ceil(nH+z*np.sqrt(max(0,vH))))
    nF=int(np.ceil(nF+z*np.sqrt(max(0,vF))))
    bO=max(0,nO-state.O);bH=max(0,nH-state.H);bF=max(0,nF-state.F)
    sp=MAX_LOAD-state.ld();tot=bO+bH+bF
    if tot>sp>0: sc=sp/tot;bO=int(bO*sc);bH=int(bH*sc);bF=int(bF*sc)
    cst=bO*2+bH*1+bF*2
    if cst>state.M>0: sc=state.M/cst;bO=int(bO*sc);bH=int(bH*sc);bF=int(bF*sc)
    return bO,bH,bF

def move_toward(state,target):
    dx=np.sign(target[0]-state.x) or 0
    dy=np.sign(target[1]-state.y) or 0
    if dx==0 and dy==0: return state.pos
    return (state.x+dx,state.y+dy)

def enum_skels():
    nodes=ALL_NODES; im=["W1","W2","W3","S1","S2"]
    dist={}
    for n1 in nodes:
        for n2 in nodes: dist[(n1,n2)]=md(nodes[n1],nodes[n2])
    skels=[]
    def dfs(cur,path,travel):
        if len(skels)>=5000: return
        dE=dist[(cur,"E")]
        if travel+dE<=MAX_DAYS: skels.append(tuple(path+["E"]))
        for nxt in im:
            if nxt==cur: continue
            if cur in ["S1","S2"] and nxt in ["S1","S2"]: continue
            if len(path)>8: continue
            d=dist[(cur,nxt)]
            if d==0 or travel+d>MAX_DAYS: continue
            dfs(nxt,path+[nxt],travel+d)
    dfs("B",["B"],0)
    return skels,nodes

# Common state features (13 dims)
def state_feats(s):
    dE=md(s.pos,E); rem=(MAX_DAYS-s.day+1)/MAX_DAYS
    min_to_E=dE; t_p=min_to_E/max(1,MAX_DAYS-s.day+1)
    wnm,wg,wm=at_work(s); at_w=1.0 if wnm else 0.0
    wp_rem=(wm-s.c)/max(1,wm) if wnm else 0.0
    at_s=1.0 if at_supply(s) else 0.0
    # Z gain feature: if work at this point for all remaining quota
    gain_feat=0.0
    if wnm: gain_feat=wg*(wm-s.c)/max(1,wm)
    return np.array([1.0,dE/58.0,s.O/MAX_LOAD,s.H/MAX_LOAD,s.F/MAX_LOAD,
        s.M/750.0,rem,s.ld()/MAX_LOAD,at_w,wp_rem,gain_feat/30.0,
        s.c/5.0,at_s,t_p,1.0 if s.pos==E else 0.0],dtype=np.float64)
NF=15

def sim_base(skeleton,node_pos):
    s=St(x=B[0],y=B[1],O=INIT_O,H=INIT_H,F=INIT_F,M=INIT_M,Z=INIT_Z,c=0,day=1)
    log=[]; si=1; svs=[]
    while s.day<=MAX_DAYS:
        if s.pos==E:
            for sv in svs: sv["ret_Z"]=s.Z; sv["ret_M"]=s.M
            return s.Z,s.M,log,svs
        svs.append({"state":s.clone(),"ret_Z":0,"ret_M":0})
        entry={"day":s.day,"pos":(s.x,s.y),"O":s.O,"H":s.H,"F":s.F,"M":s.M,"Z":s.Z,"c":s.c}
        log.append(entry); w=sw(); wx="N" if w==0 else "S"
        sn=at_supply(s); wn,wg,wm=at_work(s)
        if sn and si<len(skeleton)-1:
            b=compute_buy(s,skeleton,si,node_pos)
            ns=buy_act(s,*b)
            if ns is None: return None
            entry["action"]=f"buy({b[0]},{b[1]},{b[2]})"; entry["weather"]=wx; entry["buy"]=b
            ns=act(ns,"idle",w)
            if ns is None: return None
            s=ns; si+=1; continue
        if wn:
            if w==0 and s.c<wm:
                ns=act(s,"work",w,work_gain=wg)
                entry["action"]="work"; entry["gain"]=wg
            else:
                ns=act(s,"idle",w)
                entry["action"]="idle_leave"; entry["gain"]=0; si+=1
            entry["weather"]=wx; entry["buy"]=None
            if ns is None: return None
            s=ns; continue
        if si>=len(skeleton): return None
        tgt_pos=node_pos[skeleton[si]]
        if s.pos==tgt_pos: si+=1; continue
        npos=move_toward(s,tgt_pos)
        ns=act(s,"move",w,new_pos=npos)
        if ns is None: return None
        entry["action"]="move"; entry["weather"]=wx
        entry["new_pos"]=npos; entry["gain"]=0; entry["buy"]=None
        s=ns
    return None

def sim_adp(skeleton,node_pos,thZ,thM,ws=None):
    s=St(x=B[0],y=B[1],O=INIT_O,H=INIT_H,F=INIT_F,M=INIT_M,Z=INIT_Z,c=0,day=1)
    log=[]; si=1; svs=[]
    def predZ(st): return 0.0 if thZ is None else thZ@state_feats(st)
    def predM(st): return 0.0 if thM is None else thM@state_feats(st)
    while s.day<=MAX_DAYS:
        if s.pos==E:
            for sv in svs: sv["ret_Z"]=s.Z; sv["ret_M"]=s.M
            return s.Z,s.M,log,svs
        svs.append({"state":s.clone(),"ret_Z":0,"ret_M":0})
        entry={"day":s.day,"pos":(s.x,s.y),"O":s.O,"H":s.H,"F":s.F,"M":s.M,"Z":s.Z,"c":s.c}
        log.append(entry)
        w=sw() if ws is None else ws[s.day-1]; wx="N" if w==0 else "S"
        sn=at_supply(s); wn,wg,wm=at_work(s)
        if sn and si<len(skeleton)-1:
            b=compute_buy(s,skeleton,si,node_pos)
            ns=buy_act(s,*b)
            if ns is None: return None
            entry["action"]=f"buy({b[0]},{b[1]},{b[2]})"; entry["weather"]=wx; entry["buy"]=b
            ns=act(ns,"idle",w)
            if ns is None: return None
            s=ns; si+=1; continue
        if wn:
            ns_w=act(s,"work",w,work_gain=wg) if w==0 and s.c<wm else None
            ns_i=act(s,"idle",w)
            if ns_w and ns_i:
                pZw,pMw=predZ(ns_w),predM(ns_w)
                pZi,pMi=predZ(ns_i),predM(ns_i)
                if pZw>pZi or (pZw==pZi and pMw>pMi):
                    ns=ns_w; entry["action"]="work"; entry["gain"]=wg
                else:
                    ns=ns_i; entry["action"]="idle_leave"; entry["gain"]=0; si+=1
            elif ns_i:
                ns=ns_i; entry["action"]="idle_leave"; entry["gain"]=0; si+=1
            else: return None
            entry["weather"]=wx; entry["buy"]=None; s=ns; continue
        if si>=len(skeleton): return None
        tgt_pos=node_pos[skeleton[si]]
        if s.pos==tgt_pos: si+=1; continue
        npos=move_toward(s,tgt_pos)
        ns=act(s,"move",w,new_pos=npos)
        if ns is None: return None
        entry["action"]="move"; entry["weather"]=wx
        entry["new_pos"]=npos; entry["gain"]=0; entry["buy"]=None
        s=ns
    return None

def fit_theta(samples,key):
    X=np.array([state_feats(s) for s,_ in samples])
    y=np.array([r for _,r in samples],dtype=np.float64)
    I=np.eye(NF)
    theta=np.linalg.solve(X.T@X+1.0*I,X.T@y)
    return theta

def solve_adp():
    t0=time.time()
    print("\nEnumerating skeletons...")
    skels,node_pos=enum_skels()
    print(f"  {len(skels)} skeletons")
    train_skels=[s for s in skels if any(n in WORK_INFO for n in s[1:-1])]
    n_train=min(len(train_skels),1000)

    # Iteration 1: Base policy
    print("\n"+"="*50)
    print("API Iteration 1: Base policy MC")
    print("="*50)
    sZ,sM=[],[]
    sk_s=[]
    print(f"  {n_train} skeletons, 200 MC each")
    for i,sk in enumerate(train_skels[:n_train]):
        if i%100==0: print(f"    {i}/{n_train}")
        tZ,tM,sc=0,0,0
        for _ in range(200):
            r=sim_base(sk,node_pos)
            if r:
                Zf,Mf,lg,svs=r; tZ+=Zf;tM+=Mf;sc+=1
                for sv in svs:
                    sZ.append((sv["state"],sv["ret_Z"]))
                    sM.append((sv["state"],sv["ret_M"]))
        if sc>=20: sk_s.append((sk,tZ/sc,tM/sc,sc))
    print(f"  {len(sZ)} state-return pairs, {len(sk_s)} feasible skeletons")

    # Show top base-policy skeletons
    sk_s.sort(key=lambda x:x[1],reverse=True)
    print("  Top base-policy skeletons:")
    for sk,az,am,sc in sk_s[:5]:
        print(f"    {' -> '.join(sk)}: Z={az:.1f}, M={am:.1f}, success={sc}/200")

    print("\nFitting V_Z, V_M...")
    if len(sZ)>50000:
        idx=np.random.choice(len(sZ),50000,replace=False)
        sZ=[sZ[i] for i in idx]; sM=[sM[i] for i in idx]
    thZ=fit_theta(sZ,"ret_Z"); thM=fit_theta(sM,"ret_M")
    print(f"  Fitted. theta_Z range: {thZ.min():.1f}~{thZ.max():.1f}, theta_M range: {thM.min():.1f}~{thM.max():.1f}")

    # ADP iter1 eval
    print("\nADP iter1 eval (top 30 skeletons):")
    test_sk=[sk for sk,_,_,_ in sk_s[:30]]+[("B","E")]
    bestZ,bestM=-1,-1; best_sk=None
    for sk in test_sk:
        tZ,tM,sc=0,0,0
        for _ in range(200):
            r=sim_adp(sk,node_pos,thZ,thM)
            if r: Zf,Mf,_,_=r; tZ+=Zf;tM+=Mf;sc+=1
        if sc>30:
            aZ=tZ/sc;aM=tM/sc
            if aZ>bestZ or (aZ==bestZ and aM>bestM): bestZ,bestM=aZ,aM; best_sk=sk
    if best_sk:
        print(f"  Best: {' -> '.join(best_sk)}, E[Z]={bestZ:.1f}, M={bestM:.1f}")
    else:
        best_sk=("B","E"); bestZ=INIT_Z; bestM=INIT_M
        print("  No feasible skeleton, using B->E")

    # Iteration 2: On-policy
    print("\n"+"="*50)
    print("API Iteration 2: On-policy sampling")
    print("="*50)
    sZ2,sM2=[],[]
    n2=min(len(train_skels),500)
    for i,sk in enumerate(train_skels[:n2]):
        for _ in range(30):
            r=sim_adp(sk,node_pos,thZ,thM)
            if r:
                Zf,Mf,lg,svs=r
                for sv in svs:
                    sZ2.append((sv["state"],sv["ret_Z"]))
                    sM2.append((sv["state"],sv["ret_M"]))
    print(f"  {len(sZ2)} new samples")

    allZ=sZ+sZ2; allM=sM+sM2
    print("  Refitting V_Z, V_M...")
    if len(allZ)>50000:
        idx=np.random.choice(len(allZ),50000,replace=False)
        allZ=[allZ[i] for i in idx]; allM=[allM[i] for i in idx]
    thZ2=fit_theta(allZ,"ret_Z"); thM2=fit_theta(allM,"ret_M")
    print(f"  Refitted. theta_Z range: {thZ2.min():.1f}~{thZ2.max():.1f}")

    # Final ADP evaluation
    print("\n"+"="*50)
    print("Final ADP evaluation")
    print("="*50)
    bestZ,bestM=-1,-1; best_sk=None; best_log=None
    for sk in test_sk:
        tZ,tM,sc=0,0,0; lZ,lM=-1,-1; lL=None
        for _ in range(200):
            r=sim_adp(sk,node_pos,thZ2,thM2)
            if r:
                Zf,Mf,lg,_=r; tZ+=Zf;tM+=Mf;sc+=1
                if Zf>lZ or (Zf==lZ and Mf>lM): lZ,lM=Zf,Mf; lL=lg
        if sc>30:
            aZ=tZ/sc;aM=tM/sc
            if aZ>bestZ or (aZ==bestZ and aM>bestM):
                bestZ,bestM=aZ,aM; best_sk=sk; best_log=lL
    if best_sk:
        print(f"  Best: {' -> '.join(best_sk)}")
        print(f"  E[Z]={bestZ:.1f}, E[M]={bestM:.1f}")
    else: best_sk=("B","E");bestZ=INIT_Z;bestM=INIT_M

    # Deterministic replay
    print("\n"+"="*50)
    print("Deterministic replay")
    print("="*50)
    rng=random.Random(42)
    ws2=[0 if rng.random()<PN else 1 for _ in range(MAX_DAYS)]
    r=sim_adp(best_sk,node_pos,thZ2,thM2,ws=ws2)
    fZ,fM,fL,_=(r if r else (bestZ,bestM,best_log,[]))
    print(f"  Sample weather: Z={fZ}, M={fM}")

    # MC evaluation
    print("\nMC evaluation (2000 runs)")
    tZ,tM,sc=0,0,0
    for _ in range(2000):
        r=sim_adp(best_sk,node_pos,thZ2,thM2)
        if r: Zf,Mf,_,_=r; tZ+=Zf;tM+=Mf;sc+=1
    mcZ,mcM=tZ/max(1,sc),tM/max(1,sc)
    print(f"  {sc}/2000, E[Z]={mcZ:.1f}, E[M]={mcM:.1f}")

    elapsed=time.time()-t0
    print(f"\nTotal: {elapsed:.1f}s")
    return {"skeleton":best_sk,"avg_Z":bestZ,"avg_M":bestM,"final_Z":fZ,"final_M":fM,
            "daily_log":fL,"elapsed":elapsed,"mc_Z":mcZ,"mc_M":mcM}

def gen_xls(result,out="result.xls"):
    try: import openpyxl
    except: import subprocess;subprocess.run(["pip","install","openpyxl"],check=True);import openpyxl
    wb=openpyxl.Workbook();ws=wb.active;ws.title="ADP"
    hd=["Day","PosX","PosY","Weather","Action","Point","FuelO","WaterH","FoodF","MoneyM","TargetZ","c","BuyO","BuyH","BuyF"]
    for c,h in enumerate(hd,1): ws.cell(row=1,column=c,value=h)
    iv=[0,B[0],B[1],"-","Start","B",INIT_O,INIT_H,INIT_F,INIT_M,INIT_Z,0,0,0,0]
    for c,v in enumerate(iv,1): ws.cell(row=2,column=c,value=v)
    lg=result.get("daily_log",[])
    for i,e in enumerate(lg,3):
        ws.cell(row=i,column=1,value=e["day"])
        ws.cell(row=i,column=2,value=e["pos"][0])
        ws.cell(row=i,column=3,value=e["pos"][1])
        ws.cell(row=i,column=4,value=e["weather"])
        ws.cell(row=i,column=5,value=e["action"])
        wp="";ps=e["pos"]
        for nm,(cr,_,_) in WORK_INFO.items():
            if ps==cr: wp=nm;break
        if ps==S1: wp="S1"
        elif ps==S2: wp="S2"
        ws.cell(row=i,column=6,value=wp)
        ws.cell(row=i,column=7,value=e["O"])
        ws.cell(row=i,column=8,value=e["H"])
        ws.cell(row=i,column=9,value=e["F"])
        ws.cell(row=i,column=10,value=e["M"])
        ws.cell(row=i,column=11,value=e["Z"])
        ws.cell(row=i,column=12,value=e["c"])
        bu=e.get("buy")
        ws.cell(row=i,column=13,value=bu[0] if bu else 0)
        ws.cell(row=i,column=14,value=bu[1] if bu else 0)
        ws.cell(row=i,column=15,value=bu[2] if bu else 0)
    wb.save(out);print(f"  Excel: {out}")

if __name__=="__main__":
    print("="*50)
    print("  Problem 3 | ADP (2-iter API)")
    print("="*50)
    res=solve_adp()
    if res.get("daily_log"): gen_xls(res,"result.xls")
    sm={"problem":"Task 3 (ADP)","method":"API_2iter",
        "skeleton":list(res["skeleton"]),"avg_Z":res["avg_Z"],"avg_M":res["avg_M"],
        "sample_Z":res["final_Z"],"sample_M":res["final_M"],
        "mc_Z":res.get("mc_Z",0),"mc_M":res.get("mc_M",0),"elapsed_s":res["elapsed"]}
    with open("adp_task3_result.json","w",encoding="utf-8") as f:
        json.dump(sm,f,ensure_ascii=False,indent=2)
    print("  JSON: adp_task3_result.json\n  Complete!")