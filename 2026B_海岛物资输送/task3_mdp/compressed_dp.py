
import numpy as np
from math import comb, log, exp
from collections import namedtuple
import time, os

# ========== Parameters ==========
class Params:
    def __init__(self):
        self.T=90; self.capacity=400
        self.B=(1,15); self.E=(30,15)
        self.S=[(12,16),(21,16)]; self.W=[(6,21),(15,9),(24,24)]
        self.waypoints=[(10,15),(15,15),(20,15)]
        self.macro_nodes=[self.B]+self.waypoints+self.S+self.W+[self.E]
        self.n_nodes=len(self.macro_nodes)
        self.n_res_levels=3; self.res_step=100
        self.n_m_levels=4; self.m_step=375
        self.reward=[20,15,28]; self.max_work=[4,5,3]
        self.price=[2,1,2]
        self.normal_move=[2,3,2]; self.normal_idle=[1,1,1]; self.normal_work=[5,4,3]
        self.storm_move=[8,4,3]; self.storm_idle=[3,3,2]; self.storm_work=[8,6,6]
        self.p_normal=0.8; self.p_storm=0.2
        self.init_O=100; self.init_H=150; self.init_F=100; self.init_M=750; self.init_Z=200
        self._precompute()

    def _precompute(self):
        n=self.n_nodes
        self.dist=np.zeros((n,n),dtype=int)
        for i in range(n):
            for j in range(n):
                self.dist[i,j]=abs(self.macro_nodes[i][0]-self.macro_nodes[j][0])+abs(self.macro_nodes[i][1]-self.macro_nodes[j][1])
        combos=[]
        for o in range(self.n_res_levels):
            for h in range(self.n_res_levels):
                for f in range(self.n_res_levels):
                    if o+h+f<=self.n_res_levels-1: combos.append((o,h,f))
        self.res_combos=combos
        self.n_res_combos=len(combos)
        self.res_combo_to_idx={c:i for i,c in enumerate(combos)}
        ws=[(0,0)]
        for d in range(1,5): ws.append((d,1))
        for d in range(1,6): ws.append((d,2))
        for d in range(1,4): ws.append((d,3))
        self.work_states=ws
        self.n_work_states=len(ws)
        self.work_state_to_idx={s:i for i,s in enumerate(ws)}

p=Params()

# ========== Binomial PMF ==========
def binom_pmf(k,n,pr):
    if k<0 or k>n: return 0.0
    if pr==0: return 1.0 if k==0 else 0.0
    if pr==1: return 1.0 if k==n else 0.0
    # Use log for numerical stability
    try:
        l=log(comb(n,k))+k*log(pr)+(n-k)*log(1-pr)
        return exp(l) if l>-700 else 0.0
    except: return 0.0

# ========== State encoding ==========
def encode_state(pos_idx,res_idx,m_bin,work_idx,t_rem):
    return (((pos_idx*p.n_res_combos+res_idx)*p.n_m_levels+m_bin)*p.n_work_states+work_idx)*(p.T+1)+t_rem

def total_states():
    return p.n_nodes*p.n_res_combos*p.n_m_levels*p.n_work_states*(p.T+1)

# ========== Precompute transitions ==========
def precompute_transitions():
    move_trans={}
    wi_trans={}
    # MOVE transitions
    for orig in range(p.n_nodes):
        for targ in range(p.n_nodes):
            if orig==targ: continue
            d=p.dist[orig,targ]
            if d==0: continue
            for res_idx,(o_b,h_b,f_b) in enumerate(p.res_combos):
                key=(orig,targ,res_idx)
                outcomes=[]
                o_val=o_b*p.res_step; h_val=h_b*p.res_step; f_val=f_b*p.res_step
                for k in range(d+1):
                    prob=binom_pmf(k,d,p.p_storm)
                    if prob<1e-12: continue
                    cO=d*p.normal_move[0]+k*(p.storm_move[0]-p.normal_move[0])
                    cH=d*p.normal_move[1]+k*(p.storm_move[1]-p.normal_move[1])
                    cF=d*p.normal_move[2]+k*(p.storm_move[2]-p.normal_move[2])
                    no=o_val-cO; nh=h_val-cH; nf=f_val-cF
                    if no<0 or nh<0 or nf<0: continue
                    ob2=int(np.clip(no//p.res_step,0,p.n_res_levels-1))
                    hb2=int(np.clip(nh//p.res_step,0,p.n_res_levels-1))
                    fb2=int(np.clip(nf//p.res_step,0,p.n_res_levels-1))
                    if ob2+hb2+fb2>p.n_res_levels-1: continue
                    nr=p.res_combo_to_idx.get((ob2,hb2,fb2))
                    if nr is None: continue
                    outcomes.append((prob,nr))
                if outcomes:
                    total=sum(o[0] for o in outcomes)
                    if total>0:
                        outcomes=[(pt/total,nr) for pt,nr in outcomes]
                    move_trans[key]=outcomes
                else:
                    move_trans[key]=[]
    # IDLE transitions
    for node in range(p.n_nodes):
        for res_idx,(o_b,h_b,f_b) in enumerate(p.res_combos):
            o_val=o_b*p.res_step; h_val=h_b*p.res_step; f_val=f_b*p.res_step
            # IDLE
            outcomes=[]
            for weather,prb in [('normal',p.p_normal),('storm',p.p_storm)]:
                if weather=='normal': cO,cH,cF=p.normal_idle
                else: cO,cH,cF=p.storm_idle
                no=o_val-cO; nh=h_val-cH; nf=f_val-cF
                if no<0 or nh<0 or nf<0: continue
                ob2=int(np.clip(no//p.res_step,0,p.n_res_levels-1))
                hb2=int(np.clip(nh//p.res_step,0,p.n_res_levels-1))
                fb2=int(np.clip(nf//p.res_step,0,p.n_res_levels-1))
                if ob2+hb2+fb2>p.n_res_levels-1: continue
                nr=p.res_combo_to_idx.get((ob2,hb2,fb2))
                if nr is None: continue
                outcomes.append((prb,nr))
            if outcomes:
                total=sum(o[0] for o in outcomes)
                wi_trans[(node,res_idx,'idle')]=[(p/total,nr) for p,nr in outcomes]
            else:
                wi_trans[(node,res_idx,'idle')]=[]
            # WORK (only at work nodes)
            if node in [6,7,8]:
                outcomes=[]
                for weather,prb in [('normal',p.p_normal),('storm',p.p_storm)]:
                    if weather=='normal': cO,cH,cF=p.normal_work
                    else: cO,cH,cF=p.storm_work
                    no=o_val-cO; nh=h_val-cH; nf=f_val-cF
                    if no<0 or nh<0 or nf<0: continue
                    ob2=int(np.clip(no//p.res_step,0,p.n_res_levels-1))
                    hb2=int(np.clip(nh//p.res_step,0,p.n_res_levels-1))
                    fb2=int(np.clip(nf//p.res_step,0,p.n_res_levels-1))
                    if ob2+hb2+fb2>p.n_res_levels-1: continue
                    nr=p.res_combo_to_idx.get((ob2,hb2,fb2))
                    if nr is None: continue
                    outcomes.append((prb,nr))
                if outcomes:
                    total=sum(o[0] for o in outcomes)
                    wi_trans[(node,res_idx,'work')]=[(p/total,nr) for p,nr in outcomes]
                else:
                    wi_trans[(node,res_idx,'work')]=[]
    return move_trans, wi_trans

# ========== DP Solver ==========
def solve_compressed_dp(cache_path='dp_value.npy'):
    if os.path.exists(cache_path):
        print(f'Loading cached DP: {cache_path}')
        return np.load(cache_path)
    print('Precomputing transitions...')
    move_trans,wi_trans=precompute_transitions()
    print(f'Move: {len(move_trans)}, Work/Idle: {len(wi_trans)} entries')
    
    total=total_states()
    print(f'Total states: {total:,}')
    ZS=100000; FV=-1e9
    V=np.full(total,FV,dtype=np.float64)
    
    for m_bin in range(p.n_m_levels):
        for res_idx in range(p.n_res_combos):
            sidx=encode_state(9,res_idx,m_bin,0,0)  # work_idx 0 = (0,0)
            V[sidx]=m_bin*p.m_step
    
    targets={}
    for pos in range(p.n_nodes):
        if pos==9: continue
        ts=[]
        for t in range(p.n_nodes):
            if t==pos: continue
            d=p.dist[pos,t]
            if d>0: ts.append((t,d))
        targets[pos]=ts
    
    t0=time.time()
    for tr in range(1,p.T+1):
        if tr%10==0:
            el=time.time()-t0; eta=el/tr*(p.T-tr)
            print(f'  DP t_rem={tr}/{p.T}  elapsed={el:.0f}s  ETA={eta:.0f}s')
        for pos in range(9):
            is_w=pos in [6,7,8]; is_s=pos in [4,5]
            wpid={6:1,7:2,8:3}.get(pos,0)
            for ri in range(p.n_res_combos):
                ob,hb,fb=p.res_combos[ri]
                ov=ob*p.res_step; hv=hb*p.res_step; fv=fb*p.res_step
                for mb in range(p.n_m_levels):
                    mv=mb*p.m_step
                    for wi,(wc,wpt) in enumerate(p.work_states):
                        if not is_w and wpt!=0: continue
                        if is_w and wpt!=wpid and wpt!=0: continue
                        if wpt==0 and wc!=0: continue
                        sidx=encode_state(pos,ri,mb,wi,tr)
                        bv=FV
                        # IDLE
                        ki=(pos,ri,'idle')
                        if ki in wi_trans and tr>=1:
                            iv=0.0
                            for prob,nr in wi_trans[ki]:
                                ns=encode_state(pos,nr,mb,wi,tr-1)
                                iv+=prob*V[ns]
                            bv=max(bv,iv)
                        # WORK
                        if is_w and wpt==wpid and wc<p.max_work[wpid-1]:
                            kw=(pos,ri,'work')
                            if kw in wi_trans and tr>=1:
                                wv=0.0; reward=p.reward[wpid-1]
                                nwc=wc+1
                                nws=p.work_state_to_idx.get((nwc,wpid))
                                if nws is not None:
                                    for prob,nr in wi_trans[kw]:
                                        ns=encode_state(pos,nr,mb,nws,tr-1)
                                        wv+=prob*(reward*ZS+V[ns])
                                bv=max(bv,wv)
                        # MOVE
                        for targ,d in targets[pos]:
                            if d>tr: continue
                            key=(pos,targ,ri)
                            if key not in move_trans: continue
                            mv2=0.0
                            for prob,nr in move_trans[key]:
                                nwc2=0; nwpt2=0
                                if targ in [6,7,8]: nwpt2={6:1,7:2,8:3}[targ]
                                nws2=p.work_state_to_idx.get((nwc2,nwpt2),0)
                                ns=encode_state(targ,nr,mb,nws2,tr-d)
                                mv2+=prob*V[ns]
                            bv=max(bv,mv2)
                        # BUY (at supply point: evaluate buy+idle and buy+move)
                        if is_s:
                            rem_cap=p.capacity-(ov+hv+fv)
                            # Smart buy strategies
                            buys=[(0,0,0)]
                            if rem_cap>0 and mv>0:
                                # Proportional fill
                                pr_sum=sum(p.price)
                                for frac in [0.3,0.6,1.0]:
                                    bo=int(frac*rem_cap*0.35); bh=int(frac*rem_cap*0.30); bf=int(frac*rem_cap*0.35)
                                    bo=min(bo,rem_cap); bh=min(bh,rem_cap-bo); bf=min(bf,rem_cap-bo-bh)
                                    cost=bo*p.price[0]+bh*p.price[1]+bf*p.price[2]
                                    if cost<=mv: buys.append((bo,bh,bf))
                                # Individual resource fills
                                for rsrc,cost_pu,max_b in [(0,p.price[0],rem_cap),(1,p.price[1],rem_cap),(2,p.price[2],rem_cap)]:
                                    for frac in [0.5,1.0]:
                                        amt=int(frac*min(max_b,mv//cost_pu))
                                        if amt>0:
                                            bb=[0,0,0]; bb[rsrc]=amt
                                            cost=amt*cost_pu
                                            if cost<=mv: buys.append(tuple(bb))
                            buys=list(set(buys))
                            for bo,bh,bf in buys:
                                if bo==0 and bh==0 and bf==0: continue
                                cost=bo*p.price[0]+bh*p.price[1]+bf*p.price[2]
                                if cost>mv: continue
                                no2=ov+bo; nh2=hv+bh; nf2=fv+bf; nm2=mv-cost
                                ob2=int(np.clip(no2//p.res_step,0,4))
                                hb2=int(np.clip(nh2//p.res_step,0,4))
                                fb2=int(np.clip(nf2//p.res_step,0,4))
                                if ob2+hb2+fb2>4: continue
                                nr2=p.res_combo_to_idx.get((ob2,hb2,fb2))
                                if nr2 is None: continue
                                mb2=int(np.clip(int(nm2//p.m_step),0,3))
                                post_val=-1e9
                                ki2=(pos,nr2,'idle')
                                if ki2 in wi_trans and tr>=1:
                                    iv2=0.0
                                    for prob,nr3 in wi_trans[ki2]:
                                        ns2=encode_state(pos,nr3,mb2,wi,tr-1)
                                        iv2+=prob*V[ns2]
                                    post_val=max(post_val,iv2)
                                for targ,d in targets[pos]:
                                    if d>tr: continue
                                    k2=(pos,targ,nr2)
                                    if k2 not in move_trans: continue
                                    mv3=0.0
                                    for prob,nr3 in move_trans[k2]:
                                        nwc3=0; nwpt3=0
                                        if targ in [6,7,8]: nwpt3={6:1,7:2,8:3}[targ]
                                        nws3=p.work_state_to_idx.get((nwc3,nwpt3),0)
                                        ns3=encode_state(targ,nr3,mb2,nws3,tr-d)
                                        mv3+=prob*V[ns3]
                                    post_val=max(post_val,mv3)
                                if post_val>-1e8:
                                    bv=max(bv,post_val-cost*1e-6)
                        V[sidx]=bv
    el=time.time()-t0
    print(f'DP complete in {el:.0f}s, saving...')
    np.save(cache_path,V)
    return V

def map_to_compressed(pos, O, H, F, M, consec_work, work_pt, t_rem, V):
    ob=int(np.clip(O//p.res_step,0,p.n_res_levels-1))
    hb=int(np.clip(H//p.res_step,0,p.n_res_levels-1))
    fb=int(np.clip(F//p.res_step,0,p.n_res_levels-1))
    if ob+hb+fb>p.n_res_levels-1:
        tot=ob+hb+fb; scl=(p.n_res_levels-1)/tot
        ob=int(ob*scl); hb=int(hb*scl); fb=int(fb*scl)
    ri=p.res_combo_to_idx.get((ob,hb,fb))
    if ri is None: return None,None
    mb=int(np.clip(M//p.m_step,0,p.n_m_levels-1))
    wc=consec_work if work_pt>0 else 0
    ws=p.work_state_to_idx.get((wc,work_pt))
    if ws is None: return None,None
    best_v=-1e9; best_n=-1
    for ni in range(p.n_nodes):
        d=abs(pos[0]-p.macro_nodes[ni][0])+abs(pos[1]-p.macro_nodes[ni][1])
        if d>t_rem: continue
        exp_O=O-d*(p.p_normal*p.normal_move[0]+p.p_storm*p.storm_move[0])
        exp_H=H-d*(p.p_normal*p.normal_move[1]+p.p_storm*p.storm_move[1])
        exp_F=F-d*(p.p_normal*p.normal_move[2]+p.p_storm*p.storm_move[2])
        if exp_O<0 or exp_H<0 or exp_F<0: continue
        ob2=int(np.clip(exp_O//p.res_step,0,4))
        hb2=int(np.clip(exp_H//p.res_step,0,4))
        fb2=int(np.clip(exp_F//p.res_step,0,4))
        if ob2+hb2+fb2>4:
            tot=ob2+hb2+fb2; scl=4/tot
            ob2=int(ob2*scl); hb2=int(hb2*scl); fb2=int(fb2*scl)
        ri2=p.res_combo_to_idx.get((ob2,hb2,fb2))
        if ri2 is None: continue
        nwp=0
        if ni in [6,7,8]: nwp={6:1,7:2,8:3}[ni]
        nws=p.work_state_to_idx.get((0,nwp),0)
        sidx=encode_state(ni,ri2,mb,nws,max(0,t_rem-d))
        v=V[sidx]
        if v>best_v: best_v=v; best_n=ni
        if best_v > -1e8 and best_n >= 0:
            best_d = abs(pos[0]-p.macro_nodes[best_n][0])+abs(pos[1]-p.macro_nodes[best_n][1])
            best_v = best_v - best_d * 10.0
        return best_v,best_n

if __name__=='__main__':
    print('Solving compressed DP...')
    V=solve_compressed_dp()
    nz=np.sum(V>-1e8)
    print(f'Non-failing states: {nz} / {len(V)}')
