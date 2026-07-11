#!/usr/bin/env python3
"""Q2: Geometric Prediction + Multi-start NLP Solver v2. Precision: 6 decimals."""
import numpy as np
from scipy.optimize import minimize, Bounds
import time, sys

g=9.8; Vm=300.0; Vs=3.0; R=10.0; Tmax=20.0; DT=0.005
FY1_0=np.array([17800.0,0.0,1800.0])
M1_0=np.array([20000.0,0.0,2000.0])
TGT_C=np.array([0.0,200.0,5.0])
DECOY=np.array([0.0,0.0,0.0])
dM=DECOY-M1_0; distD=np.linalg.norm(dM); dM_u=dM/distD
Tf=distD/Vm; NT=int(Tf/DT)+1
TGRID=np.linspace(0,Tf,NT)
M1T=M1_0+Vm*dM_u*TGRID[:,np.newaxis]
TGT_PTS=np.array([[0,200,5],[7,200,5],[-7,200,5],[0,207,5],[0,193,5],[0,200,10],[0,200,0]])

def occ_time(xb,yb,zb,tb):
    if tb>=Tf: return 0.0
    t0,t1=tb,min(tb+Tmax,Tf)
    if t0>=t1: return 0.0
    i0=int(t0/DT); i1=min(NT,int(t1/DT)+1)
    n=i1-i0
    if n<=0: return 0.0
    ts=TGRID[i0:i1]; Ms=M1T[i0:i1]
    zc=zb-Vs*(ts-tb)
    occ=np.zeros(n,dtype=bool)
    for tp in TGT_PTS:
        ld=tp-Ms; ll=np.linalg.norm(ld,axis=1)
        v=ll>1e-9
        C=np.column_stack([np.full(n,xb),np.full(n,yb),zc])
        ts2=C-Ms; cr=np.cross(ld,ts2); cn=np.linalg.norm(cr,axis=1)
        ds=np.full(n,np.inf); ds[v]=cn[v]/ll[v]
        dt=np.sum(ts2*ld,axis=1); lm=np.full(n,-1.0); lm[v]=dt[v]/(ll[v]**2)
        occ|=(ds<=R)&(lm>=0)&(lm<=1)
    return np.sum(occ)*DT

def occ_from_vars(x):
    a,v,td,tb=x
    xb=FY1_0[0]+v*np.cos(a)*tb
    yb=FY1_0[1]+v*np.sin(a)*tb
    zb=FY1_0[2]-0.5*g*(tb-td)**2
    if zb<-10: return 0.0
    return occ_time(xb,yb,zb,tb)

def smooth_occ(xb,yb,zb,tb,sigma=0.5):
    if tb>=Tf: return 0.0
    t0,t1=tb,min(tb+Tmax,Tf)
    if t0>=t1: return 0.0
    i0=int(t0/DT); i1=min(NT,int(t1/DT)+1)
    n=i1-i0
    if n<=0: return 0.0
    ts=TGRID[i0:i1]; Ms=M1T[i0:i1]
    zc=zb-Vs*(ts-tb)
    ss=np.zeros(n)
    for tp in TGT_PTS:
        ld=tp-Ms; ll=np.linalg.norm(ld,axis=1); v=ll>1e-9
        C=np.column_stack([np.full(n,xb),np.full(n,yb),zc])
        ts2=C-Ms; cr=np.cross(ld,ts2); cn=np.linalg.norm(cr,axis=1)
        ds=np.full(n,1e6); ds[v]=cn[v]/ll[v]
        dt=np.sum(ts2*ld,axis=1); lm=np.full(n,-1.0); lm[v]=dt[v]/(ll[v]**2)
        # Clamp for numerical stability
        md=np.clip((R-ds)/sigma,-50,50)
        m0=np.clip(lm/sigma,-50,50)
        m1=np.clip((1-lm)/sigma,-50,50)
        sd=1/(1+np.exp(-md))
        s0=1/(1+np.exp(-m0))
        s1=1/(1+np.exp(-m1))
        ss=np.maximum(ss,sd*s0*s1)
    return np.clip(np.sum(ss)*DT,0,Tmax)

def smooth_obj(x):
    a,v,td,tb=x
    xb=FY1_0[0]+v*np.cos(a)*tb
    yb=FY1_0[1]+v*np.sin(a)*tb
    zb=FY1_0[2]-0.5*g*(tb-td)**2
    if zb<-10: return 0.0
    return -smooth_occ(xb,yb,zb,tb,sigma=0.5)

def disc_obj(x): return -occ_from_vars(x)

def gen_starts(max_s=50):
    """Improved geometric prediction: multi-target LOS + lateral offsets + grid."""
    starts=[]

    # Multiple target points for broader LOS coverage
    all_tgts = TGT_PTS  # 7 target points

    # Time grid: coarse + fine near early times
    tb_vals = np.unique(np.concatenate([
        np.linspace(0.3, 3.0, 20),       # early window (where best solutions lie)
        np.linspace(3.0, Tf-0.5, 20)     # rest of flight
    ]))

    for tb in tb_vals:
        Mt = M1_0 + Vm * dM_u * tb
        for tp in all_tgts:
            los = tp - Mt
            ln = np.linalg.norm(los)
            if ln < 1e-6: continue
            lu = los / ln
            dmn = R * 1.1
            dmx = ln - R * 1.1
            if dmx <= dmn: continue

            # Dense sampling along LOS
            for D in np.linspace(dmn, dmx, 15):
                Cbase = Mt + D * lu
                xb0, yb0, zb0 = Cbase

                # Add perpendicular offsets (lateral)
                # Direction perpendicular to LOS in y-z plane
                perp = np.cross(lu, np.array([1,0,0]))
                if np.linalg.norm(perp) < 0.1:
                    perp = np.cross(lu, np.array([0,1,0]))
                perp = perp / np.linalg.norm(perp)

                for off in np.linspace(-R*0.8, R*0.8, 5):
                    C = Cbase + off * perp
                    xb, yb, zb = C

                    dx = xb - FY1_0[0]
                    dy = yb - FY1_0[1]
                    dxy = np.sqrt(dx*dx + dy*dy)
                    if tb <= 0 or dxy <= 0: continue

                    v = dxy / tb
                    if not (69.99 <= v <= 140.01): continue

                    alpha = np.arctan2(dy, dx)
                    fh = max(0.0, FY1_0[2] - zb)
                    ft = np.sqrt(2.0 * fh / g) if fh > 0 else 0.0
                    td = tb - ft
                    if td < 0: continue

                    oc = occ_time(xb, yb, zb, tb)
                    if oc > 0.01:
                        starts.append((alpha, v, td, tb, oc))

    # Add existing best as seed
    ao_ex = np.radians(6.01); vo_ex = 96.53; tdo_ex = 0.244; tbo_ex = 1.0
    xbo=FY1_0[0]+vo_ex*np.cos(ao_ex)*tbo_ex
    ybo=FY1_0[1]+vo_ex*np.sin(ao_ex)*tbo_ex
    zbo=FY1_0[2]-0.5*g*(tbo_ex-tdo_ex)**2
    oc_ex = occ_time(xbo,ybo,zbo,tbo_ex)
    starts.append((ao_ex,vo_ex,tdo_ex,tbo_ex,oc_ex))

    # Add perturbation around existing best
    for _ in range(10):
        da=ao_ex+np.random.uniform(-0.1,0.1)
        dv=vo_ex+np.random.uniform(-10,10)
        dtd=tdo_ex+np.random.uniform(-0.2,0.2)
        dtb=tbo_ex+np.random.uniform(-0.3,0.3)
        if dtd<0: dtd=0
        if dtb<=dtd: dtb=dtd+0.01
        xb=FY1_0[0]+dv*np.cos(da)*dtb
        yb=FY1_0[1]+dv*np.sin(da)*dtb
        zb=FY1_0[2]-0.5*g*(dtb-dtd)**2
        oc=occ_time(xb,yb,zb,dtb)
        if oc>0.01: starts.append((da,dv,dtd,dtb,oc))

    # Sort and deduplicate
    starts.sort(key=lambda s: -s[4])
    flt = []
    for s in starts:
        a, v, td, tb, oc = s
        tc = False
        for f in flt:
            da = abs(a - f[0])
            if da > np.pi: da = 2*np.pi - da
            if da < 0.015 and abs(v - f[1]) < 1.5 and abs(tb - f[3]) < 0.4:
                tc = True; break
        if not tc: flt.append(s)
        if len(flt) >= max_s: break

    return flt[:max_s]

bounds=Bounds([0,70,0,0.1],[2*np.pi,140,Tf,Tf-0.1])
cons=[{'type':'ineq','fun':lambda x:x[3]-x[2]-1e-6},
      {'type':'ineq','fun':lambda x:FY1_0[2]-0.5*g*(x[3]-x[2])**2+1e-6}]

def run_nlp(x0,maxiter=500):
    x=np.array(x0,dtype=float)
    r1=minimize(smooth_obj,x,method='SLSQP',bounds=bounds,constraints=cons,
                options={'maxiter':maxiter,'ftol':1e-8,'disp':False})
    x=r1.x
    r2=minimize(disc_obj,x,method='SLSQP',bounds=bounds,constraints=cons,
                options={'maxiter':maxiter//2,'ftol':1e-12,'disp':False})
    return r2.x,-r2.fun,r2.success

def run_nm(x0,maxiter=800):
    def po(x):
        a,v,td,tb=x; p=0
        if v<70: p+=(70-v)*1000
        if v>140: p+=(v-140)*1000
        if td<0: p+=(-td)*1000
        if tb<=td: p+=(td-tb+0.01)*1000
        if tb>=Tf: p+=(tb-Tf+0.01)*1000
        zb=FY1_0[2]-0.5*g*(tb-td)**2
        if zb<-1: p+=(-zb)*1000
        return -occ_from_vars(x)+p
    r=minimize(po,np.array(x0),method='Nelder-Mead',
               options={'maxiter':maxiter,'xatol':1e-8,'fatol':1e-8,'disp':False})
    return r.x,-r.fun,r.success

def grid_refine(x):
    xb=np.array(x,dtype=float)
    fb=occ_from_vars(xb)
    for sf in [1e-4,1e-2,1e-5,1e-4]:
        imp=True
        while imp:
            imp=False
            for d in range(4):
                for sgn in [-1,1]:
                    xt=xb.copy(); xt[d]+=sgn*sf
                    xt[0]%=2*np.pi; xt[1]=np.clip(xt[1],70,140)
                    xt[2]=max(0,xt[2]); xt[3]=np.clip(xt[3],xt[2]+1e-6,Tf-1e-6)
                    ft=occ_from_vars(xt)
                    if ft>fb+1e-10: xb=xt; fb=ft; imp=True
            sf*=0.1
    return xb,fb

if __name__=='__main__':
    print('='*65)
    print(' Q2: Geometric Prediction + Multi-start NLP Solver v2')
    print(f' M1 flight: {Tf:.6f}s | DT={DT}s | 6-decimal precision')
    print('='*65)
    t0=time.time()
    np.random.seed(42)
    starts=gen_starts(50)
    t1=time.time()
    print(f'\n  Geometric prediction: {len(starts)} starts ({t1-t0:.2f}s)')
    print(f'\n  Top 10 geometric starts:')
    for i,s in enumerate(starts[:10]):
        a,v,td,tb,oc=s
        print(f'    {i+1:2d}. a={np.degrees(a):7.3f}deg v={v:6.2f} td={td:5.3f} tb={tb:5.3f} occ={oc:.4f}')
    if not starts: print('ERROR: no starts!'); sys.exit(1)

    print(f'\n  Running multi-start NLP ({len(starts)} starts)...')
    res=[]
    for i,s in enumerate(starts):
        a,v,td,tb,oi=s; x0=np.array([a,v,td,tb])
        try: xo,fo,ok=run_nlp(x0)
        except: ok=False
        if not ok or fo<oi*0.5:
            try: xo,fo,ok=run_nm(x0)
            except: ok=False
        if not ok: xo,fo=x0,oi
        xr,fr=grid_refine(xo)
        res.append({'i':i,'io':oi,'x':xr,'f':fr})
        if (i+1)%10==0:
            bf=max(r['f'] for r in res)
            print(f'    ... {i+1}/{len(starts)} done ({time.time()-t1:.1f}s), best={bf:.4f}s')

    t2=time.time()
    res.sort(key=lambda r:-r['f'])
    print(f'\n  NLP done: {t2-t1:.2f}s')

    print(f'\n  Top 20 NLP results:')
    for i,r in enumerate(res[:20]):
        a,v,td,tb=r['x']; oc=r['f']; imp=oc-r['io']
        print(f'    {i+1:2d}. a={np.degrees(a):10.6f}deg v={v:10.6f} td={td:10.6f} tb={tb:10.6f} occ={oc:10.6f} ({imp:+.6f})')

    best=res[0]; ao,vo,tdo,tbo=best['x']; oco=best['f']
    xbo=FY1_0[0]+vo*np.cos(ao)*tbo
    ybo=FY1_0[1]+vo*np.sin(ao)*tbo
    zbo=FY1_0[2]-0.5*g*(tbo-tdo)**2
    dxo=FY1_0[0]+vo*np.cos(ao)*tdo
    dyo=FY1_0[1]+vo*np.sin(ao)*tdo
    print(f'\n'+'='*65)
    print('  GLOBAL OPTIMAL SOLUTION (6-decimal precision)')
    print('='*65)
    print(f'  Decision variables:')
    print(f'    alpha         = {ao:.6f} rad  ({np.degrees(ao):.6f} deg)')
    print(f'    speed v       = {vo:.6f} m/s')
    print(f'    drop time td  = {tdo:.6f} s')
    print(f'    detonate tb   = {tbo:.6f} s')
    print(f'    fall duration = {tbo-tdo:.6f} s')
    print(f'  Derived positions:')
    print(f'    FY1 initial   = ({FY1_0[0]:.1f}, {FY1_0[1]:.1f}, {FY1_0[2]:.1f})')
    print(f'    Drop point    = ({dxo:.6f}, {dyo:.6f}, {FY1_0[2]:.6f})')
    print(f'    Detonation    = ({xbo:.6f}, {ybo:.6f}, {zbo:.6f})')
    print(f'  Objective:')
    print(f'    Max occlusion = {oco:.6f} s')
    print(f'\n  Constraint verification:')
    ok=True
    for l,c in [('v in [70,140]',70<=vo<=140),('td >= 0',tdo>=0),
                 ('tb > td',tbo>tdo),('zb >= 0',zbo>=0),('tb < Tf',tbo<Tf),
                 ('td < Tf',tdo<Tf)]:
        st='[OK]' if c else '[FAIL]'
        print(f'    {st} {l}')
        if not c: ok=False
    print(f'    All constraints: {"PASS" if ok else "FAIL"}')

    aos=[r['f'] for r in res]
    print(f'\n  Consistency analysis:')
    print(f'    Global best:            {oco:.6f} s')
    print(f'    Mean (top 10):          {np.mean(aos[:10]):.6f} s')
    print(f'    Std (top 10):           {np.std(aos[:10]):.6f} s')
    tc=sum(1 for o in aos if abs(o-oco)<0.005)
    print(f'    Converged near best:    {tc}/{len(res)} (within 0.005s)')
    print(f'  Total time: {time.time()-t0:.2f}s')
    print('='*65)
