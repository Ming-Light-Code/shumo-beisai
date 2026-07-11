import numpy as np, time

g=9.8; Vm=300.0; Vs=3.0; R=10.0; Tmax=20.0; DT=0.005
FY1_0=np.array([17800.0,0.0,1800.0]); M1_0=np.array([20000.0,0.0,2000.0])
TGT_C=np.array([0.0,200.0,5.0]); DECOY=np.array([0.0,0.0,0.0])
dM=DECOY-M1_0; distD=np.linalg.norm(dM); dM_u=dM/distD; Tf=distD/Vm
NT=int(Tf/DT)+1; TGRID=np.linspace(0,Tf,NT); M1T=M1_0+Vm*dM_u*TGRID[:,np.newaxis]
TGT_PTS=np.array([[0,200,5],[7,200,5],[-7,200,5],[0,207,5],[0,193,5],[0,200,10],[0,200,0]])
MAX_FALL=np.sqrt(2*FY1_0[2]/g)

def occ_time(xb,yb,zb,tb):
    if tb>=Tf: return 0.0
    t0,t1=tb,min(tb+Tmax,Tf)
    if t0>=t1: return 0.0
    i0=int(t0/DT); i1=min(NT,int(t1/DT)+1); n=i1-i0
    if n<=0: return 0.0
    ts=TGRID[i0:i1]; Ms=M1T[i0:i1]; zc=zb-Vs*(ts-tb)
    occ=np.zeros(n,dtype=bool)
    for tp in TGT_PTS:
        ld=tp-Ms; ll=np.linalg.norm(ld,axis=1); v=ll>1e-9
        C=np.column_stack([np.full(n,xb),np.full(n,yb),zc])
        ts2=C-Ms; cr=np.cross(ld,ts2); cn=np.linalg.norm(cr,axis=1)
        ds=np.full(n,np.inf); ds[v]=cn[v]/ll[v]
        dt_d=np.sum(ts2*ld,axis=1); lm=np.full(n,-1.0); lm[v]=dt_d[v]/(ll[v]**2)
        occ|=(ds<=R)&(lm>=0)&(lm<=1)
    return np.sum(occ)*DT

def occ_union(xb1,yb1,zb1,tb1,xb2,yb2,zb2,tb2):
    if tb1>=Tf and tb2>=Tf: return 0.0
    i0=int(max(0,min(tb1,tb2))/DT); i1=min(NT,int(Tf/DT)+1); n=i1-i0
    if n<=0: return 0.0
    ts=TGRID[i0:i1]; Ms=M1T[i0:i1]; ou=np.zeros(n,dtype=bool)
    for xb,yb,zb,tb in [(xb1,yb1,zb1,tb1),(xb2,yb2,zb2,tb2)]:
        if tb>=Tf: continue
        zc=zb-Vs*np.maximum(0,ts-tb); o=np.zeros(n,dtype=bool)
        for tp in TGT_PTS:
            ld=tp-Ms; ll=np.linalg.norm(ld,axis=1); v=ll>1e-9
            C=np.column_stack([np.full(n,xb),np.full(n,yb),zc])
            ts2=C-Ms; cr=np.cross(ld,ts2); cn=np.linalg.norm(cr,axis=1)
            ds=np.full(n,np.inf); ds[v]=cn[v]/ll[v]
            dt_d=np.sum(ts2*ld,axis=1); lm=np.full(n,-1.0); lm[v]=dt_d[v]/(ll[v]**2)
            o|=(ds<=R)&(lm>=0)&(lm<=1)&(ts>=tb)
        ou|=o
    return np.sum(ou)*DT

def fitness(x):
    a,v,td1,tb1,td2,tb2=x
    xb1=FY1_0[0]+v*np.cos(a)*tb1; yb1=FY1_0[1]+v*np.sin(a)*tb1
    zb1=FY1_0[2]-0.5*g*(tb1-td1)**2
    if zb1<-10: zb1=-10
    xb2=FY1_0[0]+v*np.cos(a)*tb2; yb2=FY1_0[1]+v*np.sin(a)*tb2
    zb2=FY1_0[2]-0.5*g*(tb2-td2)**2
    if zb2<-10: zb2=-10
    return occ_union(xb1,yb1,zb1,tb1,xb2,yb2,zb2,tb2)

# Best from previous sweep: a=6.48deg v=89.29 td1=0 tb1=0.05 td2=1.0 tb2=1.2 -> 6.160s
# Finer sweep around this optimum
print("Finer sweep around a=6.48deg v=89.3 tb1=0.05 tb2=1.2")
t0=time.time()
best_x=np.array([np.radians(6.482759),89.285714,0,0.05,1.0,1.2])
best_f=6.160

# Sweep params: finer grid around optimum
for a_d in np.linspace(3, 12, 20):
    for v_d in np.linspace(70, 120, 20):
        for tb1_d in [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08]:
            for tb2_d in np.linspace(0.8, 2.0, 20):
                for td2_d in np.linspace(0.5, 2.0, 8):
                    td1_d=0.0
                    x_try=np.array([np.radians(a_d),v_d,td1_d,tb1_d,td1_d+td2_d,tb2_d])
                    if x_try[3]<=x_try[2] or x_try[5]<=x_try[4]: continue
                    if x_try[3]>x_try[2]+MAX_FALL or x_try[5]>x_try[4]+MAX_FALL: continue
                    ft=fitness(x_try)
                    if ft>best_f:
                        best_f=ft; best_x=x_try.copy()
                        print(f"  {ft:.6f}s a={a_d:.3f}deg v={v_d:.2f} tb1={tb1_d:.3f} tb2={tb2_d:.3f} td2_d={td2_d:.3f}")

print(f"\nBest: {best_f:.6f}s")
a,v,td1,tb1,td2,tb2=best_x
xb1=FY1_0[0]+v*np.cos(a)*tb1; yb1=FY1_0[1]+v*np.sin(a)*tb1; zb1=FY1_0[2]-0.5*g*(tb1-td1)**2
xb2=FY1_0[0]+v*np.cos(a)*tb2; yb2=FY1_0[1]+v*np.sin(a)*tb2; zb2=FY1_0[2]-0.5*g*(tb2-td2)**2
o1=occ_time(xb1,yb1,zb1,tb1); o2=occ_time(xb2,yb2,zb2,tb2)
print(f"a={np.degrees(a):.6f}deg v={v:.6f} td1={td1:.6f} tb1={tb1:.6f} td2={td2:.6f} tb2={tb2:.6f}")
print(f"o1={o1:.4f}s o2={o2:.4f}s overlap={o1+o2-best_f:.4f}")
print(f"Gain: {best_f-5.850:.4f}s")
print(f"Time: {time.time()-t0:.1f}s")
