"""
fast_eval.py -- Fast vectorized shielding evaluation.
Pre-computes missile trajectories and uses numpy broadcasting for speed.
Uses simplified center-line check for screening and full sphere projection for verification.
"""
import numpy as np, math
from math import cos, sin, pi, sqrt, atan2

G=9.8; MS=300.0; CS=3.0; CR=10.0; CD=20.0; LI=1.0; DTS=0.1
VMIN,VMAX=70.0,140.0
TT=np.array([0.0,200.0,0.0]); FT=np.array([0.0,0.0,0.0])
MI=np.array([[2e4,0,2e3],[1.9e4,600,2.1e3],[1.8e4,-600,1.9e3]])
MN=["M1","M2","M3"]
DI=np.array([[17800,0,1800],[12000,1400,1400],[6000,-3000,700],[11000,2000,1800],[13000,-2000,1300]])
DN=["FY1","FY2","FY3","FY4","FY5"]; ND,NM,RT,HT=5,3,7.0,10.0

MD=np.zeros((NM,3))
for k in range(NM): d=FT-MI[k]; MD[k]=d/np.linalg.norm(d)
TMAX=max(np.linalg.norm(FT-MI[k]) for k in range(NM))/MS
NTS=int(TMAX/DTS)+1; TA=np.arange(NTS)*DTS
MP=np.zeros((NM,NTS,3))
for k in range(NM):
    for ti in range(NTS): MP[k,ti]=MI[k]+TA[ti]*MS*MD[k]

LP=np.zeros((NM,NTS,6))
for k in range(NM):
    for ti in range(NTS):
        mx,my,mz=MP[k,ti]
        LP[k,ti,0]=TT[0]-mx; LP[k,ti,1]=TT[1]-my; LP[k,ti,2]=TT[2]-mz
        LP[k,ti,3]=mx; LP[k,ti,4]=my; LP[k,ti,5]=mz

def _gen_pts(nc,ns):
    pts=np.zeros((2*nc+2*ns,3)); idx=0
    for k in range(nc):
        a=2.0*np.pi*k/nc
        pts[idx]=[RT*np.cos(a),TT[1]+RT*np.sin(a),TT[2]+HT]; idx+=1
    for k in range(nc):
        a=2.0*np.pi*k/nc
        pts[idx]=[RT*np.cos(a),TT[1]+RT*np.sin(a),TT[2]]; idx+=1
    for k in range(ns):
        z=HT*k/(ns-1); pts[idx]=[-RT,TT[1],z]; idx+=1
    for k in range(ns):
        z=HT*k/(ns-1); pts[idx]=[RT,TT[1],z]; idx+=1
    return pts
PTS=_gen_pts(128,64)
M_PTS=len(PTS)

def smoke_burst_pos_fast(init_pos,speed,heading,tr,delay):
    d=np.array([cos(heading),sin(heading),0.])
    return init_pos+speed*(tr+delay)*d-np.array([0.,0.,0.5*G*delay*delay])

def fast_shield_center(di,th,v,tk,tr,tf):
    """Center-line approximation: smoke near missile-target ray. Very fast."""
    r0=DI[di]; ct=math.cos(th); st=math.sin(th); td=tr+tf
    xd=r0[0]+v*ct*td; yd=r0[1]+v*st*td; zd=r0[2]-0.5*G*tf*tf
    tsi=max(0,int(td/DTS)); tei=min(NTS,int((td+CD)/DTS)+1)
    if tei<=tsi: return 0.0
    ta_s=TA[tsi:tei]; active=ta_s>=td
    if not np.any(active): return 0.0
    ta_a=ta_s[active]; cz=zd-CS*(ta_a-td)
    ttx=LP[tk,tsi:tei,0][active]; tty=LP[tk,tsi:tei,1][active]
    ttz=LP[tk,tsi:tei,2][active]
    mpx=LP[tk,tsi:tei,3][active]; mpy=LP[tk,tsi:tei,4][active]
    mpz=LP[tk,tsi:tei,5][active]
    wcx=xd-mpx; wcy=yd-mpy; wcz=cz-mpz
    c2=ttx*ttx+tty*tty+ttz*ttz+1e-10; c1=wcx*ttx+wcy*tty+wcz*ttz
    s=np.clip(c1/c2,0.0,1.0)
    px=mpx+s*ttx; py=mpy+s*tty; pz=mpz+s*ttz
    return float(np.sum((xd-px)**2+(yd-py)**2+(cz-pz)**2<CR*CR))*DTS

def sphere_occluded(mpos,cpos):
    """Unit sphere projection: check if smoke sphere at cpos covers all target PTS from mpos"""
    mx,my,mz=mpos; cx,cy,cz=cpos
    wcx,wcy,wcz=cx-mx,cy-my,cz-mz
    dn=math.sqrt(wcx*wcx+wcy*wcy+wcz*wcz)
    if dn<1e-10 or dn<=CR: return True
    dc=np.array([wcx/dn,wcy/dn,wcz/dn])
    cth=math.sqrt(max(0.0,1.0-(CR/dn)**2))
    vx,vy,vz=PTS[:,0]-mx,PTS[:,1]-my,PTS[:,2]-mz
    vn=np.sqrt(vx*vx+vy*vy+vz*vz)+1e-10
    return float(np.min((vx*dc[0]+vy*dc[1]+vz*dc[2])/vn))>=cth

def sphere_shield_dur(di,th,v,tk,tr,tf):
    """Full sphere projection shielding check for single grenade (accurate but slower)"""
    r0=DI[di]; ct=math.cos(th); st=math.sin(th); td=tr+tf
    xd=r0[0]+v*ct*td; yd=r0[1]+v*st*td; zd=r0[2]-0.5*G*tf*tf
    tsi=max(0,int(td/DTS)); tei=min(NTS,int((td+CD)/DTS)+1)
    if tei<=tsi: return 0.0
    ta_s=TA[tsi:tei]; active=ta_s>=td
    if not np.any(active): return 0.0
    ta_a=ta_s[active]; total=0.0; act_idx=np.where(active)[0]
    for idx_a in range(len(ta_a)):
        ti=tsi+act_idx[idx_a]; t=ta_a[idx_a]
        if sphere_occluded(MP[tk,ti],np.array([xd,yd,zd-CS*(t-td)])): total+=DTS
    return total

def multi_shield_center(di,th,v,tk,grenades):
    """Center-line union shielding for multiple grenades"""
    shielded=np.zeros(NTS,dtype=bool)
    for g in grenades:
        tr,tf=g[0],g[1]; td=tr+tf
        r0=DI[di]; ct=math.cos(th); st=math.sin(th)
        xd=r0[0]+v*ct*td; yd=r0[1]+v*st*td; zd=r0[2]-0.5*G*tf*tf
        tsi=max(0,int(td/DTS)); tei=min(NTS,int((td+CD)/DTS)+1)
        if tei<=tsi: continue
        ta_s=TA[tsi:tei]; active=(ta_s>=td)
        if not np.any(active): continue
        ta_a=ta_s[active]; cz=zd-CS*(ta_a-td)
        ttx=LP[tk,tsi:tei,0][active]; tty=LP[tk,tsi:tei,1][active]
        ttz=LP[tk,tsi:tei,2][active]
        mpx=LP[tk,tsi:tei,3][active]; mpy=LP[tk,tsi:tei,4][active]
        mpz=LP[tk,tsi:tei,5][active]
        wcx=xd-mpx; wcy=yd-mpy; wcz=cz-mpz
        c2=ttx*ttx+tty*tty+ttz*ttz+1e-10; c1=wcx*ttx+wcy*tty+wcz*ttz
        s=np.clip(c1/c2,0.0,1.0)
        px=mpx+s*ttx; py=mpy+s*tty; pz=mpz+s*ttz
        idxs=np.where(active)[0]
        for j,orig_i in enumerate(idxs): shielded[tsi+orig_i]|=(xd-px[j])**2+(yd-py[j])**2+(cz[j]-pz[j])**2<CR*CR
    return float(np.sum(shielded))*DTS

def multi_sphere_shield(di,th,v,tk,grenades):
    """Full sphere projection union shielding for multiple grenades"""
    shielded=np.zeros(NTS,dtype=bool)
    for g in grenades:
        tr,tf=g[0],g[1]; td=tr+tf
        r0=DI[di]; ct=math.cos(th); st=math.sin(th)
        xd=r0[0]+v*ct*td; yd=r0[1]+v*st*td; zd=r0[2]-0.5*G*tf*tf
        tsi=max(0,int(td/DTS)); tei=min(NTS,int((td+CD)/DTS)+1)
        if tei<=tsi: continue
        ta_s=TA[tsi:tei]; active=(ta_s>=td)
        if not np.any(active): continue
        ta_a=ta_s[active]; act_idx=np.where(active)[0]
        for idx_a in range(len(ta_a)):
            ti=tsi+act_idx[idx_a]; t=ta_a[idx_a]
            if shielded[ti]: continue
            if sphere_occluded(MP[tk,ti],np.array([xd,yd,zd-CS*(t-td)])): shielded[ti]=True
    return float(np.sum(shielded))*DTS

# print(f"fast_eval: ...")  # suppressed
