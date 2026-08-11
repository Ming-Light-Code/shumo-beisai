#!/usr/bin/env python3
import numpy as np, time
from itertools import product
ALL_XY={'B':(1,15),'E':(30,15),'W1':(6,21),'W2':(15,9),'W3':(24,24),'S1':(12,16),'S2':(21,16)}
IDX2NAME=['B','E','W1','W2','W3','S1','S2']
XY=[(1,15),(30,15),(6,21),(15,9),(24,24),(12,16),(21,16)]
IO,IH,IF=100,150,100;IM,IZ=750,200;LL,MD=400,90;PN,PS=0.8,0.2
CN=np.array([2,3,2]);SN=np.array([1,1,1]);WN=np.array([5,4,3])
CT=np.array([8,4,3]);ST=np.array([3,3,2]);WT=np.array([8,6,6])
CE=PN*CN+PS*CT;SE=PN*SN+PS*ST;WE=PN*WN+PS*WT
WY=[20,15,28];WM=[4,5,3]
def md(a,b):return abs(a[0]-b[0])+abs(a[1]-b[1])
D=np.zeros((7,7),int)
for i in range(7):
    for j in range(7):D[i,j]=md(XY[i],XY[j])
print('Q3 Summary Runner - finding optimal plan...')
INT=[2,3,4,5,6]
mb=min(D[0,i] for i in INT);me=min(D[i,1] for i in INT)
mii=min(D[i,j] for i in INT for j in INT if i!=j)
ms=min(4,(MD-mb-me)//mii+1)
bz=-1;bm=-1;bp=None;cnt=0;fs=0;t0=time.time()
for sl in range(ms+1):
    for seq in product(range(5),repeat=sl):
        cnt+=1;pid=[0]+[INT[s]+1 for s in seq]+[1];ok=True
        for k in range(2,len(pid)):
            if pid[k]==pid[k-1]:ok=False;break
        if not ok:continue
        m=len(pid)-2;tt=sum(D[pid[k],pid[k+1]] for k in range(m+1))
        if tt>MD:continue
        rem=MD-tt;ws=[];ss=[]
        for k in range(m+1):
            pt=pid[k+1]
            if 3<=pt<=5:ws.append((k,pt-3))
            elif pt in[6,7]:ss.append(k)
        nw=len(ws)
        if nw==0:continue
        zc=sum(2*WM[w]*WY[w] for _,w in ws)
        zu=IZ+min(rem*max(WY),zc)
        if zu<=bz:continue
        w1=[0]*nw;b_=[0]*nw;w2=[0]*nw;r=rem
        od=sorted(range(nw),key=lambda j:WY[ws[j][1]],reverse=True)
        for o in od:wi=ws[o][1];w1[o]=min(WM[wi],r);r-=w1[o]
        for o in od:
            wi=ws[o][1]
            if r>0 and w1[o]==WM[wi]:b_[o]=1;r-=1;w2[o]=min(WM[wi],r);r-=w2[o]
        ns=len(ss);T=tt+sum(w1)+sum(b_)+sum(w2)
        cO=np.zeros(T);cH=np.zeros(T);cF=np.zeros(T);isp=np.zeros(T,bool);s2k=np.zeros(T,int);dy=0;sx=0
        for k in range(m+1):
            dt=D[pid[k],pid[k+1]]
            for dd in range(dt):
                cO[dy]=CE[0];cH[dy]=CE[1];cF[dy]=CE[2]
                if dd==dt-1 and k in ss:isp[dy]=True;s2k[dy]=sx;sx+=1
                dy+=1
            wk=[j for j,(sk,_) in enumerate(ws) if sk==k]
            if wk:
                wi=wk[0];wh=ws[wi][1]
                for _ in range(w1[wi]):cO[dy]=WE[0];cH[dy]=WE[1];cF[dy]=WE[2];dy+=1
                if b_[wi]:cO[dy]=SE[0];cH[dy]=SE[1];cF[dy]=SE[2];dy+=1
                for _ in range(w2[wi]):cO[dy]=WE[0];cH[dy]=WE[1];cF[dy]=WE[2];dy+=1
        O=IO;H=IH;F=IF;M_=IM;buy=np.zeros((ns,3),int)
        for t in range(T):
            O-=cO[t];H-=cH[t];F-=cF[t];O=max(0,O);H=max(0,H);F=max(0,F)
            if isp[t]:
                no=nh=nf=0
                for tt_ in range(t+1,min(T,t+40)):no+=cO[tt_];nh+=cH[tt_];nf+=cF[tt_]
                bo=max(0,int(np.ceil(no-O)));bh=max(0,int(np.ceil(nh-H)));bf=max(0,int(np.ceil(nf-F)))
                sk=s2k[t];cost=bo*2+bh+bf*2
                if cost>M_ or O+bo+H+bh+F+bf>LL:break
                O+=bo;H+=bh;F+=bf;M_-=cost
                if sk<ns:buy[sk]=[bo,bh,bf]
            if O<0 or H<0 or F<0:break
        if O<0 or H<0 or F<0 or M_<0:continue
        fs+=1;Zf=IZ
        for wi in range(nw):Zf+=(w1[wi]+w2[wi])*WY[ws[wi][1]]
        if Zf>bz or (Zf==bz and M_>bm):bz=Zf;bm=M_;bp={'pid':pid,'w1':w1,'b':b_,'w2':w2,'buy':buy,'ws':ws,'ss':ss,'tt':tt,'T':T}
el=time.time()-t0
print(f'Searched {cnt} skels, {fs} feasible, {el:.1f}s')
if bp:
    rt=' -> '.join(IDX2NAME[p] for p in bp['pid'])
    print(f'Best: {rt}')
    print(f'Travel={bp["tt"]}d, Z_exp={bz}, M_exp={bm}')
    for i,(si,w) in enumerate(bp['ws']):
        print(f'  W{w+1}: w1={bp["w1"][i]}, stop={bp["b"][i]}, w2={bp["w2"][i]}')
    for i,sk in enumerate(bp['ss']):
        print(f'  S{i+1}: buy=({bp["buy"][i,0]},{bp["buy"][i,1]},{bp["buy"][i,2]})')
    # Monte Carlo simulation
    print(f'\n--- Monte Carlo (500 runs) ---')
    pid=bp['pid'];w1=bp['w1'];b=bp['b'];w2=bp['w2'];buy=bp['buy'];ws=bp['ws'];ss=bp['ss']
    m=len(pid)-2;Zv=[];Mv=[];Dv=[];succ=0
    for sim in range(500):
        wseq=np.random.random(MD)>PN;O=IO;H=IH;F=IF;M=IM;Z=IZ;x,y=XY[0];dy=0;dd=False;ar=False
        for k in range(m+1):
            if dd:break
            dt=D[pid[k],pid[k+1]];tx,ty=XY[pid[k+1]]
            for _ in range(dt):
                if dy>=MD:dd=True;break
                st=wseq[dy];cm=CT if st else CN;O-=cm[0];H-=cm[1];F-=cm[2];dy+=1
                if O<0 or H<0 or F<0:dd=True;break
                if abs(tx-x)>=abs(ty-y):x+=np.sign(tx-x) if x!=tx else 0
                else:y+=np.sign(ty-y) if y!=ty else 0
            if dd:break
            if k in ss:
                si=ss.index(k);bo=bh=bf=0
                if si<buy.shape[0]:bo,bh,bf=int(buy[si,0]),int(buy[si,1]),int(buy[si,2])
                cst=bo*2+bh+bf*2
                if M>=cst and O+bo+H+bh+F+bf<=LL:O+=bo;H+=bh;F+=bf;M-=cst
            wk=[j for j,(sk,_) in enumerate(ws) if sk==k]
            if wk:
                wi=wk[0];wh=ws[wi][1]
                for _ in range(w1[wi]):
                    if dy>=MD:dd=True;break
                    st=wseq[dy];cw=WT if st else WN;O-=cw[0];H-=cw[1];F-=cw[2];Z+=WY[wh];dy+=1
                    if O<0 or H<0 or F<0:dd=True;break
                if dd:break
                if b[wi]:
                    if dy>=MD:dd=True;break
                    st=wseq[dy];cs=ST if st else SN;O-=cs[0];H-=cs[1];F-=cs[2];dy+=1
                if dd:break
                for _ in range(w2[wi]):
                    if dy>=MD:dd=True;break
                    st=wseq[dy];cw=WT if st else WN;O-=cw[0];H-=cw[1];F-=cw[2];Z+=WY[wh];dy+=1
                    if O<0 or H<0 or F<0:dd=True;break
        if not dd:
            d2e=md((x,y),XY[1]);rem=MD-dy
            if d2e<=rem:
                for _ in range(d2e):
                    if dy>=MD:break
                    st=wseq[dy] if dy<len(wseq) else False;cm=CT if st else CN
                    O-=cm[0];H-=cm[1];F-=cm[2];dy+=1
                    if O<0 or H<0 or F<0:break
                if O>=0 and H>=0 and F>=0:ar=True
        if ar:succ+=1;Zv.append(Z);Mv.append(M);Dv.append(dy)
    if Zv:
        print(f'Success rate: {succ/500:.1%} ({succ}/500)')
        print(f'Z: mean={np.mean(Zv):.0f} std={np.std(Zv):.0f} max={np.max(Zv):.0f}')
        print(f'M: mean={np.mean(Mv):.0f} std={np.std(Mv):.0f}')
        print(f'Days: mean={np.mean(Dv):.1f}')
    else:
        print('ZERO success in MC simulation!')
else:
    print('NO FEASIBLE PLAN FOUND')
