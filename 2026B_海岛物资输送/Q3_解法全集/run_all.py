#!/usr/bin/env python3
import numpy as np,time
from itertools import product

XY=[(1,15),(30,15),(6,21),(15,9),(24,24),(12,16),(21,16)]
NAMES=['B','E','W1','W2','W3','S1','S2']
IO,IH,IF=100,150,100;IM,IZ=750,200;LL,MD=400,90;PN,PS=0.8,0.2
CN=np.array([2,3,2]);SN=np.array([1,1,1]);WN=np.array([5,4,3])
CT=np.array([8,4,3]);ST=np.array([3,3,2]);WT=np.array([8,6,6])
CE=PN*CN+PS*CT;SE=PN*SN+PS*ST;WE=PN*WN+PS*WT
WY=[20,15,28];WM=[4,5,3]

def md(a,b):return abs(a[0]-b[0])+abs(a[1]-b[1])
D=np.zeros((7,7),int)
for i in range(7):
    for j in range(7):D[i,j]=md(XY[i],XY[j])

print('='*60)
print('  Q3 Solution Results')
print('='*60)
print(f'Init: O={IO} H={IH} F={IF} M={IM} Z={IZ}')
print(f'Load limit={LL}, Days={MD}')
print(f'P(normal)={PN}, P(storm)={PS}')
print()
print('Key distances:')
print(f'  B->E = {D[0,1]}')
print(f'  B->S1 = {D[0,5]}, B->S2 = {D[0,6]}')
print(f'  B->W1 = {D[0,2]}, B->W2 = {D[0,3]}, B->W3 = {D[0,4]}')
print(f'  W3->E = {D[4,1]}')

print()
print('--- Feasibility analysis ---')
print(f'E[O] for move: {CE[0]:.1f}, stop: {SE[0]:.1f}, work: {WE[0]:.1f}')
print(f'E[H] for move: {CE[1]:.1f}, stop: {SE[1]:.1f}, work: {WE[1]:.1f}')
print(f'E[F] for move: {CE[2]:.1f}, stop: {SE[2]:.1f}, work: {WE[2]:.1f}')

routes_to_test = [
    [0,5,2,3,6,4,1],
    [0,2,3,4,1],
    [0,2,3,6,4,1],
    [0,5,2,3,4,1],
    [0,2,5,3,6,4,1],
]

for pid in routes_to_test:
    rt='->'.join(NAMES[p] for p in pid)
    m=len(pid)-2
    tt=sum(D[pid[k],pid[k+1]] for k in range(m+1))
    rem=MD-tt
    needO=CE[0]*tt;needH=CE[1]*tt;needF=CE[2]*tt
    print(f'{rt}: travel={tt}d, remain={rem}d, need_O={needO:.0f}, need_H={needH:.0f}, need_F={needF:.0f}')
    if needO>IO:print(f'  WARNING: O shortfall={needO-IO:.0f}')
    if needH>IH:print(f'  WARNING: H shortfall={needH-IH:.0f}')
    if needF>IF:print(f'  WARNING: F shortfall={needF-IF:.0f}')
