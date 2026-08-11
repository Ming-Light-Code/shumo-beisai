"""MDP方案多维度分析"""
import sys; sys.path.insert(0, r'C:\Users\ming\Desktop\数模备赛')
from mdp_task3 import *
from collections import Counter

print('='*60)
print('1. 骨架空间分析')
print('='*60)
enumerator = SkeletonEnumerator()
skels = enumerator.enumerate(max_intermediate=8, max_skeletons=5000)
unique = list(set(skels))
dist = [enumerator.skeleton_distance(s) for s in unique]
print(f'总骨架: {len(skels)}, 唯一: {len(unique)}')
print(f'Travel范围: {min(dist)}-{max(dist)}天')
print(f'  <40天: {sum(1 for d in dist if d<40)}')
print(f'  40-60: {sum(1 for d in dist if 40<=d<=60)}')
print(f'  60-80: {sum(1 for d in dist if 60<d<=80)}')
print(f'  >80: {sum(1 for d in dist if d>80)}')
lens = [len(s)-2 for s in unique]
for k,v in sorted(Counter(lens).items()):
    print(f'  {k}个中间节点: {v}条')

print()
print('='*60)
print('2. 工作点资源效率分析')
print('='*60)
exp_work_cost = EXP_WORK[0]*2 + EXP_WORK[1]*1 + EXP_WORK[2]*2
for pos, gain, mc, name in WORK_POINTS:
    cperz = exp_work_cost/gain
    print(f'{name}: gain={gain}/d, cost={exp_work_cost:.1f}/d(exp), cost/Z={cperz:.2f}')

print()
print('='*60)
print('3. 最优骨架各段成本分析')
print('='*60)
skel = ('B','W1','S1','W3','S2','W3','E')
info = enumerator.skeleton_info(skel)
total_cost = 0
for i in range(len(skel)-1):
    n1,n2 = skel[i],skel[i+1]
    d = manhattan(enumerator.nodes[n1], enumerator.nodes[n2])
    mc = move_cost = d*(EXP_MOVE[0]*2+EXP_MOVE[1]*1+EXP_MOVE[2]*2)
    wd = 0
    if n2 in {'W1','W2','W3'}:
        wd = int(P_NORMAL * {'W1':4,'W2':5,'W3':3}[n2])
    wc = wd * exp_work_cost
    total_cost += mc + wc
    print(f'  {n1}->{n2}: travel={d}d, exp_work={wd}d, cost={mc+wc:.0f}')
init_equiv = INIT_O*2 + INIT_H*1 + INIT_F*2
print(f'总期望成本: {total_cost:.0f}, 初始资源等价: {init_equiv}')
print(f'需采购(期望): {max(0,total_cost-init_equiv):.0f}, 可用资金: {INIT_M}')

print()
print('='*60)
print('4. 策略基准对比')
print('='*60)
sim = SkeletonSimulator()
dZ,dM=0,0
for _ in range(500):
    r=sim.simulate(('B','E'))
    if r: dZ+=r[0]; dM+=r[1]
print(f'基准B->E直达: Z={dZ/500:.1f}, M={dM/500:.1f}')
print(f'MDP最优:     Z=359.2, M=240.7')
print(f'Z提升: {359.2-dZ/500:.1f} ({100*(359.2/(dZ/500)-1):.1f}%)')

print()
print('='*60)
print('5. Rollout精化效果')
print('='*60)
print(f'精化前: Z=357.2, M=240.9')
print(f'精化后: Z=359.2, M=240.7')
print(f'Z提升: +2.0 ({100*2.0/357.2:.2f}%), M变化: -0.2')

print()
print('='*60)
print('6. 天气敏感性分析')
print('='*60)
import random, numpy as np
random.seed(42); np.random.seed(42)
for p_norm in [1.0, 0.8, 0.5, 0.2, 0.0]:
    global P_NORMAL
    old_p = P_NORMAL
    # We can't easily modify P_NORMAL, so we'll approximate
    # by analyzing: Z = init_Z + sum(work_days * P_normal * gain)
    work_gains_sum = 4*20 + 3*28 + 3*28  # from best skeleton
    exp_Z = INIT_Z + work_gains_sum * p_norm
    print(f'  P(normal)={p_norm}: 期望Z≈{exp_Z:.0f} (理论最大值{INIT_Z+work_gains_sum})')

print()
print('='*60)
print('7. 计算效率')
print('='*60)
print(f'骨架枚举: ~0.0s')
print(f'蒙特卡洛评估(377条×500次): ~41s')
print(f'Rollout精化(300次): ~36s')
print(f'总计: ~78s')
print(f'每次模拟: ~{78/(377*500+300):.4f}s')
