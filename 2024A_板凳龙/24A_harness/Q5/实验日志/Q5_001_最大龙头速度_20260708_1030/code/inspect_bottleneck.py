"""验证脚本2：拆解临界构型 s_head*=14.48 处逐节点放大系数 K_i 与增量 k_i，
定位放大发生在哪些链段(揭示：跨越圆弧曲率突变处的链段放大最显著，
同弧内链段因圆周自相似性 k_incr 恒为1)。对应总结md"创新点"的物理解释依据。
"""
import numpy as np
import geometry_q4 as g
from solve_q5 import amplification, chain_positions

track = g.Track(ratio=2.0, theta_max=100.0, n_grid=300000)
s_head = 14.48
K = amplification(track, s_head)
s, P, T = chain_positions(track, s_head)
print("i, s_i, K_i, k_i(增量)")
prev = 1.0
for i in range(0, 15):
    k_incr = K[i] / prev if i > 0 else 1.0
    print(f"{i:3d}  s={s[i]:9.4f}  K={K[i]:.6f}  k_incr={k_incr:.6f}")
    prev = K[i]
print("L1(弧1端点) =", track.sc["L1"], " LS(调头曲线全长) =", track.LS)
