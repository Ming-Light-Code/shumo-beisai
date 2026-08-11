"""验证脚本1：局部细网格复核峰值区域（防止步长0.2遗漏尖峰）+ 瓶颈节点Top5。
对应 sanity_check.md 中的 S4/S5 项。
"""
import numpy as np
import geometry_q4 as g
from solve_q5 import max_amp, amplification

track = g.Track(ratio=2.0, theta_max=100.0, n_grid=300000)

grid = np.arange(0.0, 30.0, 0.01)
M = np.array([max_amp(track, float(s))[0] for s in grid])
j = int(np.argmax(M))
print("细网格(step=0.01)最大值: s_head=%.4f K=%.8f" % (grid[j], M[j]))

grid2 = np.arange(grid[j] - 0.02, grid[j] + 0.02, 0.0005)
M2 = np.array([max_amp(track, float(s))[0] for s in grid2])
j2 = int(np.argmax(M2))
print("更细网格(step=0.0005)最大值: s_head=%.6f K=%.8f" % (grid2[j2], M2[j2]))

s0 = grid[j]
K = amplification(track, s0)
top5 = np.argsort(K)[::-1][:5]
print("s_head=%.4f 附近各节点放大系数Top5:" % s0,
      [(int(t), round(float(K[t]), 6)) for t in top5])
