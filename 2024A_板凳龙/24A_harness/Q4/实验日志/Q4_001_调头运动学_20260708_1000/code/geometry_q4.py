"""Q4 几何内核：螺线、S形调头曲线构造、复合轨道 track(s)（弧长参数化）。

依据 Q4/实现方案/Q4_v1_调头运动学方案.md。
坐标约定：s=0 于 P_in（调头开始），+s 为龙头行进方向（盘入->调头->盘出）。
"""
import numpy as np
from scipy.optimize import brentq
from scipy.interpolate import PchipInterpolator

# ----------------------------- 常量 -----------------------------
P_PITCH = 1.7
B = P_PITCH / (2.0 * np.pi)          # r = B*theta
R_BOUND = 4.5                        # 调头空间半径
THETA_C = R_BOUND / B                # P_in 处极角


def _rot(v, ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def spiral_point(theta):
    r = B * theta
    return np.array([r * np.cos(theta), r * np.sin(theta)])


def spiral_arclen(theta):
    s = np.sqrt(theta * theta + 1.0)
    return 0.5 * B * (theta * s + np.arcsinh(theta))


# ----------------------------- S 曲线构造 -----------------------------
def build_scurve(ratio=2.0, r_entry=R_BOUND):
    """构造 S 形调头曲线。ratio = R1/R2（题给=2），r_entry=切点半径（默认边界4.5）。
    返回 dict：R1,R2,O1,O2,J,P_in,P_out,n1,t_in,alpha1,alpha2,L1,L2,LS,dir1,dir2,
              ang1_start, ang2_start, D(=R1+R2)。
    """
    theta_c = r_entry / B
    Pin = spiral_point(theta_c)
    Pout = -Pin
    # 螺线切向（增 theta 方向，向外）
    tvec = np.array([np.cos(theta_c) - theta_c * np.sin(theta_c),
                     np.sin(theta_c) + theta_c * np.cos(theta_c)])
    t_in = tvec / np.linalg.norm(tvec)          # 向外单位切向
    # 外法线：取与 Pin 同向（Pin·n1>0）
    n1 = _rot(t_in, np.pi / 2)
    if np.dot(Pin, n1) < 0:
        n1 = -n1
    c = float(np.dot(Pin, n1))                    # >0
    D = float(np.dot(Pin, Pin)) / c               # R1+R2 (常数)
    R2 = D / (ratio + 1.0)
    R1 = ratio * R2
    # S 形：sigma1=sigma2=-1
    O1 = Pin - R1 * n1
    O2 = -Pin + R2 * n1
    d12 = np.linalg.norm(O1 - O2)
    J = O1 + R1 * (O2 - O1) / d12                 # junction（外切点）

    # 弧1：绕 O1，起点 Pin，终点 J；遍历方向由 Pin 处切向=龙头行进方向(-t_in) 定
    def arc_dir(O, Pstart, travel_dir):
        rad = Pstart - O
        tang_ccw = _rot(rad, np.pi / 2) / np.linalg.norm(rad)   # +1(逆时针)时的切向
        return 1.0 if np.dot(tang_ccw, travel_dir) > 0 else -1.0

    travel_in = -t_in                              # 龙头在 Pin 的行进方向（向内盘）
    dir1 = arc_dir(O1, Pin, travel_in)
    ang1_start = np.arctan2((Pin - O1)[1], (Pin - O1)[0])
    ang1_end = np.arctan2((J - O1)[1], (J - O1)[0])
    alpha1 = _sweep(ang1_start, ang1_end, dir1)
    L1 = R1 * alpha1

    # 弧2：绕 O2，起点 J，终点 Pout；方向由 J 处切向连续（=弧1末端切向）定
    rad1_end = J - O1
    tang_J = _rot(rad1_end, dir1 * np.pi / 2) / np.linalg.norm(rad1_end)  # 弧1末端行进切向
    dir2 = arc_dir(O2, J, tang_J)
    ang2_start = np.arctan2((J - O2)[1], (J - O2)[0])
    ang2_end = np.arctan2((Pout - O2)[1], (Pout - O2)[0])
    alpha2 = _sweep(ang2_start, ang2_end, dir2)
    L2 = R2 * alpha2

    return dict(R1=R1, R2=R2, O1=O1, O2=O2, J=J, P_in=Pin, P_out=Pout,
                n1=n1, t_in=t_in, c=c, D=D,
                alpha1=alpha1, alpha2=alpha2, L1=L1, L2=L2, LS=L1 + L2,
                dir1=dir1, dir2=dir2, ang1_start=ang1_start, ang2_start=ang2_start)


def _sweep(a0, a1, direction):
    """从角 a0 沿 direction(+1 逆/-1 顺) 到 a1 的正张角 (0,2pi)。"""
    d = (a1 - a0) * direction
    d = d % (2 * np.pi)
    if d < 1e-12:
        d += 2 * np.pi
    return d


# ----------------------------- 复合轨道 track(s) -----------------------------
class Track:
    def __init__(self, ratio=2.0, theta_max=70.0, n_grid=200000):
        self.sc = build_scurve(ratio)
        self.LS = self.sc["LS"]
        # 预计算 螺线弧长(相对 P_in 向外) -> theta 的单调样条
        thetas = np.linspace(THETA_C, theta_max, n_grid)
        a = spiral_arclen(thetas) - spiral_arclen(THETA_C)   # >=0
        self._theta_of_a = PchipInterpolator(a, thetas)
        self._a_max = a[-1]

    def theta_of_arclen(self, a):
        if np.any(a > self._a_max):
            raise ValueError("超出螺线预计算范围，增大 theta_max")
        return self._theta_of_a(a)

    def point(self, s):
        return self._eval(s, want_tangent=False)

    def point_tangent(self, s):
        return self._eval(s, want_tangent=True)

    def _eval(self, s, want_tangent):
        sc = self.sc
        if s <= 0.0:
            a = -s
            th = float(self.theta_of_arclen(a))
            pt = spiral_point(th)
            if not want_tangent:
                return pt
            # 盘入切向（+s=向内=减 theta 方向）
            dP = np.array([np.cos(th) - th * np.sin(th),
                           np.sin(th) + th * np.cos(th)])
            T = -dP / np.linalg.norm(dP)
            return pt, T
        elif s >= self.LS:
            a = s - self.LS
            ph = float(self.theta_of_arclen(a))
            pt = -spiral_point(ph)
            if not want_tangent:
                return pt
            dP = np.array([np.cos(ph) - ph * np.sin(ph),
                           np.sin(ph) + ph * np.cos(ph)])
            # 盘出：point=-spiral(phi)，a=s-LS 随 s 增而增，phi 随 a 增而增，
            # 故 d(point)/ds = d(-spiral)/dphi * dphi/ds = -dP（dphi/ds>0，符号沿用负号）
            T = (-dP) / np.linalg.norm(dP)
            return pt, T
        else:
            # 调头段
            if s <= sc["L1"]:
                O, R, ang0, dirn = sc["O1"], sc["R1"], sc["ang1_start"], sc["dir1"]
                ds = s
            else:
                O, R, ang0, dirn = sc["O2"], sc["R2"], sc["ang2_start"], sc["dir2"]
                ds = s - sc["L1"]
            ang = ang0 + dirn * (ds / R)
            rad = np.array([np.cos(ang), np.sin(ang)])
            pt = O + R * rad
            if not want_tangent:
                return pt
            T = _rot(rad, dirn * np.pi / 2)      # 行进方向切向（单位）
            return pt, T
