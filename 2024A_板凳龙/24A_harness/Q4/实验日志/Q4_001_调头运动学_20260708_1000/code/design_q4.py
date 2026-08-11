"""Q4 设计问题："能否调整圆弧，仍保持各部分相切，使得调头曲线变短？"

严格对应 Q4_模型假设清单.md §四 的既定自由度：固定切点 P_in,P_out 于调头空间边界
r=4.5（与全问运动学模型一致），唯一可调的是两弧半径比 ratio=R1/R2（题给=2）。
在此严格约束下求 L_S(ratio) 是否存在比题给更短的取值 —— 这是本题的**主答案**。

（extended）另附一段扩展讨论：若进一步放松，允许切点本身沿螺线内移（r_entry<4.5，
不再要求恰好在调头空间边界），则 L_S 可继续缩短，但这已超出"调整圆弧比例"的字面
范围、且在当前模型下退化为无下界的开放问题（无额外约束时可无限内移），因此不作为
正式答案，只作为方法论延伸列出。
"""
import os
import numpy as np
import geometry_q4 as g


def LS_of_ratio(ratio):
    sc = g.build_scurve(ratio=ratio)
    return sc["LS"], sc


def max_radius_of_scurve(sc, npts=2000):
    """S曲线上离中心最远距离（判断是否超出 r<=4.5 调头空间）。"""
    rmax = 0.0
    for (O, R, ang0, dirn, L) in [
        (sc["O1"], sc["R1"], sc["ang1_start"], sc["dir1"], sc["L1"]),
        (sc["O2"], sc["R2"], sc["ang2_start"], sc["dir2"], sc["L2"]),
    ]:
        for u in np.linspace(0, L, npts):
            ang = ang0 + dirn * (u / R)
            pt = O + R * np.array([np.cos(ang), np.sin(ang)])
            rmax = max(rmax, np.hypot(*pt))
    return rmax


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(os.path.dirname(here), "outputs")

    # ================= 主答案：固定切点(r_entry=4.5)，仅调整半径比 =================
    LS2, sc2 = LS_of_ratio(2.0)
    rmax2 = max_radius_of_scurve(sc2)
    print(f"[given] ratio=2 (题给): LS={LS2:.6f} m, R1={sc2['R1']:.6f}, R2={sc2['R2']:.6f}, "
          f"S曲线最大半径={rmax2:.6f} m (<=4.5? {rmax2<=4.5+1e-6})")

    ratios = np.linspace(0.2, 8.0, 200)
    Ls = np.array([LS_of_ratio(r)[0] for r in ratios])
    print(f"[scan] ratio∈[0.2,8]（200点）: LS 范围 = [{Ls.min():.9f}, {Ls.max():.9f}] m, "
          f"极差={Ls.max()-Ls.min():.2e} m（相对量级1e-13，属浮点噪声）")
    print("[analytic] 由外切条件 R1+R2=|P_in|^2/c=D（仅由切点位置决定，与比例无关），"
          "可证扫掠角 alpha1=alpha2 恒定，故 LS=(R1+R2)*alpha=D*alpha 与 ratio 严格无关（几何恒等式，非数值巧合）。")
    print(f"[conclusion-primary] 固定切点于边界 r=4.5（与本问运动学模型一致）时，"
          f"调头曲线长 LS≡{LS2:.6f} m 是**与半径比无关的不变量** -> "
          f"仅调整两弧半径比例：**不能**使调头曲线变短（也不会变长）。")

    # ================= 扩展讨论：切点内移（超出题面"调整圆弧比例"字面范围） =================
    print("\n[extended] 扩展讨论（非正式答案）：若放松为允许切点半径 r_entry 本身可调（"
          "即调头可以在还未到达调头空间边界前就开始），保持 ratio=2 不变：")
    rho_list = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]
    entry_rows = []
    for rho in rho_list:
        sc = g.build_scurve(ratio=2.0, r_entry=rho)
        entry_rows.append((rho, sc["LS"], sc["R1"], sc["R2"]))
        print(f"    r_entry={rho:.2f} m: LS={sc['LS']:.6f} m  (R1={sc['R1']:.4f},R2={sc['R2']:.4f})")
    print("[extended-caveat] LS 随 r_entry 减小而单调下降，且当前模型未引入额外约束"
          "（如与Q2同类的自碰撞检测）来限定 r_entry 的下界，故该方向在数学上退化为"
          "'无下界的开放问题'（r_entry->0 时 LS->0），不构成一个有唯一解的最优化问题，"
          "因此不作为本题正式答案，仅作为可能的后续深化方向列出。")

    np.savez(os.path.join(outdir, "q4_design_entry.npz"),
             rho=np.array([r[0] for r in entry_rows]),
             LS=np.array([r[1] for r in entry_rows]))
    np.savez(os.path.join(outdir, "q4_design.npz"),
             ratios=ratios, Ls=Ls, LS2=LS2, D=sc2["D"])

    import csv
    with open(os.path.join(outdir, "q4_design_summary.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["项目", "值"])
        w.writerow(["题给构型 ratio=R1/R2", "2.000000"])
        w.writerow(["题给 R1 (m)", f"{sc2['R1']:.6f}"])
        w.writerow(["题给 R2 (m)", f"{sc2['R2']:.6f}"])
        w.writerow(["调头曲线长 LS (m)（与ratio无关的不变量）", f"{LS2:.6f}"])
        w.writerow(["ratio扫描[0.2,8]下LS极差 (m)", f"{Ls.max()-Ls.min():.2e}（浮点噪声，非真实变化）"])
        w.writerow(["主答案：仅调整半径比能否变短", "不能（LS为不变量）"])
        w.writerow(["扩展讨论：允许切点内移(r_entry)能否变短", "能，但退化为无下界开放问题，不作为正式答案"])
    print("[write] q4_design_summary.csv")


if __name__ == "__main__":
    main()
