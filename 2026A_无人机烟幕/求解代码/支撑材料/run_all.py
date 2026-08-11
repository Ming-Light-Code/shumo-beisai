# -*- coding: utf-8 -*-
"""run_all.py —— 依次运行问题一至问题五

整个文件夹迁移后可直接运行: python run_all.py
各问题文件也可独立运行: python problem1.py
"""
import sys, os, subprocess, time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)

print("=" * 70)
print("  烟幕干扰弹投放策略 —— 全题求解")
print("=" * 70)

problems = [
    ("问题一: 单机单弹 (给定参数)",    "problem1.py"),
    ("问题二: 单机单弹 (最优 NLP)",    "problem2.py"),
    ("问题三 DE: 单机三弹 (定向枚举)",  "problem3_de.py"),
    ("问题三 NLP: 单机三弹 (波束搜索)", "problem3_nlp.py"),
    ("问题四: 三机协同",               "problem4.py"),
    ("问题五: 多机多弹多目标",         "problem5.py"),
]

t_start_all = time.time()
for title, fname in problems:
    print("\n" + "=" * 50)
    print(f"  {title}")
    print("=" * 50)
    filepath = os.path.join(SCRIPT_DIR, fname)
    t0 = time.time()
    result = subprocess.run(
        [sys.executable, filepath],
        cwd=SCRIPT_DIR,
        capture_output=False,
    )
    elapsed = time.time() - t0
    if result.returncode != 0:
        print(f"\n  !! {fname} 运行失败 (exit code: {result.returncode})")
    else:
        print(f"\n  {fname} 完成 (耗时 {elapsed:.1f} s)")

total_elapsed = time.time() - t_start_all
print("\n" + "=" * 70)
print(f"  全部求解完成 (总耗时 {total_elapsed:.0f} s / {total_elapsed / 60:.1f} min)")
print("=" * 70)
