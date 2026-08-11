# -*- coding: utf-8 -*-
"""run_all.py -- 依次运行全部求解器"""
import sys, os, time

_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(_DIR)
sys.path.insert(0, _DIR)

print("=" * 70)
print("  烟幕干扰弹投放策略 -- 全题求解")
print("=" * 70)

for title, fname in [
    ("问题一: 单机单弹（给定参数）", "problem1.py"),
    ("问题二: 单机单弹（最优 NLP）", "problem2.py"),
    ("问题三 DE: 单机三弹（定向枚举）", "problem3_de.py"),
    ("问题三 NLP: 单机三弹（波束搜索）", "problem3_nlp.py"),
    ("问题四: 三机协同", "problem4.py"),
    ("问题五: 多机多弹多目标", "problem5.py"),
]:
    print("\n" + "-" * 50)
    print(f"  {title}")
    print("-" * 50)
    t0 = time.time()
    with open(os.path.join(_DIR, fname), encoding="utf-8") as f:
        exec(f.read())

print("\n" + "=" * 70)
print("  全部求解完成")
print("=" * 70)
