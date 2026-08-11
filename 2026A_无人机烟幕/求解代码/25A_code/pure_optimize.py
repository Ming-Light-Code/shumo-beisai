#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure-Python optimization routines replacing scipy.optimize.
differential_evolution: global search via DE algorithm.
minimize: local refinement via coordinate descent with line search.
"""

import numpy as np
import time


def differential_evolution(func, bounds, popsize=30, maxiter=200,
                            tol=1e-6, seed=None, disp=False, polish=True):
    """
    Pure-Python differential evolution (DE/rand/1/bin).

    Parameters:
        func: objective f(x) -> float (to be MINIMIZED)
        bounds: list of (lo, hi) for each dimension
        popsize: population size (default 30)
        maxiter: maximum generations
        tol: convergence tolerance on std of population fitness
        seed: random seed
        disp: print progress
        polish: apply local refinement to best solution

    Returns: result object with .x (best params) and .fun (best value)
    """
    rng = np.random.RandomState(seed)
    n_dim = len(bounds)
    lo = np.array([b[0] for b in bounds])
    hi = np.array([b[1] for b in bounds])

    # Initialize population uniformly
    pop = lo + rng.uniform(0, 1, (popsize, n_dim)) * (hi - lo)
    fitness = np.array([func(p) for p in pop])

    best_idx = np.argmin(fitness)
    best_x = pop[best_idx].copy()
    best_f = fitness[best_idx]

    F = 0.85   # mutation factor
    CR = 0.9   # crossover probability

    for gen in range(maxiter):
        for i in range(popsize):
            # Select three distinct random individuals (a, b, c != i)
            idxs = [j for j in range(popsize) if j != i]
            a, b, c = pop[rng.choice(idxs, 3, replace=False)]

            # Mutation: v = a + F * (b - c)
            mutant = a + F * (b - c)

            # Crossover (binomial)
            cross_mask = rng.uniform(0, 1, n_dim) < CR
            if not np.any(cross_mask):
                cross_mask[rng.randint(n_dim)] = True
            trial = np.where(cross_mask, mutant, pop[i])

            # Boundary handling: reflect
            trial = np.clip(trial, lo, hi)

            # Selection
            trial_f = func(trial)
            if trial_f <= fitness[i]:
                pop[i] = trial
                fitness[i] = trial_f
                if trial_f < best_f:
                    best_x = trial.copy()
                    best_f = trial_f

        # Convergence check
        if np.std(fitness) < tol * (1 + abs(best_f)):
            if disp:
                print(f"  DE converged at generation {gen}, best={best_f:.6f}")
            break

        if disp and (gen + 1) % 50 == 0:
            print(f"  DE gen {gen+1}/{maxiter}, best={best_f:.6f}")

    # Polish: local refinement via coordinate descent
    if polish:
        best_x, best_f = _coordinate_descent(func, best_x, bounds,
                                              max_iter=100, disp=disp)

    return _DE_Result(best_x, best_f)


def _coordinate_descent(func, x0, bounds, max_iter=200, disp=False):
    """
    Simple coordinate descent with line search along each dimension.
    For unconstrained or box-constrained problems.
    """
    x = np.array(x0, dtype=float)
    lo = np.array([b[0] for b in bounds])
    hi = np.array([b[1] for b in bounds])
    best_f = func(x)
    n_dim = len(x)

    for iteration in range(max_iter):
        improved = False
        for d in range(n_dim):
            # Try positive and negative steps of decreasing size
            step = (hi[d] - lo[d]) * 0.01
            for _ in range(8):
                found = False
                for sgn in [-1, 1]:
                    xt = x.copy()
                    xt[d] = np.clip(x[d] + sgn * step, lo[d], hi[d])
                    ft = func(xt)
                    if ft < best_f - 1e-12:
                        x[d] = xt[d]
                        best_f = ft
                        improved = True
                        found = True
                if not found:
                    step *= 0.5
                else:
                    break
        if not improved:
            break

    return x, best_f


def minimize(func, x0, args=(), method=None, bounds=None,
             constraints=None, options=None):
    """
    Minimize a scalar function (scipy.optimize.minimize compatible interface).
    Falls back to coordinate descent with penalty for constraints.
    """
    x0 = np.array(x0, dtype=float)
    n_dim = len(x0)

    if bounds is None:
        bounds = [(-1e10, 1e10)] * n_dim
    lo = np.array([b[0] for b in bounds])
    hi = np.array([b[1] for b in bounds])

    if options is None:
        options = {}
    max_iter = options.get("maxiter", 500)

    # Build penalized objective
    def penalized(x):
        val = func(x)
        penalty = 0.0
        # Box constraints
        for i in range(n_dim):
            if x[i] < lo[i]:
                penalty += 1e6 * (lo[i] - x[i])
            if x[i] > hi[i]:
                penalty += 1e6 * (x[i] - hi[i])
        # Inequality constraints
        if constraints:
            for con in constraints:
                if con["type"] == "ineq":
                    c_val = con["fun"](x)
                    if c_val < 0:
                        penalty += 1e6 * abs(c_val)
        return val + penalty

    x_opt, f_opt = _coordinate_descent(penalized, x0, bounds,
                                        max_iter=max_iter)

    return _Minimize_Result(x_opt, f_opt, True)


class _DE_Result:
    def __init__(self, x, fun):
        self.x = x
        self.fun = fun
        self.success = True


class _Minimize_Result:
    def __init__(self, x, fun, success):
        self.x = x
        self.fun = fun
        self.success = success


if __name__ == "__main__":
    # Quick test
    def rosenbrock(x):
        return (1-x[0])**2 + 100*(x[1]-x[0]**2)**2
    bounds = [(-5, 5), (-5, 5)]
    res = differential_evolution(rosenbrock, bounds, popsize=20, maxiter=100,
                                 seed=42, disp=True)
    print(f"DE result: x={res.x}, f={res.fun}")
