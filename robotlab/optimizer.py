"""Sample-efficient optimiser for expensive, noisy, failure-prone evaluations (each one drives a real robot).

  * Gaussian-process surrogate (Matern 5/2, per-dimension lengthscales fitted by marginal likelihood), numpy only.
  * Expected improvement, maximised over candidates drawn INSIDE A TRUST REGION around the incumbent
    (TuRBO-style, perturbing only a few coordinates at a time).  The region grows after real improvements and shrinks
    after failures, so the search walks away from the known-good configuration instead of jumping across the space:
    this is the 'safe exploration' idea of SafeOpt applied to the planner parameters (the hard safety is the gate).
  * Failed / unsafe evaluations are winsorised so one crash cannot wreck the model.
  * Noise-aware: improvements must exceed the measured noise to move the incumbent.
All coordinates live in the unit cube; mapping to Nav2 parameters is param_space's job.
"""
import math

import numpy as np


def _matern52(A, B, ls):
    A, B = A / ls, B / ls
    d2 = (A ** 2).sum(1)[:, None] + (B ** 2).sum(1)[None, :] - 2 * A @ B.T
    d = np.sqrt(np.maximum(d2, 0.0))
    s5 = math.sqrt(5.0)
    return (1 + s5 * d + 5.0 / 3.0 * d2.clip(0)) * np.exp(-s5 * d)


class GP(object):
    def __init__(self, noise_var=0.02):
        self.noise_var = noise_var

    def _nll(self, X, y, ls, sf2):
        K = sf2 * _matern52(X, X, ls) + (self.noise_var + 1e-6) * np.eye(len(X))
        try:
            L = np.linalg.cholesky(K)
        except np.linalg.LinAlgError:
            return 1e18
        a = np.linalg.solve(L.T, np.linalg.solve(L, y))
        return 0.5 * float(y @ a) + float(np.log(np.diag(L)).sum())

    def fit(self, X, y):
        self.X = np.asarray(X, dtype=float)
        y = np.asarray(y, dtype=float)
        self.ym, self.ys = float(np.mean(y)), float(np.std(y)) or 1.0
        z = (y - self.ym) / self.ys
        d = self.X.shape[1]
        ls = np.full(d, 0.4)
        sf2 = 1.0
        best = self._nll(self.X, z, ls, sf2)
        for _ in range(2):                                  # coordinate ascent on log-lengthscales
            for j in range(d):
                for f in (0.5, 0.75, 1.33, 2.0):
                    l2 = ls.copy()
                    l2[j] = min(max(l2[j] * f, 0.05), 5.0)
                    v = self._nll(self.X, z, l2, sf2)
                    if v < best - 1e-9:
                        best, ls = v, l2
            for f in (0.5, 0.75, 1.33, 2.0):
                v = self._nll(self.X, z, ls, sf2 * f)
                if v < best - 1e-9:
                    best, sf2 = v, sf2 * f
        self.ls, self.sf2, self.z = ls, sf2, z
        K = sf2 * _matern52(self.X, self.X, ls) + (self.noise_var + 1e-6) * np.eye(len(self.X))
        self.L = np.linalg.cholesky(K)
        self.alpha = np.linalg.solve(self.L.T, np.linalg.solve(self.L, z))
        return self

    def predict(self, Xs):
        Ks = self.sf2 * _matern52(np.asarray(Xs, dtype=float), self.X, self.ls)
        mu = Ks @ self.alpha
        v = np.linalg.solve(self.L, Ks.T)
        var = np.maximum(self.sf2 - (v ** 2).sum(0), 1e-9)
        return mu * self.ys + self.ym, var * self.ys ** 2


def expected_improvement(mu, var, best):
    s = np.sqrt(var)
    z = (best - mu) / s
    cdf = 0.5 * (1 + np.vectorize(math.erf)(z / math.sqrt(2)))
    pdf = np.exp(-0.5 * z ** 2) / math.sqrt(2 * math.pi)
    return (best - mu) * cdf + s * pdf


class TrustRegionBO(object):
    """Minimise J(x), x in [0,1]^d.  Usage: x = opt.ask(); ...evaluate...; opt.tell(x, J, n_repeats)."""

    def __init__(self, dim, x0, seed=0, noise_sd=0.1, tr=0.25, tr_min=0.03, tr_max=0.5, y_cap=8.0, n_init=None, k_dims=8):
        self.d, self.rng = dim, np.random.RandomState(seed)
        self.X, self.Y, self.N = [], [], []
        self.noise_sd, self.tr, self.tr_min, self.tr_max, self.y_cap = noise_sd, tr, tr_min, tr_max, y_cap
        self.x0 = np.asarray(x0, dtype=float)
        self.n_init = n_init if n_init is not None else max(5, min(2 * dim, 12))
        self.k_dims = min(k_dims, dim)
        self.fail_streak = 0
        self.best_i = None
        self.converged = False
        self.history = []

    # incumbent = lowest mean among points that were evaluated; ties broken towards more repeats
    def incumbent(self):
        if not self.Y:
            return self.x0, None
        i = int(np.argmin(self.Y))
        return self.X[i], self.Y[i]

    def tell(self, x, y, n=1):
        x = np.asarray(x, dtype=float)
        for i, xi in enumerate(self.X):                     # same point again: pool the measurements
            if np.allclose(xi, x, atol=1e-9):
                tot = self.N[i] + n
                self.Y[i] = (self.Y[i] * self.N[i] + y * n) / tot
                self.N[i] = tot
                return
        prev = self.incumbent()[1]
        self.X.append(x)
        self.Y.append(float(y))
        self.N.append(n)
        thr = max(0.02, 2.0 * self.noise_sd / math.sqrt(max(n, 1)))
        if prev is None:
            return
        if y < prev - thr:
            self.tr = min(self.tr_max, self.tr * 1.5)
            self.fail_streak = 0
        else:
            self.fail_streak += 1
            if self.fail_streak >= 3:
                self.tr *= 0.6
                self.fail_streak = 0
                if self.tr < self.tr_min:
                    self.converged = True

    def ask(self):
        n = len(self.X)
        center = self.incumbent()[0]
        lo, hi = np.clip(center - self.tr, 0, 1), np.clip(center + self.tr, 0, 1)
        if n < self.n_init:                                  # space-filling start inside the region around x0
            return self._lhs_point(n, lo, hi)
        y = np.minimum(np.asarray(self.Y), self.y_cap)
        gp = GP(noise_var=(self.noise_sd / (np.std(y) or 1.0)) ** 2).fit(np.array(self.X), y)
        cand = np.tile(center, (3000, 1))
        for r in range(len(cand)):                           # perturb a random subset of coordinates
            k = self.rng.randint(1, self.k_dims + 1)
            dims = self.rng.choice(self.d, size=k, replace=False)
            cand[r, dims] = self.rng.uniform(lo[dims], hi[dims])
        mu, var = gp.predict(cand)
        ei = expected_improvement(mu, var, float(np.min(gp.predict(np.array(self.X))[0])))
        # never repeat an evaluated point exactly
        return cand[int(np.argmax(ei))]

    def _lhs_point(self, n, lo, hi):
        if n == 0:
            return self.x0.copy()
        if not hasattr(self, '_lhs'):
            m = self.n_init
            perm = [self.rng.permutation(m) for _ in range(self.d)]
            self._lhs = (np.array(perm).T + self.rng.uniform(size=(m, self.d))) / m
        u = self._lhs[n % len(self._lhs)]
        x = lo + u * (hi - lo)
        mask = self.rng.uniform(size=self.d) < max(0.3, self.k_dims / self.d)   # move only some coordinates
        return np.where(mask, x, self.x0)


def coordinate_search(f, x0, step=0.2, min_step=0.03, max_evals=60, patience=5):
    """The 'step up / step down on one parameter' method from the tuning protocol, as a fallback and for sweeps.
    f(x)->J.  Returns (best_x, best_J, trace)."""
    x = np.asarray(x0, dtype=float).copy()
    best = f(x)
    trace = [(x.copy(), best)]
    evals, since = 1, 0
    while step >= min_step and evals < max_evals and since < patience * len(x):
        improved = False
        for j in range(len(x)):
            for s in (+step, -step):
                y = x.copy()
                y[j] = min(max(y[j] + s, 0.0), 1.0)
                if y[j] == x[j]:
                    continue
                v = f(y)
                evals += 1
                trace.append((y.copy(), v))
                if v < best - 1e-9:
                    x, best, improved = y, v, True
                    break
            if evals >= max_evals:
                break
        since = 0 if improved else since + len(x)
        if not improved:
            step *= 0.5
    return x, best, trace
