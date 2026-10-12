import numpy as np


def signflip_p(d):
    """One-sided exact p-value that the mean paired difference d (baseline - candidate) is at least what we saw by chance."""
    d = np.asarray(d, dtype=float)
    n = len(d)
    if n <= 16:
        signs = ((np.arange(2 ** n)[:, None] >> np.arange(n)[None, :]) & 1) * 2 - 1
        means = (signs * d[None, :]).mean(1)
    else:
        means = (np.random.RandomState(0).choice([-1.0, 1.0], size=(20000, n)) * d[None, :]).mean(1)
    return float(np.mean(means >= d.mean() - 1e-12))


def paired(a, b, boot=2000):
    """Compare two lists of paired losses (lower is better). gain > 0 means b is better."""
    d = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    rng = np.random.RandomState(0)
    ci = np.percentile([rng.choice(d, len(d)).mean() for _ in range(boot)], [2.5, 97.5])
    return {'gain': float(d.mean()), 'p': signflip_p(d), 'ci95': [float(ci[0]), float(ci[1])], 'n': len(d)}
