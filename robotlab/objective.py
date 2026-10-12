"""Metrics -> one loss (lower is better), with a per-term breakdown so a human or agent can see WHERE it is weak.

Each term is a dimensionless 'badness' (about 1.0 = clearly mediocre, 0 = ideal) times a weight.  A trial that fails
(no arrival, safety-layer fault) gets a large constant plus the fraction of the way left, so unsafe or broken
candidates always rank below any candidate that arrives.  Session loss = (1-lam) * weighted mean + lam * worst
scenario, so one scenario cannot be sacrificed for the others.  Defaults live in objective.yaml; the objective hash is
logged with every candidate so results from different objectives are never mixed."""
import hashlib
import json
import math

import numpy as np

DEFAULT = {
    'fail_base': 10.0,
    'worst_weight': 0.3,
    'terms': {
        # name: (metric key, scale = "this value counts as 1.0", cap, weight)
        'delay': ('delay_ratio', 0.5, 3.0, 2.0),                 # trip took 50 % longer than the ideal profile
        'speed': ('speed_attainment', None, 1.0, 1.0),           # special: 1 - attainment
        'path_rms': ('ct_rms', 0.10, 3.0, 1.5),                  # 10 cm RMS from the global plan
        'path_max': ('ct_max', 0.30, 3.0, 0.5),
        'jerk': ('jerk_rms', 2.0, 3.0, 1.0),                     # m/s^3
        'jitter_tv': ('tv_excess_per_m', 1.0, 3.0, 1.0),
        'reversals': ('reversals_per_m', 1.0, 3.0, 1.5),
        'hf_cmd': ('hf_ratio', 0.10, 3.0, 0.5),
        'goal_osc': ('goal_oscillations', 2.0, 3.0, 1.5),
        'goal_rise': ('goal_rise_max', 0.10, 3.0, 1.0),
        'settle': ('settle_time', 4.0, 3.0, 1.0),
        'clearance': ('clear_viol', 0.25, 3.0, 3.0),             # time spent inside the comfort band, squared
        'clear_min': ('clear_min', None, 1.0, 2.0),              # special: hinge below 3 cm
        'crawl': ('speed_near_obstacles', None, 1.0, 0.7),       # special: 1 - speed near obstacles (too timid)
        'stall': ('stall_frac', 0.10, 3.0, 1.5),
        'stall_long': ('stall_longest_s', 5.0, 3.0, 1.0),
        'gate': ('gate_limited_frac', 0.20, 3.0, 1.0),           # relying on the safety layer = not tuned yet
        'recover': ('recoveries', 1.0, 3.0, 2.0),
        'rate': ('rate_ratio', None, 3.0, 2.0),                  # special: shortfall below 0.9 of the controller rate
        'loc_jump': ('loc_jumps', 1.0, 3.0, 2.0),
    },
}


def objective_hash(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()[:12]


def trial_loss(m, obj=None):
    obj = obj or DEFAULT
    if not m.get('valid'):
        return obj['fail_base'] + 5.0, {'invalid': 1.0}
    if m.get('fault') or not m.get('success'):
        return obj['fail_base'] + 5.0 * min(1.0, m.get('progress_left', 1.0)), {
            'fail': obj['fail_base'], 'progress_left': 5.0 * min(1.0, m.get('progress_left', 1.0))}
    br, tot_w, tot = {}, 0.0, 0.0
    for name, (key, scale, cap, w) in obj['terms'].items():
        if key not in m:
            continue
        x = m[key]
        if name == 'speed':
            phi = 1.0 - x
        elif name == 'clear_min':
            phi = max(0.0, (0.03 - x) / 0.03)
        elif name == 'crawl':
            phi = 1.0 - x
        elif name == 'rate':
            phi = max(0.0, 0.9 - x) * 5.0
        else:
            phi = x / scale
        phi = float(min(max(phi, 0.0), cap))
        br[name] = w * phi
        tot += w * phi
        tot_w += w
    return tot / max(tot_w, 1e-9) * 3.0, br      # x3 so that "mediocre on every term" lands near 1-3, a clean run near 0.3-0.8


def session_loss(per_scenario, obj=None, weights=None):
    """per_scenario: {scenario_id: [trial losses]}.  Returns (J, per-scenario means)."""
    obj = obj or DEFAULT
    means = dict((k, float(np.mean(v))) for k, v in per_scenario.items() if len(v))
    if not means:
        return float('inf'), {}
    w = dict((k, (weights or {}).get(k, 1.0)) for k in means)
    mean = sum(w[k] * means[k] for k in means) / sum(w.values())
    lam = obj.get('worst_weight', 0.3)
    return (1 - lam) * mean + lam * max(means.values()), means
