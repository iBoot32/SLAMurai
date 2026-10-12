"""Common experiment helpers on top of Robot: run a route with given Nav2 parameters, sweep, compare, optimize, log everything.
Agent-written scripts call these. Nothing here picks speeds, poses or ranges: the script does."""
import copy
import itertools
import json
import os
import re
import time

import numpy as np

import metrics as M
import objective as OBJ
import optimizer as OPT
import params as P
import stats
import trial as T
from robot import Robot, load_profile


class Halt(Exception):
    """The safety layer reported a fault. The script stops."""


class Budget(Exception):
    pass


class Run(object):
    """One route driven with one parameter set."""

    def __init__(self, label, params_changes, legs, loss):
        self.label, self.changes, self.legs, self.loss = label, params_changes, legs, loss

    def metric(self, name):
        v = [l['metrics'][name] for l in self.legs if name in l['metrics']]
        return float(np.mean(v)) if v else float('nan')

    def summary(self):
        ends = ','.join(l['trial']['events']['end_reason'] for l in self.legs) or 'route not started'
        return '%s: loss %.3f (%s) %s' % (self.label, self.loss, ends, self.changes)


class Lab(object):
    def __init__(self, robot, base, work, loss=None, metric_cfg=None, max_trials=None, max_minutes=None):
        """loss: function(metrics dict) -> float (default objective.trial_loss). base: Nav2 parameter dict to start from."""
        self.robot, self.base, self.work = robot, base, work
        self.aliases = robot.p.get('aliases', {})
        self.loss_fn = loss or (lambda m: OBJ.trial_loss(m)[0])
        self.metric_cfg = metric_cfg or {}
        self.max_trials, self.max_minutes = max_trials, max_minutes
        self.t0, self.n_trials, self.n_runs = time.time(), 0, 0
        self.applied = None
        os.makedirs(os.path.join(work, 'trials'), exist_ok=True)

    # ---- parameters
    def params(self, changes=None):
        return P.apply(self.base, changes or {}, self.aliases)

    def limits(self, params):
        out = {}
        for k, path in self.robot.p.get('limits_from_params', {}).items():
            try:
                out[k] = abs(float(P.get(params, path)))
            except (KeyError, TypeError):
                pass
        return out

    def _apply(self, params):
        if params != self.applied:
            self.robot.restart_nav2(params, initial_pose=self.robot.pose())
            self.applied = copy.deepcopy(params)

    # ---- running
    def _check_budget(self, n):
        if self.max_trials is not None and self.n_trials + n > self.max_trials:
            raise Budget('trial budget used')
        if self.max_minutes is not None and (time.time() - self.t0) / 60 > self.max_minutes:
            raise Budget('time budget used')

    def run(self, route, changes=None, label=None, **drive_kw):
        """Apply the parameter changes (Nav2 restarts if they differ from what runs now), drive the route, score every leg."""
        self._check_budget(len(route) - 1)
        params = self.params(changes)
        self._apply(params)
        self.n_runs += 1
        label = label or 'run%d' % self.n_runs
        trials = T.run_route(self.robot, route, **dict({'limits': self.limits(params)}, **drive_kw))
        legs = []
        for k, tr in enumerate(trials):
            m = M.compute(tr, self.metric_cfg)
            legs.append({'trial': tr, 'metrics': m, 'loss': float(self.loss_fn(m))})
            self.n_trials += 1
            self._save(label, k, tr, changes)
        loss = float(np.mean([l['loss'] for l in legs])) if legs else OBJ.DEFAULT['fail_base'] + 5.0
        r = Run(label, dict(changes or {}), legs, loss)
        self._log({'label': label, 'changes': r.changes, 'loss': loss, 'legs': [
            {'end': l['trial']['events']['end_reason'], 'loss': l['loss'], 'metrics': l['metrics']} for l in legs]})
        if any(l['trial']['events']['end_reason'] == 'fault' for l in legs):
            raise Halt('safety layer fault: %s' % legs[-1]['trial']['events']['faults'])
        return r

    def _save(self, label, k, tr, changes):
        plans = tr['plans']
        np.savez_compressed(os.path.join(self.work, 'trials', '%04d_%s_leg%d.npz' % (self.n_trials, re.sub(r'[^A-Za-z0-9_.-]+', '_', label)[:40], k)),
                            t=tr['t'], pose=tr['pose'], vel=tr['vel'], cmd_in=tr['cmd_in'], cmd_out=tr['cmd_out'], clear=tr['clear'],
                            plan_first=plans[0][1] if plans else np.zeros((0, 2)), plan_last=plans[-1][1] if plans else np.zeros((0, 2)),
                            limit=np.array(tr['limit']), start=np.array(tr['start']), goal=np.array(tr['goal']),
                            end=np.array(tr['events']['end_reason']), changes=np.array(json.dumps(changes or {}, default=float)))

    def _log(self, row):
        with open(os.path.join(self.work, 'lab.jsonl'), 'a') as f:
            f.write(json.dumps(row, default=float) + '\n')

    # ---- experiments
    def sweep(self, grid, route, repeats=3, mode='grid'):
        """grid: {param: [values]}. mode 'grid' = all combinations, 'oat' = one parameter at a time. Each round runs the base and
        every combination once (so slow drift hits all of them alike). Returns rows sorted by mean loss with the paired gain
        over the base and a sign-flip p-value."""
        if mode == 'oat':
            combos = [{n: x} for n in grid for x in grid[n]]
        else:
            names = list(grid)
            combos = [dict(zip(names, vals)) for vals in itertools.product(*[grid[n] for n in names])]
        base_l, loss = [], [[] for _ in combos]
        for r in range(repeats):
            base_l.append(self.run(route, {}, 'sweep_base#%d' % r).loss)
            for i, c in enumerate(combos):
                loss[i].append(self.run(route, c, 'sweep:%s#%d' % (json.dumps(c, default=float), r)).loss)
        rows = []
        for c, l in zip(combos, loss):
            s = stats.paired(base_l, l)
            rows.append({'changes': c, 'loss': float(np.mean(l)), 'sd': float(np.std(l)), 'gain': s['gain'], 'p': s['p']})
        rows.sort(key=lambda r: r['loss'])
        return {'base_loss': float(np.mean(base_l)), 'base_sd': float(np.std(base_l)), 'rows': rows}

    def compare(self, a, b, route, repeats=8):
        """Alternate runs of parameter changes a and b. gain > 0 means b is better."""
        la, lb = [], []
        for r in range(repeats):
            la.append(self.run(route, a, 'cmp_a#%d' % r).loss)
            lb.append(self.run(route, b, 'cmp_b#%d' % r).loss)
        return dict(stats.paired(la, lb), loss_a=float(np.mean(la)), loss_b=float(np.mean(lb)))

    def optimize(self, bounds, route, evals=30, repeats=1, confirm=8, alpha=0.05, min_gain=0.02, seed=0):
        """Trust-region Bayesian optimization of {param: (lo, hi)} (add 'log' as a third item for a log scale), starting from
        the base. The best few candidates are then compared with the base on alternating runs. Returns the accepted changes
        or None if nothing beat the base robustly."""
        names = list(bounds)

        def to_x(v):
            return np.array([self._u(bounds[n], v[n]) for n in names])

        def to_v(x):
            return dict((n, self._val(bounds[n], u)) for n, u in zip(names, x))
        x0 = to_x(dict((n, P.get(self.base, self.aliases.get(n, [n])[0])) for n in names))
        bo = OPT.TrustRegionBO(len(names), x0, seed=seed)
        for _ in range(evals):
            x = np.clip(bo.ask(), 0, 1)
            bo.tell(x, np.mean([self.run(route, to_v(x), 'opt').loss for _ in range(repeats)]), repeats)
            if bo.converged:
                break
        order = np.argsort(bo.Y)[:3]
        best = None
        for i in order:
            c = to_v(bo.X[i])
            s = self.compare({}, c, route, confirm)
            if s['gain'] > min_gain and s['p'] < alpha / len(order) and (best is None or s['gain'] > best[1]['gain']):
                best = (c, s)
        self._log({'label': 'optimize', 'names': names, 'accepted': best[0] if best else None, 'stats': best[1] if best else None})
        return best

    @staticmethod
    def _u(b, x):
        lo, hi = b[0], b[1]
        return (np.log(x) - np.log(lo)) / (np.log(hi) - np.log(lo)) if len(b) > 2 else (x - lo) / (hi - lo)

    @staticmethod
    def _val(b, u):
        lo, hi = b[0], b[1]
        return float(np.exp(np.log(lo) + u * (np.log(hi) - np.log(lo)))) if len(b) > 2 else float(lo + u * (hi - lo))

    def adopt(self, changes):
        """Make these changes part of the base for later runs."""
        for k, v in changes.items():
            for path in self.aliases.get(k, [k]):
                P.put(self.base, path, v)


def connect(profile, host=None, work=None, base=None, require_safe=True, **kw):
    """Connect to the robot described by the profile and return a Lab. Refuses if the profile has a safety status stream and
    it shows the safety layer not armed or faulted."""
    prof = load_profile(profile)
    robot = Robot(prof, host=host)
    robot.wait_ready()
    s = robot.status()
    if require_safe and s is not None and (not s['armed'] or s['faults']):
        robot.close()
        raise RuntimeError('safety layer not ready: armed=%s faults=%s' % (s['armed'], s['faults']))
    path = base or os.path.join(prof.get('_dir', '.'), prof['nav2_params'])
    work = work or os.path.join('runs', time.strftime('%Y%m%d_%H%M%S'))
    return Lab(robot, P.load(path), work, **kw)
