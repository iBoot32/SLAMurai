"""Drive the robot to a goal and record what happened. A trial is a dict of arrays that metrics.compute() understands."""
import math
import time

import numpy as np

import metrics as M


def _d(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def stop(robot, wait=6.0):
    robot.cancel()
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait:
        v = robot.velocity()
        if math.hypot(v[0], v[1]) < 0.01 and abs(v[2]) < 0.02:
            return True
        robot.sleep(0.1)
    return False


def drive(robot, goal, limits=None, tol=0.12, timeout=180.0, stall_s=25.0, blocked_s=15.0, rate_hz=20.0, settle_s=0.5):
    """Send one goal and record until the robot arrives and stops, the safety layer reports a fault, it stalls, or time runs out.
    goal = (x, y, yaw). limits (optional) = {'v_cmd', 'w_cmd', 'acc', 'dec'} used by the metrics."""
    start = robot.pose()
    if start is None:
        raise RuntimeError('no robot pose')
    rec = dict((k, []) for k in ('t', 'pose', 'vel', 'cmd_in', 'cmd_out', 'clear', 'limit'))
    plans, faults = [], []
    rec0 = robot.count('recovery')
    robot.plan()
    t0 = time.monotonic()
    robot.send_goal(*goal)
    best, last_prog = _d(start, goal), (t0, _d(start, goal))
    ok_since = still_since = None
    end = 'timeout'
    while time.monotonic() - t0 < timeout:
        now = time.monotonic()
        t = now - t0
        p = robot.pose()
        if p is not None:
            pl = robot.plan()
            if pl is not None:
                plans.append((t, pl))
            v, ci, co, s = robot.velocity(), robot.cmd_in(), robot.cmd_out(), robot.status()
            rec['t'].append(t); rec['pose'].append(p); rec['vel'].append(v); rec['cmd_in'].append(ci); rec['cmd_out'].append(co)
            rec['clear'].append(3.0 if not s or s['clearance'] is None else s['clearance'])
            rec['limit'].append(s['limit'] if s else '')
            faults = robot.faults()
            if faults:
                end = 'fault'
                break
            d, sp = _d(p, goal), math.hypot(v[0], v[1])
            if d < tol and sp < 0.03:
                ok_since = ok_since or now
                if now - ok_since > settle_s:
                    end = 'reached'
                    break
            else:
                ok_since = None
            if d < best - 0.1:
                best, last_prog = d, (now, d)
            if now - last_prog[0] > stall_s and d > tol:
                end = 'stalled'
                break
            if math.hypot(*ci[:2]) > 0.03 and math.hypot(co[0], co[1]) < 0.005 and abs(co[2]) < 0.01:
                still_since = still_since or now
                if now - still_since > blocked_s:
                    end = 'stalled'
                    break
            else:
                still_since = None
        robot.sleep(1.0 / rate_hz)
    stop(robot)
    S = [x for x in robot.samples_between(t0, time.monotonic()) if x[1] is not None] if hasattr(robot, 'samples_between') else []
    if len(S) > len(rec['t']):                          # full-rate telemetry: replace the loop's samples, keep its gate fields
        lt = np.array(rec['t']) if rec['t'] else np.zeros(1)
        idx = [min(len(rec['clear']) - 1, int(np.searchsorted(lt, x[0] - t0))) for x in S] if rec['clear'] else None
        rec = {'t': [x[0] - t0 for x in S], 'pose': [x[1] for x in S], 'vel': [x[2] for x in S], 'cmd_in': [x[3] for x in S],
               'cmd_out': [x[4] for x in S], 'clear': [rec['clear'][i] for i in idx] if idx else [3.0] * len(S),
               'limit': [rec['limit'][i] for i in idx] if idx else [''] * len(S)}
    cin = np.array(rec['cmd_in']) if rec['cmd_in'] else np.zeros((1, 3))
    lim = dict(limits or {})
    lim.setdefault('v_cmd', max(0.05, float(np.hypot(cin[:, 0], cin[:, 1]).max())))
    out = dict((k, np.array(v)) for k, v in rec.items() if k != 'limit')
    out.update(limit=rec['limit'], plans=plans, start=start, goal=tuple(goal), limits=lim,
               cmd_times=np.array([c - t0 for c in robot.cmd_times() if c >= t0]),
               events={'end_reason': end, 'recoveries': robot.count('recovery') - rec0, 'faults': faults})
    return out


def run_route(robot, poses, gap_s=5.0, **kw):
    """Go to poses[0] (not recorded), then drive pose to pose. Returns one trial per leg.
    A pose is (x, y) or (x, y, yaw); a missing yaw points along the leg. gap_s: pause between legs, because Nav2 drops a goal that arrives right after it finished one."""
    P = []
    for k, p in enumerate(poses):
        if len(p) >= 3:
            P.append(tuple(p[:3]))
            continue
        a, b = (p, poses[k + 1]) if k + 1 < len(poses) else (poses[k - 1], p)
        P.append((p[0], p[1], math.atan2(b[1] - a[1], b[0] - a[0])))
    first = drive(robot, P[0], **kw)
    if first['events']['end_reason'] != 'reached':
        return []
    out = []
    for g in P[1:]:
        robot.sleep(gap_s)
        tr = drive(robot, g, **kw)
        out.append(tr)
        if tr['events']['end_reason'] == 'fault':
            break
    return out


def score(trial, cfg=None):
    return M.compute(trial, cfg)
