"""Objective metrics for one navigation trial.  Pure numpy; the same code scores real trials and simulated ones.

A trial record (`rec`) is a dict:
  t          (N,)    seconds since the goal was sent, sampled ~20 Hz
  pose       (N,3)   best-estimate map pose x, y, yaw (AMCL advanced by odometry)
  vel        (N,3)   measured body velocity vx, vy, w (wheel odometry)
  cmd_in     (N,3)   what the planner asked for (/cmd_vel, held between messages)
  cmd_out    (N,3)   what the safety gate let through (/cmd_vel_safe)
  clear      (N,)    gate's nearest-return distance minus the guard radius (m)
  limit      list[N] the gate's limiting reason per sample ('' = not limited)
  plans      list of (t, (M,2)) global-plan snapshots
  start, goal  (x, y, yaw)
  cmd_times  (K,)    arrival times of /cmd_vel messages (controller rate check)
  events     dict: end_reason ('reached'|'timeout'|'fault'|'stalled'|'aborted'), recoveries (int), faults (list)
  limits     dict: v_cmd, w_cmd, acc, dec (the planner limits used), controller_hz
"""
import math

import numpy as np

GATE_SAFETY_LABELS = ('obstacle', 'fence', 'tether', 'unknown_sector', 'blind_wedge', 'no_map_pose', 'cov', 'rot_block',
                      'brake_actual')


def _runs(mask, t):
    """Longest run (seconds) of True in mask; a run ends at the first False sample after it (or at the last sample)."""
    best, start = 0.0, None
    n = len(mask)
    for i in range(n):
        if mask[i] and start is None:
            start = i
        if start is not None and (not mask[i] or i == n - 1):
            best = max(best, t[i] - t[start])
            start = None
    return best


def dist_to_polyline(P, poly):
    """Distance from each point in P (n,2) to the polyline poly (m,2)."""
    if len(poly) == 1:
        return np.hypot(P[:, 0] - poly[0, 0], P[:, 1] - poly[0, 1])
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    ap = P[:, None, :] - a[None, :, :]
    u = np.clip((ap * ab[None]).sum(2) / L2[None], 0.0, 1.0)
    proj = a[None] + u[..., None] * ab[None]
    return np.sqrt(((P[:, None, :] - proj) ** 2).sum(2)).min(1)


def _smooth(x, k):
    if len(x) < k or k <= 1:
        return x
    ker = np.ones(k) / k
    pad = k // 2
    xp = np.pad(x, ((pad, k - 1 - pad), (0, 0)), mode='edge') if x.ndim == 2 else np.pad(x, (pad, k - 1 - pad), mode='edge')
    if x.ndim == 2:
        return np.stack([np.convolve(xp[:, j], ker, mode='valid') for j in range(x.shape[1])], axis=1)
    return np.convolve(xp, ker, mode='valid')


def compute(rec, cfg=None):
    cfg = cfg or {}
    t = np.asarray(rec['t'], dtype=float)
    pose = np.asarray(rec['pose'], dtype=float)
    vel = np.asarray(rec['vel'], dtype=float)
    cin = np.asarray(rec['cmd_in'], dtype=float)
    cout = np.asarray(rec['cmd_out'], dtype=float)
    clear = np.asarray(rec['clear'], dtype=float)
    clear = np.clip(np.where(np.isnan(clear), 3.0, clear), -0.5, 3.0)           # (numpy 1.13 on Ubuntu 18.04: no nan_to_num(posinf=))
    lim = rec['limits']
    v_lim = max(float(lim['v_cmd']), 1e-3)
    a_acc, a_dec = max(float(lim.get('acc', 0.4)), 0.05), max(float(lim.get('dec', 0.55)), 0.05)
    goal = np.asarray(rec['goal'][:2], dtype=float)
    ev = rec.get('events', {})
    N = len(t)
    m = {}
    m['n'] = N
    m['duration'] = float(t[-1] - t[0]) if N > 1 else 0.0
    if N < 5:
        m['success'] = 0.0
        m['valid'] = 0.0
        return m
    dg = np.hypot(pose[:, 0] - goal[0], pose[:, 1] - goal[1])
    dt = np.diff(t)
    step = np.hypot(np.diff(pose[:, 0]), np.diff(pose[:, 1]))
    jumpy = step > 0.15
    step = np.where(jumpy, 0.0, step)
    path_len = float(step.sum())
    plans = rec.get('plans') or []
    if plans:
        poly0 = np.asarray(plans[0][1], dtype=float)
        ref_len = float(np.hypot(*np.diff(poly0, axis=0).T).sum()) if len(poly0) > 1 else float(dg[0])
    else:
        ref_len = float(dg[0])
    ref_len = max(ref_len, 0.1)
    v_meas = np.hypot(vel[:, 0], vel[:, 1])

    m['success'] = 1.0 if ev.get('end_reason') == 'reached' else 0.0
    m['fault'] = 1.0 if ev.get('end_reason') == 'fault' or ev.get('faults') else 0.0
    m['valid'] = 1.0
    m['time_to_goal'] = m['duration']
    m['ref_len'] = ref_len
    m['path_len'] = path_len
    m['path_efficiency'] = ref_len / max(path_len, ref_len)
    t_ideal = ref_len / v_lim + 0.5 * v_lim / a_acc + 0.5 * v_lim / a_dec
    m['t_ideal'] = t_ideal
    m['delay_ratio'] = max(0.0, (m['duration'] - t_ideal) / t_ideal)
    m['final_error'] = float(dg[-1])
    m['progress_left'] = float(dg[-1] / max(dg[0], 0.1))

    # ---- path adherence (distance to the active global plan, ignoring the last 0.25 m)
    if plans:
        ct = np.full(N, np.nan)
        times = np.array([p[0] for p in plans])
        idx = np.clip(np.searchsorted(times, t, side='right') - 1, 0, len(plans) - 1)
        for k in np.unique(idx):
            sel = np.nonzero((idx == k) & (dg > 0.25))[0]
            if len(sel):
                ct[sel] = dist_to_polyline(pose[sel, :2], np.asarray(plans[k][1], dtype=float))
        ok = ~np.isnan(ct)
        if ok.any():
            m['ct_rms'] = float(np.sqrt(np.mean(ct[ok] ** 2)))
            m['ct_p95'] = float(np.percentile(ct[ok], 95))
            m['ct_max'] = float(ct[ok].max())
    for k in ('ct_rms', 'ct_p95', 'ct_max'):
        m.setdefault(k, 0.0)

    # ---- speed
    t_acc = v_lim / a_acc
    d_brake = v_lim ** 2 / (2 * a_dec) + 0.15
    cruise = (t - t[0] > t_acc) & (dg > d_brake)
    m['v_peak'] = float(v_meas.max())
    if cruise.sum() >= 5:
        m['speed_attainment'] = float(min(1.0, v_meas[cruise].mean() / v_lim))
    else:
        m['speed_attainment'] = float(min(1.0, m['v_peak'] / v_lim))
    still = (v_meas < 0.02) & (dg > 0.3) & (t - t[0] > 0.5)
    m['stall_frac'] = float(still.mean())
    m['stall_longest_s'] = float(_runs(still, t))

    # ---- smoothness (on what the robot was actually told to do), jitter (on what the planner asked for)
    k = max(3, int(round(0.15 / max(np.median(dt), 1e-3))))
    co = _smooth(cout[:, :2], k)
    acc = np.gradient(co, t, axis=0)
    jerk = np.gradient(_smooth(acc, k), t, axis=0)
    m['jerk_rms'] = float(np.sqrt(np.mean((jerk ** 2).sum(1))))
    m['acc_rms'] = float(np.sqrt(np.mean((acc ** 2).sum(1))))
    dci = np.diff(cin[:, :2], axis=0)
    tv = float(np.hypot(dci[:, 0], dci[:, 1]).sum())
    vpk_cmd = max(float(np.hypot(cin[:, 0], cin[:, 1]).max()), 0.05)
    m['tv_excess_per_m'] = float(max(0.0, tv / vpk_cmd - 2.0) / max(path_len, 0.5))
    rev = 0
    for j in (0, 1):
        s = 0
        for x in cin[:, j]:
            if abs(x) < 0.03:
                continue
            sg = 1 if x > 0 else -1
            if s and sg != s:
                rev += 1
            s = sg
    m['reversals_per_m'] = float(rev / max(path_len, 0.5))
    # high-frequency share of the command spectrum
    tt = np.arange(t[0], t[-1], 0.05)
    if len(tt) >= 40:
        hf, tot = 0.0, 0.0
        for j in range(3):
            x = np.interp(tt, t, cin[:, j])
            x = x - _smooth(x, 21)
            sp = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
            fr = np.fft.rfftfreq(len(x), 0.05)
            hf += sp[fr > 1.5].sum()
            tot += sp[fr > 0.05].sum() + 1e-12
        m['hf_ratio'] = float(hf / tot)
    else:
        m['hf_ratio'] = 0.0

    # ---- goal behaviour
    near = np.nonzero(dg < 0.5)[0]
    count, rise_max = 0, 0.0
    if len(near):
        mn = float('inf')
        for d in dg[near[0]:]:
            if d < mn:
                mn = d
            elif d > mn + 0.03:
                count += 1
                rise_max = max(rise_max, d - mn)
                mn = d
    m['goal_oscillations'] = float(count)
    m['goal_rise_max'] = float(rise_max)
    close = np.nonzero(dg < 0.3)[0]
    m['settle_time'] = float(t[-1] - t[close[0]]) if len(close) and m['success'] else (m['duration'] if m['success'] else 0.0)

    # ---- clearance
    comfort = float(cfg.get('clear_comfort', 0.10))
    m['clear_min'] = float(clear.min())
    m['clear_p5'] = float(np.percentile(clear, 5))
    m['clear_mean'] = float(clear.mean())
    m['clear_viol'] = float(np.mean(np.maximum(0.0, (comfort - clear) / comfort) ** 2))
    near_obs = (clear < 0.4) & (dg > 0.5) & (t - t[0] > t_acc)
    m['speed_near_obstacles'] = float(min(1.0, v_meas[near_obs].mean() / v_lim)) if near_obs.sum() >= 5 else 1.0

    # ---- safety layer involvement
    labels = np.array(rec.get('limit') or [''] * N)
    moving_cmd = np.hypot(cin[:, 0], cin[:, 1]) > 0.03
    safety = np.isin(labels, GATE_SAFETY_LABELS) & moving_cmd
    m['gate_limited_frac'] = float(safety.sum() / max(moving_cmd.sum(), 1))
    nz = np.hypot(cin[:, 0], cin[:, 1]) > 0.03
    cut = 1.0 - np.hypot(cout[nz, 0], cout[nz, 1]) / np.maximum(np.hypot(cin[nz, 0], cin[nz, 1]), 1e-6)
    m['gate_cut_mean'] = float(np.clip(cut, 0, 1).mean()) if nz.any() else 0.0

    # ---- health
    m['recoveries'] = float(ev.get('recoveries', 0))
    ct_times = np.asarray(rec.get('cmd_times', []), dtype=float)
    hz = float(lim.get('controller_hz', 0.0))
    if hz > 0 and m['duration'] > 2.0 and len(ct_times) > 0:
        m['rate_ratio'] = float(min(1.5, (len(ct_times) / m['duration']) / hz))
    else:
        m['rate_ratio'] = 1.0
    dpm = np.hypot(np.diff(pose[:, 0]), np.diff(pose[:, 1]))
    expl = v_meas[:-1] * dt + 0.03
    m['loc_jumps'] = float(np.sum(dpm > np.maximum(0.08, expl + 0.06)))
    if rec.get('scan_agree') is not None and len(rec['scan_agree']):
        m['scan_agree'] = float(np.mean(rec['scan_agree']))
    return m
