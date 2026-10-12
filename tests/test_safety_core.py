import copy
import math
import os
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'robot_side', 'safety'))
from safety_core import SafetyCore, Zone, stop_safe_speed, wheel_speeds   # noqa: E402

CFG = yaml.safe_load(open(os.path.join(HERE, '..', 'robot_side', 'config', 'safety_config.yaml')))
N = 360
ANG0, DA = -math.pi, 2 * math.pi / N


def cfg(**kw):
    c = copy.deepcopy(CFG)
    c['lidar']['frame_yaw_rad'] = 0.0           # scan frame == base frame, easier to reason about in tests
    c['lidar']['axis_offset_xy_m'] = [0.0, 0.0]
    c['lidar']['blind_wedges_deg'] = []
    c.update(kw)
    return c


def scan_with_wall(dist, az=0.0, width_deg=40.0, far=6.0):
    """ranges for a straight wall whose nearest point is `dist` away in direction az, seen over +-width/2"""
    a = ANG0 + DA * np.arange(N)
    r = np.full(N, far)
    d = (a - az + math.pi) % (2 * math.pi) - math.pi
    m = np.abs(d) < math.radians(width_deg / 2)
    r[m] = np.minimum(far, dist / np.cos(d[m]))
    return r


def noisy(scan, t):
    return scan + np.random.RandomState(int(t * 1000) % 99991).normal(0, 0.002, N)


def make(c=None, **kw):
    core = SafetyCore(c or cfg(**kw))
    t = 0.0
    core.on_scan(t, ANG0, DA, scan_with_wall(5.0, far=6.0))
    core.on_odom(t, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ok, msg = core.arm(t, 600)
    assert ok, msg
    return core


def run(core, cmd, seconds, t0=0.0, scan=None, odom=None, hb=True, dt=0.05, cmd_every=True):
    t = t0
    out = (0, 0, 0)
    info = {}
    for _ in range(int(seconds / dt)):
        t += dt
        if scan is not None:
            core.on_scan(t, ANG0, DA, noisy(scan, t))
        o = odom or (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        core.on_odom(t, *o)
        if hb:
            core.heartbeat(t)
        if cmd_every and cmd is not None:
            core.on_cmd(t, *cmd)
        out, info = core.step(t)
    return out, info, t


FREE = scan_with_wall(5.0)


def test_stop_safe_speed_solves_the_inequality():
    for gap in (0.06, 0.2, 0.5, 1.5):
        for vo in (0.0, 0.25):
            v = stop_safe_speed(gap, 0.4, 0.5, 0.05, vo)
            need = v * 0.4 + v * v / (2 * 0.5) + 0.05 + vo * (0.4 + v / 0.5)
            if v > 0:
                assert abs(need - gap) < 1e-6          # tight (not wastefully slow) and satisfied
            else:
                assert 0.05 + vo * 0.4 >= gap - 1e-12  # standing still is only chosen when even v=0 does not fit
    assert stop_safe_speed(0.05, 0.4, 0.5, 0.05) == 0.0
    assert stop_safe_speed(float('inf'), 0.4, 0.5, 0.05) == float('inf')


def test_limits_clamp_garbage_and_over_limit():
    c = make()
    out, _, _ = run(c, (5.0, -5.0, 50.0), 4.0, scan=FREE)
    s = math.hypot(out[0], out[1])
    assert s <= CFG['v_max_m_s'] + 1e-9 and abs(out[2]) <= CFG['w_max_rad_s'] + 1e-9
    w = wheel_speeds(*out, CFG['wheel_azimuths_deg'], CFG['wheel_centre_radius_m'])
    assert max(abs(x) for x in w) <= CFG['wheel_cap_eff_m_s'] + 1e-9
    for bad in ((float('nan'), 0, 0), (float('inf'), 0, 0), (0, float('-inf'), 0), (0, 0, float('nan'))):
        c.on_cmd(10.0, *bad)
        out, info = c.step(10.0)
        assert out == (0.0, 0.0, 0.0)


def test_acceleration_limit_and_free_slowing():
    c = make()
    prev = (0.0, 0.0, 0.0)
    t = 0.0
    for i in range(40):
        t += 0.05
        c.on_scan(t, ANG0, DA, noisy(FREE, t))
        c.on_odom(t, 0, 0, 0, 0, 0, 0)
        c.heartbeat(t)
        c.on_cmd(t, 0.2, 0.0, 0.0)
        out, _ = c.step(t)
        assert out[0] - prev[0] <= CFG['a_acc_m_s2'] * 0.05 + 1e-9
        prev = out
    assert prev[0] == pytest.approx(0.2, abs=1e-6)
    c.on_cmd(t + 0.05, 0.0, 0.0, 0.0)
    out, _ = c.step(t + 0.05)
    assert out[0] == 0.0                               # slowing is immediate


@pytest.mark.parametrize('what', ['scan', 'odom', 'cmd', 'heartbeat'])
def test_stale_inputs_stop(what):
    c = make()
    out, _, t = run(c, (0.2, 0, 0), 3.0, scan=FREE)
    assert out[0] > 0.1
    kw = dict(scan=FREE, hb=True)
    dur = 1.2 if what != 'heartbeat' else CFG['heartbeat_timeout_s'] + 0.6
    if what == 'scan':
        kw['scan'] = None
    if what == 'heartbeat':
        kw['hb'] = False
    cmd_every = what != 'cmd'
    if what == 'odom':
        # odometry silent: use the core directly
        t += 1.0
        c.on_scan(t, ANG0, DA, FREE); c.heartbeat(t); c.on_cmd(t, 0.2, 0, 0)
        out, info = c.step(t)
    else:
        out, info, t = run(c, (0.2, 0, 0), dur, t0=t, cmd_every=cmd_every, **kw)
    assert out == (0.0, 0.0, 0.0), info


def test_scan_dead_for_3s_latches():
    c = make()
    run(c, (0.2, 0, 0), 1.0, scan=FREE)
    out, info, t = run(c, (0.2, 0, 0), 4.0, t0=1.0, scan=None)
    assert 'SCAN_DEAD' in c.faults
    out, info, _ = run(c, (0.2, 0, 0), 1.0, t0=t, scan=FREE)
    assert out == (0.0, 0.0, 0.0)                       # stays latched although the scan is back


def test_wall_ahead_limits_speed_and_never_below_margin():
    c = make()
    wall = scan_with_wall(0.50)
    out, info, _ = run(c, (0.2, 0, 0), 5.0, scan=wall)
    gap = 0.50 - CFG['robot_radius_m'] - CFG['lateral_margin_m']
    t_lat = CFG['latency_fixed_s'] + CFG['scan_time_s']
    allowed = stop_safe_speed(gap, t_lat, CFG['a_brake_m_s2'], CFG['stop_margin_m'], CFG['dyn_obstacle_speed_m_s'])
    assert 0 < out[0] <= allowed + 1e-6 < 0.2
    # at 0.30 m centre distance nothing forward is allowed
    c2 = make()
    out, info, _ = run(c2, (0.2, 0, 0), 3.0, scan=scan_with_wall(0.30))
    assert out[0] == 0.0


def test_backing_out_and_sliding_are_allowed():
    c = make()
    wall = scan_with_wall(0.30)
    out, info, _ = run(c, (-0.15, 0, 0), 3.0, scan=wall)         # directly away from the wall
    assert out[0] < -0.1
    c = make()
    out, _, _ = run(c, (0, 0.15, 0), 3.0, scan=scan_with_wall(0.40, az=0.0, width_deg=60))
    assert out[1] > 0.1


def test_wall_alongside_is_kept_at_corridor_distance():
    """driving parallel to a wall inside the corridor half-width is blocked, outside it is free"""
    c = make()
    near = scan_with_wall(0.20, az=math.pi / 2, width_deg=170)    # wall on the left at 0.20 m
    out, _, _ = run(c, (0.2, 0, 0), 3.0, scan=near)
    assert out[0] == 0.0
    c = make()
    far = scan_with_wall(0.40, az=math.pi / 2, width_deg=170)
    out, _, _ = run(c, (0.2, 0, 0), 3.0, scan=far)
    assert out[0] > 0.15


def test_rotation_blocked_only_when_something_is_inside_the_hull():
    c = make()
    out, _, _ = run(c, (0, 0, 0.4), 2.0, scan=scan_with_wall(0.30, width_deg=20))
    assert out[2] > 0.3
    c = make()
    out, _, _ = run(c, (0, 0, 0.4), 2.0, scan=scan_with_wall(0.17, width_deg=20))
    assert out[2] == 0.0


def test_unknown_sector_creeps():
    c = make()
    r = FREE.copy()
    a = ANG0 + DA * np.arange(N)
    r[np.abs(a) < math.radians(25)] = np.inf
    out, info, _ = run(c, (0.2, 0, 0), 4.0, scan=r)
    assert 0 < math.hypot(out[0], out[1]) <= CFG['unknown_creep_m_s'] + 1e-9


def test_blind_wedge_caps_speed_in_that_direction():
    cc = cfg()
    cc['lidar']['blind_wedges_deg'] = [[90.0, 2.0]]
    c = SafetyCore(cc)
    c.on_scan(0.0, ANG0, DA, FREE)
    c.on_odom(0.0, 0, 0, 0, 0, 0, 0)
    assert c.arm(0.0, 600)[0]
    out, _, _ = run(c, (0.0, 0.2, 0), 4.0, scan=FREE)
    assert math.hypot(*out[:2]) <= CFG['blind_wedge_speed_cap_m_s'] + 1e-9


def test_tether_soft_then_hard():
    c = make()
    R = CFG['tether_radius_m']
    # robot already near the tether edge and driving outward: speed collapses before the edge
    out, info, t = run(c, (0.2, 0, 0), 2.0, scan=FREE, odom=(R - 0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert out[0] < 0.1
    # beyond radius + extra: hard latch
    out, info, t = run(c, (0.2, 0, 0), 0.5, t0=t, scan=FREE, odom=(R + 0.5, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert 'TETHER' in c.faults or 'ODOM_JUMP' in c.faults
    assert out == (0.0, 0.0, 0.0)


def test_odom_jump_latches():
    c = make()
    run(c, (0.2, 0, 0), 1.0, scan=FREE)
    c.on_odom(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    c.on_odom(2.05, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert 'ODOM_JUMP' in c.faults


def test_contact_latches_after_three_scans():
    c = make()
    bad = scan_with_wall(0.16, width_deg=10)
    for i in range(3):
        c.on_scan(1.0 + 0.1 * i, ANG0, DA, bad + i * 1e-3)
    assert 'CONTACT' in c.faults


def test_frozen_scan_latches():
    c = make()
    s = FREE.copy()
    for i in range(8):
        c.on_scan(1.0 + 0.1 * i, ANG0, DA, s)
    assert 'SCAN_DEAD' in c.faults


def test_faults_need_a_human_and_arm_refuses_while_latched():
    c = make()
    c._fault(1.0, 'TETHER', 'test')
    out, info = c.step(1.1)
    assert out == (0, 0, 0) and info['state'] == 'FAULT'
    ok, msg = c.arm(2.0, 100)
    assert not ok
    c.clear_faults(2.0)
    c.on_scan(2.0, ANG0, DA, FREE)
    c.on_odom(2.0, 0, 0, 0, 0, 0, 0)
    assert c.arm(2.0, 100)[0]


def test_session_lease_expires():
    c = make()
    c.on_cmd(0.1, 0.1, 0, 0)
    out, info, t = run(c, (0.1, 0, 0), 1.0, scan=FREE)
    assert out[0] > 0
    c.session_end = t + 0.2
    out, info, t = run(c, (0.1, 0, 0), 1.0, t0=t, scan=FREE)
    assert 'SESSION_TIMEOUT' in c.faults and out == (0.0, 0.0, 0.0)


def test_motion_mismatch_catches_reversed_wheels():
    """odometry says +x, scan matching says -x (a wheel wired backwards, or localisation jumped)"""
    c = make()
    t = 1.0
    for i in range(1, 8):
        t += 0.5
        c.on_odom(t, 0.35 * i, 0.0, 0.0, 0.0, 0.0, 0.0)
        c.on_map_pose(t, -0.35 * i, 0.0, 0.0, 1e-4, 1e-4)
    assert 'MOTION_MISMATCH' in c.faults


def test_motion_consistency_tolerates_a_rotated_map_frame_and_noise():
    c = make()
    T = 0.7
    rng = np.random.RandomState(0)
    t = 1.0
    for i in range(1, 12):
        t += 0.5
        ox, oy = 0.3 * i, 0.1 * i
        mx = math.cos(T) * ox - math.sin(T) * oy + 5.0 + rng.normal(0, 0.02)
        my = math.sin(T) * ox + math.cos(T) * oy - 2.0 + rng.normal(0, 0.02)
        c.on_odom(t, ox, oy, 0.0, 0.0, 0.0, 0.0)
        c.on_map_pose(t, mx, my, T, 1e-4, 1e-4)
    assert 'MOTION_MISMATCH' not in c.faults


def _zone_disc(radius=1.0, res=0.05, size=4.0):
    n = int(size / res)
    yy, xx = np.mgrid[0:n, 0:n]
    cx = cy = n / 2
    d = (np.hypot(xx + 0.5 - cx, yy + 0.5 - cy) * res)
    sdf = (radius - d).astype(np.float32)
    return Zone(sdf, res, -size / 2, -size / 2)


def test_zone_fence_stops_at_edge_allows_retreat_and_latches_breach():
    z = _zone_disc(1.0)
    c = SafetyCore(cfg(), z)
    c.on_scan(0.0, ANG0, DA, FREE)
    c.on_odom(0.0, 0, 0, 0, 0, 0, 0)
    assert c.arm(0.0, 600)[0]
    t = 0.0
    # robot at map (0.8, 0): 0.1 m from the edge, driving outward
    for i in range(60):
        t += 0.05
        c.on_scan(t, ANG0, DA, noisy(FREE, t))
        c.on_odom(t, 0, 0, 0, 0, 0, 0)
        c.on_map_pose(t, 0.9, 0.0, 0.0, 1e-4, 1e-4)
        c.heartbeat(t)
        c.on_cmd(t, 0.2, 0, 0)
        out, info = c.step(t)
    assert out[0] < 0.1, info
    # retreat inward is allowed
    for i in range(30):
        t += 0.05
        c.on_scan(t, ANG0, DA, noisy(FREE, t))
        c.on_odom(t, 0, 0, 0, 0, 0, 0)
        c.on_map_pose(t, 0.9, 0.0, 0.0, 1e-4, 1e-4)
        c.heartbeat(t)
        c.on_cmd(t, -0.2, 0, 0)
        out, info = c.step(t)
    assert out[0] < -0.1, info
    # outside the zone by more than 15 cm for 1 s -> hard fault
    for i in range(30):
        t += 0.05
        c.on_scan(t, ANG0, DA, noisy(FREE, t))
        c.on_odom(t, 0, 0, 0, 0, 0, 0)
        c.on_map_pose(t, 1.2, 0.0, 0.0, 1e-4, 1e-4)
        c.heartbeat(t)
        c.on_cmd(t, 0.0, 0, 0)
        c.step(t)
    assert 'FENCE_BREACH' in c.faults


def test_zone_needs_a_map_pose():
    c = SafetyCore(cfg(), _zone_disc())
    c.on_scan(0.0, ANG0, DA, FREE)
    c.on_odom(0.0, 0, 0, 0, 0, 0, 0)
    assert c.arm(0.0, 600)[0]
    out, info, _ = run(c, (0.2, 0, 0), 2.0, scan=FREE)
    assert math.hypot(*out[:2]) <= CFG['unknown_creep_m_s'] + 1e-9
