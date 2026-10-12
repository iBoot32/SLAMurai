#!/usr/bin/env python3
"""Offline pre-flight: every check that can be made without the robot.  Exit code 0 only if all hard checks pass.
Run it before every session (the CLI refuses to start motion if it fails) and after any edit of config, firmware or
parameter space.   python tools/preflight.py [--zone zone.npz]"""
import argparse
import ast
import hashlib
import json
import math
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
REPO = os.path.abspath(os.path.join(ROOT, '..'))
CFG = os.path.join(ROOT, 'config')
sys.path.insert(0, os.path.join(REPO, 'robotlab'))
sys.path.insert(0, os.path.join(ROOT, 'safety'))

RUNTIME_FILES = ['safety/safety_core.py', 'safety/safety_gate.py', 'safety/commission.py', 'tools/nav2_manager.py',
                 'tools/limits_check.py', 'tools/nav2_proc.py', '../autonav/nav/src/main.py']

results = []


def check(name, ok, detail='', hard=True):
    results.append((name, ok, detail, hard))


def config_hash(cfg):
    keep = {k: v for k, v in cfg.items() if k not in ('why', 'nav', 'lock_sha256')}
    return hashlib.sha256(json.dumps(keep, sort_keys=True).encode()).hexdigest()[:16]


def py36_ok(path):
    """Runtime code must parse as Python 3.6 (Ubuntu 18.04 / ROS 2 Dashing) and avoid 3.7+ library features."""
    src = open(path, encoding='utf-8').read()
    try:
        try:
            tree = ast.parse(src, feature_version=(3, 6))
        except TypeError:                   # interpreter older than 3.8 cannot emulate 3.6, but is itself a valid target
            tree = ast.parse(src)
    except SyntaxError as e:
        return False, 'syntax: %s' % e
    bad = []
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom) and n.module in ('dataclasses', '__future__') and any(a.name == 'annotations' for a in n.names):
            bad.append('line %d: %s import' % (n.lineno, n.module))
        if isinstance(n, ast.Import) and any(a.name == 'dataclasses' for a in n.names):
            bad.append('line %d: dataclasses (3.7+)' % n.lineno)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in ('fromisoformat', 'perf_counter_ns', 'monotonic_ns'):
            bad.append('line %d: %s (3.7+)' % (n.lineno, n.func.attr))
        if isinstance(n, ast.NamedExpr) if hasattr(ast, 'NamedExpr') else False:
            bad.append('line %d: walrus operator' % n.lineno)
    return (not bad), '; '.join(bad)


def define_value(text, name):
    m = re.search(r'#define\s+%s\s+\(?([-+0-9.eE]+)f?\)?' % name, text)
    return float(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--zone')
    ap.add_argument('--skip-commission', action='store_true')
    a = ap.parse_args()

    cfg = yaml.safe_load(open(os.path.join(CFG, 'safety_config.yaml')))
    prof = yaml.safe_load(open(os.path.join(CFG, 'robot_profile.yaml')))
    cad = json.load(open(os.path.join(CFG, 'cad_profile.json')))

    # 1. config integrity and human approval
    h = config_hash(cfg)
    check('safety config matches its lock hash', h == cfg.get('lock_sha256'), '%s vs %s' % (h, cfg.get('lock_sha256')))
    ap_path = os.path.join(CFG, 'safety_config.approved')
    appr = open(ap_path).read().split()[0] if os.path.exists(ap_path) else None
    check('safety config approved by a human (tools/approve_safety_config.py)', appr == h, 'approved=%s' % appr)

    # 2. physics consistency
    check('guard radius >= CAD circumradius', cfg['robot_radius_m'] >= cad['footprint_circumradius_m'] + 0.005,
          '%.3f vs %.3f' % (cfg['robot_radius_m'], cad['footprint_circumradius_m']))
    check('wheel centre radius equals CAD', abs(cfg['wheel_centre_radius_m'] - cad['wheel_centre_radius_m']) < 1e-3)
    wm = cfg['v_max_m_s'] + cfg['wheel_centre_radius_m'] * cfg['w_max_rad_s']
    check('v_max + R*w_max <= wheel cap', wm <= cfg['wheel_cap_eff_m_s'] + 1e-6, '%.4f vs %.4f' % (wm, cfg['wheel_cap_eff_m_s']))
    check('brake decel <= firmware decel', cfg['a_brake_m_s2'] <= prof['robot']['fw_decel_xy'])
    check('accel limit <= firmware accel', cfg['a_acc_m_s2'] <= prof['robot']['fw_accel_xy'] + 1e-9)
    need = cfg['gap_needed_at_vmax_m']
    check('full speed needs a gap shorter than the lidar range', need < cfg['lidar']['range_max_m'] / 2, '%.2f m' % need)
    check('tether radius fits inside the session', cfg['tether_radius_m'] > 0.5)

    # 3. firmware constants equal the config
    fw = os.path.join(REPO, 'control', 'motor_controller_xdrive_safe', 'motor_controller_xdrive_safe.ino')
    txt = open(fw).read()
    for name, key, tol in (('V_MAX_XY', 'v_max_m_s', 1e-6), ('W_MAX', 'w_max_rad_s', 1e-6),
                           ('ROBOT_CENTER_TO_WHEEL_RADIUS', 'wheel_centre_radius_m', 1e-6), ('WHEEL_RADIUS_M', None, 0)):
        v = define_value(txt, name)
        if key:
            check('firmware %s == config %s' % (name, key), v is not None and abs(v - cfg[key]) <= tol, '%s vs %s' % (v, cfg[key]))
    check('firmware accel/decel equal profile', define_value(txt, 'ACCEL_XY') == prof['robot']['fw_accel_xy'] and
          define_value(txt, 'DECEL_XY') == prof['robot']['fw_decel_xy'])
    check('firmware command timeout <= scan-stale budget', re.search(r'CMD_TIMEOUT_US = 250000', txt) is not None)

    # 4. Dashing parameter validity of the file that will be launched
    import dashing_keys as DK
    nav = yaml.safe_load(open(os.path.join(CFG, 'nav2_params.derived.yaml')))
    warns = [m for s, m in DK.check_nav2_yaml(nav) if s == 'warn']
    check('derived Nav2 params use only keys Dashing declares', not warns, '; '.join(warns[:3]), hard=False)

    # 5. runtime code compatible with Python 3.6 / Dashing
    for f in RUNTIME_FILES:
        p = os.path.normpath(os.path.join(ROOT, f))
        if os.path.exists(p):
            ok, d = py36_ok(p)
            check('python3.6-compatible: %s' % f, ok, d)

    # 7. only the gate may publish to the motors
    bridge = open(os.path.join(REPO, 'autonav', 'nav', 'src', 'main.py')).read()
    check('serial bridge listens to /cmd_vel_safe only', "'/cmd_vel_safe'" in bridge and "'/cmd_vel'," not in bridge)
    pub = [f for f in os.listdir(os.path.join(REPO, 'robotlab')) if f.endswith('.py') and
           re.search(r"create_publisher\([^)]*/cmd_vel", open(os.path.join(REPO, 'robotlab', f)).read())]
    check('robotlab never publishes a velocity command', not pub, str(pub))

    # 8. commissioning
    if not a.skip_commission:
        cp = os.path.join(CFG, 'commissioned.json')
        ok = os.path.exists(cp) and abs(json.load(open(cp))['lidar_yaw_rad'] - cfg['lidar']['frame_yaw_rad']) < 1e-6
        check('lidar orientation commissioned for this frame_yaw_rad (safety/commission.py)', ok)

    # 9. zone and scenarios
    if a.zone:
        import numpy as np
        z = dict(np.load(a.zone))
        ii, jj = np.nonzero(z['allowed'])
        xs = z['origin_x'] + (jj + .5) * z['resolution']
        ys = z['origin_y'] + (ii + .5) * z['resolution']
        seed = z['seed']
        far = float(np.hypot(xs - seed[0], ys - seed[1]).max())
        check('tether radius encloses the whole zone from the seed point', cfg['tether_radius_m'] >= far + 0.3,
              'tether %.1f m, farthest zone point %.1f m' % (cfg['tether_radius_m'], far))
    width = max(len(r[0]) for r in results)
    fails = 0
    for name, ok, detail, hard in results:
        tag = 'PASS' if ok else ('FAIL' if hard else 'WARN')
        fails += (not ok) and hard
        print('%s  %-*s %s' % (tag, width, name, ('' if ok else '  <- ' + detail)))
    print('\n%d checks, %d hard failures' % (len(results), fails))
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
