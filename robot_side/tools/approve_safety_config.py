#!/usr/bin/env python3
"""HUMAN step.  Prints the safety limits in plain words, asks for confirmation, then writes
config/safety_config.approved containing the config hash.  The gate refuses to start with any other config.
The tuning agent must not run this (it is not exposed in the agent CLI)."""
import hashlib
import json
import os
import sys
import time
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, '..', 'config')


def config_hash(cfg):
    keep = {k: v for k, v in cfg.items() if k not in ('why', 'nav', 'lock_sha256')}
    return hashlib.sha256(json.dumps(keep, sort_keys=True).encode()).hexdigest()[:16]


def main():
    cfg = yaml.safe_load(open(os.path.join(CFG, 'safety_config.yaml')))
    h = config_hash(cfg)
    if h != cfg.get('lock_sha256'):
        sys.exit('config hash does not match its lock: regenerate with tools/derive_params.py')
    print('Safety limits to approve (hash %s):' % h)
    print('  robot guard circle      %.3f m (+ %.2f m sideways margin), stop margin %.2f m' % (
        cfg['robot_radius_m'], cfg['lateral_margin_m'], cfg['stop_margin_m']))
    print('  speed limits            %.2f m/s, %.2f rad/s, wheel rim <= %.3f m/s' % (
        cfg['v_max_m_s'], cfg['w_max_rad_s'], cfg['wheel_cap_eff_m_s']))
    print('  assumed braking         %.3f m/s^2, fixed latency %.2f s, scan time %.2f s' % (
        cfg['a_brake_m_s2'], cfg['latency_fixed_s'], cfg['scan_time_s']))
    print('  tether from arm point   %s, session max %d s' % (
        'none' if cfg['tether_radius_m'] <= 0 else '%.1f m (hard stop +%.2f m)' % (cfg['tether_radius_m'], cfg['tether_hard_extra_m']), cfg['max_session_s']))
    print('  lidar yaw in base frame %.4f rad  (MUST have been checked with commission.py)' % cfg['lidar']['frame_yaw_rad'])
    if '--yes' not in sys.argv:
        if input('Type APPROVE to accept these limits: ').strip() != 'APPROVE':
            sys.exit('not approved')
    open(os.path.join(CFG, 'safety_config.approved'), 'w').write('%s  approved %s by %s\n' % (
        h, time.strftime('%Y-%m-%d %H:%M:%S'), os.environ.get('USERNAME') or os.environ.get('USER') or '?'))
    print('written config/safety_config.approved')


if __name__ == '__main__':
    main()
