#!/usr/bin/env python3
"""Derive the safety limits and the physics-determined Nav2 parameters from CAD + hardware numbers.

Inputs : config/cad_profile.json   (tools/extract_cad_profile.py)   config/robot_profile.yaml (hand inputs)
         autonav/nav/params/nav2_params.yaml (the user's file: only the keys below are touched, the rest is kept)
Outputs: config/safety_config.yaml     limits the gate / firmware enforce (hash-locked, human approved)
         config/nav2_params.derived.yaml   user's file + derived keys
         config/derived_report.md      every number with the formula that produced it

Nothing here is tuned to a particular environment: these are the quantities that follow from size, motors, lidar,
compute.  Environment-dependent parameters (critic weights, inflation shape, AMCL noise ...) are what tuning/ searches.
"""
import argparse, copy, hashlib, json, math, os
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, '..', 'config')
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))


def stop_distance(v, a_brake, t_lat):
    """Distance covered from the moment an obstacle appears until the robot is at rest."""
    return v * t_lat + v * v / (2.0 * a_brake)


def derive(cad, prof):
    r, L, c, pol, E = prof['robot'], prof['lidar'], prof['compute'], prof['policy'], prof['environment']
    out, why = {}, {}

    def put(key, val, text):
        out[key] = val
        why[key] = text

    R_c = cad['footprint_circumradius_m']
    pad = 0.01
    put('robot_radius_m', round(R_c + pad, 4),
        'CAD circumradius %.3f (dome wall) + %.2f pad; a circle is exact for an omni robot' % (R_c, pad))

    steps_per_m = r['steps_per_rev'] / (2 * math.pi * r['wheel_radius_m'])
    cap = r['wheel_steps_per_sec_max'] / steps_per_m
    put('wheel_speed_cap_m_s', round(cap, 4), 'wheel_steps_per_sec_max / (steps_per_rev / (2 pi r))')
    Rw = cad['wheel_centre_radius_m']
    put('wheel_centre_radius_m', Rw, 'CAD wheel-centre plane radius (the v13-era firmware had 0.151)')
    w_max = pol['w_max_rad_s']
    put('w_max_rad_s', w_max, 'policy')
    v_max = pol['wheel_cap_fraction'] * cap - Rw * w_max
    v_max = min(math.floor(v_max * 100) / 100.0, pol.get('v_max_cap_m_s', v_max))
    put('v_max_m_s', v_max, 'wheel_cap_fraction*cap - R_wheel*w_max: the fastest wheel runs at |v| + R*w along its drive direction')

    a_fw = min(r['fw_decel_xy'], r['usable_accel_ms2'])
    a_brake = round(pol['brake_decel_fraction'] * a_fw, 3)
    put('a_brake_m_s2', a_brake, 'brake_decel_fraction * min(firmware decel, usable accel)')
    put('a_acc_m_s2', min(r['fw_accel_xy'], r['usable_accel_ms2']), 'firmware ACCEL_XY')
    put('fw_decel_xy_m_s2', r['fw_decel_xy'], 'firmware DECEL_XY (largest decel_lim a planner may assume)')
    put('a_brake_w_rad_s2', round(pol['brake_decel_fraction'] * r['fw_decel_w'], 3), 'policy * firmware')
    put('a_acc_w_rad_s2', r['fw_accel_w'], 'firmware ACCEL_W')

    scan_time = 1.0 / L['scan_hz_min']
    t_fixed = 0.05 + 0.02 + 0.05
    put('scan_time_s', round(scan_time, 3), '1 / slowest scan rate (points in one scan are up to this old)')
    put('latency_fixed_s', t_fixed, 'gate 20 Hz period + bridge hop + firmware command period/loop')
    d_stop = stop_distance(v_max, a_brake, t_fixed + scan_time + 0.05)
    put('stop_distance_at_vmax_m', round(d_stop, 4),
        'v*t + v^2/(2a), t = fixed + scan_time + 0.05 s typical scan transport age')
    put('lateral_margin_m', pol['lateral_margin_m'], 'policy: extra half-width of the swept corridor')
    put('stop_margin_m', pol['stop_margin_m'], 'policy')
    put('dyn_obstacle_speed_m_s', pol['dyn_obstacle_speed_m_s'], 'policy')
    need = d_stop + pol['stop_margin_m'] + pol['dyn_obstacle_speed_m_s'] * (t_fixed + scan_time + 0.05 + v_max / a_brake)
    put('gap_needed_at_vmax_m', round(need, 3), 'free distance beyond the robot circle needed to run at v_max')

    out['wheel_azimuths_deg'] = r.get('wheel_azimuths_deg', [45.0, 135.0, 225.0, 315.0])
    out['wheel_cap_eff_m_s'] = round(pol['wheel_cap_fraction'] * cap, 4)
    out['lidar'] = {
        'frame_yaw_rad': L['frame_yaw_rad'],
        'axis_offset_xy_m': cad['lidar_axis_offset_xy_m'],
        'range_min_m': L['range_min_m'],
        'range_max_m': L['range_max_use_m'],
        'self_return_radius_m': 0.10,
        'blind_wedges_deg': [[a, 2.0] for a in cad['lidar_cap_blade_azimuths_deg']],
        'scan_plane_height_m': cad['scan_plane_height_m'],
    }
    why['lidar.blind_wedges_deg'] = ('cap blade azimuths from CAD, half-width 2 deg >= %.2f deg shadow + A1 beam width'
                                     % cad['lidar_blade_shadow_halfwidth_deg'])
    for k in ('unknown_creep_m_s', 'blind_wedge_speed_cap_m_s', 'tether_radius_m', 'tether_hard_extra_m', 'max_session_s',
              'heartbeat_timeout_s', 'scan_stale_s', 'cmd_stale_s', 'odom_stale_s', 'amcl_travel_stale_m', 'amcl_cov_limit_m'):
        out[k] = pol[k]

    # ---------------- Nav2 (Dashing parameter names) ----------------
    nav = {}
    nav['robot_radius'] = out['robot_radius_m']
    d_half = max(0.08, d_stop)
    k_infl = round(math.log(2.0) / d_half, 2)
    why['nav.inflation_cost_scaling_factor'] = ('ln2 / stop_distance: the cost falls to 50 %% of lethal one stopping distance '
                                                'outside the robot circle -> %.2f' % k_infl)
    nav['local_inflation_radius'] = round(out['robot_radius_m'] + 2.0 * d_half + 0.05, 2)
    nav['global_inflation_radius'] = round(out['robot_radius_m'] + max(0.30, 2.5 * d_half), 2)
    why['nav.inflation_radius'] = ('robot radius + ~2 stopping distances (local) / + max(0.30, 2.5) stops (global); a %.2f m door '
                                   'leaves %.2f m of soft cost, not lethal' % (E['min_door_width_m'],
                                                                              E['min_door_width_m'] / 2 - out['robot_radius_m']))
    nav['inflation_cost_scaling_factor'] = k_infl
    nav['max_vel'] = v_max
    nav['max_vel_theta'] = w_max
    nav['acc_lim'] = out['a_acc_m_s2']
    nav['decel_lim'] = -r['fw_decel_xy']
    nav['acc_lim_theta'] = r['fw_accel_w']
    nav['decel_lim_theta'] = -r['fw_decel_w']
    ctrl_hz = c['controller_hz_max']
    nav['controller_frequency'] = ctrl_hz
    sim_time = (d_stop + 2 * out['robot_radius_m']) / v_max
    nav['sim_time'] = round(min(3.0, max(1.5, sim_time)) * 10) / 10.0
    why['nav.sim_time'] = ('(stopping distance + 2 robot radii) / v_max: DWB must look far enough to stop and to judge a gap; '
                           'clamped to [1.5, 3] s')
    budget_us = 0.55 * 1e6 / ctrl_hz
    steps = max(10, int(v_max * nav['sim_time'] / 0.03))
    n_critics = 5
    n_traj_max = int(budget_us / (c['dwb_pose_eval_us'] * steps * n_critics))
    vx = vy = max(5, min(12, int(round(n_traj_max ** (1 / 3.0) * 1.6))))
    vth = max(3, min(7, vx // 2))
    while vx * vy * vth > n_traj_max and vx > 5:
        vx -= 1
        vy -= 1
    nav['vx_samples'], nav['vy_samples'], nav['vtheta_samples'] = vx, vy, vth
    why['nav.samples'] = ('trajectory count limited so ~%d pose evaluations x %.1f us stay inside 55 %% of one control period '
                          '(%.0f ms)' % (vx * vy * vth * steps * n_critics, c['dwb_pose_eval_us'], budget_us / 1000))
    nav['local_costmap_size'] = float(math.ceil(2 * (v_max * nav['sim_time'] + 2 * out['robot_radius_m'] + 0.5)))
    nav['local_update_frequency'] = max(5.0, ctrl_hz / 2)
    nav['obstacle_range'] = 3.0
    nav['raytrace_range'] = 3.5
    nav['amcl_laser_min_range'] = L['range_min_m']
    nav['amcl_laser_max_range'] = L['range_max_use_m']
    nav['amcl_update_min_d'] = 0.05
    why['nav.amcl_update_min_d'] = ('>= distance travelled per scan at v_max (%.3f m); finer only costs CPU'
                                    % (v_max / L['scan_hz_typ']))
    nav['amcl_transform_tolerance'] = 0.5
    max_beams = 60
    nav['amcl_max_beams'] = max_beams
    nav['amcl_max_particles'] = int(min(800, max(200, c['amcl_beam_particles_per_s'] / 5.0 / max_beams)))
    nav['amcl_min_particles'] = max(100, nav['amcl_max_particles'] // 3)
    out['nav'] = nav
    out['why'] = why
    return out


def apply_to_nav2_yaml(base, d):
    y = copy.deepcopy(base)
    n = d['nav']
    dwb = y['dwb_controller']['ros__parameters']
    dwb.update({'min_vel_x': -n['max_vel'], 'max_vel_x': n['max_vel'], 'min_vel_y': -n['max_vel'], 'max_vel_y': n['max_vel'],
                'max_speed_xy': n['max_vel'], 'max_vel_theta': n['max_vel_theta'],
                'acc_lim_x': n['acc_lim'], 'acc_lim_y': n['acc_lim'], 'acc_lim_theta': n['acc_lim_theta'],
                'decel_lim_x': n['decel_lim'], 'decel_lim_y': n['decel_lim'], 'decel_lim_theta': n['decel_lim_theta'],
                'sim_time': n['sim_time'], 'vx_samples': n['vx_samples'], 'vy_samples': n['vy_samples'],
                'vtheta_samples': n['vtheta_samples'], 'controller_frequency': n['controller_frequency'],
                # Dashing's OdomSubscriber subscribes to the topic "odom" unless `odom_topic` is set.  This stack publishes
                # /odometry/raw and the EKF's /odometry/filtered, so without this key DWB sees zero velocity.
                # Confirm on the robot with `ros2 topic info /odom` before trusting this line.
                'odom_topic': '/odometry/filtered'})
    for costmap, infl, upd in (('local_costmap', n['local_inflation_radius'], n['local_update_frequency']),
                               ('global_costmap', n['global_inflation_radius'], None)):
        cm = y[costmap][costmap]['ros__parameters']
        cm['robot_radius'] = n['robot_radius']
        cm['inflation_layer'] = {'inflation_radius': infl, 'cost_scaling_factor': n['inflation_cost_scaling_factor']}
        if upd:
            cm['update_frequency'] = upd
            cm['width'] = int(n['local_costmap_size'])
            cm['height'] = int(n['local_costmap_size'])
    y['local_costmap']['local_costmap']['ros__parameters']['scan'].update(
        {'obstacle_range': n['obstacle_range'], 'raytrace_range': n['raytrace_range']})
    # Dashing's obstacle layer declares obstacle_range / raytrace_range; the user's file had obstacle_max_range /
    # raytrace_max_range for the global map, which no Dashing node reads (so it ran with the 2.5 m / 3.0 m defaults).
    g = y['global_costmap']['global_costmap']['ros__parameters']['scan']
    g.pop('obstacle_max_range', None)
    g.pop('raytrace_max_range', None)
    g.update({'obstacle_range': 5.0, 'raytrace_range': 5.5})
    # Dashing reads oscillation_reset_* under their plain names; the dotted `Oscillation.` spelling is ignored.
    for k in ('oscillation_reset_dist', 'oscillation_reset_angle'):
        dwb[k] = dwb.pop('Oscillation.' + k, dwb.get(k, 0.05 if k.endswith('dist') else 0.2))
    y['amcl']['ros__parameters'].update(
        {'laser_min_range': n['amcl_laser_min_range'], 'laser_max_range': n['amcl_laser_max_range'],
         'update_min_d': n['amcl_update_min_d'], 'transform_tolerance': n['amcl_transform_tolerance'],
         'max_beams': n['amcl_max_beams'], 'max_particles': n['amcl_max_particles'],
         'min_particles': n['amcl_min_particles']})
    return y


def config_hash(d):
    keep = {k: v for k, v in d.items() if k not in ('why', 'nav', 'lock_sha256')}
    return hashlib.sha256(json.dumps(keep, sort_keys=True).encode()).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nav2', default=os.path.join(ROOT, 'autonav', 'nav', 'params', 'nav2_params.yaml'))
    ap.add_argument('--zone', help='zone.npz: the tether radius is set to enclose the whole zone from its seed point')
    a = ap.parse_args()
    cad = json.load(open(os.path.join(CFG, 'cad_profile.json')))
    prof = yaml.safe_load(open(os.path.join(CFG, 'robot_profile.yaml')))
    if a.zone:
        import numpy as np
        z = np.load(a.zone)
        ii, jj = np.nonzero(z['allowed'])
        xs = float(z['origin_x']) + (jj + .5) * float(z['resolution'])
        ys = float(z['origin_y']) + (ii + .5) * float(z['resolution'])
        far = float(np.hypot(xs - z['seed'][0], ys - z['seed'][1]).max())
        prof['policy']['tether_radius_m'] = math.ceil((far + 0.5) * 10) / 10.0
        print('tether radius set to %.1f m (farthest zone point %.1f m from the seed): ARM THE ROBOT AT THE SEED POINT' % (
            prof['policy']['tether_radius_m'], far))
    d = derive(cad, prof)
    safety = {k: v for k, v in d.items() if k not in ('nav', 'why')}
    safety['lock_sha256'] = config_hash(safety)
    yaml.safe_dump(safety, open(os.path.join(CFG, 'safety_config.yaml'), 'w'), sort_keys=True)
    base = yaml.safe_load(open(a.nav2))
    yaml.safe_dump(apply_to_nav2_yaml(base, d), open(os.path.join(CFG, 'nav2_params.derived.yaml'), 'w'), sort_keys=False)
    lines = ['# Derived parameters (generated by tools/derive_params.py)', '']
    for k, v in d.items():
        if k in ('why', 'nav', 'lidar'):
            continue
        lines.append('- `%s` = %s%s' % (k, v, ('  - ' + d['why'][k]) if k in d['why'] else ''))
    lines += ['', '## Nav2 (Dashing key names)'] + ['- `%s` = %s' % (k, v) for k, v in d['nav'].items()]
    lines += ['', '## Reasons'] + ['- `%s`: %s' % (k, v) for k, v in d['why'].items()]
    lines += ['', 'safety_config lock: `%s`' % safety['lock_sha256']]
    open(os.path.join(CFG, 'derived_report.md'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
