"""Static check of a Nav2 params dict against the safety config.  Used on the robot by nav2_manager.py (it refuses to start
Nav2 with parameters that exceed what the robot may do) and on the PC before a request is sent.  Pure python."""


def check_limits(y, safety):
    """Return a list of problems (empty = acceptable)."""
    bad = []
    try:
        d = y['dwb_controller']['ros__parameters']
    except Exception:
        return ['no dwb_controller block']
    v, w = safety['v_max_m_s'], safety['w_max_rad_s']
    for k in ('max_speed_xy', 'max_vel_x', 'max_vel_y'):
        if k in d and abs(float(d[k])) > v + 1e-6:
            bad.append('%s=%s exceeds v_max %.3f' % (k, d[k], v))
    for k in ('min_vel_x', 'min_vel_y'):
        if k in d and abs(float(d[k])) > v + 1e-6:
            bad.append('%s=%s exceeds v_max %.3f' % (k, d[k], v))
    if 'max_vel_theta' in d and abs(float(d['max_vel_theta'])) > w + 1e-6:
        bad.append('max_vel_theta=%s exceeds w_max %.3f' % (d['max_vel_theta'], w))
    for k in ('acc_lim_x', 'acc_lim_y'):
        if k in d and abs(float(d[k])) > safety['a_acc_m_s2'] + 1e-6:
            bad.append('%s=%s exceeds firmware accel %.2f' % (k, d[k], safety['a_acc_m_s2']))
    for k in ('decel_lim_x', 'decel_lim_y'):
        if k in d and abs(float(d[k])) > safety.get('fw_decel_xy_m_s2', 0.55) + 1e-6:
            bad.append('%s=%s exceeds firmware decel' % (k, d[k]))
    if 'acc_lim_theta' in d and abs(float(d['acc_lim_theta'])) > safety['a_acc_w_rad_s2'] + 1e-6:
        bad.append('acc_lim_theta exceeds firmware')
    if 'controller_frequency' in d and not (1.0 <= float(d['controller_frequency']) <= 20.0):
        bad.append('controller_frequency outside 1..20 Hz')
    for blk in ('local_costmap', 'global_costmap'):
        try:
            r = float(y[blk][blk]['ros__parameters']['robot_radius'])
        except Exception:
            continue
        if r < safety['robot_radius_m'] - 1e-6:
            bad.append('%s robot_radius %.3f below the guard radius %.3f' % (blk, r, safety['robot_radius_m']))
    return bad
