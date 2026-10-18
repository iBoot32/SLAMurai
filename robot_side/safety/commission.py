#!/usr/bin/env python3
"""HUMAN commissioning of the one thing the safety layer cannot check by itself: that the lidar's angle zero points
where safety_config.yaml says it does (frame_yaw_rad).  If this is wrong by 90 deg the gate watches the wrong side of
the robot, so the gate refuses to be approved until this check has passed (tools/preflight.py looks for the marker).

Procedure (robot on the floor, motors NOT powered or wheels off the ground):
  1. start the lidar only   2. python3 commission.py
  3. put a box/book ~0.4 m from the robot centre IN FRONT (the side with the camera) when asked, then LEFT (front fixes the rotation, left fixes the handedness).
The script reads /scan, applies frame_yaw_rad, and checks that the nearest return is in the sector you were asked for.
On success it writes config/commissioned.json {yaw, utc, user}.  Also asks you to confirm by hand:
  * wheel directions: command +x in cmdvel_gui with the robot lifted - every wheel must drive the robot forward
  * the 360 deg spin calibration of ROBOT_CENTER_TO_WHEEL_RADIUS was done (firmware comment says to trim it)
  * the real robot mass is within 20 % of robot_profile.yaml
"""
import json
import math
import os
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, '..', 'config')


def nearest_sector(ranges, angle_min, inc, yaw, offset_xy=(0.0, 0.0), rmin=0.15, rmax=2.0):
    """Base-frame azimuth (deg, 0 = front, +90 = left) of the nearest valid return, or None."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + inc * np.arange(len(r)) + yaw
    ok = np.isfinite(r) & (r >= rmin) & (r <= rmax)
    if not ok.any():
        return None, None
    x = r[ok] * np.cos(a[ok]) + offset_xy[0]
    y = r[ok] * np.sin(a[ok]) + offset_xy[1]
    d = np.hypot(x, y)
    k = int(np.argmin(d))
    return math.degrees(math.atan2(y[k], x[k])), float(d[k])


def check(expected_deg, ranges, angle_min, inc, yaw, tol_deg=30.0):
    az, d = nearest_sector(ranges, angle_min, inc, yaw)
    if az is None:
        return False, 'no return between 0.15 and 2 m'
    diff = abs((az - expected_deg + 180.0) % 360.0 - 180.0)
    return diff <= tol_deg, 'nearest return at %.0f deg (%.2f m), expected %.0f deg' % (az, d, expected_deg)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    cfg = yaml.safe_load(open(os.path.join(CFG, 'safety_config.yaml')))
    yaw = cfg['lidar']['frame_yaw_rad']
    rclpy.init()
    node = Node('commission')
    last = {}
    node.create_subscription(LaserScan, '/scan', lambda m: last.update(m=m), qos_profile_sensor_data)

    def newest():
        last.clear()
        t0 = time.time()
        while 'm' not in last and time.time() - t0 < 5:
            rclpy.spin_once(node, timeout_sec=0.2)
        return last.get('m')

    passed = True
    for name, deg in (('IN FRONT (camera side)', 0.0), ('on the LEFT (robot facing the camera side, left hand)', 90.0)):
        input('Put a box 0.2 to 1 m from the robot centre %s, clear everything else within 2 m, press Enter: ' % name)
        m = newest()
        if m is None:
            print('no /scan received')
            sys.exit(1)
        ok, msg = check(deg, m.ranges, m.angle_min, m.angle_increment, yaw)
        print(('PASS  ' if ok else 'FAIL  ') + msg)
        passed &= ok
    for q in ('Every wheel drives the robot forward for +x (tested lifted)? [y/N] ',
              'The 360 degree spin calibration of ROBOT_CENTER_TO_WHEEL_RADIUS was done? [y/N] ',
              'The robot weighs within 20%% of %.1f kg? [y/N] ' % 5.0):
        passed &= input(q).strip().lower() == 'y'
    if not passed:
        sys.exit('commissioning FAILED: fix the cause (e.g. lidar_joint yaw in the URDF / frame_yaw_rad) and rerun')
    json.dump({'lidar_yaw_rad': yaw, 'utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
               'user': os.environ.get('USER', '?')}, open(os.path.join(CFG, 'commissioned.json'), 'w'))
    print('written config/commissioned.json')


if __name__ == '__main__':
    main()
