#!/usr/bin/env python3
"""Records the IMU and wheel odometry on the robot, then prints bias, noise and gyro scale against wheel yaw.
  imu_check.py static --secs 60    robot still, motors energised, do not touch it
  imu_check.py spin --secs 30      keep still 3 s, rotate with the cmdvel GUI (gate on), then keep still 3 s"""
import argparse
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['static', 'spin'])
    ap.add_argument('--secs', type=float, default=60.0)
    ap.add_argument('--imu', default='/camera/imu')
    ap.add_argument('--odom', default='/odometry/raw')
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('imu_check')
    imu, od = [], []
    n.create_subscription(Imu, a.imu, lambda m: imu.append(
        [time.monotonic(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
         m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]), qos_profile_sensor_data)
    n.create_subscription(Odometry, a.odom, lambda m: od.append(
        [time.monotonic(), yaw_of(m.pose.pose.orientation), m.twist.twist.angular.z]), qos_profile_sensor_data)
    end = time.monotonic() + a.secs
    print('recording %.0f s' % a.secs)
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    if len(imu) < 50:
        raise SystemExit('only %d IMU samples on %s' % (len(imu), a.imu))
    g = np.array(imu)
    t = g[:, 0] - g[0, 0]
    w, acc = g[:, 1:4], g[:, 4:7]
    dt = np.concatenate([[0.0], np.diff(t)])
    ax = 'xyz'
    print('imu samples %d, rate %.1f Hz' % (len(t), len(t) / t[-1]))
    if a.mode == 'static':
        h = len(w) // 2
        print('gyro bias rad/s', np.round(w.mean(0), 5), ' deg/min', np.round(np.degrees(w.mean(0)) * 60, 2))
        print('gyro noise std rad/s', np.round(w.std(0), 5))
        print('bias change first->second half rad/s', np.round(w[h:].mean(0) - w[:h].mean(0), 5))
        print('accel mean m/s2', np.round(acc.mean(0), 3), ' norm %.3f (9.81 expected)' % np.linalg.norm(acc.mean(0)))
        print('accel noise std m/s2', np.round(acc.std(0), 4))
        return
    o = np.array(od)
    if len(o) < 10:
        raise SystemExit('only %d odometry samples' % len(o))
    k = t < 3.0
    kt = t > t[-1] - 3.0
    bias = np.concatenate([w[k], w[kt]]).mean(0)
    print('still-phase gyro std rad/s', np.round(np.concatenate([w[k], w[kt]]).std(0), 5))
    ang = ((w - bias) * dt[:, None]).sum(0)
    yaw = np.unwrap(o[:, 1])
    wheel = yaw[-1] - yaw[0]
    print('gyro integral deg', dict(zip(ax, np.round(np.degrees(ang), 2))), ' wheel yaw change deg %.2f' % math.degrees(wheel))
    i = int(np.argmax(np.abs(ang)))
    print('yaw axis = %s, gyro/wheel scale %.4f (sign %s)' % (ax[i], ang[i] / wheel if wheel else float('nan'),
                                                          '+' if ang[i] * wheel > 0 else '-'))
    wt = np.interp(t, o[:, 0] - g[0, 0], o[:, 2])
    r = w[:, i] - bias[i]
    print('rate rms diff gyro-wheel %.4f rad/s, correlation %.3f' % (np.sqrt(np.mean((r - wt) ** 2)), np.corrcoef(r, wt)[0, 1]))
    mid = ~k & ~kt
    print('accel std still %s vs moving %s m/s2' % (np.round(acc[k].std(0), 3), np.round(acc[mid].std(0), 3)))


if __name__ == '__main__':
    main()
