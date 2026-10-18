#!/usr/bin/env python3
"""Batches what robotlab records into one JSON message, so rosbridge carries it.
Dashing's rosbridge handles one incoming message per 100 ms in total, so subscribing to odometry, TF and commands through it
gives each stream ~2 Hz. This node samples locally at --hz and publishes every --batch samples on /robotlab/telemetry.
Sample: [t, x, y, yaw | null (map pose), vx, vy, w, cmd_in vx, vy, w, cmd_out vx, vy, w].
Message: {"s": [samples], "cmd_t": [cmd_in arrival times], "status": last gate status, "plan": [[x, y], ...] if new, "log": [texts]}."""
import argparse
import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Telemetry(Node):
    def __init__(self, a):
        super().__init__('robotlab_telemetry')
        self.a = a
        self.keys = [(a.map_frame, a.odom_frame), (a.odom_frame, a.base_frame)]
        self.vel = self.cin = self.cout = (0.0, 0.0, 0.0)
        self.samples, self.cmd_t, self.log = [], [], []
        self.status = self.plan = None
        self.tf = {}                                       # (parent, child) -> (x, y, yaw); Dashing has no Python tf2_ros
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)
        self.create_subscription(Odometry, a.odom, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, a.cmd_in, self.on_cin, 10)
        self.create_subscription(Twist, a.cmd_out, self.on_cout, 10)
        self.create_subscription(String, a.status, lambda m: setattr(self, 'status', m.data), 10)
        self.create_subscription(Path, a.plan, self.on_plan, 1)
        self.create_subscription(Log, '/rosout', lambda m: self.log.append(m.msg[:200]), 50)
        self.pub = self.create_publisher(String, '/robotlab/telemetry', 10)
        self.create_timer(1.0 / a.hz, self.on_timer)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tf(self, m):
        for t in m.transforms:
            k = (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
            if k in self.keys:
                r = t.transform
                self.tf[k] = (r.translation.x, r.translation.y, yaw_of(r.rotation))

    def map_pose(self):
        mo, ob = self.tf.get(self.keys[0]), self.tf.get(self.keys[1])
        if mo is None or ob is None:
            return [None, None, None]
        c, s = math.cos(mo[2]), math.sin(mo[2])
        yaw = math.atan2(math.sin(mo[2] + ob[2]), math.cos(mo[2] + ob[2]))
        return [round(mo[0] + c * ob[0] - s * ob[1], 4), round(mo[1] + s * ob[0] + c * ob[1], 4), round(yaw, 4)]

    def on_odom(self, m):
        t = m.twist.twist
        self.vel = (t.linear.x, t.linear.y, t.angular.z)

    def on_cin(self, m):
        self.cin = (m.linear.x, m.linear.y, m.angular.z)
        self.cmd_t.append(round(self.now(), 3))

    def on_cout(self, m):
        self.cout = (m.linear.x, m.linear.y, m.angular.z)

    def on_plan(self, m):
        p = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
        self.plan = [[round(x, 3), round(y, 3)] for x, y in p[::max(1, len(p) // 200)]]

    def on_timer(self):
        pose = self.map_pose()
        self.samples.append([round(self.now(), 3)] + pose + [round(v, 4) for v in self.vel + self.cin + self.cout])
        if len(self.samples) < self.a.batch:
            return
        msg = {'s': self.samples, 'cmd_t': self.cmd_t, 'status': json.loads(self.status) if self.status else None}
        if self.plan is not None:
            msg['plan'] = self.plan
        if self.log:
            msg['log'] = self.log
        self.samples, self.cmd_t, self.log, self.plan = [], [], [], None
        self.pub.publish(String(data=json.dumps(msg)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--odom', default='/odometry/raw')
    ap.add_argument('--cmd-in', default='/cmd_vel')
    ap.add_argument('--cmd-out', default='/cmd_vel_safe')
    ap.add_argument('--status', default='/safety/status')
    ap.add_argument('--plan', default='/plan')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--hz', type=float, default=20.0)
    ap.add_argument('--batch', type=int, default=4)
    a, _ = ap.parse_known_args()
    rclpy.init()
    rclpy.spin(Telemetry(a))


if __name__ == '__main__':
    main()
