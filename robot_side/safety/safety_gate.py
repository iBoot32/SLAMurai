#!/usr/bin/env python3
"""ROS 2 (Dashing) wrapper around safety_core.SafetyCore.  This node is the ONLY publisher of /cmd_vel_safe, and the
serial bridge listens to /cmd_vel_safe and nothing else, so everything that can move the robot passes through here.

  in : /cmd_vel (Nav2 / teleop / agent, untrusted)   /scan   /odometry/raw (+ /fw_flags)   /amcl_pose
       /safety/operator (JSON, arm/clear need the operator token)   /safety/heartbeat (dead-man, from the trial runner)
  out: /cmd_vel_safe   /safety/status (JSON, 10 Hz)

Start:  python3 safety_gate.py --config safety_config.yaml [--zone zone.npz] [--token-file /root/.slamurai_token]
It refuses to start unless safety_config.yaml hashes to the value in safety_config.approved (written by a human with
tools/approve_safety_config.py), so the limits cannot be loosened by editing the file at runtime.
Everything is stamped with the local monotonic clock at reception: no dependence on message stamps or clock sync.
"""
import argparse
import hashlib
import json
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from safety_core import SafetyCore, Zone      # noqa: E402

import numpy as np                            # noqa: E402
import rclpy                                  # noqa: E402
from rclpy.node import Node                   # noqa: E402
from rclpy.qos import QoSProfile, qos_profile_sensor_data   # noqa: E402
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped   # noqa: E402
from nav_msgs.msg import Odometry             # noqa: E402
from sensor_msgs.msg import LaserScan         # noqa: E402
from std_msgs.msg import String, Int32        # noqa: E402

GATE_HZ = 20.0


def config_hash(cfg):
    keep = {k: v for k, v in cfg.items() if k not in ('why', 'nav', 'lock_sha256')}
    return hashlib.sha256(json.dumps(keep, sort_keys=True).encode()).hexdigest()[:16]


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_config(path, approved_path):
    cfg = yaml.safe_load(open(path))
    h = config_hash(cfg)
    if cfg.get('lock_sha256') != h:
        raise SystemExit('safety config does not match its own lock hash (edited by hand?): %s vs %s' % (cfg.get('lock_sha256'), h))
    if approved_path:
        try:
            ap = open(approved_path).read().split()[0]
        except Exception:
            raise SystemExit('no approval file %s: run tools/approve_safety_config.py after reviewing the config' % approved_path)
        if ap != h:
            raise SystemExit('safety config %s is not the approved one (%s)' % (h, ap))
    return cfg, h


class SafetyGate(Node):
    def __init__(self, cfg, cfg_hash, zone, token_hash, log_path):
        super().__init__('safety_gate')
        self.core = SafetyCore(cfg, zone)
        self.cfg_hash = cfg_hash
        self.token_hash = token_hash
        self.log_path = log_path
        self.fw_flags = 0
        self.last_state = None
        self.last_faults = []
        self.pub = self.create_publisher(Twist, '/cmd_vel_safe', 5)
        self.pub_status = self.create_publisher(String, '/safety/status', 5)
        # best-effort subscriptions connect to both best-effort and reliable publishers (Nav2, teleop GUI, EKF bridge)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/raw', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Int32, '/fw_flags', self.on_fw, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, qos_profile_sensor_data)
        self.create_subscription(String, '/safety/operator', self.on_operator, QoSProfile(depth=10))
        self.create_subscription(String, '/safety/heartbeat', self.on_heartbeat, QoSProfile(depth=10))
        self.create_timer(1.0 / GATE_HZ, self.on_timer)
        self.status_div = 0
        self.log('gate started, config %s' % cfg_hash)

    # --- plumbing
    def now(self):
        return time.monotonic()

    def log(self, text):
        self.get_logger().info(text)
        if self.log_path:
            try:
                with open(self.log_path, 'a') as f:
                    f.write(json.dumps({'wall': time.time(), 'msg': text}) + '\n')
            except Exception:
                pass

    def on_cmd(self, m):
        self.core.on_cmd(self.now(), m.linear.x, m.linear.y, m.angular.z)

    def on_scan(self, m):
        self.core.on_scan(self.now(), m.angle_min, m.angle_increment, np.asarray(m.ranges, dtype=float))

    def on_fw(self, m):
        self.fw_flags = int(m.data)

    def on_odom(self, m):
        p, t = m.pose.pose, m.twist.twist
        self.core.on_odom(self.now(), p.position.x, p.position.y, float(yaw_of(p.orientation)),
                          t.linear.x, t.linear.y, t.angular.z, self.fw_flags)

    def on_amcl(self, m):
        p = m.pose.pose
        c = m.pose.covariance
        self.core.on_map_pose(self.now(), p.position.x, p.position.y, float(yaw_of(p.orientation)), c[0], c[7])

    def on_heartbeat(self, m):
        self.core.heartbeat(self.now())

    def on_operator(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        cmd = d.get('cmd')
        now = self.now()
        if cmd == 'disarm':
            self.core.disarm(now)
            self.log('disarmed by operator message')
            return
        if cmd in ('arm', 'clear'):
            if self.token_hash and hashlib.sha256(str(d.get('token', '')).encode()).hexdigest() != self.token_hash:
                self.log('REJECTED %s: bad token' % cmd)
                return
            if cmd == 'clear':
                self.core.clear_faults(now)
                self.log('faults cleared by operator')
            else:
                ok, msg = self.core.arm(now, float(d.get('lease_s', 600.0)))
                self.log('arm %s: %s' % ('OK' if ok else 'REFUSED', msg))

    # --- 20 Hz output
    def on_timer(self):
        out, info = self.core.step(self.now())
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = float(out[0]), float(out[1]), float(out[2])
        self.pub.publish(tw)
        st = info.get('state')
        if st != self.last_state or info.get('faults') != self.last_faults:
            self.log('state %s faults %s limit %s' % (st, info.get('faults'), info.get('limit')))
            self.last_state, self.last_faults = st, list(info.get('faults', []))
        self.status_div += 1
        if self.status_div >= 2:
            self.status_div = 0
            core = self.core
            s = {k: (None if isinstance(v, float) and (v != v or v in (float('inf'), float('-inf'))) else v)
                 for k, v in info.items() if k != 'cmd'}
            s.update({'cfg': self.cfg_hash, 'cmd_in': list(core.cmd), 'armed': core.armed,
                      'lease_left': max(0.0, core.session_end - self.now()) if core.armed else 0.0,
                      'limited_s': {k: round(v, 2) for k, v in core.stats['limited_s'].items()},
                      'estops': core.stats['estops'], 'fw_flags': self.fw_flags,
                      'zone': core.zone.digest() if core.zone is not None else None,
                      'fault_detail': {k: v[1] for k, v in core.faults.items()}})
            msg = String()
            msg.data = json.dumps(s)
            self.pub_status.publish(msg)

    def shutdown(self):
        tw = Twist()
        try:
            self.pub.publish(tw)
        except Exception:
            pass


def main(argv=None):
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--config', default=os.path.join(here, '..', 'config', 'safety_config.yaml'))
    ap.add_argument('--approved', default=os.path.join(here, '..', 'config', 'safety_config.approved'))
    ap.add_argument('--zone', default=None)
    ap.add_argument('--token-file', default=None, help='file whose content is the operator token; only its sha256 is kept')
    ap.add_argument('--log', default='/tmp/safety_gate.log')
    ap.add_argument('--no-approval-check', action='store_true', help='development only')
    a, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    cfg, h = load_config(a.config, None if a.no_approval_check else a.approved)
    zone = Zone.load(a.zone) if a.zone else None
    token_hash = hashlib.sha256(open(a.token_file).read().strip().encode()).hexdigest() if a.token_file else None
    if token_hash is None:
        print('WARNING: no --token-file: anybody on the ROS graph can arm the gate', file=sys.stderr)
    rclpy.init(args=ros_args)
    node = SafetyGate(cfg, h, zone, token_hash, a.log)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
