#!/usr/bin/env python3
"""ROBOT side.  Lets the PC restart Nav2 with a new parameter file over rosbridge (Dashing reads parameters only at start-up).

  listens  /autotune/nav2_restart  std_msgs/String  JSON {"id": n, "yaml": "<nav2 params>", "initial_pose": [x, y, yaw]}  or {"stop": true}
  publishes /autotune/nav2_status  std_msgs/String  JSON {"id": n, "state": "starting|active|error|stopped", "msg": ""}  at 2 Hz

Before starting Nav2 it checks the requested parameters against the approved safety config (limits_check.py): a request that
asks for more speed/acceleration than the robot may do, or a smaller robot radius, is refused.  (The gate would bound the motion
anyway; this just keeps the planner honest.)  It starts nothing else and takes no other input.
Run (shell with ROS 2 Dashing sourced):  python3 nav2_manager.py --map <map.yaml> --bt <bt.xml> [--package slamurai_nav]
Python 3.6 compatible."""
import argparse
import json
import os
import sys
import threading

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (HERE,):
    sys.path.insert(0, p)

import rclpy                                   # noqa: E402
from rclpy.node import Node                    # noqa: E402
from rclpy.qos import QoSProfile               # noqa: E402
from std_msgs.msg import String                # noqa: E402

from limits_check import check_limits          # noqa: E402
from nav2_proc import Nav2Process              # noqa: E402


class Manager(Node):
    def __init__(self, safety, nav2):
        super().__init__('autotune_nav2_manager')
        self.safety, self.nav2 = safety, nav2
        self.status = {'id': 0, 'state': 'idle', 'msg': ''}
        self.busy = False
        self.pub = self.create_publisher(String, '/autotune/nav2_status', QoSProfile(depth=5))
        self.create_subscription(String, '/autotune/nav2_restart', self.on_request, QoSProfile(depth=5))
        self.create_timer(0.5, self.publish_status)

    def publish_status(self):
        m = String()
        m.data = json.dumps(self.status)
        self.pub.publish(m)

    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except Exception:
            return
        if req.get('stop'):
            self.nav2.stop()
            self.status = {'id': req.get('id', 0), 'state': 'stopped', 'msg': ''}
            return
        rid = req.get('id', 0)
        if self.busy:
            self.status = {'id': rid, 'state': 'error', 'msg': 'busy: a restart is already running'}
            return
        try:
            params = yaml.safe_load(req['yaml'])
        except Exception as e:
            self.status = {'id': rid, 'state': 'error', 'msg': 'bad yaml: %s' % e}
            return
        problems = check_limits(params, self.safety)
        if problems:
            self.status = {'id': rid, 'state': 'error', 'msg': 'refused: ' + '; '.join(problems)}
            self.get_logger().error(self.status['msg'])
            return
        self.busy = True
        self.status = {'id': rid, 'state': 'starting', 'msg': ''}
        threading.Thread(target=self.run_restart, args=(rid, params, req.get('initial_pose')), daemon=True).start()

    def run_restart(self, rid, params, pose):
        try:
            self.nav2.restart(params, initial_pose=pose)
            self.status = {'id': rid, 'state': 'active', 'msg': ''}
        except Exception as e:
            self.status = {'id': rid, 'state': 'error', 'msg': str(e)}
        finally:
            self.busy = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(HERE, '..', 'config', 'safety_config.yaml'))
    ap.add_argument('--map', default='')
    ap.add_argument('--bt', default='')
    ap.add_argument('--package', default='slamurai_nav')
    ap.add_argument('--autostart', default='', help='Nav2 parameter file to start with at launch (as the stock launch does)')
    a, ros_args = ap.parse_known_args()
    safety = yaml.safe_load(open(a.config))
    rclpy.init(args=ros_args)
    nav2 = Nav2Process(package=a.package, map_yaml=a.map, bt_xml=a.bt)
    node = Manager(safety, nav2)
    if a.autostart:
        node.busy = True
        node.status = {'id': 0, 'state': 'starting', 'msg': ''}
        threading.Thread(target=node.run_restart, args=(0, yaml.safe_load(open(a.autostart)), None), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        nav2.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
