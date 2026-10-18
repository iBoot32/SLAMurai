#!/usr/bin/env python3
"""HUMAN step: arm (or disarm) the safety gate for a tuning session.  Not part of the agent's CLI.

  python3 arm_session.py --minutes 30 --token-file /etc/slamurai/operator_token          arm
  python3 arm_session.py --disarm                                                         disarm now
  python3 arm_session.py --clear --token-file ...                                         clear a latched fault (after you looked!)

Before arming: robot on the floor in the cleared zone at its start point, a person within reach of the power switch,
safety/commission.py passed, tools/preflight.py clean.  The session ends by itself when the lease runs out; a hard
fault latches until you clear it here."""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=30.0)
    ap.add_argument('--token-file')
    ap.add_argument('--disarm', action='store_true')
    ap.add_argument('--clear', action='store_true')
    a = ap.parse_args()
    token = open(a.token_file).read().strip() if a.token_file else ''
    rclpy.init()
    n = Node('arm_session')
    pub = n.create_publisher(String, '/safety/operator', 10)
    st = {}
    n.create_subscription(String, '/safety/status', lambda m: st.update(j=json.loads(m.data)), QoSProfile(depth=5))
    if a.disarm:
        cmd = {'cmd': 'disarm'}
    elif a.clear:
        cmd = {'cmd': 'clear', 'token': token}
    else:
        cmd = {'cmd': 'arm', 'lease_s': a.minutes * 60.0, 'token': token}
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if int((time.time() - t0) * 10) % 5 == 0:
            m = String()
            m.data = json.dumps(cmd)
            pub.publish(m)
    j = st.get('j')
    if not j:
        sys.exit('no /safety/status: is the gate running?')
    print('gate state %s, armed=%s, faults=%s, lease_left=%.0f s, config %s' % (
        j.get('state'), j.get('armed'), j.get('faults'), j.get('lease_left', 0), j.get('cfg')))
    if not a.disarm and not a.clear and not j.get('armed'):
        sys.exit('NOT ARMED: %s' % (j.get('fault_detail') or j.get('limit') or 'see /tmp/safety_gate.log'))


if __name__ == '__main__':
    main()
