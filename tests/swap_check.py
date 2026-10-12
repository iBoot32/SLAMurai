"""Run in its own process (one roslibpy connection per process): the fake robot and the client use a profile with renamed topics."""
import copy
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 'robotlab')]
from fake_robot import FakeRobot   # noqa: E402
from robot import Robot            # noqa: E402

p = yaml.safe_load(open(os.path.join(HERE, '..', 'robotlab', 'profiles', 'slamurai.yaml')))
for sec in ('goal', 'heartbeat', 'restart'):
    p[sec]['topic'] = '/bot' + p[sec]['topic']
p['restart']['status_topic'] = '/bot' + p['restart']['status_topic']
for s in p['streams'].values():
    s['topic'] = '/bot' + s['topic']
p['status'] = {'faults': 'errors', 'armed': 'enabled', 'clearance': 'nearest', 'limit': 'why'}
fake = FakeRobot(p)
robot = Robot(p, host='127.0.0.1', port=fake.start())
robot.wait_ready()
st = robot.status()
assert st['armed'] and st['faults'] == [] and st['clearance'] == 1.5, st
robot.send_goal(2.0, 2.0, 0.0)
robot.sleep(1.0)
pose = robot.pose()
assert abs(pose[0] - 2.0) < 0.1, pose
robot.close()
fake.stop()
print('ok')
