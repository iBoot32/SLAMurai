"""Sweep two Nav2 parameters over a route and rank them against the base. Paths follow the Nav2 yaml: node/ros__parameters/name.
python recipes/speed_sweep.py   (edit PROFILE, ROUTE, GRID)"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'robotlab'))
from lab import connect  # noqa: E402

PROFILE = os.path.join(HERE, '..', 'robotlab', 'profiles', 'slamurai.yaml')
ROUTE = [(1.0, 1.0), (4.0, 1.0), (4.0, 3.0)]
GRID = {'dwb_controller/ros__parameters/max_speed_xy': [0.15, 0.2], 'dwb_controller/ros__parameters/sim_time': [1.5, 2.5]}

lab = connect(PROFILE, work=os.path.join(HERE, '..', 'runs', 'speed_sweep'))
out = lab.sweep(GRID, ROUTE, repeats=3)
print('base loss %.3f (sd %.3f)' % (out['base_loss'], out['base_sd']))
for row in out['rows']:
    print('%8.3f  gain %+.3f  p %.3f  %s' % (row['loss'], row['gain'], row['p'], row['changes']))
