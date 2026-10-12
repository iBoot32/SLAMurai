"""Measure the current behaviour: drive a route several times with the parameters as they are and print the metrics.
python recipes/baseline.py   (edit PROFILE, ROUTE, REPEATS)"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'robotlab'))
from lab import connect  # noqa: E402

PROFILE = os.path.join(HERE, '..', 'robotlab', 'profiles', 'slamurai.yaml')
ROUTE = [(1.0, 1.0), (4.0, 1.0), (4.0, 3.0)]
REPEATS = 5

lab = connect(PROFILE, work=os.path.join(HERE, '..', 'runs', 'baseline'))
KEYS = ['duration', 'speed_attainment', 'ct_rms', 'ct_max', 'jerk_rms', 'clear_min', 'gate_limited_frac', 'recoveries']
print('%-6s ' % 'run' + ' '.join('%17s' % k for k in KEYS))
for r in range(REPEATS):
    run = lab.run(ROUTE, label='baseline#%d' % r)
    print('%-6d ' % r + ' '.join('%17.3f' % run.metric(k) for k in KEYS))
