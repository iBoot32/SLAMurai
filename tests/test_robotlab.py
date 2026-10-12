"""robotlab against a fake rosbridge robot on localhost. Needs: pip install roslibpy websockets."""
import os
import subprocess
import sys
import time

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 'robotlab')]
pytest.importorskip('roslibpy')
pytest.importorskip('websockets')

import lab as LAB                    # noqa: E402
import optimizer as OPT              # noqa: E402
import params as P                   # noqa: E402
import stats                         # noqa: E402
from fake_robot import FakeRobot     # noqa: E402
from robot import Robot              # noqa: E402

PROFILE = os.path.join(HERE, '..', 'robotlab', 'profiles', 'slamurai.yaml')
SPEED = 'dwb_controller/ros__parameters/max_speed_xy'
BASE = {'dwb_controller': {'ros__parameters': {'max_speed_xy': 0.2, 'max_vel_theta': 0.5, 'acc_lim_x': 0.4, 'decel_lim_x': -0.5}}}


@pytest.fixture(scope='module')
def rig():
    prof = yaml.safe_load(open(PROFILE))
    prof['_dir'] = os.path.dirname(PROFILE)
    fake = FakeRobot(prof)
    port = fake.start()
    robot = Robot(prof, host='127.0.0.1', port=port)
    robot.wait_ready()
    yield fake, robot, prof
    robot.close()
    fake.stop()


def test_streams_pose_and_status_come_through_the_profile(rig):
    fake, robot, _ = rig
    assert robot.pose() == pytest.approx((1.0, 2.0, 0.0), abs=0.05)         # map = odom + (1, 2) from the tf chain
    s = robot.status()
    assert s['armed'] and s['faults'] == [] and s['clearance'] == 1.5


def test_goal_plan_cancel_and_heartbeat(rig):
    fake, robot, _ = rig
    robot.plan()
    robot.send_goal(2.5, 2.0, 0.0)
    robot.sleep(0.3)
    plan = robot.plan()
    assert plan is not None and plan.shape[1] == 2 and abs(plan[-1][0] - 2.5) < 0.05
    fake.sent_goals.clear()
    robot.cancel()                                                         # a goal at the current pose
    robot.sleep(0.3)
    assert len(fake.sent_goals) == 1
    hb = fake.heartbeats
    time.sleep(1.0)
    assert fake.heartbeats == hb                                           # no sleep() running: no heartbeat
    robot.sleep(1.0)
    assert fake.heartbeats > hb


def test_restart_nav2_and_refusal(rig):
    fake, robot, _ = rig
    assert robot.restart_nav2(P.apply(BASE, {SPEED: 0.3}))
    assert fake.speed == 0.3
    with pytest.raises(RuntimeError, match='refused'):
        robot.restart_nav2(P.apply(BASE, {SPEED: 5.0}))


def test_lab_run_records_scores_and_logs(rig, tmp_path):
    _, robot, _ = rig
    lab = LAB.Lab(robot, BASE, str(tmp_path))
    route = [(1.0, 2.0), (2.0, 2.0), (2.0, 3.0)]
    slow, fast = lab.run(route, {SPEED: 0.15}, 'slow'), lab.run(route, {SPEED: 0.3}, 'fast')
    assert len(slow.legs) == 2 and all(l['trial']['events']['end_reason'] == 'reached' for l in slow.legs)
    assert fast.metric('duration') < slow.metric('duration')
    assert np.isfinite(slow.loss) and np.isfinite(fast.loss)
    assert len(os.listdir(os.path.join(str(tmp_path), 'trials'))) == 4
    assert open(os.path.join(str(tmp_path), 'lab.jsonl')).read().count('"label"') == 2


def test_lab_sweep_compare_and_budget(rig, tmp_path):
    _, robot, _ = rig
    lab = LAB.Lab(robot, BASE, str(tmp_path), max_trials=100)
    route = [(1.0, 2.0), (2.0, 2.0)]
    out = lab.sweep({SPEED: [0.15, 0.3]}, route, repeats=2)
    assert len(out['rows']) == 2 and out['rows'][0]['loss'] <= out['rows'][1]['loss']
    cmp = lab.compare({SPEED: 0.15}, {SPEED: 0.3}, route, repeats=2)
    assert set(cmp) >= {'gain', 'p', 'ci95', 'loss_a', 'loss_b'}
    lab.max_trials = lab.n_trials
    with pytest.raises(LAB.Budget):
        lab.run(route)


def test_params_apply_rejects_unknown_paths():
    assert P.apply(BASE, {SPEED: 0.1})['dwb_controller']['ros__parameters']['max_speed_xy'] == 0.1
    with pytest.raises(KeyError):
        P.apply(BASE, {'dwb_controller/ros__parameters/nope': 1})


def test_signflip_and_paired_stats():
    assert stats.signflip_p([1.0] * 8) == pytest.approx(1 / 256)
    assert stats.signflip_p([1, -1, 1, -1]) > 0.4
    assert stats.paired([2.0, 2.0, 2.0], [1.0, 1.0, 1.0])['gain'] == 1.0


def test_trust_region_bo_finds_a_quadratic_minimum():
    f = lambda x: float(((x - 0.7) ** 2).sum())
    bo = OPT.TrustRegionBO(2, np.array([0.2, 0.2]), seed=1, noise_sd=0.01)
    for _ in range(40):
        x = np.clip(bo.ask(), 0, 1)
        bo.tell(x, f(x))
    assert bo.incumbent()[1] < 0.03


def test_a_robot_with_other_topic_names_needs_only_another_profile():
    r = subprocess.run([sys.executable, os.path.join(HERE, 'swap_check.py')], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
    assert r.returncode == 0, r.stdout.decode()[-1500:]
