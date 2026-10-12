"""Whole-codebase offline checks against the files that actually run on the robot (no ROS, no hardware needed):
 * the REAL serial bridge (autonav/nav/src/main.py) <-> the REAL firmware (.ino compiled on the PC) closed loop,
 * the topic wiring between firmware bridge, safety gate, EKF, Nav2 parameters, rosbridge clients and cortex,
 * packaging: every node the launch files start is installed, importable beside its siblings and has a shebang."""
import ast
import collections
import importlib.util
import math
import os
import re
import subprocess
import sys
import time

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ROOT = os.path.abspath(os.path.join(HERE, '..'))
NAV = os.path.join(ROOT, 'autonav', 'nav')
FW = os.path.join(HERE, 'firmware')
import fake_ros                                                          # noqa: E402
import test_firmware as TF                                               # noqa: E402


def read(*p):
    return open(os.path.join(ROOT, *p), encoding='utf-8').read()


# ---------------------------------------------------------------------------------- bridge <-> firmware closed loop
class FwPipe(object):
    """Serial port stand-in whose other end is the compiled real firmware."""

    def __init__(self):
        exe = os.path.join(FW, 'fw_pipe' + ('.exe' if os.name == 'nt' else ''))
        for _ in range(6):
            r = subprocess.run(TF.CC + ['-w', '-std=c++14', '-O1', '-I', os.path.join(FW, 'stubs'), '-o', exe,
                                        os.path.join(FW, 'fw_pipe.cpp')], cwd=FW, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if r.returncode == 0:
                break
            time.sleep(1.5)
        assert r.returncode == 0, r.stdout.decode(errors='replace')[-1500:]
        self.exe = exe
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, cwd=FW)
        self.rx = collections.deque()
        self.in_waiting = 0
        self.writes = []

    def write(self, b):
        self.writes.append(b.decode())
        self.p.stdin.write(b'W' + b.strip() + b'\n')

    def advance(self, us):
        self.p.stdin.write(('R%d\n' % us).encode())
        self.p.stdin.flush()
        while True:
            l = self.p.stdout.readline().decode().strip()
            if l == 'E' or not l:
                break
            self.rx.append(l[2:])

    def readline(self):
        self.in_waiting = max(0, len(self.rx) - 1)
        return (self.rx.popleft() + '\n').encode() if self.rx else b''

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(5)
        except Exception:
            self.p.kill()


class Clock(object):
    t = 1000.0

    @classmethod
    def monotonic(cls):
        return cls.t


def load_bridge(ser):
    fake_ros.install() if hasattr(fake_ros, 'install') else None
    sys.modules['serial'] = type(sys)('serial')
    sys.modules['serial'].Serial = type(ser)
    spec = importlib.util.spec_from_file_location('bridge_real', os.path.join(NAV, 'src', 'main.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.time = Clock
    return m


def cmd(x, y=0.0, w=0.0):
    return fake_ros.Twist(linear=fake_ros.NS(x=x, y=y, z=0.0), angular=fake_ros.NS(x=0.0, y=0.0, z=w))


@pytest.mark.skipif(TF.CC is None, reason='no C++ compiler')
def test_real_bridge_and_real_firmware_closed_loop():
    ser = FwPipe()
    try:
        bridge = load_bridge(ser)
        n = bridge.CmdVelSerialOdom(ser)
        odom_cb, send_cb = n.timers[0][1], n.timers[1][1]
        Clock.t = 1000.0
        n.cmd_rx = Clock.t

        def run(seconds, command, feed_gate=True, bridge_alive=True):
            for _ in range(int(seconds / 0.01)):
                Clock.t += 0.01
                if feed_gate and command is not None and int(round(Clock.t * 100)) % 5 == 0:
                    n.deliver('/cmd_vel_safe', command)           # the gate's 20 Hz output
                if bridge_alive and int(round(Clock.t * 100)) % 5 == 0:
                    send_cb()
                ser.advance(10000)
                if bridge_alive and int(round(Clock.t * 100)) % 2 == 0:
                    odom_cb()

        def vel():
            t = n.pubs['/odometry/raw'].msgs[-1].twist.twist
            return t.linear.x, t.linear.y, t.angular.z

        run(0.5, None, feed_gate=False)
        assert len(n.pubs['/odometry/raw'].msgs) > 10, 'firmware odometry never reached the bridge'
        run(2.0, cmd(0.15, 0.05, 0.2))                            # normal gated command
        vx, vy, w = vel()
        assert abs(vx - 0.15) < 0.02 and abs(vy - 0.05) < 0.02 and abs(w - 0.2) < 0.05, (vx, vy, w)
        o = n.pubs['/odometry/raw'].msgs[-1]
        assert o.pose.pose.position.x > 0.1 and o.pose.pose.orientation.z > 0.0       # moved forward, turned left
        assert n.pubs['/fw_flags'].msgs, 'firmware flags field never republished'
        run(2.0, cmd(1.0, 0.0, 3.0))                              # a bridge fed far too much (gate bypassed): the firmware clamps
        vx, vy, w = vel()
        assert math.hypot(vx, vy) <= 0.21 + 0.02 and abs(w) <= 0.5 + 0.05, (vx, vy, w)
        run(0.7, cmd(0.15), feed_gate=False)                      # gate goes silent: bridge sends zeros after 0.3 s
        assert ser.writes[-1].strip() == '0.0000,0.0000,0.0000'
        assert math.hypot(*vel()[:2]) < 0.03, vel()
        run(1.0, cmd(0.15, 0, 0))                                 # restart
        assert vel()[0] > 0.1
        run(0.8, None, feed_gate=False, bridge_alive=False)       # the Jetson process dies: firmware watchdog stops the robot
        ser.rx.clear()
        ser.advance(100000)
        last = [float(v) for v in ser.rx[-1].split(',')] if ser.rx else None
        assert last is not None and math.hypot(last[3], last[4]) < 0.03, last
    finally:
        ser.close()


# ---------------------------------------------------------------------------------- topic names and install list
def topics(path, kinds):
    src = read(*path)
    return set(re.findall(r"(?:%s)\([^)]*?['\"](/[A-Za-z0-9_/]+)['\"]" % "|".join(kinds), src, re.S))


def test_profile_topics_match_what_the_robot_side_code_uses():
    prof = yaml.safe_load(read('robotlab', 'profiles', 'slamurai.yaml'))
    bridge = os.path.join('autonav', 'nav', 'src', 'main.py').split(os.sep)
    gate, mgr = ['robot_side', 'safety', 'safety_gate.py'], ['robot_side', 'tools', 'nav2_manager.py']
    assert topics(bridge, ['create_subscription']) == {'/cmd_vel_safe'} and '/cmd_vel_safe' in topics(gate, ['create_publisher'])
    s = prof['streams']
    assert s['cmd_out']['topic'] in topics(gate, ['create_publisher'])                 # only the gate feeds the motors
    assert s['odom']['topic'] in topics(bridge, ['create_publisher'])
    assert s['status']['topic'] in topics(gate, ['create_publisher'])
    assert prof['heartbeat']['topic'] in topics(gate, ['create_subscription'])
    assert prof['restart']['topic'] in topics(mgr, ['create_subscription'])
    assert prof['restart']['status_topic'] in topics(mgr, ['create_publisher'])
    ekf = yaml.safe_load(read('autonav', 'nav', 'config', 'ekf.yaml'))
    e = ekf.get('ekf_filter_node', ekf)['ros__parameters']
    assert e['odom0'] == s['odom']['topic'] and e['base_link_frame'] in prof['pose']['base_frames'] and e['publish_tf']
    assert yaml.safe_load(read('robot_side', 'config', 'nav2_params.derived.yaml'))['dwb_controller']['ros__parameters']['odom_topic'] == '/odometry/filtered'
    cortex = read('cortex', 'slamurai_interface.py')
    assert prof['goal']['topic'] in cortex and s['amcl']['topic'] in cortex          # the existing UI uses the same names


def test_launched_nodes_are_installed_with_lf_shebang_and_their_imports():
    cmake = read('autonav', 'nav', 'CMakeLists.txt')
    files = [os.path.normpath(os.path.join(NAV, x)) for blk in re.findall(r'PROGRAMS(.*?)DESTINATION', cmake, re.S) for x in blk.split()]
    names = set(os.path.basename(f) for f in files)
    launched = set()
    for f in os.listdir(os.path.join(NAV, 'launch')):
        launched |= set(re.findall(r"package='slamurai_nav'.*?node_executable='([^']+)'", read('autonav', 'nav', 'launch', f), re.S))
    assert launched <= names, launched - names
    for f in files:
        assert os.path.isfile(f), f
        if os.path.basename(f) in launched:
            first = open(f, 'rb').readline()
            assert first.startswith(b'#!') and b'\r' not in first, 'bad shebang: ' + f
        for node in ast.walk(ast.parse(open(f, encoding='utf-8').read())):
            mods = [node.module] if isinstance(node, ast.ImportFrom) and node.module else (
                [a.name for a in node.names] if isinstance(node, ast.Import) else [])
            for m in mods:
                local = [d for d in ('safety', 'tools') if os.path.isfile(os.path.join(ROOT, 'robot_side', d, m.split('.')[0] + '.py'))]
                assert not local or m.split('.')[0] + '.py' in names, '%s imports %s which is not installed' % (os.path.basename(f), m)
