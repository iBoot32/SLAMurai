"""Restart Nav2 with a new parameter file (needed on Dashing: parameters are read once at node start-up).

Starts `ros2 launch <pkg> nav2_only_launch.py params:=<yaml> map:=<map> bt:=<bt>` in its own process group and waits
until the lifecycle manager has brought bt_navigator to 'active'.  The robot is held still by the safety gate while
Nav2 is down (no /cmd_vel -> stale command -> zero output), so restarting is safe at any time.
Needs a shell where ROS 2 Dashing is sourced.  Standard library only."""
import os
import signal
import subprocess
import sys
import tempfile
import time

import yaml


class Nav2Process(object):
    def __init__(self, package='slamurai_nav', launch='nav2_only_launch.py', map_yaml='', bt_xml='', log_dir=None):
        self.package, self.launch, self.map_yaml, self.bt_xml = package, launch, map_yaml, bt_xml
        self.proc = None
        self.log_dir = log_dir or tempfile.gettempdir()

    def stop(self, grace=8.0):
        if self.proc is None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=grace)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.proc = None

    def write_params(self, params_dict, initial_pose=None, path=None):
        d = dict(params_dict)
        if initial_pose is not None and 'amcl' in d:
            ip = d['amcl']['ros__parameters'].setdefault('initial_pose', {})
            ip.update({'x': float(initial_pose[0]), 'y': float(initial_pose[1]), 'z': 0.0, 'yaw': float(initial_pose[2])})
            d['amcl']['ros__parameters']['set_initial_pose'] = True
        path = path or os.path.join(self.log_dir, 'autotune_nav2_params.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump(d, f)
        return path

    def restart(self, params_dict, initial_pose=None, ready_timeout=120.0):
        self.stop()
        pfile = self.write_params(params_dict, initial_pose)
        cmd = ['ros2', 'launch', self.package, self.launch, 'params:=' + pfile]
        if self.map_yaml:
            cmd.append('map:=' + self.map_yaml)
        if self.bt_xml:
            cmd.append('bt:=' + self.bt_xml)
        log = open(os.path.join(self.log_dir, 'autotune_nav2.log'), 'ab')
        self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        t0 = time.time()
        while time.time() - t0 < ready_timeout:
            if self.proc.poll() is not None:
                raise RuntimeError('nav2 launch exited with code %s (see autotune_nav2.log)' % self.proc.returncode)
            if self.active('/bt_navigator') and self.active('/amcl') and self.active('/dwb_controller'):
                return pfile
            time.sleep(2.0)
        raise RuntimeError('nav2 did not become active within %.0f s' % ready_timeout)

    @staticmethod
    def active(node):
        try:
            out = subprocess.check_output(['ros2', 'lifecycle', 'get', node], stderr=subprocess.STDOUT, timeout=8).decode()
            return 'active' in out and 'inactive' not in out
        except Exception:
            return False
