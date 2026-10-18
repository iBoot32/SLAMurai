"""Robot connection over rosbridge. All robot-specific names come from the profile (see profiles/slamurai.yaml)."""
import json
import math
import os
import re
import threading
import time

import numpy as np
import yaml

import roslibpy


def load_profile(profile):
    if isinstance(profile, dict):
        return profile
    prof = yaml.safe_load(open(profile))
    prof['_dir'] = os.path.dirname(os.path.abspath(profile))
    return prof


def yaw_of(q):
    return math.atan2(2.0 * (q['w'] * q['z'] + q['x'] * q['y']), 1.0 - 2.0 * (q['y'] ** 2 + q['z'] ** 2))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ConnectionLost(RuntimeError):
    pass


class Robot(object):
    def __init__(self, profile, host=None, port=None, connect_timeout=15.0):
        self.p = load_profile(profile)
        self.lock = threading.Lock()
        self.v = {}                           # stream name -> latest parsed value
        self.rx = {}                          # stream name -> arrival time
        self.cmd_t = []
        self.new_plan = None
        self.counts = {}
        self.tf = {}
        self.restart_status = {}
        self.last_hb = 0.0
        self.lost_since = None
        host, port = host or self.p['host'], int(port or self.p.get('port', 9090))
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        t0 = time.time()
        while not self.ros.is_connected:
            if time.time() - t0 > connect_timeout:
                raise ConnectionLost('cannot reach rosbridge at ws://%s:%d' % (host, port))
            time.sleep(0.1)
        self.topics = []
        for name, s in self.p.get('streams', {}).items():
            kw = {}
            if 'throttle_ms' in s:
                kw['throttle_rate'] = s['throttle_ms']
            if 'queue_length' in s:
                kw['queue_length'] = s['queue_length']
            t = roslibpy.Topic(self.ros, s['topic'], s['type'], **kw)
            t.subscribe(lambda m, n=name, ty=s['type'].split('/')[-1]: self._on(n, ty, m))
            self.topics.append(t)
        self.pub = {}
        for key in ('goal', 'heartbeat', 'restart'):
            if key in self.p:
                t = roslibpy.Topic(self.ros, self.p[key]['topic'], self.p[key]['type'])
                t.advertise()
                self.pub[key] = t
        if 'restart' in self.p:
            r = self.p['restart']
            t = roslibpy.Topic(self.ros, r['status_topic'], r.get('status_type', 'std_msgs/String'))
            t.subscribe(self._on_restart_status)
            self.topics.append(t)

    # ---- incoming messages (roslibpy thread)
    def _on(self, name, ty, m):
        now = time.monotonic()
        with self.lock:
            if ty == 'Odometry':
                q, t = m['pose']['pose'], m['twist']['twist']
                self.v[name] = (q['position']['x'], q['position']['y'], yaw_of(q['orientation']),
                                t['linear']['x'], t['linear']['y'], t['angular']['z'])
            elif ty == 'PoseWithCovarianceStamped':
                q = m['pose']['pose']
                self.v[name] = (q['position']['x'], q['position']['y'], yaw_of(q['orientation']))
                self.v['amcl_odom'] = self.v.get('odom')
            elif ty == 'Twist':
                self.v[name] = (m['linear']['x'], m['linear']['y'], m['angular']['z'])
                if name == 'cmd_in':
                    self.cmd_t.append(now)
            elif ty == 'Path':
                pts = [(q['pose']['position']['x'], q['pose']['position']['y']) for q in m.get('poses', [])]
                if len(pts) >= 2:
                    self.v[name] = np.array(pts)
                    self.new_plan = self.v[name]
            elif ty == 'Log':
                txt = m.get('msg') or ''
                for k, pat in self.p.get('log_patterns', {}).items():
                    if re.search(pat, txt):
                        self.counts[k] = self.counts.get(k, 0) + 1
            elif ty == 'String':
                try:
                    self.v[name] = json.loads(m['data'])
                except ValueError:
                    self.v[name] = m['data']
            elif ty == 'TFMessage':
                pc = self.p.get('pose', {})
                for tr in m.get('transforms', []):
                    key = (tr['header']['frame_id'].lstrip('/'), tr['child_frame_id'].lstrip('/'))
                    if key[0] in (pc.get('map_frame'), pc.get('odom_frame')):
                        t = tr['transform']
                        self.tf[key] = (t['translation']['x'], t['translation']['y'], yaw_of(t['rotation']), now)
            else:
                self.v[name] = m
            self.rx[name] = now

    def _on_restart_status(self, m):
        try:
            self.restart_status = json.loads(m['data'])
        except ValueError:
            pass

    # ---- reading
    def pose(self, max_age=1.5, max_odom_travel=0.5):
        """Robot pose (x, y, yaw) in the map frame, or None. Uses the TF chain if available, else the last localization
        fix advanced by odometry."""
        pc = self.p.get('pose', {})
        now = time.monotonic()
        mo = self.tf.get((pc.get('map_frame'), pc.get('odom_frame')))
        ob = next((self.tf[(pc['odom_frame'], b)] for b in pc.get('base_frames', []) if (pc['odom_frame'], b) in self.tf), None)
        if mo and ob and now - mo[3] < 10.0 and now - ob[3] < max_age:
            c, s = math.cos(mo[2]), math.sin(mo[2])
            return (mo[0] + c * ob[0] - s * ob[1], mo[1] + s * ob[0] + c * ob[1], wrap(mo[2] + ob[2]))
        with self.lock:
            a, a0, o = self.v.get('amcl'), self.v.get('amcl_odom'), self.v.get('odom')
        if a is None or a0 is None or o is None:
            return None
        dx, dy = o[0] - a0[0], o[1] - a0[1]
        if math.hypot(dx, dy) > max_odom_travel:
            return None
        th = wrap(a[2] - a0[2])
        c, s = math.cos(th), math.sin(th)
        return (a[0] + c * dx - s * dy, a[1] + s * dx + c * dy, wrap(a[2] + wrap(o[2] - a0[2])))

    def velocity(self):
        o = self.v.get('odom')
        return tuple(o[3:6]) if o else (0.0, 0.0, 0.0)

    def cmd_in(self):
        return self.v.get('cmd_in', (0.0, 0.0, 0.0))

    def cmd_out(self):
        return self.v.get('cmd_out', (0.0, 0.0, 0.0))

    def cmd_times(self):
        with self.lock:
            return list(self.cmd_t)

    def plan(self):
        """The newest global plan (n, 2) if one arrived since the last call, else None."""
        with self.lock:
            p, self.new_plan = self.new_plan, None
        return p

    def count(self, name):
        return self.counts.get(name, 0)

    def status(self, max_age=2.5):
        """Safety status mapped through the profile: faults, armed, clearance, limit (and the raw message). None if stale."""
        raw = self.v.get('status')
        if not isinstance(raw, dict) or time.monotonic() - self.rx.get('status', 0) > max_age:
            return None
        k = self.p.get('status', {})
        return {'faults': list(raw.get(k.get('faults', 'faults'), []) or []), 'armed': bool(raw.get(k.get('armed', 'armed'))),
                'clearance': raw.get(k.get('clearance', 'min_clear')), 'limit': raw.get(k.get('limit', 'limit'), '') or '',
                'raw': raw}

    def faults(self):
        s = self.status()
        if s is None:
            return ['STATUS_STALE'] if 'status' in self.p.get('streams', {}) else []
        return s['faults']

    # ---- acting
    def send_goal(self, x, y, yaw=0.0):
        g, now = self.p['goal'], time.time()
        self.pub['goal'].publish(roslibpy.Message({
            'header': {'frame_id': g.get('frame', 'map'), 'stamp': {'sec': int(now), 'nanosec': int((now % 1) * 1e9)}},
            'pose': {'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                     'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}}}))
        self.v['cmd_in'] = (0.0, 0.0, 0.0)

    def cancel(self):
        """Dashing has no cancel topic: a new goal at the current pose replaces the old one."""
        p = self.pose()
        if p:
            self.send_goal(*p)

    def heartbeat(self):
        h, t = self.p.get('heartbeat'), time.monotonic()
        if h and t - self.last_hb >= h.get('period_s', 0.4):
            self.last_hb = t
            self.pub['heartbeat'].publish(roslibpy.Message({'data': h.get('data', 'hb')}))

    def sleep(self, dt):
        """Wait dt seconds. Heartbeats go out only from here, so a hung script stops the robot."""
        end = time.monotonic() + dt
        while True:
            if not self.ros.is_connected:
                self.lost_since = self.lost_since or time.monotonic()
                if time.monotonic() - self.lost_since > 1.0:
                    raise ConnectionLost('rosbridge connection lost')
            else:
                self.lost_since = None
            self.heartbeat()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(left, 0.05))

    def wait_ready(self, timeout=20.0):
        need = ['odom'] + (['status'] if 'status' in self.p.get('streams', {}) else [])
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.sleep(0.1)
            if all(n in self.v for n in need) and self.pose() is not None:
                return True
        missing = [n for n in need if n not in self.v] + (['pose'] if self.pose() is None else [])
        raise ConnectionLost('no data yet for: ' + ', '.join(missing))

    def restart_nav2(self, params, initial_pose=None):
        """Restart Nav2 with this parameter dict. Returns when Nav2 reports active."""
        if 'restart' not in self.p:
            raise RuntimeError('profile has no restart helper: restart Nav2 by hand with the new parameters')
        r = self.p['restart']
        rid = int(time.time() * 1000) % 10 ** 9
        self.restart_status = {}
        self.pub['restart'].publish(roslibpy.Message({'data': json.dumps({
            'id': rid, 'yaml': yaml.safe_dump(params), 'initial_pose': list(initial_pose) if initial_pose else None})}))
        t0 = time.monotonic()
        while time.monotonic() - t0 < r.get('timeout_s', 240):
            self.sleep(0.5)
            st = self.restart_status
            if st.get('id') == rid and st.get('state') == 'active':
                return True
            if st.get('id') == rid and st.get('state') == 'error':
                raise RuntimeError('Nav2 restart failed: %s' % st.get('msg'))
        raise RuntimeError('Nav2 did not become active in time: %s' % self.restart_status)

    def close(self):
        try:
            self.ros.terminate()
        except Exception:
            pass
