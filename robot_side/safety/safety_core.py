"""Safety core: a pure-Python velocity gate with no ROS imports, so it can be unit-tested, fuzzed and read in one sitting.

Contract
--------
The only thing allowed to move the robot is `SafetyCore.step()`.  It takes the (untrusted) velocity command from Nav2 /
the tuning agent and returns the velocity that is actually sent to the motors.  Whatever the command is (NaN, huge,
into a wall, out of the zone, from a crashed planner, stale) the output satisfies, by construction:

  1. |v| <= v_max, |w| <= w_max, every wheel <= wheel_cap_eff  (clamp, then common scale)
  2. output accelerates no faster than a_acc (slowing is never limited)
  3. the robot can still stop before the nearest scan return, the zone edge or the tether (speed-and-separation):
         gap >= v*t + v^2/(2a) + margin + v_obs*(t + v/a),     t = fixed latency + scan_time + scan age
  4. motion toward a closer-than-allowed obstacle is blocked, motion away is allowed (the robot can always back out)
  5. any stale input (scan, odom, command, heartbeat), unknown lidar sector, lost localization -> stop or creep
  6. hard faults (tether, odom jump, contact, motion mismatch, scan dead, firmware fault) latch to zero output until a
     human re-arms.

Conventions: base frame x forward, y left, yaw CCW; SI units; time `now` in seconds on one monotonic clock that the
caller supplies (so tests control time).  Python 3.6 compatible (ROS 2 Dashing / Ubuntu 18.04): no dataclasses.
"""
import math
import numpy as np

# fault names (hard = latched until a human re-arms)
HARD = ('TETHER', 'ODOM_JUMP', 'CONTACT', 'MOTION_MISMATCH', 'SCAN_DEAD', 'FW_FAULT', 'FENCE_BREACH', 'LOCALIZATION_LOST',
        'SESSION_TIMEOUT')

FW_BIT_TETHER, FW_BIT_TIMEOUT, FW_BIT_CLAMP, FW_BIT_WHEELSCALE, FW_BIT_MALFORMED, FW_BIT_BUMPER = 1, 2, 4, 8, 16, 32
FW_HARD_MASK = FW_BIT_TETHER | FW_BIT_BUMPER


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wheel_speeds(vx, vy, w, azimuths_deg, R):
    """Rim speed of each wheel (same formula as the firmware): v_i = -sin(t) vx + cos(t) vy + R w."""
    out = []
    for deg in azimuths_deg:
        t = math.radians(deg)
        out.append(-math.sin(t) * vx + math.cos(t) * vy + R * w)
    return out


def stop_safe_speed(gap, t_lat, a_brake, margin, v_obs=0.0):
    """Largest speed v (>=0) for which  gap >= v*t + v^2/(2a) + margin + v_obs*(t + v/a)  still holds."""
    if gap == float('inf'):
        return float('inf')
    B = t_lat + v_obs / a_brake
    C = margin + v_obs * t_lat - gap
    if C >= 0.0:
        return 0.0
    disc = B * B - 2.0 * C / a_brake
    return max(0.0, a_brake * (-B + math.sqrt(disc)))


class Zone(object):
    """Allowed-area signed distance field in the map frame (metres inside the zone, negative outside)."""

    def __init__(self, sdf, resolution, origin_x, origin_y):
        self.sdf = np.asarray(sdf, dtype=np.float32)
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)

    def digest(self):
        """Short hash of the zone, published in the gate status so the PC can check it plans inside the same zone."""
        import hashlib
        return hashlib.sha1(np.ascontiguousarray(self.sdf, dtype=np.float32).tobytes()).hexdigest()[:12]

    @classmethod
    def load(cls, path):
        d = np.load(path)
        return cls(d['sdf'], float(d['resolution']), float(d['origin_x']), float(d['origin_y']))

    def value(self, x, y):
        """sdf at map point(s); points outside the grid are 'outside' (-1)."""
        i = np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        j = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        ok = (i >= 0) & (i < self.sdf.shape[0]) & (j >= 0) & (j < self.sdf.shape[1])
        v = np.full(np.shape(i), -1.0, dtype=np.float32)
        v[ok] = self.sdf[i[ok], j[ok]]
        return v

    def free_distance(self, x, y, ux, uy, max_d=2.0, step=0.02):
        """Distance along (ux,uy) from (x,y) until the zone is left.  0 if already outside and not moving inward."""
        s0 = float(self.value(np.array([x]), np.array([y]))[0])
        d = np.arange(step, max_d + step, step)
        v = self.value(x + ux * d, y + uy * d)
        if s0 <= 0.0:
            return 0.0 if (len(v) == 0 or v[min(2, len(v) - 1)] <= s0 + 1e-6) else float('inf')
        out = np.nonzero(v <= 0.0)[0]
        return float(d[out[0]] - step) if len(out) else float('inf')


class SafetyCore(object):
    def __init__(self, cfg, zone=None):
        self.c = cfg
        self.zone = zone
        L = cfg['lidar']
        self.R = float(cfg['robot_radius_m'])
        self.lidar_yaw = float(L['frame_yaw_rad'])
        self.lidar_off = np.array(L['axis_offset_xy_m'], dtype=float)
        self.range_min = float(L['range_min_m'])
        self.range_max = float(L['range_max_m'])
        self.self_r = float(L['self_return_radius_m'])
        self.wedges = [(math.radians(a), math.radians(h)) for a, h in L['blind_wedges_deg']]
        self.contact_r = self.R - 0.02       # a return this close is inside the physical hull: contact / sensor corruption
        self.rot_block_r = self.R - 0.01
        # swept-corridor radius: guard circle + sideways margin, so a path that drifts a little (wheel slip, odometry scale,
        # lidar noise) still clears walls that the robot is driving along
        self.R_eff = self.R + float(cfg.get('lateral_margin_m', 0.03))
        self.stop_margin = float(cfg['stop_margin_m'])
        self.graze_r = self.R + 0.005        # a return already inside the corridor may be passed at least this far from the centre
        self.reset_state()

    # ------------------------------------------------------------------ state
    def reset_state(self):
        self.armed = False
        self.faults = {}               # latched hard faults: name -> (time, detail)
        self.session_end = 0.0
        self.last_hb = -1e9
        self.tether_origin = None
        self.P = np.zeros((0, 2))
        self.unknown_ang = np.zeros(0)
        self.n_beams = 0
        self.scan_rx = -1e9
        self.scan_count = 0
        self.contact_scans = 0
        self.odom = None               # (x, y, yaw, vx, vy, w)
        self.odom_rx = -1e9
        self.fw_flags = 0
        self.map_pose = None           # (x, y, yaw, cov_xx, cov_yy) at last AMCL update
        self.map_odom_at = None        # odom pose when the AMCL update arrived
        self.map_rx = -1e9
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_rx = -1e9
        self.last_out = (0.0, 0.0, 0.0)
        self.last_step = None
        self.prev_ranges = None
        self.frozen_scans = 0
        self.scan_dead_since = None
        self.loc_bad_since = None
        self.fence_out_since = None
        self.mm_base = None            # (odom_xy, odom_yaw, map_xy, map_yaw) baseline for motion-consistency check
        self.mm_bad = 0
        self.stats = {'limited_s': {}, 'estops': 0, 'steps': 0}

    def arm(self, now, lease_s):
        """Human/operator action.  Refuses while a hard fault is latched or inputs are missing."""
        if self.faults:
            return False, 'hard fault latched: %s' % sorted(self.faults)
        if self.odom is None or now - self.odom_rx > self.c['odom_stale_s']:
            return False, 'no fresh odometry'
        if self.scan_count == 0 or now - self.scan_rx > self.c['scan_stale_s']:
            return False, 'no fresh scan'
        if self.fw_flags & FW_HARD_MASK:
            return False, 'firmware reports fault flags %d' % self.fw_flags
        self.session_end = now + min(float(lease_s), float(self.c['max_session_s']))
        self.last_hb = now
        self.tether_origin = (self.odom[0], self.odom[1])
        self.mm_base = None
        self.mm_bad = 0
        self.last_out = (0.0, 0.0, 0.0)
        self.scan_dead_since = self.loc_bad_since = self.fence_out_since = None
        self.armed = True
        return True, 'armed until t=%.1f' % self.session_end

    def disarm(self, now, reason='operator'):
        self.armed = False

    def clear_faults(self, now):
        self.faults = {}

    def heartbeat(self, now):
        self.last_hb = now

    def _fault(self, now, name, detail=''):
        if name not in self.faults:
            self.faults[name] = (now, detail)
            self.stats['estops'] += 1

    # ------------------------------------------------------------------ inputs
    def on_scan(self, now, angle_min, angle_inc, ranges):
        r = np.asarray(ranges, dtype=float)
        n = len(r)
        # a driver that keeps republishing one scan looks fresh by timestamp; real scans never repeat exactly
        if self.prev_ranges is not None and len(self.prev_ranges) == n and np.array_equal(self.prev_ranges, r):
            self.frozen_scans += 1
        else:
            self.frozen_scans = 0
        self.prev_ranges = r
        if self.frozen_scans >= 6 and self.armed:
            self._fault(now, 'SCAN_DEAD', 'identical scan %d times in a row' % self.frozen_scans)
        ang = angle_min + angle_inc * np.arange(n)
        fin = np.isfinite(r)
        valid = fin & (r >= self.range_min) & (r <= self.range_max)
        near_self = valid & (r < self.self_r)
        valid &= ~near_self
        unknown = (~fin) | (r < self.range_min)
        a = ang[valid] + self.lidar_yaw
        P = np.stack([r[valid] * np.cos(a), r[valid] * np.sin(a)], axis=1) + self.lidar_off
        self.P = P
        self.unknown_ang = (ang[unknown] + self.lidar_yaw)
        self.n_beams = n
        self.scan_rx = now
        self.scan_count += 1
        if len(P):
            d = np.hypot(P[:, 0], P[:, 1])
            self.contact_scans = self.contact_scans + 1 if float(d.min()) < self.contact_r else 0
        else:
            self.contact_scans = 0
        if self.contact_scans >= 3 and self.armed:
            self._fault(now, 'CONTACT', 'scan return inside %.3f m for 3 scans' % self.contact_r)

    def on_odom(self, now, x, y, yaw, vx, vy, w, fw_flags=0):
        if self.odom is not None:
            if math.hypot(x - self.odom[0], y - self.odom[1]) > 0.25 and self.armed:
                self._fault(now, 'ODOM_JUMP', 'odometry jumped %.2f m' % math.hypot(x - self.odom[0], y - self.odom[1]))
        self.odom = (x, y, yaw, vx, vy, w)
        self.odom_rx = now
        self.fw_flags = int(fw_flags)
        if self.armed and (self.fw_flags & FW_HARD_MASK):
            self._fault(now, 'FW_FAULT', 'firmware flags %d' % self.fw_flags)

    def on_map_pose(self, now, x, y, yaw, cov_xx, cov_yy):
        self.map_pose = (x, y, yaw, cov_xx, cov_yy)
        self.map_odom_at = self.odom
        self.map_rx = now
        if self.odom is None:
            return
        # motion consistency between wheel odometry and scan matching: catches a reversed wheel, a lost map pose,
        # or a robot that is stuck/slipping while odometry keeps counting.
        base = self.mm_base
        cur = ((self.odom[0], self.odom[1]), self.odom[2], (x, y), yaw)
        if base is None:
            self.mm_base = cur
            return
        do = np.array(cur[0]) - np.array(base[0])
        if np.hypot(do[0], do[1]) < 0.30:
            return
        T = wrap(base[3] - base[1])
        cT, sT = math.cos(T), math.sin(T)
        exp_m = np.array([cT * do[0] - sT * do[1], sT * do[0] + cT * do[1]])
        dm = np.array(cur[2]) - np.array(base[2])
        err = float(np.hypot(*(dm - exp_m)))
        if err > max(0.15, 0.5 * float(np.hypot(do[0], do[1]))):
            self.mm_bad += 1
            if self.mm_bad >= 3 and self.armed:
                self._fault(now, 'MOTION_MISMATCH', 'scan-matching moved %.2f m, odometry says %.2f m (error %.2f)' % (
                    np.hypot(*dm), np.hypot(do[0], do[1]), err))
        else:
            self.mm_bad = 0
        self.mm_base = cur

    def set_clearance(self, centre_to_object_m):
        """Stop with the robot centre this far from a lidar object (the same number the zone editor keeps from walls).
        Never closer than 2 cm beyond the corridor circle, so the physical hull stays clear."""
        self.stop_margin = max(0.02, float(centre_to_object_m) - self.R_eff)

    def on_cmd(self, now, vx, vy, w):
        self.cmd = (vx, vy, w)
        self.cmd_rx = now

    # ------------------------------------------------------------------ geometry helpers
    def _scan_gap(self, u, extra=0.0):
        """Distance the robot circle can travel along unit vector u before touching a scan return (inf if clear).
        `extra` widens the corridor for the sideways drift of a path that curves because the robot rotates while moving."""
        P = self.P
        if len(P) == 0:
            return float('inf')
        along = P[:, 0] * u[0] + P[:, 1] * u[1]
        lat = -P[:, 0] * u[1] + P[:, 1] * u[0]
        Re = self.R_eff + extra
        inside = (along * along + lat * lat) < Re * Re
        if (inside & (along > 0.0) & (np.abs(lat) < self.graze_r)).any():
            return 0.0                                   # already inside the corridor circle and heading at it
        m = (~inside) & (along > 0.0) & (np.abs(lat) < Re)
        if not m.any():
            return float('inf')
        s = along[m] - np.sqrt(np.maximum(Re * Re - lat[m] ** 2, 0.0))
        return float(s.min())

    def _unknown_fraction(self, u, half=math.radians(15.0)):
        if self.n_beams == 0:
            return 1.0
        h = math.atan2(u[1], u[0])
        beams_in = max(1, int(round(self.n_beams * (2 * half) / (2 * math.pi))))
        if len(self.unknown_ang) == 0:
            return 0.0
        d = np.abs((self.unknown_ang - h + math.pi) % (2 * math.pi) - math.pi)
        return float((d < half).sum()) / beams_in

    def _tether_gap(self, u):
        if self.tether_origin is None or self.odom is None:
            return float('inf')
        px, py = self.odom[0] - self.tether_origin[0], self.odom[1] - self.tether_origin[1]
        Rt = float(self.c['tether_radius_m'])
        # solve |p + u s| = Rt
        b = px * u[0] + py * u[1]
        c = px * px + py * py - Rt * Rt
        disc = b * b - c
        if c >= 0.0:
            return 0.0 if b >= 0.0 else float('inf')
        return max(0.0, -b + math.sqrt(max(disc, 0.0)))

    def _map_pose_now(self):
        """AMCL pose advanced by the odometry travelled since that update; None if unusable (missing / stale)."""
        mp = self.map_pose
        if mp is None or self.odom is None or self.map_odom_at is None:
            return None
        o0, o1 = self.map_odom_at, self.odom
        dx, dy = o1[0] - o0[0], o1[1] - o0[1]
        if math.hypot(dx, dy) > self.c['amcl_travel_stale_m']:
            return None
        T = wrap(mp[2] - o0[2])
        cT, sT = math.cos(T), math.sin(T)
        return (mp[0] + cT * dx - sT * dy, mp[1] + sT * dx + cT * dy, wrap(mp[2] + wrap(o1[2] - o0[2])))

    def _fence_check(self, now):
        """Hard fault if the (usable) map pose is more than 15 cm (three map cells, > AMCL noise + quantisation) outside
        the zone for 1 s.  The soft fence in _fence_gap() already stops the robot AT the edge; this is the backstop."""
        if self.zone is None:
            return None
        pose = self._map_pose_now()
        if pose is None:
            return None
        if float(self.zone.value(np.array([pose[0]]), np.array([pose[1]]))[0]) < -0.15:
            self.fence_out_since = self.fence_out_since if self.fence_out_since is not None else now
            if now - self.fence_out_since > 1.0:
                self._fault(now, 'FENCE_BREACH', 'map pose %.2f, %.2f is outside the allowed zone' % (pose[0], pose[1]))
        else:
            self.fence_out_since = None
        return pose

    def _fence_gap(self, u, pose):
        """(gap along u to the zone edge, usable).  Without a usable map pose the zone cannot be checked."""
        if self.zone is None:
            return float('inf'), True
        if pose is None:
            return 0.0, False
        c, s = math.cos(pose[2]), math.sin(pose[2])
        um = (c * u[0] - s * u[1], s * u[0] + c * u[1])
        return self.zone.free_distance(pose[0], pose[1], um[0], um[1]), True

    def _t_lat(self, now):
        age = max(0.0, now - self.scan_rx)
        return float(self.c['latency_fixed_s']) + float(self.c['scan_time_s']) + age

    # ------------------------------------------------------------------ main step
    def step(self, now):
        c = self.c
        info = {'state': 'DISARMED', 'limit': '', 'faults': sorted(self.faults), 'cmd': self.cmd}
        dt = 0.05 if self.last_step is None else min(0.2, max(0.005, now - self.last_step))
        self.last_step = now
        self.stats['steps'] += 1
        zero = (0.0, 0.0, 0.0)

        def finish(out, limit):
            self.last_out = out
            info['limit'] = limit
            info['out'] = out
            if limit:
                lim = self.stats['limited_s']
                lim[limit] = lim.get(limit, 0.0) + dt
            return out, info

        if self.armed and now > self.session_end:
            self._fault(now, 'SESSION_TIMEOUT', 'session lease expired')
        if not self.armed:
            return finish(zero, 'disarmed')
        info['state'] = 'ARMED'
        if self.faults:
            info['state'] = 'FAULT'
            info['faults'] = sorted(self.faults)
            return finish(zero, 'fault')

        # ---- stale inputs -> stop (hard fault if the scan stays dead)
        scan_age = now - self.scan_rx
        info['scan_age'] = scan_age
        if scan_age > c['scan_stale_s']:
            self.scan_dead_since = self.scan_dead_since if self.scan_dead_since is not None else now
            if now - self.scan_dead_since > 3.0:
                self._fault(now, 'SCAN_DEAD', 'no scan for 3 s')
            return finish(zero, 'scan_stale')
        self.scan_dead_since = None
        if self.odom is None or now - self.odom_rx > c['odom_stale_s']:
            return finish(zero, 'odom_stale')
        if now - self.last_hb > c['heartbeat_timeout_s']:
            return finish(zero, 'heartbeat')
        if now - self.cmd_rx > c['cmd_stale_s']:
            return finish(zero, 'cmd_stale')

        # ---- tether hard limit
        px, py = self.odom[0] - self.tether_origin[0], self.odom[1] - self.tether_origin[1]
        if math.hypot(px, py) > c['tether_radius_m'] + c['tether_hard_extra_m']:
            self._fault(now, 'TETHER', 'odometry %.2f m from the arming point' % math.hypot(px, py))
            info['state'] = 'FAULT'
            return finish(zero, 'fault')

        pose = self._fence_check(now)
        if self.faults:
            info['state'] = 'FAULT'
            return finish(zero, 'fault')

        # ---- condition the command
        vx, vy, w = self.cmd
        limit = ''
        if not all(math.isfinite(v) for v in (vx, vy, w)):
            return finish(zero, 'bad_cmd')
        s = math.hypot(vx, vy)
        if s > c['v_max_m_s']:
            vx, vy, s, limit = vx * c['v_max_m_s'] / s, vy * c['v_max_m_s'] / s, c['v_max_m_s'], 'clamp'
        if abs(w) > c['w_max_rad_s']:
            w, limit = math.copysign(c['w_max_rad_s'], w), 'clamp'
        ws = wheel_speeds(vx, vy, w, c['wheel_azimuths_deg'], c['wheel_centre_radius_m'])
        mx = max(abs(v) for v in ws)
        if mx > c['wheel_cap_eff_m_s']:
            k = c['wheel_cap_eff_m_s'] / mx
            vx, vy, w, s, limit = vx * k, vy * k, w * k, s * k, 'clamp'

        # ---- acceleration limit (slowing is free)
        lx, ly, lw = self.last_out
        pure_slowing = (math.hypot(vx, vy) < math.hypot(lx, ly) - 1e-9) and (vx * lx + vy * ly >= 0.0)
        if not pure_slowing:
            dvx, dvy = vx - lx, vy - ly
            n = math.hypot(dvx, dvy)
            lim = c['a_acc_m_s2'] * dt
            if n > lim:
                vx, vy = lx + dvx * lim / n, ly + dvy * lim / n
                limit = limit or 'rate'
        if abs(w) > abs(lw) + 1e-9 or w * lw < 0:
            lim_w = c['a_acc_w_rad_s2'] * dt
            if abs(w - lw) > lim_w:
                w = lw + math.copysign(lim_w, w - lw)
                limit = limit or 'rate'
        s = math.hypot(vx, vy)

        # ---- environment limits for the commanded direction
        t_lat = self._t_lat(now)
        a_b = c['a_brake_m_s2']
        v_allow = float('inf')
        why = ''
        if s > 1e-3:
            u = (vx / s, vy / s)
            # translating while rotating bends the path: lateral drift ~ s w T^2 / 2 over the stopping horizon T
            horizon = t_lat + s / a_b
            gs = self._scan_gap(u, min(0.10, 0.5 * s * abs(w) * horizon * horizon))
            # something moving toward us matters only while we are moving (a standing robot cannot avoid it anyway)
            vo = c['dyn_obstacle_speed_m_s'] * min(1.0, s / 0.10)
            v_s = stop_safe_speed(gs, t_lat, a_b, self.stop_margin, vo)
            info['gap_scan'] = gs
            if v_s < v_allow:
                v_allow, why = v_s, 'obstacle'
            gt = self._tether_gap(u)
            v_t = stop_safe_speed(gt, t_lat, a_b, 0.10, 0.0)
            info['gap_tether'] = gt
            if v_t < v_allow:
                v_allow, why = v_t, 'tether'
            gf, usable = self._fence_gap(u, pose)
            info['gap_fence'] = gf
            if not usable:
                # localization not usable: only creep (the scan check above still protects walls)
                if self.loc_bad_since is None:
                    self.loc_bad_since = now
                if now - self.loc_bad_since > 15.0:
                    self._fault(now, 'LOCALIZATION_LOST', 'map pose unusable for 15 s')
                if self.zone is not None and c['unknown_creep_m_s'] < v_allow:
                    v_allow, why = c['unknown_creep_m_s'], 'no_map_pose'
            else:
                self.loc_bad_since = None
                cov = self.map_pose[3] + self.map_pose[4] if self.map_pose else 0.0
                if self.map_pose and math.sqrt(max(cov, 0.0)) > c['amcl_cov_limit_m'] and c['unknown_creep_m_s'] < v_allow:
                    v_allow, why = c['unknown_creep_m_s'], 'cov'
                v_f = stop_safe_speed(gf, t_lat, a_b, c.get('fence_margin_m', 0.05), 0.0)   # ~ one map cell + AMCL error
                if v_f < v_allow:
                    v_allow, why = v_f, 'fence'
            if self._unknown_fraction(u) > 0.5 and c['unknown_creep_m_s'] < v_allow:
                v_allow, why = c['unknown_creep_m_s'], 'unknown_sector'
            h = math.atan2(u[1], u[0])
            for a, hw in self.wedges:
                if abs(wrap(h - a)) < hw + math.radians(3.0) and c['blind_wedge_speed_cap_m_s'] < v_allow:
                    v_allow, why = c['blind_wedge_speed_cap_m_s'], 'blind_wedge'
            if v_allow < s:
                k = max(0.0, v_allow) / s
                vx, vy, w = vx * k, vy * k, w * k
                limit = why
            # the robot may already be moving in another direction (direction change): brake if that is unsafe
            ox, oy = self.odom[3], self.odom[4]
            so = math.hypot(ox, oy)
            if so > 0.03:
                uo = (ox / so, oy / so)
                go = self._scan_gap(uo)
                if so > stop_safe_speed(go, t_lat, a_b, 0.0, 0.0) + 0.02:
                    vx = vy = 0.0
                    w = 0.0
                    limit = 'brake_actual'
                    self.stats['estops'] += 1

        # ---- rotation: blocked only if something is inside the physical circle
        if abs(w) > 1e-3 and len(self.P):
            if float(np.hypot(self.P[:, 0], self.P[:, 1]).min()) < self.rot_block_r:
                w = 0.0
                limit = limit or 'rot_block'
        info['min_clear'] = (float(np.hypot(self.P[:, 0], self.P[:, 1]).min()) - self.R) if len(self.P) else float('inf')
        info['t_lat'] = t_lat
        if len(self.P):                                  # nearest return per 30 deg sector, base frame, 0 = front, counter-clockwise
            ang = np.degrees(np.arctan2(self.P[:, 1], self.P[:, 0])) % 360.0
            d = np.hypot(self.P[:, 0], self.P[:, 1])
            info['sector_m'] = [round(float(d[(ang // 30).astype(int) == k].min()), 2) if ((ang // 30).astype(int) == k).any() else None
                                for k in range(12)]
        return finish((vx, vy, w), limit)
