"""A minimal rosbridge server that behaves like a Nav2 robot, driven by the same profile the client uses.
A point robot moves straight to the goal at the speed set in the (restarted) Nav2 parameters. For wire tests only."""
import asyncio
import json
import math
import threading
import time

import websockets
import yaml
from websockets.asyncio.server import serve


class FakeRobot(object):
    TIME_SCALE = 10.0                       # simulated motion runs this much faster than the speed parameter says

    def __init__(self, profile, offset=(1.0, 2.0)):
        self.p, self.offset = profile, offset
        self.x = self.y = self.yaw = 0.0   # odom frame; map = odom + offset
        self.goal, self.vx, self.vy = None, 0.0, 0.0
        self.speed, self.heartbeats = 0.2, 0
        self.restart = {'id': 0, 'state': 'idle', 'msg': ''}
        self.clients, self.pending, self.sent_goals = {}, [], []
        self.stop_flag, self.ready, self.port = False, threading.Event(), None
        self.thread = threading.Thread(target=lambda: asyncio.run(self._main()), daemon=True)

    def start(self):
        self.thread.start()
        self.ready.wait(10)
        return self.port

    def stop(self):
        self.stop_flag = True
        self.thread.join(5)

    async def _main(self):
        async def handler(ws):
            self.clients[ws] = set()
            try:
                async for raw in ws:
                    m = json.loads(raw)
                    if m.get('op') == 'subscribe':
                        self.clients[ws].add(m['topic'])
                    elif m.get('op') == 'publish':
                        self._on_publish(m['topic'], m.get('msg', {}))
            finally:
                self.clients.pop(ws, None)
        async with serve(handler, '127.0.0.1', 0) as server:
            self.port = server.sockets[0].getsockname()[1]
            self.ready.set()
            tick = 0
            while not self.stop_flag:
                tick += 1
                await self._step(0.02, tick)
                await asyncio.sleep(0.02)

    def _on_publish(self, topic, msg):
        if topic == self.p['goal']['topic']:
            q, pos = msg['pose']['orientation'], msg['pose']['position']
            self.goal = (pos['x'] - self.offset[0], pos['y'] - self.offset[1])
            self.sent_goals.append((pos['x'], pos['y'], 2 * math.atan2(q['z'], q['w'])))
            self.new_plan = True
        elif topic == self.p['heartbeat']['topic']:
            self.heartbeats += 1
        elif topic == self.p['restart']['topic']:
            req = json.loads(msg['data'])
            params = yaml.safe_load(req['yaml'])
            speed = params.get('dwb_controller', {}).get('ros__parameters', {}).get('max_speed_xy', 0.2)
            if speed > 1.0:
                self.restart = {'id': req['id'], 'state': 'error', 'msg': 'refused: too fast'}
            else:
                self.restart = {'id': req['id'], 'state': 'starting', 'msg': ''}
                self.pending.append((time.monotonic() + 0.3, req['id'], speed))

    async def _send(self, topic, msg):
        for ws, subs in list(self.clients.items()):
            if topic in subs:
                try:
                    await ws.send(json.dumps({'op': 'publish', 'topic': topic, 'msg': msg}))
                except Exception:
                    pass

    async def _step(self, dt, tick):
        for item in list(self.pending):
            if time.monotonic() >= item[0]:
                self.pending.remove(item)
                self.speed = item[2]
                self.restart = {'id': item[1], 'state': 'active', 'msg': ''}
        self.vx = self.vy = 0.0
        if self.goal:
            dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
            d = math.hypot(dx, dy)
            if d < 0.02:
                self.goal = None
            else:
                v = min(self.speed * self.TIME_SCALE, d / dt)
                self.vx, self.vy = v * dx / d, v * dy / d
                self.x, self.y = self.x + self.vx * dt, self.y + self.vy * dt
        s, st = self.p['streams'], self.p['status']
        twist = {'linear': {'x': self.vx, 'y': self.vy, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}
        if tick % 5 == 0:
            await self._send(s['odom']['topic'], {
                'pose': {'pose': {'position': {'x': self.x, 'y': self.y, 'z': 0}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}},
                'twist': {'twist': {'linear': {'x': self.vx, 'y': self.vy, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}}})
            await self._send(s['cmd_in']['topic'], twist)
            await self._send(s['cmd_out']['topic'], twist)
            pc = self.p['pose']
            tf = lambda par, ch, x, y: {'header': {'frame_id': par}, 'child_frame_id': ch, 'transform': {
                'translation': {'x': x, 'y': y, 'z': 0.0}, 'rotation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}}
            await self._send(s['tf']['topic'], {'transforms': [
                tf(pc['map_frame'], pc['odom_frame'], *self.offset), tf(pc['odom_frame'], pc['base_frames'][0], self.x, self.y)]})
        if getattr(self, 'new_plan', False) and self.goal:
            self.new_plan = False
            pts = [(self.x + (self.goal[0] - self.x) * k / 10.0 + self.offset[0], self.y + (self.goal[1] - self.y) * k / 10.0 + self.offset[1])
                   for k in range(11)]
            await self._send(s['plan']['topic'], {'poses': [{'pose': {'position': {'x': a, 'y': b, 'z': 0}}} for a, b in pts]})
        if tick % 10 == 0:
            await self._send(s['status']['topic'], {'data': json.dumps({st['faults']: [], st['armed']: True, st['clearance']: 1.5, st['limit']: ''})})
            await self._send(self.p['restart']['status_topic'], {'data': json.dumps(self.restart)})
