#!/usr/bin/env python3
"""Zone editor: paint where the robot may drive, live. Started by the launch when safety:=true and a display exists.

Modes: ALLOW paints regions the robot may enter (if any exist, only they count); KEEP-OUT paints regions it must never enter.
Tools: brush, rectangle, polygon (click points, Enter or double-click to close, Esc to cancel), eraser. Every change is
checked and sent to the gate at once (/safety/zone) and saved to ~/.slamurai/zone_live.npz, which the gate restores at start.
A change is not sent while the robot would be outside the new zone, so the gate cannot be tripped by an edit.
The gate keeps a 0.30 m clearance from walls and unknown cells by default (slider)."""
import os
import sys
import time

import numpy as np
import rclpy
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageDraw
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 'safety')]      # installed, both sit in one directory
from zone_build import compose, zone_from_allowed   # noqa: E402

SAVE = os.path.expanduser('~/.slamurai/zone_live.npz')
VIEW = os.path.expanduser('~/.slamurai/zone_view')   # view rotation in quarter turns
SHOW = 760                                          # longest side of the map on screen, pixels


class ZoneEditor(Node):
    def __init__(self):
        super().__init__('zone_editor')
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/safety/zone', latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 5)
        self.free = self.paint = self.meta = None
        self.pose, self.undo, self.shown, self.pending = None, [], None, None
        self.drag, self.poly, self.last_draw = None, [], 0.0
        try:
            self.k = int(open(VIEW).read()) % 4
        except Exception:
            self.k = 1                       # 1: map x points up, y left, like RViz's default top-down view

        self.root = tk.Tk()
        self.root.title('Safety zone')
        self.canvas = tk.Canvas(self.root, width=SHOW, height=SHOW, bg='#222', highlightthickness=0)
        self.canvas.grid(row=0, column=0, padx=8, pady=8)
        side = ttk.Frame(self.root, padding=8)
        side.grid(row=0, column=1, sticky='n')
        self.mode, self.tool = tk.StringVar(value='allow'), tk.StringVar(value='brush')
        self.size, self.clear = tk.DoubleVar(value=0.30), tk.DoubleVar(value=0.30)
        self.anyway = tk.BooleanVar(value=False)
        ttk.Label(side, text='Paint').pack(anchor='w')
        for t, v in (('Allowed (green)', 'allow'), ('Keep-out (red)', 'keepout')):
            ttk.Radiobutton(side, text=t, variable=self.mode, value=v).pack(anchor='w')
        ttk.Label(side, text='Tool').pack(anchor='w', pady=(8, 0))
        for t, v in (('Brush', 'brush'), ('Rectangle', 'rect'), ('Polygon', 'poly'), ('Eraser', 'erase')):
            ttk.Radiobutton(side, text=t, variable=self.tool, value=v, command=self.cancel_poly).pack(anchor='w')
        ttk.Label(side, text='Brush radius (m)').pack(anchor='w', pady=(8, 0))
        tk.Scale(side, variable=self.size, from_=0.05, to=1.5, resolution=0.05, orient='horizontal', length=170).pack(anchor='w')
        ttk.Label(side, text='Wall clearance (m)').pack(anchor='w')
        sp = tk.Spinbox(side, textvariable=self.clear, from_=0.0, to=2.0, increment=0.05, format='%.2f', width=8,
                        command=self.changed)
        sp.pack(anchor='w')
        sp.bind('<Return>', lambda e: self.changed())
        sp.bind('<FocusOut>', lambda e: self.changed())
        ttk.Checkbutton(side, text='Apply even if the robot is outside', variable=self.anyway).pack(anchor='w', pady=(8, 0))
        for t, f in (('Rotate view', self.rotate), ('Undo', self.do_undo), ('Clear paint', self.clear_paint), ('No zone (whole map)', self.no_zone)):
            ttk.Button(side, text=t, command=f).pack(anchor='w', fill='x', pady=2)
        self.info = tk.Label(side, text='waiting for /map ...', justify='left', wraplength=190, anchor='w')
        self.info.pack(anchor='w', pady=8)
        c = self.canvas
        c.bind('<ButtonPress-1>', self.press)
        c.bind('<B1-Motion>', self.motion)
        c.bind('<ButtonRelease-1>', self.release)
        c.bind('<Double-Button-1>', lambda e: self.close_poly())
        self.root.bind('<Return>', lambda e: self.close_poly())
        self.root.bind('<Escape>', lambda e: self.cancel_poly())
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.alive = True
        self.root.after(50, self.tick)

    # ---- ROS
    def on_map(self, m):
        w, h = m.info.width, m.info.height
        meta = (w, h, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
        if meta == self.meta:
            return
        self.meta = meta
        d = np.asarray(m.data, dtype=np.int8).reshape(h, w)       # row 0 = lowest y
        self.free = d == 0
        self.base = np.zeros((h, w, 3), np.uint8)
        self.base[d < 0] = (60, 60, 60)
        self.base[d == 0] = (235, 235, 235)
        self.base[d > 0] = (0, 0, 0)
        self.paint = np.zeros((h, w), np.uint8)
        try:
            s = np.load(SAVE)
            if s['paint'].shape == self.paint.shape and abs(float(s['resolution']) - meta[2]) < 1e-6:
                self.paint = s['paint'].copy()
        except Exception:
            pass
        self.s = SHOW / float(max(w, h))
        self.draw()
        if self.paint.any():
            self.changed()

    def on_pose(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def tick(self):
        if not self.alive:
            return
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0)
        self.show_robot()
        self.root.after(50, self.tick)

    # ---- geometry. Upright = x right, y up (the stored grid). The screen shows it turned k quarter turns CCW.
    def dims(self):
        return int(self.meta[0] * self.s), int(self.meta[1] * self.s)

    def up(self, x, y):
        Wu, Hu = self.dims()
        return [(x, y), (Wu - y, x), (Wu - x, Hu - y), (y, Hu - x)][self.k]

    def down(self, x, y):
        Wu, Hu = self.dims()
        return [(x, y), (y, Wu - x), (Wu - x, Hu - y), (Hu - y, x)][self.k]

    def cell(self, x, y):
        w, h = self.meta[0], self.meta[1]
        ux, uy = self.up(x, y)
        return min(max(int(ux / self.s), 0), w - 1), min(max(int(h - 1 - uy / self.s), 0), h - 1)

    def to_px(self, x, y):
        w, h, res, ox, oy = self.meta
        return self.down((x - ox) / res * self.s, (h - (y - oy) / res) * self.s)

    def rotate(self):
        self.k = (self.k + 1) % 4
        try:
            os.makedirs(os.path.dirname(VIEW), exist_ok=True)
            open(VIEW, 'w').write(str(self.k))
        except Exception:
            pass
        self.cancel_poly()
        self.draw()

    # ---- painting
    def value(self):
        return 0 if self.tool.get() == 'erase' else (1 if self.mode.get() == 'allow' else 2)

    def push_undo(self):
        self.undo.append(self.paint.copy())
        del self.undo[:-20]

    def disc(self, j, i):
        r = self.size.get() / self.meta[2]
        h, w = self.paint.shape
        i0, i1, j0, j1 = max(int(i - r), 0), min(int(i + r) + 1, h), max(int(j - r), 0), min(int(j + r) + 1, w)
        ii, jj = np.ogrid[i0:i1, j0:j1]
        self.paint[i0:i1, j0:j1][(ii - i) ** 2 + (jj - j) ** 2 <= r * r] = self.value()

    def press(self, e):
        if self.paint is None:
            return
        j, i = self.cell(e.x, e.y)
        t = self.tool.get()
        if t in ('brush', 'erase'):
            self.push_undo()
            self.disc(j, i)
            self.drag = 'brush'
        elif t == 'rect':
            self.drag = (e.x, e.y)
            self.rect_id = self.canvas.create_rectangle(e.x, e.y, e.x, e.y, outline='#ffd400', width=2)
        elif t == 'poly':
            self.poly.append((e.x, e.y))
            self.canvas.create_oval(e.x - 3, e.y - 3, e.x + 3, e.y + 3, fill='#ffd400', tags='poly')
            if len(self.poly) > 1:
                self.canvas.create_line(self.poly[-2] + self.poly[-1], fill='#ffd400', width=2, tags='poly')

    def motion(self, e):
        if self.drag == 'brush':
            self.disc(*self.cell(e.x, e.y))
            if time.monotonic() - self.last_draw > 0.25:
                self.draw()
        elif self.drag:
            self.canvas.coords(self.rect_id, self.drag[0], self.drag[1], e.x, e.y)

    def release(self, e):
        if self.drag == 'brush':
            self.drag = None
            self.changed()
        elif self.drag:
            x0, y0 = self.drag
            self.canvas.delete(self.rect_id)
            self.drag = None
            self.push_undo()
            (j0, i0), (j1, i1) = self.cell(x0, y0), self.cell(e.x, e.y)
            self.paint[min(i0, i1):max(i0, i1) + 1, min(j0, j1):max(j0, j1) + 1] = self.value()
            self.changed()

    def close_poly(self):
        if len(self.poly) >= 3:
            self.push_undo()
            h, w = self.paint.shape
            im = Image.new('L', (w, h), 0)
            ImageDraw.Draw(im).polygon([tuple(v / self.s for v in self.up(x, y)) for x, y in self.poly], fill=1)
            m = np.array(im)[::-1].astype(bool)                   # screen rows go down, map rows go up
            self.paint[m] = self.value()
            self.changed()
        self.cancel_poly()

    def cancel_poly(self):
        self.poly = []
        self.canvas.delete('poly')

    def do_undo(self):
        if self.undo:
            self.paint = self.undo.pop()
            self.changed()

    def clear_paint(self):
        if self.paint is not None:
            self.push_undo()
            self.paint[:] = 0
            self.changed()

    def no_zone(self):
        """Whole map allowed: the scan still protects, only the fence is gone."""
        if self.paint is not None:
            self.push_undo()
            self.paint[:] = 0
            self.paint[self.free] = 1
            self.send(self.paint == 1, 'whole map')
            self.draw()

    # ---- apply + draw
    def allowed(self):
        return compose(self.free, self.paint, self.meta[2], self.clear.get())

    def changed(self):
        if self.paint is None:
            return
        try:
            self.clear.get()
        except tk.TclError:
            return
        self.draw()
        al = self.allowed()
        if not al.any():
            self.info.config(text='Nothing is allowed here: not sent.', fg='#b00020')
            return
        if self.pose and not self.anyway.get():
            z = zone_from_allowed(al, *self.meta[2:])
            if float(z.value(np.array([self.pose[0]]), np.array([self.pose[1]]))[0]) < 0.0:
                self.info.config(text='The robot would be outside this zone: not sent. Move the paint or tick "Apply even if ...".',
                                 fg='#b00020')
                return
        self.send(al, 'zone')

    def send(self, al, what):
        w, h, res, ox, oy = self.meta
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution, m.info.width, m.info.height = res, w, h
        m.info.origin.position.x, m.info.origin.position.y = ox, oy
        m.info.origin.orientation.w = 1.0
        m.data = (al.astype(np.int8) * 100).ravel().tolist()
        self.pub.publish(m)
        try:
            os.makedirs(os.path.dirname(SAVE), exist_ok=True)
            np.savez_compressed(SAVE, allowed=al, paint=self.paint, resolution=res, origin_x=ox, origin_y=oy)
        except Exception:
            pass
        self.info.config(text='Sent to the gate: %s, %.1f m2 allowed.' % (what, al.sum() * res * res), fg='#1b5e20')

    def draw(self):
        if self.meta is None:
            return
        self.last_draw = time.monotonic()
        img = self.base.astype(np.float32)
        al = self.allowed()
        for mask, col, a in ((self.free & ~al, (255, 90, 60), 0.35), (self.paint == 1, (40, 200, 80), 0.40),
                             (self.paint == 2, (220, 20, 20), 0.55)):
            img[mask] = img[mask] * (1 - a) + np.array(col, np.float32) * a
        w, h = self.meta[0], self.meta[1]
        Wu, Hu = self.dims()
        arr = np.ascontiguousarray(np.rot90(img.astype(np.uint8)[::-1], self.k))
        im = Image.fromarray(arr, 'RGB').resize((Hu, Wu) if self.k % 2 else (Wu, Hu))
        self.canvas.config(width=im.size[0], height=im.size[1])
        self.photo = tk.PhotoImage(data=b'P6 %d %d 255\n' % im.size + im.tobytes())
        self.canvas.delete('map')
        self.canvas.create_image(0, 0, image=self.photo, anchor='nw', tags='map')
        self.canvas.tag_lower('map')

    def show_robot(self):
        if self.meta is None or self.pose is None:
            return
        x, y = self.to_px(*self.pose)
        self.canvas.delete('robot')
        self.canvas.create_oval(x - 6, y - 6, x + 6, y + 6, fill='#1565c0', outline='white', width=2, tags='robot')

    def close(self):
        self.alive = False
        self.root.destroy()


def main():
    rclpy.init()
    node = ZoneEditor()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
