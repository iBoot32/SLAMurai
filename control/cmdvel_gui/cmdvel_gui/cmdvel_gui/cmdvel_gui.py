#!/usr/bin/env python3
"""Drive the robot by hand: on-screen joystick, keyboard, or exact values.
Publishes /cmd_vel (the safety gate sits behind it). Every input falls back to zero when released."""
import math
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

BG, PANEL, FG, DIM = '#1e2127', '#2a2f38', '#e6e6e6', '#8b93a1'
ACCENT, STOP, OK, WARN = '#4da3ff', '#d9434e', '#58c27d', '#e0a030'
JOY_R, PUCK_R, DEAD = 100, 22, 0.06
KEYS = {'w': (1, 0, 0), 'up': (1, 0, 0), 's': (-1, 0, 0), 'down': (-1, 0, 0),
        'a': (0, 1, 0), 'left': (0, 1, 0), 'd': (0, -1, 0), 'right': (0, -1, 0),
        'q': (0, 0, 1), 'e': (0, 0, -1)}


class CmdVelGUI(Node):
    def __init__(self):
        super().__init__('cmdvel_gui')
        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('feedback_topic', '/cmd_vel_safe')
        self.declare_parameter('max_linear', 0.21)
        self.declare_parameter('max_angular', 0.5)
        self.max_lin = float(self.get_parameter('max_linear').value)
        self.max_ang = float(self.get_parameter('max_angular').value)
        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(Twist, topic, 10)
        self.safe, self.safe_t = (0.0, 0.0, 0.0), 0.0
        self.create_subscription(Twist, self.get_parameter('feedback_topic').value, self.on_safe, 10)

        self.joy = (0.0, 0.0)          # unit joystick vector (right, up)
        self.joy_on = False
        self.turn = 0.0                # -1..1, positive = counter-clockwise
        self.turn_on = False
        self.keys, self.release_job = set(), {}
        self.exact = None
        self.cmd = (0.0, 0.0, 0.0)

        self.root = tk.Tk()
        self.root.title('Robot drive')
        self.root.configure(bg=BG)
        self.root.resizable(False, False)
        self.build(topic)
        self.root.bind('<KeyPress>', self.key_down)
        self.root.bind('<KeyRelease>', self.key_up)
        self.root.bind('<FocusOut>', lambda e: self.stop(clear_exact=False))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()
        self.root.mainloop()

    # ---- layout
    def build(self, topic):
        st = ttk.Style(self.root)
        st.theme_use('clam')
        st.configure('TFrame', background=BG)
        st.configure('TLabel', background=BG, foreground=FG, font=('Segoe UI', 10))
        st.configure('Dim.TLabel', foreground=DIM)
        st.configure('Big.TLabel', font=('Consolas', 11))
        st.configure('TScale', background=BG)
        left = ttk.Frame(self.root, padding=14)
        left.grid(row=0, column=0)
        right = ttk.Frame(self.root, padding=(0, 14, 14, 14))
        right.grid(row=0, column=1, sticky='n')

        ttk.Label(left, text='Move', style='Dim.TLabel').pack(anchor='w')
        size = 2 * JOY_R + 20
        self.jc = tk.Canvas(left, width=size, height=size, bg=BG, highlightthickness=0, cursor='hand2')
        self.jc.pack()
        c = size // 2
        self.jc.create_oval(c - JOY_R, c - JOY_R, c + JOY_R, c + JOY_R, fill=PANEL, outline='#3a414d', width=2)
        self.jc.create_line(c, c - JOY_R, c, c + JOY_R, fill='#3a414d')
        self.jc.create_line(c - JOY_R, c, c + JOY_R, c, fill='#3a414d')
        for t, x, y in (('FWD', c, c - JOY_R + 12), ('BACK', c, c + JOY_R - 12), ('L', c - JOY_R + 12, c), ('R', c + JOY_R - 12, c)):
            self.jc.create_text(x, y, text=t, fill=DIM, font=('Segoe UI', 8))
        self.puck = self.jc.create_oval(c - PUCK_R, c - PUCK_R, c + PUCK_R, c + PUCK_R, fill=ACCENT, outline='')
        self.jc.bind('<Button-1>', self.joy_drag)
        self.jc.bind('<B1-Motion>', self.joy_drag)
        self.jc.bind('<ButtonRelease-1>', self.joy_release)

        ttk.Label(left, text='Turn', style='Dim.TLabel').pack(anchor='w', pady=(10, 0))
        self.tc = tk.Canvas(left, width=size, height=46, bg=BG, highlightthickness=0, cursor='hand2')
        self.tc.pack()
        self.tc.create_rectangle(10, 14, size - 10, 32, fill=PANEL, outline='#3a414d', width=2)
        self.tc.create_text(24, 40, text='CCW', fill=DIM, font=('Segoe UI', 8))
        self.tc.create_text(size - 24, 40, text='CW', fill=DIM, font=('Segoe UI', 8))
        self.tpuck = self.tc.create_oval(size // 2 - 14, 5, size // 2 + 14, 41, fill=ACCENT, outline='')
        self.tc.bind('<Button-1>', self.turn_drag)
        self.tc.bind('<B1-Motion>', self.turn_drag)
        self.tc.bind('<ButtonRelease-1>', self.turn_release)

        ttk.Label(right, text='Speed limit', style='Dim.TLabel').pack(anchor='w')
        self.speed = tk.DoubleVar(value=0.5)
        ttk.Scale(right, from_=0.05, to=1.0, variable=self.speed, length=230).pack(anchor='w')
        self.speed_lbl = ttk.Label(right, style='Dim.TLabel')
        self.speed_lbl.pack(anchor='w', pady=(0, 10))

        tk.Button(right, text='STOP  (space)', command=self.stop, bg=STOP, fg='white', activebackground='#b8323c',
                  activeforeground='white', font=('Segoe UI', 14, 'bold'), relief='flat', height=2, width=18).pack(pady=(0, 12))

        box = tk.Frame(right, bg=PANEL, padx=10, pady=8)
        box.pack(fill='x')
        self.sent = tk.Label(box, bg=PANEL, fg=FG, font=('Consolas', 10), justify='left', anchor='w')
        self.sent.pack(fill='x')
        self.got = tk.Label(box, bg=PANEL, fg=DIM, font=('Consolas', 10), justify='left', anchor='w')
        self.got.pack(fill='x')
        self.note = tk.Label(box, bg=PANEL, fg=OK, font=('Segoe UI', 9), anchor='w')
        self.note.pack(fill='x', pady=(4, 0))

        ttk.Label(right, text='Exact values', style='Dim.TLabel').pack(anchor='w', pady=(12, 0))
        row = ttk.Frame(right)
        row.pack(anchor='w')
        self.ent = []
        for name, unit in (('fwd', 'm/s'), ('left', 'm/s'), ('turn', 'rad/s')):
            ttk.Label(row, text=name).grid(row=0, column=len(self.ent) * 2)
            e = tk.Entry(row, width=5, bg=PANEL, fg=FG, insertbackground=FG, relief='flat')
            e.insert(0, '0.0')
            e.grid(row=0, column=len(self.ent) * 2 + 1, padx=(2, 8))
            self.ent.append(e)
        tk.Button(right, text='Drive these until stop', command=self.drive_exact, bg=PANEL, fg=FG, relief='flat').pack(anchor='w', pady=4)

        ttk.Label(right, text='Keys: W A S D or arrows move, Q E turn, Space or Esc stops.\nDrag the joystick. Letting go stops the robot.\nCommands go to %s through the safety gate.' % topic,
                  style='Dim.TLabel', justify='left').pack(anchor='w', pady=(10, 0))

    # ---- inputs
    def joy_drag(self, e):
        c = (2 * JOY_R + 20) // 2
        dx, dy = (e.x - c) / float(JOY_R), (c - e.y) / float(JOY_R)
        n = math.hypot(dx, dy)
        if n > 1:
            dx, dy = dx / n, dy / n
        self.joy, self.joy_on, self.exact = (dx, dy), True, None
        self.jc.coords(self.puck, c + dx * JOY_R - PUCK_R, c - dy * JOY_R - PUCK_R, c + dx * JOY_R + PUCK_R, c - dy * JOY_R + PUCK_R)

    def joy_release(self, e=None):
        c = (2 * JOY_R + 20) // 2
        self.joy, self.joy_on = (0.0, 0.0), False
        self.jc.coords(self.puck, c - PUCK_R, c - PUCK_R, c + PUCK_R, c + PUCK_R)

    def turn_drag(self, e):
        w = 2 * JOY_R + 20
        t = max(-1.0, min(1.0, (w // 2 - e.x) / float(w // 2 - 24)))
        self.turn, self.turn_on, self.exact = t, True, None
        x = w // 2 - t * (w // 2 - 24)
        self.tc.coords(self.tpuck, x - 14, 5, x + 14, 41)

    def turn_release(self, e=None):
        w = 2 * JOY_R + 20
        self.turn, self.turn_on = 0.0, False
        self.tc.coords(self.tpuck, w // 2 - 14, 5, w // 2 + 14, 41)

    def key_down(self, e):
        k = e.keysym.lower()
        if k in ('space', 'escape'):
            return self.stop()
        if k in KEYS:
            if k in self.release_job:                      # key repeat: cancel the pending release
                self.root.after_cancel(self.release_job.pop(k))
            self.keys.add(k)
            self.exact = None

    def key_up(self, e):
        k = e.keysym.lower()
        if k in KEYS:
            if k in self.release_job:
                self.root.after_cancel(self.release_job[k])
            self.release_job[k] = self.root.after(70, lambda: (self.keys.discard(k), self.release_job.pop(k, None)))

    def drive_exact(self):
        try:
            v = [float(x.get()) for x in self.ent]
        except ValueError:
            self.note.config(text='Exact values must be numbers.', fg=WARN)
            return
        self.exact = (max(-self.max_lin, min(self.max_lin, v[0])), max(-self.max_lin, min(self.max_lin, v[1])),
                      max(-self.max_ang, min(self.max_ang, v[2])))

    def stop(self, clear_exact=True):
        self.keys.clear()
        if clear_exact:
            self.exact = None
        self.joy_release()
        self.turn_release()

    def on_safe(self, m):
        self.safe, self.safe_t = (m.linear.x, m.linear.y, m.angular.z), time.monotonic()

    # ---- loop
    def command(self):
        s = self.speed.get()
        if self.joy_on or self.turn_on:
            jx, jy = self.joy
            n = math.hypot(jx, jy)
            if n < DEAD:
                jx = jy = 0.0
            return (jy * s * self.max_lin, -jx * s * self.max_lin, (self.turn if abs(self.turn) > DEAD else 0.0) * s * self.max_ang)
        if self.keys:
            x = sum(KEYS[k][0] for k in self.keys)
            y = sum(KEYS[k][1] for k in self.keys)
            w = sum(KEYS[k][2] for k in self.keys)
            n = math.hypot(x, y)
            if n > 1:
                x, y = x / n, y / n
            return (max(-1, min(1, x)) * s * self.max_lin, max(-1, min(1, y)) * s * self.max_lin, max(-1, min(1, w)) * s * self.max_ang)
        return self.exact or (0.0, 0.0, 0.0)

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.cmd = self.command()
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = self.cmd
        self.pub.publish(m)
        self.speed_lbl.config(text='%d%% of  %.2f m/s, %.2f rad/s' % (self.speed.get() * 100, self.max_lin, self.max_ang))
        self.sent.config(text='You send   fwd %+.2f  left %+.2f  turn %+.2f' % self.cmd)
        fresh = time.monotonic() - self.safe_t < 1.0
        if fresh:
            self.got.config(text='Robot gets fwd %+.2f  left %+.2f  turn %+.2f' % self.safe)
        else:
            self.got.config(text='Robot gets  no data')
        moving = any(abs(v) > 1e-3 for v in self.cmd)
        if not fresh:
            self.note.config(text='No safety gate output. Is it running?', fg=WARN)
        elif moving and any(abs(a - b) > 0.03 for a, b in zip(self.cmd, self.safe)):
            self.note.config(text='Safety gate is limiting the motion.', fg=WARN)
        else:
            self.note.config(text='Ready' if not moving else 'Driving', fg=OK)
        self.root.after(50, self.tick)

    def close(self):
        for _ in range(5):
            self.pub.publish(Twist())
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGUI()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
