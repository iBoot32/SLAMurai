#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from functools import partial
import math
import time

class CmdVelGUI(Node):
    def __init__(self):
        super().__init__('cmdvel_gui')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_mode = 'text'
        self.button_vel = Twist()
        self.text_vel = Twist()
        self.keyboard_vel = Twist()

        self.key_percent = 0.5

        self.vel_limits = {
            'linear.x': [0.01, 0.35],
            'linear.y': [0.01, 0.35],
            'angular.z': [0.01, 1.75]
        }

        self.button_speeds = {
            'linear.x': 0.3,
            'linear.y': 0.3,
            'angular.z': 1.0
        }

        # --- debounce state ---
        self.keys_down = set()
        self.debounce = {}          # key -> after_id
        self.debounce_ms = 70       # debounce window

        self.root = tk.Tk()
        self.root.title("ROS2 CmdVel GUI")

        self.create_widgets()
        self.bind_keyboard()
        self.update_loop()
        self.root.mainloop()

    def create_widgets(self):
        self.entries = {}
        self.sliders = {}
        self.minmax_entries = {}

        for i, (label, attr) in enumerate([
            ('Forward/Back', 'linear.x'),
            ('Left/Right', 'linear.y'),
            ('Rotation', 'angular.z')
        ]):
            tk.Label(self.root, text=label).grid(row=i, column=0)

            self.entries[attr] = tk.Entry(self.root, width=5)
            self.entries[attr].grid(row=i, column=1)
            self.entries[attr].insert(0, "0.0")

            tk.Button(self.root, text="Stop",
                      command=partial(self.stop_text, attr)).grid(row=i, column=2)

            self.sliders[attr] = tk.Scale(
                self.root, from_=-1.0, to=1.0, resolution=0.01,
                orient=tk.HORIZONTAL, length=200,
                command=partial(self.slider_changed, attr)
            )
            self.sliders[attr].grid(row=i, column=3)

            tk.Label(self.root, text="min").grid(row=i, column=4)
            self.minmax_entries[attr+'_min'] = tk.Entry(self.root, width=5)
            self.minmax_entries[attr+'_min'].grid(row=i, column=5)
            self.minmax_entries[attr+'_min'].insert(0, str(self.vel_limits[attr][0]))

            tk.Label(self.root, text="max").grid(row=i, column=6)
            self.minmax_entries[attr+'_max'] = tk.Entry(self.root, width=5)
            self.minmax_entries[attr+'_max'].grid(row=i, column=7)
            self.minmax_entries[attr+'_max'].insert(0, str(self.vel_limits[attr][1]))

            tk.Button(self.root, text="Set",
                      command=partial(self.update_limits, attr)).grid(row=i, column=8)

        tk.Label(self.root, text="Keyboard % of max").grid(row=4, column=0)
        self.key_slider = tk.Scale(
            self.root, from_=0, to=1, resolution=0.01,
            orient=tk.HORIZONTAL, length=200,
            command=self.key_percent_changed
        )
        self.key_slider.set(self.key_percent)
        self.key_slider.grid(row=4, column=1, columnspan=3)

        self.canvas = tk.Canvas(self.root, width=250, height=250)
        self.canvas.grid(row=6, column=0, columnspan=9, pady=10)

        cx, cy, r = 125, 125, 80
        self.draw_quarter_button(cx, cy, r, 0, math.pi/2, 'Forward', 'linear.x', self.button_speeds['linear.x'])
        self.draw_quarter_button(cx, cy, r, math.pi/2, math.pi, 'Left', 'linear.y', self.button_speeds['linear.y'])
        self.draw_quarter_button(cx, cy, r, math.pi, 3*math.pi/2, 'Back', 'linear.x', -self.button_speeds['linear.x'])
        self.draw_quarter_button(cx, cy, r, 3*math.pi/2, 2*math.pi, 'Right', 'linear.y', -self.button_speeds['linear.y'])
        self.draw_circle_button(cx + r/math.sqrt(2), cy - r/math.sqrt(2), 20, 'CW', 'angular.z', -self.button_speeds['angular.z'])
        self.draw_circle_button(cx - r/math.sqrt(2), cy - r/math.sqrt(2), 20, 'CCW', 'angular.z', self.button_speeds['angular.z'])

    def key_percent_changed(self, val):
        self.key_percent = float(val)

    # --- keyboard with debounce ---
    def bind_keyboard(self):
        self.root.bind('<KeyPress>', self.on_key)
        self.root.bind('<KeyRelease>', self.on_key_release)

    def on_key(self, event):
        k = event.keysym.lower()
        self.current_mode = 'keyboard'

        # cancel pending debounce removal
        if k in self.debounce:
            self.root.after_cancel(self.debounce[k])
            del self.debounce[k]

        self.keys_down.add(k)
        self.update_keyboard_vel()

    def on_key_release(self, event):
        k = event.keysym.lower()
        if k in self.debounce:
            self.root.after_cancel(self.debounce[k])

        # schedule delayed removal
        self.debounce[k] = self.root.after(self.debounce_ms, lambda key=k: self.finish_release(key))

    def finish_release(self, key):
        if key in self.keys_down:
            self.keys_down.remove(key)
        if key in self.debounce:
            del self.debounce[key]
        self.update_keyboard_vel()

    def update_keyboard_vel(self):
        self.keyboard_vel = Twist()
        pct = self.key_percent

        max_vx = self.vel_limits['linear.x'][1]
        max_vy = self.vel_limits['linear.y'][1]
        max_wz = self.vel_limits['angular.z'][1]

        if 'w' in self.keys_down:
            self.keyboard_vel.linear.x = pct * max_vx
        if 's' in self.keys_down:
            self.keyboard_vel.linear.x = -pct * max_vx
        if 'a' in self.keys_down:
            self.keyboard_vel.linear.y = pct * max_vy
        if 'd' in self.keys_down:
            self.keyboard_vel.linear.y = -pct * max_vy
        if 'q' in self.keys_down:
            self.keyboard_vel.angular.z = pct * max_wz
        if 'e' in self.keys_down:
            self.keyboard_vel.angular.z = -pct * max_wz

    # drawing
    def draw_quarter_button(self, cx, cy, r, start, end, text, attr, val):
        x0, y0 = cx-r, cy-r
        x1, y1 = cx+r, cy+r
        arc = self.canvas.create_arc(
            x0, y0, x1, y1,
            start=math.degrees(start),
            extent=math.degrees(end-start),
            fill='lightgray'
        )
        self.canvas.tag_bind(arc, '<Button-1>', lambda e: self.button_move(attr, val))
        mid = (start+end)/2
        tx = cx + r/2 * math.cos(mid)
        ty = cy - r/2 * math.sin(mid)
        self.canvas.create_text(tx, ty, text=text)

    def draw_circle_button(self, cx, cy, radius, text, attr, val):
        oval = self.canvas.create_oval(
            cx-radius, cy-radius, cx+radius, cy+radius,
            fill='lightblue'
        )
        self.canvas.tag_bind(oval, '<Button-1>', lambda e: self.button_move(attr, val))
        self.canvas.create_text(cx, cy, text=text)

    def slider_changed(self, attr, val):
        val = float(val)
        minv, maxv = self.vel_limits[attr]
        if val > 0:
            v = minv + (maxv - minv) * val
        elif val < 0:
            v = -(minv + (maxv - minv) * abs(val))
        else:
            v = 0.0
        self.entries[attr].delete(0, tk.END)
        self.entries[attr].insert(0, str(v))
        self.set_text_mode(attr)

    def stop_text(self, attr):
        self.entries[attr].delete(0, tk.END)
        self.entries[attr].insert(0, "0.0")
        self.sliders[attr].set(0.0)
        self.set_text_mode(attr)

    def button_move(self, attr, val):
        self.current_mode = 'button'
        self.button_vel = Twist()
        if attr.startswith('linear'):
            setattr(self.button_vel.linear, attr.split('.')[1], val)
        else:
            setattr(self.button_vel.angular, attr.split('.')[1], val)

    def set_text_mode(self, attr):
        self.current_mode = 'text'
        for k in ['linear.x', 'linear.y', 'angular.z']:
            try:
                v = float(self.entries[k].get())
                minv, maxv = self.vel_limits[k]
                if abs(v) < minv:
                    v = 0.0
                elif v > 0:
                    v = min(v, maxv)
                else:
                    v = max(v, -maxv)

                if k.startswith('linear'):
                    setattr(self.text_vel.linear, k.split('.')[1], v)
                else:
                    setattr(self.text_vel.angular, k.split('.')[1], v)
            except:
                if k.startswith('linear'):
                    setattr(self.text_vel.linear, k.split('.')[1], 0.0)
                else:
                    setattr(self.text_vel.angular, k.split('.')[1], 0.0)

    def update_limits(self, attr):
        try:
            minv = float(self.minmax_entries[attr+'_min'].get())
            maxv = float(self.minmax_entries[attr+'_max'].get())
            self.vel_limits[attr] = [minv, maxv]
        except:
            pass

    def update_loop(self):
        if self.current_mode == 'text':
            msg = self.text_vel
        elif self.current_mode == 'button':
            msg = self.button_vel
        else:
            msg = self.keyboard_vel

        self.pub.publish(msg)
        self.root.after(50, self.update_loop)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
