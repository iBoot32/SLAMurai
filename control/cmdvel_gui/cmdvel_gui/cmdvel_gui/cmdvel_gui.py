#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from functools import partial
import math

class CmdVelGUI(Node):
    def __init__(self):
        super().__init__('cmdvel_gui')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_mode = 'text'  # 'text' or 'button'
        self.button_vel = Twist()
        self.text_vel = Twist()

        # Default min/max velocities
        self.vel_limits = {
            'linear.x': [0.1, 0.35],
            'linear.y': [0.1, 0.35],
            'angular.z': [0.1, 1.75]
        }

        # Default button speeds
        self.button_speeds = {
            'linear.x': 0.3,
            'linear.y': 0.3,
            'angular.z': 1.0
        }

        # GUI setup
        self.root = tk.Tk()
        self.root.title("ROS2 Dashing CmdVel GUI")

        self.create_widgets()
        self.update_loop()
        self.root.mainloop()

    def create_widgets(self):
        # --- Text boxes + sliders + min/max entries ---
        self.entries = {}
        self.sliders = {}
        self.minmax_entries = {}
        for i, (label, attr) in enumerate([('Forward/Back', 'linear.x'),
                                           ('Left/Right', 'linear.y'),
                                           ('Rotation', 'angular.z')]):
            tk.Label(self.root, text=label).grid(row=i, column=0)
            self.entries[attr] = tk.Entry(self.root, width=5)
            self.entries[attr].grid(row=i, column=1)
            self.entries[attr].insert(0, "0.0")
            tk.Button(self.root, text="Stop", command=partial(self.stop_text, attr)).grid(row=i, column=2)
            self.sliders[attr] = tk.Scale(self.root,
                                          from_=-1.0, to=1.0, resolution=0.01,
                                          orient=tk.HORIZONTAL, length=200,
                                          command=partial(self.slider_changed, attr))
            self.sliders[attr].grid(row=i, column=3)
            # min/max entries
            tk.Label(self.root, text="min").grid(row=i, column=4)
            self.minmax_entries[attr+'_min'] = tk.Entry(self.root, width=5)
            self.minmax_entries[attr+'_min'].grid(row=i, column=5)
            self.minmax_entries[attr+'_min'].insert(0, str(self.vel_limits[attr][0]))
            tk.Label(self.root, text="max").grid(row=i, column=6)
            self.minmax_entries[attr+'_max'] = tk.Entry(self.root, width=5)
            self.minmax_entries[attr+'_max'].grid(row=i, column=7)
            self.minmax_entries[attr+'_max'].insert(0, str(self.vel_limits[attr][1]))
            tk.Button(self.root, text="Set", command=partial(self.update_limits, attr)).grid(row=i, column=8)

        # --- Remote-style buttons on canvas ---
        self.canvas = tk.Canvas(self.root, width=250, height=250)
        self.canvas.grid(row=6, column=0, columnspan=9, pady=10)

        cx, cy, r = 125, 125, 80  # center and radius
        # quarter-circle buttons
        self.draw_quarter_button(cx, cy, r, 0, math.pi/2, 'Forward', 'linear.x', self.button_speeds['linear.x'])
        self.draw_quarter_button(cx, cy, r, math.pi/2, math.pi, 'Left', 'linear.y', self.button_speeds['linear.y'])
        self.draw_quarter_button(cx, cy, r, math.pi, 3*math.pi/2, 'Back', 'linear.x', -self.button_speeds['linear.x'])
        self.draw_quarter_button(cx, cy, r, 3*math.pi/2, 2*math.pi, 'Right', 'linear.y', -self.button_speeds['linear.y'])
        # CW/CCW at 45° and 135°
        self.draw_circle_button(cx + r/math.sqrt(2), cy - r/math.sqrt(2), 20, 'CW', 'angular.z', -self.button_speeds['angular.z'])
        self.draw_circle_button(cx - r/math.sqrt(2), cy - r/math.sqrt(2), 20, 'CCW', 'angular.z', self.button_speeds['angular.z'])

    # --- Drawing helpers ---
    def draw_quarter_button(self, cx, cy, r, start, end, text, attr, val):
        x0, y0 = cx-r, cy-r
        x1, y1 = cx+r, cy+r
        arc = self.canvas.create_arc(x0, y0, x1, y1, start=math.degrees(start), extent=math.degrees(end-start), fill='lightgray')
        self.canvas.tag_bind(arc, '<Button-1>', lambda e: self.button_move(attr, val))
        mid_angle = (start+end)/2
        tx = cx + r/2*math.cos(mid_angle)
        ty = cy - r/2*math.sin(mid_angle)
        self.canvas.create_text(tx, ty, text=text)

    def draw_circle_button(self, cx, cy, radius, text, attr, val):
        oval = self.canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, fill='lightblue')
        self.canvas.tag_bind(oval, '<Button-1>', lambda e: self.button_move(attr, val))
        self.canvas.create_text(cx, cy, text=text)

    # --- Handlers ---
    def slider_changed(self, attr, val):
        val = float(val)
        minv, maxv = self.vel_limits[attr]
        # map slider fraction -1..1 to velocity with min threshold
        if val > 0:
            v = minv + (maxv - minv) * val
        elif val < 0:
            v = - (minv + (maxv - minv) * abs(val))
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
                # clamp magnitude, zero if near 0
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
            self.sliders[attr].config(from_=-1.0, to=1.0)  # slider always -1..1
        except:
            pass

    # --- Publish loop ---
    def update_loop(self):
        msg = self.text_vel if self.current_mode == 'text' else self.button_vel
        self.pub.publish(msg)
        self.root.after(50, self.update_loop)  # 20 Hz

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
