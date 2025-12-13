#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses
import time
import threading

class CmdVelCLI(Node):
    def __init__(self, stdscr):
        super().__init__('cmdvel_cli')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # initial speed percentage
        self.key_percent = 0.20

        # key state
        self.keys_down = set()
        self.last_seen = {}
        self.debounce_ms = 0.210
        self.initial_grace = 0.45

        self.vel_limits = {
            'linear.x': [0.01, 0.35],
            'linear.y': [0.01, 0.35],
            'angular.z': [0.01, 1.75]
        }

        self.stdscr = stdscr
        self.init_curses()

        # start key thread
        self.thread = threading.Thread(target=self.key_loop, daemon=True)
        self.thread.start()

        self.publish_loop()

    def init_curses(self):
        self.stdscr.nodelay(True)
        curses.cbreak()
        curses.noecho()
        self.stdscr.clear()
        self.display_speed()

    def display_speed(self):
        self.stdscr.addstr(0, 0, f"Speed: {int(self.key_percent*100)}%   ")
        self.stdscr.refresh()

    # -------------------------------------------------------
    # keyboard thread
    # -------------------------------------------------------
    def key_loop(self):
        valid = {'w','a','s','d','q','e'}

        while True:
            c = self.stdscr.getch()

            # no input
            if c == -1:
                time.sleep(0.003)
                continue

            now = time.time()

            # arrow keys
            if c == curses.KEY_UP:
                self.key_percent = min(1.0, self.key_percent + 0.10)
                self.display_speed()
                continue

            if c == curses.KEY_DOWN:
                self.key_percent = max(0.0, self.key_percent - 0.10)
                self.display_speed()
                continue

            # convert others to char
            try:
                k = chr(c).lower()
            except:
                continue

            if k not in valid:
                continue

            # first press
            if k not in self.keys_down:
                self.keys_down.add(k)
                self.last_seen[k] = now + self.initial_grace
            else:
                # repeat press
                self.last_seen[k] = now

    # -------------------------------------------------------
    # compute velocity
    # -------------------------------------------------------
    def compute_vel(self):
        msg = Twist()
        pct = self.key_percent

        mx = self.vel_limits['linear.x'][1]
        my = self.vel_limits['linear.y'][1]
        mz = self.vel_limits['angular.z'][1]

        kd = self.keys_down

        if 'w' in kd:
            msg.linear.x = pct * mx
        if 's' in kd:
            msg.linear.x = -pct * mx
        if 'a' in kd:
            msg.linear.y = pct * my
        if 'd' in kd:
            msg.linear.y = -pct * my
        if 'q' in kd:
            msg.angular.z = pct * mz
        if 'e' in kd:
            msg.angular.z = -pct * mz

        return msg

    # -------------------------------------------------------
    # main 50ms loop
    # -------------------------------------------------------
    def publish_loop(self):
        while rclpy.ok():
            now = time.time()

            expired = [k for k,t in self.last_seen.items()
                       if now - t > self.debounce_ms]

            for k in expired:
                if k in self.keys_down:
                    self.keys_down.remove(k)
                del self.last_seen[k]

            msg = self.compute_vel()
            self.pub.publish(msg)

            time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    curses.wrapper(lambda stdscr: CmdVelCLI(stdscr))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
