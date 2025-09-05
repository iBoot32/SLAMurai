#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
import sys
import time
import threading

class KeyboardSerialOdom(Node):
    def __init__(self, ser, hz=20.0):
        super().__init__('keyboard_serial_odom')
        self.publisher = self.create_publisher(Odometry, '/odometry/raw', 10)
        self.ser = ser

        # Last velocity command
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Frame IDs
        self.frame_id = "odom"
        self.child_frame_id = "base_link"

        # Publish timer
        self.create_timer(1.0 / hz, self.publish_odom)

        # Launch keyboard input thread
        self.thread = threading.Thread(target=self.read_keyboard)
        self.thread.daemon = True
        self.thread.start()

    def read_keyboard(self):
        while True:
            sys.stdout.write("Enter velocities [vx vy wz]: ")
            sys.stdout.flush()
            line = sys.stdin.readline().strip()

            if not line:
                continue

            # Send over serial
            try:
                self.ser.write((line + '\n').encode())
            except Exception as e:
                self.get_logger().error(f"Serial write failed: {e}")

            # Parse and store velocities
            try:
                parts = line.split()
                self.vx = float(parts[0])
                self.vy = float(parts[1])
                self.wz = float(parts[2])
            except (IndexError, ValueError):
                self.get_logger().error("Invalid format. Expected: vx vy wz")

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        # Pose is unset ? identity quaternion, position zero
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Publish twist velocities
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.wz

        self.publisher.publish(msg)

def main():
    rclpy.init()

    port = '/dev/ttyACM0'
    baud = 1000000
    hz = 50.0  # Change publish rate here

    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(2)
    ser.reset_input_buffer()

    node = KeyboardSerialOdom(ser, hz)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ser.close()

if __name__ == '__main__':
    main()

