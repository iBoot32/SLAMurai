#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import serial
import time
import math


class CmdVelSerialOdom(Node):
    def __init__(self, ser, hz=50.0):
        super().__init__('cmdvel_serial_odom')
        self.ser = ser

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.publisher = self.create_publisher(Odometry, '/odometry/raw', 10)
        self.subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos)

        self.frame_id = "odom"
        self.child_frame_id = "base_link"

        # Current odom values
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.create_timer(1.0 / hz, self.publish_odom)

    def cmd_vel_callback(self, msg: Twist):
        cmd = f"{msg.linear.x},{msg.linear.y},{msg.angular.z}\n"
        self.ser.write(cmd.encode())

    def publish_odom(self):
        # Read any available serial line from Arduino
        try:
            line = self.ser.readline().decode().strip()
            if line:
                parts = line.split(',')
                if len(parts) == 3:
                    self.x = float(parts[0])   # already meters
                    self.y = float(parts[1])
                    self.yaw = float(parts[2])
        except Exception:
            pass  # ignore malformed lines

        # Convert yaw -> quaternion
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # 5mm std pose, 0.1deg std orientation
        msg.pose.covariance = [
            2.5e-05, 0.0,     0.0,     0.0,     0.0,     0.0,
            0.0,     2.5e-05, 0.0,     0.0,     0.0,     0.0,
            0.0,     0.0,     1.0,     0.0,     0.0,     0.0,
            0.0,     0.0,     0.0,     1.0,     0.0,     0.0,
            0.0,     0.0,     0.0,     0.0,     1.0,     0.0,
            0.0,     0.0,     0.0,     0.0,     0.0,     3.05e-06
        ]

        self.publisher.publish(msg)


def main():
    rclpy.init()

    port = '/dev/ttyACM0'
    baud = 115200
    hz = 50.0

    ser = serial.Serial(port=port, baudrate=baud, timeout=0.01)

    time.sleep(3)
    ser.reset_input_buffer()

    node = CmdVelSerialOdom(ser, hz)

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
