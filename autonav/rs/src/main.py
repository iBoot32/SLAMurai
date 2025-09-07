#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import serial
import time


class CmdVelSerialOdom(Node):
    def __init__(self, ser, hz=20.0):
        super().__init__('cmdvel_serial_odom')
        self.ser = ser

        # QoS: Best Effort for continuous streaming commands
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # ROS2 publishers/subscribers
        self.publisher = self.create_publisher(Odometry, '/odometry/raw', 10)
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos)

        # Internal velocity state
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.frame_id = "odom"
        self.child_frame_id = "base_link"

        # Timers: odometry publishing
        self.create_timer(1.0 / hz, self.publish_odom)

    def cmd_vel_callback(self, msg: Twist):
        # Store received velocities
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

        cmd = f"{self.vx},{self.vy},{self.wz}\n"
        self.ser.write(cmd.encode())

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        # Pose = zero
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Use latest velocities
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.wz

        self.publisher.publish(msg)


def main():
    rclpy.init()

    print(f"[main.py] Init main.py, waiting for cmd_vel", flush=True)

    port = '/dev/ttyACM0'
    baud = 1000000
    hz = 20.0

    # Configure non-blocking serial
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,
    )

    # Give Arduino / MCU time to reset
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
