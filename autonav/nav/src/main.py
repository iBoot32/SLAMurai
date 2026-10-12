#!/usr/bin/env python3
# Serial bridge Jetson <-> Arduino.  Differences from the original (see docs/CHANGES_IN_COPY.md):
#   * listens to /cmd_vel_safe (output of robot_side/safety/safety_gate.py) instead of /cmd_vel, so nothing can reach the
#     motors without passing the gate
#   * re-sends the last safe command at 20 Hz (keeps the firmware watchdog fed) and sends zeros if /cmd_vel_safe goes
#     silent for 0.3 s
#   * accepts the 7th odometry field (firmware flags) and republishes it on /fw_flags
#   * drains the serial buffer every tick so odometry never lags behind the 40 Hz line rate
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial
import time
import math

CMD_SILENCE_S = 0.3


class CmdVelSerialOdom(Node):
    def __init__(self, ser, hz=50.0):
        super().__init__('cmdvel_serial_odom')
        self.ser = ser

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.publisher = self.create_publisher(Odometry, '/odometry/raw', 10)
        self.flags_pub = self.create_publisher(Int32, '/fw_flags', 10)
        self.subscriber = self.create_subscription(
            Twist, '/cmd_vel_safe', self.cmd_vel_callback, qos)

        self.frame_id = "odom"
        self.child_frame_id = "base_link"

        # Current odom pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Current odom velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0

        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_rx = 0.0
        self.create_timer(1.0 / hz, self.publish_odom)
        self.create_timer(0.05, self.send_cmd)

    # Stores the newest safe command; send_cmd() writes it to the Arduino
    def cmd_vel_callback(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_rx = time.monotonic()

    def send_cmd(self):
        vx, vy, w = self.cmd
        if time.monotonic() - self.cmd_rx > CMD_SILENCE_S or not all(math.isfinite(v) for v in (vx, vy, w)):
            vx, vy, w = 0.0, 0.0, 0.0
        try:
            self.ser.write(("%.4f,%.4f,%.4f\n" % (vx, vy, w)).encode())
        except Exception as e:
            self.get_logger().error('serial write failed: %s' % e)

    def publish_odom(self):
        # Read every odom serial line that is waiting; keep the newest
        flags = None
        try:
            for _ in range(8):
                line = self.ser.readline().decode().strip()
                if line:
                    parts = line.split(',')
                    if len(parts) in (6, 7):
                        vals = [float(p) for p in parts]
                        self.x, self.y, self.yaw = vals[0], vals[1], vals[2]   # meters / radians
                        self.vx, self.vy, self.vw = vals[3], vals[4], vals[5]  # meters/sec
                        if len(parts) == 7:
                            flags = int(vals[6])
                if self.ser.in_waiting == 0:
                    break
        except Exception:
            pass  # ignore malformed lines

        if flags is not None:
            fm = Int32()
            fm.data = flags
            self.flags_pub.publish(fm)

        # Convert yaw -> quaternion
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        # Populate Pose
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 5mm std in pose, 0.1deg std orientation
        msg.pose.covariance = [
            2.5e-05, 0.0,     0.0,     0.0,     0.0,     0.0,
            0.0,     2.5e-05, 0.0,     0.0,     0.0,     0.0,
            0.0,     0.0,     1.0,     0.0,     0.0,     0.0,
            0.0,     0.0,     0.0,     1.0,     0.0,     0.0,
            0.0,     0.0,     0.0,     0.0,     1.0,     0.0,
            0.0,     0.0,     0.0,     0.0,     0.0,     3.05e-06
        ]

        # Populate Twist (Velocity)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.vw

        # Basic twist covariance
        msg.twist.covariance = [
            1e-4, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e-4, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e-4, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e-4, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e-4, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-4
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
        try:
            ser.write(b"0,0,0\n")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        ser.close()


if __name__ == '__main__':
    main()
