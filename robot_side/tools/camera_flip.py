#!/usr/bin/env python3
"""Rotates the color image by 180 degrees (camera is mounted upside down). /camera/color/image_raw -> /camera/color/image_upright."""
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    n = rclpy.create_node('camera_flip')
    pub = n.create_publisher(Image, '/camera/color/image_upright', 2)

    def on_img(m):
        c = m.step // m.width
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, c)[::-1, ::-1]
        m.data = a.tobytes()
        pub.publish(m)

    n.create_subscription(Image, '/camera/color/image_raw', on_img, qos_profile_sensor_data)
    rclpy.spin(n)


if __name__ == '__main__':
    main()
