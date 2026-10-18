#!/usr/bin/env python3
"""Camera is mounted upside down: rotates /camera/color/image_raw by 180 degrees to /camera/color/image_upright."""
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
n = rclpy.create_node('camera_flip')
pub = n.create_publisher(Image, '/camera/color/image_upright', 2)


def flip(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, m.step // m.width)
    m.data = a[::-1, ::-1].tobytes()
    pub.publish(m)


n.create_subscription(Image, '/camera/color/image_raw', flip, qos_profile_sensor_data)
rclpy.spin(n)
