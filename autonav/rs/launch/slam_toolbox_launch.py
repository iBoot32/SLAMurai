from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to params file dynamically
    toolbox_params_path = os.path.join(
        get_package_share_directory('rs'),
        'params',
        'slam_toolbox.yaml'
    )
    return LaunchDescription([
        Node(
            package="slam_toolbox",
            node_executable="async_slam_toolbox_node",
            node_name="slam_toolbox",
            output="screen",
            parameters=[toolbox_params_path],
        )
    ])

