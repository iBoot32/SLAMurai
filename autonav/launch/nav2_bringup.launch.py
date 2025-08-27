from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to params file dynamically
    nav2_params_path = os.path.join(
        get_package_share_directory('rs'),
        'params',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            output='screen',
            parameters=[nav2_params_path],
        )
    ])
