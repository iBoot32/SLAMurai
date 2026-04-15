import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():
    slamurai_nav_share_dir = get_package_share_directory('slamurai_nav')
    ekf_config_path = os.path.join(slamurai_nav_share_dir, 'config', 'ekf.yaml')
    urdf_file = os.path.join(slamurai_nav_share_dir, 'urdf', 'SLAMurai.xml')
    toolbox_params_path = os.path.join(slamurai_nav_share_dir, 'params', 'slam_toolbox.yaml')

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            )
        ),
        launch_arguments={
            'frame_id': 'lidar_link',
            'angle_min': '-3.14',
            'angle_max': '3.14',
            'angle_compensate': 'true',
        }.items()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('output', default_value='screen', description='Output mode'),
            DeclareLaunchArgument('log_level', default_value='info', description='Logging level'),

            ExecuteProcess(
                cmd=['python3', '/home/nvidia/ros2-ws/SLAMurai/control/cmdvel_gui/cmdvel_gui/cmdvel_gui/cmdvel_gui.py'],
                output='screen'
            ),

            launch_ros.actions.Node(
               package='slamurai_nav',
               node_executable='main.py',
               node_name='main',
               output='screen',
            ),

            # Robot State Publisher for TFs from URDF
            launch_ros.actions.Node(
                package='robot_state_publisher',
                node_executable='robot_state_publisher',
                node_name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': False}],
                arguments=[urdf_file]
            ),

            # EKF for odom/imu fusion
	        launch_ros.actions.Node(
                package='robot_localization',
                node_executable='ekf_node',
                node_name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config_path],
                arguments=['--ros-args', '--log-level', 'debug'],
            ),

             launch_ros.actions.Node(
                 package="slam_toolbox",
                 node_executable="async_slam_toolbox_node",
                 node_name="slam_toolbox",
                 output="screen",
                 parameters=[toolbox_params_path],
             ),

            rplidar_launch
        ]
    )

