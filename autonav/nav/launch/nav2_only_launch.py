import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Nav2 alone, with the parameter file given on the command line.  the PC restarts this for every candidate:
    Dashing's Nav2 reads each parameter once when its node is configured, so a restart is the only way to apply it."""
    share = get_package_share_directory('slamurai_nav')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'firstfloor_360.yaml')),
        DeclareLaunchArgument('bt', default_value=os.path.join(share, 'params', 'nav_w_recovery.xml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                                                       'nav2_bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params': LaunchConfiguration('params'),
                'bt_xml_file': LaunchConfiguration('bt'),
                'map': LaunchConfiguration('map'),
            }.items()),
    ])
