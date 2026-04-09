import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# Your configurable parameters list
configurable_parameters = [
    {'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
    {'name': 'serial_no', 'default': "''", 'description': 'choose device by serial number'},
    {'name': 'usb_port_id', 'default': "''", 'description': 'choose device by usb port id'},
    {'name': 'device_type', 'default': "''", 'description': 'choose device by type'},
    {'name': 'config_file', 'default': "''", 'description': 'yaml config file'},
    {'name': 'enable_pointcloud', 'default': 'false', 'description': 'enable pointcloud'},
    {'name': 'unite_imu_method', 'default': "'copy'", 'description': '[copy|linear_interpolation]'},
    {'name': 'json_file_path', 'default': "''", 'description': 'allows advanced configuration'},
    {'name': 'log_level', 'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name': 'output', 'default': 'screen', 'description': 'pipe node output [screen|log]'},
    {'name': 'depth_width', 'default': '640', 'description': 'depth image width'},
    {'name': 'depth_height', 'default': '480', 'description': 'depth image height'},
    {'name': 'enable_depth', 'default': 'false', 'description': 'enable depth stream'},
    {'name': 'color_width', 'default': '424', 'description': 'color image width'},
    {'name': 'color_height', 'default': '240', 'description': 'color image height'},
    {'name': 'enable_color', 'default': 'true', 'description': 'enable color stream'},
    {'name': 'infra_width', 'default': '-1', 'description': 'infra width'},
    {'name': 'infra_height', 'default': '-1', 'description': 'infra width'},
    {'name': 'enable_infra1', 'default': 'false', 'description': 'enable infra1 stream'},
    {'name': 'enable_infra2', 'default': 'false', 'description': 'enable infra2 stream'},
    {'name': 'infra_rgb', 'default': 'false', 'description': 'enable infra2 stream'},
    {'name': 'fisheye_width', 'default': '-1', 'description': 'fisheye width'},
    {'name': 'fisheye_height', 'default': '-1', 'description': 'fisheye width'},
    {'name': 'enable_fisheye1', 'default': 'false', 'description': 'enable fisheye1 stream'},
    {'name': 'enable_fisheye2', 'default': 'false', 'description': 'enable fisheye2 stream'},
    {'name': 'confidence_width', 'default': '-1', 'description': 'depth image width'},
    {'name': 'confidence_height', 'default': '-1', 'description': 'depth image height'},
    {'name': 'enable_confidence', 'default': 'false', 'description': 'enable depth stream'},
    {'name': 'fisheye_fps', 'default': '-1.', 'description': ''},
    {'name': 'depth_fps', 'default': '15.', 'description': ''},
    {'name': 'confidence_fps', 'default': '-1.', 'description': ''},
    {'name': 'infra_fps', 'default': '-1.', 'description': ''},
    {'name': 'color_fps', 'default': '6.', 'description': ''},
    {'name': 'gyro_fps', 'default': '200.', 'description': ''},
    {'name': 'accel_fps', 'default': '200.', 'description': ''},
    {'name': 'color_qos', 'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
    {'name': 'confidence_qos', 'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
    {'name': 'depth_qos', 'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
    {'name': 'fisheye_qos', 'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
    {'name': 'infra_qos', 'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
    {'name': 'pointcloud_qos', 'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
    {'name': 'enable_gyro', 'default': 'true', 'description': ''},
    {'name': 'enable_accel', 'default': 'true', 'description': ''},
    {'name': 'pointcloud_texture_stream', 'default': 'RS2_STREAM_COLOR', 'description': 'testure stream for pointcloud'},
    {'name': 'pointcloud_texture_index', 'default': '0', 'description': 'testure stream index for pointcloud'},
    {'name': 'enable_sync', 'default': 'false', 'description': ''},
    {'name': 'align_depth', 'default': 'false', 'description': ''},
    {'name': 'filters', 'default': "''", 'description': ''},
    {'name': 'clip_distance', 'default': '5.5', 'description': ''},
    {'name': 'linear_accel_cov', 'default': '0.01', 'description': ''},
    {'name': 'initial_reset', 'default': 'false', 'description': ''},
    {'name': 'allow_no_texture_points', 'default': 'true', 'description': ''},
    {'name': 'ordered_pc', 'default': 'false', 'description': ''},
    {'name': 'calib_odom_file', 'default': "''", 'description': "''"},
    {'name': 'topic_odom_in', 'default': "''", 'description': 'topic for T265 wheel odometry'},
    {'name': 'tf_publish_rate', 'default': '0.0', 'description': 'Rate of publishing static_tf'},
    {'name': 'diagnostics_period', 'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
    {'name': 'rosbag_filename', 'default': "''", 'description': 'A realsense bagfile to run from as a device'},
    {'name': 'temporal.holes_fill', 'default': '0', 'description': 'Persistency mode'},
    {'name': 'stereo_module.enable_auto_exposure', 'default': 'false', 'description': ''},
    {'name': 'stereo_module.exposure', 'default': '8000', 'description': ''},
    {'name': 'stereo_module.laser_power', 'default': '360.0', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'stereo_module.exposure.1', 'default': '7500', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'stereo_module.gain.1', 'default': '16', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'stereo_module.exposure.2', 'default': '1', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'stereo_module.gain.2', 'default': '16', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'wait_for_device_timeout', 'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
    {'name': 'reconnect_timeout', 'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    # Prepare dict of {param_name: LaunchConfiguration(param_name)}
    return {param['name']: LaunchConfiguration(param['name']) for param in parameters}

def launch_setup(context, *args, **kwargs):
    params = set_configurable_parameters(configurable_parameters)
    
    output_mode = LaunchConfiguration('output').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    camera_namespace = LaunchConfiguration('camera_name').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)

    return [
        launch_ros.actions.Node(
            package='realsense2_camera',
            node_namespace=camera_namespace,
            node_name=camera_name,
            node_executable='realsense2_camera_node',
            parameters=[params],
            output=output_mode,
            arguments=['--ros-args', '--log-level', log_level],
        )
    ]

def generate_launch_description():
    slamurai_nav_share_dir = get_package_share_directory('slamurai_nav')
    ekf_config_path = os.path.join(slamurai_nav_share_dir, 'config', 'ekf.yaml')
    urdf_file = os.path.join(slamurai_nav_share_dir, 'urdf', 'SLAMurai.xml')
    toolbox_params_path = os.path.join(slamurai_nav_share_dir, 'params', 'slam_toolbox.yaml')
    nav2_params_path = os.path.join(slamurai_nav_share_dir, 'params', 'nav2_params.yaml')
    map_path = os.path.join(slamurai_nav_share_dir, 'maps', 'firstfloor_360.yaml')
    bt = os.path.join(slamurai_nav_share_dir, 'params', 'nav_w_recovery.xml')

    # Use medium profile json for realsense
    json_config_path = os.path.join(slamurai_nav_share_dir, 'config', 'medium_profile.json')
    for param in configurable_parameters:
        if param['name'] == 'json_file_path':
            param['default'] = json_config_path
            break

    # Nav2 launching using bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'nav2_bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params': nav2_params_path,
            'bt_xml_file': bt,
            'map': map_path,
        }.items()
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            )
        ),
        launch_arguments={
            'frame_id': 'lidar_link',
            'scan_mode': 'Sensitivity',
            'angle_min': '-3.14159265',
            'angle_max': '3.14159265',
            'angle_compensate': 'true',
        }.items()
    )

    rosbridge_node = launch_ros.actions.Node(
        package='rosbridge_server',
        node_executable='rosbridge_websocket',
        node_name='rosbridge_websocket',
        output='screen'
    )

    web_video_node = launch_ros.actions.Node(
        package='web_video_server',
        node_executable='web_video_server',
        node_name='web_video_server',
        output='screen'
    )

    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) + [
            DeclareLaunchArgument('output', default_value='screen', description='Output mode'),
            DeclareLaunchArgument('log_level', default_value='info', description='Logging level'),
            OpaqueFunction(function=launch_setup),

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

            # launch_ros.actions.Node(
            #     package="slam_toolbox",
            #     node_executable="async_slam_toolbox_node",
            #     node_name="slam_toolbox",
            #     output="screen",
            #     parameters=[toolbox_params_path],
            # ),

            rplidar_launch,
            nav2_launch,
            rosbridge_node,
            web_video_node,
        ]
    )

