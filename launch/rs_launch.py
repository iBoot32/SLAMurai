"""Launch realsense2_camera node with tuned Medium Density parameters."""
import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

configurable_parameters = [
    {'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
    {'name': 'camera_namespace', 'default': 'camera', 'description': 'namespace for camera'},
    {'name': 'serial_no', 'default': "''", 'description': 'choose device by serial number'},
    {'name': 'usb_port_id', 'default': "''", 'description': 'choose device by usb port id'},
    {'name': 'device_type', 'default': "''", 'description': 'choose device by type'},
    {'name': 'config_file', 'default': "''", 'description': 'yaml config file'},
    {'name': 'json_file_path', 'default': "''", 'description': 'allows advanced configuration'},
    {'name': 'initial_reset', 'default': 'false', 'description': "''"},
    {'name': 'accelerate_gpu_with_glsl', 'default': "true", 'description': 'enable GPU acceleration with GLSL'},
    {'name': 'rosbag_filename', 'default': "''", 'description': 'A realsense bagfile to run from as a device'},
    {'name': 'log_level', 'default': 'info', 'description': 'debug log level'},
    {'name': 'output', 'default': 'screen', 'description': 'pipe node output'},

    # Stream tuning
    {'name': 'enable_color', 'default': 'true', 'description': ''},
    {'name': 'enable_depth', 'default': 'true', 'description': ''},
    {'name': 'enable_infra1', 'default': 'true', 'description': ''},
    {'name': 'enable_infra2', 'default': 'true', 'description': ''},
    {'name': 'rgb_camera.color_profile', 'default': '640,480,30', 'description': ''},
    {'name': 'rgb_camera.color_format', 'default': 'RGB8', 'description': ''},
    {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': ''},
    {'name': 'depth_module.depth_profile', 'default': '848,480,30', 'description': ''},
    {'name': 'depth_module.depth_format', 'default': 'Z16', 'description': ''},
    {'name': 'depth_module.infra_profile', 'default': '848,480,30', 'description': ''},
    {'name': 'depth_module.infra_format', 'default': 'Y8', 'description': ''},
    {'name': 'depth_module.infra1_format', 'default': 'Y8', 'description': ''},
    {'name': 'depth_module.infra2_format', 'default': 'Y8', 'description': ''},
    {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': ''},
    {'name': 'depth_module.hdr_enabled', 'default': 'false', 'description': ''},

    # IMU
    {'name': 'enable_gyro', 'default': 'true', 'description': ''},
    {'name': 'enable_accel', 'default': 'true', 'description': ''},
    {'name': 'gyro_fps', 'default': '200', 'description': ''},
    {'name': 'accel_fps', 'default': '250', 'description': ''},
    {'name': 'unite_imu_method', 'default': '2', 'description': ''},

    # Pointcloud
    {'name': 'pointcloud.enable', 'default': 'true', 'description': ''},
    {'name': 'pointcloud.stream_filter', 'default': '2', 'description': ''},
    {'name': 'pointcloud.stream_index_filter', 'default': '0', 'description': ''},
    {'name': 'pointcloud.ordered_pc', 'default': 'false', 'description': ''},
    {'name': 'pointcloud.allow_no_texture_points', 'default': 'true', 'description': ''},
    {'name': 'align_depth.enable', 'default': 'true', 'description': ''},

    # Filters
    {'name': 'decimation_filter.enable', 'default': 'true', 'description': ''},
    {'name': 'spatial_filter.enable', 'default': 'true', 'description': ''},
    {'name': 'temporal_filter.enable', 'default': 'true', 'description': ''},
    {'name': 'hole_filling_filter.enable', 'default': 'false', 'description': ''},
    {'name': 'disparity_filter.enable', 'default': 'false', 'description': ''},
    {'name': 'hdr_merge.enable', 'default': 'false', 'description': ''},

    # Distance
    {'name': 'clip_distance', 'default': '-2.0', 'description': 'no clipping'},

    # TF / diagnostics
    {'name': 'publish_tf', 'default': 'true', 'description': ''},
    {'name': 'tf_publish_rate', 'default': '0.0', 'description': ''},
    {'name': 'diagnostics_period', 'default': '0.0', 'description': ''},

    # Extras
    {'name': 'enable_sync', 'default': 'false', 'description': ''},
    {'name': 'enable_rgbd', 'default': 'false', 'description': ''},
    {'name': 'angular_velocity_cov', 'default': '0.01', 'description': ''},
    {'name': 'linear_accel_cov', 'default': '0.01', 'description': ''},
    {'name': 'wait_for_device_timeout', 'default': '-1.', 'description': ''},
    {'name': 'reconnect_timeout', 'default': '6.', 'description': ''},
]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    _output = LaunchConfiguration('output' + param_name_suffix)
    if(os.getenv('ROS_DISTRO') == 'foxy'):
        _output = context.perform_substitution(_output)

    return [
        launch_ros.actions.Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
            name=LaunchConfiguration('camera_name' + param_name_suffix),
            executable='realsense2_camera_node',
            parameters=[params, params_from_file],
            output=_output,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
        )
    ]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup, kwargs={'params': set_configurable_parameters(configurable_parameters)}),
        
        # RSNode for processing realsense data
        launch_ros.actions.Node(
            package='rs',
            executable='RSNode',
            name='RSNode',
            output='screen',
        ),

        # Madgwick Filter to fuse IMU data, obtain orientation from angular velocity
        launch_ros.actions.Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': True,
                'world_frame': 'enu',
                'fixed_frame': 'base_link',
                'remove_gravity_vector': True,
                'gain': 0.01,
                'orientation_stddev': 0.1,
                'input_rate': 200.0
            }],
            remappings=[
                ('/imu/data_raw', '/camera/camera/imu') # RS publishes as /camera/camera/imu, need to remap
            ]
        ),

        launch_ros.actions.Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'output_frame': 'camera_depth_frame', # Needs to match the frame of the depth image
                'range_min': 0.00,
                'range_max': 10.0,
                'scan_height': 10, # Merge 10 rows of depth image into a laserscan
            }]
        )
    ])