"""Parameter names that ROS 2 Dashing's Nav2 (navigation2 branch `dashing-devel`) actually declares, per node.

Built by reading the Dashing sources (declare_parameter / get_parameter / nav_2d_utils::searchAndGetParam calls in
dwb_local_planner.cpp, kinematic_parameters.cpp, standard_traj_generator.cpp, xy_theta_iterator.cpp,
simple_goal_checker.cpp, the dwb_critics, costmap_2d_ros.cpp, obstacle_layer.cpp, inflation_layer.cpp, amcl_node.cpp,
navfn_planner.cpp, bt_navigator.cpp, odom_subscriber.hpp, progress_checker.cpp), retrieved 2026-10-08.

Facts that matter for tuning on Dashing (all from those sources):
  * every parameter is read ONCE in on_configure(); there is no parameter-event callback anywhere, so a changed value
    only takes effect after the node is restarted (autotune restarts the Nav2 launch for each candidate);
  * the velocity controller is the `dwb_controller` node and it hosts the local costmap, so DWB and local-costmap
    parameters sit under different top-level YAML keys (`dwb_controller:` and `local_costmap: local_costmap:`);
  * oscillation_reset_dist/angle/time and forward_point_distance are read with get_parameter_or on the plain node
    name, NOT as `Oscillation.oscillation_reset_dist`: the dotted spelling is silently ignored;
  * keys under `controller_server:`, `recoveries_server:` (Foxy-era names) are not read by any Dashing node.
"""

CRITICS = ['BaseObstacle', 'ObstacleFootprint', 'GoalAlign', 'GoalDist', 'PathAlign', 'PathDist', 'Oscillation',
           'PreferForward', 'RotateToGoal', 'Twirling']

DWB = set([
    'controller_frequency', 'odom_topic', 'min_x_velocity_threshold', 'min_y_velocity_threshold', 'min_theta_velocity_threshold',
    'critics', 'prune_plan', 'prune_distance', 'debug_trajectory_details', 'trajectory_generator_name', 'goal_checker_name',
    'use_dwa', 'transform_tolerance', 'default_critic_namespaces',
    'min_vel_x', 'min_vel_y', 'max_vel_x', 'max_vel_y', 'max_vel_theta', 'min_speed_xy', 'max_speed_xy', 'min_speed_theta',
    'acc_lim_x', 'acc_lim_y', 'acc_lim_theta', 'decel_lim_x', 'decel_lim_y', 'decel_lim_theta',
    'sim_time', 'discretize_by_time', 'time_granularity', 'linear_granularity', 'angular_granularity', 'sim_granularity',
    'angular_sim_granularity', 'vx_samples', 'vy_samples', 'vtheta_samples', 'sim_period',
    'xy_goal_tolerance', 'yaw_goal_tolerance', 'rot_stopped_velocity', 'trans_stopped_velocity',
    'forward_point_distance', 'oscillation_reset_dist', 'oscillation_reset_angle', 'oscillation_reset_time',
    'publish_evaluation', 'publish_global_plan', 'publish_transformed_plan', 'publish_local_plan', 'publish_trajectories',
    'publish_cost_grid_pc', 'required_movement_radius', 'movement_time_allowance_',
    # legacy aliases declared by dwb_local_planner
    'path_distance_bias', 'goal_distance_bias', 'occdist_scale', 'max_scaling_factor', 'scaling_speed',
])
for _c in CRITICS:
    DWB.add(_c + '.scale')
DWB.update(['BaseObstacle.sum_scores', 'Oscillation.x_only_threshold', 'PreferForward.penalty', 'PreferForward.strafe_x',
            'PreferForward.strafe_theta', 'PreferForward.theta_scale', 'GoalDist.aggregation_type', 'PathDist.aggregation_type',
            'GoalAlign.aggregation_type', 'PathAlign.aggregation_type', 'ObstacleFootprint.max_scaling_factor',
            'ObstacleFootprint.scaling_speed'])

COSTMAP = set([
    'always_send_full_costmap', 'footprint_padding', 'footprint', 'global_frame', 'height', 'width', 'lethal_cost_threshold',
    'map_topic', 'observation_sources', 'origin_x', 'origin_y', 'plugin_names', 'plugin_types', 'publish_frequency',
    'resolution', 'robot_base_frame', 'robot_radius', 'rolling_window', 'track_unknown_space', 'transform_tolerance',
    'trinary_costmap', 'unknown_cost_value', 'update_frequency', 'use_maximum', 'use_sim_time',
])
COSTMAP_LAYER = {
    'inflation_layer': set(['enabled', 'inflation_radius', 'cost_scaling_factor', 'inflate_unknown']),
    'obstacle_layer': set(['enabled', 'footprint_clearing_enabled', 'max_obstacle_height', 'combination_method']),
    'static_layer': set(['enabled', 'subscribe_to_updates', 'map_subscribe_transient_local']),
    'voxel_layer': set(['enabled', 'footprint_clearing_enabled', 'max_obstacle_height', 'combination_method',
                        'publish_voxel_map', 'origin_z', 'z_resolution', 'z_voxels', 'unknown_threshold', 'mark_threshold']),
}
OBS_SOURCE = set(['topic', 'sensor_frame', 'observation_persistence', 'expected_update_rate', 'data_type', 'min_obstacle_height',
                  'max_obstacle_height', 'inf_is_valid', 'marking', 'clearing', 'obstacle_range', 'raytrace_range'])

AMCL = set(['alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5', 'base_frame_id', 'beam_skip_distance', 'beam_skip_error_threshold',
            'beam_skip_threshold', 'do_beamskip', 'global_frame_id', 'lambda_short', 'laser_likelihood_max_dist',
            'laser_max_range', 'laser_min_range', 'laser_model_type', 'set_initial_pose', 'initial_pose.x', 'initial_pose.y',
            'initial_pose.z', 'initial_pose.yaw', 'max_beams', 'max_particles', 'min_particles', 'odom_frame_id', 'pf_err',
            'pf_z', 'recovery_alpha_fast', 'recovery_alpha_slow', 'resample_interval', 'robot_model_type', 'save_pose_rate',
            'sigma_hit', 'tf_broadcast', 'transform_tolerance', 'update_min_a', 'update_min_d', 'z_hit', 'z_max', 'z_rand',
            'z_short', 'use_sim_time', 'scan_topic', 'always_reset_initial_pose', 'first_map_only'])

NAVFN = set(['tolerance', 'use_astar', 'allow_unknown', 'use_sim_time'])
BT = set(['bt_xml_filename', 'use_sim_time'])
LIFECYCLE = set(['node_names', 'autostart', 'use_sim_time'])
MAP_SERVER = set(['yaml_filename', 'use_sim_time'])

# top-level YAML node blocks that a Dashing bringup actually reads
LIVE_NODES = {'amcl': 'amcl', 'dwb_controller': 'dwb_controller', 'bt_navigator': 'bt_navigator', 'navfn_planner': 'navfn_planner',
              'lifecycle_manager': 'lifecycle_manager', 'map_server': 'map_server', 'local_costmap': 'costmap',
              'global_costmap': 'costmap'}
# blocks that exist in the user's file but are read by no Dashing node (Foxy+ names or helper nodes)
DEAD_BLOCKS = ['controller_server', 'recoveries_server', 'amcl_map_client', 'amcl_rclcpp_node',
               'robot_state_publisher', 'local_costmap_client', 'local_costmap_rclcpp_node']


def _flatten(d, prefix=''):
    out = {}
    for k, v in (d or {}).items():
        key = prefix + str(k)
        if isinstance(v, dict) and k != 'ros__parameters':
            out.update(_flatten(v, key + '.'))
        elif isinstance(v, dict):
            out.update(_flatten(v, prefix))
        else:
            out[key] = v
    return out


def check_nav2_yaml(y):
    """Return a list of (severity, message) about a Nav2 params dict: unknown keys, dead blocks, odom topic."""
    issues = []
    for blk in DEAD_BLOCKS:
        if blk in y:
            issues.append(('info', 'block `%s:` is not read by any Dashing Nav2 node (ignored)' % blk))
    for blk, kind in LIVE_NODES.items():
        if blk not in y:
            continue
        if kind == 'costmap':
            flat = _flatten(y[blk].get(blk, y[blk]) if isinstance(y[blk], dict) else {})
            names = []
            for k in flat:
                top = k.split('.')[0]
                if k in COSTMAP:
                    continue
                if top in COSTMAP_LAYER:
                    if k.split('.', 1)[-1] not in COSTMAP_LAYER[top]:
                        issues.append(('warn', '%s: `%s` is not a Dashing %s parameter' % (blk, k, top)))
                    continue
                if len(k.split('.')) == 2 and k.split('.')[1] in OBS_SOURCE:
                    continue
                issues.append(('warn', '%s: `%s` is not declared by the Dashing costmap' % (blk, k)))
            continue
        allowed = {'amcl': AMCL, 'dwb_controller': DWB, 'bt_navigator': BT, 'navfn_planner': NAVFN,
                   'lifecycle_manager': LIFECYCLE, 'map_server': MAP_SERVER}[kind]
        flat = _flatten(y[blk])
        for k in flat:
            if k not in allowed and k != 'use_sim_time':
                hint = ''
                if k.split('.')[-1] in ('oscillation_reset_dist', 'oscillation_reset_angle', 'oscillation_reset_time'):
                    hint = ' (Dashing reads the plain name `%s`, not the dotted one)' % k.split('.')[-1]
                issues.append(('warn', '%s: `%s` is not declared by the Dashing node%s' % (blk, k, hint)))
    dwb = (y.get('dwb_controller') or {}).get('ros__parameters', {})
    if 'odom_topic' not in dwb:
        issues.append(('warn', 'dwb_controller has no odom_topic: Dashing subscribes to "odom", which this stack does not publish '
                               '(it has /odometry/raw and /odometry/filtered) -> DWB may see zero velocity. Check `ros2 topic info /odom`.'))
    return issues
