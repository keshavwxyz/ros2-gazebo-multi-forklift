"""
Navigation stack for the forklift: Nav2 plus localization (AMCL on a saved map) or SLAM.

  mode:=navigation  map_server + AMCL (map -> odom) on the saved map, Nav2
  mode:=mapping     slam_toolbox (map -> odom) makes the map; Nav2 is optional (nav:=true)

This file always starts footprint_manager (loaded/unloaded footprint) and
scan_footprint_filter (/scan -> /scan_filtered). The two localization systems never run
at the same time. Thus, map -> odom has exactly one publisher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
              'velocity_smoother', 'collision_monitor']


def generate_launch_description():
    pkg = FindPackageShare('forklift_navigation')
    params = LaunchConfiguration('params_file')
    prefix = LaunchConfiguration('frame_prefix')
    footprints = PathJoinSubstitution([pkg, 'config', 'footprints.yaml'])
    mode = LaunchConfiguration('mode')
    is_nav = IfCondition(PythonExpression(["'", mode, "' == 'navigation'"]))
    is_map = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    run_nav2 = IfCondition(PythonExpression(
        ["'", mode, "' == 'navigation' or '", LaunchConfiguration('nav'), "' == 'true'"]))
    sim = {'use_sim_time': True}

    common = [
        Node(package='forklift_navigation', executable='footprint_manager', output='screen',
             parameters=[{'footprints_file': footprints}, sim]),
        Node(package='forklift_navigation', executable='scan_footprint_filter',
             output='screen', parameters=[{'footprints_file': footprints,
                                           'frame_prefix': prefix}, sim]),
    ]

    localization = GroupAction(condition=is_nav, actions=[
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[params, sim, {'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, sim]),
        # lifecycle_starter, not nav2_lifecycle_manager: it tries again when a change_state
        # response is lost. It does not stop and wait (see
        # forklift_navigation/lifecycle_starter.py).
        Node(package='forklift_navigation', executable='lifecycle_starter',
             name='lifecycle_starter_localization', output='screen',
             parameters=[sim, {'node_names': ['map_server', 'amcl'],
                               # the start pose that the operator knows, with its real uncertainty
                               'set_initial_pose': True,
                               'initial_x': LaunchConfiguration('x'),
                               'initial_y': LaunchConfiguration('y'),
                               'initial_yaw': LaunchConfiguration('yaw'),
                               # AMCL must then publish map -> odom (if not,
                               # the node starts AMCL again)
                               'verify_tf': ['map', 'base_footprint'],
                               'frame_prefix': prefix}]),
    ])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={'use_sim_time': 'true', 'slam_params_file': PathJoinSubstitution(
            [pkg, 'config', 'slam_toolbox_mapping.yaml'])}.items(),
        condition=is_map)

    remap_cmd = [('cmd_vel', 'cmd_vel_nav')]
    # If a Nav2 server has a crash, the launch starts it again and lifecycle_starter activates it
    # again (planner_server had a segfault one time in approximately 40 launches, in Nav2).
    # The launch does not respawn localization. A new AMCL would not have the pose of the robot.
    # Thus, that crash must stay visible.
    respawn = {'respawn': True, 'respawn_delay': 2.0}
    nav2 = GroupAction(condition=run_nav2, actions=[
        Node(package='nav2_controller', executable='controller_server', output='screen', **respawn,
             parameters=[params, sim], remappings=remap_cmd),
        Node(package='nav2_planner', executable='planner_server', output='screen', **respawn,
             parameters=[params, sim]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen', **respawn,
             parameters=[params, sim], remappings=remap_cmd),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen', **respawn,
             parameters=[params, sim]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             output='screen', **respawn, parameters=[params, sim],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             output='screen', **respawn, parameters=[params, sim]),
        Node(package='forklift_navigation', executable='lifecycle_starter',
             name='lifecycle_starter_navigation', output='screen',
             parameters=[sim, {'node_names': NAV2_NODES}]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [pkg, 'config', 'nav2_params.yaml']),
            description='Nav2 parameters (namespace-rewritten copy for a namespaced robot)'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('mode', default_value='navigation',
                              description='navigation | mapping'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='also run Nav2 in mapping mode'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [pkg, 'maps', 'warehouse.yaml'])),
        DeclareLaunchArgument('x', default_value='-6.0', description='initial pose (map)'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        *common, localization, slam, nav2,
    ])
