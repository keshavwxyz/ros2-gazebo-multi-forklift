"""
One forklift instance in a Gazebo world that runs.

This launch file starts the description, spawn, sensor bridge, controllers, rear safety,
EKF, perception, navigation, load-handling skills and mission manager.

  namespace:=""            the initial single robot: root namespace, frames without a
                           prefix, and the package config files with no changes
  namespace:=forklift_1    all items are in /forklift_1. The TF frames are
                           "forklift_1/<frame>". The Gazebo entity and topics are
                           "forklift_1". All robots share the `map` frame.

sim.launch.py (one robot) and multi_sim.launch.py (two robots) use this file. There are no
duplicate files for each robot. Each robot uses the same xacro, nodes and config files with
a namespace. For a robot with a namespace, rewrite_params writes a temporary copy of each
ROS parameter file. In the copy:
  - The parameters are in the namespace.
  - The robot frame names have the frame prefix.
  - Absolute topic names become relative names. Thus, they resolve in the namespace.
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import yaml

# These frames are specific to one robot (URDF links and odom). All robots share `map`.
ROBOT_FRAMES = {'odom', 'base_footprint', 'base_link', 'lidar_link', 'rear_lidar_link',
                'camera_link', 'camera_optical_frame', 'imu_link', 'fork_tip_frame'}
FRAME_KEY = re.compile(r'(^|_)frame(_id)?$')
TOPIC_KEY = re.compile(r'(^|_)topic$|^odom0$|^imu0$')


def share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def _rewrite(node, prefix):
    if isinstance(node, dict):
        out = {}
        for k, v in node.items():
            if isinstance(v, str) and FRAME_KEY.search(str(k)) and v in ROBOT_FRAMES:
                v = prefix + v
            elif isinstance(v, str) and TOPIC_KEY.search(str(k)) and v.startswith('/'):
                v = v[1:]
            out[k] = _rewrite(v, prefix)
        return out
    if isinstance(node, list):
        return [_rewrite(v, prefix) for v in node]
    return node


def rewrite_params(path, namespace, prefix, out_dir):
    """Return a parameter file for this robot (the initial file for the root namespace)."""
    if not namespace:
        return path
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    data = {namespace: _rewrite(data, prefix)}
    out = os.path.join(out_dir, f'{namespace}_{os.path.basename(path)}')
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out


def add_peer_obstacles(path, namespace, out_dir):
    """
    Add the peer outlines of the fleet agent (fleet/peer_obstacles) to the two costmaps.

    The fork tines of other robots are below the LiDAR scan plane. Thus, the fleet agent sends
    their outlines from the heartbeats of the peers (forklift_fleet/fleet_agent.py). This
    source only marks obstacles. The LiDAR of the robot clears old points. The cloud is in the
    map frame. The origin of the map frame is the "sensor origin", thus the obstacle range is long.
    """
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    root = data[namespace] if namespace else data
    for name in ('local_costmap', 'global_costmap'):
        layer = root[name][name]['ros__parameters']['obstacle_layer']
        layer['observation_sources'] += ' peers'
        layer['peers'] = {'topic': 'fleet/peer_obstacles', 'data_type': 'PointCloud2',
                          'marking': True, 'clearing': False, 'min_obstacle_height': 0.0,
                          'max_obstacle_height': 2.0, 'obstacle_max_range': 50.0,
                          'obstacle_min_range': 0.0, 'observation_persistence': 0.0,
                          'expected_update_rate': 0.0}
    out = os.path.join(out_dir, f'{namespace or "root"}_fleet_nav2_params.yaml')
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out


def robot_bridge_config(namespace, robot_name, out_dir):
    """Gazebo<->ROS topic map for one robot. It comes from the file of the default robot."""
    path = share('forklift_simulation', 'config', 'gz_bridge.yaml')
    if not namespace:
        return path
    with open(path, encoding='utf-8') as f:
        entries = yaml.safe_load(f)
    for e in entries:
        e['gz_topic_name'] = re.sub(r'^/forklift/', f'/{robot_name}/', e['gz_topic_name'])
        e['ros_topic_name'] = f'/{namespace}{e["ros_topic_name"]}'
    out = os.path.join(out_dir, f'{namespace}_gz_bridge.yaml')
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(entries, f, sort_keys=False)
    return out


def include(pkg, launch_file, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(pkg, 'launch', launch_file)),
        launch_arguments={k: str(v) for k, v in args.items()}.items())


def robot(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    ns = arg('namespace').strip('/')
    prefix = f'{ns}/' if ns else ''
    robot_name = ns or 'forklift'                  # prefix of the Gazebo entity and topics
    out_dir = tempfile.mkdtemp(prefix=f'forklift_{robot_name}_')

    def params(pkg, name):
        return rewrite_params(share(pkg, 'config', name), ns, prefix, out_dir)
    controllers = params('forklift_control', 'controllers.yaml')
    if ns:
        # diff_drive_controller adds the namespace prefix to its frames a second time
        with open(controllers, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        data[ns]['forklift_base_controller']['ros__parameters']['tf_frame_prefix_enable'] = \
            False
        with open(controllers, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False)
    xacro_args = [f'controllers_file:={controllers}', f'robot_name:={robot_name}',
                  f'frame_prefix:={prefix}', f'ros_namespace:={"/" + ns if ns else ""}']
    navigation_mode = arg('mode')
    sim = {'use_sim_time': True}
    actions = [
        include('forklift_description', 'robot_state_publisher.launch.py',
                sim_backend='gazebo', use_sim_time='true', frame_prefix=prefix,
                extensions=share('forklift_simulation', 'urdf', 'forklift_gazebo.xacro'),
                extra_xacro_args=' '.join(xacro_args)),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'warehouse', '-name', robot_name,
                        '-topic', 'robot_description', '-x', arg('x'), '-y', arg('y'),
                        '-z', '0.02', '-Y', arg('yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             output='screen',
             parameters=[{'config_file': robot_bridge_config(ns, robot_name, out_dir),
                          'use_sim_time': True}]),
        include('forklift_control', 'control.launch.py', use_sim_time='true',
                controller_manager=f'/{prefix}controller_manager',
                controllers_file=controllers,
                twist_mux_file=params('forklift_control', 'twist_mux.yaml'),
                lift_file=params('forklift_control', 'lift_controller.yaml'),
                rear_safety_file=params('forklift_control', 'rear_safety.yaml'),
                frame_prefix=prefix),
        include('forklift_localization', 'ekf.launch.py', use_sim_time='true',
                ekf_file=params('forklift_localization', 'ekf.yaml')),
    ]
    if arg('perception').lower() == 'true':
        actions.append(include('forklift_perception', 'perception.launch.py',
                               use_sim_time='true',
                               params_file=params('forklift_perception', 'qr_detector.yaml')))
    fleet = arg('fleet_agent').lower() == 'true'
    peers = [p for p in arg('fleet_peers').split(',') if p.strip()]
    if navigation_mode != 'none':
        nav2_params = params('forklift_navigation', 'nav2_params.yaml')
        if fleet:
            nav2_params = add_peer_obstacles(nav2_params, ns, out_dir)
        actions.append(include('forklift_navigation', 'navigation.launch.py',
                               mode=navigation_mode, x=arg('x'), y=arg('y'), yaw=arg('yaw'),
                               frame_prefix=prefix, params_file=nav2_params))
        actions.append(Node(
            package='forklift_control', executable='load_handler', output='screen',
            parameters=[params('forklift_control', 'load_handler.yaml'), sim,
                        {'frame_prefix': prefix}]))
    if navigation_mode == 'navigation' and arg('mission_manager').lower() == 'true':
        record_dir = os.path.expanduser('~/.ros/forklift_missions') + (f'/{ns}' if ns else '')
        actions.append(Node(
            package='forklift_task_manager', executable='mission_manager', output='screen',
            parameters=[params('forklift_task_manager', 'mission_manager.yaml'), sim,
                        {'autostart': arg('start_mission').lower() == 'true',
                         'frame_prefix': prefix, 'robot_id': robot_name,
                         'record_dir': record_dir}]))
    if fleet:
        actions.append(Node(
            package='forklift_fleet', executable='fleet_agent', output='screen',
            parameters=[sim, {'robot_id': robot_name, 'frame_prefix': prefix,
                              'peers': peers or ['']}]))
    return [GroupAction([PushRosNamespace(ns), *actions]) if ns else GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='"" = single robot at the root; e.g. forklift_1'),
        DeclareLaunchArgument('x', default_value='-6.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('mode', default_value='navigation',
                              description='navigation | mapping | none'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('mission_manager', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='false'),
        DeclareLaunchArgument('fleet_peers', default_value='',
                              description='other robots (comma-separated namespaces)'),
        DeclareLaunchArgument('fleet_agent', default_value='false',
                              description='Report status to a fleet coordinator'),
        OpaqueFunction(function=robot),
    ])
