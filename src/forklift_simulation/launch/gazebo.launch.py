"""
Start Gazebo Sim with the warehouse world, the forklift and the QR-labelled loads.

This file also bridges the topics of the robot to ROS.

The loads come from the mission config (the only source of these values): one load for
each box at its pickup pose. The file uses the models from scripts/generate_load_models.py.
With decoys:=true, the file adds props with QR codes that are not valid, for the tests only
(config/test_decoys.yaml).

robot_state_publisher must publish /robot_description (sim_backend:=gazebo) with the
forklift_gazebo.xacro of this package as the `extensions` argument. sim.launch.py of
forklift_bringup does this.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from forklift_task_manager.mission_config import default_mission_path, load_mission
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def gazebo(args, condition):
    # Start without a shell. Thus, SIGINT/SIGTERM from launch go to Gazebo itself.
    # (gz_sim.launch.py of ros_gz_sim puts it in `sh -c`. This left orphan servers that
    # continued to publish into later runs.) If Gazebo stops, the full bringup stops.
    return ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', *args, LaunchConfiguration('world')],
        output='screen', name='gazebo', sigterm_timeout='5', sigkill_timeout='5',
        on_exit=Shutdown(reason='Gazebo exited'), condition=condition)


def _spawn(name, sdf_file, x, y, yaw):
    return Node(
        package='ros_gz_sim', executable='create', output='screen', name=f'spawn_{name}',
        arguments=['-world', 'warehouse', '-name', name, '-file', sdf_file,
                   '-x', str(x), '-y', str(y), '-z', '0.002', '-Y', str(yaw)])


def _model_file(models_dir, model, expected_payload):
    """Return the SDF path of the model. Raise an error if it is missing or has a wrong payload."""
    config = os.path.join(models_dir, model, 'model.config')
    if not os.path.exists(config):
        raise RuntimeError(f'missing Gazebo model {model}; run '
                           'src/forklift_simulation/scripts/generate_load_models.py')
    match = re.search(r'payload=(\S+)</description>', open(config, encoding='utf-8').read())
    if not match or match.group(1) != expected_payload:
        raise RuntimeError(f'{model} carries QR payload {match and match.group(1)!r}, mission '
                           f'expects {expected_payload!r}; regenerate the load models')
    return os.path.join(models_dir, model, 'model.sdf')


def spawn_loads(context):
    share = get_package_share_directory('forklift_simulation')
    models_dir = os.path.join(share, 'models')
    actions = []
    if LaunchConfiguration('spawn_loads').perform(context).lower() == 'true':
        mission_file = LaunchConfiguration('mission_file').perform(context) or \
            default_mission_path()
        mission = load_mission(mission_file)
        for box in mission.boxes:
            pose = mission.pickups[box.pickup]
            model = f'qr_load_{box.box_id.lower()}'
            actions.append(_spawn(f'load_{box.box_id.lower()}',
                                  _model_file(models_dir, model, box.payload),
                                  pose.x, pose.y, pose.yaw))
        # Ground truth of each load, for evaluation only (the
        # tests make sure that no load is teleported).
        names = [f'load_{b.box_id.lower()}' for b in mission.boxes]
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='load_truth_bridge',
            output='screen', parameters=[{'use_sim_time': True}],
            arguments=[f'/model/{n}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'
                       for n in names],
            remappings=[(f'/model/{n}/pose', f'/ground_truth/loads/{n}') for n in names]))
    if LaunchConfiguration('decoys').perform(context).lower() == 'true':
        import yaml
        with open(os.path.join(share, 'config', 'test_decoys.yaml'), encoding='utf-8') as f:
            for d in yaml.safe_load(f)['decoys']:
                model = f'qr_load_{d["name"].lower()}'
                actions.append(_spawn(d['name'], _model_file(models_dir, model, d['payload']),
                                      d['x'], d['y'], d['yaw']))
    return actions


def generate_launch_description():
    pkg = FindPackageShare('forklift_simulation')
    gui = LaunchConfiguration('gui')

    # In server-only mode, the camera and the LiDAR use EGL. Thus, no display is necessary.
    gz_headless = gazebo(['-s', '--headless-rendering'], UnlessCondition(gui))
    gz_with_gui = gazebo([], IfCondition(gui))

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        condition=IfCondition(LaunchConfiguration('spawn_robot')),
        arguments=[
            '-world', 'warehouse', '-name', 'forklift', '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
            '-z', '0.02', '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # Simulation clock: one for each world (the robot bridges carry only robot topics).
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('spawn_robot')),
        parameters=[{
            'config_file': PathJoinSubstitution([pkg, 'config', 'gz_bridge.yaml']),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [pkg, 'worlds', 'warehouse.sdf'])),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Open the Gazebo GUI (costly on a laptop)'),
        DeclareLaunchArgument('spawn_robot', default_value='true',
                              description='Spawn the default robot here (false when '
                                          'forklift_bringup robot.launch.py spawns robots)'),
        DeclareLaunchArgument('x', default_value='-6.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('mission_file', default_value='',
                              description='Mission YAML ("" = forklift_task_manager default)'),
        DeclareLaunchArgument('spawn_loads', default_value='true',
                              description='Spawn one QR-labelled load per mission box'),
        DeclareLaunchArgument('decoys', default_value='false',
                              description='Spawn test-only props with invalid QR codes'),
        # Lets the worlds use model:// assets that this package installs (Phase 4 boxes).
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  PathJoinSubstitution([pkg, 'models'])),
        gz_headless,
        gz_with_gui,
        spawn,
        OpaqueFunction(function=spawn_loads),
        clock_bridge,
        bridge,
    ])
