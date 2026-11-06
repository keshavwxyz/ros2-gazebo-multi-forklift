"""
Gazebo bringup for two forklifts.

  ros2 launch forklift_bringup multi_sim.launch.py                    # robots + coordinator
  ros2 launch forklift_bringup multi_sim.launch.py start_mission:=true
  ros2 action send_goal /fleet/run forklift_interfaces/action/RunMission "{}"

This bringup starts:
  - One shared world with the QR-labelled loads.
  - One robot.launch.py instance for each robot in forklift_fleet/config/fleet.yaml
    (namespace, TF prefix, and spawn pose, which is also the home pose). Each robot has
    its own controllers, rear safety, EKF, AMCL, Nav2, perception, load-handling skills,
    mission manager and fleet agent.
  - One fleet coordinator.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def include(pkg, launch_file, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(pkg, 'launch', launch_file)),
        launch_arguments={k: v if not isinstance(v, str) else v for k, v in args.items()}
        .items())


def robots(context):
    fleet_file = share('forklift_fleet', 'config', 'fleet.yaml')
    with open(fleet_file, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['fleet_coordinator']['ros__parameters']
    actions = []
    for rid in cfg['robots']:
        x, y, yaw = cfg[f'home.{rid}'] if f'home.{rid}' in cfg else cfg['home'][rid]
        # Give each robot.launch.py argument explicitly. Included launch files get the
        # configurations of the parent. Here, start_mission:=true must start the *fleet*
        # mission. It must not start the mission manager of each robot.
        actions.append(include('forklift_bringup', 'robot.launch.py', namespace=rid,
                               x=str(x), y=str(y), yaw=str(yaw), mode='navigation',
                               perception=LaunchConfiguration('perception'),
                               mission_manager='true', start_mission='false',
                               fleet_agent='true',
                               fleet_peers=','.join(p for p in cfg['robots'] if p != rid)))
    actions.append(Node(
        package='forklift_fleet', executable='fleet_coordinator', output='screen',
        parameters=[fleet_file, {'use_sim_time': True,
                                 'autostart': LaunchConfiguration('start_mission')
                                 .perform(context).lower() == 'true'}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='false',
                              description='Start the fleet mission automatically'),
        include('forklift_simulation', 'gazebo.launch.py', gui=LaunchConfiguration('gui'),
                spawn_robot='false'),
        OpaqueFunction(function=robots),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', share('forklift_bringup', 'rviz', 'multi.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
