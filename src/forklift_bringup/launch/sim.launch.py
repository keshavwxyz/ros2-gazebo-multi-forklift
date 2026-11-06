"""
Full Gazebo simulation bringup with one forklift.

This bringup starts the world, robot, controllers, rear safety, EKF, perception,
navigation, load handling and mission manager. For two forklifts, use multi_sim.launch.py.

  ros2 launch forklift_bringup sim.launch.py                    # saved-map navigation
  ros2 launch forklift_bringup sim.launch.py start_mission:=true # run the 3-box mission
  ros2 launch forklift_bringup sim.launch.py mode:=mapping spawn_loads:=false
  ros2 launch forklift_bringup sim.launch.py mode:=none         # no localization / Nav2
  ros2 launch forklift_bringup sim.launch.py gui:=true rviz:=true

AMCL also uses the spawn pose (x, y, yaw, world frame) as its initial pose. The frame of
the saved map aligns with the world/mission frame (docs/navigation.md).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=arguments.items())


def generate_launch_description():
    sim_share = FindPackageShare('forklift_simulation')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [sim_share, 'worlds', 'warehouse.sdf'])),
        DeclareLaunchArgument('x', default_value='-6.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('perception', default_value='true',
                              description='Start the QR perception node'),
        DeclareLaunchArgument('decoys', default_value='false',
                              description='Spawn test-only props with invalid QR codes'),
        DeclareLaunchArgument('spawn_loads', default_value='true',
                              description='Spawn the QR-labelled loads (false for mapping)'),
        DeclareLaunchArgument('mode', default_value='navigation',
                              description='navigation (saved map + AMCL + Nav2) | mapping '
                                          '(slam_toolbox) | none'),
        DeclareLaunchArgument('start_mission', default_value='false',
                              description='Start the mission automatically (otherwise send '
                                          'a /mission/run goal)'),

        include('forklift_simulation', 'gazebo.launch.py',
                world=LaunchConfiguration('world'), gui=LaunchConfiguration('gui'),
                decoys=LaunchConfiguration('decoys'),
                spawn_loads=LaunchConfiguration('spawn_loads'), spawn_robot='false'),
        # The robot: this is the same per-robot launch file that the two-forklift bringup uses.
        # Here it is in the root namespace (frames without a prefix, initial config files).
        include('forklift_bringup', 'robot.launch.py', namespace='',
                x=LaunchConfiguration('x'), y=LaunchConfiguration('y'),
                yaw=LaunchConfiguration('yaw'), mode=LaunchConfiguration('mode'),
                perception=LaunchConfiguration('perception'),
                start_mission=LaunchConfiguration('start_mission'),
                mission_manager='true', fleet_agent='false'),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare('forklift_bringup'), 'rviz', 'sim.rviz'])],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
