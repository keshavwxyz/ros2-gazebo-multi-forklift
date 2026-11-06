"""
Bringup with ros2_control mock hardware and no simulator (CI and tier-2 integration tests).

This bringup runs the same controllers, twist_mux and lift action server as the Gazebo
bringup. It has no sensors and no EKF. The mock hardware only sends back the joint commands.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers = PathJoinSubstitution(
        [FindPackageShare('forklift_control'), 'config', 'controllers.yaml'])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('forklift_description'), 'launch',
                 'robot_state_publisher.launch.py'])),
            launch_arguments={'sim_backend': 'mock', 'use_sim_time': 'false'}.items()),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controllers, {'use_sim_time': False}],
             remappings=[('~/robot_description', '/robot_description')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('forklift_control'), 'launch', 'control.launch.py'])),
            launch_arguments={'use_sim_time': 'false'}.items()),
    ])
