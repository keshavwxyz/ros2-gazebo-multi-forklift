"""Publish the forklift robot_description and TF with robot_state_publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('forklift_description'), 'urdf', 'forklift.urdf.xacro'])

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' sim_backend:=', LaunchConfiguration('sim_backend'),
            ' params_file:=', LaunchConfiguration('params_file'),
            ' extensions:=', LaunchConfiguration('extensions'),
            ' ', LaunchConfiguration('extra_xacro_args'),
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('sim_backend', default_value='mock',
                              description='ros2_control hardware plugin: mock | gazebo'),
        DeclareLaunchArgument('params_file', default_value='""',
                              description='Absolute path to a geometry YAML (empty = default)'),
        DeclareLaunchArgument('extensions', default_value='""',
                              description='Absolute path to a simulator-specific xacro'),
        DeclareLaunchArgument('extra_xacro_args', default_value='',
                              description='Additional "name:=value" xacro arguments, '
                                          'e.g. for the extension file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('frame_prefix', default_value='',
                              description='Prepended to every TF frame, e.g. "forklift_1/"'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
            }],
        ),
    ])
