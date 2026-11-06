"""Start the QR perception node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mission_file', default_value='',
                              description='Mission YAML ("" = package default)'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('forklift_perception'), 'config', 'qr_detector.yaml'])),
        Node(
            package='forklift_perception', executable='qr_detector', output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'mission_file': LaunchConfiguration('mission_file')}],
        ),
    ])
