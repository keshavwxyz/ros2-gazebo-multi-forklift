"""Start the EKF that fuses wheel odometry and IMU into odom -> base_footprint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ekf_file', default_value=PathJoinSubstitution(
            [FindPackageShare('forklift_localization'), 'config', 'ekf.yaml']),
            description='EKF parameters (namespace-rewritten copy for a namespaced robot)'),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('ekf_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
