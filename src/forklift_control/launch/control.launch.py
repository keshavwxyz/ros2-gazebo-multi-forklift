"""
Start the ros2_control controllers and the command layer.

  twist_mux (navigation / approach / teleop, e-stop lock) -> rear_safety (rear LiDAR)
      -> forklift_base_controller/cmd_vel;  lift_controller (SetForkHeight action)

This file does not depend on the backend. It needs a controller_manager that runs (the
gz_ros2_control plugin of Gazebo, or ros2_control_node with the mock backend). The
spawners wait until the controller manager is available. Thus, no startup delays are
necessary. All topic names are relative. For a robot with a namespace, include this file
in a PushRosNamespace and give config files that have the namespace (see
forklift_bringup/launch/robot.launch.py). The default values are for one robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('forklift_control')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def cfg(name):
        return PathJoinSubstitution([pkg, 'config', name])
    spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'forklift_base_controller', 'fork_lift_controller',
                   '--controller-manager', LaunchConfiguration('controller_manager'),
                   '--controller-manager-timeout', '120',
                   '--param-file', LaunchConfiguration('controllers_file')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', output='screen',
        parameters=[LaunchConfiguration('twist_mux_file'), {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', 'cmd_vel/safety_in')],
    )
    rear_safety = Node(
        package='forklift_control', executable='rear_safety', output='screen',
        parameters=[LaunchConfiguration('rear_safety_file'),
                    {'use_sim_time': use_sim_time,
                     'frame_prefix': LaunchConfiguration('frame_prefix')}],
        remappings=[('cmd_vel_in', 'cmd_vel/safety_in'),
                    ('cmd_vel_out', 'forklift_base_controller/cmd_vel')],
    )
    lift = Node(
        package='forklift_control', executable='lift_controller', output='screen',
        parameters=[LaunchConfiguration('lift_file'), {'use_sim_time': use_sim_time}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager'),
        DeclareLaunchArgument('controllers_file', default_value=cfg('controllers.yaml')),
        DeclareLaunchArgument('twist_mux_file', default_value=cfg('twist_mux.yaml')),
        DeclareLaunchArgument('lift_file', default_value=cfg('lift_controller.yaml')),
        DeclareLaunchArgument('rear_safety_file', default_value=cfg('rear_safety.yaml')),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        spawner, twist_mux, rear_safety, lift,
    ])
