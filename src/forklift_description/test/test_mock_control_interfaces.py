"""
Launch test: the ros2_control block of the URDF loads in a real controller_manager.

The test uses the mock_components backend, thus no simulator is necessary. It makes sure
of these items:
  - Each joint has the expected command and state interfaces.
  - The hardware component becomes active.
  - robot_state_publisher makes the full TF tree from /joint_states.
"""

import pathlib
import time
import unittest

from controller_manager_msgs.srv import ListHardwareComponents, ListHardwareInterfaces
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import xacro

XACRO = pathlib.Path(__file__).resolve().parents[1] / 'urdf' / 'forklift.urdf.xacro'

EXPECTED_COMMAND = {
    'left_drive_wheel_joint/velocity',
    'right_drive_wheel_joint/velocity',
    'fork_lift_joint/velocity',
}
EXPECTED_STATE = {
    'left_drive_wheel_joint/position', 'left_drive_wheel_joint/velocity',
    'right_drive_wheel_joint/position', 'right_drive_wheel_joint/velocity',
    'fork_lift_joint/position', 'fork_lift_joint/velocity', 'fork_lift_joint/effort',
}
EXPECTED_FRAMES = [
    'base_link', 'left_drive_wheel_link', 'right_drive_wheel_link', 'left_caster_link',
    'right_caster_link', 'counterweight_link', 'overhead_guard_link', 'mast_link',
    'fork_carriage_link', 'left_fork_link', 'right_fork_link', 'fork_tip_frame',
    'lidar_link', 'camera_link', 'camera_optical_frame', 'imu_link',
]


@pytest.mark.launch_test
def generate_test_description():
    robot_description = xacro.process_file(str(XACRO), mappings={'sim_backend': 'mock'}).toxml()
    rsp = launch_ros.actions.Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])
    control = launch_ros.actions.Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[{'update_rate': 50}],
        remappings=[('~/robot_description', '/robot_description')])
    return launch.LaunchDescription([rsp, control, launch_testing.actions.ReadyToTest()])


def call(node, client, request, timeout=10.0):
    assert client.wait_for_service(timeout_sec=timeout), f'{client.srv_name} unavailable'
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), f'{client.srv_name} timed out'
    return future.result()


class TestMockControlInterfaces(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mock_interface_tester')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for_interfaces(self, timeout=20.0):
        client = self.node.create_client(ListHardwareInterfaces,
                                         '/controller_manager/list_hardware_interfaces')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            resp = call(self.node, client, ListHardwareInterfaces.Request())
            if resp.command_interfaces:
                return resp
            time.sleep(0.5)   # controller_manager waits for /robot_description first
        self.fail('controller_manager never loaded the hardware interfaces')

    def test_hardware_interfaces(self):
        resp = self.wait_for_interfaces()
        command = {i.name for i in resp.command_interfaces}
        state = {i.name for i in resp.state_interfaces}
        self.assertEqual(command, EXPECTED_COMMAND)
        self.assertEqual(state, EXPECTED_STATE)
        # No interface is claimed until the controllers start (Phase 2).
        self.assertTrue(all(i.is_available and not i.is_claimed for i in resp.command_interfaces))

    def test_hardware_component_active(self):
        self.wait_for_interfaces()
        client = self.node.create_client(ListHardwareComponents,
                                         '/controller_manager/list_hardware_components')
        resp = call(self.node, client, ListHardwareComponents.Request())
        names = {c.name: c.state.label for c in resp.component}
        self.assertEqual(names, {'ForkliftSystem': 'active'})

    def test_tf_tree_complete(self):
        # Publish the joint states for the movable joints (there are no controllers yet).
        pub = self.node.create_publisher(JointState, '/joint_states', 10)
        buffer = Buffer()
        TransformListener(buffer, self.node)
        msg = JointState(
            name=['left_drive_wheel_joint', 'right_drive_wheel_joint', 'fork_lift_joint'],
            position=[0.0, 0.0, 0.4])
        missing = list(EXPECTED_FRAMES)
        deadline = time.monotonic() + 15.0
        while missing and time.monotonic() < deadline:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            missing = [f for f in missing
                       if not buffer.can_transform('base_footprint', f, rclpy.time.Time())]
        self.assertEqual(missing, [], f'frames missing from TF: {missing}')
        tip = buffer.lookup_transform('base_footprint', 'fork_tip_frame', rclpy.time.Time())
        self.assertAlmostEqual(tip.transform.translation.z, 0.05 + 0.4, places=4)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
