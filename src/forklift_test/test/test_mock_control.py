"""
Tier-2 integration test without a simulator (ros2_control mock hardware).

The test uses the same controllers, twist_mux and lift action server as the Gazebo bringup.
It makes sure of these items:
  - The lift action gets to the goals and rejects goals that are not valid.
  - The drive commands get to the wheel joints.
  - The e-stop lock blocks the drive commands.
This test runs in CI.
"""

import unittest

from forklift_test.helpers import Driver, set_fork_height, spin_for, wait_for_controllers
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    mock = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('forklift_bringup'), 'launch', 'mock.launch.py'])))
    return launch.LaunchDescription([mock, launch_testing.actions.ReadyToTest()])


class TestMockControl(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mock_control_tester')
        cls.js = None

        def on_js(msg):
            cls.js = msg
        cls.node.create_subscription(JointState, '/joint_states', on_js, 10)
        cls.estop = cls.node.create_publisher(Bool, '/estop', 10)
        cls.driver = Driver(cls.node)
        wait_for_controllers(cls.node, ['joint_state_broadcaster', 'forklift_base_controller',
                                        'fork_lift_controller'], timeout_wall=60.0)
        spin_for(cls.node, 1.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def joint(self, name, field='position'):
        return getattr(self.js, field)[self.js.name.index(name)]

    def test_lift_action_reaches_goals(self):
        for height in (0.5, 0.0):
            with self.subTest(height=height):
                accepted, result = set_fork_height(self.node, height)
                self.assertTrue(accepted)
                self.assertTrue(result.success, result.message)
                spin_for(self.node, 0.2)
                self.assertAlmostEqual(self.joint('fork_lift_joint'), height, delta=0.01)

    def test_lift_action_rejects_out_of_range(self):
        self.assertFalse(set_fork_height(self.node, 1.5)[0])
        self.assertFalse(set_fork_height(self.node, -0.2)[0])

    def test_drive_command_reaches_wheels_and_estop_blocks(self):
        start = self.joint('left_drive_wheel_joint')
        self.driver.drive(0.3, 0.0, 1.5)
        moved = self.joint('left_drive_wheel_joint')
        self.assertGreater(moved - start, 0.5, 'wheel did not turn')
        self.driver.drive(0.0, 0.0, 1.5)

        self.estop.publish(Bool(data=True))
        spin_for(self.node, 0.3)
        before = self.joint('left_drive_wheel_joint')
        self.driver.drive(0.3, 0.0, 1.5)
        self.assertLess(abs(self.joint('left_drive_wheel_joint') - before), 0.05,
                        'wheels turned with e-stop engaged')
        self.estop.publish(Bool(data=False))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
