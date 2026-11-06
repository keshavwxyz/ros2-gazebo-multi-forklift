"""
Tier-3 simulation test: drive, stop, e-stop and fork lift in Gazebo.

The commands go through the real command path (twist_mux -> diff_drive_controller, and the
SetForkHeight action -> lift_controller -> joint_trajectory_controller). The test uses the
Gazebo ground truth (/ground_truth/odom) only to *evaluate* the result and the odometry.
The test methods have numbers because they use one simulated robot and run in sequence.

The robot spawns at (-6, -2), pointed to +x. That lane is clear for approximately 11 m.
The turning circle in place (radius approximately 1.4 m) does not touch the columns at
y = 0. (From the default home pose, the forks touch a column after approximately 2.9 m.
This blocked the rotation test.)
"""

import math
import unittest

from forklift_test.helpers import (angle_diff, assert_clean_shutdown, Driver, set_fork_height,
                                   spin_for, wait_for_controllers, yaw_of)
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

LIFT_TOLERANCE = 0.01   # m


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('forklift_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'x': '-6.0', 'y': '-2.0', 'yaw': '0.0', 'mode': 'none'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class Pose2D:

    def __init__(self, odom):
        p = odom.pose.pose
        self.x, self.y, self.yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        self.v = odom.twist.twist.linear.x
        self.w = odom.twist.twist.angular.z


class TestMotion(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('motion_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.latest = {}
        for topic in ('/ground_truth/odom', '/odometry/filtered',
                      '/forklift_base_controller/odom'):
            cls.node.create_subscription(Odometry, topic,
                                         lambda m, t=topic: cls.latest.__setitem__(t, m), 10)
        cls.node.create_subscription(JointState, '/joint_states',
                                     lambda m: cls.latest.__setitem__('js', m), 10)
        cls.estop = cls.node.create_publisher(Bool, '/estop', 10)
        cls.driver = Driver(cls.node)
        wait_for_controllers(cls.node, ['joint_state_broadcaster', 'forklift_base_controller',
                                        'fork_lift_controller'])
        spin_for(cls.node, 2.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def pose(self, topic):
        self.assertIn(topic, self.latest, f'no data on {topic}')
        return Pose2D(self.latest[topic])

    def snapshot(self):
        return {t: self.pose(t) for t in ('/ground_truth/odom', '/odometry/filtered',
                                          '/forklift_base_controller/odom')}

    def lift_position(self):
        js = self.latest['js']
        return js.position[js.name.index('fork_lift_joint')]

    def assert_odometry_matches_truth(self, before, after, pos_tol, yaw_tol_deg):
        gt0, gt1 = before['/ground_truth/odom'], after['/ground_truth/odom']
        gt_dist = math.hypot(gt1.x - gt0.x, gt1.y - gt0.y)
        gt_dyaw = angle_diff(gt1.yaw, gt0.yaw)
        for topic in ('/odometry/filtered', '/forklift_base_controller/odom'):
            o0, o1 = before[topic], after[topic]
            with self.subTest(estimate=topic):
                self.assertAlmostEqual(math.hypot(o1.x - o0.x, o1.y - o0.y), gt_dist,
                                       delta=pos_tol)
                self.assertAlmostEqual(math.degrees(angle_diff(angle_diff(o1.yaw, o0.yaw),
                                                               gt_dyaw)), 0.0,
                                       delta=yaw_tol_deg)
        return gt_dist, gt_dyaw

    # ------------------------------------------------------------------ drive

    def test_1_drive_forward_and_stop(self):
        before = self.snapshot()
        self.driver.drive(0.5, 0.0, 4.0)
        self.driver.drive(0.0, 0.0, 2.0)
        after = self.snapshot()
        dist, dyaw = self.assert_odometry_matches_truth(before, after, 0.05, 1.0)
        # 4 s at 0.5 m/s with 0.5 m/s^2 ramps: approximately 1.75-2.0 m
        self.assertGreater(dist, 1.6)
        self.assertLess(dist, 2.1)
        self.assertLess(abs(math.degrees(dyaw)), 1.0, 'robot did not drive straight')
        self.assertLess(abs(after['/ground_truth/odom'].v), 0.01, 'robot did not stop')

    def test_2_deadman_stops_robot(self):
        self.driver.drive(0.4, 0.0, 2.0)
        self.assertGreater(self.pose('/ground_truth/odom').v, 0.3)
        # stop the commands: the timeouts of the mux and the controller must stop the robot
        spin_for(self.node, 2.5)
        self.assertLess(abs(self.pose('/ground_truth/odom').v), 0.01)

    def test_3_rotate_in_place(self):
        # Turn left, then turn right again. Thus, the subsequent tests continue on the clear lane.
        for w, sign in ((0.5, 1.0), (-0.5, -1.0)):
            with self.subTest(angular_velocity=w):
                before = self.snapshot()
                self.driver.drive(0.0, w, 3.0)
                self.driver.drive(0.0, 0.0, 1.5)
                after = self.snapshot()
                dist, dyaw = self.assert_odometry_matches_truth(before, after, 0.05, 2.0)
                self.assertGreater(sign * math.degrees(dyaw), 60.0)
                self.assertLess(dist, 0.05, 'rotation should be about the drive axle')

    def test_4_estop_blocks_and_releases(self):
        self.estop.publish(Bool(data=True))
        spin_for(self.node, 0.5)
        before = self.pose('/ground_truth/odom')
        self.driver.drive(0.5, 0.0, 2.0)
        after = self.pose('/ground_truth/odom')
        self.assertLess(math.hypot(after.x - before.x, after.y - before.y), 0.02,
                        'robot moved with e-stop engaged')
        self.estop.publish(Bool(data=False))
        spin_for(self.node, 0.5)
        self.driver.drive(0.3, 0.0, 2.0)
        moved = self.pose('/ground_truth/odom')
        self.driver.drive(0.0, 0.0, 1.5)
        self.assertGreater(math.hypot(moved.x - after.x, moved.y - after.y), 0.2,
                           'robot did not move after e-stop release')

    # ------------------------------------------------------------------ lift

    def test_5_lift_raise_and_lower(self):
        for height in (0.6, 1.0, 0.25, 0.0):
            with self.subTest(height=height):
                accepted, result = set_fork_height(self.node, height)
                self.assertTrue(accepted)
                self.assertTrue(result.success, result.message)
                spin_for(self.node, 0.3)
                self.assertAlmostEqual(self.lift_position(), height, delta=LIFT_TOLERANCE)

    def test_6_lift_rejects_out_of_range_goals(self):
        for height in (1.2, -0.05):
            with self.subTest(height=height):
                accepted, _ = set_fork_height(self.node, height)
                self.assertFalse(accepted)
        self.assertAlmostEqual(self.lift_position(), 0.0, delta=LIFT_TOLERANCE)

    def test_7_lift_holds_while_driving(self):
        accepted, result = set_fork_height(self.node, 0.3)
        self.assertTrue(accepted and result.success)
        self.driver.drive(0.4, 0.0, 2.0)
        self.driver.drive(0.0, 0.0, 1.5)
        self.assertAlmostEqual(self.lift_position(), 0.3, delta=LIFT_TOLERANCE)
        accepted, result = set_fork_height(self.node, 0.0)
        self.assertTrue(accepted and result.success)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited(self, proc_info):
        assert_clean_shutdown(self, proc_info)
