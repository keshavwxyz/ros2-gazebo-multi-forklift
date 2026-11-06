"""
EKF regression test (no simulator): the EKF must not lose late wheel odometry.

The wheel odometry and the IMU come from different processes. Thus, the odometry can
arrive some hundred ms late in a burst (measured in Gazebo: 345 ms). During this time,
the IMU messages move the filter forward. robot_localization discards measurements that
are older than its last update if `smooth_lagged_data` is not enabled. The motion in that
period is then lost (the Gazebo motion test showed a distance that was 0.25 m too small).

The test uses the real EKF config with synthetic data. The IMU data is on time. The
odometry is for a straight motion at 0.5 m/s. The first 0.4 s of the odometry arrive
0.4 s late. After 2 s of motion, the filtered position must be 1.0 m.
"""

import time
import unittest

from geometry_msgs.msg import Quaternion
import launch
from launch.substitutions import PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Imu

SPEED, DURATION, LAG = 0.5, 2.0, 0.4


@pytest.mark.launch_test
def generate_test_description():
    ekf = launch_ros.actions.Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[PathJoinSubstitution([FindPackageShare('forklift_localization'), 'config',
                                          'ekf.yaml']), {'use_sim_time': False}])
    return launch.LaunchDescription([ekf, launch_testing.actions.ReadyToTest()])


class TestLaggedOdometry(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('ekf_lag_tester')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_late_odometry_is_fused(self):
        node = self.node
        odom_pub = node.create_publisher(Odometry, '/forklift_base_controller/odom', 50)
        imu_pub = node.create_publisher(Imu, '/imu/data', 50)
        latest = {}
        node.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: latest.__setitem__('f', m), 10)

        def odom(stamp, v):
            m = Odometry()
            m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, 'odom', 'base_footprint'
            m.twist.twist.linear.x = v
            m.twist.covariance[0] = m.twist.covariance[7] = 0.0025
            m.twist.covariance[35] = 0.01
            for i in (14, 21, 28):
                m.twist.covariance[i] = 1e6
            return m

        def imu(stamp):
            m = Imu()
            m.header.stamp, m.header.frame_id = stamp, 'imu_link'
            m.orientation = Quaternion(w=1.0)
            m.angular_velocity_covariance[0] = m.angular_velocity_covariance[4] = 4e-6
            m.angular_velocity_covariance[8] = 4e-6
            m.linear_acceleration.z = 9.81
            m.linear_acceleration_covariance[0] = 4e-4
            return m

        # Wait for the EKF while the robot is stationary.
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and 'f' not in latest:
            now = node.get_clock().now().to_msg()
            odom_pub.publish(odom(now, 0.0))
            imu_pub.publish(imu(now))
            rclpy.spin_once(node, timeout_sec=0.02)
        self.assertIn('f', latest, 'EKF produced no output')
        x0 = latest['f'].pose.pose.position.x

        start = node.get_clock().now()
        held, released = [], False
        # stamps that the test really published, in seconds
        first_moving = first_stopped = None
        t_next_odom = t_next_imu = 0.0
        while True:
            t = (node.get_clock().now() - start).nanoseconds * 1e-9
            if t > DURATION + 1.0:
                break
            stamp = node.get_clock().now().to_msg()
            if t >= t_next_imu:
                imu_pub.publish(imu(stamp))          # IMU always on time (100 Hz)
                t_next_imu += 0.01
            if t >= t_next_odom:
                moving = t < DURATION
                msg = odom(stamp, SPEED if moving else 0.0)
                secs = stamp.sec + stamp.nanosec * 1e-9
                if moving and first_moving is None:
                    first_moving = secs
                if not moving and first_stopped is None:
                    first_stopped = secs
                if t < LAG:
                    held.append(msg)                 # first 0.4 s of odometry is delayed...
                else:
                    if not released:                 # ...then it arrives in a burst
                        for h in held:
                            odom_pub.publish(h)
                        released = True
                    odom_pub.publish(msg)
                t_next_odom += 0.02
            rclpy.spin_once(node, timeout_sec=0.002)

        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        travelled = latest['f'].pose.pose.position.x - x0
        self.assertTrue(Time.from_msg(latest['f'].header.stamp) > start)
        # Expected distance from the stamps that the test really sent (this is robust to
        # publisher jitter at high CPU load): the velocity measurements give SPEED from
        # first_moving to first_stopped.
        expected = SPEED * (first_stopped - first_moving)
        self.assertAlmostEqual(travelled, expected, delta=0.05,
                               msg=f'EKF travelled {travelled:.3f} m, expected '
                                   f'{expected:.3f} m (late odometry lost?)')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
