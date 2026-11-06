"""
Tier-3 simulation test: sensors, joint states, timestamps and TF in Gazebo.

The test starts the full headless Gazebo bringup. For each sensor stream, it examines the
message type, rate, frame id, timestamp consistency with /clock and physical plausibility.
Then it examines the TF tree, its publishers and the ground-truth isolation.
This test needs Gazebo Sim with GPU/EGL rendering (camera, gpu_lidar).
"""

import math
import unittest

from forklift_test.helpers import (assert_clean_shutdown, spin_for, stamp_s, TopicRecorder,
                                   wait_for_controllers)
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

ROBOT_FRAMES = [
    'base_link', 'left_drive_wheel_link', 'right_drive_wheel_link', 'left_caster_link',
    'right_caster_link', 'counterweight_link', 'overhead_guard_link', 'mast_link',
    'fork_carriage_link', 'left_fork_link', 'right_fork_link', 'fork_tip_frame',
    'lidar_link', 'camera_link', 'camera_optical_frame', 'imu_link',
]
# topic: (type, expected Hz, frame_id)
STREAMS = {
    '/scan': (LaserScan, 10.0, 'lidar_link'),
    '/imu/data': (Imu, 100.0, 'imu_link'),
    '/camera/image_raw': (Image, 10.0, 'camera_optical_frame'),
    '/camera/camera_info': (CameraInfo, 10.0, 'camera_optical_frame'),
    '/joint_states': (JointState, 50.0, None),
    '/forklift_base_controller/odom': (Odometry, 50.0, 'odom'),
    '/odometry/filtered': (Odometry, 30.0, 'odom'),
}
RECORD_SECONDS = 5.0


@pytest.mark.launch_test
def generate_test_description():
    # mode:=none: this test examines the layers below localization and navigation.
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('forklift_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'mode': 'none'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestSensorsAndTf(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('sensor_tf_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        wait_for_controllers(cls.node, ['joint_state_broadcaster', 'forklift_base_controller',
                                        'fork_lift_controller'])
        topics = {t: v[0] for t, v in STREAMS.items()}
        topics['/tf'] = TFMessage
        cls.rec = TopicRecorder(cls.node, topics)
        spin_for(cls.node, 2.0)          # let the EKF and the sensors become stable
        cls.rec.clear()
        spin_for(cls.node, RECORD_SECONDS)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def last(self, topic):
        self.assertTrue(self.rec.msgs[topic], f'no messages on {topic}')
        return self.rec.msgs[topic][-1][1]

    # ---------------------------------------------------------------- streams

    def test_rates(self):
        for topic, (_, hz, _) in STREAMS.items():
            with self.subTest(topic=topic):
                rate = self.rec.rate(topic)
                self.assertGreater(rate, 0.8 * hz, f'{topic}: {rate:.1f} Hz < {hz} Hz')
                self.assertLess(rate, 1.2 * hz, f'{topic}: {rate:.1f} Hz > {hz} Hz')

    def test_frame_ids(self):
        for topic, (_, _, frame) in STREAMS.items():
            if frame is None:
                continue
            with self.subTest(topic=topic):
                frames = {m.header.frame_id for _, m in self.rec.msgs[topic]}
                self.assertEqual(frames, {frame})
        for topic in ('/forklift_base_controller/odom', '/odometry/filtered'):
            self.assertEqual(self.last(topic).child_frame_id, 'base_footprint')

    def test_timestamps_follow_sim_clock(self):
        for topic, (_, hz, _) in STREAMS.items():
            with self.subTest(topic=topic):
                msgs = self.rec.msgs[topic]
                stamps = [stamp_s(m.header.stamp) for _, m in msgs]
                self.assertTrue(all(b >= a for a, b in zip(stamps, stamps[1:])),
                                'stamps not monotonic')
                # Stamp compared with the latest /clock received
                # with it. /clock and the data are on
                # different topics, and their sequence is not
                # guaranteed. Thus, a message can arrive
                # before some 1 ms clock ticks. 20 ms permits that but
                # finds mix-ups of wall and sim time.
                lags = [clock - s for clock, s in zip([c for c, _ in msgs], stamps)
                        if clock is not None]
                self.assertTrue(lags)
                self.assertGreaterEqual(min(lags), -0.02, 'stamp ahead of /clock')
                self.assertLess(max(lags), max(3.0 / hz, 0.1), f'max lag {max(lags):.3f} s')
                self.assertGreater(stamps[0], 0.0, 'zero stamp (wall/sim time mix-up?)')

    # ---------------------------------------------------------------- plausibility

    def test_scan_plausible(self):
        scan = self.last('/scan')
        r = np.array(scan.ranges, dtype=float)
        self.assertEqual(len(r), 381)
        finite = r[np.isfinite(r)]
        self.assertGreater(len(finite), 0.5 * len(r))
        self.assertTrue(np.all((finite >= scan.range_min) & (finite <= scan.range_max)))
        angles = scan.angle_min + np.arange(len(r)) * scan.angle_increment

        def at(deg):
            return r[np.argmin(np.abs(angles - math.radians(deg)))]
        # Spawn at (-6, 0), pointed to +x: side walls at y = +/-6 and a column face 4.1 m in front.
        self.assertAlmostEqual(at(90), 6.0, delta=0.1)
        self.assertAlmostEqual(at(-90), 6.0, delta=0.1)
        self.assertAlmostEqual(at(0), 4.10, delta=0.1)
        # The lowered fork shanks block the scan near +/-61 deg (documented self-occlusion).
        self.assertLess(at(61), 0.3)
        self.assertLess(at(-61), 0.3)

    def test_imu_plausible_at_rest(self):
        msgs = [m for _, m in self.rec.msgs['/imu/data']]
        acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                         m.linear_acceleration.z] for m in msgs])
        gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                         for m in msgs])
        self.assertAlmostEqual(np.linalg.norm(acc.mean(axis=0)), 9.81, delta=0.2)
        self.assertGreater(acc.mean(axis=0)[2], 9.5, 'gravity should point along +z of imu_link')
        self.assertLess(np.abs(gyro.mean(axis=0)).max(), 0.01)
        self.assertGreater(gyro.std(axis=0).min(), 0.0, 'IMU is noise-free (unrealistic)')
        cov = msgs[-1].angular_velocity_covariance
        self.assertGreater(cov[8], 0.0, 'gyro covariance must be set')

    def test_camera_image_and_info(self):
        img, info = self.last('/camera/image_raw'), self.last('/camera/camera_info')
        self.assertEqual((img.width, img.height), (1280, 720))
        self.assertEqual((info.width, info.height), (1280, 720))
        self.assertIn(img.encoding, ('rgb8', 'bgr8'))
        pixels = np.frombuffer(bytes(img.data), dtype=np.uint8).reshape(img.height, img.width, 3)
        self.assertGreater(pixels.std(), 10.0, 'image looks blank/uniform')
        self.assertGreater(pixels.mean(), 20.0, 'image is black (rendering failed?)')
        fx_expected = 640.0 / math.tan(1.2217 / 2)
        self.assertAlmostEqual(info.k[0], fx_expected, delta=1.0)
        self.assertAlmostEqual(info.k[2], 640.0, delta=1.0)
        self.assertAlmostEqual(info.k[5], 360.0, delta=1.0)

    def test_joint_states(self):
        js = self.last('/joint_states')
        self.assertEqual(set(js.name),
                         {'left_drive_wheel_joint', 'right_drive_wheel_joint', 'fork_lift_joint'})
        lift = js.name.index('fork_lift_joint')
        # Spawn at lift.initial_height. The lift controller holds it at startup_height.
        self.assertAlmostEqual(js.position[lift], 0.02, delta=0.005)
        # The lift actuator holds the carriage + forks: (60 + 2*25) kg * g.
        self.assertAlmostEqual(js.effort[lift], 110.0 * 9.81, delta=20.0)

    def test_odometry_at_rest(self):
        for topic in ('/forklift_base_controller/odom', '/odometry/filtered'):
            with self.subTest(topic=topic):
                odom = self.last(topic)
                p = odom.pose.pose.position
                self.assertLess(math.hypot(p.x, p.y), 0.05, 'robot drifted while idle')
                self.assertLess(abs(odom.twist.twist.linear.x), 0.02)

    # ---------------------------------------------------------------- TF

    def test_tf_tree_complete_from_odom(self):
        missing = [f for f in ['base_footprint'] + ROBOT_FRAMES
                   if not self.tf_buffer.can_transform('odom', f, rclpy.time.Time())]
        self.assertEqual(missing, [])

    def test_tf_publishers(self):
        """Only the documented transform sources publish TF (no duplicate edges)."""
        static = {i.node_name for i in self.node.get_publishers_info_by_topic('/tf_static')}
        self.assertEqual(static, {'robot_state_publisher'})
        dyn = {i.node_name for i in self.node.get_publishers_info_by_topic('/tf')}
        # diff_drive_controller always makes a /tf publisher. It must not publish.
        self.assertEqual(dyn, {'robot_state_publisher', 'ekf_filter_node',
                               'forklift_base_controller'})
        client = self.node.create_client(GetParameters,
                                         '/forklift_base_controller/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(GetParameters.Request(names=['enable_odom_tf']))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIs(future.result().values[0].bool_value, False)

    def test_odom_to_base_published_only_by_ekf(self):
        """Each odom->base_footprint transform on /tf has an EKF output stamp."""
        ekf_stamps = {(m.header.stamp.sec, m.header.stamp.nanosec)
                      for _, m in self.rec.msgs['/odometry/filtered']}
        tf_stamps = [(t.header.stamp.sec, t.header.stamp.nanosec)
                     for _, msg in self.rec.msgs['/tf'] for t in msg.transforms
                     if t.child_frame_id == 'base_footprint']
        self.assertGreater(len(tf_stamps), 50)
        parents = {t.header.frame_id for _, msg in self.rec.msgs['/tf'] for t in msg.transforms
                   if t.child_frame_id == 'base_footprint'}
        self.assertEqual(parents, {'odom'})
        foreign = [s for s in tf_stamps if s not in ekf_stamps]
        # The first and the last message can be outside the recording period.
        self.assertLessEqual(len(foreign), 2, f'{len(foreign)} transforms not from the EKF')

    def test_odom_to_base_from_ekf_only(self):
        t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        ekf = self.last('/odometry/filtered').pose.pose.position
        self.assertAlmostEqual(t.transform.translation.x, ekf.x, delta=0.02)
        self.assertAlmostEqual(t.transform.translation.y, ekf.y, delta=0.02)

    def test_ground_truth_is_isolated(self):
        """The ground truth is available for the evaluation, but no autonomy node uses it."""
        subs = {i.node_name for i in
                self.node.get_subscriptions_info_by_topic('/ground_truth/odom')}
        self.assertEqual(subs - {self.node.get_name()}, set())
        frames = self.tf_buffer.all_frames_as_yaml()
        for frame in ('ground_truth_world', 'ground_truth_base'):
            self.assertNotIn(frame, frames, f'{frame} leaked into TF')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited(self, proc_info):
        assert_clean_shutdown(self, proc_info)
