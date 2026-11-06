"""
Tier-3 simulation test: rear LiDAR and reverse-motion safety (one forklift, no Nav2).

The robot stands in the open hall at (-3, 0.8), pointed to +x. The test spawns and removes
a box behind it (test scenery, not part of the autonomy). Expected results:
  * /scan_rear: LaserScan, frame rear_lidar_link, 8 Hz or more (sim time), the TF mount is
    behind the robot and points rearward; /scan (front) has no change.
  * clear behind -> reverse motion is permitted;
  * box 1.0 m behind the rear face -> reverse motion becomes slower and stops outside the
    0.35 m stop zone (STOP is reported with a reason); forward motion is still permitted;
  * box removed -> reverse motion is permitted again.
The ground truth (evaluation only) measures the real gap and motion.
"""

import math
import os
import subprocess
import time
import unittest

from forklift_interfaces.msg import SafetyStatus
from forklift_test.helpers import (assert_clean_shutdown, Driver, spin_for, spin_until,
                                   TopicRecorder, wait_for_controllers, yaw_of)
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
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

START = (-3.0, 0.8, 0.0)
REAR_FACE = -1.18                 # rear face of the footprint, base_footprint x
STOP_DISTANCE = 0.35
BOX = 'rear_obstacle'


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('forklift_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'x': str(START[0]), 'y': str(START[1]), 'yaw': str(START[2]),
                          'mode': 'none', 'perception': 'false',
                          'spawn_loads': 'false'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def gz(*args, timeout=10):
    return subprocess.run(['gz', *args], capture_output=True, text=True, timeout=timeout,
                          env=dict(os.environ))


def spawn_box(x, y):
    sdf = (f'<sdf version="1.9"><model name="{BOX}"><static>true</static><link name="l">'
           '<collision name="c"><geometry><box><size>0.4 0.8 0.6</size></box></geometry>'
           '</collision><visual name="v"><geometry><box><size>0.4 0.8 0.6</size></box>'
           '</geometry></visual></link></model></sdf>')
    req = f"sdf: '{sdf}' pose: {{position: {{x: {x}, y: {y}, z: 0.3}}}}"
    r = gz('service', '-s', '/world/warehouse/create', '--reqtype', 'gz.msgs.EntityFactory',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', req)
    return 'data: true' in r.stdout


def remove_box():
    r = gz('service', '-s', '/world/warehouse/remove', '--reqtype', 'gz.msgs.Entity',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
           '--req', f'name: "{BOX}" type: MODEL')
    return 'data: true' in r.stdout


class TestRearSafety(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('rear_safety_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.latest = {}
        cls.node.create_subscription(Odometry, '/ground_truth/odom',
                                     lambda m: cls.latest.__setitem__('gt', m), 10)
        cls.node.create_subscription(SafetyStatus, '/safety/rear_status',
                                     lambda m: cls.latest.__setitem__('rear', m), 10)
        cls.tf = Buffer()
        cls.tf_listener = TransformListener(cls.tf, cls.node)
        cls.driver = Driver(cls.node)
        wait_for_controllers(cls.node, ['joint_state_broadcaster', 'forklift_base_controller',
                                        'fork_lift_controller'])
        assert spin_until(cls.node, lambda: 'gt' in cls.latest and 'rear' in cls.latest, 60)
        spin_for(cls.node, 3.0)

    @classmethod
    def tearDownClass(cls):
        remove_box()
        cls.node.destroy_node()
        rclpy.shutdown()

    def robot(self):
        p = self.latest['gt'].pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def gap_behind(self, box_x):
        """Ground-truth distance from the rear face of the footprint to the box front face."""
        rx, _, _ = self.robot()
        return (rx + REAR_FACE) - (box_x + 0.2)

    def reverse(self, seconds, v=-0.2):
        start = self.robot()
        states = []
        clock = self.node.get_clock()
        t0 = clock.now()
        while (clock.now() - t0).nanoseconds * 1e-9 < seconds:
            self.driver.drive(v, 0.0, 0.2)
            states.append(self.latest['rear'].state)
        self.driver.drive(0.0, 0.0, 1.0)
        end = self.robot()
        return math.hypot(end[0] - start[0], end[1] - start[1]), states

    def test_1_rear_scan_is_valid(self):
        rec = TopicRecorder(self.node, {'/scan_rear': LaserScan, '/scan': LaserScan})
        spin_for(self.node, 5.0)
        rear = [m for _, m in rec.msgs['/scan_rear']]
        self.assertGreater(len(rear), 5, 'no rear scans')
        msg = rear[-1]
        self.assertEqual(msg.header.frame_id, 'rear_lidar_link')
        self.assertEqual(len(msg.ranges), 181)
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in rear]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        print(f'rear scan: {rate:.1f} Hz (sim time), {len(msg.ranges)} rays, '
              f'frame {msg.header.frame_id}')
        self.assertGreater(rate, 8.0)
        front = [m for _, m in rec.msgs['/scan']]
        t = self.tf.lookup_transform('base_footprint', 'rear_lidar_link', Time())
        tr = t.transform.translation
        self.assertAlmostEqual(tr.x, -1.19, delta=1e-3)
        self.assertAlmostEqual(tr.z, 0.12, delta=1e-3)
        self.assertAlmostEqual(abs(yaw_of(t.transform.rotation)), math.pi, delta=1e-3)
        self.assertGreater(len(front), 5, 'front LiDAR stopped')
        self.assertEqual(front[-1].header.frame_id, 'lidar_link')

    def test_2_reverse_allowed_when_clear(self):
        self.assertIn(self.latest['rear'].state, ('CLEAR', 'WARN'))
        moved, states = self.reverse(2.5)
        print(f'clear: reversed {moved:.2f} m, states {sorted(set(states))}')
        self.assertGreater(moved, 0.3)
        self.assertNotIn('STOP', states)

    def test_3_obstacle_appears_and_reverse_stops_outside_the_stop_zone(self):
        rx, ry, _ = self.robot()
        box_x = rx + REAR_FACE - 1.0 - 0.2           # front face 1.0 m behind the rear face
        self.assertTrue(spawn_box(box_x, ry), 'could not spawn the obstacle')
        type(self).box_x = box_x
        self.assertTrue(spin_until(self.node, lambda: (
            self.latest['rear'].nearest_obstacle < 1.2), 15.0), 'obstacle not seen')
        seen = self.latest['rear'].nearest_obstacle
        print(f'obstacle: rear LiDAR {seen:.2f} m, ground truth {self.gap_behind(box_x):.2f} m')
        self.assertAlmostEqual(seen, self.gap_behind(box_x), delta=0.08)
        moved, states = self.reverse(12.0)
        gap = self.gap_behind(box_x)
        rear = self.latest['rear']
        print(f'blocked: reversed {moved:.2f} m, final gap {gap:.2f} m, state {rear.state} '
              f'({rear.reason}); interventions {rear.interventions}')
        self.assertIn('STOP', states)
        self.assertGreater(gap, STOP_DISTANCE - 0.05, 'robot entered the stop zone')
        self.assertLess(gap, 0.75, 'stopped far too early')
        self.assertEqual(rear.state, 'STOP')
        self.assertIn('behind', rear.reason)
        v = self.latest['gt'].twist.twist.linear
        self.assertLess(math.hypot(v.x, v.y), 0.02)

    def test_4_forward_still_allowed_with_obstacle_behind(self):
        start = self.robot()
        self.driver.drive(0.2, 0.0, 2.0)
        self.driver.drive(0.0, 0.0, 1.0)
        end = self.robot()
        self.assertGreater(end[0] - start[0], 0.2)

    def test_5_reverse_resumes_after_the_obstacle_is_removed(self):
        self.assertTrue(remove_box())
        self.assertTrue(spin_until(self.node, lambda: (
            self.latest['rear'].nearest_obstacle > 1.5), 15.0))
        moved, states = self.reverse(2.0)
        print(f'after removal: reversed {moved:.2f} m, states {sorted(set(states))}')
        self.assertGreater(moved, 0.2)
        time.sleep(0.1)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assert_clean_shutdown(self, proc_info)
