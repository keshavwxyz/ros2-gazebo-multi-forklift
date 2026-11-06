"""
Tier-3 simulation test: the full autonomous three-box mission.

The robot starts at home (saved-map navigation). A RunMission goal starts the mission
manager. The mission manager uses perception, Nav2 and the load-handling skills. It
delivers each box to the destination from its QR code. Then the robot goes home. During
the first movement, the test pauses the mission (the robot must stop) and then resumes it.

The evaluation uses the Gazebo ground truth (the robot never sees it):
  * each load stops at its destination pose, level on the platform;
  * no load moves faster than physically possible (no teleport);
  * the robot stops at home.
"""

import math
import time
import unittest

from action_msgs.msg import GoalStatus
from forklift_interfaces.action import RunMission
from forklift_interfaces.msg import MissionStatus
from forklift_task_manager.mission_config import default_mission_path, load_mission
from forklift_test.helpers import (angle_diff, assert_clean_shutdown, spin_for, spin_until,
                                   wait_for_controllers, yaw_of)
from geometry_msgs.msg import PoseStamped
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
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

MISSION = load_mission(default_mission_path())
HOME = MISSION.home
MAX_LOAD_SPEED = 0.6          # m/s: robot limit 0.4 + margin; a higher speed = teleport
PLACE_TOL, PLACE_YAW_TOL_DEG, REST_Z_TOL, TILT_TOL_DEG = 0.10, 5.0, 0.01, 3.0
HOME_TOL = 0.30
MISSION_TIMEOUT_WALL = 2400.0


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('forklift_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'x': str(HOME.x), 'y': str(HOME.y), 'yaw': str(HOME.yaw),
                          'mode': 'navigation'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestMission(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mission_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.latest = {}
        cls.load_last = {}
        cls.load_max_speed = {b: 0.0 for b in MISSION.box_ids}
        cls.status_trace = []

        def on_load(box_id):
            def cb(m):
                t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                p = m.pose.position
                prev = cls.load_last.get(box_id)
                if prev is not None and t > prev[0]:
                    v = math.dist(prev[1:4], (p.x, p.y, p.z)) / (t - prev[0])
                    cls.load_max_speed[box_id] = max(cls.load_max_speed[box_id], v)
                cls.load_last[box_id] = (t, p.x, p.y, p.z, m.pose)
            return cb
        for b in MISSION.box_ids:
            cls.node.create_subscription(PoseStamped, f'/ground_truth/loads/load_{b.lower()}',
                                         on_load(b), 50)
        cls.node.create_subscription(Odometry, '/ground_truth/odom',
                                     lambda m: cls.latest.__setitem__('gt', m), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def on_status(m):
            cls.latest['status'] = m
            key = (m.state, m.current_box_id)
            if not cls.status_trace or cls.status_trace[-1] != key:
                cls.status_trace.append(key)
        cls.node.create_subscription(MissionStatus, '/mission/status', on_status, latched)
        cls.client = ActionClient(cls.node, RunMission, '/mission/run')
        wait_for_controllers(cls.node, ['joint_state_broadcaster', 'forklift_base_controller',
                                        'fork_lift_controller'])
        assert cls.client.wait_for_server(timeout_sec=120.0), 'mission manager missing'
        assert spin_until(cls.node, lambda: 'gt' in cls.latest and
                          len(cls.load_last) == len(MISSION.box_ids), 60)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def speed(self):
        v = self.latest['gt'].twist.twist
        return math.hypot(v.linear.x, v.linear.y)

    def call(self, name):
        cli = self.node.create_client(Trigger, name)
        self.assertTrue(cli.wait_for_service(timeout_sec=10.0))
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        return fut.result()

    def test_1_complete_mission(self):
        start = time.monotonic()
        send = self.client.send_goal_async(RunMission.Goal())
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        self.assertTrue(handle is not None and handle.accepted, 'mission goal rejected')
        result = handle.get_result_async()

        # Pause while the robot moves to the first pickup: the robot must stop and stay stopped.
        moving = spin_until(self.node, lambda: (
            self.latest['status'].state == 'NAVIGATE_TO_PICKUP' and self.speed() > 0.15), 300)
        self.assertTrue(moving, 'robot never drove towards the first pickup')
        self.assertTrue(self.call('/mission/pause').success)
        spin_for(self.node, 4.0)
        stopped = []
        for _ in range(20):
            spin_for(self.node, 0.25)
            stopped.append(self.speed())
        print(f'paused: max speed over 5 s after a 4 s stop window {max(stopped):.3f} m/s')
        self.assertEqual(self.latest['status'].state, 'PAUSED')
        self.assertTrue(self.latest['status'].paused)
        self.assertLess(max(stopped), 0.02, 'robot kept moving while paused')
        self.assertTrue(self.call('/mission/resume').success)

        rclpy.spin_until_future_complete(self.node, result, timeout_sec=MISSION_TIMEOUT_WALL)
        self.assertTrue(result.done(), 'mission did not finish in time')
        status, res = result.result().status, result.result().result
        print(f'mission: {res.message}; {time.monotonic() - start:.0f} s wall')
        print('state trace: ' + ' > '.join(f'{s}{"(" + b + ")" if b else ""}'
                                           for s, b in self.status_trace))
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED, res.message)
        self.assertTrue(res.success)
        self.assertEqual(list(res.boxes_delivered), list(MISSION.box_ids))
        self.assertEqual(list(res.boxes_failed), [])
        for b in MISSION.box_ids:
            for state in ('NAVIGATE_TO_PICKUP', 'DETECT_AND_IDENTIFY_BOX', 'VERIFY_PICKUP',
                          'NAVIGATE_TO_DESTINATION', 'VERIFY_PLACEMENT'):
                self.assertIn((state, b), self.status_trace, f'{b} never passed {state}')
        self.assertEqual(self.status_trace[-1][0], 'MISSION_COMPLETE')

    def test_2_every_load_rests_at_its_destination(self):
        spin_for(self.node, 2.0)
        for b in MISSION.box_ids:
            dest_id = MISSION.box(b).destination
            dest = MISSION.destinations[dest_id]
            _, x, y, z, pose = self.load_last[b]
            q = pose.orientation
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
            err = math.hypot(x - dest.pose.x, y - dest.pose.y)
            yaw_err = math.degrees(abs(angle_diff(yaw_of(q), dest.pose.yaw)))
            print(f'ground truth: {b} {100 * err:.1f} cm / {yaw_err:.1f} deg from {dest_id}, '
                  f'z {z:.3f} m, tilt {tilt:.1f} deg; max speed '
                  f'{self.load_max_speed[b]:.2f} m/s')
            self.assertLess(err, PLACE_TOL, f'{b} not at {dest_id}')
            self.assertLess(yaw_err, PLACE_YAW_TOL_DEG)
            self.assertAlmostEqual(z, dest.surface_height, delta=REST_Z_TOL,
                                   msg=f'{b} not resting on the platform')
            self.assertLess(tilt, TILT_TOL_DEG)

    def test_3_no_load_teleported(self):
        for b in MISSION.box_ids:
            self.assertLess(self.load_max_speed[b], MAX_LOAD_SPEED, f'{b} teleported')

    def test_4_robot_returned_home(self):
        p = self.latest['gt'].pose.pose
        err = math.hypot(p.position.x - HOME.x, p.position.y - HOME.y)
        print(f'robot final: {100 * err:.1f} cm from home')
        self.assertLess(err, HOME_TOL)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assert_clean_shutdown(self, proc_info)
