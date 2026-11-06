"""
Tier-3 simulation test: pickup, transport and placement of a real (physics) load.

The robot starts at the pickup staging pose of BOX_A (saved-map navigation mode). It picks
up BOX_A with the PickupLoad skill. It carries the box with Nav2 to the staging pose of
SHELF_01 and puts it there with PlaceLoad. The test examines the failure handling with:
requests that are not valid, an incorrect destination, a pickup request far from its box
(the skill first moves the robot there), a pickup during transport, and an empty pickup.
(This world cannot give an identity mismatch at a correct pickup. The unit tests of the
skill and of the mission state machine examine that condition.)

The evaluation uses the Gazebo ground truth (robot and load poses, the robot never sees them):
  * no teleport: the load never moves faster than physically possible between two poses (30 Hz);
  * supported: during transport, the load keeps a fixed pose relative to the robot. It is
    level and at the transport height;
  * placed: the load stops at the destination pose, on the platform, and does not move.
"""

import math
import time
import unittest

from action_msgs.msg import GoalStatus
from forklift_interfaces.action import PickupLoad, PlaceLoad
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
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

MISSION = load_mission(default_mission_path())
START = MISSION.pickup_staging('BOX_A')
LOAD = 'load_box_a'
MAX_LOAD_SPEED = 0.6          # m/s: robot limit 0.4 + margin; a higher speed = teleport
CARRY_TOL, CARRY_Z_TOL, TILT_TOL_DEG = 0.02, 0.01, 3.0


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('forklift_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'x': str(START.x), 'y': str(START.y), 'yaw': str(START.yaw),
                          'mode': 'navigation'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def pose_tuple(p):
    q = p.orientation
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
    return p.position.x, p.position.y, p.position.z, yaw_of(q), tilt


class TestPickupPlace(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('pickup_place_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.latest = {}
        cls.load_track = []        # (sim time, x, y, z, yaw, tilt)
        cls.carry_track = []       # load in the robot frame during transport: (dx, dy, z, tilt)
        cls.carrying = False

        def on_load(m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            cls.load_track.append((t, *pose_tuple(m.pose)))
            cls.latest['load'] = cls.load_track[-1]
            if cls.carrying and 'gt' in cls.latest:
                rx, ry, ryaw = cls.robot()
                x, y, z, _, tilt = cls.load_track[-1][1:]
                c, s = math.cos(-ryaw), math.sin(-ryaw)
                cls.carry_track.append((c * (x - rx) - s * (y - ry),
                                        s * (x - rx) + c * (y - ry), z, tilt))
        cls.node.create_subscription(PoseStamped, f'/ground_truth/loads/{LOAD}', on_load, 50)
        cls.node.create_subscription(Odometry, '/ground_truth/odom',
                                     lambda m: cls.latest.__setitem__('gt', m), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(Bool, '/navigation/load_state',
                                     lambda m: cls.latest.__setitem__('loaded', m.data), latched)
        cls.pickup = ActionClient(cls.node, PickupLoad, '/handling/pickup')
        cls.place = ActionClient(cls.node, PlaceLoad, '/handling/place')
        cls.nav = ActionClient(cls.node, NavigateToPose, '/navigate_to_pose')
        wait_for_controllers(cls.node, ['joint_state_broadcaster', 'forklift_base_controller',
                                        'fork_lift_controller'])
        for client in (cls.pickup, cls.place, cls.nav):
            assert client.wait_for_server(timeout_sec=120.0), 'action server missing'
        assert spin_until(cls.node, lambda: 'gt' in cls.latest and 'load' in cls.latest, 60)
        spin_for(cls.node, 3.0)
        cls.initial_load = cls.latest['load']

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def robot(cls):
        p = cls.latest['gt'].pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def run_action(self, client, goal, timeout=300.0):
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        self.assertTrue(handle is not None and handle.accepted, 'goal rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=timeout)
        self.assertTrue(result.done(), 'action timed out')
        return result.result().status, result.result().result

    def navigate(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose.x, pose.y
        goal.pose.pose.orientation.z = math.sin(pose.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(pose.yaw / 2)
        status, _ = self.run_action(self.nav, goal)
        return status

    # ------------------------------------------------------------------ tests

    def test_1_invalid_requests_are_rejected_without_motion(self):
        before = self.robot()
        _, r = self.run_action(self.pickup, PickupLoad.Goal(box_id='BOX_Z'))
        self.assertEqual(r.error_code, PickupLoad.Result.ERROR_INVALID_REQUEST, r.message)
        _, r = self.run_action(self.place, PlaceLoad.Goal(box_id='BOX_A',
                                                          destination_id='SHELF_01'))
        self.assertEqual(r.error_code, PlaceLoad.Result.ERROR_NOT_CARRYING, r.message)
        after = self.robot()
        self.assertLess(math.hypot(after[0] - before[0], after[1] - before[1]), 0.01)

    def test_2_pickup_box_a(self):
        start = time.monotonic()
        status, r = self.run_action(self.pickup, PickupLoad.Goal(box_id='BOX_A'))
        print(f'pickup: {r.message}; alignment {100 * r.lateral_error:.1f} cm / '
              f'{r.heading_error_deg:.1f} deg; {time.monotonic() - start:.0f} s wall')
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED, r.message)
        self.assertTrue(r.success)
        type(self).carrying = True          # monitor the load from here, for all subsequent steps
        self.carry_track.clear()
        self.assertAlmostEqual(r.measured_load_kg, MISSION.load.mass, delta=5.0)
        self.assertLess(abs(r.lateral_error), 0.03)
        spin_for(self.node, 0.5)
        # The load is on the forks: at the transport height and fully in the pallet.
        _, x, y, z, _, tilt = self.latest['load']
        self.assertAlmostEqual(z, 0.18, delta=CARRY_Z_TOL, msg='pallet not at transport height')
        self.assertLess(tilt, TILT_TOL_DEG)
        rx, ry, ryaw = self.robot()
        ahead = (x - rx) * math.cos(ryaw) + (y - ry) * math.sin(ryaw)
        side = -(x - rx) * math.sin(ryaw) + (y - ry) * math.cos(ryaw)
        print(f'load on forks: {ahead:.3f} m ahead, {100 * side:.1f} cm lateral, z {z:.3f} m')
        # Fully on the forks: the near face of the pallet is between the shanks (heel 0.38) and
        # fork length - pallet depth (0.20) farther out -> the
        # load center is 0.78..0.98 m in front.
        self.assertGreaterEqual(ahead, 0.78)
        self.assertLessEqual(ahead, 0.98)
        self.assertLess(abs(side), 0.04)
        self.assertTrue(self.latest.get('loaded'), 'footprint not switched to loaded')

    def test_3_wrong_destination_is_rejected(self):
        before = self.robot()
        _, r = self.run_action(self.place, PlaceLoad.Goal(box_id='BOX_A',
                                                          destination_id='SHELF_02'))
        self.assertEqual(r.error_code, PlaceLoad.Result.ERROR_DESTINATION_MISMATCH, r.message)
        after = self.robot()
        self.assertLess(math.hypot(after[0] - before[0], after[1] - before[1]), 0.01)

    def test_4_transport_keeps_the_load_supported(self):
        """From the pickup end through the transport, the load keeps its pose on the forks."""
        status = self.navigate(MISSION.destination_staging('SHELF_01'))
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertGreater(len(self.carry_track), 300, 'too few load samples while carrying')
        dx = [c[0] for c in self.carry_track]
        dy = [c[1] for c in self.carry_track]
        zs = [c[2] for c in self.carry_track]
        tilts = [c[3] for c in self.carry_track]
        print(f'transport: {len(dx)} samples; load-in-robot drift x '
              f'{100 * (max(dx) - min(dx)):.1f}'
              f' cm, y {100 * (max(dy) - min(dy)):.1f} cm, z {zs[0]:.3f}..{max(zs):.3f} m, '
              f'max tilt {max(tilts):.2f} deg')
        self.assertLess(max(dx) - min(dx), CARRY_TOL, 'load slid along the forks')
        self.assertLess(max(dy) - min(dy), CARRY_TOL, 'load slid sideways')
        self.assertLess(max(zs) - min(zs), CARRY_Z_TOL, 'load height changed while carried')
        self.assertLess(max(tilts), TILT_TOL_DEG, 'load tilted on the forks')

    def test_5_place_at_shelf_01(self):
        start = time.monotonic()
        status, r = self.run_action(self.place, PlaceLoad.Goal(box_id='BOX_A',
                                                               destination_id='SHELF_01'))
        print(f'place: {r.message}; camera-measured error {100 * r.placement_error:.1f} cm / '
              f'{r.placement_yaw_error_deg:.1f} deg; {time.monotonic() - start:.0f} s wall')
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED, r.message)
        type(self).carrying = False
        spin_for(self.node, 1.0)
        dest = MISSION.destinations['SHELF_01']
        _, x, y, z, yaw, tilt = self.latest['load']
        err = math.hypot(x - dest.pose.x, y - dest.pose.y)
        yaw_err = abs(math.degrees(angle_diff(yaw, dest.pose.yaw)))
        print(f'ground truth: load {100 * err:.1f} cm / {yaw_err:.1f} deg from SHELF_01, '
              f'z {z:.3f} m')
        self.assertLess(err, 0.10)
        self.assertLess(yaw_err, 5.0)
        self.assertAlmostEqual(z, dest.surface_height, delta=0.01, msg='not resting on platform')
        self.assertLess(tilt, TILT_TOL_DEG)
        self.assertFalse(self.latest.get('loaded'), 'footprint not switched back to unloaded')

    def test_6_placed_load_stays_put(self):
        before = self.latest['load']
        spin_for(self.node, 3.0)
        after = self.latest['load']
        self.assertLess(math.hypot(after[1] - before[1], after[2] - before[2]), 0.005)
        rx, ry, _ = self.robot()
        self.assertGreater(math.hypot(after[1] - rx, after[2] - ry), 1.5, 'robot still at load')

    def test_7_remote_pickup_navigates_first(self):
        """From SHELF_01, a BOX_B pickup moves to the PICK_02 staging with Nav2, then picks."""
        _, r = self.run_action(self.pickup, PickupLoad.Goal(box_id='BOX_B'), timeout=400.0)
        self.assertTrue(r.success, r.message)
        self.assertAlmostEqual(r.measured_load_kg, MISSION.load.mass, delta=5.0)

    def test_8_pickup_while_carrying_is_rejected_without_motion(self):
        before = self.robot()
        _, r = self.run_action(self.pickup, PickupLoad.Goal(box_id='BOX_C'))
        self.assertEqual(r.error_code, PickupLoad.Result.ERROR_INVALID_REQUEST, r.message)
        self.assertIn('already carrying', r.message)
        after = self.robot()
        self.assertLess(math.hypot(after[0] - before[0], after[1] - before[1]), 0.01)

    def test_8b_place_box_b_at_its_destination(self):
        _, r = self.run_action(self.place, PlaceLoad.Goal(box_id='BOX_B',
                                                          destination_id='SHELF_03'),
                               timeout=400.0)
        self.assertTrue(r.success, r.message)

    def test_8c_empty_pickup_reports_box_not_visible(self):
        self.assertEqual(self.navigate(MISSION.pickup_staging('BOX_A')),
                         GoalStatus.STATUS_SUCCEEDED)
        _, r = self.run_action(self.pickup, PickupLoad.Goal(box_id='BOX_A'))
        self.assertEqual(r.error_code, PickupLoad.Result.ERROR_BOX_NOT_VISIBLE, r.message)

    def test_9_load_never_teleported(self):
        """Two sequential ground-truth load poses give a physically possible speed."""
        track = self.load_track
        self.assertGreater(len(track), 1000)
        worst = 0.0
        for (t0, x0, y0, z0, *_), (t1, x1, y1, z1, *_) in zip(track, track[1:]):
            dt = t1 - t0
            if dt <= 0:
                continue
            step = math.dist((x0, y0, z0), (x1, y1, z1))
            worst = max(worst, step / dt if step > 0.002 else 0.0)
            self.assertLess(step, MAX_LOAD_SPEED * dt + 0.005,
                            f'load jumped {step:.3f} m in {dt:.3f} s at t={t1:.2f}')
        print(f'no teleport: {len(track)} load poses, max speed {worst:.2f} m/s')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited(self, proc_info):
        assert_clean_shutdown(self, proc_info)
