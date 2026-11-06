"""
Tier-3 simulation test: conflicts between the two forklifts in Gazebo.

  1 rear-to-rear: the two robots move rearward to each other (teleop through the normal
    command chain). Expected result: the rear LiDAR of each robot sees the other robot,
    and its rear safety stops it. The robots stop with a gap between their rear faces.
  2 head-on in the hall: the test puts the robots on the centerline on opposite sides
    (Nav2 goals from the test). The coordinator sends the two robots home, thus their
    routes meet head-on. Expected result: the coordinator finds a PATH / HEAD_ON conflict.
    One robot yields while the other robot passes. The conflict is RESOLVED. The two
    robots get home, and (ground truth) their bodies never touch.
"""

import math
import unittest

from action_msgs.msg import GoalStatus
from forklift_interfaces.action import RunMission
from forklift_interfaces.msg import ConflictEvent, RobotStatus, SafetyStatus
from forklift_test.helpers import (assert_clean_shutdown, Driver, spin_for, spin_until,
                                   yaw_of)
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

HOME = {'forklift_1': (-6.0, 0.0), 'forklift_2': (5.0, 0.0)}


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('forklift_bringup'), 'launch', 'multi_sim.launch.py'])),
        launch_arguments={'perception': 'false'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


# Robot outline as convex parts in base_footprint: chassis + wheels + rear LiDAR housing
# (x from -1.23 to 0.41, |y| <= 0.61) and the fork tines (x to 1.38, |y| <= 0.25).
PARTS = (((-1.23, -0.61), (0.41, -0.61), (0.41, 0.61), (-1.23, 0.61)),
         ((0.41, -0.25), (1.38, -0.25), (1.38, 0.25), (0.41, 0.25)))


def body(x, y, yaw):
    """Return the convex parts of the robot as lists of corners in the world."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [[(x + c * px - s * py, y + s * px + c * py) for px, py in part] for part in PARTS]


def convex_overlap(a, b):
    """Separating-axis test for two convex quadrilaterals."""
    for poly in (a, b):
        for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
            nx, ny = y2 - y1, x1 - x2
            pa = [nx * px + ny * py for px, py in a]
            pb = [nx * px + ny * py for px, py in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def overlap(a, b):
    return any(convex_overlap(p, q) for p in a for q in b)


class TestFleetConflicts(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fleet_conflict_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.gt, cls.rear, cls.status, cls.conflicts = {}, {}, {}, []
        cls.overlaps = 0
        cls.phase, cls.first_overlap = 'setup', None
        cls.nav, cls.drive = {}, {}
        for rid in HOME:
            cls.node.create_subscription(Odometry, f'/{rid}/ground_truth/odom',
                                         lambda m, r=rid: cls._on_gt(r, m), 10)
            cls.node.create_subscription(SafetyStatus, f'/{rid}/safety/rear_status',
                                         lambda m, r=rid: cls.rear.__setitem__(r, m), 10)
            cls.node.create_subscription(RobotStatus, f'/{rid}/fleet/robot_status',
                                         lambda m, r=rid: cls.status.__setitem__(r, m), 10)
            cls.nav[rid] = ActionClient(cls.node, NavigateToPose, f'/{rid}/navigate_to_pose')
            cls.drive[rid] = Driver(cls.node, f'/{rid}/cmd_vel/teleop')
        cls.node.create_subscription(ConflictEvent, '/fleet/conflicts',
                                     lambda m: cls.conflicts.append(m), 50)
        cls.fleet = ActionClient(cls.node, RunMission, '/fleet/run')
        assert cls.fleet.wait_for_server(timeout_sec=180.0)
        for c in cls.nav.values():
            assert c.wait_for_server(timeout_sec=120.0)
        assert spin_until(cls.node, lambda: len(cls.status) == 2 and all(
            s.localized for s in cls.status.values()), 180)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_gt(cls, rid, m):
        p = m.pose.pose
        cls.gt[rid] = (p.position.x, p.position.y, yaw_of(p.orientation),
                       math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
        if len(cls.gt) == 2 and overlap(body(*cls.gt['forklift_1'][:3]),
                                        body(*cls.gt['forklift_2'][:3])):
            if cls.overlaps == 0:
                cls.first_overlap = (cls.phase, {r: tuple(round(v, 2) for v in g)
                                                 for r, g in cls.gt.items()})
            cls.overlaps += 1

    def goto(self, rid, x, y, yaw):
        goal = NavigateToPose.Goal(pose=PoseStamped())
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = (
            math.sin(yaw / 2), math.cos(yaw / 2))
        for _ in range(5):            # bt_navigator stops the goal until it has the robot pose
            send = self.nav[rid].send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60.0)
            status = GoalStatus.STATUS_UNKNOWN
            if send.done() and send.result() is not None and send.result().accepted:
                res = send.result().get_result_async()
                rclpy.spin_until_future_complete(self.node, res, timeout_sec=300.0)
                status = res.result().status if res.done() else GoalStatus.STATUS_UNKNOWN
            if status == GoalStatus.STATUS_SUCCEEDED:
                return status
            spin_for(self.node, 3.0)
        return status

    def test_1_rear_to_rear_both_rear_lidars_stop_the_robots(self):
        # forklift_1 is on the west position, pointed west. forklift_2 is to the east of it,
        # pointed east. The rear faces (x = -1.23 behind each axle) are 1.14 m apart. No robot must
        # pass the other robot to get there. With y = 2.5, the
        # fork swing (1.44 m) stays clear of the
        # columns at (+/-1.5, 0) when forklift_2 turns, and of the shelves (faces at y = 4.8).
        self.assertEqual(self.goto('forklift_1', -4.6, 2.5, math.pi),
                         GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(self.goto('forklift_2', -1.0, 2.5, 0.0), GoalStatus.STATUS_SUCCEEDED)
        spin_for(self.node, 2.0)
        seen = {r: set() for r in HOME}
        clock = self.node.get_clock()
        t0 = clock.now()
        while (clock.now() - t0).nanoseconds * 1e-9 < 14.0:
            for rid in HOME:                 # the two robots move rearward: to each other
                self.drive[rid].drive(-0.12, 0.0, 0.05)
                if rid in self.rear:
                    seen[rid].add(self.rear[rid].state)
        for rid in HOME:
            self.drive[rid].drive(0.0, 0.0, 0.5)
        spin_for(self.node, 1.0)
        x1, x2 = self.gt['forklift_1'][0], self.gt['forklift_2'][0]
        gap = (x2 - 1.23) - (x1 + 1.23)           # rear face to rear face, along x
        print(f'rear-to-rear: gap {gap:.2f} m; states f1 {sorted(seen["forklift_1"])}, '
              f'f2 {sorted(seen["forklift_2"])}; rear readings '
              f'{self.rear["forklift_1"].nearest_obstacle:.2f} / '
              f'{self.rear["forklift_2"].nearest_obstacle:.2f} m')
        for rid in HOME:
            self.assertIn('STOP', seen[rid], f'{rid} rear safety never stopped it')
            self.assertLess(self.gt[rid][3], 0.02, f'{rid} still moving')
        self.assertGreater(gap, 0.25, 'robots got too close')
        self.assertEqual(self.overlaps, 0)

    def test_2_passing_conflict_is_resolved_and_both_reach_home(self):
        # Head-on: forklift_2 is on the centerline of the hall, to the west of the columns, pointed
        # east. forklift_1 is on the centerline, to the east of the columns, pointed west. The home
        # of each robot is behind the other robot. Thus, their routes are on the same line in
        # opposite directions.
        # Positioning: Nav2 goals from the test, one robot at a time. No robot moves near the fork
        # tines of the other robot (they are below the scan plane and the costmap cannot see them):
        # first, forklift_2 moves straight forward (east), out
        # of the path. Then forklift_1 moves to
        # the east side of the centerline. Then forklift_2 turns and moves to the west side.
        self.phase = 'positioning'
        self.assertEqual(self.goto('forklift_2', 4.0, 2.5, 0.0), GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(self.goto('forklift_1', 3.5, 0.0, math.pi),
                         GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(self.goto('forklift_2', -3.5, 0.0, 0.0), GoalStatus.STATUS_SUCCEEDED)
        spin_for(self.node, 2.0)
        positioning_overlaps, self.overlaps = self.overlaps, 0
        print(f'positioning: body overlap samples {positioning_overlaps}, first '
              f'{self.first_overlap}')
        self.first_overlap = None
        self.phase = 'fleet'
        self.conflicts.clear()
        send = self.fleet.send_goal_async(RunMission.Goal(box_ids=['NONE']))
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600.0)
        result = res.result().result
        for c in self.conflicts:
            print(f'  conflict {c.type} {c.status}: {c.resolution} [{c.resource}] '
                  f'{c.wait_time:.1f} s')
        print(f'passing: {result.message}; body overlap samples {self.overlaps}, first '
              f'{self.first_overlap}')
        self.assertTrue(result.success, result.message)
        path = [c for c in self.conflicts if c.type in ('PATH', 'HEAD_ON', 'DEADLOCK')]
        self.assertTrue(path, 'crossing routes produced no conflict')
        self.assertTrue(any(c.status == 'RESOLVED' for c in path))
        for rid, home in HOME.items():
            self.assertLess(math.dist(self.gt[rid][:2], home), 0.4, f'{rid} not home')
        self.assertEqual(self.overlaps, 0)
        self.assertEqual(positioning_overlaps, 0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assert_clean_shutdown(self, proc_info)
