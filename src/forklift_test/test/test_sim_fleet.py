"""
Tier-3 simulation test: two forklifts and the fleet coordinator in one Gazebo world.

Expected results:
  1 isolation: each robot has its own controllers, and rear and front scans with frame ids
    that have the prefix. TF map -> forklift_N/base_footprint is available for the two
    robots at their home poses (independent localization). Heartbeats come from the two robots.
  2 the fleet mission delivers all three boxes to the destinations on their QR codes. A box
    never has two owners. The two robots stop at home. Ground truth (evaluation only): each
    load is at its destination. No load moved faster than physically possible. The bodies of
    the two robots never touched.
"""

import math
import time
import unittest

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from forklift_interfaces.action import RunMission
from forklift_interfaces.msg import ConflictEvent, FleetStatus, RobotStatus
from forklift_task_manager.mission_config import default_mission_path, load_mission
from forklift_test.helpers import (angle_diff, assert_clean_shutdown, spin_for, spin_until,
                                   yaw_of)
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
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

MISSION = load_mission(default_mission_path())
ROBOTS = {'forklift_1': (-6.0, 0.0), 'forklift_2': (5.0, 0.0)}
HOME_YAW = {'forklift_1': 0.0, 'forklift_2': math.pi}
MAX_LOAD_SPEED = 0.6


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('forklift_bringup'), 'launch', 'multi_sim.launch.py'])))
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


# Robot outline as convex parts in base_footprint: chassis + wheels + rear LiDAR housing
# (x from -1.23 to 0.41, |y| <= 0.61) and the fork tines (x to 1.38, |y| <= 0.25).
PARTS = (((-1.23, -0.61), (0.41, -0.61), (0.41, 0.61), (-1.23, 0.61)),
         ((0.41, -0.25), (1.38, -0.25), (1.38, 0.25), (0.41, 0.25)))


def body_corners(x, y, yaw):
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


class TestFleet(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fleet_sim_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.gt, cls.status, cls.robot_status = {}, None, {}
        cls.owners = {}                    # box -> set of all owners that the test saw
        cls.conflicts = []
        cls.load_last, cls.load_vmax = {}, {b: 0.0 for b in MISSION.box_ids}
        cls.robot_overlap = 0
        for rid in ROBOTS:
            cls.node.create_subscription(Odometry, f'/{rid}/ground_truth/odom',
                                         lambda m, r=rid: cls._on_gt(r, m), 10)
            cls.node.create_subscription(RobotStatus, f'/{rid}/fleet/robot_status',
                                         lambda m, r=rid: cls.robot_status.__setitem__(r, m),
                                         10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(FleetStatus, '/fleet/status', cls._on_status, latched)
        cls.node.create_subscription(ConflictEvent, '/fleet/conflicts',
                                     lambda m: cls.conflicts.append(m), 50)
        for b in MISSION.box_ids:
            cls.node.create_subscription(PoseStamped, f'/ground_truth/loads/load_{b.lower()}',
                                         lambda m, b=b: cls._on_load(b, m), 50)
        cls.tf = Buffer()
        cls.tf_listener = TransformListener(cls.tf, cls.node)
        cls.fleet = ActionClient(cls.node, RunMission, '/fleet/run')
        assert cls.fleet.wait_for_server(timeout_sec=180.0), 'fleet coordinator missing'
        assert spin_until(cls.node, lambda: len(cls.robot_status) == 2 and all(
            s.localized for s in cls.robot_status.values()), 180), 'robots not localized'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_gt(cls, rid, m):
        p = m.pose.pose
        cls.gt[rid] = (p.position.x, p.position.y, yaw_of(p.orientation))
        if len(cls.gt) == 2 and overlap(body_corners(*cls.gt['forklift_1']),
                                        body_corners(*cls.gt['forklift_2'])):
            cls.robot_overlap += 1

    @classmethod
    def _on_status(cls, m):
        cls.status = m
        for t in m.tasks:
            if t.owner:
                cls.owners.setdefault(t.box_id, set()).add((t.owner, t.assignment_id))

    @classmethod
    def _on_load(cls, b, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.position
        prev = cls.load_last.get(b)
        if prev is not None and t > prev[0]:
            cls.load_vmax[b] = max(cls.load_vmax[b],
                                   math.dist(prev[1:4], (p.x, p.y, p.z)) / (t - prev[0]))
        cls.load_last[b] = (t, p.x, p.y, p.z, m.pose)

    def test_1_namespaces_tf_and_sensors_are_isolated(self):
        for rid, home in ROBOTS.items():
            cli = self.node.create_client(ListControllers, f'/{rid}/controller_manager/'
                                                           'list_controllers')
            self.assertTrue(cli.wait_for_service(timeout_sec=30.0))
            expected = {'joint_state_broadcaster', 'forklift_base_controller',
                        'fork_lift_controller'}
            # When the two robots are localized, the spawners of the
            # second robot can still be in progress.
            deadline = time.monotonic() + 60.0
            while True:
                fut = cli.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
                active = {c.name for c in fut.result().controller if c.state == 'active'} \
                    if fut.result() is not None else set()
                if active == expected or time.monotonic() > deadline:
                    break
                spin_for(self.node, 2.0)
            self.assertEqual(active, expected, rid)
            for topic, frame in (('scan', 'lidar_link'), ('scan_rear', 'rear_lidar_link')):
                got = []
                sub = self.node.create_subscription(LaserScan, f'/{rid}/{topic}',
                                                    lambda m: got.append(m),
                                                    qos_profile_sensor_data)
                self.assertTrue(spin_until(self.node, lambda: got, 15.0), f'{rid}/{topic}')
                self.assertEqual(got[0].header.frame_id, f'{rid}/{frame}')
                self.node.destroy_subscription(sub)
            t = self.tf.lookup_transform('map', f'{rid}/base_footprint', Time())
            est = (t.transform.translation.x, t.transform.translation.y)
            print(f'{rid}: localized at ({est[0]:.2f}, {est[1]:.2f}), home {home}')
            self.assertLess(math.dist(est, home), 0.3)
            self.assertTrue(self.tf.can_transform(f'{rid}/base_footprint',
                                                  f'{rid}/rear_lidar_link', Time()))

    def test_2_fleet_mission_delivers_all_boxes(self):
        start = time.monotonic()
        send = self.fleet.send_goal_async(RunMission.Goal())
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        self.assertTrue(handle is not None and handle.accepted)
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=3000.0)
        self.assertTrue(res.done(), 'fleet mission did not finish')
        status, result = res.result().status, res.result().result
        print(f'fleet: {result.message}; {time.monotonic() - start:.0f} s wall')
        for c in self.conflicts:
            print(f'  conflict {c.type} {c.status}: {c.resolution} [{c.resource}] '
                  f'{c.wait_time:.1f} s')
        print('  owners: ' + '; '.join(f'{b}: {sorted(o)}' for b, o in self.owners.items()))
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED, result.message)
        self.assertEqual(sorted(result.boxes_delivered), sorted(MISSION.box_ids))
        for t in self.status.tasks:
            self.assertEqual(t.state, 'DELIVERED')
            self.assertEqual(t.qr_destination_id, t.destination_id)
        spin_for(self.node, 2.0)
        for rid, home in ROBOTS.items():
            x, y, yaw = self.gt[rid]
            est = self.robot_status[rid]
            print(f'{rid}: final ground truth ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg): '
                  f'{100 * math.dist((x, y), home):.1f} cm / '
                  f'{math.degrees(abs(angle_diff(yaw, HOME_YAW[rid]))):.1f} deg from home; '
                  f'own estimate ({est.x:.3f}, {est.y:.3f}, {math.degrees(est.yaw):.1f} deg), '
                  f'estimate error {100 * math.dist((x, y), (est.x, est.y)):.1f} cm / '
                  f'{math.degrees(abs(angle_diff(yaw, est.yaw))):.1f} deg')
            self.assertLess(math.dist(self.gt[rid][:2], home), 0.4, f'{rid} not home')

    def test_3_loads_rest_at_their_destinations_without_teleport(self):
        for b in MISSION.box_ids:
            d = MISSION.destinations[MISSION.box(b).destination]
            _, x, y, z, pose = self.load_last[b]
            err = math.hypot(x - d.pose.x, y - d.pose.y)
            yaw_err = math.degrees(abs(angle_diff(yaw_of(pose.orientation), d.pose.yaw)))
            print(f'ground truth: {b} {100 * err:.1f} cm / {yaw_err:.1f} deg from '
                  f'{MISSION.box(b).destination}, z {z:.3f}, max speed '
                  f'{self.load_vmax[b]:.2f} m/s')
            self.assertLess(err, 0.10)
            self.assertAlmostEqual(z, d.surface_height, delta=0.01)
            self.assertLess(self.load_vmax[b], MAX_LOAD_SPEED)

    def test_4_robots_never_overlapped(self):
        print(f'robot body overlap samples: {self.robot_overlap}')
        self.assertEqual(self.robot_overlap, 0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assert_clean_shutdown(self, proc_info)
