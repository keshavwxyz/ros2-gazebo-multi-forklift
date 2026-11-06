"""
Tier-3 simulation test: saved-map localization and Nav2 navigation in Gazebo.

The robot starts at home with the saved map. The goals are the staging poses of the
mission, from warehouse_mission.yaml. A background monitor operates at 5 Hz. It compares
the AMCL estimate with the Gazebo ground truth. It also makes sure that the footprint at
the *true* pose never touches a mapped obstacle, a parked load or the test obstacle. The
test uses the ground truth only for the evaluation. The robot never gets it.

The tests run in sequence on one robot: TF, staging-pose tour, localization stability, no
collisions, loaded footprint, path around a dynamic obstacle, recovery for a goal that
the robot cannot get to.
"""

import collections
import math
import os
import subprocess
import time
import unittest

from action_msgs.msg import GoalStatus
import cv2
from forklift_navigation.footprint import contains, load_footprints
from forklift_task_manager.mission_config import default_mission_path, load_mission
from forklift_test.helpers import (angle_diff, assert_clean_shutdown, spin_for, spin_until,
                                   wait_for_controllers, yaw_of)
from geometry_msgs.msg import PolygonStamped, PoseStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

GOAL_POS_TOL, GOAL_YAW_TOL_DEG = 0.15, 5.0
LOC_POS_TOL, LOC_YAW_TOL_DEG = 0.15, 3.0
OBSTACLE = (-4.6, -1.0, 0.7)      # x, y, edge: crate put on the path home -> PICK_01


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('forklift_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'mode': 'navigation'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def share(pkg, *parts):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(pkg), *parts)


def rectangle_outline(cx, cy, yaw, sx, sy, step=0.05):
    hx, hy = sx / 2, sy / 2
    corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    pts = []
    for i in range(4):
        a, b = np.array(corners[i]), np.array(corners[(i + 1) % 4])
        n = max(2, int(np.linalg.norm(b - a) / step))
        pts.extend(a + (b - a) * t for t in np.linspace(0, 1, n, endpoint=False))
    pts = np.array(pts)
    c, s = math.cos(yaw), math.sin(yaw)
    return pts @ np.array([[c, s], [-s, c]]) + [cx, cy]


class TestSimNavigation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('navigation_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.mission = load_mission(default_mission_path())
        cls.footprints = load_footprints(share('forklift_navigation', 'config',
                                               'footprints.yaml'))
        cls.load_state = False
        cls.latest = {}
        cls.samples = []           # (t, gt(x,y,yaw), amcl(x,y,yaw) or None, collision str)
        cls.obstacle_outline = None
        cls._load_map_obstacles()
        cls.load_outlines = [
            rectangle_outline(p.x, p.y, p.yaw, cls.mission.load.pallet_depth,
                              cls.mission.load.pallet_width)
            for p in cls.mission.pickups.values()]
        # (stamp, pose), approximately 4 s at 50 Hz
        cls.gt_history = collections.deque(maxlen=200)

        def on_gt(m):
            cls.latest['gt'] = m
            p = m.pose.pose
            cls.gt_history.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                   (p.position.x, p.position.y, yaw_of(p.orientation))))
        cls.node.create_subscription(Odometry, '/ground_truth/odom', on_gt, 50)
        cls.node.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                     lambda m: cls.latest.__setitem__('fp', m), 10)
        cls.tf = Buffer()
        cls.tf_listener = TransformListener(cls.tf, cls.node)
        cls.nav = ActionClient(cls.node, NavigateToPose, '/navigate_to_pose')
        cls.node.create_timer(0.2, cls._monitor)
        wait_for_controllers(cls.node, ['joint_state_broadcaster', 'forklift_base_controller',
                                        'fork_lift_controller'])
        assert cls.nav.wait_for_server(timeout_sec=120.0), 'Nav2 did not come up'
        assert spin_until(cls.node, lambda: 'gt' in cls.latest and cls._amcl() is not None, 60)
        spin_for(cls.node, 2.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------ monitoring

    @classmethod
    def _load_map_obstacles(cls):
        meta_path = share('forklift_navigation', 'maps', 'warehouse.yaml')
        meta = yaml.safe_load(open(meta_path, encoding='utf-8'))
        img = cv2.imread(os.path.join(os.path.dirname(meta_path), meta['image']),
                         cv2.IMREAD_GRAYSCALE)
        rows, cols = np.nonzero(img < 50)
        res, (ox, oy, _) = meta['resolution'], meta['origin']
        cls.map_obstacles = np.column_stack([ox + (cols + 0.5) * res,
                                             oy + (img.shape[0] - rows - 0.5) * res])

    @classmethod
    def _amcl(cls, with_stamp=False):
        """Return the latest AMCL estimate (TF map -> base_footprint), and its stamp if set."""
        try:
            t = cls.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return (None, None) if with_stamp else None
        pose = (t.transform.translation.x, t.transform.translation.y,
                yaw_of(t.transform.rotation))
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        return (pose, stamp) if with_stamp else pose

    @classmethod
    def _gt_at(cls, stamp):
        """Ground-truth pose nearest in time to `stamp` (None if more than 30 ms away)."""
        best = min(cls.gt_history, key=lambda m: abs(m[0] - stamp), default=None)
        if best is None or abs(best[0] - stamp) > 0.03:
            return None
        return best[1]

    @classmethod
    def _gt(cls):
        p = cls.latest['gt'].pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    @classmethod
    def _footprint_world(cls, pose):
        poly = cls.footprints['loaded' if cls.load_state else 'unloaded']
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return poly @ np.array([[c, s], [-s, c]]) + pose[:2]

    @classmethod
    def _monitor(cls):
        if 'gt' not in cls.latest:
            return
        gt = cls._gt()
        poly = cls._footprint_world(gt)
        near = cls.map_obstacles[np.linalg.norm(cls.map_obstacles - gt[:2], axis=1) < 2.0]
        hit = ''
        if len(near) and contains(poly, near).any():
            hit = 'mapped obstacle'
        for outline in cls.load_outlines:
            if contains(poly, outline).any():
                hit = 'parked load'
        if cls.obstacle_outline is not None and contains(poly, cls.obstacle_outline).any():
            hit = 'test obstacle'
        # Localization error: the latest AMCL TF compared with the ground truth at the stamp of
        # that TF (the latest TF can be up to one EKF period older than the latest ground truth).
        amcl, stamp = cls._amcl(with_stamp=True)
        gt_then = cls._gt_at(stamp) if amcl is not None else None
        cls.samples.append((time.monotonic(), gt, (amcl, gt_then) if gt_then else None, hit))

    # ------------------------------------------------------------------ helpers

    def navigate(self, x, y, yaw, timeout=240.0):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        feedback = []
        send = self.nav.send_goal_async(goal, feedback_callback=lambda f: feedback.append(
            f.feedback))
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        self.assertTrue(handle is not None and handle.accepted, 'goal rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=timeout)
        self.assertTrue(result.done(), f'navigation to ({x}, {y}) timed out')
        return result.result().status, feedback

    def assert_at(self, pose, name):
        gx, gy, gyaw = self._gt()
        err = math.hypot(gx - pose.x, gy - pose.y)
        yaw_err = abs(math.degrees(angle_diff(gyaw, pose.yaw)))
        print(f'{name}: final error {100 * err:.1f} cm, {yaw_err:.1f} deg')
        self.assertLess(err, GOAL_POS_TOL, f'{name}: {err:.3f} m from goal')
        self.assertLess(yaw_err, GOAL_YAW_TOL_DEG, f'{name}: {yaw_err:.1f} deg from goal')

    def set_loaded(self, loaded):
        client = self.node.create_client(SetBool, '/navigation/set_load_state')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(SetBool.Request(data=loaded))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        type(self).load_state = loaded

    def published_half_width_at(self, x_body):
        """Half-width of the published footprint of the costmap at body x (from its outline)."""
        msg = self.latest['fp']
        pts = np.array([[p.x, p.y] for p in msg.polygon.points])
        # published in the costmap frame (odom): transform it back into base_footprint with TF
        t = self.tf.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time())
        yaw = yaw_of(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        body = pts @ np.array([[c, s], [-s, c]]) + [t.transform.translation.x,
                                                    t.transform.translation.y]
        sel = body[np.abs(body[:, 0] - x_body) < 0.05]
        return float(np.max(np.abs(sel[:, 1]))) if len(sel) else 0.0

    # ------------------------------------------------------------------ tests

    def test_1_tf_tree_and_initial_localization(self):
        for parent, child in (('map', 'odom'), ('odom', 'base_footprint'),
                              ('map', 'base_footprint'), ('base_footprint', 'lidar_link')):
            self.assertTrue(self.tf.can_transform(parent, child, rclpy.time.Time()),
                            f'{parent} -> {child} missing')
        publishers = {i.node_name for i in self.node.get_publishers_info_by_topic('/tf')}
        self.assertIn('amcl', publishers)
        self.assertNotIn('slam_toolbox', publishers, 'two map->odom authorities')
        amcl, gt = self._amcl(), self._gt()
        self.assertLess(math.hypot(amcl[0] - gt[0], amcl[1] - gt[1]), 0.10)

    def test_2_staging_pose_tour(self):
        """Go to each pickup and storage staging pose in the mission sequence."""
        for box in self.mission.boxes:
            for name, pose in ((f'{box.box_id} pickup staging',
                                self.mission.pickup_staging(box.box_id)),
                               (f'{box.destination} staging',
                                self.mission.destination_staging(box.destination))):
                start = time.monotonic()
                status, _ = self.navigate(pose.x, pose.y, pose.yaw)
                print(f'{name}: {time.monotonic() - start:.0f} s wall')
                self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED, name)
                spin_for(self.node, 0.5)
                self.assert_at(pose, name)

    def test_3_localization_stable(self):
        """AMCL follows the ground truth through the full tour, also during the turns."""
        pos_err, yaw_err, where = [], [], []
        for _, _, pair, _ in self.samples:
            if pair is None:
                continue
            amcl, gt = pair
            pos_err.append(math.hypot(amcl[0] - gt[0], amcl[1] - gt[1]))
            yaw_err.append(abs(math.degrees(angle_diff(amcl[2], gt[2]))))
            where.append(gt)
        worst = int(np.argmax(pos_err))
        print(f'localization worst at true pose ({where[worst][0]:.2f}, {where[worst][1]:.2f}, '
              f'{math.degrees(where[worst][2]):.0f} deg)')
        self.assertGreater(len(pos_err), 300, 'too few localization samples')
        print(f'localization: {len(pos_err)} samples, position error median '
              f'{100 * np.median(pos_err):.1f} cm / max {100 * max(pos_err):.1f} cm, yaw '
              f'median {np.median(yaw_err):.2f} deg / max {max(yaw_err):.2f} deg')
        self.assertLess(max(pos_err), LOC_POS_TOL)
        self.assertLess(max(yaw_err), LOC_YAW_TOL_DEG)

    def test_4_no_collisions(self):
        hits = [(round(gt[0], 2), round(gt[1], 2), hit) for _, gt, _, hit in self.samples if hit]
        self.assertEqual(hits, [], f'footprint overlapped obstacles at {hits[:5]}')

    def test_5_loaded_footprint(self):
        """The load switch makes the fork section of the footprint wider. Navigation operates."""
        self.assertAlmostEqual(self.published_half_width_at(1.41), 0.28, delta=0.03)
        self.set_loaded(True)
        self.assertTrue(spin_until(
            self.node, lambda: abs(self.published_half_width_at(1.41) - 0.43) < 0.03, 15.0),
            'costmap footprint did not switch to loaded')
        home = self.mission.home
        status, _ = self.navigate(home.x, home.y, home.yaw)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assert_at(home, 'home (loaded footprint)')
        self.set_loaded(False)
        self.assertTrue(spin_until(
            self.node, lambda: abs(self.published_half_width_at(1.41) - 0.28) < 0.03, 15.0))

    def test_6_dynamic_obstacle_detour(self):
        """The LiDAR sees a crate on the planned path, and the robot goes around it."""
        x, y, edge = OBSTACLE
        sdf = (f'<sdf version="1.10"><model name="test_crate"><static>true</static><link '
               f'name="l"><collision name="c"><geometry><box><size>{edge} {edge} 1.0</size>'
               f'</box></geometry></collision><visual name="v"><geometry><box><size>{edge} '
               f'{edge} 1.0</size></box></geometry></visual></link></model></sdf>')
        subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'warehouse',
                        '-name', 'test_crate', '-string', sdf, '-x', str(x), '-y', str(y),
                        '-z', '0.5'], check=True, timeout=60)
        type(self).obstacle_outline = rectangle_outline(x, y, 0.0, edge, edge)
        target = self.mission.pickup_staging('BOX_A')
        status, _ = self.navigate(target.x, target.y, target.yaw)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assert_at(target, 'BOX_A staging around the crate')
        hits = [s for s in self.samples if s[3] == 'test obstacle']
        self.assertEqual(hits, [], 'robot hit the dropped crate')

    def test_7_recovery_behaviours_on_unreachable_goal(self):
        """A goal in a column has no plan: the BT must try recoveries, then stop the goal."""
        start = len(self.samples)
        status, feedback = self.navigate(1.5, 0.0, 0.0, timeout=300.0)
        recoveries = max((f.number_of_recoveries for f in feedback), default=0)
        print(f'unreachable goal: status {status}, recoveries {recoveries}')
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
        self.assertGreaterEqual(recoveries, 1, 'no recovery behaviour was attempted')
        hits = [s for s in self.samples[start:] if s[3]]
        self.assertEqual(hits, [], 'collision during recovery behaviours')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited(self, proc_info):
        assert_clean_shutdown(self, proc_info)
