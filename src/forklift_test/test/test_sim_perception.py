"""
Tier-3 simulation test: QR perception on Gazebo camera images.

The robot starts with two decoy props in front of it (QR codes that are not valid). Then
it moves (with its EKF odometry) to a viewing pose in front of each mission box. At each
stop, the published QrDetectionArray must contain the correct results:
  * decoys: decoded but rejected (unknown box / malformed payload), never valid;
  * boxes: the correct box id and destination. The estimated QR pose is transformed into
    the world through TF and the *ground-truth* pose of the robot (evaluation only). It
    must be within 5 cm and 5 deg of the box position in the mission config.
Perception itself never sees the ground truth or the Gazebo entity names.
"""

import math
import unittest

from forklift_interfaces.msg import QrDetectionArray
from forklift_task_manager.mission_config import default_mission_path, load_mission
from forklift_test.helpers import (angle_diff, assert_clean_shutdown, Driver, goto_pose_odom,
                                   spin_for, spin_until, wait_for_controllers, yaw_of)
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
import rclpy
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

SPAWN = (-6.0, -1.8, -math.pi / 2)        # pointed at the decoys (south)
VIEW_Y = -1.8                             # viewing line in front of the pickup row
POS_TOL, YAW_TOL_DEG = 0.05, 5.0


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('forklift_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'x': str(SPAWN[0]), 'y': str(SPAWN[1]), 'yaw': str(SPAWN[2]),
                          'decoys': 'true', 'perception': 'true', 'mode': 'none'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def planar(x, y, yaw, z=0.0):
    t = np.eye(4)
    t[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    t[:3, 3] = [x, y, z]
    return t


class TestSimPerception(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('perception_tester',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.mission = load_mission(default_mission_path())
        cls.latest = {}
        cls.frames = []
        cls.node.create_subscription(Odometry, '/odometry/filtered',
                                     lambda m: cls.latest.__setitem__('ekf', m), 10)
        cls.node.create_subscription(Odometry, '/ground_truth/odom',
                                     lambda m: cls.latest.__setitem__('gt', m), 10)
        cls.node.create_subscription(QrDetectionArray, '/perception/qr_detections',
                                     cls.frames.append, 10)
        cls.tf = Buffer()
        cls.tf_listener = TransformListener(cls.tf, cls.node)
        cls.driver = Driver(cls.node)
        wait_for_controllers(cls.node, ['joint_state_broadcaster', 'forklift_base_controller',
                                        'fork_lift_controller'])
        assert spin_until(cls.node, lambda: 'ekf' in cls.latest and 'gt' in cls.latest, 30.0)
        spin_for(cls.node, 1.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------ helpers

    def collect(self, seconds=3.0):
        """Return the detections from the frames received during the next `seconds` (sim time)."""
        self.frames.clear()
        spin_for(self.node, seconds)
        self.assertGreater(len(self.frames), 0.5 * seconds * 10 * 0.8,
                           'perception published too few frames')
        return [d for f in self.frames for d in f.detections]

    def goto_world(self, x, y, yaw):
        """Move to a world pose. The origin of the odom frame is the spawn pose."""
        sx, sy, syaw = SPAWN
        dx, dy = x - sx, y - sy
        ox = math.cos(-syaw) * dx - math.sin(-syaw) * dy
        oy = math.sin(-syaw) * dx + math.cos(-syaw) * dy
        goto_pose_odom(self.node, self.driver, lambda: self.latest['ekf'],
                       (ox, oy, angle_diff(yaw, syaw)))

    def world_pose_of(self, det):
        """Map a detection pose to the world: TF camera->base, then ground truth (evaluation)."""
        t = self.tf.lookup_transform('base_footprint', det.header.frame_id, rclpy.time.Time())
        base_cam = np.eye(4)
        base_cam[:3, :3] = quat_to_matrix(t.transform.rotation)
        tr = t.transform.translation
        base_cam[:3, 3] = [tr.x, tr.y, tr.z]
        cam_qr = np.eye(4)
        cam_qr[:3, :3] = quat_to_matrix(det.pose.orientation)
        p = det.pose.position
        cam_qr[:3, 3] = [p.x, p.y, p.z]
        gt = self.latest['gt'].pose.pose
        world_base = planar(gt.position.x, gt.position.y, yaw_of(gt.orientation))
        return world_base @ base_cam @ cam_qr

    def expected_qr_world(self, box):
        pose = self.mission.pickups[box.pickup]
        qx, qy, qz = self.mission.load.qr_center
        return planar(pose.x, pose.y, pose.yaw) @ planar(qx, qy, 0.0, qz), pose.yaw

    # ------------------------------------------------------------------ tests

    def test_1_decoys_are_rejected(self):
        dets = self.collect()
        verdicts = {}
        for d in dets:
            verdicts.setdefault(d.payload, set()).add((d.valid, d.rejection_reason.split(':')[0]))
        self.assertIn('BOX_Z:SHELF_01', verdicts, f'seen: {sorted(verdicts)}')
        self.assertIn('PALLET-42', verdicts, f'seen: {sorted(verdicts)}')
        self.assertEqual(verdicts['BOX_Z:SHELF_01'], {(False, 'unknown_box')})
        self.assertEqual(verdicts['PALLET-42'], {(False, 'malformed')})
        self.assertFalse([d for d in dets if d.valid], 'a decoy was accepted as valid')

    def check_box(self, box_id):
        box = self.mission.box(box_id)
        pickup = self.mission.pickups[box.pickup]
        self.goto_world(pickup.x, VIEW_Y, -math.pi / 2)
        dets = [d for d in self.collect() if d.payload == box.payload]
        self.assertTrue(dets, f'{box.payload} not decoded from the camera image')
        self.assertTrue(all(d.valid for d in dets))
        self.assertEqual({(d.box_id, d.destination_id) for d in dets},
                         {(box.box_id, box.destination)})
        self.assertTrue(all(d.pose_valid for d in dets))
        expected, expected_yaw = self.expected_qr_world(box)
        errors, yaw_errors = [], []
        for d in dets:
            est = self.world_pose_of(d)
            errors.append(np.linalg.norm(est[:3, 3] - expected[:3, 3]))
            normal = est[:3, 2]                     # QR z axis = face normal, it must be level
            self.assertLess(abs(normal[2]), math.sin(math.radians(YAW_TOL_DEG)),
                            'estimated face normal is not horizontal')
            yaw_errors.append(abs(math.degrees(angle_diff(math.atan2(normal[1], normal[0]),
                                                          expected_yaw))))
        print(f'{box_id}: {len(dets)} detections, position error median '
              f'{1000 * np.median(errors):.0f} mm / max {1000 * max(errors):.0f} mm, '
              f'yaw error median {np.median(yaw_errors):.1f} deg, '
              f'distance {dets[-1].pose.position.z:.2f} m, side {dets[-1].side_length_px:.0f} px')
        self.assertLess(np.median(errors), POS_TOL)
        self.assertLess(np.median(yaw_errors), YAW_TOL_DEG)

    def test_2_box_a(self):
        self.check_box('BOX_A')

    def test_3_box_b(self):
        self.check_box('BOX_B')

    def test_4_box_c(self):
        self.check_box('BOX_C')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exited(self, proc_info):
        assert_clean_shutdown(self, proc_info)
