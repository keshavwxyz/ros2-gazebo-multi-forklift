"""
Integration test: the qr_detector node on synthetic camera images (no simulator).

The test publishes CameraInfo and synthetic images through ROS. Then it examines the
published QrDetectionArray:
  - The node identifies valid boxes with an accurate pose.
  - The node reports malformed and unknown payloads as not valid, with a reason.
  - An empty scene gives an empty detection array (but it is present), with the image stamp.
"""

import math
import time
import unittest

from cv_bridge import CvBridge
from forklift_interfaces.msg import QrDetectionArray
from forklift_perception import synthetic_scene as scene
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

W, H, HFOV = 1280, 720, 1.2217
K = scene.camera_matrix(W, H, HFOV)


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='forklift_perception', executable='qr_detector', output='screen',
        parameters=[{'max_rate': 0.0}])          # process each frame in the test
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


def view(distance, lateral=0.0, yaw_deg=0.0):
    pitch, yaw = math.radians(40), math.radians(yaw_deg)
    normal = np.array([math.sin(yaw) * math.cos(pitch), -math.sin(pitch),
                       -math.cos(pitch) * math.cos(yaw)])
    return scene.look_at_rotation(normal), np.array([lateral, 0.0, distance])


class TestQrDetectorNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('qr_node_tester')
        cls.bridge = CvBridge()
        cls.results = {}
        cls.node.create_subscription(
            QrDetectionArray, '/perception/qr_detections',
            lambda m: cls.results.__setitem__(m.header.stamp.sec, m), 10)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        cls.img_pub = cls.node.create_publisher(Image, '/camera/image_raw', reliable)
        cls.info_pub = cls.node.create_publisher(CameraInfo, '/camera/camera_info', 1)
        cls.seq = 100

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def run_frame(self, labels, timeout=15.0):
        """Publish one frame (with camera_info) and return the detections for it."""
        type(self).seq += 1
        stamp_sec = self.seq
        info = CameraInfo(width=W, height=H, k=K.reshape(-1).tolist(),
                          p=np.hstack([K, np.zeros((3, 1))]).reshape(-1).tolist(),
                          d=[0.0] * 5, distortion_model='plumb_bob')
        info.header.frame_id = 'camera_optical_frame'
        msg = self.bridge.cv2_to_imgmsg(scene.render(labels, K, seed=stamp_sec), 'mono8')
        msg.header.frame_id = 'camera_optical_frame'
        msg.header.stamp.sec = stamp_sec
        info.header.stamp = msg.header.stamp
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and stamp_sec not in self.results:
            # Publish again until the node has matched the
            # publishers of the test and sent a response.
            self.info_pub.publish(info)
            self.img_pub.publish(msg)
            end = time.monotonic() + 0.5
            while time.monotonic() < end and stamp_sec not in self.results:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIn(stamp_sec, self.results, 'no QrDetectionArray for the frame')
        return self.results[stamp_sec]

    def test_valid_box_identified_with_pose(self):
        rot, t = view(2.4, lateral=0.2, yaw_deg=15)
        out = self.run_frame([('BOX_B:SHELF_03', rot, t)])
        self.assertEqual(out.header.frame_id, 'camera_optical_frame')
        self.assertEqual(len(out.detections), 1)
        d = out.detections[0]
        self.assertTrue(d.valid, d.rejection_reason)
        self.assertEqual((d.box_id, d.destination_id), ('BOX_B', 'SHELF_03'))
        self.assertEqual(d.header.stamp, out.header.stamp)
        self.assertTrue(d.pose_valid)
        p = d.pose.position
        self.assertLess(np.linalg.norm(np.array([p.x, p.y, p.z]) - t), 0.04)
        q = d.pose.orientation
        self.assertAlmostEqual(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2, 1.0, places=6)
        self.assertGreater(d.confidence, 0.5)
        self.assertGreater(d.side_length_px, 40)

    def test_all_three_boxes_in_one_frame(self):
        labels = [(p, *view(2.7, lateral=x)) for p, x in (
            ('BOX_A:SHELF_01', -0.8), ('BOX_B:SHELF_03', 0.0), ('BOX_C:SHELF_02', 0.8))]
        out = self.run_frame(labels)
        ids = sorted(d.box_id for d in out.detections if d.valid)
        self.assertEqual(ids, ['BOX_A', 'BOX_B', 'BOX_C'])

    def test_malformed_and_unknown_payloads_rejected(self):
        labels = [('hello world', *view(2.4, lateral=-0.5)),
                  ('BOX_Z:SHELF_01', *view(2.4, lateral=0.5))]
        out = self.run_frame(labels)
        by_payload = {d.payload: d for d in out.detections}
        self.assertEqual(set(by_payload), {'hello world', 'BOX_Z:SHELF_01'})
        for d in by_payload.values():
            self.assertFalse(d.valid)
            self.assertTrue(d.rejection_reason)
        self.assertEqual(by_payload['hello world'].box_id, '')       # the parse is not possible
        self.assertEqual(by_payload['BOX_Z:SHELF_01'].box_id, 'BOX_Z')
        self.assertTrue(by_payload['hello world'].rejection_reason.startswith(
            'invalid_characters'))
        self.assertTrue(by_payload['BOX_Z:SHELF_01'].rejection_reason.startswith(
            'unknown_box'))

    def test_empty_scene_publishes_empty_array(self):
        out = self.run_frame([])
        self.assertEqual(list(out.detections), [])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
