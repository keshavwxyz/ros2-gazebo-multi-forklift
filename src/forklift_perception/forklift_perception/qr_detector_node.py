"""
QR perception node.

The node subscribes to the camera image and the camera info. It does these steps:
  1. It finds and decodes QR codes from the pixels only.
  2. It validates each payload against the mission configuration.
  3. It estimates the pose of the code with planar PnP.
  4. It publishes forklift_interfaces/QrDetectionArray for each processed frame.
An empty array means that no code is visible. Thus, the subscribers can tell "no
detection" from "no data". The node also publishes codes that are not valid or that it
cannot decode, with valid=false and a reason. Thus, the task manager can respond to them.

The node never reads the simulator state: no entity names, no ground truth.
"""

import signal

import cv2
from cv_bridge import CvBridge, CvBridgeError
from forklift_interfaces.msg import QrDetection, QrDetectionArray
from forklift_perception.pose_estimation import confidence, estimate_qr_pose
from forklift_perception.qr_detection import QrDetector
from forklift_task_manager.mission_config import default_mission_path, load_mission
from geometry_msgs.msg import Point, Pose, Quaternion
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class QrDetectorNode(Node):

    def __init__(self):
        super().__init__('qr_detector')
        p = self.declare_parameter
        mission_file = p('mission_file', '').value or default_mission_path()
        image_topic = p('image_topic', 'camera/image_raw').value
        info_topic = p('camera_info_topic', 'camera/camera_info').value
        self.max_rate = p('max_rate', 10.0).value
        self.publish_debug = p('publish_debug_image', False).value
        min_side_px = p('min_side_px', 12.0).value

        self.mission = load_mission(mission_file)
        self.edge = self.mission.qr_edge_length
        self.detector = QrDetector(min_side_px=min_side_px)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_processed = None
        self.rejections_logged = set()

        # Large images: RELIABLE + depth 1 (best effort discards fragmented frames, see
        # docs/sensor_setup.md). With depth 1, the node always uses the newest frame.
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, info_topic, self._on_info, 1)
        self.create_subscription(Image, image_topic, self._on_image, image_qos)
        self.pub = self.create_publisher(QrDetectionArray, 'perception/qr_detections', 10)
        self.debug_pub = self.create_publisher(Image, 'perception/qr_debug', 1)
        self.get_logger().info(
            f'qr_detector ready: {len(self.mission.boxes)} boxes from {mission_file}, '
            f'QR edge {self.edge:.3f} m')

    def _on_info(self, msg):
        k = np.array(msg.k, np.float64).reshape(3, 3)
        if k[0, 0] <= 0 or k[1, 1] <= 0:
            self.get_logger().error('camera_info has an invalid camera matrix',
                                    throttle_duration_sec=10.0)
            return
        self.camera_matrix, self.dist_coeffs = k, np.array(msg.d, np.float64)

    def _on_image(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('waiting for camera_info', throttle_duration_sec=5.0)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_processed is not None and self.max_rate > 0 and \
                stamp - self.last_processed < 0.95 / self.max_rate:
            return
        self.last_processed = stamp
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cannot convert image: {exc}', throttle_duration_sec=5.0)
            return

        out = QrDetectionArray(header=msg.header)
        for cand in self.detector.detect(gray):
            out.detections.append(self._make_detection(msg.header, cand))
        self.pub.publish(out)
        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(msg.header, gray, out.detections)

    def _make_detection(self, header, cand):
        det = QrDetection(header=header, payload=cand.payload)
        det.corners_px = [float(v) for v in cand.corners.reshape(-1)]
        det.side_length_px = cand.side_length_px
        result = self.mission.validate_payload(cand.payload)
        det.valid, det.box_id, det.destination_id = (result.valid, result.box_id,
                                                     result.destination_id)
        det.rejection_reason = '' if result.valid else f'{result.status}: {result.reason}'
        pose = estimate_qr_pose(cand.corners, self.edge, self.camera_matrix, self.dist_coeffs)
        if pose is not None:
            det.pose_valid = True
            x, y, z, w = pose.quaternion_xyzw()
            det.pose = Pose(position=Point(x=float(pose.tvec[0]), y=float(pose.tvec[1]),
                                           z=float(pose.tvec[2])),
                            orientation=Quaternion(x=x, y=y, z=z, w=w))
            det.confidence = confidence(cand.side_length_px, pose.reprojection_error_px,
                                        bool(cand.payload))
        if not result.valid and cand.payload not in self.rejections_logged:
            self.rejections_logged.add(cand.payload)
            self.get_logger().warn(f'rejected QR {cand.payload!r}: {det.rejection_reason}')
        return det

    def _publish_debug(self, header, gray, detections):
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for d in detections:
            pts = np.array(d.corners_px, np.float32).reshape(4, 2).astype(np.int32)
            color = (0, 200, 0) if d.valid else (0, 0, 255)
            cv2.polylines(img, [pts], True, color, 3)
            label = d.payload or '<undecoded>'
            if d.pose_valid:
                label += f'  {d.pose.position.z:.2f} m'
            cv2.putText(img, label, tuple(int(v) for v in pts[0] - [0, 10]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8', header=header))


def main(args=None):
    rclpy.init(args=args)
    node = QrDetectorNode()
    try:
        # Some messages cause an rclpy error. This occurred in the two-robot graph:
        # 'Unable to convert call argument'. Discard the message and continue.
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except RuntimeError as exc:
                if not rclpy.ok():
                    break                   # the node stops
                node.get_logger().error(f'dropped a message that could not be taken: {exc}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # During a SIGINT shutdown, rclpy can raise an exception from a context that is partly
        # removed (for example, when it takes a message). Only exceptions that occur while ROS
        # runs are real failures.
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl-C must not stop the cleanup
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
