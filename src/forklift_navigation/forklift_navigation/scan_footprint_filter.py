"""
Scan self-filter: make LiDAR returns that hit the robot itself not valid.

The 2D LiDAR sees the lowered fork shanks (and, during transport, possibly the pallet). If
the node did not remove them, these returns would be *in* the robot footprint. SLAM and
AMCL would use them as map structure. The costmaps would use them as obstacles. The
collision monitor of Nav2 would see an immediate collision. This node sets each return in
the current footprint (plus a margin) to NaN ("erroneous measurement", REP-117). Then it
publishes the scan again. It uses /navigation/load_state of the footprint manager and
takes the polygon from the same footprints file. Thus, the loaded/unloaded change also
applies here (unloaded until it gets a different state).
"""

import math
import signal

from forklift_navigation.footprint import load_footprints, self_hit_mask
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ScanFootprintFilter(Node):

    def __init__(self):
        super().__init__('scan_footprint_filter')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_footprint').value
        self.base_frame = p('frame_prefix', '').value + self.base_frame
        self.margin = p('margin', 0.05).value
        self.footprints = load_footprints(p('footprints_file', '').value)
        self.footprint = self.footprints['unloaded']
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.sensor_in_base = None             # (x, y, yaw) of the scan frame
        self.pub = self.create_publisher(LaserScan, p('output_topic', 'scan_filtered').value,
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, p('input_topic', 'scan').value, self._on_scan,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'navigation/load_state', self._on_load_state, latched)

    def _on_load_state(self, msg):
        self.footprint = self.footprints['loaded' if msg.data else 'unloaded']

    def _on_scan(self, msg):
        if self.sensor_in_base is None:
            try:
                t = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except TransformException as exc:
                self.get_logger().warn(f'waiting for TF {self.base_frame}<-'
                                       f'{msg.header.frame_id}: {exc}',
                                       throttle_duration_sec=5.0)
                return
            self.sensor_in_base = (t.transform.translation.x, t.transform.translation.y,
                                   yaw_from_quaternion(t.transform.rotation))
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        ranges[self_hit_mask(ranges, angles, self.sensor_in_base, self.footprint,
                             self.margin)] = np.nan
        out = LaserScan(header=msg.header, angle_min=msg.angle_min, angle_max=msg.angle_max,
                        angle_increment=msg.angle_increment, time_increment=msg.time_increment,
                        scan_time=msg.scan_time, range_min=msg.range_min,
                        range_max=msg.range_max, ranges=ranges.tolist(),
                        intensities=msg.intensities)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFootprintFilter()
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
