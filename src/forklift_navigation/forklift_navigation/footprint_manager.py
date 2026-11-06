"""
Footprint manager: changes the footprint of the local costmap between unloaded and loaded.

Nav2 cannot know that a pallet is on the forks. Thus, the node that changes the load state
(the task manager after a verified pickup or placement) calls /navigation/set_load_state
(std_srvs/SetBool, true = loaded). This node publishes the applicable polygon on the
`footprint` topic of the local costmap. The costmap publishes it again as
published_footprint. The collision monitor and the scan self-filter use that topic. The
node sends the polygon again each second. Thus, a costmap that starts again gets the
correct footprint.
"""

import signal

from forklift_navigation.footprint import load_footprints
from geometry_msgs.msg import Point32, Polygon
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class FootprintManager(Node):

    def __init__(self):
        super().__init__('footprint_manager')
        path = self.declare_parameter('footprints_file', '').value
        topic = self.declare_parameter('footprint_topic', 'local_costmap/footprint').value
        self.footprints = load_footprints(path)
        for required in ('unloaded', 'loaded'):
            if required not in self.footprints:
                raise ValueError(f'{path}: footprint "{required}" missing')
        self.loaded = False
        self.pub = self.create_publisher(Polygon, topic, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, 'navigation/load_state', latched)
        self.create_service(SetBool, 'navigation/set_load_state', self._on_set)
        self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info(f'footprint manager ready ({path})')

    def _publish(self):
        poly = self.footprints['loaded' if self.loaded else 'unloaded']
        self.pub.publish(Polygon(points=[Point32(x=float(x), y=float(y)) for x, y in poly]))
        self.state_pub.publish(Bool(data=self.loaded))

    def _on_set(self, request, response):
        changed = request.data != self.loaded
        self.loaded = request.data
        self._publish()
        state = 'loaded' if self.loaded else 'unloaded'
        response.success = True
        response.message = f'footprint {state}' + ('' if changed else ' (unchanged)')
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FootprintManager()
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
