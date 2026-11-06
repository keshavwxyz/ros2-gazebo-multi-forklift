"""
Rear safety filter: the last stage before the drive controller.

  cmd_vel_in  (twist_mux output: navigation, docking skills, teleop)
      -> rear_safety (rear LiDAR) ->  cmd_vel_out (forklift_base_controller/cmd_vel)

If the rear LiDAR sees an obstacle in the corridor behind the robot, the filter decreases
or stops the reverse motion (rear_safety_logic.py). The filter does not change forward
motion. The e-stop lock stays in twist_mux, before this filter. The filter publishes the
status and the reason for each intervention on safety/rear_status. The load handler, the
mission manager and the fleet coordinator use this status. Thus, they can identify
"blocked behind" from other failures.
"""

import math
import signal

from forklift_control.rear_safety_logic import (decide, RearSafetyConfig, scan_points)
from forklift_interfaces.msg import SafetyStatus
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RearSafety(Node):

    def __init__(self):
        super().__init__('rear_safety')

        def p(name, default):
            return self.declare_parameter(name, default).value

        defaults = RearSafetyConfig()
        self.cfg = RearSafetyConfig(**{k: float(p(k, getattr(defaults, k)))
                                       for k in defaults.__dataclass_fields__})
        prefix = p('frame_prefix', '')
        self.base_frame = prefix + 'base_footprint'
        self.scan_timeout = p('scan_timeout', 0.5)
        self.cmd_timeout = p('cmd_timeout', 0.5)
        self.points = []
        self.scan_time = None
        self.sensor_pose = None
        self.stopped = False
        self.interventions = 0
        self.last_state = None
        self.last_cmd = None
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.out = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)
        self.status_pub = self.create_publisher(SafetyStatus, 'safety/rear_status', 10)
        self.create_subscription(LaserScan, 'scan_rear', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'cmd_vel_in', self._on_cmd, 10)
        self.create_timer(0.2, self._on_timer)
        self.status = SafetyStatus(state='FAULT', reason='no rear scan yet')
        self.get_logger().info(
            f'rear safety: stop {self.cfg.stop_distance:.2f} m, warn '
            f'{self.cfg.warn_distance:.2f} m, corridor half-width '
            f'{self.cfg.half_width + self.cfg.side_margin:.2f} m')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg):
        if self.sensor_pose is None:
            try:
                t = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except TransformException:
                return
            self.sensor_pose = (t.transform.translation.x, t.transform.translation.y,
                                yaw_of(t.transform.rotation))
        self.points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                  msg.range_min, msg.range_max, self.sensor_pose)
        self.scan_time = self.now()

    def fresh(self):
        return self.scan_time is not None and self.now() - self.scan_time <= self.scan_timeout

    def _on_cmd(self, msg):
        self.last_cmd = self.now()
        v, w = msg.twist.linear.x, msg.twist.angular.z
        d = decide(v, w, self.points, self.fresh(), self.stopped, self.cfg)
        self.stopped = d.state == 'STOP'
        out = TwistStamped()
        out.header = msg.header
        out.twist = msg.twist
        out.twist.linear.x, out.twist.angular.z = float(d.linear_x), float(d.angular_z)
        self.out.publish(out)
        if d.linear_x != v or d.angular_z != w:
            self.interventions += 1
        self.update_status(d, v)

    def _on_timer(self):
        # When there are no commands, continue to report the
        # result that a reverse command would get.
        if self.last_cmd is None or self.now() - self.last_cmd > self.cmd_timeout:
            d = decide(-0.05, 0.0, self.points, self.fresh(), self.stopped, self.cfg)
            state = d.state if d.state in ('STOP', 'FAULT') else (
                'WARN' if d.nearest < self.cfg.warn_distance else 'CLEAR')
            self.stopped = state == 'STOP'
            self.update_status(type(d)(state, 0.0, 0.0, d.nearest,
                                       d.reason if state in ('STOP', 'FAULT') else ''), 0.0)
        self.publish_status()

    def update_status(self, d, cmd_v):
        s = self.status
        s.state, s.reason = d.state, d.reason
        s.nearest_obstacle = float(d.nearest)
        s.commanded_linear_x, s.allowed_linear_x = float(cmd_v), float(d.linear_x)
        s.interventions = self.interventions
        if d.state != self.last_state:
            # One call site for each severity: rclpy raises an
            # exception if a call site changes severity.
            text = f'rear: {d.state}' + (f' ({d.reason})' if d.reason else '')
            if d.state in ('STOP', 'FAULT'):
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
            self.last_state = d.state
            self.publish_status()

    def publish_status(self):
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status.header.frame_id = self.base_frame
        self.status_pub.publish(self.status)


def main(args=None):
    rclpy.init(args=args)
    node = RearSafety()
    try:
        # A safety filter must not stop because of one message that it cannot decode. rclpy
        # raised "Unable to convert call argument" one time when it took a message (two-robot
        # bringup). Discard that message and continue. A missing scan already causes FAULT.
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
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
