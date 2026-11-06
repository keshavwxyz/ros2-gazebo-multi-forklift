"""
Fleet agent: robot side of the coordination protocol (one for each forklift, in its namespace).

The agent publishes fleet/robot_status (forklift_interfaces/RobotStatus) at 2 Hz. This is
the heartbeat. It uses only the estimates and reports of the robot: AMCL pose (map frame),
EKF speed, mission-manager state, fork load sensing, rear-LiDAR safety state and the next
metres of the Nav2 plan. The commands come from the coordinator through the interfaces of
the robot (mission/run action, mission/pause and mission/resume services, Nav2 actions).
The agent itself never moves the robot.

The agent also publishes fleet/peer_obstacles (sensor_msgs/PointCloud2, map frame, 5 Hz).
These points fill the outline of each other robot (body and fork tines) at the pose that
the robot gives in its heartbeat. The two Nav2 costmaps use this cloud as an extra marking
source (robot.launch.py adds it when the fleet agent runs). The tines are below the two
LiDAR scan planes. Without this cloud, the costmap of a robot sees a different robot only
by its chassis, 1.4 m behind the fork tips. (In Gazebo, two forklifts that moved head-on
touched fork tip to fork tip.) When the peer moves away, the LiDAR rays of the robot clear
the old points. A heartbeat is a maximum of approximately 0.5 s old. Thus, the agent makes
the outline larger by `peer_margin` plus the speed of the peer multiplied by `peer_latency`.
"""

import math
import signal

from action_msgs.msg import GoalStatus, GoalStatusArray
from forklift_interfaces.msg import ForkState, MissionStatus, RobotStatus, SafetyStatus
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

ROBOT_STATE = {
    'INITIALIZE': 'TASK_ASSIGNED', 'CHECK_SYSTEMS': 'TASK_ASSIGNED',
    'LOCALIZE': 'TASK_ASSIGNED', 'SELECT_NEXT_BOX': 'TASK_ASSIGNED',
    'NAVIGATE_TO_PICKUP': 'NAVIGATING_TO_PICKUP',
    'DETECT_AND_IDENTIFY_BOX': 'PICKING', 'APPROACH_BOX': 'PICKING',
    'ALIGN_FORKS': 'PICKING', 'PICKUP': 'PICKING', 'VERIFY_PICKUP': 'PICKING',
    'NAVIGATE_TO_DESTINATION': 'NAVIGATING_TO_DESTINATION',
    'ALIGN_FOR_PLACEMENT': 'PLACING', 'PLACE_BOX': 'PLACING', 'VERIFY_PLACEMENT': 'PLACING',
    'UPDATE_MISSION_STATE': 'PLACING', 'RETURN_HOME': 'RETURNING',
    'PAUSED': 'PAUSED', 'EMERGENCY_STOP': 'PAUSED',
}
RUNNING = set(ROBOT_STATE)
ACTIVE_GOAL = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
               GoalStatus.STATUS_CANCELING)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# Robot outline as rectangles in base_footprint (x0, x1, half-width): chassis + wheels +
# rear LiDAR housing, and the fork tines (footprints.yaml, forklift_params.yaml).
OUTLINE = ((-1.23, 0.41, 0.61), (0.41, 1.38, 0.28))


def outline_points(x, y, yaw, margin=0.1, ahead=0.0, spacing=0.1):
    """Points that fill a robot outline (larger by `margin`, longer to the front by `ahead`)."""
    pts = []
    for x0, x1, h in OUTLINE:
        lo, hi, w = x0 - margin, x1 + margin + ahead, h + margin
        xs = np.linspace(lo, hi, int(math.ceil((hi - lo) / spacing)) + 1)
        ys = np.linspace(-w, w, int(math.ceil(2 * w / spacing)) + 1)
        gx, gy = np.meshgrid(xs, ys)
        pts.append(np.stack([gx.ravel(), gy.ravel()], axis=1))
    local = np.concatenate(pts)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * local[:, 0] - s * local[:, 1],
                     y + s * local[:, 0] + c * local[:, 1]], axis=1)


def path_ahead(poses, x, y, length=6.0, spacing=0.25):
    """Decrease the points of a Nav2 plan, for `length` metres from the point nearest to (x, y)."""
    pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
    if not pts:
        return []
    i0 = min(range(len(pts)), key=lambda i: math.dist(pts[i], (x, y)))
    out, travelled, since = [pts[i0]], 0.0, 0.0
    for a, b in zip(pts[i0:], pts[i0 + 1:]):
        step = math.dist(a, b)
        travelled += step
        since += step
        if travelled > length:
            break
        if since >= spacing:
            out.append(b)
            since = 0.0
    return out


class FleetAgent(Node):

    def __init__(self):
        super().__init__('fleet_agent')
        self.robot_id = self.declare_parameter('robot_id', 'forklift').value
        self.max_pos_var = self.declare_parameter('max_position_variance', 0.25).value
        self.base_frame = self.declare_parameter('frame_prefix', '').value + 'base_footprint'
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mission = self.amcl = self.odom = self.fork = self.rear = self.plan = None
        self.plan_time = 0.0
        self.nav_active = False
        self.latched = latched
        self.subscribe_latched()
        self.create_timer(10.0, self._watchdog)
        self.create_subscription(Odometry, 'odometry/filtered',
                                 lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(ForkState, 'fork/state', lambda m: setattr(self, 'fork', m), 10)
        self.create_subscription(SafetyStatus, 'safety/rear_status',
                                 lambda m: setattr(self, 'rear', m), 10)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status',
                                 self._on_nav_status, 10)
        self.pub = self.create_publisher(RobotStatus, 'fleet/robot_status', 10)
        self.create_timer(0.5, self.publish)
        # Outlines of the other robots for the costmaps of this robot.
        self.peer_margin = self.declare_parameter('peer_margin', 0.10).value
        self.peer_latency = self.declare_parameter('peer_latency', 0.5).value
        self.peer_timeout = self.declare_parameter('peer_timeout', 3.0).value
        self.peers = {}
        for peer in self.declare_parameter('peers', ['']).value:
            if peer and peer != self.robot_id:
                self.create_subscription(RobotStatus, f'/{peer}/fleet/robot_status',
                                         lambda m: self.peers.__setitem__(
                                             m.robot_id, (self.now(), m)), 10)
        self.peer_pub = self.create_publisher(PointCloud2, 'fleet/peer_obstacles', 10)
        self.create_timer(0.2, self.publish_peers)
        self.get_logger().info(f'fleet agent for {self.robot_id}: '
                               f'{self.resolve_topic_name("fleet/robot_status")}')

    def subscribe_latched(self):
        self.mission_sub = self.create_subscription(
            MissionStatus, 'mission/status', lambda m: setattr(self, 'mission', m),
            self.latched)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', lambda m: setattr(self, 'amcl', m),
            self.latched)

    def _watchdog(self):
        """Subscribe again to latched topics that sent no data (one match was missed)."""
        if self.mission is None or self.amcl is None:
            self.get_logger().warn('mission/status or amcl_pose not received yet: '
                                   're-subscribing')
            self.destroy_subscription(self.mission_sub)
            self.destroy_subscription(self.amcl_sub)
            self.subscribe_latched()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_plan(self, msg):
        self.plan, self.plan_time = msg, self.now()

    def _on_nav_status(self, msg):
        self.nav_active = any(s.status in ACTIVE_GOAL for s in msg.status_list)

    def publish(self):
        s = RobotStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'map'
        s.robot_id = self.robot_id
        m = self.mission
        mission_state = m.state if m is not None else 'IDLE'
        s.mission_state = mission_state
        s.mission_running = mission_state in RUNNING
        s.nav_active = self.nav_active
        s.state = ROBOT_STATE.get(mission_state, 'IDLE')
        if m is not None:
            s.current_box_id = m.current_box_id if s.mission_running else ''
            s.current_destination_id = m.current_destination_id if s.mission_running else ''
            s.boxes_delivered = list(m.boxes_delivered)
            s.boxes_picked = list(m.boxes_picked)
            s.failure_reason = m.failure_reason
        # Pose: TF map -> base (AMCL correction + EKF: current, high rate). The localization
        # quality comes from the AMCL covariance when its pose message is available.
        try:
            t = self.tf.lookup_transform('map', self.base_frame, Time())
            s.x, s.y = float(t.transform.translation.x), float(t.transform.translation.y)
            s.yaw = float(yaw_of(t.transform.rotation))
            have_tf = True
        except TransformException:
            have_tf = False
        if self.amcl is not None:
            c = self.amcl.pose.covariance
            s.localized = have_tf and bool(c[0] < self.max_pos_var and c[7] < self.max_pos_var)
        else:
            s.localized = have_tf
        if self.odom is not None:
            v = self.odom.twist.twist.linear
            s.speed = float(math.hypot(v.x, v.y))
        s.carrying = bool(self.fork is not None and self.fork.load_detected)
        if mission_state == 'MISSION_FAILED' and s.carrying:
            s.state = 'FAULT'
        if self.rear is not None:
            s.rear_state, s.rear_nearest = self.rear.state, float(self.rear.nearest_obstacle)
        if self.plan is not None and self.nav_active and self.now() - self.plan_time < 5.0:
            s.path_ahead = [Point(x=x, y=y) for x, y in
                            path_ahead(self.plan.poses, s.x, s.y)]
        self.pub.publish(s)

    def publish_peers(self):
        pts = [outline_points(m.x, m.y, m.yaw, self.peer_margin, m.speed * self.peer_latency)
               for t, m in self.peers.values()
               if m.localized and self.now() - t <= self.peer_timeout]
        xy = np.concatenate(pts) if pts else np.zeros((0, 2))
        cloud = np.column_stack([xy, np.full(len(xy), 0.1)]).astype(np.float32)
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        self.peer_pub.publish(point_cloud2.create_cloud_xyz32(header, cloud))


def main(args=None):
    rclpy.init(args=args)
    node = FleetAgent()
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
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
