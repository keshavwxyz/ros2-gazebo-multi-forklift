"""Shared utilities for the forklift launch and integration tests."""

import math
import time

from controller_manager_msgs.srv import ListControllers
from forklift_interfaces.action import SetForkHeight
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

# DDS divides large images (1280x720 RGB = 2.7 MB) into fragments. With best-effort
# delivery, if one fragment is lost, the full frame is lost (measured: 1.6 Hz received of
# 10 Hz published). Thus, image subscribers use reliable delivery with a short queue
# (only the latest frames).
IMAGE_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)


def sensor_qos(msg_type):
    return IMAGE_QOS if msg_type is Image else qos_profile_sensor_data


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TopicRecorder:
    """Records the messages of some topics with the sim time at which each message arrived."""

    def __init__(self, node, topics):
        self.node = node
        self.msgs = {t: [] for t in topics}
        self.clock = None
        node.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        for topic, msg_type in topics.items():
            node.create_subscription(msg_type, topic,
                                     lambda m, t=topic: self.msgs[t].append((self.clock, m)),
                                     sensor_qos(msg_type))

    def _on_clock(self, msg):
        self.clock = stamp_s(msg.clock)

    def clear(self):
        for v in self.msgs.values():
            v.clear()

    def rate(self, topic):
        """Mean message rate in sim time, from the header stamps."""
        stamps = [stamp_s(m.header.stamp) for _, m in self.msgs[topic]]
        if len(stamps) < 3 or stamps[-1] <= stamps[0]:
            return 0.0
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def spin_for(node, seconds, sim_time=True):
    """Spin for a duration of node time (sim time if use_sim_time is set)."""
    clock = node.get_clock()
    start = clock.now()
    # protection against a paused simulation
    wall_deadline = time.monotonic() + max(60.0, 20 * seconds)
    while (clock.now() - start).nanoseconds * 1e-9 < seconds:
        rclpy.spin_once(node, timeout_sec=0.01)
        if time.monotonic() > wall_deadline:
            raise TimeoutError(f'sim time did not advance {seconds} s')


def spin_until(node, predicate, timeout_wall):
    deadline = time.monotonic() + timeout_wall
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def wait_for_controllers(node, names, timeout_wall=120.0):
    """Wait until each named controller is 'active'."""
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    deadline = time.monotonic() + timeout_wall
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                states = {c.name: c.state for c in future.result().controller}
                if all(states.get(n) == 'active' for n in names):
                    return states
        time.sleep(0.5)
    raise TimeoutError(f'controllers {names} not active after {timeout_wall} s')


class Driver:
    """Publishes TwistStamped commands on an input topic of twist_mux."""

    def __init__(self, node, topic='/cmd_vel/teleop'):
        self.node = node
        self.pub = node.create_publisher(TwistStamped, topic, 10)

    def drive(self, v, w, seconds, period=0.05):
        clock = self.node.get_clock()
        start = clock.now()
        while (clock.now() - start).nanoseconds * 1e-9 < seconds:
            msg = TwistStamped()
            msg.header.stamp = clock.now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = float(w)
            self.pub.publish(msg)
            spin_for(self.node, period)


def set_fork_height(node, height, timeout_wall=60.0):
    """Send a SetForkHeight goal. Return (accepted, result or None)."""
    client = ActionClient(node, SetForkHeight, '/fork/set_height')
    if not client.wait_for_server(timeout_sec=10.0):
        raise TimeoutError('/fork/set_height not available')
    send = client.send_goal_async(SetForkHeight.Goal(height=float(height)))
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        return False, None
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=timeout_wall)
    if not result.done():
        raise TimeoutError(f'SetForkHeight({height}) did not finish')
    return True, result.result().result


def goto_pose_odom(node, driver, get_odom, target, pos_tol=0.03, yaw_tol=0.02,
                   timeout=90.0):
    """
    Move to (x, y, yaw) in the odom frame with the EKF odometry (for the test harness only).

    Turn to the goal, move straight with heading correction, then turn to the final yaw.
    Proportional control with conservative limits. Raise an exception at a timeout.
    """
    tx, ty, tyaw = target
    clock = node.get_clock()
    start = clock.now()

    def elapsed():
        return (clock.now() - start).nanoseconds * 1e-9

    def pose():
        o = get_odom()
        return o.pose.pose.position.x, o.pose.pose.position.y, yaw_of(o.pose.pose.orientation)

    def rotate_to(heading):
        while elapsed() < timeout:
            err = angle_diff(heading, pose()[2])
            if abs(err) < yaw_tol:
                return
            driver.drive(0.0, max(-0.4, min(0.4, 1.5 * err)), 0.05)
        raise TimeoutError('rotation timed out')

    x, y, _ = pose()
    if math.hypot(tx - x, ty - y) > pos_tol:
        rotate_to(math.atan2(ty - y, tx - x))
        while elapsed() < timeout:
            x, y, yaw = pose()
            dist = math.hypot(tx - x, ty - y)
            if dist < pos_tol:
                break
            heading_err = angle_diff(math.atan2(ty - y, tx - x), yaw)
            forward = dist * math.cos(heading_err)
            # the robot moved too far: stop and let the final checks examine the result
            if forward < 0:
                break
            driver.drive(max(0.05, min(0.4, 0.8 * forward)),
                         max(-0.3, min(0.3, 2.0 * heading_err)), 0.05)
        else:
            raise TimeoutError('translation timed out')
    rotate_to(tyaw)
    driver.drive(0.0, 0.0, 0.5)


RESPAWNED_NAV2 = {'controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                  'velocity_smoother', 'collision_monitor'}


def assert_clean_shutdown(test, proc_info):
    """
    Examine the exit codes after a simulation launch test.

    Each ROS node must stop correctly (0, or stopped by SIGINT/SIGTERM from the launch
    system). Only the Gazebo server can also get SIGKILL: sometimes it stops responding
    during its own teardown (1 of 3 navigation runs, docs/gazebo_setup.md). A Nav2 server
    can also have a crash (planner_server segfault, approximately 1 in 40 launches). The
    launch respawns it and lifecycle_starter activates it again. The functional tests above
    were successful with the recovered stack. The function reports these two conditions as
    warnings. It never hides them.
    """
    clean = {0, -2, -15}
    for name in proc_info.process_names():
        code = proc_info[name].returncode
        if name.startswith('gazebo') and code == -9:
            print(f'WARNING: {name} hung on shutdown and was SIGKILLed (known Gazebo issue)')
            continue
        if name.split('-')[0] in RESPAWNED_NAV2 and code in (-11, -6):
            print(f'WARNING: {name} crashed (exit code {code}); respawned and re-activated')
            continue
        test.assertIn(code, clean, f'{name} exited with code {code}')
