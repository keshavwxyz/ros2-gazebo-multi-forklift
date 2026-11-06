"""
Set the Nav2 lifecycle nodes to ACTIVE. This node is robust to lost service responses.

This node replaces the autostart of nav2_lifecycle_manager. With Fast DDS, a lifecycle
node sometimes does not send its change_state *response* during startup ("failed to send
response ... (timeout)": discovery of the reply reader of the client was not complete).
nav2_lifecycle_manager then waits permanently and localization does not become active
(this occurred one time in approximately 10 navigation launches).

This node sets each managed node to ACTIVE, in sequence. After each request (with or
without a response), it reads the real state of the node again with get_state. If
necessary, it tries again. Because it reads the state again, a new try is safe: usually,
a transition with a lost response already occurred.

After the startup, the node continues to supervise (supervise_period). If a managed node
is in a state that is not ACTIVE, the node sets it to ACTIVE again (a respawned process
starts UNCONFIGURED). One time, the planner_server of Nav2 had a segfault during configure
(1 crash in approximately 40 launches, in the Nav2 binary). The launch respawns the Nav2
servers and this node activates them again.

Optional (set_initial_pose): the node then publishes the start pose, which the operator
knows, on /initialpose with an explicit, small covariance. The parameter-based initial
pose of nav2_amcl has no covariance setting and starts with a wide particle cloud. At
home, the scan gives almost no constraint on x. Thus, the cloud sometimes moved to a
solution 0.5 m off during the first turn (measured: 54.6 cm in 1 of 3 runs; 3-8 cm in
the other runs).

With verify_tf = [parent, child] (child gets frame_prefix), the node then makes sure that
localization publishes (parent -> child is available in verify_timeout). With two robots,
AMCL one time accepted the initial pose but did not publish map -> odom (discovery did not
find a subscription; this is the same intermittent fault as above). Nav2 then did not
become active. If the transform is not available, the node cycles the last managed node
(AMCL): deactivate and cleanup (configure makes its subscriptions again), then activate.
Then the node sends the initial pose again.
"""

import math
import signal
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

TRANSITION_FOR = {State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
                  State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE}


def call(node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def make_clients(node, name):
    # relative names: they resolve in the namespace of the starter (= the namespace of the robot)
    return (node.create_client(GetState, f'{name}/get_state'),
            node.create_client(ChangeState, f'{name}/change_state'))


def bring_up(node, name, clients, request_timeout, deadline):
    get_state, change = clients
    log = node.get_logger()
    while time.monotonic() < deadline:
        if not (get_state.wait_for_service(timeout_sec=1.0) and
                change.wait_for_service(timeout_sec=1.0)):
            continue
        state = call(node, get_state, GetState.Request(), request_timeout)
        if state is None:
            log.warn(f'{name}: get_state response lost, retrying')
            continue
        current = state.current_state.id
        if current == State.PRIMARY_STATE_ACTIVE:
            log.info(f'{name}: active')
            return True
        transition = TRANSITION_FOR.get(current)
        # the transition is in progress or the node is finalized: wait and examine again
        if transition is None:
            time.sleep(0.2)
            continue
        response = call(node, change, ChangeState.Request(
            transition=Transition(id=transition)), request_timeout)
        if response is None:
            log.warn(f'{name}: change_state response lost; re-checking the actual state')
        elif not response.success:
            log.error(f'{name}: transition {transition} rejected; retrying')
            time.sleep(1.0)
    log.error(f'{name}: not active before the deadline')
    return False


def supervise(node, names, clients, request_timeout, period, bring_up_timeout):
    """Activate again each managed node that is not ACTIVE (for example, after a respawn)."""
    log = node.get_logger()
    while rclpy.ok():
        end = time.monotonic() + period
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for name in names:
            get_state = clients[name][0]
            if not get_state.service_is_ready():
                # the process stopped; a respawn (if configured) will be visible again
                continue
            state = call(node, get_state, GetState.Request(), request_timeout)
            if state is None or state.current_state.id == State.PRIMARY_STATE_ACTIVE:
                continue                    # lost response: examine again at the next period
            log.error(f'{name}: found {state.current_state.label} (restarted?); '
                      'bringing it up again')
            if bring_up(node, name, clients[name], request_timeout,
                        time.monotonic() + bring_up_timeout):
                log.info(f'{name}: recovered')


def wait_for_tf(node, tf, parent, child, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if tf.can_transform(parent, child, Time()):
            return True
    return False


def cycle(node, name, clients, request_timeout, deadline):
    """Set a node back to UNCONFIGURED (deactivate, cleanup). Then set it to ACTIVE again."""
    get_state, change = clients
    for transition in (Transition.TRANSITION_DEACTIVATE, Transition.TRANSITION_CLEANUP):
        call(node, change, ChangeState.Request(transition=Transition(id=transition)),
             request_timeout)
    state = call(node, get_state, GetState.Request(), request_timeout)
    node.get_logger().warn(f'{name}: cycled, now '
                           f'{state.current_state.label if state else "unknown"}')
    return bring_up(node, name, clients, request_timeout, deadline)


def publish_initial_pose(node, x, y, yaw, stddev, frame, repeats=5):
    pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    sx, sy, syaw = stddev
    msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35] = (
        sx * sx, sy * sy, syaw * syaw)
    end = time.monotonic() + 10.0
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(repeats):          # sent again: there is no acknowledgement of the delivery
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
    node.get_logger().info(f'initial pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg), '
                           f'stddev {sx:.2f} m / {math.degrees(syaw):.1f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = Node('lifecycle_starter')
    names = node.declare_parameter('node_names', ['']).value
    request_timeout = node.declare_parameter('request_timeout', 5.0).value
    total_timeout = node.declare_parameter('total_timeout', 180.0).value
    give_start_pose = node.declare_parameter('set_initial_pose', False).value
    # dynamic typing: launch arguments such as x:=0 arrive as integers
    pose = [float(node.declare_parameter(k, 0.0, ParameterDescriptor(dynamic_typing=True)).value)
            for k in ('initial_x', 'initial_y', 'initial_yaw')]
    stddev = node.declare_parameter('initial_pose_stddev', [0.05, 0.05, 0.035]).value
    period = node.declare_parameter('supervise_period', 2.0).value       # <= 0: off
    verify = [f for f in node.declare_parameter('verify_tf', ['']).value if f]
    if len(verify) == 2:            # the child frame is a frame of this robot
        verify[1] = node.declare_parameter('frame_prefix', '').value + verify[1]
    verify_timeout = node.declare_parameter('verify_timeout', 20.0).value
    verify_attempts = node.declare_parameter('verify_attempts', 2).value
    clients = {name: make_clients(node, name) for name in names}
    ok = True
    try:
        deadline = time.monotonic() + total_timeout
        # the sequence is important (for example, map_server before amcl)
        for name in names:
            ok = bring_up(node, name, clients[name], request_timeout, deadline) and ok
            if not ok:
                break
        if ok:
            node.get_logger().info(f'all managed nodes active: {", ".join(names)}')
            if give_start_pose:
                publish_initial_pose(node, *pose, stddev, 'map')
            if len(verify) == 2:
                tf = Buffer()
                listener = TransformListener(tf, node)  # noqa: F841 (kept alive)
                for attempt in range(verify_attempts + 1):
                    if wait_for_tf(node, tf, *verify, verify_timeout):
                        node.get_logger().info(f'{verify[0]} -> {verify[1]} available')
                        break
                    if attempt == verify_attempts:
                        node.get_logger().error(
                            f'no {verify[0]} -> {verify[1]} after {attempt} restarts of '
                            f'{names[-1]}; continuing to supervise')
                        break
                    node.get_logger().warn(
                        f'no {verify[0]} -> {verify[1]} within {verify_timeout:.0f} s: '
                        f'restarting {names[-1]}')
                    if cycle(node, names[-1], clients[names[-1]], request_timeout,
                             time.monotonic() + total_timeout) and give_start_pose:
                        publish_initial_pose(node, *pose, stddev, 'map')
            if period > 0:
                supervise(node, names, clients, request_timeout, period, total_timeout)
            else:
                # continue to run. Thus, the launch does not report a stopped process.
                rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
