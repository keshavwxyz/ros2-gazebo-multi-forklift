"""
Fork lift action server.

This node gives forklift_interfaces/action/SetForkHeight on /fork/set_height. It publishes
forklift_interfaces/msg/ForkState on /fork/state. It is the only node that sends commands
to the lift. For each goal, the node does these steps:
  1. It plans a trapezoidal profile. The joint velocity limit is the maximum speed.
  2. It sends the profile to the fork_lift_controller (joint_trajectory_controller on the
     velocity interface).
  3. It compares the measured joint position (/joint_states) with the profile. It finds
     stalls, timeouts and old sensor data.
The joint limits come from /robot_description.

Concurrency: no operation blocks. The supervision runs in a timer. The execute coroutine of
the action waits for a future, and the timer completes this future. Thus, a single-threaded
executor processes the joint states, the clock and the action in sequence. (An earlier
execute loop blocked. With the multi-threaded executor of rclpy, the subscriptions did not
get data for up to approximately 1 s.)

The node does not depend on the backend. Its behavior is the same with mock, Gazebo and
(later) Isaac Sim hardware, because it only uses ros2_control topics.
"""

from dataclasses import dataclass
import math
import signal

from builtin_interfaces.msg import Duration
from forklift_control.lift_logic import (at_target, estimate_load_kg, joint_limits_from_urdf,
                                         moving_mass_from_urdf, TrackingMonitor,
                                         TrapezoidProfile, validate_goal)
from forklift_interfaces.action import SetForkHeight
from forklift_interfaces.msg import ForkState
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def to_duration(seconds):
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


@dataclass
class ActiveGoal:
    handle: object
    profile: TrapezoidProfile
    tolerance: float
    t0: float
    deadline: float
    tracker: TrackingMonitor
    done: Future
    ticks: int = 0


class LiftController(Node):

    def __init__(self):
        super().__init__('lift_controller')
        p = self.declare_parameter
        self.joint_name = p('joint_name', 'fork_lift_joint').value
        trajectory_topic = p('trajectory_topic', 'fork_lift_controller/joint_trajectory').value
        self.default_velocity = p('default_velocity', 0.15).value
        self.max_acceleration = p('max_acceleration', 0.3).value
        self.waypoint_period = p('waypoint_period', 0.1).value
        self.default_tolerance = p('default_tolerance', 0.005).value
        self.settle_velocity = p('settle_velocity', 0.01).value
        # The node lifts the forks one time at startup, off the lower hard stop. In Gazebo, if a
        # lift joint is on its limit while the robot moves, the joint can stay there
        # (docs/gazebo_setup.md). 0 = off.
        self.startup_height = p('startup_height', 0.02).value
        self.startup_done = self.startup_height <= 0.0
        self.startup_sent, self.startup_attempts = None, 0
        control_rate = p('control_rate', 50.0).value
        state_rate = p('state_publish_rate', 20.0).value
        self.timeout_margin = p('timeout_margin', 3.0).value
        self.tracking_error_limit = p('tracking_error_limit', 0.05).value
        self.tracking_error_time = p('tracking_error_time', 1.0).value
        self.joint_state_timeout = p('joint_state_timeout', 0.5).value
        self.load_sensing_enabled = p('load_sensing_enabled', False).value
        # Lift effort with empty forks. If <= 0, the node calculates it from robot_description (the
        # mass that the lift joint carries x g). Thus, the load
        # estimate does not need a manual calibration.
        self.empty_effort = p('empty_effort', 0.0).value
        self.gravity = p('gravity', 9.81).value
        self.load_threshold_kg = p('load_threshold_kg', 5.0).value
        self.feedback_every = max(1, round(control_rate / 10.0))   # 10 Hz feedback

        self.limits = None
        self.position = None
        self.velocity = 0.0
        self.effort = 0.0
        self.state_stamp = None         # header stamp of the latest joint state (s)
        self.target = None              # active goal or hold height
        self.fault = ''
        self.goal = None                # ActiveGoal

        self.latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.desc_sub = self.create_subscription(String, 'robot_description',
                                                 self._on_description, self.latched)
        self.js_sub = self.create_subscription(JointState, 'joint_states',
                                               self._on_joint_states, 10)
        self.create_timer(5.0, self._input_watchdog)
        self.create_timer(0.5, self._startup_raise)
        self.traj_pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.state_pub = self.create_publisher(ForkState, 'fork/state', 10)
        self.create_timer(1.0 / control_rate, self._control_tick)
        self.create_timer(1.0 / state_rate, self._publish_state)

        self.action_server = ActionServer(
            self, SetForkHeight, 'fork/set_height',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info('lift_controller ready, waiting for /robot_description')

    # ------------------------------------------------------------------ inputs

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_description(self, msg):
        try:
            self.limits = joint_limits_from_urdf(msg.data, self.joint_name)
        except Exception as exc:  # noqa: BLE001 - ValueError or malformed XML
            self.get_logger().error(f'cannot read lift limits: {exc}')
            return
        lim = self.limits
        self.get_logger().info(f'{self.joint_name} limits [{lim.lower:.3f}, {lim.upper:.3f}] m, '
                               f'{lim.velocity:.3f} m/s')
        if self.empty_effort <= 0.0:
            mass = moving_mass_from_urdf(msg.data, self.joint_name)
            self.empty_effort = mass * self.gravity
            self.get_logger().info(f'empty-fork effort {self.empty_effort:.1f} N '
                                   f'({mass:.1f} kg carried by {self.joint_name})')

    def _on_joint_states(self, msg):
        if self.joint_name not in msg.name:
            return
        i = msg.name.index(self.joint_name)
        self.position = msg.position[i]
        self.velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
        self.effort = msg.effort[i] if i < len(msg.effort) else 0.0
        self.state_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _input_watchdog(self):
        """
        Make the subscriptions again if they did not receive data.

        This occurred one time: a node in one of two robots with a namespace did not receive its
        description or its joint states.
        """
        if self.limits is None:
            self.get_logger().warn('no robot_description yet: re-subscribing')
            self.destroy_subscription(self.desc_sub)
            self.desc_sub = self.create_subscription(String, 'robot_description',
                                                     self._on_description, self.latched)
        if self.position is None:
            self.get_logger().warn('no joint_states yet: re-subscribing')
            self.destroy_subscription(self.js_sub)
            self.js_sub = self.create_subscription(JointState, 'joint_states',
                                                   self._on_joint_states, 10)

    def _startup_raise(self):
        """When the trajectory controller is ready, lift the forks off the hard stop."""
        if self.startup_done or self.limits is None or self.position is None or \
                self.goal is not None or self._state_age() > self.joint_state_timeout:
            return
        if self.position >= self.startup_height - self.default_tolerance:
            if self.startup_sent is not None:
                self.get_logger().info(f'startup: forks at {self.position:.3f} m')
            self.startup_done = True
            return
        if self.traj_pub.get_subscription_count() == 0:
            return                       # the controller is not ready: the command would be lost
        duration = abs(self.startup_height - self.position) / self.default_velocity + 0.5
        if self.startup_sent is not None and self.now() < self.startup_sent + duration + 2.0:
            return                       # the lift moves now
        if self.startup_attempts >= 10:
            self.get_logger().error('startup: forks did not leave the lower hard stop')
            self.startup_done = True
            return
        self.startup_attempts += 1
        self.startup_sent = self.now()
        self.target = self.startup_height
        self._send_trajectory([(duration, self.startup_height, 0.0)])
        self.get_logger().info(f'startup: forks {self.position:.3f} -> '
                               f'{self.startup_height:.3f} m (off the lower hard stop, '
                               f'attempt {self.startup_attempts})')

    def _state_age(self):
        return math.inf if self.state_stamp is None else self.now() - self.state_stamp

    # ------------------------------------------------------------------ outputs

    def _send_trajectory(self, waypoints):
        """Send (t, position, velocity) waypoints. They replace the current trajectory."""
        msg = JointTrajectory(joint_names=[self.joint_name])
        # A zero stamp tells the controller to start immediately on its clock.
        for t, pos, vel in waypoints:
            msg.points.append(JointTrajectoryPoint(
                positions=[pos], velocities=[vel], time_from_start=to_duration(t)))
        self.traj_pub.publish(msg)

    def _hold(self, height):
        """Stop smoothly at `height` (for an abort or a cancel)."""
        self.target = height
        self._send_trajectory([(0.25, height, 0.0)])

    def _publish_state(self):
        if self.position is None:
            return
        msg = ForkState()
        msg.header.stamp = rclpy.time.Time(seconds=self.state_stamp).to_msg()
        msg.height = self.position
        msg.target_height = self.target if self.target is not None else math.nan
        if self.limits is not None:
            msg.min_height, msg.max_height = self.limits.lower, self.limits.upper
        msg.moving = abs(self.velocity) > self.settle_velocity
        msg.at_target = self.target is not None and at_target(
            self.position, self.velocity, self.target, self.default_tolerance,
            self.settle_velocity)
        if self.load_sensing_enabled:
            msg.estimated_load_kg = estimate_load_kg(self.effort, self.empty_effort,
                                                     self.gravity)
            msg.load_detected = msg.estimated_load_kg >= self.load_threshold_kg
        stale = self._state_age() > self.joint_state_timeout
        msg.fault = bool(self.fault) or stale
        msg.fault_reason = self.fault or ('joint_states stale' if stale else '')
        self.state_pub.publish(msg)

    # ------------------------------------------------------------------ action

    def _on_goal(self, goal):
        velocity = goal.max_velocity or self.default_velocity
        ok, reason = validate_goal(goal.height, velocity, self.limits)
        if ok and self._state_age() > self.joint_state_timeout:
            ok, reason = False, 'no recent /joint_states for the lift joint'
        if not ok:
            self.get_logger().warn(f'rejected SetForkHeight({goal.height:.3f}): {reason}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle):
        if self.goal is not None:   # the newest goal replaces the goal that runs
            self._complete('preempt', 'pre-empted by a newer goal', hold=False)
        goal_handle.execute()

    async def _execute(self, goal_handle):
        goal = goal_handle.request
        velocity = goal.max_velocity or self.default_velocity
        profile = TrapezoidProfile(self.position, goal.height, velocity, self.max_acceleration)
        now = self.now()
        self.target, self.fault = goal.height, ''
        done = Future()
        self.goal = ActiveGoal(
            handle=goal_handle, profile=profile,
            tolerance=goal.tolerance or self.default_tolerance,
            t0=now, deadline=now + profile.duration + self.timeout_margin,
            tracker=TrackingMonitor(self.tracking_error_limit, self.tracking_error_time),
            done=done)
        self._send_trajectory(profile.sample(self.waypoint_period))
        outcome, message = await done

        final = self.position
        {'succeed': goal_handle.succeed, 'cancel': goal_handle.canceled}.get(
            outcome, goal_handle.abort)()
        text = f'SetForkHeight {outcome}: {message} (height {final:.3f} m)'
        # rclpy does not let one call site change the severity. Thus, there are two call sites.
        if outcome == 'succeed':
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        return SetForkHeight.Result(success=(outcome == 'succeed'), final_height=final,
                                    message=message)

    def _complete(self, outcome, message, hold):
        goal, self.goal = self.goal, None
        if outcome == 'abort':
            self.fault = message
        if hold:
            self._hold(self.position)
        goal.done.set_result((outcome, message))

    def _control_tick(self):
        """Compare the active goal with its profile (runs at control_rate)."""
        g = self.goal
        if g is None:
            return
        if g.handle.is_cancel_requested:
            return self._complete('cancel', 'cancelled', hold=True)
        age = self._state_age()
        if age > self.joint_state_timeout:
            return self._complete('abort', f'joint_states stale during lift motion '
                                           f'(age {age:.2f} s)', hold=True)
        if at_target(self.position, self.velocity, g.profile.target, g.tolerance,
                     self.settle_velocity):
            return self._complete('succeed', 'reached target', hold=False)
        # Compare the measurement with the profile at the time of the *measurement*.
        reference, _ = g.profile.at(self.state_stamp - g.t0)
        if g.tracker.update(reference, self.position, self.state_stamp):
            return self._complete('abort', f'tracking error > {self.tracking_error_limit:.3f} m '
                                           f'for {self.tracking_error_time:.1f} s '
                                           f'(stall or obstruction)', hold=True)
        if self.now() > g.deadline:
            return self._complete('abort', 'timeout before reaching target', hold=True)
        g.ticks += 1
        if g.ticks % self.feedback_every == 0:
            g.handle.publish_feedback(SetForkHeight.Feedback(
                current_height=self.position, error=g.profile.target - self.position))


def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
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
