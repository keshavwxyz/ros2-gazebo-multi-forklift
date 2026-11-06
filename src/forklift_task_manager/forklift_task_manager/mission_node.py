"""
Mission manager: does the three-box mission with the state machine in mission_fsm.py.

Interfaces:
  action  /mission/run      forklift_interfaces/RunMission (cancel = stop the mission)
  service /mission/pause    std_srvs/Trigger   (delayed during pickup / placement)
  service /mission/resume   std_srvs/Trigger
  topic   /estop            std_msgs/Bool      (true = emergency stop; twist_mux also locks
                                                the motion with it; resume needs /estop false)
  topic   /mission/status   forklift_interfaces/MissionStatus (2 Hz and at each change)

The data that each state uses (only data from the robot, never the ground truth of the simulator):
  INITIALIZE       action servers: Nav2, /handling/pickup, /handling/place, /fork/set_height
  CHECK_SYSTEMS    new data on /scan, /imu/data,
  /odometry/filtered, /fork/state, QR detector output
  LOCALIZE         /amcl_pose covariance below the limits and TF map -> base_footprint
  NAVIGATE_*       Nav2 NavigateToPose to the staging poses from the mission configuration
  DETECT_AND_...   a valid QR detection for the selected box: its *decoded* destination
                   becomes the task (the node never uses the configured destination directly)
  PICKUP/PLACE_BOX the load-handling skills (forklift_control/load_handler.py)
  RETURN_HOME      Nav2 to the configured home pose
Each state has a timeout. At the end of the mission, the
node also writes the mission record as JSON.
"""

import datetime
import json
import math
import os
import signal

from forklift_interfaces.action import PickupLoad, PlaceLoad, RunMission, SetForkHeight
from forklift_interfaces.msg import ForkState, MissionStatus, QrDetectionArray
from forklift_task_manager.mission_config import default_mission_path, load_mission
from forklift_task_manager.mission_fsm import (MANIPULATION, Mission, Outcome,
                                               PICKUP_PHASES, PLACE_PHASES, S)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

STATUS_SUCCEEDED = 4                 # action_msgs/GoalStatus


def error_names(result_type):
    """Map the ERROR_* constants of an action Result to their names (without the prefix)."""
    return {getattr(result_type, k): k[len('ERROR_'):]
            for k in dir(result_type) if k.startswith('ERROR_')}


# Sensors that CHECK_SYSTEMS uses: key -> (topic, type, QoS).
SENSORS = {
    'scan': ('scan', LaserScan, qos_profile_sensor_data),
    'imu': ('imu/data', Imu, qos_profile_sensor_data),
    'odom': ('odometry/filtered', Odometry, 10),
    'fork': ('fork/state', ForkState, 10),
    'qr': ('perception/qr_detections', QrDetectionArray, 10),
}


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.mission = load_mission(p('mission_file', '') or default_mission_path())
        self.retries = {s.value: p(f'retries.{s.value}', 0) for s in (
            S.INITIALIZE, S.CHECK_SYSTEMS, S.LOCALIZE, S.NAVIGATE_TO_PICKUP,
            S.DETECT_AND_IDENTIFY_BOX, S.PICKUP, S.NAVIGATE_TO_DESTINATION, S.PLACE_BOX,
            S.RETURN_HOME)}
        self.server_timeout = p('server_timeout', 60.0)
        self.sensor_timeout = p('sensor_timeout', 10.0)
        self.localize_timeout = p('localize_timeout', 20.0)
        self.max_pos_var = p('max_position_variance', 0.05)      # m^2
        self.max_yaw_var = p('max_yaw_variance', 0.05)           # rad^2
        self.detect_window = p('detect_window', 6.0)
        self.nav_timeout = p('navigation_timeout', 400.0)
        self.skill_timeout = p('skill_timeout', 400.0)
        self.retry_delay = p('retry_delay', 2.0)
        self.record_dir = os.path.expanduser(p('record_dir', '~/.ros/forklift_missions'))
        self.autostart = p('autostart', False)
        self.base_frame = p('frame_prefix', '') + 'base_footprint'
        self.robot_id = p('robot_id', '')

        self.last_msg = {}
        self.detections = None
        self.amcl = None
        self.estop = False
        self.fsm = None
        self.substate = ''
        self.interrupted_manipulation = False
        self._tick = Future()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.sensor_subs = {}
        for key in SENSORS:
            self.subscribe_sensor(key)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 lambda m: setattr(self, 'amcl', m), latched)
        self.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.status_pub = self.create_publisher(MissionStatus, 'mission/status', latched)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pickup = ActionClient(self, PickupLoad, 'handling/pickup')
        self.place = ActionClient(self, PlaceLoad, 'handling/place')
        self.lift = ActionClient(self, SetForkHeight, 'fork/set_height')
        self.pickup_errors = error_names(PickupLoad.Result)
        self.place_errors = error_names(PlaceLoad.Result)

        self.create_service(Trigger, 'mission/pause', self._on_pause)
        self.create_service(Trigger, 'mission/resume', self._on_resume)
        ActionServer(self, RunMission, 'mission/run', execute_callback=self._on_run,
                     goal_callback=self._on_goal,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_timer(0.05, self._on_tick)
        self.create_timer(0.5, self.publish_status)
        if self.autostart:
            self._autostart_timer = self.create_timer(1.0, self._start_once)
        self.get_logger().info(
            f'mission_manager ready: {len(self.mission.box_ids)} boxes '
            f'({", ".join(self.mission.box_ids)}); action /mission/run')

    # ------------------------------------------------------------------ plumbing

    def _on_tick(self):
        fut, self._tick = self._tick, Future()
        fut.set_result(True)

    async def tick(self):
        await self._tick

    async def sleep(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            await self.tick()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_detections(self, msg):
        self.detections = msg
        self.last_msg['qr'] = self.now()

    @property
    def running(self):
        return self.fsm is not None and not self.fsm.finished

    def _on_goal(self, _goal):
        if self.running:
            self.get_logger().warn('rejecting mission goal: a mission is already running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _start_once(self):
        self._autostart_timer.cancel()
        if not self.running:
            self.executor.create_task(self.run_mission([], None))

    # ------------------------------------------------------------------ operator inputs

    def _on_pause(self, _req, res):
        if not self.running:
            res.success, res.message = False, 'no mission running'
        else:
            self.fsm.interrupt('pause')
            res.success = True
            res.message = ('pause deferred until the current manipulation completes'
                           if self.fsm.pause_pending else 'paused')
            self.get_logger().info(f'mission: {res.message}')
            self.publish_status()
        return res

    def _on_resume(self, _req, res):
        if self.running and self.fsm.pause_pending:
            self.fsm.pause_pending = False              # the delayed pause did not occur
            res.success, res.message = True, 'pending pause cancelled'
            self.get_logger().info(f'mission: {res.message}')
        elif not self.running or self.fsm.state not in (S.PAUSED, S.EMERGENCY_STOP):
            res.success, res.message = False, 'mission is not paused or stopped'
        elif self.estop:
            res.success, res.message = False, 'emergency stop still engaged (/estop true)'
        else:
            state = self.fsm.resume(self.interrupted_manipulation)
            self.interrupted_manipulation = False
            res.success = state != S.MISSION_FAILED
            res.message = (f'resumed: {state.value}' if res.success
                           else self.fsm.record.failure_reason)
            self.get_logger().info(f'mission: {res.message}')
            self.publish_status()
        return res

    def _on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().error('EMERGENCY STOP engaged')
            if self.running:
                self.fsm.interrupt('estop')
                self.publish_status()
        elif not msg.data and self.estop:
            self.get_logger().warn('emergency stop released; call /mission/resume to continue')
        self.estop = msg.data

    # ------------------------------------------------------------------ status / record

    def publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.fsm is None:
            msg.state = S.IDLE.value
        else:
            rec = self.fsm.record
            msg.state = self.fsm.state.value
            if self.substate and self.fsm.state in MANIPULATION | {
                    S.NAVIGATE_TO_PICKUP, S.NAVIGATE_TO_DESTINATION}:
                msg.state = self.substate
            msg.current_box_id = self.fsm.current_box
            msg.current_destination_id = self.fsm.current_destination
            msg.boxes_detected = list(rec.boxes_detected)
            msg.boxes_picked = list(rec.boxes_picked)
            msg.boxes_delivered = list(rec.boxes_delivered)
            msg.boxes_failed = list(rec.boxes_failed)
            msg.boxes_total = rec.boxes_total
            msg.paused = self.fsm.state in (S.PAUSED, S.EMERGENCY_STOP)
            msg.mission_complete = self.fsm.state == S.MISSION_COMPLETE
            msg.failure_reason = rec.failure_reason
        self.status_pub.publish(msg)

    def write_record(self, started, box_ids):
        rec = self.fsm.record
        data = {'started_ros_time': started, 'ended_ros_time': self.now(),
                'final_state': self.fsm.state.value, 'success': self.fsm.success,
                'boxes': box_ids, 'tasks_from_qr': rec.tasks,
                'detected': rec.boxes_detected, 'picked': rec.boxes_picked,
                'delivered': rec.boxes_delivered, 'failed': rec.boxes_failed,
                'failure_reason': rec.failure_reason,
                'events': [{'state': s, 'outcome': o} for s, o in rec.events]}
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            name = datetime.datetime.now().strftime('mission_%Y%m%d_%H%M%S.json')
            path = os.path.join(self.record_dir, name)
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'mission record written to {path}')
        except OSError as e:
            self.get_logger().warn(f'could not write the mission record: {e}')

    # ------------------------------------------------------------------ mission loop

    async def _on_run(self, goal_handle):
        return await self.run_mission(list(goal_handle.request.box_ids), goal_handle,
                                      goal_handle.request.skip_return_home)

    async def run_mission(self, box_ids, goal_handle, skip_return_home=False):
        unknown = [b for b in box_ids if b not in self.mission.box_ids]
        box_ids = box_ids or list(self.mission.box_ids)
        self.fsm = Mission(box_ids, self.retries, return_home=not skip_return_home)
        self.substate = ''
        self.interrupted_manipulation = False
        started = self.now()
        if unknown:
            self.fsm.state = S.MISSION_FAILED
            self.fsm.record.failure_reason = f'unknown box ids: {", ".join(unknown)}'
        else:
            self.fsm.start()
            self.get_logger().info(f'mission started: {", ".join(box_ids)}')
        while not self.fsm.finished:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                self.fsm.cancel('mission cancelled by the operator')
                break
            state = self.fsm.state
            if state in (S.PAUSED, S.EMERGENCY_STOP):
                await self.tick()
                continue
            self.substate = ''
            self.publish_status()
            self.send_feedback(goal_handle)
            outcome = await self.execute(state, goal_handle)
            if self.fsm.state != state or self.fsm.finished:
                # Interrupted (pause / emergency stop / cancel): discard the result. After resume,
                # the state runs again from its start.
                if state in MANIPULATION and self.fsm.state == S.EMERGENCY_STOP:
                    self.interrupted_manipulation = True
                continue
            if goal_handle is not None and goal_handle.is_cancel_requested:
                continue
            new = self.fsm.advance(outcome)
            if outcome.ok:
                self.get_logger().info(f'{state.value} -> {new.value}')
            else:
                self.get_logger().warn(
                    f'{state.value} failed ({outcome.code}: {outcome.message}) -> {new.value}')
                if new == state:
                    await self.sleep(self.retry_delay)   # let temporary causes go away
        self.substate = ''
        self.publish_status()
        rec = self.fsm.record
        summary = (f'{self.fsm.state.value}: delivered {len(rec.boxes_delivered)}/'
                   f'{rec.boxes_total} ({", ".join(rec.boxes_delivered) or "none"})')
        if rec.failure_reason:
            summary += f'; {rec.failure_reason}'
        if self.fsm.success:           # one call site for each severity (rclpy requirement)
            self.get_logger().info(summary)
        else:
            self.get_logger().error(summary)
        self.write_record(started, box_ids)
        result = RunMission.Result(success=self.fsm.success,
                                   boxes_delivered=list(rec.boxes_delivered),
                                   boxes_failed=list(rec.boxes_failed), message=summary)
        if goal_handle is not None:
            if self.fsm.state == S.CANCELED:
                goal_handle.canceled()
            elif self.fsm.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        return result

    def send_feedback(self, goal_handle):
        if goal_handle is None:
            return
        fb = RunMission.Feedback(state=self.substate or self.fsm.state.value,
                                 current_box_id=self.fsm.current_box,
                                 boxes_delivered_count=len(self.fsm.record.boxes_delivered),
                                 boxes_total=self.fsm.record.boxes_total)
        goal_handle.publish_feedback(fb)

    def interrupted(self, state, goal_handle):
        return (self.fsm.state != state or
                (goal_handle is not None and goal_handle.is_cancel_requested))

    async def execute(self, state, goal_handle):
        handler = {
            S.INITIALIZE: self.do_initialize, S.CHECK_SYSTEMS: self.do_check_systems,
            S.LOCALIZE: self.do_localize, S.SELECT_NEXT_BOX: self.do_instant,
            S.NAVIGATE_TO_PICKUP: self.do_nav_pickup,
            S.DETECT_AND_IDENTIFY_BOX: self.do_identify, S.PICKUP: self.do_pickup,
            S.NAVIGATE_TO_DESTINATION: self.do_nav_destination, S.PLACE_BOX: self.do_place,
            S.UPDATE_MISSION_STATE: self.do_instant, S.RETURN_HOME: self.do_return_home,
        }[state]
        return await handler(state, goal_handle)

    # ------------------------------------------------------------------ states

    async def do_instant(self, _state, _goal_handle):
        return Outcome(True)

    async def do_initialize(self, state, goal_handle):
        clients = {'Nav2 navigate_to_pose': self.nav, 'handling/pickup': self.pickup,
                   'handling/place': self.place, 'fork/set_height': self.lift}
        deadline = self.now() + self.server_timeout
        while True:
            missing = [n for n, c in clients.items() if not c.server_is_ready()]
            if not missing:
                return Outcome(True)
            if self.now() > deadline:
                return Outcome(False, 'SYSTEMS_UNAVAILABLE',
                               'action servers not available: ' + ', '.join(missing))
            if self.interrupted(state, goal_handle):
                return Outcome(False, 'INTERRUPTED')
            await self.sleep(0.5)

    def subscribe_sensor(self, key):
        topic, msg_type, qos = SENSORS[key]
        if key in self.sensor_subs:
            self.destroy_subscription(self.sensor_subs[key])
        if key == 'qr':
            cb = self._on_detections
        else:
            def cb(_m, key=key):
                self.last_msg[key] = self.now()
        self.sensor_subs[key] = self.create_subscription(msg_type, topic, cb, qos)

    async def do_check_systems(self, state, goal_handle):
        t0 = self.now()
        while True:
            stale = [key for key in SENSORS if self.last_msg.get(key, -1.0) < t0]
            if not stale:
                return Outcome(True)
            if self.now() - t0 > self.sensor_timeout:
                # In a large graph, discovery can miss a publisher (this occurred with two robots).
                # Subscribe again. Thus, the new try gets a new match.
                for key in stale:
                    self.subscribe_sensor(key)
                return Outcome(False, 'SENSOR_UNAVAILABLE',
                               'no data on ' + ', '.join(SENSORS[k][0] for k in stale))
            if self.interrupted(state, goal_handle):
                return Outcome(False, 'INTERRUPTED')
            await self.tick()

    async def do_localize(self, state, goal_handle):
        deadline = self.now() + self.localize_timeout
        why = 'no /amcl_pose'
        while self.now() < deadline:
            if self.interrupted(state, goal_handle):
                return Outcome(False, 'INTERRUPTED')
            if self.amcl is not None:
                c = self.amcl.pose.covariance
                if c[0] > self.max_pos_var or c[7] > self.max_pos_var:
                    why = f'position variance too high ({c[0]:.3f}, {c[7]:.3f} m^2)'
                elif c[35] > self.max_yaw_var:
                    why = f'yaw variance too high ({c[35]:.3f} rad^2)'
                else:
                    try:
                        self.tf.lookup_transform('map', self.base_frame, Time())
                        return Outcome(True)
                    except TransformException:
                        why = 'no map -> base_footprint transform'
            await self.sleep(0.2)
        return Outcome(False, 'LOCALIZATION_FAILED', why)

    async def do_nav_pickup(self, state, goal_handle):
        return await self.navigate(self.mission.pickup_staging(self.fsm.current_box),
                                   state, goal_handle)

    async def do_nav_destination(self, state, goal_handle):
        dest = self.fsm.current_destination
        if dest not in self.mission.destinations:
            return Outcome(False, 'DESTINATION_INVALID', f'{dest!r} is not configured')
        return await self.navigate(self.mission.destination_staging(dest), state, goal_handle)

    async def do_return_home(self, state, goal_handle):
        return await self.navigate(self.mission.home, state, goal_handle)

    async def navigate(self, pose, state, goal_handle):
        goal = NavigateToPose.Goal(pose=PoseStamped())
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose.x, pose.y
        goal.pose.pose.orientation.z = math.sin(pose.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(pose.yaw / 2)
        status, _result = await self.run_action(self.nav, goal, self.nav_timeout, state,
                                                goal_handle)
        if status is None:
            return Outcome(False, 'INTERRUPTED')
        if status == 'TIMEOUT':
            return Outcome(False, 'TIMEOUT', f'navigation exceeded {self.nav_timeout:.0f} s')
        if status != STATUS_SUCCEEDED:
            return Outcome(False, 'NAVIGATION_FAILED',
                           f'Nav2 ended with status {status} going to '
                           f'({pose.x:.2f}, {pose.y:.2f})')
        return Outcome(True)

    async def do_identify(self, state, goal_handle):
        """Read the QR code of the box. The decoded destination becomes the task."""
        box = self.fsm.current_box
        t0 = self.now()
        seen_other, unreadable = '', ''
        while self.now() - t0 < self.detect_window:
            if self.interrupted(state, goal_handle):
                return Outcome(False, 'INTERRUPTED')
            msg, self.detections = self.detections, None
            for det in (msg.detections if msg is not None else []):
                if not det.valid:
                    unreadable = det.rejection_reason or 'undecodable'
                elif det.box_id == box:
                    self.get_logger().info(f'QR {det.payload!r}: task {box} -> '
                                           f'{det.destination_id}')
                    return Outcome(True, destination_id=det.destination_id, box_id=box)
                else:
                    seen_other = det.box_id
            await self.tick()
        if seen_other:
            return Outcome(False, 'IDENTITY_MISMATCH',
                           f'expected {box} at its pickup, QR reads {seen_other}')
        if unreadable:
            return Outcome(False, 'QR_UNREADABLE', unreadable)
        return Outcome(False, 'BOX_NOT_VISIBLE', f'no QR code for {box} at its pickup')

    async def do_pickup(self, state, goal_handle):
        goal = PickupLoad.Goal(box_id=self.fsm.current_box)
        return await self.run_skill(self.pickup, goal, PICKUP_PHASES, self.pickup_errors,
                                    state, goal_handle)

    async def do_place(self, state, goal_handle):
        goal = PlaceLoad.Goal(box_id=self.fsm.current_box,
                              destination_id=self.fsm.current_destination)
        return await self.run_skill(self.place, goal, PLACE_PHASES, self.place_errors,
                                    state, goal_handle)

    async def run_skill(self, client, goal, phases, errors, state, goal_handle):
        def on_feedback(fb):
            sub = phases.get(fb.feedback.phase, '')
            if sub and sub != self.substate:
                self.substate = sub
                self.publish_status()
                self.send_feedback(goal_handle)
        status, result = await self.run_action(client, goal, self.skill_timeout, state,
                                               goal_handle, on_feedback)
        if status is None:
            return Outcome(False, 'INTERRUPTED')
        if status == 'TIMEOUT':
            # The skill was cancelled before its end: the load state is not known now.
            return Outcome(False, 'LOAD_STATE_UNKNOWN',
                           f'skill exceeded {self.skill_timeout:.0f} s')
        if result is not None and result.success:
            return Outcome(True, message=result.message)
        code = errors.get(result.error_code, 'UNKNOWN') if result is not None else 'UNKNOWN'
        msg = result.message if result is not None else f'status {status}'
        return Outcome(False, code, msg)

    async def run_action(self, client, goal, timeout, state, goal_handle, feedback=None):
        """
        Send an action goal and wait for its result. Cancel the goal if an interruption occurs.

        Return (status, result). status None = interrupted,
        'TIMEOUT' = the time limit was exceeded.
        """
        if not client.wait_for_server(timeout_sec=5.0):
            return -1, None
        handle = await client.send_goal_async(goal, feedback_callback=feedback)
        if not handle.accepted:
            return -2, None
        result_future = handle.get_result_async()
        deadline = self.now() + timeout
        while not result_future.done():
            if self.interrupted(state, goal_handle) or self.now() > deadline:
                timed_out = self.now() > deadline
                await handle.cancel_goal_async()
                end = self.now() + 30.0
                while not result_future.done() and self.now() < end:
                    await self.tick()
                return ('TIMEOUT' if timed_out else None), None
            await self.tick()
        res = result_future.result()
        return res.status, res.result


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
