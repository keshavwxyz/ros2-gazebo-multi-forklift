"""
Load-handling skills: PickupLoad and PlaceLoad action servers.

The two skills start at a staging pose (Nav2 moves the robot there). They use only data
from the robot:
  - QR detections (/perception/qr_detections)
  - TF (odom, map)
  - the LiDAR (/scan)
  - the lift (SetForkHeight action, load estimate on /fork/state)
  - the drive (/cmd_vel/approach through twist_mux).
The simulator never moves the load. The forks go into the pallet pockets and lift the
pallet by contact.

Concurrency: the action callbacks are coroutines. They wait for futures (actions,
services, a 20 Hz tick). Thus, a single-threaded executor keeps all subscriptions active.
"""

import math
import signal

from forklift_control.load_handling_logic import (approach_command, ApproachGains,
                                                  docking_pose, fork_heights, is_aligned,
                                                  line_errors, load_pose_from_qr,
                                                  load_supported, nominal_load_in_base,
                                                  placement_poses, plausible_load_in_base,
                                                  Pose2D, RobustTarget)
from forklift_interfaces.action import PickupLoad, PlaceLoad, SetForkHeight
from forklift_interfaces.msg import ForkState, QrDetectionArray, SafetyStatus
from forklift_task_manager.mission_config import default_mission_path, load_mission
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


class HandlingFailure(Exception):

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class LoadHandler(Node):

    def __init__(self):
        super().__init__('load_handler')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.mission = load_mission(p('mission_file', '') or default_mission_path())
        self.fork_min_height = p('fork_min_height', 0.05)
        self.fork_thickness = p('fork_thickness', 0.04)
        self.heel_x = p('heel_x', 0.38)
        self.lidar_x = p('lidar_x', 0.25)
        self.insert_fork_top = p('insert_fork_top', 0.07)
        self.transport_clearance = p('transport_clearance', 0.18)
        self.place_clearance = p('place_clearance', 0.05)
        self.release_gap = p('release_gap', 0.05)
        self.heel_gap = p('heel_gap', 0.02)
        self.pre_insert_tip_gap = p('pre_insert_tip_gap', 0.10)
        self.lateral_tol = p('lateral_tolerance', 0.03)
        self.heading_tol = math.radians(p('heading_tolerance_deg', 2.0))
        self.location_tol = p('location_tolerance', 0.30)
        self.location_yaw_tol = math.radians(p('location_yaw_tolerance_deg', 15.0))
        self.placement_tol = p('placement_tolerance', 0.15)
        self.placement_yaw_tol = math.radians(p('placement_yaw_tolerance_deg', 8.0))
        self.face_tol = p('platform_face_tolerance', 0.12)
        self.platform_reach = p('platform_reach', 0.30)
        self.platform_face_margin = p('platform_face_margin', 0.04)
        self.approach_gains = ApproachGains(max_speed=p('approach_speed', 0.12))
        self.insert_gains = ApproachGains(max_speed=p('insert_speed', 0.06))
        self.reverse_gains = ApproachGains(max_speed=p('reverse_speed', 0.06))
        self.align_retries = p('alignment_retries', 2)
        self.retry_backoff = p('retry_backoff', 0.4)
        self.detect_timeout = p('detection_timeout', 8.0)
        self.min_detections = p('min_detections', 5)
        self.stationary_timeout = p('stationary_timeout', 5.0)
        self.stall_time = p('stall_time', 2.0)
        self.lift_timeout = p('lift_timeout', 30.0)
        self.settle_time = p('load_settle_time', 1.0)
        self.load_low, self.load_high = p('load_low_fraction', 0.5), p('load_high_fraction', 2.0)
        self.release_max_kg = p('release_max_kg', 5.0)
        self.verify_timeout = p('verify_timeout', 8.0)
        self.staging_tol = p('staging_tolerance', 0.25)
        self.staging_yaw_tol = math.radians(p('staging_yaw_tolerance_deg', 10.0))
        self.max_segment = p('max_docking_segment', 3.0)
        self.nav_timeout = p('navigation_timeout', 300.0)

        # for example, "forklift_1/" in a multi-robot configuration
        prefix = p('frame_prefix', '')
        self.base_frame, self.odom_frame = prefix + 'base_footprint', prefix + 'odom'
        self.rear_block_timeout = p('rear_block_timeout', 90.0)
        self.rear = None
        self.load_in_base = None       # the measured position of the load on the forks
        self.odom = None
        self.fork = None
        self.scan = None
        self.detections = None
        self._tick = Future()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(Odometry, 'odometry/filtered',
                                 lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(ForkState, 'fork/state', lambda m: setattr(self, 'fork', m), 10)
        self.create_subscription(SafetyStatus, 'safety/rear_status',
                                 lambda m: setattr(self, 'rear', (self.now(), m)), 10)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.create_subscription(QrDetectionArray, 'perception/qr_detections',
                                 self._on_detections, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel/approach', 10)
        self.lift = ActionClient(self, SetForkHeight, 'fork/set_height')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.load_state = self.create_client(SetBool, 'navigation/set_load_state')
        self.create_timer(0.05, self._on_tick)
        self.busy = False
        for name, action, cb in (('handling/pickup', PickupLoad, self._pickup),
                                 ('handling/place', PlaceLoad, self._place)):
            ActionServer(self, action, name, execute_callback=cb,
                         goal_callback=self._on_goal,
                         cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info('load_handler ready: /handling/pickup, /handling/place')

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

    def _on_goal(self, _goal):
        if self.busy:
            self.get_logger().warn('rejecting goal: a handling action is already running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_detections(self, msg):
        self.detections = msg

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        self.cmd_pub.publish(msg)

    def drive(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def pose_in(self, frame):
        try:
            t = self.tf.lookup_transform(frame, self.base_frame, Time())
        except TransformException:
            return None
        return Pose2D(t.transform.translation.x, t.transform.translation.y,
                      yaw_of(t.transform.rotation))

    def frame_transform(self, target, source):
        """Planar transform target <- source as a Pose2D (latest)."""
        t = self.tf.lookup_transform(target, source, Time())
        return Pose2D(t.transform.translation.x, t.transform.translation.y,
                      yaw_of(t.transform.rotation))

    def check_cancel(self, goal_handle):
        if goal_handle.is_cancel_requested:
            raise HandlingFailure('CANCELED', 'cancelled by the client')

    # ------------------------------------------------------------------ building blocks

    async def require_stationary(self, goal_handle):
        deadline, still_since = self.now() + self.stationary_timeout, None
        while self.now() < deadline:
            self.check_cancel(goal_handle)
            if self.odom is not None:
                t = self.odom.twist.twist
                if abs(t.linear.x) < 0.02 and abs(t.angular.z) < 0.02:
                    still_since = still_since or self.now()
                    if self.now() - still_since >= 0.5:
                        return
                else:
                    still_since = None
            self.stop()
            await self.tick()
        raise HandlingFailure('NOT_STATIONARY', 'robot did not come to rest')

    async def set_fork(self, height, goal_handle):
        if not self.lift.wait_for_server(timeout_sec=5.0):
            raise HandlingFailure('LIFT_FAILED', 'lift action server unavailable')
        send = self.lift.send_goal_async(SetForkHeight.Goal(height=float(height)))
        handle = await send
        if not handle.accepted:
            raise HandlingFailure('LIFT_FAILED', f'lift rejected height {height:.3f} m')
        result_future = handle.get_result_async()
        deadline = self.now() + self.lift_timeout
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await handle.cancel_goal_async()
                raise HandlingFailure('CANCELED', 'cancelled during lift motion')
            if self.now() > deadline:
                await handle.cancel_goal_async()
                raise HandlingFailure('LIFT_FAILED', 'lift did not finish in time')
            await self.tick()
        result = result_future.result().result
        if not result.success:
            raise HandlingFailure('LIFT_FAILED',
                                  f'lift to {height:.3f} m failed: {result.message}')

    async def ensure_at_staging(self, goal_handle, staging):
        """
        Move to the staging pose (map frame) with Nav2 if the robot is not there.

        The docking controller is a short-range line follower. It has no path planning and no
        obstacle avoidance. Do not use it to move between locations.
        """
        try:
            robot = self.frame_transform('map', self.base_frame)
        except TransformException:
            raise HandlingFailure('LOCALIZATION_UNAVAILABLE', 'no map -> base_footprint')
        dist = math.hypot(robot.x - staging.x, robot.y - staging.y)
        yaw_err = abs(math.atan2(math.sin(robot.yaw - staging.yaw),
                                 math.cos(robot.yaw - staging.yaw)))
        if dist <= self.staging_tol and yaw_err <= self.staging_yaw_tol:
            return
        self.get_logger().info(f'{dist:.2f} m from the staging pose: navigating there first')
        self.publish_feedback(goal_handle, 'navigate_to_staging')
        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise HandlingFailure('NAVIGATION_FAILED', 'Nav2 navigate_to_pose unavailable')
        goal = NavigateToPose.Goal(pose=PoseStamped())
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = staging.x, staging.y
        goal.pose.pose.orientation.z = math.sin(staging.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(staging.yaw / 2)
        handle = await self.nav.send_goal_async(goal)
        if not handle.accepted:
            raise HandlingFailure('NAVIGATION_FAILED', 'Nav2 rejected the staging goal')
        result_future = handle.get_result_async()
        deadline = self.now() + self.nav_timeout
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await handle.cancel_goal_async()
                raise HandlingFailure('CANCELED', 'cancelled while navigating to staging')
            if self.now() > deadline:
                await handle.cancel_goal_async()
                raise HandlingFailure('NAVIGATION_FAILED', 'navigation to staging timed out')
            await self.tick()
        if result_future.result().status != 4:          # GoalStatus.STATUS_SUCCEEDED
            raise HandlingFailure('NAVIGATION_FAILED', 'Nav2 did not reach the staging pose')
        await self.require_stationary(goal_handle)

    async def set_loaded(self, loaded):
        if not self.load_state.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('footprint manager unavailable; footprint not switched')
            return
        await self.load_state.call_async(SetBool.Request(data=loaded))

    async def measure_load(self, seconds):
        """Mean estimated load (kg) from /fork/state for `seconds` while the robot is stopped."""
        values, end = [], self.now() + seconds
        while self.now() < end:
            if self.fork is not None and not self.fork.moving:
                values.append(self.fork.estimated_load_kg)
            self.stop()
            await self.tick()
        return float(np.mean(values)) if values else float('nan')

    def detection_to_load(self, det, frame):
        """Load pose in `frame` from a QR detection (through TF at the image time)."""
        try:
            t = self.tf.lookup_transform(frame, det.header.frame_id,
                                         Time.from_msg(det.header.stamp),
                                         timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        r_frame_cam = quat_to_matrix(t.transform.rotation)
        tr = t.transform.translation
        p = det.pose.position
        q = r_frame_cam @ np.array([p.x, p.y, p.z]) + [tr.x, tr.y, tr.z]
        normal = r_frame_cam @ quat_to_matrix(det.pose.orientation)[:, 2]
        return load_pose_from_qr(q[0], q[1], math.atan2(normal[1], normal[0]),
                                 self.mission.load.qr_center[0])

    def front_detection(self, msg):
        """Return the detection of the load in front of the robot (smallest |lateral| in base)."""
        best, best_lat = None, None
        for det in msg.detections:
            if not det.pose_valid:
                continue
            load = self.detection_to_load(det, self.base_frame)
            if load is None or load.x <= 0:
                continue
            if best is None or abs(load.y) < best_lat:
                best, best_lat = det, abs(load.y)
        return best

    async def identify(self, box, goal_handle):
        """Collect detections of the box in front. Return its robust load pose in odom."""
        target, deadline, last_seen = RobustTarget(), self.now() + self.detect_timeout, None
        unreadable = other = None
        while self.now() < deadline:
            self.check_cancel(goal_handle)
            msg, self.detections = self.detections, None
            if msg is not None:
                if any(not d.valid and d.payload == '' for d in msg.detections):
                    unreadable = True
                det = self.front_detection(msg)
                if det is not None:
                    if not det.valid:
                        other = f'invalid code {det.payload!r} ({det.rejection_reason})'
                    elif det.box_id != box.box_id:
                        other = f'{det.box_id} ({det.payload})'
                    else:
                        load = self.detection_to_load(det, self.odom_frame)
                        if load is not None:
                            target.add(load)
                            last_seen = det
                if len(target) >= self.min_detections:
                    return target, last_seen
            self.stop()
            await self.tick()
        if other and len(target) == 0:
            raise HandlingFailure('IDENTITY_MISMATCH',
                                  f'expected {box.box_id}, the load ahead is {other}')
        if unreadable and len(target) == 0:
            raise HandlingFailure('QR_UNREADABLE', 'a QR code is in view but cannot be decoded')
        raise HandlingFailure('BOX_NOT_VISIBLE',
                              f'{box.box_id} not seen within {self.detect_timeout:.0f} s')

    def refine(self, target, box):
        """Add new detections of the box to the target estimate (if there are new detections)."""
        msg, self.detections = self.detections, None
        if msg is None:
            return
        det = self.front_detection(msg)
        if det is not None and det.valid and det.box_id == box.box_id:
            load = self.detection_to_load(det, self.odom_frame)
            if load is not None:
                target.add(load)

    async def drive_to(self, goal_handle, phase, target_fn, gains, reverse=False,
                       stall_check=False, timeout=None):
        """Follow the line to the pose from target_fn() (odom frame) until the robot gets to it."""
        start = self.pose_in(self.odom_frame)
        if start is None:
            raise HandlingFailure('LOCALIZATION_UNAVAILABLE', 'no odom -> base_footprint')
        dist = abs(line_errors(start, target_fn(), reverse).remaining)
        if dist > self.max_segment:
            raise HandlingFailure('ALIGNMENT_FAILED', f'{phase}: {dist:.2f} m docking segment '
                                  f'exceeds {self.max_segment:.1f} m (not at the staging pose)')
        deadline = self.now() + (timeout or (2.5 * dist / gains.max_speed + 10.0))
        stalled_since, feedback_due, blocked_since = None, 0.0, None
        while True:
            self.check_cancel(goal_handle)
            robot = self.pose_in(self.odom_frame)
            err = line_errors(robot, target_fn(), reverse)
            if err.remaining <= 0.005:
                self.stop()
                return err
            v, w = approach_command(err, gains, reverse)
            rear = self.rear_state() if reverse else 'CLEAR'
            if rear in ('STOP', 'FAULT'):
                # If an obstacle blocks the rear, wait (the safety filter holds the robot). Do not
                # continue. The wait time does not decrease the time limit of the manoeuvre.
                self.stop()
                now = self.now()
                if blocked_since is None:
                    blocked_since = now
                    self.get_logger().warn(f'{phase}: reversing blocked by the rear LiDAR '
                                           f'({self.rear[1].reason}); waiting')
                if now - blocked_since > self.rear_block_timeout:
                    raise HandlingFailure('REAR_BLOCKED', f'{phase}: rear path blocked for '
                                          f'{self.rear_block_timeout:.0f} s '
                                          f'({self.rear[1].reason})')
                deadline += 0.05
                stalled_since = None
                if now >= feedback_due:
                    feedback_due = now + 0.2
                    self.publish_feedback(goal_handle, 'rear_blocked', err)
                await self.tick()
                continue
            if blocked_since is not None:
                self.get_logger().info(f'{phase}: rear path clear after '
                                       f'{self.now() - blocked_since:.1f} s; continuing')
                blocked_since = None
            self.drive(v, w)
            if rear == 'SLOW':
                stalled_since = None             # the speed is low on purpose; this is not a stall
            elif stall_check and self.odom is not None:
                if abs(self.odom.twist.twist.linear.x) < 0.01 and abs(v) > 0.025:
                    stalled_since = stalled_since or self.now()
                    if self.now() - stalled_since > self.stall_time:
                        self.stop()
                        raise HandlingFailure('INSERTION_BLOCKED',
                                              f'{phase}: robot blocked (not moving)')
                else:
                    stalled_since = None
            if self.now() > deadline:
                self.stop()
                raise HandlingFailure('TIMEOUT', f'{phase}: did not reach the target in time')
            if self.now() >= feedback_due:
                feedback_due = self.now() + 0.2
                self.publish_feedback(goal_handle, phase, err)
            await self.tick()

    def rear_state(self):
        """Latest rear-safety state. If the report is missing or old, the state is FAULT."""
        if self.rear is None or self.now() - self.rear[0] > 1.0:
            return 'FAULT'
        return self.rear[1].state

    def publish_feedback(self, goal_handle, phase, err=None):
        fb = (PickupLoad.Feedback() if isinstance(goal_handle.request, PickupLoad.Goal)
              else PlaceLoad.Feedback())
        fb.phase = phase
        if err is not None:
            fb.lateral_error = float(err.lateral)
            fb.heading_error_deg = float(math.degrees(err.heading))
            fb.distance_remaining = float(err.remaining)
        goal_handle.publish_feedback(fb)

    def heights(self, surface_height=0.25):
        return fork_heights(self.mission.load, surface_height, self.fork_min_height,
                            self.fork_thickness, self.insert_fork_top,
                            self.transport_clearance, self.place_clearance, self.release_gap)

    def dock_standoff(self):
        """Return the base distance from the load center when the forks are fully in the pallet."""
        return self.mission.load.pallet_depth / 2 + self.heel_x + self.heel_gap

    def staging_standoff(self):
        """Return the base distance from the load center at the staging pose of the mission."""
        return (self.mission.load.pallet_depth / 2 + self.mission.staging_gap +
                self.mission.fork_tip_offset)

    def pre_standoff(self):
        """Return the base distance from the load center when the fork tips are at the pallet."""
        return (self.mission.load.pallet_depth / 2 + self.mission.fork_tip_offset +
                self.pre_insert_tip_gap)

    # ------------------------------------------------------------------ pickup

    async def _pickup(self, goal_handle):
        self.busy = True
        result = PickupLoad.Result()
        log = self.get_logger()
        try:
            try:
                box = self.mission.box(goal_handle.request.box_id)
            except KeyError:
                raise HandlingFailure('INVALID_REQUEST',
                                      f'unknown box {goal_handle.request.box_id!r}')
            if self.fork is not None and self.fork.load_detected:
                raise HandlingFailure('INVALID_REQUEST', f'already carrying a load '
                                      f'({self.fork.estimated_load_kg:.1f} kg on the forks)')
            h = self.heights()
            await self.ensure_at_staging(goal_handle, self.mission.pickup_staging(box.box_id))
            self.publish_feedback(goal_handle, 'check_stationary')
            await self.require_stationary(goal_handle)
            self.publish_feedback(goal_handle, 'forks_to_insertion_height')
            await self.set_fork(h.insert, goal_handle)

            self.publish_feedback(goal_handle, 'identify')
            target, _ = await self.identify(box, goal_handle)
            if target.spread() > 0.10:
                raise HandlingFailure('BOX_NOT_VISIBLE', 'inconsistent QR pose estimates '
                                      f'(spread {target.spread():.2f} m)')
            # Make sure that the box is at the location that the mission gives (map frame).
            try:
                map_odom = self.frame_transform('map', self.odom_frame)
            except TransformException:
                raise HandlingFailure('LOCALIZATION_UNAVAILABLE', 'no map -> odom transform')
            seen = map_odom.compose(target.estimate())
            expected = self.mission.pickups[box.pickup]
            dist = math.hypot(seen.x - expected.x, seen.y - expected.y)
            yaw_err = abs(math.atan2(math.sin(seen.yaw - expected.yaw),
                                     math.cos(seen.yaw - expected.yaw)))
            if dist > self.location_tol or yaw_err > self.location_yaw_tol:
                raise HandlingFailure(
                    'WRONG_LOCATION', f'{box.box_id} seen {dist:.2f} m / '
                    f'{math.degrees(yaw_err):.0f} deg from {box.pickup}')
            log.info(f'{box.box_id} identified at {box.pickup} ({dist * 100:.0f} cm from config)')

            # Align to the pre-insertion pose with the camera. Then compare with the tolerances.
            def refined_pre():
                # add detections while the camera sees the box
                self.refine(target, box)
                return docking_pose(target.estimate(), self.pre_standoff())

            for attempt in range(self.align_retries + 1):
                await self.drive_to(goal_handle, 'align', refined_pre, self.approach_gains)
                await self.sleep(0.5)
                # new estimate while the robot is stationary
                for _ in range(10):
                    self.refine(target, box)
                    await self.tick()
                err = line_errors(self.pose_in(self.odom_frame),
                                  docking_pose(target.estimate(), self.dock_standoff()))
                log.info(f'alignment attempt {attempt + 1}: lateral {err.lateral * 100:.1f} cm,'
                         f' heading {math.degrees(err.heading):.1f} deg')
                if is_aligned(err, self.lateral_tol, self.heading_tol):
                    break
                if attempt == self.align_retries:
                    raise HandlingFailure(
                        'ALIGNMENT_FAILED', f'lateral {err.lateral * 100:.1f} cm, heading '
                        f'{math.degrees(err.heading):.1f} deg after {attempt + 1} attempts')
                back = self.pose_in(self.odom_frame).compose(Pose2D(-self.retry_backoff, 0.0, 0.0))
                await self.drive_to(goal_handle, 'back_off', lambda: back,
                                    self.reverse_gains, reverse=True)
            result.lateral_error = float(err.lateral)
            result.heading_error_deg = float(math.degrees(err.heading))

            # Insertion: straight along the docking line (at this
            # distance, the camera cannot see the QR code).
            dock = docking_pose(target.estimate(), self.dock_standoff())
            await self.drive_to(goal_handle, 'insert', lambda: dock, self.insert_gains,
                                stall_check=True)
            await self.require_stationary(goal_handle)
            # The real position of the load on the forks (camera estimate before the insertion +
            # odometry of the insertion). The placement compensates for the insertion depth.
            measured = self.pose_in(self.odom_frame).relative(target.estimate())
            if plausible_load_in_base(measured, self.dock_standoff(), 0.05, 0.20, 0.04,
                                      math.radians(5.0)):
                self.load_in_base = measured
            else:
                log.warn(f'implausible load-on-forks estimate ({measured.x:.3f}, '
                         f'{measured.y:.3f} m); placement will assume nominal insertion')
                self.load_in_base = nominal_load_in_base(self.dock_standoff())
            log.info(f'load on forks: {self.load_in_base.x:.3f} m ahead (nominal '
                     f'{self.dock_standoff():.3f}), {100 * self.load_in_base.y:+.1f} cm lateral')

            self.publish_feedback(goal_handle, 'lift')
            await self.set_fork(h.transport, goal_handle)
            self.publish_feedback(goal_handle, 'verify_load')
            await self.sleep(self.settle_time)
            load_kg = await self.measure_load(1.0)
            result.measured_load_kg = load_kg
            if not load_supported(load_kg, self.mission.load.mass, self.load_low, self.load_high):
                # Recoverable: lower the forks and move rearward with no load.
                await self.set_fork(h.insert, goal_handle)
                staging = docking_pose(target.estimate(), self.staging_standoff())
                await self.drive_to(goal_handle, 'back_out', lambda: staging,
                                    self.reverse_gains, reverse=True)
                raise HandlingFailure('PICKUP_NOT_VERIFIED', f'lift effort indicates '
                                      f'{load_kg:.1f} kg, expected ~{self.mission.load.mass:.0f}')
            log.info(f'pickup verified: {load_kg:.1f} kg on the forks')
            await self.set_loaded(True)

            # Move rearward to the staging pose. Nav2 starts from
            # the pose where it stopped the robot.
            staging = docking_pose(target.estimate(), self.staging_standoff())
            await self.drive_to(goal_handle, 'back_out', lambda: staging, self.reverse_gains,
                                reverse=True)
            result.success, result.message = True, f'{box.box_id} picked up ({load_kg:.1f} kg)'
            goal_handle.succeed()
        except HandlingFailure as f:
            self.stop()
            result.success = False
            result.error_code = getattr(PickupLoad.Result, f'ERROR_{f.code}')
            result.message = str(f)
            log.warn(f'pickup failed [{f.code}]: {f}')
            goal_handle.canceled() if f.code == 'CANCELED' else goal_handle.abort()
        finally:
            self.stop()
            self.busy = False
        return result

    # ------------------------------------------------------------------ placement

    def platform_face_range(self):
        """Median LiDAR range in front of the robot (+/- 3 deg), or None."""
        scan = self.scan
        if scan is None:
            return None
        r = np.asarray(scan.ranges, float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        sel = r[(np.abs(a) < math.radians(3.0)) & np.isfinite(r)]
        return float(np.median(sel)) if len(sel) else None

    async def _place(self, goal_handle):
        self.busy = True
        result = PlaceLoad.Result()
        log = self.get_logger()
        req = goal_handle.request
        try:
            dest = self.mission.destinations.get(req.destination_id)
            if dest is None:
                raise HandlingFailure('DESTINATION_INVALID',
                                      f'unknown destination {req.destination_id!r}')
            try:
                box = self.mission.box(req.box_id)
            except KeyError:
                raise HandlingFailure('DESTINATION_INVALID', f'unknown box {req.box_id!r}')
            if box.destination != dest.dest_id:
                raise HandlingFailure('DESTINATION_MISMATCH', f'{box.box_id} belongs at '
                                      f'{box.destination}, not {dest.dest_id}')
            if self.fork is None or not self.fork.load_detected:
                raise HandlingFailure('NOT_CARRYING', 'lift effort shows no load on the forks')
            h = self.heights(dest.surface_height)
            await self.ensure_at_staging(goal_handle,
                                         self.mission.destination_staging(dest.dest_id))
            await self.require_stationary(goal_handle)
            self.publish_feedback(goal_handle, 'forks_to_placement_height')
            await self.set_fork(h.place_raised, goal_handle)

            try:
                odom_map = self.frame_transform(self.odom_frame, 'map')
            except TransformException:
                raise HandlingFailure('LOCALIZATION_UNAVAILABLE', 'no odom -> map transform')
            dest_load = odom_map.compose(Pose2D(dest.pose.x, dest.pose.y, dest.pose.yaw))
            carried = self.load_in_base or nominal_load_in_base(self.dock_standoff())
            pre, dock = placement_poses(dest_load, carried,
                                        self.pre_standoff() - self.dock_standoff())

            await self.drive_to(goal_handle, 'align', lambda: pre, self.approach_gains)
            await self.sleep(0.5)
            # Make sure that the platform is there: compare the LiDAR
            # range to its front face with the map.
            robot = self.pose_in(self.odom_frame)
            face = dest_load.compose(Pose2D(dest.face_offset, 0.0, 0.0))
            expected = robot.relative(face).x - self.lidar_x
            measured = self.platform_face_range()
            if measured is None or abs(measured - expected) > self.face_tol:
                raise HandlingFailure('DESTINATION_NOT_FOUND', f'platform face expected at '
                                      f'{expected:.2f} m, LiDAR reads {measured}')
            err = line_errors(robot, dock)
            if not is_aligned(err, 2 * self.lateral_tol, 2 * self.heading_tol):
                raise HandlingFailure('ALIGNMENT_FAILED', f'lateral {err.lateral * 100:.1f} cm, '
                                      f'heading {math.degrees(err.heading):.1f} deg')
            # The *measured* face also limits the insertion
            # depth. The mast uprights (their front is
            # at platform_reach, below the deck) must stay platform_face_margin from the face.
            # If the dock uses only the map, the gap is approximately 1 cm. This is less than the
            # localization error.
            room = measured + self.lidar_x - self.platform_reach - self.platform_face_margin
            if err.remaining > room:
                cut = err.remaining - room
                dock = dock.compose(Pose2D(-cut, 0.0, 0.0))
                log.info(f'insertion shortened by {100 * cut:.1f} cm to keep the mast '
                         f'{100 * self.platform_face_margin:.0f} cm from the platform face '
                         f'(LiDAR {measured:.3f} m)')

            await self.drive_to(goal_handle, 'insert', lambda: dock, self.insert_gains,
                                stall_check=True)
            await self.require_stationary(goal_handle)

            self.publish_feedback(goal_handle, 'lower')
            await self.set_fork(h.release, goal_handle)
            await self.sleep(self.settle_time)
            load_kg = await self.measure_load(1.0)
            if load_kg > self.release_max_kg:
                await self.set_fork(h.place_raised, goal_handle)       # continue to hold it
                raise HandlingFailure('PLACEMENT_NOT_VERIFIED', f'{load_kg:.1f} kg still on the '
                                      'forks after lowering (platform not taking the load)')
            log.info(f'load released ({load_kg:.1f} kg left on the forks)')
            await self.set_loaded(False)

            # Move rearward to the staging pose. At the pre-insertion pose, the fork tips touch the
            # platform face. Nav2 would then start in collision.
            staging = docking_pose(dest_load, self.staging_standoff())
            await self.drive_to(goal_handle, 'retract', lambda: staging, self.reverse_gains,
                                reverse=True, stall_check=True)
            # The forks must come out with no load (the robot would pull a caught pallet).
            after_kg = await self.measure_load(1.0)
            if after_kg > self.release_max_kg:
                raise HandlingFailure('PLACEMENT_NOT_VERIFIED', f'{after_kg:.1f} kg on the '
                                      'forks after retracting: the load came back with them')
            # Park at the insertion height (the next pickup starts there). Do not park on the lower
            # hard stop of the lift. In Gazebo, if the joint is on its limit while the robot moves,
            # the joint stays there and the next lift has a timeout (docs/gazebo_setup.md).
            await self.set_fork(h.insert, goal_handle)

            # Independent check: the camera must now see the box at its destination.
            self.publish_feedback(goal_handle, 'verify')
            self.detections = None
            target, _ = await self.identify_at_destination(box, goal_handle)
            seen = self.frame_transform('map', self.odom_frame).compose(target.estimate())
            result.placement_error = float(math.hypot(seen.x - dest.pose.x, seen.y - dest.pose.y))
            yaw_err = math.atan2(math.sin(seen.yaw - dest.pose.yaw),
                                 math.cos(seen.yaw - dest.pose.yaw))
            result.placement_yaw_error_deg = float(math.degrees(yaw_err))
            if result.placement_error > self.placement_tol or \
                    abs(yaw_err) > self.placement_yaw_tol:
                raise HandlingFailure(
                    'PLACEMENT_NOT_VERIFIED', f'{box.box_id} seen {result.placement_error:.2f} m'
                    f' / {math.degrees(yaw_err):.1f} deg from {dest.dest_id}')
            result.success = True
            result.message = (f'{box.box_id} placed at {dest.dest_id} '
                              f'({result.placement_error * 100:.0f} cm from target)')
            log.info(result.message)
            goal_handle.succeed()
        except HandlingFailure as f:
            self.stop()
            result.success = False
            result.error_code = getattr(PlaceLoad.Result, f'ERROR_{f.code}',
                                        PlaceLoad.Result.ERROR_PLACEMENT_NOT_VERIFIED)
            result.message = str(f)
            log.warn(f'placement failed [{f.code}]: {f}')
            goal_handle.canceled() if f.code == 'CANCELED' else goal_handle.abort()
        finally:
            self.stop()
            self.busy = False
        return result

    async def identify_at_destination(self, box, goal_handle):
        try:
            return await self.identify(box, goal_handle)
        except HandlingFailure as f:
            raise HandlingFailure('PLACEMENT_NOT_VERIFIED',
                                  f'box not confirmed at the destination by the camera: {f}')


def main(args=None):
    rclpy.init(args=args)
    node = LoadHandler()
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
