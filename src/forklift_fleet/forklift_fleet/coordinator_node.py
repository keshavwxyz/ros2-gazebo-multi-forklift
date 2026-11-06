"""
Fleet coordinator for the shared warehouse mission.

The coordinator gives the tasks to the forklifts and solves the conflicts between them.
It never sends commands to motors or simulator entities.

Protocol (all ROS 2, for each robot <id> = its namespace):
  robot -> coordinator  /<id>/fleet/robot_status (RobotStatus, 2 Hz heartbeat)
                        RunMission feedback / result (task progress, completion)
  coordinator -> robot  /<id>/mission/run  RunMission{box_ids: [box], skip_return_home}
                        = task offer; goal accepted = acknowledgement, rejected = refusal
                        /<id>/mission/pause, /<id>/mission/resume (yield / resume)
                        /<id>/navigate_to_pose (go home / waiting location), /<id>/backup
  shared                /fleet/status (FleetStatus, latched: tasks, robots, reservations)
                        /fleet/conflicts (ConflictEvent), /fleet/claim_task (ClaimTask)
                        /fleet/run (RunMission action: run the fleet mission)

Each cycle (2 Hz) does these steps in sequence:
  1. availability (heartbeats)
  2. task tracking (only pickups and placements that the robot verified)
  3. zone reservations
  4. conflict detection and resolution
  5. allocation and dispatch
  6. completion (all robots are at home).
All inputs are the estimates of the robots.
"""

import datetime
import json
import math
import os
import signal

from action_msgs.msg import GoalStatus
from forklift_fleet.allocator import AllocatorConfig, plan, RobotView, straight_route, TaskView
from forklift_fleet.conflicts import (choose_priority, Contender, DeadlockMonitor, in_zone,
                                      path_conflict, ReservationTable)
from forklift_fleet.registry import (ACTIVE, DELIVERED, DELIVERY_IN_PROGRESS, PICKED_UP,
                                     PICKUP_IN_PROGRESS, TaskRegistry, TransitionError)
from forklift_interfaces.action import RunMission
from forklift_interfaces.msg import (ConflictEvent, FleetStatus, Reservation, RobotStatus,
                                     TaskState)
from forklift_interfaces.srv import ClaimTask
from forklift_task_manager.mission_config import default_mission_path, load_mission
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_srvs.srv import Trigger

STATUS_SUCCEEDED = 4
MOVING_STATES = ('NAVIGATING_TO_PICKUP', 'NAVIGATING_TO_DESTINATION', 'RETURNING')


class Robot:
    """Data and command handles of one forklift in the coordinator."""

    def __init__(self, node, robot_id, home):
        self.id, self.home = robot_id, home
        self.status = None
        self.heartbeat = None
        self.available = False
        self.benched_until = 0.0
        self.failures = 0
        self.lost = False
        self.mission = None             # (task_id, goal handle) of the RunMission that runs
        self.dispatching = False
        # (purpose, target, goal handle) navigation goal of the coordinator
        self.nav = None
        self.nav_resend = None          # (purpose, target) to send again after a yield
        self.yields = set()             # conflict ids for which this robot yields
        self.paused = False
        self.home_failures = 0          # home navigations that were not successful
        ns = f'/{robot_id}'
        self.run = ActionClient(node, RunMission, f'{ns}/mission/run')
        self.nav_client = ActionClient(node, NavigateToPose, f'{ns}/navigate_to_pose')
        self.backup = ActionClient(node, BackUp, f'{ns}/backup')
        self.pause_cli = node.create_client(Trigger, f'{ns}/mission/pause')
        self.resume_cli = node.create_client(Trigger, f'{ns}/mission/resume')

    @property
    def pose(self):
        s = self.status
        return (s.x, s.y) if s is not None else None

    def at(self, target, tol=0.6):
        return self.pose is not None and math.dist(self.pose, target[:2]) <= tol


class Conflict:

    def __init__(self, cid, kind, priority, yielder, resource, now, rule, monitor):
        self.id, self.kind, self.priority, self.yielder = cid, kind, priority, yielder
        self.resource, self.start, self.rule, self.monitor = resource, now, rule, monitor
        self.status = 'ACTIVE'
        self.flipped = self.backed_up = self.moved_away = False
        self.resolution = f'{yielder} waits; {priority} has right of way ({rule})'
        self.recovery = ''
        self.applied = False            # the yield is in effect
        self.yielder_path = []          # the planned path of the yielding robot when it stopped


class Coordinator(Node):

    def __init__(self):
        super().__init__('fleet_coordinator')

        def p(name, default):
            return self.declare_parameter(name, default).value
        self.mission_cfg = load_mission(p('mission_file', '') or default_mission_path())
        ids = list(p('robots', ['forklift_1', 'forklift_2']))
        self.cfg = {k: p(k, d) for k, d in (
            ('heartbeat_timeout', 5.0), ('ack_timeout', 8.0), ('ack_retries', 2),
            ('comm_cooldown', 30.0), ('max_task_attempts', 2), ('mission_timeout', 3000.0),
            ('zone_radius', 2.0), ('reservation_ttl', 20.0), ('zone_approach_margin', 3.5),
            ('path_clearance', 2.2), ('path_lookahead', 5.0),
            ('conflict_check_distance', 9.0), ('resume_distance', 3.0),
            ('deadlock_timeout', 40.0), ('deadlock_window', 15.0),
            ('deadlock_min_progress', 0.3), ('backoff_distance', 1.0),
            ('escalate_after', 180.0), ('rear_conflict_range', 3.0),
            ('rear_wait_before_move', 8.0), ('home_tolerance', 0.25),
            ('home_attempts', 3))}
        self.alloc_cfg = AllocatorConfig(**{k: float(p(k, getattr(AllocatorConfig(), k)))
                                            for k in AllocatorConfig.__dataclass_fields__})
        self.robots = {}
        for rid in ids:
            home = p(f'home.{rid}', [0.0, 0.0, 0.0])
            self.robots[rid] = Robot(self, rid, tuple(float(v) for v in home))
        self.autostart = p('autostart', False)
        self.record_dir = os.path.expanduser(p('record_dir', '~/.ros/forklift_missions'))

        self.registry = None
        self.reservations = ReservationTable(self.cfg['reservation_ttl'])
        self.conflicts = {}
        self.conflict_count = 0
        self.events = []
        self.fleet_state = 'IDLE'
        self.failure_reason = ''
        self._tick = Future()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_subs = {rid: self.create_subscription(
            RobotStatus, f'/{rid}/fleet/robot_status', self._on_status, 10) for rid in ids}
        self.create_timer(10.0, self._status_watchdog)
        self.status_pub = self.create_publisher(FleetStatus, '/fleet/status', latched)
        self.conflict_pub = self.create_publisher(ConflictEvent, '/fleet/conflicts', 10)
        self.create_service(ClaimTask, '/fleet/claim_task', self._on_claim)
        ActionServer(self, RunMission, '/fleet/run', execute_callback=self._on_run,
                     goal_callback=lambda _g: (GoalResponse.REJECT
                                               if self.fleet_state == 'RUNNING'
                                               else GoalResponse.ACCEPT),
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_timer(0.05, self._on_tick)
        self.create_timer(1.0, self.publish_status)
        if self.autostart:
            self._auto = self.create_timer(2.0, self._start_once)
        self.zones = self._zones()
        self.get_logger().info(f'fleet coordinator ready: robots {", ".join(ids)}; '
                               f'{len(self.mission_cfg.boxes)} boxes; action /fleet/run')

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

    async def wait(self, future, timeout):
        end = self.now() + timeout
        while not future.done() and self.now() < end:
            await self.tick()
        return future.result() if future.done() else None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _start_once(self):
        self._auto.cancel()
        if self.fleet_state != 'RUNNING':
            self.executor.create_task(self.run_fleet([], None))

    def _status_watchdog(self):
        """Subscribe again to robots that sent no data (one time, a DDS match did not occur)."""
        for rid, r in self.robots.items():
            if r.heartbeat is None:
                self.get_logger().warn(f'no status from {rid} yet: re-subscribing')
                self.destroy_subscription(self.status_subs[rid])
                self.status_subs[rid] = self.create_subscription(
                    RobotStatus, f'/{rid}/fleet/robot_status', self._on_status, 10)

    def _on_status(self, msg):
        r = self.robots.get(msg.robot_id)
        if r is not None:
            r.status, r.heartbeat = msg, self.now()

    def _zones(self):
        """Place id -> (x, y, radius): around the point between its staging pose and its load."""
        m, z = self.mission_cfg, {}
        for pid, pose in m.pickups.items():
            s = m.staging_pose(pose)
            z[pid] = ((s.x + pose.x) / 2, (s.y + pose.y) / 2, self.cfg['zone_radius'])
        for did, d in m.destinations.items():
            s = m.staging_pose(d.pose)
            z[did] = ((s.x + d.pose.x) / 2, (s.y + d.pose.y) / 2, self.cfg['zone_radius'])
        for rid, r in self.robots.items():
            z[f'HOME_{rid}'] = (r.home[0], r.home[1], 1.5)
        return z

    def event(self, text, warn=False):
        self.events.append((round(self.now(), 1), text))
        if warn:                       # one call site for each severity (rclpy requirement)
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    # ------------------------------------------------------------------ claims

    def _on_claim(self, req, res):
        if self.registry is None:
            res.granted, res.reason = False, 'no fleet mission running'
            return res
        r = self.robots.get(req.robot_id)
        if r is None or not r.available:
            res.granted, res.reason = False, f'{req.robot_id} is not an available robot'
            return res
        granted, t, reason = self.registry.claim(req.box_id, req.robot_id)
        res.granted, res.reason = granted, reason
        if t is not None:
            res.task_id, res.assignment_id, res.owner = t.task_id, t.assignment_id, t.owner
        self.event(f'claim {req.box_id} by {req.robot_id}: '
                   f'{"granted" if granted else "refused"} ({reason})', warn=not granted)
        if not granted and t is not None and t.owner and t.owner != req.robot_id:
            self.report_conflict('TASK_CLAIM', [t.owner, req.robot_id], t.box_id, t.owner,
                                 req.robot_id, 'authoritative registry: one owner per box',
                                 'RESOLVED', True)
        if granted and not r.mission and not r.dispatching:
            self.executor.create_task(self.dispatch(r, t, claimed=True))
        return res

    # ------------------------------------------------------------------ main loop

    async def _on_run(self, goal_handle):
        return await self.run_fleet(list(goal_handle.request.box_ids), goal_handle)

    async def run_fleet(self, box_ids, goal_handle):
        m = self.mission_cfg
        boxes = [b for b in m.boxes if not box_ids or b.box_id in box_ids]
        self.registry = TaskRegistry([(b.box_id, b.pickup, b.destination) for b in boxes],
                                     self.cfg['max_task_attempts'], clock=self.now)
        self.fleet_state, self.failure_reason = 'RUNNING', ''
        started = self.now()
        self.event(f'fleet mission started: {", ".join(b.box_id for b in boxes)}')
        while True:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                self.fleet_state, self.failure_reason = 'CANCELED', 'cancelled by operator'
                for r in self.robots.values():
                    if r.mission is not None:
                        await r.mission[1].cancel_goal_async()
                break
            now = self.now()
            self.update_availability(now)
            self.track_tasks()
            await self.manage_zones(now)
            await self.detect_conflicts(now)
            await self.resolve_conflicts(now)
            self.dispatch_idle(now)
            if self.registry.finished() and self.everyone_home():
                self.fleet_state = ('MISSION_COMPLETE' if self.registry.all_delivered()
                                    else 'MISSION_FAILED')
                if self.fleet_state == 'MISSION_FAILED':
                    self.failure_reason = '; '.join(
                        f'{t.box_id}: {t.state} ({t.failure_reason})'
                        for t in self.registry.tasks.values() if t.state != DELIVERED)
                break
            if now - started > self.cfg['mission_timeout']:
                self.fleet_state, self.failure_reason = 'MISSION_FAILED', 'mission timeout'
                break
            if goal_handle is not None:
                done = sum(1 for t in self.registry.tasks.values() if t.state == DELIVERED)
                goal_handle.publish_feedback(RunMission.Feedback(
                    state=self.fleet_state, boxes_delivered_count=done,
                    boxes_total=len(self.registry.tasks)))
            self.publish_status()
            await self.sleep(0.5)
        delivered = [t.box_id for t in self.registry.tasks.values() if t.state == DELIVERED]
        failed = [t.box_id for t in self.registry.tasks.values() if t.state != DELIVERED]
        msg = (f'{self.fleet_state}: delivered {len(delivered)}/{len(self.registry.tasks)} '
               f'({", ".join(delivered) or "none"}) in {self.now() - started:.0f} s; '
               f'{self.conflict_count} conflicts')
        if self.failure_reason:
            msg += f'; {self.failure_reason}'
        self.event(msg, warn=self.fleet_state != 'MISSION_COMPLETE')
        self.publish_status()
        self.write_record(started)
        result = RunMission.Result(success=self.fleet_state == 'MISSION_COMPLETE',
                                   boxes_delivered=delivered, boxes_failed=failed, message=msg)
        if goal_handle is not None:
            if self.fleet_state == 'CANCELED':
                goal_handle.canceled()
            elif result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        return result

    # ------------------------------------------------------------------ availability

    def update_availability(self, now):
        for r in self.robots.values():
            fresh = r.heartbeat is not None and now - r.heartbeat <= self.cfg['heartbeat_timeout']
            ok = fresh and r.status.localized and r.status.state != 'FAULT' and \
                now >= r.benched_until
            if ok and r.lost and r.mission is not None:
                # The robot is back after it was declared lost. Its
                # task can have a different robot now.
                self.event(f'{r.id} is back; cancelling its stale mission goal', warn=True)
                self.executor.create_task(self._cancel_mission(r))
                ok = False
            if r.available and not ok:
                why = ('heartbeat lost' if not fresh else
                       'not localized' if not r.status.localized else
                       'FAULT' if r.status.state == 'FAULT' else 'benched')
                self.event(f'{r.id} unavailable ({why})', warn=True)
                if not fresh:
                    r.lost = True
                    self._orphan_tasks(r, why)
            if ok and not r.available:
                self.event(f'{r.id} available')
                r.lost = False
            r.available = ok

    async def _cancel_mission(self, r):
        _task, handle = r.mission
        await handle.cancel_goal_async()

    def _orphan_tasks(self, r, why):
        for t in self.registry.owned_by(r.id) if self.registry else []:
            # From the start of the pickup, the forks can be in (or below) the pallet.
            if t.state in (PICKUP_IN_PROGRESS, PICKED_UP, DELIVERY_IN_PROGRESS):
                self.registry.hold(t.task_id, f'{r.id} {why} with the load state unknown')
                self.event(f'{t.task_id} HELD: {r.id} {why} during {t.state}; not reassigned '
                           'until the load is checked', warn=True)
            else:
                self.registry.release(t.task_id, r.id, f'{r.id} {why} before pickup')
                self.event(f'{t.task_id} released for reassignment ({r.id} {why})', warn=True)

    # ------------------------------------------------------------------ task tracking

    def track_tasks(self):
        for r in self.robots.values():
            s = r.status
            if s is None or self.registry is None:
                continue
            for t in self.registry.owned_by(r.id):
                try:
                    if t.box_id in s.boxes_delivered:
                        self.registry.advance(t.task_id, DELIVERED, r.id,
                                              'placement verified by the robot')
                        self.event(f'{t.box_id} DELIVERED by {r.id} '
                                   f'(QR destination {t.qr_destination_id or "?"})')
                    elif t.box_id in s.boxes_picked:
                        new = (DELIVERY_IN_PROGRESS if s.state in (
                            'NAVIGATING_TO_DESTINATION', 'PLACING') else PICKED_UP)
                        if t.state != new and not (t.state == DELIVERY_IN_PROGRESS):
                            self.registry.advance(t.task_id, new, r.id, s.mission_state)
                    elif s.state == 'PICKING' and s.current_box_id == t.box_id:
                        self.registry.advance(t.task_id, PICKUP_IN_PROGRESS, r.id,
                                              s.mission_state)
                except TransitionError as e:
                    self.get_logger().error(f'task tracking: {e}')
                if s.current_box_id == t.box_id and s.current_destination_id:
                    t.qr_destination_id = s.current_destination_id

    def _on_mission_result(self, r, task_id, future):
        r.mission = None
        res = future.result()
        result = res.result
        t = self.registry.tasks[task_id]
        if t.owner != r.id:
            return                              # already released or given to a different robot
        if t.box_id in result.boxes_delivered:
            if t.state != DELIVERED:
                self.registry.advance(task_id, DELIVERED, r.id, 'mission result')
                self.event(f'{t.box_id} DELIVERED by {r.id}')
            return
        r.failures += 1
        carrying = r.status is not None and r.status.carrying
        if t.state in (PICKED_UP, DELIVERY_IN_PROGRESS) or carrying:
            self.registry.hold(task_id, f'{r.id}: {result.message}')
            self.event(f'{task_id} HELD after {r.id} failed with the load on its forks: '
                       f'{result.message}', warn=True)
        elif t.state in ACTIVE:
            self.registry.release(task_id, r.id, result.message)
            self.event(f'{task_id} released after {r.id} failed before lifting: '
                       f'{result.message}', warn=True)

    # ------------------------------------------------------------------ allocation

    def _place_xy(self, place_id):
        m = self.mission_cfg
        if place_id in m.pickups:
            s = m.staging_pose(m.pickups[place_id])
        else:
            s = m.destination_staging(place_id)
        return (s.x, s.y)

    def dispatch_idle(self, now):
        if self.registry is None:
            return
        avail = [r for r in self.robots.values() if r.available]
        idle = [r for r in avail if self.is_idle(r)]
        unassigned = self.registry.unassigned()
        if idle and unassigned:
            views = []
            for r in avail:
                v = RobotView(r.id, *r.pose, failures=r.failures)
                owned = self.registry.owned_by(r.id)
                if owned:
                    t = owned[0]
                    drop = self._place_xy(t.qr_destination_id or t.destination_id)
                    v.ready_x, v.ready_y = drop
                    v.remaining = straight_route(self.alloc_cfg)('', r.pose, drop)
                    v.targets = [drop]
                views.append(v)
            tasks = [TaskView(t.task_id, self._place_xy(t.pickup_id),
                              self._place_xy(t.destination_id), t.reserved_for)
                     for t in unassigned]
            assignment, makespan, total = plan(views, tasks, cfg=self.alloc_cfg)
            for rid, seq in assignment.items():
                for tv in seq:
                    self.registry.reserve(tv.task_id, rid)
            text = '; '.join(f'{rid}: {", ".join(t.task_id for t in seq) or "-"}'
                             for rid, seq in assignment.items())
            if text != getattr(self, '_last_plan', ''):
                where = ', '.join(f'{v.robot_id} at ({v.x:.1f}, {v.y:.1f})' for v in views)
                self.event(f'allocation: {text} (makespan {makespan:.1f} m, total '
                           f'{total:.1f} m; {where})')
                self._last_plan = text
            for r in idle:
                seq = assignment.get(r.id, [])
                if seq:
                    granted, t, reason = self.registry.claim(
                        self.registry.tasks[seq[0].task_id].box_id, r.id)
                    if granted:
                        r.dispatching = True
                        r.home_failures = 0
                        self.executor.create_task(self.dispatch(r, t))
        for r in idle:
            # The robot is idle and has no more tasks: park at home, out of the path of all robots.
            # If a home navigation fails, try again (home_attempts) until the robot is at home.
            if not r.dispatching and not r.mission and r.nav is None and not r.yields and \
                    not any(t.reserved_for == r.id for t in self.registry.unassigned()) and \
                    not self.home(r) and r.home_failures < self.cfg['home_attempts']:
                self.send_nav(r, 'home', r.home)

    def is_idle(self, r):
        s = r.status
        return (r.available and not r.dispatching and r.mission is None and not r.yields
                and s is not None and not s.mission_running and
                not self.registry.owned_by(r.id))

    async def dispatch(self, r, t, claimed=False):
        """Offer the task to the robot. An accepted goal is the acknowledgement."""
        r.dispatching = True
        try:
            # for example, the robot goes home: stop that first
            if r.nav is not None:
                await r.nav[2].cancel_goal_async()
                r.nav = None
            for attempt in range(self.cfg['ack_retries'] + 1):
                if not r.run.wait_for_server(timeout_sec=0.0):
                    await self.sleep(min(2.0, self.cfg['ack_timeout']))
                    if not r.run.server_is_ready():
                        continue
                goal = RunMission.Goal(box_ids=[t.box_id], skip_return_home=True)
                send = r.run.send_goal_async(goal)
                handle = await self.wait(send, self.cfg['ack_timeout'])
                if handle is None:
                    # A late acknowledgement must not start a second
                    # owner: cancel it when it arrives.
                    send.add_done_callback(lambda f: f.result() and f.result().accepted and
                                           f.result().cancel_goal_async())
                    self.event(f'{t.task_id} offer to {r.id}: no acknowledgement '
                               f'(attempt {attempt + 1})', warn=True)
                    continue
                if not handle.accepted:
                    self.event(f'{t.task_id} rejected by {r.id} (attempt {attempt + 1})',
                               warn=True)
                    await self.sleep(1.0)
                    continue
                r.mission = (t.task_id, handle)
                handle.get_result_async().add_done_callback(
                    lambda f, r=r, tid=t.task_id: self._on_mission_result(r, tid, f))
                self.event(f'{t.task_id} ({t.box_id}) assigned to {r.id}, acknowledged '
                           f'[{t.assignment_id}]')
                return
            self.registry.release(t.task_id, r.id, 'assignment not acknowledged')
            r.benched_until = self.now() + self.cfg['comm_cooldown']
            r.failures += 1
            self.event(f'{r.id} did not acknowledge {t.task_id}: task released, robot benched '
                       f'for {self.cfg["comm_cooldown"]:.0f} s', warn=True)
        finally:
            r.dispatching = False

    # ------------------------------------------------------------------ navigation helpers

    def send_nav(self, r, purpose, target):
        goal = NavigateToPose.Goal(pose=PoseStamped())
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[0], target[1]
        goal.pose.pose.orientation.z = math.sin(target[2] / 2)
        goal.pose.pose.orientation.w = math.cos(target[2] / 2)
        r.nav = (purpose, target, None)

        def accepted(f):
            h = f.result()
            if h is None or not h.accepted:
                r.nav = None
                return
            if r.nav is None or r.nav[1] != target:
                h.cancel_goal_async()
                return
            r.nav = (purpose, target, h)
            h.get_result_async().add_done_callback(
                lambda f: self._nav_done(r, purpose, target, f))
        if not r.nav_client.server_is_ready():
            r.nav = None
            return
        r.nav_client.send_goal_async(goal).add_done_callback(accepted)
        self.event(f'{r.id}: navigating to {purpose} ({target[0]:.1f}, {target[1]:.1f})')

    def _nav_done(self, r, purpose, target, future):
        if r.nav is None or r.nav[1] != target:
            return                      # replaced (for example, cancelled for a yield)
        r.nav = None
        status = future.result().status if future.result() is not None else None
        if purpose != 'home':
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            r.home_failures = 0
        elif status != GoalStatus.STATUS_CANCELED:
            r.home_failures += 1
            pose = f'({r.pose[0]:.2f}, {r.pose[1]:.2f})' if r.pose else 'unknown'
            self.event(f'{r.id}: home navigation ended with status {status} at {pose} '
                       f'(attempt {r.home_failures}/{self.cfg["home_attempts"]})', warn=True)

    def home(self, r):
        """At home from the estimate of the robot (Nav2 is successful in 8 cm / 3.4 deg)."""
        return r.at(r.home, self.cfg['home_tolerance'])

    def everyone_home(self):
        # If a robot has no more home attempts, it does not block the completion. The
        # warnings above stay in the record.
        live = [r for r in self.robots.values() if r.available]
        return all((self.home(r) or r.home_failures >= self.cfg['home_attempts']) and
                   r.nav is None and r.mission is None and not r.dispatching for r in live)

    # ------------------------------------------------------------------ yielding

    async def call(self, client, timeout=5.0):
        if not client.wait_for_service(timeout_sec=0.0):
            return None
        return await self.wait(client.call_async(Trigger.Request()), timeout)

    async def yield_robot(self, r, cid):
        first = not r.yields
        r.yields.add(cid)
        if not first:
            return
        if r.status is not None and r.status.mission_running:
            res = await self.call(r.pause_cli)
            r.paused = bool(res and res.success)
            self.event(f'{r.id} yields ({cid}): mission pause -> '
                       f'{res.message if res else "no response"}')
        elif r.nav is not None:
            purpose, target, h = r.nav
            r.nav, r.nav_resend = None, (purpose, target)
            if h is not None:
                await h.cancel_goal_async()
            self.event(f'{r.id} yields ({cid}): {purpose} navigation cancelled')

    async def resume_robot(self, r, cid):
        r.yields.discard(cid)
        if r.yields:
            return
        if r.paused:
            res = await self.call(r.resume_cli)
            r.paused = not (res and res.success)
            self.event(f'{r.id} resumes: {res.message if res else "no response"}')
        if r.nav_resend is not None:
            purpose, target = r.nav_resend
            r.nav_resend = None
            self.send_nav(r, purpose, target)

    # ------------------------------------------------------------------ zones

    def target_zone(self, r):
        s = r.status
        if s is None:
            return ''
        owned = self.registry.owned_by(r.id) if self.registry else []
        if owned:
            t = owned[0]
            if s.state in ('NAVIGATING_TO_DESTINATION', 'PLACING') or t.state in (
                    PICKED_UP, DELIVERY_IN_PROGRESS):
                return t.qr_destination_id or t.destination_id
            return t.pickup_id
        if (r.nav is not None and r.nav[0] == 'home') or r.nav_resend:
            return f'HOME_{r.id}'
        return ''

    async def manage_zones(self, now):
        self.reservations.expire(now)
        for r in self.robots.values():
            if r.status is None:
                continue
            target = self.target_zone(r) if r.available else ''
            for z in self.reservations.held_by(r.id, now):
                inside = in_zone(r.pose, self.zones[z]) if z in self.zones else False
                if z == target or inside:
                    self.reservations.grant(z, r.id, now)            # renew
                else:
                    self.reservations.release(z, r.id)
            if not target or target not in self.zones:
                continue
            zone = self.zones[target]
            granted, holder = self.reservations.grant(target, r.id, now, 'enter')
            occupant = next((o.id for o in self.robots.values() if o.id != r.id and
                             o.pose is not None and in_zone(o.pose, zone)), '')
            cid = f'ZONE:{target}:{r.id}'
            blocked = (not granted) or bool(occupant)
            if granted and occupant:
                self.reservations.release(target, r.id)
            near = math.dist(r.pose, zone[:2]) <= zone[2] + self.cfg['zone_approach_margin']
            if blocked and near and cid not in self.conflicts:
                other = holder if not granted else occupant
                c = Conflict(cid, 'ZONE', other, r.id, target, now,
                             f'{target} reserved by / occupied by {other}', DeadlockMonitor())
                self.conflicts[cid] = c
                self.conflict_count += 1
                self.publish_conflict(c)
                await self.yield_robot(r, cid)
                c.applied = True
            elif blocked and cid in self.conflicts:
                await self.unblock_zone(self.conflicts[cid], holder if not granted else occupant)
            elif not blocked and cid in self.conflicts:
                c = self.conflicts.pop(cid)
                c.status = 'RESOLVED'
                c.recovery = (c.recovery + '; ' if c.recovery else '') + \
                    f'{target} granted to {r.id}'
                await self.resume_robot(r, cid)
                self.publish_conflict(c, success=True)

    async def unblock_zone(self, c, other_id):
        """Make sure that the robot in the path really goes out of the zone."""
        o = self.robots.get(other_id)
        if o is None or o.status is None or c.moved_away:
            return
        # Cyclic wait: the robot in the zone yields to the robot that waits for this zone.
        for cid in list(o.yields):
            oc = self.conflicts.get(cid)
            if oc is not None and oc.priority == c.yielder:
                c.moved_away = True
                c.recovery = f'cyclic wait broken: {o.id} may leave the zone first'
                self.event(f'{c.id}: {c.recovery}', warn=True)
                oc.status, oc.recovery = 'RESOLVED', f'released to clear {c.resource}'
                self.conflicts.pop(cid, None)
                await self.resume_robot(o, cid)
                self.publish_conflict(oc, success=True)
                return
        # The robot is parked in the zone of a different robot and has no task: send it home.
        idle = (not o.status.mission_running and o.nav is None and not o.yields and
                o.mission is None and not o.dispatching)
        if idle and not self.home(o):
            c.moved_away = True
            c.recovery = f'{o.id} idle in {c.resource}: sent home'
            self.event(f'{c.id}: {c.recovery}')
            self.send_nav(o, 'home', o.home)

    # ------------------------------------------------------------------ path / rear conflicts

    def carrying(self, r):
        """
        Load on the forks from the task lifecycle (picked up, not delivered yet).

        Do not use the load_detected value of the lift (from the effort). During a head-on
        meeting in Gazebo, it was true for a short time on an empty robot. The right of way
        must not depend on a temporary value.
        """
        owned = self.registry.owned_by(r.id) if self.registry else []
        return any(t.state in (PICKED_UP, DELIVERY_IN_PROGRESS) for t in owned)

    def contender(self, r, other):
        zone = self.target_zone(r)
        inside = (zone in self.zones and self.reservations.holder(zone, self.now()) == r.id
                  and in_zone(r.pose, self.zones[zone]))
        theirs = self.target_zone(other)
        blocking = theirs in self.zones and in_zone(r.pose, self.zones[theirs])
        return Contender(r.id, carrying=self.carrying(r), inside_own_zone=inside,
                         moving=r.status.speed > 0.05 and not r.yields,
                         in_others_target=blocking)

    async def detect_conflicts(self, now):
        live = [r for r in self.robots.values() if r.available and r.status is not None]
        for i, a in enumerate(live):
            for b in live[i + 1:]:
                if any(set(c.id.split(':')[1:]) >= {a.id, b.id}
                       for c in self.conflicts.values() if c.kind in ('PATH', 'HEAD_ON')):
                    continue
                if math.dist(a.pose, b.pose) > self.cfg['conflict_check_distance']:
                    continue
                sa, sb = a.status, b.status
                if not (sa.nav_active and sb.nav_active):
                    continue                      # Nav2 itself goes around a stationary robot
                pa = [(p.x, p.y) for p in sa.path_ahead]
                pb = [(p.x, p.y) for p in sb.path_ahead]
                c1 = path_conflict(a.pose, pa, b.pose, pb, self.cfg['path_clearance'],
                                   self.cfg['path_lookahead'])
                c2 = path_conflict(b.pose, pb, a.pose, pa, self.cfg['path_clearance'],
                                   self.cfg['path_lookahead'])
                if not (c1[0] or c2[0]):
                    continue
                head_on = abs(math.cos(sa.yaw - sb.yaw) + 1.0) < 0.3
                pr, yl, rule = choose_priority(self.contender(a, b), self.contender(b, a))
                cid = f'PATH:{pr}:{yl}'
                kind = 'HEAD_ON' if head_on else 'PATH'
                point = c1[2] or c2[2]
                c = Conflict(cid, kind, pr, yl,
                             f'({point[0]:.1f}, {point[1]:.1f})' if point else '',
                             now, rule, DeadlockMonitor(self.cfg['deadlock_timeout'],
                                                        self.cfg['deadlock_window'],
                                                        self.cfg['deadlock_min_progress']))
                c.monitor.start(now)
                c.yielder_path = pb if yl == b.id else pa
                self.conflicts[cid] = c
                self.conflict_count += 1
                self.publish_conflict(c)
                await self.yield_robot(self.robots[yl], cid)
                c.applied = True
        # A different robot blocks the rear.
        for r in live:
            s = r.status
            if s.rear_state != 'STOP' or not (s.mission_running or s.nav_active):
                continue
            c, si = math.cos(s.yaw), math.sin(s.yaw)
            for o in live:
                if o.id == r.id:
                    continue
                dx, dy = o.pose[0] - s.x, o.pose[1] - s.y
                along, side = c * dx + si * dy, -si * dx + c * dy
                cid = f'REAR:{r.id}:{o.id}'
                if along < -0.5 and along > -(1.25 + self.cfg['rear_conflict_range'] + 1.2) \
                        and abs(side) < 1.8 and cid not in self.conflicts:
                    cf = Conflict(cid, 'REAR_BLOCKED', r.id, o.id, f'behind {r.id}', now,
                                  f'{r.id} is reversing and its rear LiDAR sees {o.id}',
                                  DeadlockMonitor())
                    self.conflicts[cid] = cf
                    self.conflict_count += 1
                    self.publish_conflict(cf)
                    await self.yield_robot(o, cid)
                    cf.applied = True

    async def resolve_conflicts(self, now):
        for cid, c in list(self.conflicts.items()):
            if c.kind == 'ZONE':
                if now - c.start > self.cfg['escalate_after'] and c.status == 'ACTIVE':
                    c.status = 'ESCALATED'
                    self.publish_conflict(c)
                continue
            pr, yl = self.robots[c.priority], self.robots[c.yielder]
            if pr.status is None or yl.status is None or not c.applied:
                continue
            if c.kind == 'REAR_BLOCKED':
                done = pr.status.rear_state != 'STOP' or not pr.available
                if done:
                    c.status, c.recovery = 'RESOLVED', 'rear path clear'
                    self.conflicts.pop(cid)
                    await self.resume_robot(yl, cid)
                    self.publish_conflict(c, success=True)
                elif now - c.start > self.cfg['rear_wait_before_move'] and not c.moved_away:
                    c.moved_away = True
                    c.recovery = f'{yl.id} sent to its waiting location (home)'
                    self.event(f'{cid}: {c.recovery}')
                    yl.nav_resend = None
                    self.send_nav(yl, 'waiting location', yl.home)
                    self.publish_conflict(c)
                continue
            c.monitor.update(now, pr.pose)
            pa = [(p.x, p.y) for p in pr.status.path_ahead]
            # The same test as the detection: the path in front of the priority robot against the
            # position of the yielding robot and the route on which it stopped.
            still = path_conflict(pr.pose, pa, yl.pose, c.yielder_path,
                                  self.cfg['path_clearance'], self.cfg['path_lookahead'])[0]
            apart = math.dist(pr.pose, yl.pose) > self.cfg['resume_distance']
            arrived = not pr.status.nav_active and pr.nav is None and not pr.yields and \
                not pr.status.mission_running
            if not pr.available or arrived or (pa and not still and apart):
                c.status = 'RESOLVED'
                c.recovery = c.recovery or 'priority robot passed'
                self.conflicts.pop(cid)
                await self.resume_robot(yl, cid)
                self.publish_conflict(c, success=True)
                continue
            if c.monitor.deadlocked(now):
                if not c.backed_up and yl.status.rear_state in ('CLEAR', 'WARN'):
                    c.backed_up = True
                    c.recovery = (f'deadlock: {yl.id} backs up '
                                  f'{self.cfg["backoff_distance"]:.1f} m (rear clear)')
                    self.event(f'{cid}: {c.recovery}', warn=True)
                    self.send_backup(yl)
                elif not c.flipped:
                    c.flipped = True
                    c.recovery = (f'deadlock: right of way swapped to {yl.id} '
                                  f'({yl.id} rear {yl.status.rear_state})')
                    self.event(f'{cid}: {c.recovery}', warn=True)
                    c.priority, c.yielder = yl.id, pr.id
                    c.yielder_path = pa
                    c.resolution = f'{pr.id} waits; {yl.id} has right of way (deadlock swap)'
                    await self.yield_robot(pr, cid)
                    await self.resume_robot(yl, cid)
                elif c.status == 'ACTIVE':
                    c.status = 'ESCALATED'
                    c.recovery += '; unresolved: operator attention'
                    self.event(f'{cid}: ESCALATED', warn=True)
                c.monitor.start(now)
                c.kind = 'DEADLOCK'
                self.publish_conflict(c)
            elif now - c.start > self.cfg['escalate_after'] and c.status == 'ACTIVE':
                c.status = 'ESCALATED'
                self.publish_conflict(c)

    def send_backup(self, r):
        if not r.backup.server_is_ready():
            self.event(f'{r.id}: backup action unavailable', warn=True)
            return
        goal = BackUp.Goal()
        goal.target.x = -abs(self.cfg['backoff_distance'])
        goal.speed = 0.1
        goal.time_allowance.sec = 30
        r.backup.send_goal_async(goal)

    # ------------------------------------------------------------------ reporting

    def report_conflict(self, kind, robots, resource, priority, yielder, resolution, status,
                        ok):
        self.conflict_count += 1
        c = Conflict(f'{kind}:{":".join(robots)}', kind, priority, yielder, resource,
                     self.now(), resolution, None)
        c.status, c.resolution = status, resolution
        self.publish_conflict(c, success=ok)

    def publish_conflict(self, c, success=False):
        e = ConflictEvent()
        e.header.stamp = self.get_clock().now().to_msg()
        e.conflict_id, e.type = c.id, c.kind
        e.robots = [c.priority, c.yielder]
        e.tasks = [(self.registry.owned_by(rid)[0].task_id
                    if self.registry and self.registry.owned_by(rid) else '')
                   for rid in e.robots]
        e.resource, e.priority_robot, e.yielding_robot = c.resource, c.priority, c.yielder
        e.resolution = c.resolution + (f'; {c.recovery}' if c.recovery else '')
        e.status = c.status
        e.wait_time = float(self.now() - c.start)
        e.recovery_succeeded = success
        self.conflict_pub.publish(e)
        self.events.append((round(self.now(), 1), f'conflict {e.type} {e.status}: '
                            f'{e.resolution} [{e.resource}] waited {e.wait_time:.1f} s'))
        self.get_logger().info(f'conflict {e.conflict_id} {e.type} {e.status}: '
                               f'{e.resolution} (waited {e.wait_time:.1f} s)')

    def publish_status(self):
        msg = FleetStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.fleet_state
        now = self.now()
        if self.registry is not None:
            for tid in self.registry.order:
                t = self.registry.tasks[tid]
                msg.tasks.append(TaskState(
                    task_id=t.task_id, box_id=t.box_id, pickup_id=t.pickup_id,
                    destination_id=t.destination_id, qr_destination_id=t.qr_destination_id,
                    state=t.state, owner=t.owner, reserved_for=t.reserved_for,
                    assignment_id=t.assignment_id, attempts=t.attempts,
                    last_update=t.last_update, last_confirmed=t.last_confirmed,
                    failure_reason=t.failure_reason))
        msg.robots = [r.status for r in self.robots.values() if r.status is not None]
        msg.available_robots = [r.id for r in self.robots.values() if r.available]
        msg.reservations = [Reservation(resource_id=x.resource, owner=x.owner, stamp=x.stamp,
                                        expires=x.expires, direction=x.direction)
                            for x in self.reservations.items.values() if x.expires > now]
        msg.conflicts_detected = self.conflict_count
        msg.conflicts_resolved = sum(1 for _, e in self.events if 'RESOLVED' in e)
        msg.failure_reason = self.failure_reason
        self.status_pub.publish(msg)

    def write_record(self, started):
        data = {'final_state': self.fleet_state, 'started': started, 'ended': self.now(),
                'failure_reason': self.failure_reason,
                'tasks': {t.task_id: {'box': t.box_id, 'state': t.state, 'owner': t.owner,
                                      'qr_destination': t.qr_destination_id,
                                      'attempts': t.attempts,
                                      'last_confirmed': t.last_confirmed}
                          for t in self.registry.tasks.values()},
                'task_log': [{'t': round(a, 1), 'task': b, 'from': c, 'to': d, 'robot': e,
                              'reason': f} for a, b, c, d, e, f in self.registry.log],
                'events': [{'t': t, 'event': e} for t, e in self.events],
                'conflicts_detected': self.conflict_count}
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            path = os.path.join(self.record_dir, datetime.datetime.now().strftime(
                'fleet_%Y%m%d_%H%M%S.json'))
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'fleet record written to {path}')
        except OSError as e:
            self.get_logger().warn(f'could not write the fleet record: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
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
