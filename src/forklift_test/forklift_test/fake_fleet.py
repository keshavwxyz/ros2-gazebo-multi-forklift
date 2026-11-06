"""
Fake forklifts for the coordinator integration tests (no simulator).

Each FakeRobot has the robot side of the fleet protocol, exactly as the real stack has it:
fleet/robot_status heartbeat, mission/run (RunMission), mission/pause and mission/resume
(Trigger), navigate_to_pose and backup (Nav2 actions). For a mission goal, the fake robot
moves along a straight line to the pickup staging pose and "picks" the box (boxes_picked,
carrying). Then it moves to the destination staging pose and "places" the box
(boxes_delivered). To cause faults, set attributes (heartbeat, reject_goals, freeze, ...).
"""

import math
import threading
import time

from forklift_interfaces.action import RunMission
from forklift_interfaces.msg import RobotStatus
from forklift_task_manager.mission_config import default_mission_path, load_mission
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp, NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

MISSION = load_mission(default_mission_path())


class FakeRobot:

    def __init__(self, node, robot_id, x, y, speed=2.0):
        self.node, self.id = node, robot_id
        self.x, self.y, self.yaw = x, y, 0.0
        # m/s of the fake robot (fast: thus, the tests are short)
        self.speed = speed
        self.state, self.mission_state = 'IDLE', 'IDLE'
        self.box, self.dest = '', ''
        self.carrying = self.paused = self.nav_active = False
        self.path = []
        self.picked, self.delivered = [], []
        self.goals = []                    # box lists of each accepted RunMission goal
        self.backups = 0
        self.pause_calls = self.resume_calls = 0
        self.heartbeat = True              # False: stop the heartbeat (robot lost)
        self.reject_goals = False
        self.ignore_goals = False          # never send a response (no acknowledgement)
        # stop the motion (the robot cannot move), but continue the heartbeat
        self.freeze = False
        self.lose_heartbeat_at = ''        # mission state at which the heartbeat stops
        self.rear_state = 'CLEAR'          # rear-LiDAR state that the robot reports
        self.lock = threading.Lock()
        cb = ReentrantCallbackGroup()
        ns = f'/{robot_id}'
        self.pub = node.create_publisher(RobotStatus, f'{ns}/fleet/robot_status', 10)
        node.create_timer(0.2, self.publish, callback_group=cb)
        ActionServer(node, RunMission, f'{ns}/mission/run', execute_callback=self._run,
                     goal_callback=self._goal, cancel_callback=lambda _g: CancelResponse.ACCEPT,
                     callback_group=cb)
        ActionServer(node, NavigateToPose, f'{ns}/navigate_to_pose', execute_callback=self._nav,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=cb)
        ActionServer(node, BackUp, f'{ns}/backup', execute_callback=self._backup,
                     callback_group=cb)
        node.create_service(Trigger, f'{ns}/mission/pause', self._pause, callback_group=cb)
        node.create_service(Trigger, f'{ns}/mission/resume', self._resume, callback_group=cb)

    # ------------------------------------------------------------------ status

    def publish(self):
        if not self.heartbeat:
            return
        s = RobotStatus()
        s.header.stamp = self.node.get_clock().now().to_msg()
        s.robot_id, s.state, s.mission_state = self.id, self.state, self.mission_state
        s.current_box_id, s.current_destination_id = self.box, self.dest
        s.localized, s.x, s.y, s.yaw = True, float(self.x), float(self.y), float(self.yaw)
        s.speed = 0.0 if (self.paused or self.freeze or not self.path) else 0.3
        s.carrying = self.carrying
        s.rear_state, s.rear_nearest = self.rear_state, float('inf')
        s.boxes_picked, s.boxes_delivered = list(self.picked), list(self.delivered)
        s.mission_running = self.mission_state not in (
            'IDLE', 'MISSION_COMPLETE', 'MISSION_FAILED')
        s.nav_active = self.nav_active
        s.path_ahead = [Point(x=float(px), y=float(py)) for px, py in self.path[:24]]
        self.pub.publish(s)

    # ------------------------------------------------------------------ motion

    def drive_to(self, tx, ty, goal_handle=None):
        """Move in a straight line. Publish the remaining line as the planned path."""
        self.nav_active = True
        try:
            while True:
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    return False
                d = math.hypot(tx - self.x, ty - self.y)
                n = max(1, int(d / 0.25))
                self.path = [(self.x + (tx - self.x) * i / n, self.y + (ty - self.y) * i / n)
                             for i in range(n + 1)]
                if d < 0.05:
                    return True
                if not (self.paused or self.freeze):
                    step = min(d, self.speed * 0.1)
                    self.yaw = math.atan2(ty - self.y, tx - self.x)
                    self.x += step * (tx - self.x) / d
                    self.y += step * (ty - self.y) / d
                time.sleep(0.1)
        finally:
            self.path, self.nav_active = [], False

    def wait(self, seconds, goal_handle):
        end = time.time() + seconds
        while time.time() < end or self.paused:
            if goal_handle.is_cancel_requested:
                return False
            time.sleep(0.05)
        return True

    def set_state(self, state, mission_state):
        self.state, self.mission_state = state, mission_state
        if self.lose_heartbeat_at and mission_state == self.lose_heartbeat_at:
            self.heartbeat = False

    # ------------------------------------------------------------------ servers

    def _goal(self, _goal):
        if self.ignore_goals:
            time.sleep(30.0)              # the acknowledgement is always too late
            return GoalResponse.REJECT
        busy = self.mission_state not in ('IDLE', 'MISSION_COMPLETE', 'MISSION_FAILED')
        return GoalResponse.REJECT if (self.reject_goals or busy) else GoalResponse.ACCEPT

    def _run(self, goal_handle):
        boxes = list(goal_handle.request.box_ids)
        self.goals.append(boxes)
        result = RunMission.Result()
        for box in boxes:
            b = MISSION.box(box)
            self.box, self.dest = box, ''
            self.set_state('NAVIGATING_TO_PICKUP', 'NAVIGATE_TO_PICKUP')
            p = MISSION.pickup_staging(box)
            if not self.drive_to(p.x, p.y, goal_handle) or not self.heartbeat:
                return self._abort(goal_handle, result)
            self.set_state('PICKING', 'ALIGN_FORKS')
            self.dest = b.destination                 # "decoded from the QR code"
            if not self.wait(0.5, goal_handle) or not self.heartbeat:
                return self._abort(goal_handle, result)
            self.carrying = True
            self.picked.append(box)
            self.set_state('NAVIGATING_TO_DESTINATION', 'NAVIGATE_TO_DESTINATION')
            d = MISSION.destination_staging(b.destination)
            if not self.drive_to(d.x, d.y, goal_handle) or not self.heartbeat:
                return self._abort(goal_handle, result)
            self.set_state('PLACING', 'PLACE_BOX')
            if not self.wait(0.5, goal_handle):
                return self._abort(goal_handle, result)
            self.carrying = False
            self.delivered.append(box)
        self.set_state('IDLE', 'MISSION_COMPLETE')
        self.box = self.dest = ''
        result.success = True
        result.boxes_delivered = boxes
        goal_handle.succeed()
        return result

    def _abort(self, goal_handle, result):
        self.set_state('FAULT' if self.carrying else 'IDLE', 'MISSION_FAILED')
        result.success = False
        result.message = 'fake failure'
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _nav(self, goal_handle):
        p = goal_handle.request.pose.pose.position
        ok = self.drive_to(p.x, p.y, goal_handle)
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.canceled()
        return NavigateToPose.Result()

    def _backup(self, goal_handle):
        self.backups += 1
        d = abs(goal_handle.request.target.x)
        self.x -= d * math.cos(self.yaw)
        self.y -= d * math.sin(self.yaw)
        goal_handle.succeed()
        return BackUp.Result()

    def _pause(self, _req, res):
        self.pause_calls += 1
        self.paused = True
        res.success, res.message = True, 'paused'
        return res

    def _resume(self, _req, res):
        self.resume_calls += 1
        self.paused = False
        res.success, res.message = True, 'resumed'
        return res
