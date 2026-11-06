"""
Tier-2 integration test: the real fleet coordinator node with two fake forklifts.

The fake robots (forklift_test/fake_fleet.py) have the robot side of the protocol. Thus,
the allocation, acknowledgement, availability, ownership and conflict handling of the
coordinator operate exactly as in Gazebo, but deterministic and fast. Scenarios and
expected results:

  1 nominal + duplicate claims  all 3 boxes delivered; two claims at the same time for one
                                box -> a maximum of one grant, and the coordinator offers the
                                box exactly one time; the robots go home
  2 robot lost before pickup    its task is released and the other robot delivers it
  3 robot lost during transport that task is HELD (never given to a different robot);
                                mission FAILED
  4 no acknowledgement          the robot that does not respond is out of service; the
                                other robot delivers all boxes
  5 head-on paths               one robot yields (navigation cancelled), the other robot
                                passes, the yielding robot continues; the two robots get home
  6 deadlock                    the priority robot cannot move -> the yielding robot moves
                                rearward (rear clear), then the right of way changes; the
                                two robots get home
  7 the other robot blocks a    a robot reports rear STOP during the mission with the other
    robot in reverse motion     robot immediately behind it: the robot behind yields, then
                                the coordinator sends it to its waiting location; the
                                conflict is solved when the rear is clear, and the mission
                                is complete
"""

import math
import tempfile
import threading
import time
import unittest

from forklift_interfaces.action import RunMission
from forklift_interfaces.msg import ConflictEvent, FleetStatus
from forklift_interfaces.srv import ClaimTask
from forklift_test.fake_fleet import FakeRobot
import launch
from launch.substitutions import PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

HOME = {'forklift_1': (-6.0, 0.0), 'forklift_2': (5.0, 0.0)}


@pytest.mark.launch_test
def generate_test_description():
    coordinator = launch_ros.actions.Node(
        package='forklift_fleet', executable='fleet_coordinator', output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('forklift_fleet'), 'config',
                                          'fleet.yaml']),
                    {'use_sim_time': False, 'heartbeat_timeout': 2.0, 'ack_timeout': 2.0,
                     'ack_retries': 1, 'comm_cooldown': 25.0, 'reservation_ttl': 5.0,
                     'deadlock_timeout': 6.0, 'deadlock_window': 3.0,
                     'record_dir': tempfile.mkdtemp(prefix='fleet_test_')}])
    return launch.LaunchDescription([coordinator, launch_testing.actions.ReadyToTest()])


class TestFleetCoordinator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fleet_tester')
        cls.robots = {rid: FakeRobot(cls.node, rid, *HOME[rid]) for rid in HOME}
        cls.conflicts, cls.status = [], None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(ConflictEvent, '/fleet/conflicts',
                                     lambda m: cls.conflicts.append(m), 50)
        cls.node.create_subscription(FleetStatus, '/fleet/status',
                                     lambda m: setattr(cls, 'status', m), latched)
        cls.fleet_run = ActionClient(cls.node, RunMission, '/fleet/run')
        cls.claim = cls.node.create_client(ClaimTask, '/fleet/claim_task')
        cls.executor = MultiThreadedExecutor(num_threads=8)
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()
        assert cls.fleet_run.wait_for_server(timeout_sec=30.0)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        for rid, r in self.robots.items():
            r.x, r.y = HOME[rid]
            r.heartbeat, r.reject_goals, r.ignore_goals, r.freeze = True, False, False, False
            r.lose_heartbeat_at = ''
            r.goals.clear()
            r.picked.clear()
            r.delivered.clear()
            r.carrying, r.paused = False, False
            r.state, r.mission_state = 'IDLE', 'IDLE'
            r.backups = 0
            r.rear_state = 'CLEAR'
        self.conflicts.clear()
        time.sleep(3.0)                    # heartbeats received, the robots are available again

    def place(self, rid, x, y):
        self.robots[rid].x, self.robots[rid].y = x, y

    def wait_until(self, predicate, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if predicate():
                return True
            time.sleep(0.1)
        return False

    def run_fleet(self, box_ids=(), timeout=180.0, during=None):
        send = self.fleet_run.send_goal_async(RunMission.Goal(box_ids=list(box_ids)))
        self.assertTrue(self.wait_until(send.done, 10.0))
        handle = send.result()
        self.assertTrue(handle.accepted)
        if during is not None:
            during()
        res = handle.get_result_async()
        self.assertTrue(self.wait_until(res.done, timeout), 'fleet mission did not end')
        return res.result().result

    def all_goals(self):
        return [b for r in self.robots.values() for g in r.goals for b in g]

    def test_1_nominal_mission_and_duplicate_claims(self):
        grants = []

        def concurrent_claims():
            futures = [self.claim.call_async(ClaimTask.Request(robot_id=rid, box_id='BOX_B'))
                       for rid in ('forklift_1', 'forklift_2')]
            self.assertTrue(self.wait_until(lambda: all(f.done() for f in futures), 10.0))
            grants.extend(f.result().granted for f in futures)
        result = self.run_fleet(during=concurrent_claims)
        self.assertTrue(result.success, result.message)
        self.assertEqual(sorted(result.boxes_delivered), ['BOX_A', 'BOX_B', 'BOX_C'])
        self.assertLessEqual(sum(grants), 1, 'two robots were granted the same box')
        goals = self.all_goals()
        self.assertEqual(sorted(goals), ['BOX_A', 'BOX_B', 'BOX_C'], 'a box offered twice')
        # The allocation uses the position: each robot starts with the box on its side.
        self.assertEqual(self.robots['forklift_1'].goals[0], ['BOX_A'])
        self.assertEqual(self.robots['forklift_2'].goals[0], ['BOX_C'])
        for rid, r in self.robots.items():
            self.assertLess(abs(r.x - HOME[rid][0]) + abs(r.y - HOME[rid][1]), 1.0,
                            f'{rid} not home')
        self.assertEqual(self.status.state, 'MISSION_COMPLETE')
        self.assertTrue(all(t.state == 'DELIVERED' for t in self.status.tasks))

    def test_2_robot_lost_before_pickup_task_is_reassigned(self):
        self.robots['forklift_2'].lose_heartbeat_at = 'NAVIGATE_TO_PICKUP'
        result = self.run_fleet()
        self.assertTrue(result.success, result.message)
        self.assertEqual(sorted(self.robots['forklift_1'].delivered),
                         ['BOX_A', 'BOX_B', 'BOX_C'])
        self.assertEqual(self.robots['forklift_2'].delivered, [])

    def test_3_robot_lost_while_carrying_task_is_held(self):
        self.robots['forklift_1'].lose_heartbeat_at = 'NAVIGATE_TO_DESTINATION'
        result = self.run_fleet()
        self.assertFalse(result.success)
        self.assertIn('BOX_A', result.boxes_failed)
        self.assertIn('HELD', result.message)
        self.assertNotIn('BOX_A', [b for g in self.robots['forklift_2'].goals for b in g],
                         'a box possibly on a lost robot was given to another robot')
        self.assertEqual(sorted(self.robots['forklift_2'].delivered), ['BOX_B', 'BOX_C'])
        held = [t for t in self.status.tasks if t.box_id == 'BOX_A'][0]
        self.assertEqual(held.state, 'HELD')

    def test_4_unacknowledged_assignment_benches_the_robot(self):
        self.robots['forklift_2'].ignore_goals = True
        result = self.run_fleet(timeout=240.0)
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.robots['forklift_2'].goals, [])
        self.assertEqual(sorted(self.robots['forklift_1'].delivered),
                         ['BOX_A', 'BOX_B', 'BOX_C'])
        self.robots['forklift_2'].ignore_goals = False
        # comm_cooldown: available again for the subsequent tests
        time.sleep(26.0)

    def test_5_head_on_conflict_one_yields_and_both_get_home(self):
        # The robots are at the opposite ends of the hall and have no
        # tasks. They move home along y = 0, head-on.
        self.place('forklift_1', 4.0, 0.0)
        self.place('forklift_2', -5.0, 0.0)
        self.robots['forklift_1'].speed = self.robots['forklift_2'].speed = 0.8
        time.sleep(1.0)                    # the coordinator receives the new poses first
        try:
            result = self.run_fleet(box_ids=['NONE'], timeout=120.0)
        finally:
            self.robots['forklift_1'].speed = self.robots['forklift_2'].speed = 2.0
        self.assertTrue(result.success, result.message)
        kinds = {(c.type, c.status) for c in self.conflicts}
        self.assertTrue(any(k[0] in ('HEAD_ON', 'PATH') and k[1] == 'ACTIVE' for k in kinds),
                        f'no path conflict detected: {kinds}')
        resolved = [c for c in self.conflicts if c.status == 'RESOLVED']
        self.assertTrue(resolved, 'conflict never resolved')
        # the robot id gives the decision
        self.assertEqual(resolved[0].priority_robot, 'forklift_1')
        for rid, r in self.robots.items():
            self.assertLess(abs(r.x - HOME[rid][0]) + abs(r.y - HOME[rid][1]), 1.0)

    def test_6_deadlock_backs_up_then_swaps_right_of_way(self):
        # Head-on again. When forklift_1 has the right of way, it cannot move (frozen). Without
        # deadlock recovery, forklift_2 would wait permanently.
        self.place('forklift_1', 2.5, 0.0)
        self.place('forklift_2', -1.5, 0.0)
        self.robots['forklift_1'].speed = self.robots['forklift_2'].speed = 0.8
        time.sleep(1.0)

        def script():
            if self.wait_until(lambda: any(c.status == 'ACTIVE' and
                                           c.priority_robot == 'forklift_1'
                                           for c in self.conflicts), 60.0):
                self.robots['forklift_1'].freeze = True
            self.wait_until(lambda: any('swapped' in c.resolution for c in self.conflicts),
                            90.0)
            self.robots['forklift_1'].freeze = False
        threading.Thread(target=script, daemon=True).start()
        try:
            result = self.run_fleet(box_ids=['NONE'], timeout=150.0)
        finally:
            self.robots['forklift_1'].speed = self.robots['forklift_2'].speed = 2.0
            self.robots['forklift_1'].freeze = False
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.robots['forklift_2'].backups, 1, 'yielder did not back up')
        texts = ' | '.join(c.resolution for c in self.conflicts)
        self.assertIn('backs up', texts)
        self.assertIn('swapped', texts)
        self.assertTrue(any(c.type == 'DEADLOCK' for c in self.conflicts))
        for rid, r in self.robots.items():
            self.assertLess(abs(r.x - HOME[rid][0]) + abs(r.y - HOME[rid][1]), 1.0)

    def test_7_robot_behind_a_reversing_robot_moves_away(self):
        f1, f2 = self.robots['forklift_1'], self.robots['forklift_2']
        f1.speed = 0.6

        def script():
            # forklift_1 goes to the pickup of BOX_A. When it is
            # near, forklift_2 "parks" immediately
            # behind it (north), and forklift_1 reports that its rear LiDAR is blocked.
            self.wait_until(lambda: f1.state == 'NAVIGATING_TO_PICKUP' and f1.y < -1.0, 60.0)
            f1.freeze = True
            f1.yaw = -math.pi / 2
            f2.x, f2.y = f1.x, f1.y + 2.2
            f1.rear_state = 'STOP'
            self.wait_until(lambda: any(c.type == 'REAR_BLOCKED' and 'waiting location' in
                                        c.resolution for c in self.conflicts), 40.0)
            self.wait_until(lambda: math.dist((f2.x, f2.y), HOME['forklift_2']) < 1.0, 60.0)
            f1.rear_state, f1.freeze = 'CLEAR', False
        threading.Thread(target=script, daemon=True).start()
        try:
            result = self.run_fleet(box_ids=['BOX_A'], timeout=180.0)
        finally:
            f1.speed, f1.freeze, f1.rear_state = 2.0, False, 'CLEAR'
        self.assertTrue(result.success, result.message)
        rear = [c for c in self.conflicts if c.type == 'REAR_BLOCKED']
        self.assertTrue(rear, 'rear-blocked conflict not detected')
        self.assertEqual((rear[0].priority_robot, rear[0].yielding_robot),
                         ('forklift_1', 'forklift_2'))
        self.assertTrue(any(c.status == 'RESOLVED' for c in rear))
        self.assertIn('waiting location', ' | '.join(c.resolution for c in rear))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
