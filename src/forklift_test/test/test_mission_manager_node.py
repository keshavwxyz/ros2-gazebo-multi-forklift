"""
Tier-2 integration test without a simulator: only the mission manager node.

There are no Nav2 or load-handling servers. Thus, the mission must fail correctly in
INITIALIZE (after its configured new tries) and report the reason. The node must refuse
requests for unknown boxes and operator commands when no mission runs. This test runs in CI.
"""

import json
import os
import tempfile
import unittest

from forklift_interfaces.action import RunMission
from forklift_interfaces.msg import MissionStatus
from forklift_test.helpers import spin_until
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

RECORD_DIR = tempfile.mkdtemp(prefix='forklift_mission_test_')


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='forklift_task_manager', executable='mission_manager', output='screen',
        parameters=[{'server_timeout': 1.0, 'retries.INITIALIZE': 1,
                     'record_dir': RECORD_DIR}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestMissionManagerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mission_manager_tester')
        cls.states = []
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(MissionStatus, '/mission/status',
                                     lambda m: cls.states.append(m), latched)
        cls.client = ActionClient(cls.node, RunMission, '/mission/run')
        assert cls.client.wait_for_server(timeout_sec=30.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def run_mission(self, box_ids):
        send = self.client.send_goal_async(RunMission.Goal(box_ids=box_ids))
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
        handle = send.result()
        self.assertTrue(handle.accepted)
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=60.0)
        self.assertTrue(res.done(), 'mission did not end')
        return res.result().result

    def call(self, name):
        cli = self.node.create_client(Trigger, name)
        self.assertTrue(cli.wait_for_service(timeout_sec=10.0))
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        return fut.result()

    def test_1_idle_status_and_commands_refused(self):
        self.assertTrue(spin_until(self.node, lambda: self.states, 20.0))
        self.assertEqual(self.states[-1].state, 'IDLE')
        self.assertFalse(self.call('/mission/pause').success)
        self.assertFalse(self.call('/mission/resume').success)

    def test_2_unknown_box_rejected(self):
        result = self.run_mission(['BOX_Z'])
        self.assertFalse(result.success)
        self.assertIn('unknown box ids: BOX_Z', result.message)
        self.assertEqual(list(result.boxes_delivered), [])

    def test_3_missing_servers_fail_initialize_after_retry(self):
        result = self.run_mission([])
        self.assertFalse(result.success)
        self.assertIn('MISSION_FAILED', result.message)
        self.assertIn('INITIALIZE: SYSTEMS_UNAVAILABLE', result.message)
        self.assertIn('handling/pickup', result.message)
        spin_until(self.node, lambda: self.states[-1].state == 'MISSION_FAILED', 10.0)
        self.assertEqual(self.states[-1].boxes_total, 3)
        records = sorted(os.listdir(RECORD_DIR))
        self.assertTrue(records, 'no mission record written')
        with open(os.path.join(RECORD_DIR, records[-1])) as f:
            rec = json.load(f)
        self.assertEqual(rec['final_state'], 'MISSION_FAILED')
        self.assertEqual([e['state'] for e in rec['events']], ['INITIALIZE', 'INITIALIZE'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
