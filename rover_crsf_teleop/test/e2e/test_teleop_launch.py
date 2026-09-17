# Copyright 2025 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
E2E test for the installed teleop node.

Started the way the launch file starts it, the node reaches active and stays silent without RC
input. The serial bridge is deliberately not started: it needs a real serial device, and its
absence is exactly the case worth asserting - teleop must come up anyway and say so through
diagnostics rather than failing to configure.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch_ros.actions.lifecycle_node import LifecycleNode
import launch_testing
import launch_testing.actions
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import pytest
import rclpy

NODE_NAME = 'rover_crsf_teleop_node'
NAMESPACE = 'teleop_e2e'


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.join(
        get_package_share_directory('rover_crsf_teleop'), 'config', 'rover_crsf_teleop.yaml')

    teleop = LifecycleNode(
        package='rover_crsf_teleop',
        executable='rover_crsf_teleop_node',
        name=NODE_NAME,
        namespace=NAMESPACE,
        parameters=[config],
        autostart=True,
        output='screen',
    )

    return LaunchDescription([teleop, launch_testing.actions.ReadyToTest()])


class TestTeleopLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('teleop_e2e_probe', namespace=NAMESPACE)

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def test_reaches_active_and_publishes_nothing_without_rc_input(self):
        received = []
        self.node.create_subscription(
            TwistStamped, 'teleop_elrs_cmd_vel_stamped', received.append, 10)

        client = self.node.create_client(GetState, f'{NODE_NAME}/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=15.0), 'get_state never appeared')

        state = {'id': None}

        def is_active():
            future = client.call_async(GetState.Request())
            if not self._spin_until(future.done, 2.0) or future.result() is None:
                return False
            state['id'] = future.result().current_state.id
            return state['id'] == State.PRIMARY_STATE_ACTIVE

        self.assertTrue(
            self._spin_until(is_active, 15.0),
            f'node never became active (last state id: {state["id"]})')

        # Condition-based: spin for a window and require that nothing arrived.
        self._spin_until(lambda: len(received) > 0, 1.0)
        self.assertEqual(received, [])


@launch_testing.post_shutdown_test()
class TestTeleopShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
