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
E2E test for the installed teleop node, fed through a real UDP socket.

The teleop node and rover_udp_driver's receiver run as the launch file pairs them, except on
loopback: the receiver binds 127.0.0.1 and accepts 127.0.0.1 only, standing in for the RUTX11.
In order (unittest runs the methods alphabetically):

  a. with no RC input the node reaches active and publishes nothing;
  b. CRSF from any other source (127.0.0.2) is dropped by the receiver - nothing moves;
  c. CRSF from the accepted source drives teleop_elrs_cmd_vel_stamped.
"""

import os
import socket
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions.lifecycle_node import LifecycleNode
import launch_testing
import launch_testing.actions
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import pytest
import rclpy

NODE_NAME = 'rover_crsf_teleop_node'
NAMESPACE = 'teleop_e2e'

ROUTER_IP = '127.0.0.1'
FOREIGN_IP = '127.0.0.2'  # All of 127.0.0.0/8 is loopback on Linux.
# Per process, so parallel test runs on one machine do not collide.
UDP_PORT = 30000 + os.getpid() % 10000

CRSF_ADDRESS = 0xC8
CRSF_LINK_STATISTICS = 0x14
CRSF_RC_CHANNELS_PACKED = 0x16
CHANNEL_MID = 992
CHANNEL_MAX = 1811


def _crc8_dvb_s2(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _frame(frame_type, payload):
    body = bytes([frame_type]) + bytes(payload)
    return bytes([CRSF_ADDRESS, len(body) + 1]) + body + bytes([_crc8_dvb_s2(body)])


def _rc_channels_frame(channels):
    packed = 0
    for index, value in enumerate(channels):
        packed |= (value & 0x7FF) << (11 * index)
    return _frame(CRSF_RC_CHANNELS_PACKED, packed.to_bytes(22, 'little'))


def _link_statistics_frame(uplink_link_quality):
    payload = [0] * 10
    payload[2] = uplink_link_quality
    return _frame(CRSF_LINK_STATISTICS, payload)


def _deflected_datagram():
    """Full throttle on channel 3 (linear.x in the shipped config), everything else centred."""
    channels = [CHANNEL_MID] * 16
    channels[2] = CHANNEL_MAX
    return _rc_channels_frame(channels) + _link_statistics_frame(100)


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.join(
        get_package_share_directory('rover_crsf_teleop'), 'config', 'rover_crsf_teleop.yaml')

    udp = {'udp_bind_ip': ROUTER_IP, 'udp_port': UDP_PORT, 'udp_source_ip': ROUTER_IP}

    teleop = LifecycleNode(
        package='rover_crsf_teleop',
        executable='rover_crsf_teleop_node',
        name=NODE_NAME,
        namespace=NAMESPACE,
        parameters=[config, udp],
        autostart=True,
        output='screen',
    )

    # Configured exactly as the launch file does it, from the same three values.
    receiver = Node(
        package='rover_udp_driver',
        executable='rover_udp_receiver_node',
        name='rover_crsf_udp_receiver',
        namespace=NAMESPACE,
        parameters=[{
            'ip': udp['udp_bind_ip'],
            'port': udp['udp_port'],
            'source_ip': udp['udp_source_ip'],
            'autostart': True,
        }],
        remappings=[('udp_read', 'rc/raw_udp')],
        output='screen',
    )

    return LaunchDescription([teleop, receiver, launch_testing.actions.ReadyToTest()])


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

    def _feed_from(self, source_ip, until, timeout_sec):
        """Send deflected-stick CRSF at ~50 Hz from `source_ip` until `until()` or timeout."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.bind((source_ip, 0))
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline and not until():
                sender.sendto(_deflected_datagram(), (ROUTER_IP, UDP_PORT))
                # Spinning is the pacing: 20 ms per datagram, as the receiver sends them.
                rclpy.spin_once(self.node, timeout_sec=0.02)
        return until()

    def test_a_reaches_active_and_publishes_nothing_without_rc_input(self):
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

    def _receiver_active(self):
        client = self.node.create_client(GetState, 'rover_crsf_udp_receiver/get_state')
        if not client.wait_for_service(timeout_sec=15.0):
            return False

        def is_active():
            future = client.call_async(GetState.Request())
            if not self._spin_until(future.done, 2.0) or future.result() is None:
                return False
            return future.result().current_state.id == State.PRIMARY_STATE_ACTIVE

        return self._spin_until(is_active, 15.0)

    def test_b_ignores_rc_from_a_foreign_source(self):
        received = []
        self.node.create_subscription(
            TwistStamped, 'teleop_elrs_cmd_vel_stamped', received.append, 10)

        # A negative test is only worth something if the receiver was listening: otherwise
        # silence proves nothing. test_c (same receiver, accepted source) is the positive control.
        self.assertTrue(self._receiver_active(), 'rover_crsf_udp_receiver never became active')

        # Longer than switch_settle_frames (2 s) plus margin: had the receiver let these through,
        # teleop would be commanding by now.
        self._feed_from(FOREIGN_IP, lambda: len(received) > 0, 4.0)
        self.assertEqual(received, [])

    def test_c_drives_from_rc_over_udp(self):
        received = []
        self.node.create_subscription(
            TwistStamped, 'teleop_elrs_cmd_vel_stamped', received.append, 10)

        self.assertTrue(
            self._feed_from(
                ROUTER_IP, lambda: any(m.twist.linear.x > 0.0 for m in received), 15.0),
            'a deflected stick sent over UDP never produced a forward command')


@launch_testing.post_shutdown_test()
class TestTeleopShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
