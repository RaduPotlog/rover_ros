#!/usr/bin/env python3

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

"""Stands in for rover_hardware_interface's safety PLC topics in simulation.

rover_motion_lock_node keeps the twist_mux lock closed until both hardware_interface/safety_status
and hardware_interface/safety_command_echo arrive with link_healthy set, and re-closes it once
either is older than gpio_timeout, so the simulation publishes both periodically.
`ros2 param set <node> e_stop true` simulates a pressed software E-Stop: the stop request is
echoed, the latch sets and the contactor opens, as on the real safety chain.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rover_msgs.msg import SafetyCommandEcho, SafetyStatus


class SimGpioStatePublisher(Node):

    def __init__(self):
        super().__init__("sim_gpio_state_publisher")

        self.declare_parameter("publish_frequency", 5.0)
        self.declare_parameter("e_stop", False)

        # Same QoS as the hardware interface publishers and the motion lock subscriptions:
        # periodic status streams, so volatile rather than latched.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_pub = self.create_publisher(
            SafetyStatus, "hardware_interface/safety_status", qos)
        self._echo_pub = self.create_publisher(
            SafetyCommandEcho, "hardware_interface/safety_command_echo", qos)

        period = 1.0 / self.get_parameter("publish_frequency").value
        self._timer = self.create_timer(period, self._publish)
        self._heartbeat = False

    def _publish(self):
        e_stop = bool(self.get_parameter("e_stop").value)
        self._heartbeat = not self._heartbeat
        now = self.get_clock().now().to_msg()

        status = SafetyStatus()
        status.header.stamp = now
        status.io_sample_time = now
        status.hw_e_stop_user_button = False
        # A set latch opens the contactor; latched with the contactor still closed would read as
        # welded contacts.
        status.motor_contactor_engaged = not e_stop
        status.latch_active = e_stop
        status.latch_cause = (
            SafetyStatus.LATCH_CAUSE_SW_USER_BUTTON if e_stop else SafetyStatus.LATCH_CAUSE_UNKNOWN
        )
        status.link_healthy = True
        self._status_pub.publish(status)

        echo = SafetyCommandEcho()
        echo.header.stamp = now
        echo.io_sample_time = now
        echo.sw_e_stop_user_button = e_stop
        echo.sw_e_stop_motor_driver_fault = False
        echo.sw_e_stop_latch_reset = False
        echo.cpu_wdg_heartbeat = self._heartbeat
        self._echo_pub.publish(echo)


def main():
    rclpy.init()
    node = SimGpioStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
