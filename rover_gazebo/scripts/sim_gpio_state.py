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

"""Stands in for rover_hardware_interface's gpio_state in simulation.

rover_motion_lock_node keeps the twist_mux lock closed until hardware_interface/gpio_state
arrives and re-closes it once the message is older than gpio_timeout, so the simulation
publishes the safety I/O periodically. `ros2 param set <node> e_stop true` simulates a
pressed software E-Stop.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rover_msgs.msg import GpioState


class SimGpioStatePublisher(Node):

    def __init__(self):
        super().__init__("sim_gpio_state_publisher")

        self.declare_parameter("publish_frequency", 5.0)
        self.declare_parameter("e_stop", False)

        # Same QoS as the hardware interface publisher and the motion lock subscription.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(GpioState, "hardware_interface/gpio_state", qos)

        period = 1.0 / self.get_parameter("publish_frequency").value
        self._timer = self.create_timer(period, self._publish)
        self._heartbeat = False

    def _publish(self):
        e_stop = self.get_parameter("e_stop").value
        self._heartbeat = not self._heartbeat

        msg = GpioState()
        msg.gpio_pin_hw_e_stop_user_button = False
        msg.gpio_pin_motor_contactor_engaged = True
        msg.gpio_pin_cpu_wdg_heartbeat = self._heartbeat
        msg.gpio_pin_sw_e_stop_user_button = bool(e_stop)
        msg.gpio_pin_sw_e_stop_motor_driver_fault = False
        msg.gpio_pin_sw_e_stop_latch_reset = False
        msg.gpio_pin_sw_e_stop_latch_status = bool(e_stop)
        self._pub.publish(msg)


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
