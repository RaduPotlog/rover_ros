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

"""Stands in for the safety PLC and rover_hardware_interface's E-Stop interface in simulation.

rover_motion_lock_node keeps the twist_mux lock closed until both hardware_interface/safety_status
and hardware_interface/safety_command_echo arrive with link_healthy set, and re-closes it once
either is older than gpio_timeout, so they are published periodically.

The E-Stop latch itself is SimSafetyPlc (sim_safety_plc_model.py). It is driven by:
  * the real rover's std_srvs/Trigger services hardware_interface/sw_user_e_stop_set,
    sw_user_e_stop_reset and sw_e_stop_latch_reset (drive UI, CLI);
  * the Gazebo "Rover Safety" panel (rover_gazebo_plugins), over ros_gz_bridge: the maintained
    HW E-Stop button on sim_safety/hw_e_stop_button and the three buttons on
    sim_safety/{sw_e_stop_set,sw_e_stop_reset,latch_reset}.
The panel's lamps read sim_safety/{sw_e_stop,latch_active,contactor_engaged} and the outcome of
the last request on sim_safety/result.

An open contactor leaves the real motors unpowered whatever the drive controller asks for. In
simulation the closed twist_mux lock alone is not enough: diff_drive keeps applying its last
command while that command's stamp looks fresh (any wall-clock-stamped input does, against sim
time). So while the contactor is open this node publishes zero commands on cmd_vel, the drive
controller's input, stamped with sim time.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rover_msgs.msg import SafetyCommandEcho, SafetyStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from sim_safety_plc_model import SimSafetyPlc, TriggerResult


class SimSafetyPlcNode(Node):

    def __init__(self):
        super().__init__("sim_safety_plc")

        self.declare_parameter("publish_frequency", 5.0)
        self.declare_parameter("latch_set_at_startup", False)
        # rad/s; the URDF's velocity_state_zero_tolerance on the rover.
        self.declare_parameter("wheel_velocity_zero_tolerance", 0.05)

        self._plc = SimSafetyPlc(
            latch_set_at_startup=self.get_parameter("latch_set_at_startup").value)
        self._zero_tolerance = self.get_parameter("wheel_velocity_zero_tolerance").value
        self._wheel_velocities = None

        # Same QoS as the hardware interface publishers and the motion lock subscriptions:
        # periodic status streams, so volatile rather than latched.
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_pub = self.create_publisher(
            SafetyStatus, "hardware_interface/safety_status", status_qos)
        self._echo_pub = self.create_publisher(
            SafetyCommandEcho, "hardware_interface/safety_command_echo", status_qos)

        # Panel feedback is latched so a panel (re)connecting sees the state at once.
        panel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sw_e_stop_pub = self.create_publisher(Bool, "sim_safety/sw_e_stop", panel_qos)
        self._latch_pub = self.create_publisher(Bool, "sim_safety/latch_active", panel_qos)
        self._contactor_pub = self.create_publisher(
            Bool, "sim_safety/contactor_engaged", panel_qos)
        self._result_pub = self.create_publisher(String, "sim_safety/result", panel_qos)

        # Contactor stand-in, see the module docstring. Same input as twist_mux's output.
        self._cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

        self.create_subscription(
            Bool, "sim_safety/hw_e_stop_button", self._on_hw_button, 10)
        self.create_subscription(
            Empty, "sim_safety/sw_e_stop_set", lambda _: self._report(self._sw_set()), 10)
        self.create_subscription(
            Empty, "sim_safety/sw_e_stop_reset", lambda _: self._report(self._sw_reset()), 10)
        self.create_subscription(
            Empty, "sim_safety/latch_reset", lambda _: self._report(self._latch_reset()), 10)
        self.create_subscription(JointState, "joint_states", self._on_joint_states, 10)

        self.create_service(
            Trigger, "hardware_interface/sw_user_e_stop_set", self._trigger(self._sw_set))
        self.create_service(
            Trigger, "hardware_interface/sw_user_e_stop_reset", self._trigger(self._sw_reset))
        self.create_service(
            Trigger, "hardware_interface/sw_e_stop_latch_reset", self._trigger(self._latch_reset))

        period = 1.0 / self.get_parameter("publish_frequency").value
        self._timer = self.create_timer(period, self._publish)
        self._heartbeat = False
        self._latch_reset_requested = False

    def _sw_set(self) -> TriggerResult:
        return self._plc.sw_set()

    def _sw_reset(self) -> TriggerResult:
        return self._plc.sw_reset(self._wheels_stopped())

    def _latch_reset(self) -> TriggerResult:
        self._latch_reset_requested = True
        return self._plc.latch_reset()

    def _trigger(self, action):
        def callback(_request, response):
            result = self._report(action())
            response.success = result.success
            response.message = result.message
            return response
        return callback

    def _report(self, result: TriggerResult) -> TriggerResult:
        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().warning(result.message)
        self._result_pub.publish(String(data=result.message))
        self._publish_panel_state()
        self._cut_drive_if_contactor_open()
        return result

    def _on_hw_button(self, msg: Bool):
        if msg.data == self._plc.hw_button:
            return  # the panel republishes its button state periodically
        self._plc.set_hw_button(msg.data)
        self._report(TriggerResult(
            True, "HW E-Stop pressed" if msg.data else "HW E-Stop released"))

    def _on_joint_states(self, msg: JointState):
        self._wheel_velocities = [
            velocity for name, velocity in zip(msg.name, msg.velocity)
            if name.endswith("_wheel_joint")
        ]

    def _wheels_stopped(self) -> bool:
        # No wheel feedback yet means the check can't pass, as on the rover.
        if not self._wheel_velocities:
            return False
        return all(abs(v) <= self._zero_tolerance for v in self._wheel_velocities)

    def _cut_drive_if_contactor_open(self):
        if self._plc.contactor_engaged:
            return
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"
        self._cmd_vel_pub.publish(stop)

    def _publish_panel_state(self):
        self._sw_e_stop_pub.publish(Bool(data=self._plc.sw_user_coil))
        self._latch_pub.publish(Bool(data=self._plc.latch_active))
        self._contactor_pub.publish(Bool(data=self._plc.contactor_engaged))

    def _publish(self):
        self._heartbeat = not self._heartbeat
        now = self.get_clock().now().to_msg()

        status = SafetyStatus()
        status.header.stamp = now
        status.io_sample_time = now
        status.hw_e_stop_user_button = self._plc.hw_button
        status.motor_contactor_engaged = self._plc.contactor_engaged
        status.latch_active = self._plc.latch_active
        status.latch_cause = self._plc.latch_cause
        status.link_healthy = True
        self._status_pub.publish(status)

        echo = SafetyCommandEcho()
        echo.header.stamp = now
        echo.io_sample_time = now
        echo.sw_e_stop_user_button = self._plc.sw_user_coil
        echo.sw_e_stop_motor_driver_fault = False
        # The rover echoes the reset coil for one pulse; one publish period stands in for it.
        echo.sw_e_stop_latch_reset = self._latch_reset_requested
        echo.cpu_wdg_heartbeat = self._heartbeat
        self._echo_pub.publish(echo)
        self._latch_reset_requested = False

        self._publish_panel_state()
        self._cut_drive_if_contactor_open()


def main():
    rclpy.init()
    node = SimSafetyPlcNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
