// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVER_TWIST_MUX_DOMAIN_SAFETY_IO_FLAGS_HPP_
#define ROVER_TWIST_MUX_DOMAIN_SAFETY_IO_FLAGS_HPP_

namespace rover_twist_mux::domain
{

/**
 * @brief The safety-IO pin states that decide whether the rover may be commanded to move.
 * @details Mirrors the subset of rover_msgs/GpioState this package cares about, but as plain
 *          bools so the policy stays free of ROS types. Polarity follows the hardware interface:
 *          every `*_e_stop_*` flag is `true` when that stop is ACTIVE (see
 *          rover_hardware_interface EmergencyStop::readEStopState() — "the port reports `true`
 *          when the E-Stop is triggered"). `motor_contactor_engaged` is the one inverted signal:
 *          `true` means the contactor is CLOSED, i.e. the drive is live.
 *
 *          Deliberately absent: the CPU watchdog line (GpioState.gpio_pin_cpu_wdg_heartbeat).
 *          It is a liveness heartbeat the safety controller drives as a square wave, not a fault
 *          flag, so neither level says anything about whether motion is safe. A stalled heartbeat
 *          is caught by the safety relay, which latches the e-stop — covered here by
 *          `sw_e_stop_latch_status`.
 *
 *          Defaults are the worst case on purpose: a default-constructed value denies motion, so
 *          a field that is never populated fails safe rather than opening the command path.
 */
struct SafetyIoFlags
{
    /// Hardware E-Stop button. `true` = pressed.
    bool hw_e_stop_user_button = true;

    /// Software E-Stop user button. `true` = triggered.
    bool sw_e_stop_user_button = true;

    /// Motor-driver fault stop. `true` = faulted.
    bool sw_e_stop_motor_driver_fault = true;

    /// Safety latch. `true` = latched (a stop is being held until an explicit reset).
    bool sw_e_stop_latch_status = true;

    /// Motor contactor. `true` = ENGAGED / drive live. Note the inverted sense.
    bool motor_contactor_engaged = false;
};

}  // namespace rover_twist_mux::domain

#endif  // ROVER_TWIST_MUX_DOMAIN_SAFETY_IO_FLAGS_HPP_
