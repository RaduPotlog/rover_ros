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

#ifndef ROVER_HARDWARE_INTERFACE_UTILS_HPP_
#define ROVER_HARDWARE_INTERFACE_UTILS_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

namespace rover_hardware_interface
{

// Default for DrivetrainSettings::motor_acceleration - the value that used to be hard-coded.
constexpr float kDefaultMotorAcceleration = 2.0f;
// Defaults for DrivetrainSettings::motor_current_limit / motor_supply_voltage - the previously
// hard-coded 10 A limit on the 24 V supply (gain 20).
constexpr float kDefaultMotorCurrentLimit = 10.0f;
constexpr float kDefaultMotorSupplyVoltage = 24.0f;

// Current regulator gain of the motor controller's current loop (e.g.
// PhidgetDCMotor_setCurrentRegulatorGain()), derived from the current limit and supply voltage
// with Phidget's rule of thumb gain = current_limit * (voltage / 12). Derived rather than
// configured so it can't drift out of step with the limit.
inline float motorCurrentRegulatorGain(const float current_limit_a, const float supply_voltage_v)
{
    return current_limit_a * (supply_voltage_v / 12.0f);
}

struct DrivetrainSettings
{
    float motor_torque_constant;
    float gear_ratio;
    float gearbox_efficiency;
    float encoder_resolution;
    float max_rpm_motor_speed;
    unsigned driver_comm_timeout_ms;
    // Scales a driver's raw current feedback reading into amps before it's multiplied by
    // motor_torque_constant/gear_ratio/gearbox_efficiency to get torque (see
    // MotorStateReading::getTorque() in domain/driver_data_snapshot.cpp). This is a property of
    // the driver backend's raw feedback units, not of the drivetrain itself - e.g. Phidget
    // motor controllers report current in deci-amps, so PhidgetRoverDriver's caller supplies
    // 0.1f here. A different driver backend reporting raw current in a different unit supplies
    // its own value instead of silently inheriting Phidget's convention.
    float raw_current_to_amps_scale;
    // Timeout (ms) for the motor driver's own hardware watchdog (e.g. Phidget's
    // PhidgetDCMotor_enableFailsafe()) - see MotorDriverInterface::armFailsafe() in domain/
    // driver.hpp. Optional in the URDF; absent means kDefaultMotorFailsafeTimeoutMs, so an
    // existing URDF keeps working unchanged.
    unsigned motor_failsafe_timeout_ms;
    // On-board duty-cycle ramp of the motor controller, in duty/s (e.g.
    // PhidgetDCMotor_setAcceleration()). It adds lag the ROS side cannot see, so it is exposed to
    // let a closed wheel-speed loop own the dynamics. Optional in the URDF.
    float motor_acceleration{kDefaultMotorAcceleration};
    // Motor controller current limit in A (e.g. PhidgetDCMotor_setCurrentLimit()). Optional in
    // the URDF.
    float motor_current_limit{kDefaultMotorCurrentLimit};
    // Motor supply voltage in V; only used to derive the current regulator gain (see
    // motorCurrentRegulatorGain()). Optional in the URDF.
    float motor_supply_voltage{kDefaultMotorSupplyVoltage};
};

constexpr unsigned kDefaultMotorFailsafeTimeoutMs = 500;

// `log_warning`, if provided, receives each attempt-failure message instead of the default
// std::cerr fallback - lets infrastructure callers (e.g. RoverSystem, which has an rclcpp::Logger)
// route these through RCLCPP_WARN_STREAM instead. Left optional (default nullptr -> std::cerr)
// rather than a hard rclcpp dependency here, since this header is transitively included from
// domain/driver_data_snapshot.hpp and must stay ROS-free (see the comment in utils.cpp and
// scripts/check_domain_purity.sh).
bool operationWithAttempts(
    const std::function<void()> operation,
    const unsigned max_attempts,
    const std::function<void()> on_error = []() {},
    const std::chrono::milliseconds delay_between_attempts = std::chrono::milliseconds(0),
    const std::function<void(const std::string &)> & log_warning = nullptr);

bool checkIfJointNameContainValidSequence(
    const std::string & name,
    const std::string & sequence);

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_UTILS_HPP_
