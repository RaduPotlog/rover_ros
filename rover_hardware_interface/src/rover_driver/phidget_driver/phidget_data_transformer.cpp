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

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_data_transformer.hpp"

#include <cmath>
#include <iostream>

#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

FlagError::FlagError(
    const std::vector<std::string> & flag_names,
    const std::vector<std::string> & suppressed_flags_names)
: flag_names_(flag_names)
{
    for (size_t i = 0; i < suppressed_flags_names.size(); ++i) {
        for (size_t j = 0; j < flag_names_.size(); ++j) {
            if (suppressed_flags_names[i] == flag_names_[j]) {
                suppressed_flags_.set(j);
            }
        }
    }
}

std::string FlagError::getErrorLog() const
{
    std::string error_msg = "";
  
    for (std::size_t i = 0; i < flag_names_.size(); i++) {
        if ((flags_ & (~suppressed_flags_)).test(i)) {
            error_msg += flag_names_[i] + " ";
        }
    }
  
    return error_msg;
}

FaultFlag::FaultFlag()
: FlagError(
    {
        "emergency_stop",
        "motor_setup_fault",
    }
)
{

}

rover_msgs::msg::FaultFlag FaultFlag::getMessage() const
{
  rover_msgs::msg::FaultFlag fault_flags_msg;

    fault_flags_msg.emergency_stop = flags_.test(4);
    fault_flags_msg.motor_setup_fault = flags_.test(5);

    return fault_flags_msg;
}

std::map<std::string, bool> FaultFlag::getErrorMap() const
{
    std::map<std::string, bool> error_map;
        
    for (std::size_t i = 0; i < flag_names_.size(); i++) {
        error_map["fault_flag." + flag_names_[i]] = flags_.test(i);
    }

    return error_map;
}

RuntimeError::RuntimeError()
: FlagError(
    {
      "safety_stop_active",
    },
    {
      "safety_stop_active",
    }
)
{

}

rover_msgs::msg::RuntimeError RuntimeError::getMessage() const
{
    rover_msgs::msg::RuntimeError runtime_errors_msg;

    runtime_errors_msg.safety_stop_active = flags_.test(1);

    return runtime_errors_msg;
}

std::map<std::string, bool> RuntimeError::getErrorMap() const
{
    std::map<std::string, bool> error_map;
  
    for (std::size_t i = 0; i < flag_names_.size(); i++) {
        error_map["runtime_error." + flag_names_[i]] = flags_.test(i);
    }

    return error_map;
}

PhidgetVelocityCommandDataTransformer::PhidgetVelocityCommandDataTransformer(
    const DrivetrainSettings & drivetrain_settings)
{
    radians_per_second_to_phidget_cmd_ = drivetrain_settings.gear_ratio * (1.0f / (2.0f * M_PI)) * 
                                         60.0f * (kMaxPhidgetCmdValue / drivetrain_settings.max_rpm_motor_speed);
}

float PhidgetVelocityCommandDataTransformer::convert(const float cmd) const
{
    return clampVelCmd(cmd * radians_per_second_to_phidget_cmd_);
}

PhidgetMotorStateTransformer::PhidgetMotorStateTransformer(const DrivetrainSettings & drivetrain_settings)
{
    phidget_pos_feedback_to_radians_ = (1.0f / drivetrain_settings.encoder_resolution) *
                                       (1.0f / drivetrain_settings.gear_ratio) * (2.0f * M_PI);

    phidget_vel_feedback_to_radians_per_second_ = (drivetrain_settings.max_rpm_motor_speed / 1000.0f) * 
                                                  (1.0f / drivetrain_settings.gear_ratio) *
                                                  (1.0f / 60.0f) * (2.0f * M_PI);

    phidget_current_feedback_to_newton_meters_ = (1.0f / 10.0f) * drivetrain_settings.motor_torque_constant * 
                                                 drivetrain_settings.gear_ratio * 
                                                 drivetrain_settings.gearbox_efficiency;
}

void PhidgetMotorStateTransformer::setData(const MotorDriverState & motor_state) 
{ 
    motor_state_ = motor_state;
};

float PhidgetMotorStateTransformer::getPosition() const
{ 
    return static_cast<float>(motor_state_.pos) * phidget_pos_feedback_to_radians_; 
}

float PhidgetMotorStateTransformer::getVelocity() const
{
    return motor_state_.vel * phidget_vel_feedback_to_radians_per_second_;
}

float PhidgetMotorStateTransformer::getTorque() const
{
    return motor_state_.current * phidget_current_feedback_to_newton_meters_;
}

PhidgetDriverDataTransformer::PhidgetDriverDataTransformer(const DrivetrainSettings & drivetrain_settings)
: motor_state_(drivetrain_settings)
{

}

void PhidgetDriverDataTransformer::setMotorsStates(
    const MotorDriverState & state,
    const bool data_timed_out)
{
    motor_state_.setData(state);
    
    motor_states_data_timed_out_ = data_timed_out;
}

void PhidgetDriverDataTransformer::setDriverState(
    const DriverState & state, 
    const bool data_timed_out)
{    
    driver_state_.setTemperature(state.temp);
    driver_state_.setDriverCurrent(state.driver_current);

    fault_flags_.setData(state.fault_flags);
    runtime_error_.setData(state.runtime_stat_flag);
    
    driver_state_data_timed_out_ = data_timed_out;
}

const PhidgetMotorStateTransformer & PhidgetDriverDataTransformer::getMotorState(
    const std::uint8_t channel) const
{
    if (channel == PhidgetDriver::motorChannelDefault) {
        return motor_state_;
    }

    throw std::runtime_error("Invalid channel number");
}

const PhidgetDriverStateTransformer & PhidgetDriverDataTransformer::getDriverState() const 
{ 
    return driver_state_;
}

const FaultFlag & PhidgetDriverDataTransformer::getFaultFlag() const 
{ 
    return fault_flags_; 
}

const RuntimeError & PhidgetDriverDataTransformer::getRuntimeError(const std::uint8_t channel) const
{
    if (channel == PhidgetDriver::motorChannelDefault) {
        return runtime_error_;
    }

    throw std::runtime_error("Invalid channel number");
}

} // namespace rover_hardware_interface