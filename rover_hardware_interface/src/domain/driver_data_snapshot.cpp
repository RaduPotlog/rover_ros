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

#include "rover_hardware_interface/domain/driver_data_snapshot.hpp"

#include <cmath>
#include <stdexcept>

namespace rover_hardware_interface
{

FlagError::FlagError(
    const std::vector<std::string> & flag_names,
    const std::vector<std::string> & suppressed_flags_names)
: flag_names_(&flag_names)
{
    for (size_t i = 0; i < suppressed_flags_names.size(); ++i) {
        for (size_t j = 0; j < flag_names_->size(); ++j) {
            if (suppressed_flags_names[i] == (*flag_names_)[j]) {
                suppressed_flags_.set(j);
            }
        }
    }
}

std::string FlagError::getErrorLog() const
{
    std::string error_msg = "";

    for (std::size_t i = 0; i < flag_names_->size(); i++) {
        if ((flags_ & (~suppressed_flags_)).test(i)) {
            error_msg += (*flag_names_)[i] + " ";
        }
    }

    return error_msg;
}

const std::vector<std::string> FaultFlag::kFlagNames = {
    "emergency_stop",
    "motor_setup_fault",
};

FaultFlag::FaultFlag()
: FlagError(kFlagNames)
{

}

std::map<std::string, bool> FaultFlag::getErrorMap() const
{
    std::map<std::string, bool> error_map;

    for (std::size_t i = 0; i < flag_names_->size(); i++) {
        error_map["fault_flag." + (*flag_names_)[i]] = flags_.test(i);
    }

    return error_map;
}

const std::vector<std::string> RuntimeError::kFlagNames = {
    "safety_stop_active",
};

const std::vector<std::string> RuntimeError::kSuppressedFlagNames = {
    "safety_stop_active",
};

RuntimeError::RuntimeError()
: FlagError(kFlagNames, kSuppressedFlagNames)
{

}

std::map<std::string, bool> RuntimeError::getErrorMap() const
{
    std::map<std::string, bool> error_map;

    for (std::size_t i = 0; i < flag_names_->size(); i++) {
        error_map["runtime_error." + (*flag_names_)[i]] = flags_.test(i);
    }

    return error_map;
}

MotorStateReading::MotorStateReading(const DrivetrainSettings & drivetrain_settings)
{
    phidget_pos_feedback_to_radians_ = (2.0f * M_PI) / (drivetrain_settings.encoder_resolution * drivetrain_settings.gear_ratio);

    phidget_vel_feedback_to_radians_per_second_ = (2.0f * M_PI) / 60.0f / drivetrain_settings.gear_ratio;

    phidget_current_feedback_to_newton_meters_ = (1.0f / 10.0f) * drivetrain_settings.motor_torque_constant *
                                                 drivetrain_settings.gear_ratio *
                                                 drivetrain_settings.gearbox_efficiency;
}

void MotorStateReading::setData(const MotorDriverState & motor_state)
{
    motor_state_ = motor_state;
}

float MotorStateReading::getPosition() const
{
    return static_cast<float>(motor_state_.pos) * phidget_pos_feedback_to_radians_;
}

float MotorStateReading::getVelocity() const
{
    return static_cast<float>(motor_state_.vel) * phidget_vel_feedback_to_radians_per_second_;
}

float MotorStateReading::getTorque() const
{
    return static_cast<float>(motor_state_.current) * phidget_current_feedback_to_newton_meters_;
}

DriverDataSnapshot::DriverDataSnapshot(const DrivetrainSettings & drivetrain_settings)
: motor_state_(drivetrain_settings)
{

}

void DriverDataSnapshot::setMotorsStates(
    const MotorDriverState & state,
    const bool data_timed_out)
{
    motor_state_.setData(state);

    motor_states_data_timed_out_ = data_timed_out;
}

void DriverDataSnapshot::setDriverState(
    const DriverState & state,
    const bool data_timed_out)
{
    driver_state_.setTemperature(state.temp);
    driver_state_.setDriverCurrent(state.driver_current);

    fault_flags_.setData(state.fault_flags);
    runtime_error_.setData(state.runtime_stat_flag);

    driver_state_data_timed_out_ = data_timed_out;
}

const MotorStateReading & DriverDataSnapshot::getMotorState(
    const MotorNames channel) const
{
    if (channel == MotorNames::DEFAULT) {
        return motor_state_;
    }

    throw std::runtime_error("Invalid channel number");
}

const DriverStateReading & DriverDataSnapshot::getDriverState() const
{
    return driver_state_;
}

const FaultFlag & DriverDataSnapshot::getFaultFlag() const
{
    return fault_flags_;
}

const RuntimeError & DriverDataSnapshot::getRuntimeError(const MotorNames channel) const
{
    if (channel == MotorNames::DEFAULT) {
        return runtime_error_;
    }

    throw std::runtime_error("Invalid channel number");
}

std::map<std::string, bool> DriverDataSnapshot::getFlagErrorMap() const
{
    std::map<std::string, bool> flag_error_map;

    flag_error_map.merge(fault_flags_.getErrorMap());

    auto runtime_error_map = rover_hardware_interface::prefixMapKeys(
        runtime_error_.getErrorMap(), "motor_state.");

    flag_error_map.merge(std::move(runtime_error_map));

    return flag_error_map;
}

std::map<std::string, bool> DriverDataSnapshot::getErrorMap() const
{
    std::map<std::string, bool> error_map;

    const auto flag_error_map = getFlagErrorMap();
        error_map.insert(flag_error_map.begin(), flag_error_map.end());

    error_map.emplace("motor_states_data_timed_out", isMotorStatesDataTimedOut());
    error_map.emplace("driver_state_data_timed_out", isDriverStateDataTimedOut());

    return error_map;
}

} // namespace rover_hardware_interface
