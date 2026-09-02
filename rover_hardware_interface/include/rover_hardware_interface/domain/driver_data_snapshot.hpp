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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_DRIVER_DATA_SNAPSHOT_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_DRIVER_DATA_SNAPSHOT_HPP_

#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "rover_hardware_interface/domain/driver.hpp"
#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

template <typename T>
std::map<std::string, T> prefixMapKeys(
    const std::map<std::string, T> & map, const std::string & prefix)
{
    std::map<std::string, T> prefixed_map;

    for (const auto & [key, value] : map) {
        prefixed_map[prefix + key] = value;
    }

    return prefixed_map;
}

class FlagError
{

public:

    FlagError(
        const std::vector<std::string> & flag_names,
        const std::vector<std::string> & suppressed_flags_names = {});

    virtual ~FlagError() = default;

    void setData(const std::uint8_t flags)
    {
        flags_ = flags;
    }

    bool isError() const
    {
        return (flags_ & (~suppressed_flags_)).any();
    }

    std::string getErrorLog() const;

protected:

    const std::vector<std::string> flag_names_;

    std::bitset<8> suppressed_flags_ = 0;
    std::bitset<8> flags_ = 0;
};

class FaultFlag : public FlagError
{

    public:

    FaultFlag();

    // Allocation-free accessors for the RT-reachable message-marshaling path
    // (system_ros_interface.cpp); index order matches the flag_names_ passed to FlagError's
    // constructor, same as getErrorMap() below, so the two can never disagree.
    bool isEmergencyStop() const { return flags_.test(kEmergencyStopBit); }
    bool isMotorSetupFault() const { return flags_.test(kMotorSetupFaultBit); }

    std::map<std::string, bool> getErrorMap() const;

private:

    static constexpr std::size_t kEmergencyStopBit = 0;
    static constexpr std::size_t kMotorSetupFaultBit = 1;
};

class RuntimeError : public FlagError
{

public:

    RuntimeError();

    // See FaultFlag's accessors above for the allocation-free-RT-path rationale.
    bool isSafetyStopActive() const { return flags_.test(kSafetyStopActiveBit); }

    std::map<std::string, bool> getErrorMap() const;

private:

    static constexpr std::size_t kSafetyStopActiveBit = 0;
};

class DriverStateReading
{

public:

    DriverStateReading() = default;

    void setTemperature(const std::int16_t temp)
    {
        temp_ = temp;
    }

    void setDriverCurrent(const std::int16_t driver_current)
    {
        driver_current_ = driver_current;
    }

    std::int16_t getTemperature() const
    {
        return temp_;
    }

    float getDriverCurrent() const
    {
        return driver_current_;
    }

private:

    std::int16_t temp_ = 0;
    float driver_current_ = 0.0;
};

class MotorStateReading
{

public:

    MotorStateReading(const DrivetrainSettings & drivetrain_settings);

    void setData(const MotorDriverState & motor_state);

    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;

private:

    float phidget_pos_feedback_to_radians_;
    float phidget_vel_feedback_to_radians_per_second_;
    float phidget_current_feedback_to_newton_meters_;

    MotorDriverState motor_state_ = {0, 0, 0, 0.0};
};

class DriverDataSnapshot
{

public:

    DriverDataSnapshot(const DrivetrainSettings & drivetrain_settings);

    void setMotorsStates(
        const MotorDriverState & state,
        const bool data_timed_out);

    void setDriverState(const DriverState & state, const bool data_timed_out);

    bool isFlagError() const {
        return fault_flags_.isError();
    }

    bool isError() const {
        return isFlagError() || motor_states_data_timed_out_ || driver_state_data_timed_out_;
    }

    bool isCommunicationError() const {
        return motor_states_data_timed_out_ || driver_state_data_timed_out_;
    }

    const MotorStateReading & getMotorState(const MotorNames channel) const;

    const DriverStateReading & getDriverState() const;

    bool isMotorStatesDataTimedOut() const { return motor_states_data_timed_out_; }
    bool isDriverStateDataTimedOut() const { return driver_state_data_timed_out_; }

    const FaultFlag & getFaultFlag() const;

    const RuntimeError & getRuntimeError(const MotorNames channel) const;

    std::map<std::string, bool> getFlagErrorMap() const;
    std::map<std::string, bool> getErrorMap() const;

private:

    MotorStateReading motor_state_;
    DriverStateReading driver_state_;

    FaultFlag fault_flags_;
    RuntimeError runtime_error_;

    bool motor_states_data_timed_out_ = false;
    bool driver_state_data_timed_out_ = false;
};

} // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_DRIVER_DATA_SNAPSHOT_HPP_
