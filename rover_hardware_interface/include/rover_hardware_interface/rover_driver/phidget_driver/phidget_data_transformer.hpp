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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rover_msgs/msg/fault_flag.hpp"
#include "rover_msgs/msg/runtime_error.hpp"

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"
#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

struct DrivetrainSettings
{
    float motor_torque_constant;
    float gear_ratio;
    float gearbox_efficiency;
    float encoder_resolution;
    float max_rpm_motor_speed;
};

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
    
    rover_msgs::msg::FaultFlag getMessage() const;
    
    std::map<std::string, bool> getErrorMap() const;
};

class RuntimeError : public FlagError
{

public:

    RuntimeError();
    
    rover_msgs::msg::RuntimeError getMessage() const;
    
    std::map<std::string, bool> getErrorMap() const;
};

class PhidgetVelocityCommandDataTransformer
{

public:
    
    PhidgetVelocityCommandDataTransformer(const DrivetrainSettings & drivetrain_settings);
    
    float convert(const float cmd) const;

private:
    
    static constexpr float kMaxPhidgetCmdValue = 1.0f;

    inline float clampVelCmd(const float cmd) const
    {
        return std::clamp(cmd, -kMaxPhidgetCmdValue, kMaxPhidgetCmdValue);
    }

    float radians_per_second_to_phidget_cmd_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
};

class PhidgetDriverStateTransformer
{

public:
  
    PhidgetDriverStateTransformer() 
    {

    }

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

class PhidgetMotorStateTransformer
{

public:

    PhidgetMotorStateTransformer(const DrivetrainSettings & drivetrain_settings);

    void setData(const MotorDriverState & motor_state);

    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;

private:

    float phidget_pos_feedback_to_radians_;
    float phidget_vel_feedback_to_radians_per_second_;
    float phidget_current_feedback_to_newton_meters_;

    MotorDriverState motor_state_ = {0, 0, 0};
};

class PhidgetDriverDataTransformer
{

public:

    PhidgetDriverDataTransformer(const DrivetrainSettings & drivetrain_settings);

    void setMotorsStates(
        const MotorDriverState & state,
        const bool data_timed_out);
  
    void setDriverState(const DriverState & state, const bool data_timed_out);

    const PhidgetMotorStateTransformer & getMotorState(const std::uint8_t channel) const;

    const PhidgetDriverStateTransformer & getDriverState() const;

    const FaultFlag & getFaultFlag() const;
    
    const RuntimeError & getRuntimeError(const std::uint8_t channel) const;

private:
  
    PhidgetMotorStateTransformer motor_state_;
    PhidgetDriverStateTransformer driver_state_;

    FaultFlag fault_flags_;
    RuntimeError runtime_error_;

    bool motor_states_data_timed_out_ = false;
    bool driver_state_data_timed_out_ = false;
};

} // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_