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

#include "rover_hardware_interface/rover_system/rover_a1_system.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

#include "rover_hardware_interface/rover_driver/rover_a1_driver.hpp"

namespace rover_hardware_interface
{

RoverA1System::RoverA1System() 
: RoverSystem(joints_) 
{

}

void RoverA1System::defineRoverDriver()
{
    rover_driver_ = std::make_shared<RoverA1Driver>(drivetrain_settings_);
}

void RoverA1System::updateHwStates(const rclcpp::Time & time)
{
    const auto rear_left_data = rover_driver_->getData(DriverNames::REAR_LEFT);
    const auto rear_right_data = rover_driver_->getData(DriverNames::REAR_RIGHT);

    const auto rear_left = rear_left_data.getMotorState(MotorChannels::DEFAULT);
    const auto rear_right = rear_right_data.getMotorState(MotorChannels::DEFAULT);

    //TODO: Compile switch for open loop and closed loop
    rclcpp::Duration period = time - last_time_;
    last_time_ = time;
    double period_sec = period.seconds();

    for (size_t i = 0; i < hw_states_positions_.size(); ++i) {
        hw_states_positions_[i] += hw_commands_velocities_[i] * period_sec;
    }

    for (size_t i = 0; i < hw_states_velocities_.size(); ++i) {
        hw_states_velocities_[i] = hw_commands_velocities_[i];
    }

    hw_states_efforts_[0] = rear_left.getTorque();
    hw_states_efforts_[1] = rear_right.getTorque();
    hw_states_efforts_[2] = rear_left.getTorque();
    hw_states_efforts_[3] = rear_right.getTorque();
}

void RoverA1System::updateDriverStateMsg()
{
    const auto driver_rear_left_data = rover_driver_->getData(DriverNames::REAR_LEFT);
    const auto driver_rear_right_data = rover_driver_->getData(DriverNames::REAR_RIGHT);

    system_ros_interface_->updateMsgDriversStates(DriverNames::REAR_LEFT, 
        driver_rear_left_data.getDriverState());
    system_ros_interface_->updateMsgErrorFlags(DriverNames::REAR_LEFT, 
        driver_rear_left_data);
    
    system_ros_interface_->updateMsgDriversStates(DriverNames::REAR_RIGHT, 
        driver_rear_right_data.getDriverState());
    system_ros_interface_->updateMsgErrorFlags(DriverNames::REAR_RIGHT, 
        driver_rear_right_data);
    
    // TODO: Handle communication error
}

std::vector<float> RoverA1System::getSpeedCmd() const
{
    return { static_cast<float>(hw_commands_velocities_[0]), 
             static_cast<float>(hw_commands_velocities_[1]),
             static_cast<float>(hw_commands_velocities_[2]),
             static_cast<float>(hw_commands_velocities_[3])
           };
}

}  // namespace rover_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rover_hardware_interface::RoverA1System, hardware_interface::SystemInterface)
