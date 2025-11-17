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

#ifndef ROVER_HARDWARE_INTERFACES_ROVER_SYSTEM_ROVER_A1_SYSTEM_HPP_
#define ROVER_HARDWARE_INTERFACES_ROVER_SYSTEM_ROVER_A1_SYSTEM_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rover_hardware_interface/rover_system/rover_system.hpp"

namespace rover_hardware_interface
{

class RoverA1System : public RoverSystem
{

public:

    RCLCPP_SHARED_PTR_DEFINITIONS(RoverA1System)

    RoverA1System();

    ~RoverA1System() = default;

protected:
    
    void defineRoverDriver() override;

    void updateHwStates(const rclcpp::Time & time) override;

    void updateDriverStateMsg() override;
    
    std::vector<float> getSpeedCmd() const;

    static const inline std::vector<std::string> joints_ = {"fl", "fr", "rl", "rr"};

    // Write operation lock
    std::shared_ptr<std::mutex> rover_driver_write_mtx_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
    
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time last_time_{0, 0, RCL_STEADY_TIME};
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACES_ROVER_SYSTEM_ROVER_A1_SYSTEM_HPP_
