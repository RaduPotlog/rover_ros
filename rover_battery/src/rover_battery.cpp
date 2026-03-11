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

#include <iostream>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "rover_battery/rover_battery_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto rover_battery_node =
        std::make_shared<rover_battery::RoverBatteryNode>("rover_battery_node");

    rover_battery_node->init();
    
    try {
        rclcpp::spin(rover_battery_node);
    } catch (const std::runtime_error & e) {
        std::cerr << "[rover_battery] Caught exception: " << e.what() << std::endl;
    }

    std::cout << "[rover_battery] Shutting down" << std::endl;
    
    rclcpp::shutdown();

    return 0;
}