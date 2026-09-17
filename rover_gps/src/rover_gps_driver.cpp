// Copyright 2026 Mechatronics Academy
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

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rover_gps/infrastructure/rover_gps_driver_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const auto logger = rclcpp::get_logger("rover_gps_driver");
    int exit_code = 0;

    try {
        // Construction can throw on an invalid parameter override.
        auto rover_gps_driver_node =
            std::make_shared<rover_gps::RoverGpsDriverNode>("rover_gps_driver");

        rclcpp::spin(rover_gps_driver_node->get_node_base_interface());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(logger, "Caught exception: %s", e.what());
        exit_code = 1;
    }

    RCLCPP_INFO(logger, "Shutting down");

    rclcpp::shutdown();

    return exit_code;
}
