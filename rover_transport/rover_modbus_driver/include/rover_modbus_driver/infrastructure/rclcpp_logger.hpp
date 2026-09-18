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

#ifndef ROVER_MODBUS_DRIVER_INFRASTRUCTURE_RCLCPP_LOGGER_HPP_
#define ROVER_MODBUS_DRIVER_INFRASTRUCTURE_RCLCPP_LOGGER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rover_modbus_driver/domain/logger_port.hpp"

namespace rover::transport::modbus
{

// Routes the client's log lines to /rosout. The only rclcpp dependency in this package,
// and it lives in infrastructure so the _core library stays ROS-free.
class RclcppLogger : public LoggerPort
{

public:

    explicit RclcppLogger(rclcpp::Logger logger);

    void warn(const std::string & message) override;

    void error(const std::string & message) override;

private:

    rclcpp::Logger logger_;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_INFRASTRUCTURE_RCLCPP_LOGGER_HPP_
