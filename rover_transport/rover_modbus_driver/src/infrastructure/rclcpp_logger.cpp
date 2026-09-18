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

#include "rover_modbus_driver/infrastructure/rclcpp_logger.hpp"

namespace rover::transport::modbus
{

RclcppLogger::RclcppLogger(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

void RclcppLogger::warn(const std::string & message)
{
    RCLCPP_WARN_STREAM(logger_, message);
}

void RclcppLogger::error(const std::string & message)
{
    RCLCPP_ERROR_STREAM(logger_, message);
}

}  // namespace rover::transport::modbus
