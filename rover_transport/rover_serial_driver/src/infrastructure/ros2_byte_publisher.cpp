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

#include "rover_serial_driver/infrastructure/ros2_byte_publisher.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include "rover_serial_driver/infrastructure/serial_msg_conversions.hpp"

namespace rover::transport::serial
{

Ros2BytePublisher::Ros2BytePublisher(PublisherPtr publisher, rclcpp::Context::SharedPtr context)
: publisher_(std::move(publisher)),
  context_(std::move(context))
{
}

void Ros2BytePublisher::publish(const std::vector<uint8_t> & buffer, std::size_t length)
{
    // Runs on the ASIO thread, which keeps receiving after Ctrl-C has shut the context down.
    if (!publisher_ || !rclcpp::ok(context_)) {
        return;
    }

    std_msgs::msg::UInt8MultiArray out;
    toMsg(buffer, out, length);
    try {
        publisher_->publish(out);
    } catch (const rclcpp::exceptions::RCLError & e) {
        // Never let this escape the ASIO thread: rmw_zenoh also refuses publishes during
        // shutdown while the context is still valid. Drop the bytes.
        if (rclcpp::ok(context_)) {
            RCLCPP_WARN(
                rclcpp::get_logger("Ros2BytePublisher"), "Dropped bytes, publish failed: %s", e.what());
        }
    }
}

}  // namespace rover::transport::serial
