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

#include "rover_udp_driver/infrastructure/ros2_udp_packet_publisher.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include "rover_udp_driver/infrastructure/udp_packet_conversions.hpp"

namespace rover::transport::udp
{

Ros2UdpPacketPublisher::Ros2UdpPacketPublisher(
    PublisherPtr publisher,
    UdpEndpoint endpoint,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Context::SharedPtr context)
: publisher_(std::move(publisher)),
  endpoint_(std::move(endpoint)),
  clock_(std::move(clock)),
  context_(std::move(context))
{
}

void Ros2UdpPacketPublisher::publish(const std::vector<uint8_t> & buffer, std::size_t length)
{
    // Runs on the ASIO thread, which keeps receiving after Ctrl-C has shut the context down.
    if (!publisher_ || !rclcpp::ok(context_)) {
        return;
    }

    udp_msgs::msg::UdpPacket out;
    toMsg(buffer, out, length);

    out.header.frame_id = endpoint_.ip();
    out.header.stamp = clock_->now();
    out.address = endpoint_.ip();
    out.src_port = endpoint_.port();

    try {
        publisher_->publish(out);
    } catch (const rclcpp::exceptions::RCLError & e) {
        // Never let this escape the ASIO thread: rmw_zenoh also refuses publishes during
        // shutdown while the context is still valid. Drop the datagram.
        if (rclcpp::ok(context_)) {
            RCLCPP_WARN(
                rclcpp::get_logger("Ros2UdpPacketPublisher"), "Dropped datagram, publish failed: %s", e.what());
        }
    }
}

}  // namespace rover::transport::udp
