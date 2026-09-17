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

#include "rover_udp_driver/infrastructure/udp_packet_conversions.hpp"

namespace rover::transport::udp
{

Ros2UdpPacketPublisher::Ros2UdpPacketPublisher(
    PublisherPtr publisher,
    UdpEndpoint endpoint,
    rclcpp::Clock::SharedPtr clock)
: publisher_(std::move(publisher)),
  endpoint_(std::move(endpoint)),
  clock_(std::move(clock))
{
}

void Ros2UdpPacketPublisher::publish(const std::vector<uint8_t> & buffer, std::size_t length)
{
    if (!publisher_) {
        return;
    }

    udp_msgs::msg::UdpPacket out;
    toMsg(buffer, out, length);

    out.header.frame_id = endpoint_.ip();
    out.header.stamp = clock_->now();
    out.address = endpoint_.ip();
    out.src_port = endpoint_.port();

    publisher_->publish(out);
}

}  // namespace rover::transport::udp
