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

#ifndef ROVER_UDP_DRIVER_INFRASTRUCTURE_ROS2_UDP_PACKET_PUBLISHER_HPP_
#define ROVER_UDP_DRIVER_INFRASTRUCTURE_ROS2_UDP_PACKET_PUBLISHER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/clock.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "rover_io_context/domain/ports.hpp"
#include "rover_udp_driver/domain/udp_endpoint.hpp"

namespace rover::transport::udp
{

// BytePublisherPort over the receiver node's lifecycle publisher. Also does the packet
// shaping (frame_id / stamp / address / src_port) that upstream had inline in
// UdpReceiverNode::receiver_callback - it needs the ROS clock, so it belongs here rather
// than in the conversion free functions.
class Ros2UdpPacketPublisher : public BytePublisherPort
{

public:

    using PublisherPtr =
        rclcpp_lifecycle::LifecyclePublisher<udp_msgs::msg::UdpPacket>::SharedPtr;

    Ros2UdpPacketPublisher(
        PublisherPtr publisher,
        UdpEndpoint endpoint,
        rclcpp::Clock::SharedPtr clock,
        rclcpp::Context::SharedPtr context);

    void publish(const std::vector<uint8_t> & buffer, std::size_t length) override;

private:

    PublisherPtr publisher_;
    UdpEndpoint endpoint_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Context::SharedPtr context_;
};

}  // namespace rover::transport::udp

#endif  // ROVER_UDP_DRIVER_INFRASTRUCTURE_ROS2_UDP_PACKET_PUBLISHER_HPP_
