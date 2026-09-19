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

#ifndef ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ROS2_BYTE_PUBLISHER_HPP_
#define ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ROS2_BYTE_PUBLISHER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/context.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "rover_io_context/domain/ports.hpp"

namespace rover::transport::serial
{

// BytePublisherPort over the node's lifecycle publisher: this is what keeps
// InboundByteBridge free of ROS.
class Ros2BytePublisher : public BytePublisherPort
{

public:

    using PublisherPtr =
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8MultiArray>::SharedPtr;

    Ros2BytePublisher(PublisherPtr publisher, rclcpp::Context::SharedPtr context);

    void publish(const std::vector<uint8_t> & buffer, std::size_t length) override;

private:

    PublisherPtr publisher_;
    rclcpp::Context::SharedPtr context_;
};

}  // namespace rover::transport::serial

#endif  // ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ROS2_BYTE_PUBLISHER_HPP_
