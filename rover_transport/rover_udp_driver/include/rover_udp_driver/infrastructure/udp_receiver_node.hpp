// Copyright 2021 LeoDrive, Copyright 2021 the Autoware Foundation
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
//
// Modified 2026 by Mechatronics Academy: relayouted from udp_driver/udp_receiver_node.hpp
// (ros-drivers/transport_drivers v1.2.0).

#ifndef ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_RECEIVER_NODE_HPP_
#define ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_RECEIVER_NODE_HPP_

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "rover_io_context/application/inbound_byte_bridge.hpp"
#include "rover_io_context/domain/ports.hpp"
#include "rover_io_context/infrastructure/io_context.hpp"
#include "rover_udp_driver/domain/udp_endpoint.hpp"
#include "rover_udp_driver/infrastructure/ros2_udp_packet_publisher.hpp"

namespace rover::transport::udp
{

// Binds a UDP socket and republishes every datagram on `udp_read`.
class UdpReceiverNode final : public rclcpp_lifecycle::LifecycleNode
{

public:

    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using UdpPacket = udp_msgs::msg::UdpPacket;

    explicit UdpReceiverNode(const rclcpp::NodeOptions & options);

    UdpReceiverNode(const rclcpp::NodeOptions & options, const IoContext & ctx);

    ~UdpReceiverNode() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:

    void declareParameters();

    // With the `autostart` parameter, configures and activates the node from the executor right
    // after construction - for component containers, where launch_ros' ComposableLifecycleNode
    // autostart misses the namespace and never reaches the node.
    void scheduleAutostart();

    std::optional<UdpEndpoint> readEndpoint();

    void releaseResources();

    std::unique_ptr<IoContext> owned_ctx_;
    const IoContext & ctx_;

    std::unique_ptr<ByteStreamPort> socket_;
    std::unique_ptr<Ros2UdpPacketPublisher> packet_publisher_;
    std::unique_ptr<InboundByteBridge> inbound_;

    rclcpp_lifecycle::LifecyclePublisher<UdpPacket>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace rover::transport::udp

#endif  // ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_RECEIVER_NODE_HPP_
