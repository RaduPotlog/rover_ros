// Copyright 2021 LeoDrive.
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
// Developed by LeoDrive, 2021
//
// Modified 2026 by Mechatronics Academy: relayouted from udp_driver/udp_sender_node.hpp
// (ros-drivers/transport_drivers v1.2.0).

#ifndef ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_SENDER_NODE_HPP_
#define ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_SENDER_NODE_HPP_

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "rover_io_context/application/outbound_byte_bridge.hpp"
#include "rover_io_context/domain/ports.hpp"
#include "rover_io_context/infrastructure/io_context.hpp"
#include "rover_udp_driver/domain/udp_endpoint.hpp"

namespace rover::transport::udp
{

// Sends every packet received on `udp_write` to the configured endpoint, but only while
// the node is active.
class UdpSenderNode final : public rclcpp_lifecycle::LifecycleNode
{

public:

    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using UdpPacket = udp_msgs::msg::UdpPacket;

    explicit UdpSenderNode(const rclcpp::NodeOptions & options);

    UdpSenderNode(const rclcpp::NodeOptions & options, const IoContext & ctx);

    ~UdpSenderNode() override;

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

    void subscriberCallback(const UdpPacket::SharedPtr msg);

    void releaseResources();

    std::unique_ptr<IoContext> owned_ctx_;
    const IoContext & ctx_;

    std::unique_ptr<ByteStreamPort> socket_;
    std::unique_ptr<OutboundByteBridge> outbound_;

    rclcpp::Subscription<UdpPacket>::SharedPtr subscriber_;

    rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace rover::transport::udp

#endif  // ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_SENDER_NODE_HPP_
