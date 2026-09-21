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
// Modified 2026 by Mechatronics Academy: relayouted from
// udp_driver/src/udp_receiver_node.cpp (ros-drivers/transport_drivers v1.2.0).

#include "rover_udp_driver/infrastructure/udp_receiver_node.hpp"

#include <memory>
#include <optional>
#include <string>

#include "rover_udp_driver/infrastructure/asio_udp_socket.hpp"

namespace rover::transport::udp
{

namespace
{

constexpr const char * kReadTopic = "udp_read";
constexpr std::size_t kOwnedContextThreads = 1;

}  // namespace

UdpReceiverNode::UdpReceiverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("udp_receiver_node", options),
  owned_ctx_{std::make_unique<IoContext>(kOwnedContextThreads)},
  ctx_{*owned_ctx_}
{
    declareParameters();
}

UdpReceiverNode::UdpReceiverNode(
    const rclcpp::NodeOptions & options,
    const IoContext & ctx)
: rclcpp_lifecycle::LifecycleNode("udp_receiver_node", options),
  owned_ctx_{nullptr},
  ctx_{ctx}
{
    declareParameters();
}

UdpReceiverNode::~UdpReceiverNode()
{
    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void UdpReceiverNode::declareParameters()
{
    declare_parameter<std::string>("ip", "");
    declare_parameter<int>("port", 0);
}

std::optional<UdpEndpoint> UdpReceiverNode::readEndpoint()
{
    std::string error;
    auto endpoint = UdpEndpoint::fromParameters(
        get_parameter("ip").as_string(),
        static_cast<int>(get_parameter("port").as_int()),
        error);

    if (!endpoint) {
        RCLCPP_ERROR(get_logger(), "Invalid UDP endpoint: %s", error.c_str());
        return std::nullopt;
    }

    RCLCPP_INFO(
        get_logger(), "ip: %s, port: %u", endpoint->ip().c_str(),
        static_cast<unsigned>(endpoint->port()));

    return endpoint;
}

UdpReceiverNode::CallbackReturn UdpReceiverNode::on_configure(
    const rclcpp_lifecycle::State & state)
{
    (void)state;

    const auto endpoint = readEndpoint();
    if (!endpoint) {
        return CallbackReturn::FAILURE;
    }

    publisher_ = create_publisher<UdpPacket>(kReadTopic, rclcpp::QoS(100));

    try {
        socket_ = makeUdpReceiver(ctx_, *endpoint);
        socket_->open();
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
            get_logger(), "Error creating UDP receiver: %s:%u - %s",
            endpoint->ip().c_str(), static_cast<unsigned>(endpoint->port()), ex.what());
        releaseResources();
        return CallbackReturn::FAILURE;
    }

    packet_publisher_ = std::make_unique<Ros2UdpPacketPublisher>(
        publisher_, *endpoint, get_clock(), get_node_base_interface()->get_context());
    inbound_ = std::make_unique<InboundByteBridge>(*socket_, *packet_publisher_);
    inbound_->start();

    RCLCPP_DEBUG(get_logger(), "UDP receiver successfully configured.");

    return CallbackReturn::SUCCESS;
}

UdpReceiverNode::CallbackReturn UdpReceiverNode::on_activate(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    publisher_->on_activate();
    RCLCPP_DEBUG(get_logger(), "UDP receiver activated.");
    return CallbackReturn::SUCCESS;
}

UdpReceiverNode::CallbackReturn UdpReceiverNode::on_deactivate(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    publisher_->on_deactivate();
    RCLCPP_DEBUG(get_logger(), "UDP receiver deactivated.");
    return CallbackReturn::SUCCESS;
}

UdpReceiverNode::CallbackReturn UdpReceiverNode::on_cleanup(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    releaseResources();
    RCLCPP_DEBUG(get_logger(), "UDP receiver cleaned up.");
    return CallbackReturn::SUCCESS;
}

UdpReceiverNode::CallbackReturn UdpReceiverNode::on_shutdown(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    releaseResources();
    RCLCPP_DEBUG(get_logger(), "UDP receiver shutting down.");
    return CallbackReturn::SUCCESS;
}

void UdpReceiverNode::releaseResources()
{
    // Close first so the ASIO thread stops feeding the bridge before it is destroyed.
    if (socket_) {
        socket_->close();
    }

    inbound_.reset();
    packet_publisher_.reset();
    socket_.reset();

    publisher_.reset();
}

}  // namespace rover::transport::udp
