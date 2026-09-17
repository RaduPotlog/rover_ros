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
// Modified 2026 by Mechatronics Academy: relayouted from
// udp_driver/src/udp_sender_node.cpp (ros-drivers/transport_drivers v1.2.0).

#include "rover_udp_driver/infrastructure/udp_sender_node.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rover_udp_driver/infrastructure/asio_udp_socket.hpp"
#include "rover_udp_driver/infrastructure/udp_packet_conversions.hpp"

namespace rover::transport::udp
{

namespace
{

constexpr const char * kWriteTopic = "udp_write";
constexpr std::size_t kOwnedContextThreads = 1;

}  // namespace

UdpSenderNode::UdpSenderNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("udp_sender_node", options),
  owned_ctx_{std::make_unique<IoContext>(kOwnedContextThreads)},
  ctx_{*owned_ctx_}
{
    declareParameters();
}

UdpSenderNode::UdpSenderNode(
    const rclcpp::NodeOptions & options,
    const IoContext & ctx)
: rclcpp_lifecycle::LifecycleNode("udp_sender_node", options),
  owned_ctx_{nullptr},
  ctx_{ctx}
{
    declareParameters();
}

UdpSenderNode::~UdpSenderNode()
{
    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void UdpSenderNode::declareParameters()
{
    declare_parameter<std::string>("ip", "");
    declare_parameter<int>("port", 0);
}

std::optional<UdpEndpoint> UdpSenderNode::readEndpoint()
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

UdpSenderNode::CallbackReturn UdpSenderNode::on_configure(
    const rclcpp_lifecycle::State & state)
{
    (void)state;

    const auto endpoint = readEndpoint();
    if (!endpoint) {
        return CallbackReturn::FAILURE;
    }

    try {
        socket_ = makeUdpSender(ctx_, *endpoint);
        if (!socket_->isOpen()) {
            socket_->open();
        }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
            get_logger(), "Error creating UDP sender: %s:%u - %s",
            endpoint->ip().c_str(), static_cast<unsigned>(endpoint->port()), ex.what());
        releaseResources();
        return CallbackReturn::FAILURE;
    }

    outbound_ = std::make_unique<OutboundByteBridge>(*socket_);

    subscriber_ = create_subscription<UdpPacket>(
        kWriteTopic,
        rclcpp::QoS(rclcpp::KeepLast(32)).best_effort(),
        [this](const UdpPacket::SharedPtr msg) {subscriberCallback(msg);});

    RCLCPP_DEBUG(get_logger(), "UDP sender successfully configured.");

    return CallbackReturn::SUCCESS;
}

UdpSenderNode::CallbackReturn UdpSenderNode::on_activate(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    outbound_->setActive(true);
    RCLCPP_DEBUG(get_logger(), "UDP sender activated.");
    return CallbackReturn::SUCCESS;
}

UdpSenderNode::CallbackReturn UdpSenderNode::on_deactivate(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    outbound_->setActive(false);
    RCLCPP_DEBUG(get_logger(), "UDP sender deactivated.");
    return CallbackReturn::SUCCESS;
}

UdpSenderNode::CallbackReturn UdpSenderNode::on_cleanup(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    releaseResources();
    RCLCPP_DEBUG(get_logger(), "UDP sender cleaned up.");
    return CallbackReturn::SUCCESS;
}

UdpSenderNode::CallbackReturn UdpSenderNode::on_shutdown(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    releaseResources();
    RCLCPP_DEBUG(get_logger(), "UDP sender shutting down.");
    return CallbackReturn::SUCCESS;
}

void UdpSenderNode::releaseResources()
{
    outbound_.reset();

    if (socket_) {
        socket_->close();
        socket_.reset();
    }

    subscriber_.reset();
}

void UdpSenderNode::subscriberCallback(const UdpPacket::SharedPtr msg)
{
    // The ACTIVE gate lives in OutboundByteBridge now.
    std::vector<uint8_t> out;
    fromMsg(msg, out);
    outbound_->send(out);
}

}  // namespace rover::transport::udp
