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
// serial_driver/src/serial_bridge_node.cpp (ros-drivers/transport_drivers v1.2.0).

#include "rover_serial_driver/infrastructure/serial_bridge_node.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rover_serial_driver/infrastructure/asio_serial_port.hpp"
#include "rover_serial_driver/infrastructure/serial_msg_conversions.hpp"

namespace rover::transport::serial
{

namespace
{

constexpr const char * kReadTopic = "serial_read";
constexpr const char * kWriteTopic = "serial_write";
constexpr std::size_t kOwnedContextThreads = 2;

}  // namespace

SerialBridgeNode::SerialBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("serial_bridge_node", options),
  owned_ctx_{std::make_unique<IoContext>(kOwnedContextThreads)},
  ctx_{*owned_ctx_}
{
    declareParameters();
}

SerialBridgeNode::SerialBridgeNode(
    const rclcpp::NodeOptions & options,
    const IoContext & ctx)
: rclcpp_lifecycle::LifecycleNode("serial_bridge_node", options),
  owned_ctx_{nullptr},
  ctx_{ctx}
{
    declareParameters();
}

SerialBridgeNode::~SerialBridgeNode()
{
    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void SerialBridgeNode::declareParameters()
{
    declare_parameter<std::string>("device_name", "");
    declare_parameter<int>("baud_rate", 0);
    declare_parameter<std::string>("flow_control", "");
    declare_parameter<std::string>("parity", "");
    declare_parameter<std::string>("stop_bits", "");
}

std::optional<SerialPortConfig> SerialBridgeNode::readConfig()
{
    device_name_ = get_parameter("device_name").as_string();
    if (device_name_.empty()) {
        RCLCPP_ERROR(get_logger(), "device_name must not be empty");
        return std::nullopt;
    }

    std::string error;
    auto config = SerialPortConfig::fromStrings(
        static_cast<int>(get_parameter("baud_rate").as_int()),
        get_parameter("flow_control").as_string(),
        get_parameter("parity").as_string(),
        get_parameter("stop_bits").as_string(),
        error);

    if (!config) {
        RCLCPP_ERROR(get_logger(), "Invalid serial configuration: %s", error.c_str());
        return std::nullopt;
    }

    return config;
}

SerialBridgeNode::CallbackReturn SerialBridgeNode::on_configure(
    const rclcpp_lifecycle::State & state)
{
    (void)state;

    const auto config = readConfig();
    if (!config) {
        return CallbackReturn::FAILURE;
    }

    publisher_ = create_publisher<UInt8MultiArray>(kReadTopic, rclcpp::QoS{100});

    try {
        port_ = makeSerialPort(ctx_, device_name_, *config);
        port_->open();
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
            get_logger(), "Error creating serial port: %s - %s",
            device_name_.c_str(), ex.what());
        releaseResources();
        return CallbackReturn::FAILURE;
    }

    byte_publisher_ = std::make_unique<Ros2BytePublisher>(publisher_);
    inbound_ = std::make_unique<InboundByteBridge>(*port_, *byte_publisher_);
    outbound_ = std::make_unique<OutboundByteBridge>(*port_);
    inbound_->start();

    subscriber_ = create_subscription<UInt8MultiArray>(
        kWriteTopic,
        rclcpp::QoS(rclcpp::KeepLast(32)).best_effort(),
        [this](const UInt8MultiArray::SharedPtr msg) {subscriberCallback(msg);});

    RCLCPP_DEBUG(get_logger(), "Serial port successfully configured.");

    return CallbackReturn::SUCCESS;
}

SerialBridgeNode::CallbackReturn SerialBridgeNode::on_activate(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    publisher_->on_activate();
    outbound_->setActive(true);
    RCLCPP_DEBUG(get_logger(), "Serial bridge activated.");
    return CallbackReturn::SUCCESS;
}

SerialBridgeNode::CallbackReturn SerialBridgeNode::on_deactivate(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    outbound_->setActive(false);
    publisher_->on_deactivate();
    RCLCPP_DEBUG(get_logger(), "Serial bridge deactivated.");
    return CallbackReturn::SUCCESS;
}

SerialBridgeNode::CallbackReturn SerialBridgeNode::on_cleanup(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    releaseResources();
    RCLCPP_DEBUG(get_logger(), "Serial bridge cleaned up.");
    return CallbackReturn::SUCCESS;
}

SerialBridgeNode::CallbackReturn SerialBridgeNode::on_shutdown(
    const rclcpp_lifecycle::State & state)
{
    (void)state;
    releaseResources();
    RCLCPP_DEBUG(get_logger(), "Serial bridge shutting down.");
    return CallbackReturn::SUCCESS;
}

void SerialBridgeNode::releaseResources()
{
    // Order matters: the bridges hold references to the port and the publisher adapter,
    // so they go first.
    inbound_.reset();
    outbound_.reset();
    byte_publisher_.reset();

    if (port_) {
        port_->close();
        port_.reset();
    }

    subscriber_.reset();
    publisher_.reset();
}

void SerialBridgeNode::subscriberCallback(const UInt8MultiArray::SharedPtr msg)
{
    // The ACTIVE gate lives in OutboundByteBridge now, so this callback does not need to
    // inspect the lifecycle state.
    std::vector<uint8_t> out;
    fromMsg(msg, out);
    outbound_->send(out);
}

}  // namespace rover::transport::serial
