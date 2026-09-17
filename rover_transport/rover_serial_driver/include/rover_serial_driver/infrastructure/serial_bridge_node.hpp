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
// serial_driver/serial_bridge_node.hpp (ros-drivers/transport_drivers v1.2.0). The
// parameter parsing moved to the domain layer, the bridging behaviour to
// rover_io_context's application layer, and the rclcpp_components registration was dropped
// in favour of an explicit main().

#ifndef ROVER_SERIAL_DRIVER_INFRASTRUCTURE_SERIAL_BRIDGE_NODE_HPP_
#define ROVER_SERIAL_DRIVER_INFRASTRUCTURE_SERIAL_BRIDGE_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "rover_io_context/application/inbound_byte_bridge.hpp"
#include "rover_io_context/application/outbound_byte_bridge.hpp"
#include "rover_io_context/domain/ports.hpp"
#include "rover_io_context/infrastructure/io_context.hpp"
#include "rover_serial_driver/domain/serial_port_config.hpp"
#include "rover_serial_driver/infrastructure/ros2_byte_publisher.hpp"

namespace rover::transport::serial
{

// Owns a UART and bridges it to two topics:
//   - publishes raw bytes read from the device on `serial_read`;
//   - writes bytes received on `serial_write` back to the device, but only while ACTIVE.
//
// Topic and parameter names are unchanged from upstream - rover_crsf_teleop, and any
// upstream documentation, depend on them.
class SerialBridgeNode final : public rclcpp_lifecycle::LifecycleNode
{

public:

    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;

    explicit SerialBridgeNode(const rclcpp::NodeOptions & options);

    // Shares an externally owned IoContext, so several transport nodes can run on one
    // thread pool.
    SerialBridgeNode(const rclcpp::NodeOptions & options, const IoContext & ctx);

    ~SerialBridgeNode() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:

    // Declared once, in the constructor, so a cleanup -> configure cycle does not
    // re-declare them.
    void declareParameters();

    // Reads the parameters into a config, or returns nullopt after logging why.
    std::optional<SerialPortConfig> readConfig();

    void subscriberCallback(const UInt8MultiArray::SharedPtr msg);

    void releaseResources();

    std::unique_ptr<IoContext> owned_ctx_;
    const IoContext & ctx_;

    std::string device_name_;

    std::unique_ptr<ByteStreamPort> port_;
    std::unique_ptr<Ros2BytePublisher> byte_publisher_;
    std::unique_ptr<InboundByteBridge> inbound_;
    std::unique_ptr<OutboundByteBridge> outbound_;

    rclcpp_lifecycle::LifecyclePublisher<UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<UInt8MultiArray>::SharedPtr subscriber_;
};

}  // namespace rover::transport::serial

#endif  // ROVER_SERIAL_DRIVER_INFRASTRUCTURE_SERIAL_BRIDGE_NODE_HPP_
