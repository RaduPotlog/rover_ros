// Copyright 2025 Mechatronics Academy
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

#include "rover_battery/infrastructure/rover_battery_node.hpp"

#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "rover_battery/domain/battery_classifier.hpp"
#include "rover_battery/domain/bms_frame.hpp"
#include "rover_battery/infrastructure/ros2_battery_state_publisher.hpp"

namespace rover_battery
{

using std::placeholders::_1;

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

}  // namespace

RoverBatteryNode::RoverBatteryNode(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & options)
: Node(node_name, ns, options)
, diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
    const domain::BatteryIdentity defaults;

    auto capacity_descriptor = describe("Design capacity of the pack [Ah].");
    capacity_descriptor.floating_point_range.resize(1);
    capacity_descriptor.floating_point_range[0].from_value = 0.1;
    capacity_descriptor.floating_point_range[0].to_value = 1000.0;
    identity_.design_capacity = static_cast<float>(declare_parameter(
        "design_capacity", static_cast<double>(defaults.design_capacity), capacity_descriptor));

    identity_.serial_number = declare_parameter(
        "serial_number", defaults.serial_number,
        describe("Serial number reported on rover_battery/battery_status."));

    auto timeout_descriptor = describe(
        "Publish a watchdog-expired battery state if no BMS packet arrives within this time [ms].");
    timeout_descriptor.integer_range.resize(1);
    timeout_descriptor.integer_range[0].from_value = 1;
    timeout_descriptor.integer_range[0].to_value = 600000;
    watchdog_timeout_ = std::chrono::milliseconds(
        declare_parameter<std::int64_t>("watchdog_timeout_ms", 10000, timeout_descriptor));
}

void RoverBatteryNode::init()
{
    diagnostic_updater_->setHardwareID("RoverBattery");

    monitor_battery_ = std::make_unique<application::MonitorBatteryUseCase>(
        std::make_shared<infrastructure::Ros2BatteryStatePublisher>(*this, diagnostic_updater_),
        identity_);

    // Timer first: the subscription callback resets it.
    battery_read_timeout_ = create_wall_timer(
        watchdog_timeout_,
        std::bind(&RoverBatteryNode::batteryUdpDataSubscriberTimeoutCallback, this));

    battery_subscriber_ = create_subscription<udp_msgs::msg::UdpPacket>(
        "rover_battery_udp_data", 100,
        std::bind(&RoverBatteryNode::batteryUdpDataCallback, this, _1));
}

void RoverBatteryNode::batteryUdpDataCallback(const udp_msgs::msg::UdpPacket::SharedPtr msg)
{
    // Only a real BMS frame feeds the watchdog: a wrong-size packet or the bridge's no-data
    // payload must let it expire, so a lost BMS still ends in the watchdog state.
    if (msg->data.size() != domain::kBmsPayloadSize) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Packet size mismatch! Received: %zu bytes, Expected: %zu bytes.",
            msg->data.size(), domain::kBmsPayloadSize);
        return;
    }

    domain::BmsFrame frame;
    std::memcpy(&frame.data, msg->data.data(), sizeof(frame.data));
    std::memcpy(&frame.alarms, msg->data.data() + sizeof(frame.data), sizeof(frame.alarms));

    if (domain::isNoDataFrame(frame)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "BMS bridge reports no BMS data (BLE link down or BMS not answering).");
        return;
    }

    battery_read_timeout_->reset();
    monitor_battery_->onFrame(frame);
}

void RoverBatteryNode::batteryUdpDataSubscriberTimeoutCallback()
{
    RCLCPP_WARN(get_logger(), "Battery UDP receiver timeout...");

    monitor_battery_->onDataTimeout();
}

}  // namespace rover_battery
