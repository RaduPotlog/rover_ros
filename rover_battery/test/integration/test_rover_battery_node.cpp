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

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/charging_status.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

#include "rover_battery/domain/bms_frame.hpp"
#include "rover_battery/infrastructure/rover_battery_node.hpp"

using namespace std::chrono_literals;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using ChargingStatusMsg = rover_msgs::msg::ChargingStatus;
using UdpPacketMsg = udp_msgs::msg::UdpPacket;

namespace
{

std::vector<uint8_t> makePacket(float soc, int status, int cells)
{
    rover_battery::domain::BmsFrame frame;
    frame.data.packVoltage = 52.0f;
    frame.data.packCurrent = 4.0f;
    frame.data.packSOC = soc;
    frame.data.chargeDischargeStatus = status;
    frame.data.numberOfCells = cells;
    frame.data.numOfTempSensors = 2;

    std::vector<uint8_t> bytes(rover_battery::domain::kBmsPayloadSize);
    std::memcpy(bytes.data(), &frame.data, sizeof(frame.data));
    std::memcpy(bytes.data() + sizeof(frame.data), &frame.alarms, sizeof(frame.alarms));
    return bytes;
}

class RoverBatteryNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
    static void TearDownTestSuite() {rclcpp::shutdown();}

    void startNode(const std::vector<rclcpp::Parameter> & overrides)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(overrides);
        battery_node_ = std::make_shared<rover_battery::RoverBatteryNode>(
            "rover_battery_node", "/rover_battery_test", options);
        battery_node_->init();

        tester_ = std::make_shared<rclcpp::Node>("tester", "/rover_battery_test");
        udp_pub_ = tester_->create_publisher<UdpPacketMsg>("/rover_battery_udp_data", 100);
        battery_sub_ = tester_->create_subscription<BatteryStateMsg>(
            "rover_battery/battery_status", 10,
            [this](BatteryStateMsg::SharedPtr msg) {battery_msgs_.push_back(*msg);});
        charging_sub_ = tester_->create_subscription<ChargingStatusMsg>(
            "rover_battery/charging_status", 10,
            [this](ChargingStatusMsg::SharedPtr msg) {charging_msgs_.push_back(*msg);});

        executor_.add_node(battery_node_);
        executor_.add_node(tester_);
    }

    void TearDown() override
    {
        executor_.remove_node(tester_);
        executor_.remove_node(battery_node_);
    }

    /** Spins until `done` returns true or `timeout` elapses; returns `done()`. */
    bool spinUntil(
        const std::function<bool()> & done, std::chrono::milliseconds timeout,
        const std::function<void()> & each_iteration = {})
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            if (each_iteration) {
                each_iteration();
            }
            executor_.spin_some(50ms);
        }
        return done();
    }

    bool waitForDiscovery()
    {
        return spinUntil(
            [this] {
                return udp_pub_->get_subscription_count() > 0 &&
                       battery_sub_->get_publisher_count() > 0 &&
                       charging_sub_->get_publisher_count() > 0;
            }, 5s);
    }

    void publishPacket(const std::vector<uint8_t> & bytes)
    {
        UdpPacketMsg msg;
        msg.data = bytes;
        udp_pub_->publish(msg);
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rover_battery::RoverBatteryNode> battery_node_;
    rclcpp::Node::SharedPtr tester_;
    rclcpp::Publisher<UdpPacketMsg>::SharedPtr udp_pub_;
    rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
    rclcpp::Subscription<ChargingStatusMsg>::SharedPtr charging_sub_;
    std::vector<BatteryStateMsg> battery_msgs_;
    std::vector<ChargingStatusMsg> charging_msgs_;
};

}  // namespace

TEST_F(RoverBatteryNodeTest, PublishesStateForValidPacket)
{
    startNode({rclcpp::Parameter("serial_number", "TEST-SN")});
    ASSERT_TRUE(waitForDiscovery());

    const auto packet = makePacket(60.0f, 1, 8);
    ASSERT_TRUE(spinUntil(
        [this] {return !battery_msgs_.empty() && !charging_msgs_.empty();}, 5s,
        [&] {publishPacket(packet);}));

    const auto & state = battery_msgs_.front();
    EXPECT_TRUE(state.present);
    EXPECT_FLOAT_EQ(state.percentage, 0.6f);
    EXPECT_FLOAT_EQ(state.voltage, 52.0f);
    EXPECT_EQ(state.power_supply_status, BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING);
    EXPECT_EQ(state.power_supply_health, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);
    EXPECT_EQ(state.cell_voltage.size(), 8u);
    EXPECT_EQ(state.serial_number, "TEST-SN");

    const auto & charging = charging_msgs_.front();
    EXPECT_TRUE(charging.charging);
    EXPECT_EQ(charging.charger_type, ChargingStatusMsg::WIRED);
    EXPECT_FLOAT_EQ(charging.current, 4.0f);
}

TEST_F(RoverBatteryNodeTest, IgnoresPacketWithWrongSize)
{
    startNode({});
    ASSERT_TRUE(waitForDiscovery());

    auto bad_packet = makePacket(60.0f, 1, 8);
    bad_packet.pop_back();
    spinUntil([] {return false;}, 500ms, [&] {publishPacket(bad_packet);});
    EXPECT_TRUE(battery_msgs_.empty());
    EXPECT_TRUE(charging_msgs_.empty());

    // The same node still answers a valid packet.
    const auto good_packet = makePacket(60.0f, 1, 8);
    ASSERT_TRUE(spinUntil(
        [this] {return !battery_msgs_.empty();}, 5s, [&] {publishPacket(good_packet);}));
    EXPECT_TRUE(battery_msgs_.front().present);
}

TEST_F(RoverBatteryNodeTest, PublishesWatchdogStateWithoutData)
{
    startNode({rclcpp::Parameter("watchdog_timeout_ms", 200)});
    ASSERT_TRUE(waitForDiscovery());

    ASSERT_TRUE(spinUntil([this] {return !battery_msgs_.empty();}, 5s));

    const auto & state = battery_msgs_.front();
    EXPECT_FALSE(state.present);
    EXPECT_EQ(state.power_supply_health,
              BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE);
}

TEST(RoverBatteryNodeParameters, RejectsOutOfRangeValues)
{
    rclcpp::init(0, nullptr);

    for (const auto & parameter : {rclcpp::Parameter("watchdog_timeout_ms", 0),
                                   rclcpp::Parameter("design_capacity", 0.0)})
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({parameter});
        EXPECT_THROW(
            rover_battery::RoverBatteryNode("rover_battery_node", "/", options),
            rclcpp::exceptions::InvalidParameterValueException) << parameter.get_name();
    }

    rclcpp::shutdown();
}
