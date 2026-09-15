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

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rover_msgs/msg/gpio_state.hpp"
#include "rover_msgs/msg/system_status.hpp"

#include "rover_safety/safety_node.hpp"

using namespace std::chrono_literals;
using TriggerSrv = std_srvs::srv::Trigger;

namespace
{

/**
 * Runs the real rover_safety_node in-process against a fake hardware interface. The power-off
 * command is replaced by one that writes ROVER_SHUTDOWN_REASON to a file.
 */
class SafetyNodeShutdownTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
        reason_file_ = std::filesystem::temp_directory_path() /
                       (std::string("rover_safety_node_") + test_info->name() + "_reason.txt");
        std::filesystem::remove(reason_file_);

        hardware_node_ = std::make_shared<rclcpp::Node>("fake_hardware_interface");
        e_stop_service_ = hardware_node_->create_service<TriggerSrv>(
            "hardware_interface/sw_user_e_stop_set",
            [this](const TriggerSrv::Request::SharedPtr, TriggerSrv::Response::SharedPtr response) {
                ++e_stop_calls_;
                response->success = true;
            });
        hardware_executor_.add_node(hardware_node_);
        hardware_thread_ = std::thread([this]() { hardware_executor_.spin(); });

        client_node_ = std::make_shared<rclcpp::Node>("shutdown_test_client");
        shutdown_client_ = client_node_->create_client<TriggerSrv>("rover_safety_node/shutdown");
    }

    void TearDown() override
    {
        if (safety_thread_.joinable()) {
            safety_executor_.cancel();
            safety_thread_.join();
        }
        safety_node_.reset();
        hardware_executor_.cancel();
        hardware_thread_.join();
        std::filesystem::remove(reason_file_);
    }

    void startSafetyNode(const std::string & power_off_command, double retry_backoff = 30.0)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"bt_project_path", std::string(ROVER_SAFETY_SOURCE_DIR "/behavior_trees/RoverSafetyBT.btproj")},
            {"shutdown_hosts_path", std::string(ROVER_SAFETY_SOURCE_DIR "/config/shutdown_hosts.yaml")},
            {"plugin_libs", std::vector<std::string>{
                "execute_command_bt_node", "shutdown_hosts_from_file_bt_node", "signal_shutdown_bt_node",
                "tick_after_timeout_bt_node"}},
            {"ros_plugin_libs", std::vector<std::string>{
                "call_set_bool_service_bt_node", "call_trigger_service_bt_node"}},
            {"timer_frequency", 20.0},
            {"bt_server_port", 16666},
            {"shutdown.bt_server_port", 17777},
            {"shutdown.command", power_off_command},
            {"shutdown.command_timeout", 5.0},
            {"shutdown.retry_backoff", retry_backoff},
        });

        safety_node_ = std::make_shared<rover_safety::SafetyNode>("rover_safety_node", options);
        ASSERT_EQ(
            safety_node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

        safety_executor_.add_node(safety_node_->get_node_base_interface());
        safety_executor_.add_node(client_node_);
        safety_thread_ = std::thread([this]() { safety_executor_.spin(); });
    }

    std::string writeReasonCommand() const
    {
        return "printf '%s' \"$ROVER_SHUTDOWN_REASON\" > '" + reason_file_.string() + "'";
    }

    TriggerSrv::Response::SharedPtr callShutdown()
    {
        EXPECT_TRUE(shutdown_client_->wait_for_service(5s));
        auto future = shutdown_client_->async_send_request(std::make_shared<TriggerSrv::Request>());
        EXPECT_EQ(future.wait_for(5s), std::future_status::ready);
        return future.get();
    }

    /** Polls `condition` until it holds or `timeout` passes. */
    static bool waitFor(const std::function<bool()> & condition, std::chrono::seconds timeout = 15s)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (condition()) {
                return true;
            }
            std::this_thread::sleep_for(20ms);
        }
        return condition();
    }

    std::string readReasonFile() const
    {
        std::ifstream file(reason_file_);
        return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    }

    std::filesystem::path reason_file_;
    std::atomic<int> e_stop_calls_{0};

    rclcpp::Node::SharedPtr hardware_node_;
    rclcpp::Service<TriggerSrv>::SharedPtr e_stop_service_;
    rclcpp::executors::SingleThreadedExecutor hardware_executor_;
    std::thread hardware_thread_;

    std::shared_ptr<rover_safety::SafetyNode> safety_node_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Client<TriggerSrv>::SharedPtr shutdown_client_;
    rclcpp::executors::SingleThreadedExecutor safety_executor_;
    std::thread safety_thread_;
};

}  // namespace

TEST_F(SafetyNodeShutdownTest, ServiceRequestTripsEStopAndPowersOffOnce)
{
    startSafetyNode(writeReasonCommand());

    const auto first = callShutdown();
    EXPECT_TRUE(first->success) << first->message;

    const auto second = callShutdown();
    EXPECT_FALSE(second->success);

    ASSERT_TRUE(waitFor([this]() { return std::filesystem::exists(reason_file_); }));
    EXPECT_TRUE(waitFor([this]() { return readReasonFile() == "Requested via the ~/shutdown service."; }))
        << readReasonFile();
    EXPECT_EQ(e_stop_calls_.load(), 1);

    // Power-off accepted: later requests are refused.
    ASSERT_TRUE(waitFor([this]() { return callShutdown()->message.find("powering off") != std::string::npos; }));
    EXPECT_EQ(e_stop_calls_.load(), 1);
}

TEST_F(SafetyNodeShutdownTest, FailedPowerOffIsRetriedAfterBackoff)
{
    const auto marker = reason_file_.string() + ".attempts";
    std::filesystem::remove(marker);
    // Fails on the first attempt, succeeds on the second.
    startSafetyNode(
        "if [ -e '" + marker + "' ]; then " + writeReasonCommand() + "; else touch '" + marker + "'; exit 3; fi",
        1.0);

    EXPECT_TRUE(callShutdown()->success);

    // In progress, then refused during the backoff once the command failed.
    ASSERT_TRUE(waitFor([this]() { return callShutdown()->message.find("retry") != std::string::npos; }));
    EXPECT_FALSE(std::filesystem::exists(reason_file_));

    ASSERT_TRUE(waitFor([this]() { return callShutdown()->success; }));
    EXPECT_TRUE(waitFor([this]() { return std::filesystem::exists(reason_file_); }));
    EXPECT_EQ(e_stop_calls_.load(), 2);
    std::filesystem::remove(marker);
}

TEST_F(SafetyNodeShutdownTest, FatalBatteryTemperatureShutsDown)
{
    startSafetyNode(writeReasonCommand());

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto battery_pub = hardware_node_->create_publisher<sensor_msgs::msg::BatteryState>(
        "rover_battery/battery_status", qos);
    auto gpio_pub = hardware_node_->create_publisher<rover_msgs::msg::GpioState>(
        "hardware_interface/gpio_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    auto system_pub = hardware_node_->create_publisher<rover_msgs::msg::SystemStatus>("system_status", qos);

    sensor_msgs::msg::BatteryState battery;
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    battery.temperature = 75.0f;  // above the default fatal limit of 60 C

    gpio_pub->publish(rover_msgs::msg::GpioState());

    // Published until the tree reacts: the subscriptions may not be matched yet.
    ASSERT_TRUE(waitFor([&]() {
        battery_pub->publish(battery);
        system_pub->publish(rover_msgs::msg::SystemStatus());
        return std::filesystem::exists(reason_file_);
    }));
    EXPECT_TRUE(waitFor([this]() { return readReasonFile() == "Fatal battery temperature"; }))
        << readReasonFile();
    EXPECT_EQ(e_stop_calls_.load(), 1);
}
