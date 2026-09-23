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
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_msgs/srv/set_led_animation.hpp"

#include "rover_safety/led_safety_node.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using lifecycle_msgs::msg::State;
using SetLedAnimationSrv = rover_msgs::srv::SetLedAnimation;

namespace
{

/**
 * Runs the real rover_led_safety_node in-process, autostarted before its led/set_animation server
 * exists - the bringup race that used to leave it unconfigured for good.
 */
class LedSafetyConfigureRetryTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        helper_node_ = std::make_shared<rclcpp::Node>("led_safety_retry_test_helper");
        diagnostics_sub_ = helper_node_->create_subscription<DiagnosticArray>(
            "/diagnostics", 10, [this](const DiagnosticArray::SharedPtr msg) {
                const std::lock_guard<std::mutex> lock(mutex_);
                for (const auto & status : msg->status) {
                    latest_[status.name] = status;
                }
            });
        helper_executor_.add_node(helper_node_);
        helper_thread_ = std::thread([this]() { helper_executor_.spin(); });

        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"bt_project_path",
             std::string(ROVER_SAFETY_SOURCE_DIR "/behavior_trees/RoverLedSafetyBT.btproj")},
            {"plugin_libs", std::vector<std::string>{"tick_after_timeout_bt_node"}},
            {"ros_plugin_libs", std::vector<std::string>{"call_set_led_animation_service_bt_node"}},
            {"bt_server_port", 15555},
            {"autostart_node", true},
            {"configure_retry_period", 1.0},
        });

        led_node_ = std::make_shared<rover_safety::LedSafetyNode>("rover_led_safety_node", options);
        led_executor_.add_node(led_node_->get_node_base_interface());
        led_thread_ = std::thread([this]() { led_executor_.spin(); });
    }

    void TearDown() override
    {
        led_executor_.cancel();
        led_thread_.join();
        led_node_.reset();
        helper_executor_.cancel();
        helper_thread_.join();
    }

    /** Polls `condition` until it holds or `timeout` passes. */
    static bool waitFor(const std::function<bool()> & condition, std::chrono::seconds timeout)
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

    /** Latest status whose name ends with `suffix` (the updater prefixes the node name). */
    std::optional<DiagnosticStatus> latestStatus(const std::string & suffix)
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & [name, status] : latest_) {
            if (name.size() >= suffix.size() &&
                name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
                return status;
            }
        }
        return std::nullopt;
    }

    uint8_t ledState() const { return led_node_->get_current_state().id(); }

    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Service<SetLedAnimationSrv>::SharedPtr led_service_;
    rclcpp::executors::SingleThreadedExecutor helper_executor_;
    std::thread helper_thread_;

    std::mutex mutex_;
    std::map<std::string, DiagnosticStatus> latest_;

    std::shared_ptr<rover_safety::LedSafetyNode> led_node_;
    rclcpp::executors::SingleThreadedExecutor led_executor_;
    std::thread led_thread_;
};

}  // namespace

TEST_F(LedSafetyConfigureRetryTest, ConfiguresOnceTheLedServiceAppears)
{
    // Without led/set_animation the tree cannot be built: the node reports why and stays
    // unconfigured and unsubscribed.
    ASSERT_TRUE(waitFor([this]() {
        const auto tree = latestStatus("LED safety behavior tree");
        return tree && tree->level == DiagnosticStatus::ERROR;
    }, 30s));
    EXPECT_EQ(ledState(), State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_NE(latestStatus("LED safety behavior tree")->message.find("led/set_animation"), std::string::npos)
        << latestStatus("LED safety behavior tree")->message;

    ASSERT_TRUE(waitFor([this]() { return latestStatus("LED safety inputs").has_value(); }, 10s));
    EXPECT_NE(latestStatus("LED safety inputs")->message.find("not subscribed"), std::string::npos)
        << latestStatus("LED safety inputs")->message;

    // The LED controller comes up late: a retry picks it up and the node activates.
    led_service_ = helper_node_->create_service<SetLedAnimationSrv>(
        "led/set_animation",
        [](const SetLedAnimationSrv::Request::SharedPtr, SetLedAnimationSrv::Response::SharedPtr response) {
            response->success = true;
        });

    ASSERT_TRUE(waitFor([this]() { return ledState() == State::PRIMARY_STATE_ACTIVE; }, 30s))
        << "state: " << static_cast<int>(ledState());

    EXPECT_TRUE(waitFor([this]() {
        const auto tree = latestStatus("LED safety behavior tree");
        return tree && tree->level != DiagnosticStatus::ERROR;
    }, 10s));
}
