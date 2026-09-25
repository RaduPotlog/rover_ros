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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "rover_msgs/msg/led_animation.hpp"
#include "rover_msgs/msg/safety_status.hpp"
#include "rover_msgs/srv/set_led_animation.hpp"

#include "rover_safety/led_safety_node.hpp"

using namespace std::chrono_literals;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using LedAnimationMsg = rover_msgs::msg::LedAnimation;
using SafetyStatusMsg = rover_msgs::msg::SafetyStatus;
using SetLedAnimationSrv = rover_msgs::srv::SetLedAnimation;
using lifecycle_msgs::msg::State;

namespace
{

/**
 * Runs the real rover_led_safety_node in-process with the shipped tree, against a fake LED
 * controller that records every led/set_animation request and accepts it.
 */
class LedSafetyNodeAnimationsTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        helper_node_ = std::make_shared<rclcpp::Node>("led_safety_animations_test_helper");
        led_service_ = helper_node_->create_service<SetLedAnimationSrv>(
            "led/set_animation",
            [this](const SetLedAnimationSrv::Request::SharedPtr request,
                   SetLedAnimationSrv::Response::SharedPtr response) {
                {
                    const std::lock_guard<std::mutex> lock(mutex_);
                    requests_.push_back(*request);
                }
                response->success = true;
            });
        battery_pub_ = helper_node_->create_publisher<BatteryStateMsg>("rover_battery/battery_status", 10);
        safety_pub_ = helper_node_->create_publisher<SafetyStatusMsg>(
            "hardware_interface/safety_status",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
        publish_timer_ = helper_node_->create_wall_timer(100ms, [this]() { publishInputs(); });
        helper_executor_.add_node(helper_node_);
        helper_thread_ = std::thread([this]() { helper_executor_.spin(); });

        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"bt_project_path",
             std::string(ROVER_SAFETY_SOURCE_DIR "/behavior_trees/RoverLedSafetyBT.btproj")},
            {"plugin_libs", std::vector<std::string>{"tick_after_timeout_bt_node"}},
            {"ros_plugin_libs", std::vector<std::string>{"call_set_led_animation_service_bt_node"}},
            {"bt_server_port", 15565},
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

    /** 10 Hz, like rover_battery and the hardware interface. */
    void publishInputs()
    {
        if (!publishing_) {
            return;
        }

        BatteryStateMsg battery;
        battery.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
        battery.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
        battery.percentage = 0.9f;
        battery_pub_->publish(battery);

        SafetyStatusMsg safety;
        safety.hw_e_stop_user_button = e_stop_pressed_;
        safety.link_healthy = true;
        safety_pub_->publish(safety);
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

    /** How many times animation `id` was requested. */
    std::size_t requested(unsigned id)
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        return std::count_if(requests_.begin(), requests_.end(), [id](const auto & request) {
            return request.animation.id == id;
        });
    }

    uint8_t ledState() const { return led_node_->get_current_state().id(); }

    rclcpp::Node::SharedPtr helper_node_;
    rclcpp::Service<SetLedAnimationSrv>::SharedPtr led_service_;
    rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
    rclcpp::Publisher<SafetyStatusMsg>::SharedPtr safety_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::executors::SingleThreadedExecutor helper_executor_;
    std::thread helper_thread_;
    std::atomic<bool> publishing_{false};
    std::atomic<bool> e_stop_pressed_{false};

    std::mutex mutex_;
    std::vector<SetLedAnimationSrv::Request> requests_;

    std::shared_ptr<rover_safety::LedSafetyNode> led_node_;
    rclcpp::executors::SingleThreadedExecutor led_executor_;
    std::thread led_thread_;
};

}  // namespace

TEST_F(LedSafetyNodeAnimationsTest, RequestsAnimationsForPublishedInputs)
{
    ASSERT_TRUE(waitFor([this]() { return ledState() == State::PRIMARY_STATE_ACTIVE; }, 30s))
        << "state: " << static_cast<int>(ledState());

    publishing_ = true;

    // An idle, discharging rover: each channel shows its animation once.
    EXPECT_TRUE(waitFor([this]() {
        return requested(LedAnimationMsg::NO_ERROR) >= 1 && requested(LedAnimationMsg::BATTERY_NOMINAL) >= 1 &&
               requested(LedAnimationMsg::READY) >= 1;
    }, 10s));
    std::this_thread::sleep_for(1s);
    EXPECT_EQ(requested(LedAnimationMsg::NO_ERROR), 1u);
    EXPECT_EQ(requested(LedAnimationMsg::BATTERY_NOMINAL), 1u);
    EXPECT_EQ(requested(LedAnimationMsg::READY), 1u);
    EXPECT_EQ(requested(LedAnimationMsg::E_STOP), 0u);

    // The hardware E-Stop button is pressed.
    e_stop_pressed_ = true;
    EXPECT_TRUE(waitFor([this]() { return requested(LedAnimationMsg::E_STOP) >= 1; }, 5s));
    std::this_thread::sleep_for(1s);
    EXPECT_EQ(requested(LedAnimationMsg::E_STOP), 1u);
}
