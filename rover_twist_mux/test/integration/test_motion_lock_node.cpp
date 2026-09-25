// Copyright 2026 Mechatronics Academy
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
// ROS-level tests for MotionLockNode: that the fail-safe behaviour the policy unit tests describe
// actually reaches the wire.
//
// The policy and the health evaluator were already covered, but nothing checked the node itself -
// so nothing verified that a silent hardware interface really does produce motion_lock=true on
// the topic twist_mux consumes, only that a function would have returned "locked" if it had been
// called with the right arguments. The QoS, the topic names and the both-topics-required rule all
// live in the node, and all three are places where a mistake produces a lock that never asserts.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rover_msgs/msg/safety_command_echo.hpp"
#include "rover_msgs/msg/safety_status.hpp"
#include "rover_twist_mux/infrastructure/motion_lock_node.hpp"

namespace rover_twist_mux
{
namespace
{

using namespace std::chrono_literals;

// Matches SystemROSInterface's publisher exactly. A mismatch on any of the three settings and the
// node's subscription silently receives nothing, which would look identical to "locked because
// nothing arrived" - the failure this file exists to distinguish.
rclcpp::QoS safetyQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

class MotionLockNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        harness_ = std::make_shared<rclcpp::Node>("motion_lock_test_harness");

        status_pub_ = harness_->create_publisher<rover_msgs::msg::SafetyStatus>(
            "hardware_interface/safety_status", safetyQos());
        echo_pub_ = harness_->create_publisher<rover_msgs::msg::SafetyCommandEcho>(
            "hardware_interface/safety_command_echo", safetyQos());

        lock_sub_ = harness_->create_subscription<std_msgs::msg::Bool>(
            "motion_lock", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const std_msgs::msg::Bool & msg) { last_lock_ = msg.data; });

        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("publish_frequency", 50.0),
            rclcpp::Parameter("gpio_timeout", 0.3),
        });

        node_ = std::make_shared<MotionLockNode>("rover_motion_lock_node", options);

        executor_.add_node(harness_);
        executor_.add_node(node_);
    }

    void TearDown() override
    {
        executor_.remove_node(node_);
        executor_.remove_node(harness_);
        node_.reset();
        harness_.reset();
    }

    // Spins until `predicate` holds or the deadline passes. Everything here is driven by the
    // node's own publish timer, so the tests wait on observed output rather than on a fixed sleep.
    template <typename PredicateT>
    bool spinUntil(PredicateT predicate, const std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();

            if (predicate()) {
                return true;
            }

            std::this_thread::sleep_for(2ms);
        }

        executor_.spin_some();
        return predicate();
    }

    bool spinUntilLockIs(const bool locked, const std::chrono::milliseconds timeout = 2000ms)
    {
        return spinUntil(
            [this, locked] { return last_lock_.has_value() && *last_lock_ == locked; }, timeout);
    }

    // An all-clear safety state: nothing tripped, link healthy, contactor closed.
    void publishAllClear(const bool link_healthy = true)
    {
        rover_msgs::msg::SafetyStatus status;
        status.hw_e_stop_user_button = false;
        status.latch_active = false;
        status.motor_contactor_engaged = true;
        status.link_healthy = link_healthy;
        status_pub_->publish(status);

        rover_msgs::msg::SafetyCommandEcho echo;
        echo.sw_e_stop_user_button = false;
        echo.sw_e_stop_motor_driver_fault = false;
        echo_pub_->publish(echo);
    }

    std::shared_ptr<rclcpp::Node> harness_;
    std::shared_ptr<MotionLockNode> node_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Publisher<rover_msgs::msg::SafetyStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<rover_msgs::msg::SafetyCommandEcho>::SharedPtr echo_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_sub_;

    std::optional<bool> last_lock_;
};

// The startup case. With no hardware interface running there is no safety state, and the rover
// must refuse to drive rather than assume it is safe to.
TEST_F(MotionLockNodeTest, LocksBeforeAnySafetyStateArrives)
{
    EXPECT_TRUE(spinUntilLockIs(true));
}

TEST_F(MotionLockNodeTest, UnlocksOnceBothTopicsReportAllClear)
{
    ASSERT_TRUE(spinUntilLockIs(true));

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
        publishAllClear();

        if (spinUntil([this] { return last_lock_.has_value() && !*last_lock_; }, 60ms)) {
            break;
        }
    }

    EXPECT_TRUE(last_lock_.has_value());
    EXPECT_FALSE(*last_lock_) << "the lock never released with both topics reporting all clear";
}

// Only one of the two topics arriving must not be enough: acting on half the picture would read
// the missing half's stop conditions as "not active".
TEST_F(MotionLockNodeTest, StaysLockedWhenOnlySafetyStatusArrives)
{
    rover_msgs::msg::SafetyStatus status;
    status.hw_e_stop_user_button = false;
    status.latch_active = false;
    status.motor_contactor_engaged = true;
    status.link_healthy = true;

    const auto deadline = std::chrono::steady_clock::now() + 700ms;
    while (std::chrono::steady_clock::now() < deadline) {
        status_pub_->publish(status);
        spinUntil([] { return false; }, 40ms);
    }

    ASSERT_TRUE(last_lock_.has_value());
    EXPECT_TRUE(*last_lock_) << "the lock released on safety_status alone";
}

// The hardware interface dying is the case the whole node exists for. Publishing stops; the lock
// must re-assert once the messages age past gpio_timeout.
TEST_F(MotionLockNodeTest, ReLocksWhenTheSafetyStateGoesStale)
{
    const auto unlock_deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < unlock_deadline) {
        publishAllClear();

        if (spinUntil([this] { return last_lock_.has_value() && !*last_lock_; }, 60ms)) {
            break;
        }
    }

    ASSERT_TRUE(last_lock_.has_value());
    ASSERT_FALSE(*last_lock_) << "precondition: the lock should have released first";

    // Stop publishing. gpio_timeout is 0.3 s.
    EXPECT_TRUE(spinUntilLockIs(true, 3000ms))
        << "the lock did not re-assert after the safety state went stale";
}

// One topic going quiet while the other keeps arriving: the fresh one must not hide the stale
// one. The unit tests pin how the two ages combine; this pins that the node really hands over
// both, since passing the status age twice would pass every one of them.
TEST_F(MotionLockNodeTest, ReLocksWhenOnlyTheCommandEchoGoesStale)
{
    const auto unlock_deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < unlock_deadline) {
        publishAllClear();

        if (spinUntil([this] { return last_lock_.has_value() && !*last_lock_; }, 60ms)) {
            break;
        }
    }

    ASSERT_TRUE(last_lock_.has_value());
    ASSERT_FALSE(*last_lock_) << "precondition: the lock should have released first";

    rover_msgs::msg::SafetyStatus status;
    status.hw_e_stop_user_button = false;
    status.latch_active = false;
    status.motor_contactor_engaged = true;
    status.link_healthy = true;

    // Keep safety_status fresh and stop safety_command_echo. gpio_timeout is 0.3 s.
    bool relocked = false;
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!relocked && std::chrono::steady_clock::now() < deadline) {
        status_pub_->publish(status);
        relocked = spinUntil([this] { return last_lock_.has_value() && *last_lock_; }, 40ms);
    }

    EXPECT_TRUE(relocked)
        << "the lock stayed released on a fresh safety_status and a stale safety_command_echo";
}

// A link the hardware interface reports as down keeps the messages flowing, so staleness alone
// never catches it - the values in them are simply last-known-good.
TEST_F(MotionLockNodeTest, LocksWhenTheSafetyLinkIsReportedUnhealthy)
{
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
        publishAllClear(/*link_healthy=*/false);
        spinUntil([] { return false; }, 40ms);
    }

    ASSERT_TRUE(last_lock_.has_value());
    EXPECT_TRUE(*last_lock_) << "the lock released on an unhealthy safety link";
}

}  // namespace
}  // namespace rover_twist_mux
