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
//
// Integration test for SystemROSInterface: unlike the domain/ unit tests, this exercises the
// real rclcpp graph (services, publishers) that RoverSystem/RoverA1System drive in
// on_configure()/read(), without requiring any Phidget/Modbus hardware - SystemROSInterface has
// no dependency on either.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rover_hardware_interface/system_ros_interface/system_ros_interface.hpp"

namespace rover_hardware_interface
{
namespace
{

class SystemROSInterfaceTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// Bounded, event-driven wait: spins `node` until `predicate()` is true or `timeout` elapses.
// Used instead of a plain sleep() so the test resolves as soon as the condition is met (typically
// well under `timeout`) while still tolerating discovery/scheduling latency in CI.
template <typename PredicateT>
bool spinUntil(
    const rclcpp::Node::SharedPtr & node, PredicateT predicate,
    const std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (!predicate()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        rclcpp::spin_some(node);
    }

    return true;
}

}  // namespace

TEST_F(SystemROSInterfaceTest, RegisteredTriggerServiceInvokesCallbackAndReportsSuccess)
{
    SystemROSInterface ros_interface("test_system_ros_interface_srv");

    std::atomic_bool called{false};
    ros_interface.addService<TriggerSrv, std::function<void()>>(
        "test_trigger_service", std::function<void()>([&called]() { called = true; }));

    auto client_node = std::make_shared<rclcpp::Node>("test_system_ros_interface_srv_client");
    auto client = client_node->create_client<std_srvs::srv::Trigger>("test_trigger_service");

    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    ASSERT_EQ(
        rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(5)),
        rclcpp::FutureReturnCode::SUCCESS);

    const auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(called);
}

TEST_F(SystemROSInterfaceTest, ThrowingTriggerCallbackReportsFailureWithTheExceptionMessage)
{
    // The failure branch of ROSServiceWrapper::callbackWrapper(). This is the path every E-Stop
    // service rejection takes (e.g. "Can't reset User E-Stop: velocity commands are not zero."):
    // the exception must be turned into success=false plus the what() string, not escape into
    // the executor.
    SystemROSInterface ros_interface("test_system_ros_interface_srv_throw");

    const std::string error_message = "deliberate failure from the service callback";

    ros_interface.addService<TriggerSrv, std::function<void()>>(
        "test_throwing_trigger_service",
        std::function<void()>([&error_message]() { throw std::runtime_error(error_message); }));

    auto client_node =
        std::make_shared<rclcpp::Node>("test_system_ros_interface_srv_throw_client");
    auto client =
        client_node->create_client<std_srvs::srv::Trigger>("test_throwing_trigger_service");

    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    ASSERT_EQ(
        rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(5)),
        rclcpp::FutureReturnCode::SUCCESS);

    const auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->message, error_message);
}

TEST_F(SystemROSInterfaceTest, PublishesDriverStateAfterUpdate)
{
    SystemROSInterface ros_interface("test_system_ros_interface_pub");

    auto client_node = std::make_shared<rclcpp::Node>("test_system_ros_interface_pub_client");

    RoverDriverStateMsg received;
    std::atomic_bool got_msg{false};
    auto subscription = client_node->create_subscription<RoverDriverStateMsg>(
        "hardware_interface/rover_driver_state", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [&](const RoverDriverStateMsg::SharedPtr msg) {
            received = *msg;
            got_msg = true;
        });

    // Wait for pub/sub discovery to complete before publishing - otherwise a single publish
    // (below) could be sent before the subscription is matched and would never be received.
    ASSERT_TRUE(spinUntil(
        client_node, [&]() { return subscription->get_publisher_count() > 0; },
        std::chrono::seconds(5)));

    ros_interface.updateMsgError(true);
    ros_interface.publishRobotDriverState();

    ASSERT_TRUE(spinUntil(client_node, [&]() { return got_msg.load(); }, std::chrono::seconds(5)));
    EXPECT_TRUE(received.error);
    EXPECT_EQ(received.driver_states.size(), 4u);
}

TEST_F(SystemROSInterfaceTest, RoutesPlantReadingsToSafetyStatus)
{
    SystemROSInterface ros_interface("test_system_ros_interface_safety_status");
    auto client_node = std::make_shared<rclcpp::Node>("test_system_ros_interface_safety_status_client");

    SafetyStatusMsg received;
    std::atomic_bool got_msg{false};
    auto subscription = client_node->create_subscription<SafetyStatusMsg>(
        "hardware_interface/safety_status",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        [&](const SafetyStatusMsg::SharedPtr msg) {
            received = *msg;
            got_msg = true;
        });

    ASSERT_TRUE(spinUntil(
        client_node, [&]() { return subscription->get_publisher_count() > 0; },
        std::chrono::seconds(5)));

    ros_interface.updateMsgGpioStates({
        {RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN, true},
        {RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED, false},
        {RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS, true},
    });

    SafetyLinkHealth health;
    health.watchdog_running = true;
    health.poll_running = true;
    health.last_poll_age_ms = 20;
    ros_interface.updateSafetyLinkState(health);

    ros_interface.publishSafetyMsgs();

    ASSERT_TRUE(spinUntil(client_node, [&]() { return got_msg.load(); }, std::chrono::seconds(5)));
    EXPECT_TRUE(received.hw_e_stop_user_button);
    EXPECT_FALSE(received.motor_contactor_engaged);
    EXPECT_TRUE(received.latch_active);
    EXPECT_TRUE(received.link_healthy);
    EXPECT_EQ(received.latch_cause, SafetyStatusMsg::LATCH_CAUSE_UNKNOWN);
}

// The coils software drives must land in the echo message, never in the one consumers gate on.
TEST_F(SystemROSInterfaceTest, RoutesCommandEchoesToSafetyCommandEcho)
{
    SystemROSInterface ros_interface("test_system_ros_interface_safety_echo");
    auto client_node = std::make_shared<rclcpp::Node>("test_system_ros_interface_safety_echo_client");

    SafetyCommandEchoMsg received;
    std::atomic_bool got_msg{false};
    auto subscription = client_node->create_subscription<SafetyCommandEchoMsg>(
        "hardware_interface/safety_command_echo",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        [&](const SafetyCommandEchoMsg::SharedPtr msg) {
            received = *msg;
            got_msg = true;
        });

    ASSERT_TRUE(spinUntil(
        client_node, [&]() { return subscription->get_publisher_count() > 0; },
        std::chrono::seconds(5)));

    ros_interface.updateMsgGpioStates({
        {RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON, true},
        {RoverControllerGpio::GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT, false},
        {RoverControllerGpio::GPIO_CPU_WDG_HEARTBEAT, true},
        {RoverControllerGpio::GPIO_SW_E_STOP_LATCH_RESET, false},
    });
    ros_interface.updateSafetyLinkState(SafetyLinkHealth{});
    ros_interface.publishSafetyMsgs();

    ASSERT_TRUE(spinUntil(client_node, [&]() { return got_msg.load(); }, std::chrono::seconds(5)));
    EXPECT_TRUE(received.sw_e_stop_user_button);
    EXPECT_FALSE(received.sw_e_stop_motor_driver_fault);
    EXPECT_TRUE(received.cpu_wdg_heartbeat);
    EXPECT_FALSE(received.sw_e_stop_latch_reset);
}

// A link that has never polled must not claim to be healthy - the pin values in that case are
// default-constructed, not observed.
TEST_F(SystemROSInterfaceTest, LinkIsNotHealthyBeforeTheFirstSuccessfulPoll)
{
    SystemROSInterface ros_interface("test_system_ros_interface_safety_link");
    auto client_node = std::make_shared<rclcpp::Node>("test_system_ros_interface_safety_link_client");

    SafetyStatusMsg received;
    std::atomic_bool got_msg{false};
    auto subscription = client_node->create_subscription<SafetyStatusMsg>(
        "hardware_interface/safety_status",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        [&](const SafetyStatusMsg::SharedPtr msg) {
            received = *msg;
            got_msg = true;
        });

    ASSERT_TRUE(spinUntil(
        client_node, [&]() { return subscription->get_publisher_count() > 0; },
        std::chrono::seconds(5)));

    SafetyLinkHealth health;
    health.watchdog_running = true;
    health.poll_running = true;
    // last_poll_age_ms stays kUnknownAgeMs.
    ros_interface.updateSafetyLinkState(health);
    ros_interface.publishSafetyMsgs();

    ASSERT_TRUE(spinUntil(client_node, [&]() { return got_msg.load(); }, std::chrono::seconds(5)));
    EXPECT_FALSE(received.link_healthy);
}

}  // namespace rover_hardware_interface
