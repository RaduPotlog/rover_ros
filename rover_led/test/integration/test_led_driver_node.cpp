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

// LedDriverNode lifecycle behaviour over real topics: frames are only
// forwarded while active, and the LEDs are cleared on activate/deactivate.

#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

#include "rover_msgs/srv/set_led_brightness.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rover_led/domain/sk9822_frame_encoder.hpp"
#include "rover_led/infrastructure/led_driver_node.hpp"

using namespace std::chrono_literals;
using Bytes = std::vector<std::uint8_t>;

namespace
{

constexpr int kNumLed = 2;

class LedDriverNodeTest : public ::testing::Test
{

protected:

    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }

    void SetUp() override
    {
        driver_ = makeDriver(false);
        helper_ = std::make_shared<rclcpp::Node>("led_driver_test_helper", ns_);

        frame_pub_ = helper_->create_publisher<sensor_msgs::msg::Image>("led/channel_1_frame", 5);
        udp_sub_ = helper_->create_subscription<udp_msgs::msg::UdpPacket>(
            "udp_write/led_channel_1", 10, [this](const udp_msgs::msg::UdpPacket & msg) {
                packets_.push_back(msg.data);
            });
        brightness_client_ = helper_->create_client<rover_msgs::srv::SetLedBrightness>("led/set_brightness");

        executor_.add_node(driver_->get_node_base_interface());
        executor_.add_node(helper_);
    }

    void TearDown() override
    {
        executor_.remove_node(helper_);
        executor_.remove_node(driver_->get_node_base_interface());
    }

    // A unique namespace keeps this test off any topics of a running robot.
    static std::shared_ptr<rover_led::LedDriverNode> makeDriver(
        const bool autostart, const bool handshake = false)
    {
        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
        options.parameter_overrides({
            {"channel_1_num_led", kNumLed},
            {"channel_2_num_led", kNumLed},
            {"frame_timeout", 5.0},
            {"autostart", autostart},
            {"led_control_handshake", handshake},
        });

        return std::make_shared<rover_led::LedDriverNode>(options);
    }

    // Fake hardware/led_control_enable server. Replies are held until
    // replyToHardwareRequests(), so tests control when control is granted.
    void startFakeHardware()
    {
        hardware_service_ = helper_->create_service<std_srvs::srv::SetBool>(
            "hardware/led_control_enable",
            [this](const std::shared_ptr<rmw_request_id_t> header,
                   const std::shared_ptr<std_srvs::srv::SetBool::Request> request) {
                hardware_requests_.push_back(request->data);
                pending_hardware_replies_.push_back(header);
            });
    }

    void replyToHardwareRequests()
    {
        for (const auto & header : pending_hardware_replies_) {
            std_srvs::srv::SetBool::Response response;
            response.success = true;
            hardware_service_->send_response(*header, response);
        }

        pending_hardware_replies_.clear();
    }

    // Spins until `done` holds or the timeout expires.
    bool spinUntil(const std::function<bool()> & done, const std::chrono::milliseconds timeout = 3s)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (!done()) {
            if (std::chrono::steady_clock::now() > deadline) {
                return false;
            }

            executor_.spin_some(10ms);
        }

        return true;
    }

    bool waitForConnections()
    {
        return spinUntil([this] {
            return frame_pub_->get_subscription_count() > 0 && udp_sub_->get_publisher_count() > 0;
        });
    }

    sensor_msgs::msg::Image frame(const Bytes & data)
    {
        sensor_msgs::msg::Image image;
        image.header.stamp = helper_->now();
        image.encoding = "rgba8";
        image.height = 1;
        image.width = kNumLed;
        image.step = kNumLed * 4;
        image.data = data;

        return image;
    }

    static Bytes blankPayload()
    {
        rover_led::SK9822FrameEncoder encoder;

        return encoder.encodeForUdpBridge(Bytes(kNumLed * 4, 0));
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rover_led::LedDriverNode> driver_;
    rclcpp::Node::SharedPtr helper_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub_;
    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr udp_sub_;
    rclcpp::Client<rover_msgs::srv::SetLedBrightness>::SharedPtr brightness_client_;
    std::deque<Bytes> packets_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr hardware_service_;
    std::vector<bool> hardware_requests_;
    std::vector<std::shared_ptr<rmw_request_id_t>> pending_hardware_replies_;

    inline static const std::string ns_ = "/led_driver_test_" + std::to_string(getpid());
};

}  // namespace

TEST_F(LedDriverNodeTest, StartsUnconfigured)
{
    EXPECT_EQ(driver_->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LedDriverNodeTest, AutostartConfiguresAndActivates)
{
    // Replace the fixture's manually driven node with a self-starting one.
    executor_.remove_node(driver_->get_node_base_interface());
    driver_ = makeDriver(true);
    executor_.add_node(driver_->get_node_base_interface());

    EXPECT_TRUE(spinUntil([this] {
        return driver_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    }));
}

TEST_F(LedDriverNodeTest, ForwardsFramesOnlyWhileActive)
{
    ASSERT_EQ(driver_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(waitForConnections());

    // Inactive: frames are dropped.
    frame_pub_->publish(frame(Bytes(kNumLed * 4, 255)));
    EXPECT_FALSE(spinUntil([this] { return !packets_.empty(); }, 500ms));

    // Activating clears the LEDs.
    ASSERT_EQ(driver_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(spinUntil([this] { return !packets_.empty(); }));
    EXPECT_EQ(packets_.front(), blankPayload());
    packets_.clear();

    // Active: frames are encoded for the UDP bridge.
    const Bytes data{255, 0, 0, 255, 0, 0, 255, 255};
    frame_pub_->publish(frame(data));
    ASSERT_TRUE(spinUntil([this] { return !packets_.empty(); }));
    EXPECT_EQ(packets_.front(), rover_led::SK9822FrameEncoder().encodeForUdpBridge(data));
    packets_.clear();

    // Deactivating clears the LEDs, then frames are dropped again.
    ASSERT_EQ(driver_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(spinUntil([this] { return !packets_.empty(); }));
    EXPECT_EQ(packets_.front(), blankPayload());
    packets_.clear();

    frame_pub_->publish(frame(data));
    EXPECT_FALSE(spinUntil([this] { return !packets_.empty(); }, 500ms));
}

TEST_F(LedDriverNodeTest, DropsMalformedFrames)
{
    driver_->configure();
    ASSERT_TRUE(waitForConnections());
    driver_->activate();
    ASSERT_TRUE(spinUntil([this] { return !packets_.empty(); }));
    packets_.clear();

    auto wrong_width = frame(Bytes((kNumLed + 1) * 4, 255));
    wrong_width.width = kNumLed + 1;
    frame_pub_->publish(wrong_width);

    EXPECT_FALSE(spinUntil([this] { return !packets_.empty(); }, 500ms));
}

TEST_F(LedDriverNodeTest, SetBrightnessValidatesTheRange)
{
    driver_->configure();
    ASSERT_TRUE(spinUntil([this] { return brightness_client_->service_is_ready(); }));

    auto call = [this](const float value) {
        auto request = std::make_shared<rover_msgs::srv::SetLedBrightness::Request>();
        request->data = value;
        auto future = brightness_client_->async_send_request(request);
        EXPECT_EQ(executor_.spin_until_future_complete(future, 3s), rclcpp::FutureReturnCode::SUCCESS);

        return future.get();
    };

    const auto ok = call(0.5f);
    EXPECT_TRUE(ok->success);
    EXPECT_EQ(ok->message, "Changed brightness to 0.50");

    EXPECT_FALSE(call(1.5f)->success);
}

TEST_F(LedDriverNodeTest, BrightnessIsLatchedAppliedAndKeptAcrossReconfigure)
{
    std::vector<float> reported;
    auto brightness_sub = helper_->create_subscription<std_msgs::msg::Float32>(
        "led/brightness", rclcpp::QoS(1).reliable().transient_local(),
        [&reported](const std_msgs::msg::Float32 & msg) { reported.push_back(msg.data); });

    driver_->configure();
    ASSERT_TRUE(waitForConnections());
    driver_->activate();
    ASSERT_TRUE(spinUntil([&reported] { return !reported.empty(); }));
    EXPECT_FLOAT_EQ(reported.back(), 1.0f);

    auto request = std::make_shared<rover_msgs::srv::SetLedBrightness::Request>();
    request->data = 0.25f;
    auto future = brightness_client_->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future, 3s), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);

    ASSERT_TRUE(spinUntil([&reported] { return reported.back() == 0.25f; }));
    EXPECT_DOUBLE_EQ(driver_->get_parameter("global_brightness").as_double(), 0.25);

    // The SK9822 global current field carries it: ceil(0.25 * 31) = 8.
    packets_.clear();
    frame_pub_->publish(frame(Bytes(kNumLed * 4, 255)));
    ASSERT_TRUE(spinUntil([this] { return !packets_.empty(); }));
    EXPECT_EQ(packets_.front().at(7), 0xE0 | 8);

    // A reconfigure keeps the brightness set through the service.
    reported.clear();
    driver_->deactivate();
    driver_->cleanup();
    driver_->configure();
    driver_->activate();
    ASSERT_TRUE(spinUntil([&reported] { return !reported.empty(); }));
    EXPECT_FLOAT_EQ(reported.back(), 0.25f);
}

class LedDriverNodeHandshakeTest : public LedDriverNodeTest
{

protected:

    void SetUp() override
    {
        LedDriverNodeTest::SetUp();

        executor_.remove_node(driver_->get_node_base_interface());
        driver_ = makeDriver(false, true);
        executor_.add_node(driver_->get_node_base_interface());

        startFakeHardware();
    }
};

TEST_F(LedDriverNodeHandshakeTest, ForwardsFramesOnlyOnceControlIsGranted)
{
    driver_->configure();
    ASSERT_TRUE(waitForConnections());
    ASSERT_EQ(driver_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ASSERT_TRUE(spinUntil([this] { return !hardware_requests_.empty(); }));
    EXPECT_EQ(hardware_requests_, (std::vector<bool>{true}));

    // Not granted yet: frames are dropped.
    const Bytes data{255, 0, 0, 255, 0, 0, 255, 255};
    frame_pub_->publish(frame(data));
    EXPECT_FALSE(spinUntil([this] { return !packets_.empty(); }, 500ms));

    // Grant: the LEDs are cleared, then frames flow.
    replyToHardwareRequests();
    ASSERT_TRUE(spinUntil([this] { return !packets_.empty(); }));
    EXPECT_EQ(packets_.front(), blankPayload());
    packets_.clear();

    frame_pub_->publish(frame(data));
    ASSERT_TRUE(spinUntil([this] { return !packets_.empty(); }));
    EXPECT_EQ(packets_.front(), rover_led::SK9822FrameEncoder().encodeForUdpBridge(data));

    // Deactivating releases control.
    driver_->deactivate();
    ASSERT_TRUE(spinUntil([this] { return hardware_requests_.size() == 2; }));
    EXPECT_FALSE(hardware_requests_.back());
}

TEST_F(LedDriverNodeHandshakeTest, GrantArrivingAfterDeactivationIsHandedBack)
{
    driver_->configure();
    ASSERT_TRUE(waitForConnections());
    driver_->activate();
    ASSERT_TRUE(spinUntil([this] { return !hardware_requests_.empty(); }));

    // The grant for this activation only arrives once the node is inactive.
    driver_->deactivate();
    replyToHardwareRequests();

    ASSERT_TRUE(spinUntil([this] { return hardware_requests_.size() == 2; }));
    EXPECT_EQ(hardware_requests_, (std::vector<bool>{true, false}));

    // Re-activating starts a fresh request; the stale grant was not adopted.
    packets_.clear();
    driver_->activate();
    ASSERT_TRUE(spinUntil([this] { return hardware_requests_.size() == 3; }));
    EXPECT_TRUE(hardware_requests_.back());
    EXPECT_FALSE(spinUntil([this] { return !packets_.empty(); }, 500ms));
}

// Ctrl-C path: shutting the context down finalizes the node on the executor, before the
// middleware goes away (otherwise it is destroyed while still Active).
TEST(LedDriverNodeShutdownTest, ContextShutdownFinalizesTheNode)
{
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);

    rclcpp::NodeOptions options;
    options.context(context);
    options.arguments({"--ros-args", "-r", "__ns:=/led_driver_shutdown_test_" + std::to_string(getpid())});
    options.parameter_overrides({{"autostart", true}, {"led_control_handshake", false}});

    auto driver = std::make_shared<rover_led::LedDriverNode>(options);

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(executor_options);
    executor.add_node(driver->get_node_base_interface());

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (driver->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
           std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(10ms);
    }
    ASSERT_EQ(driver->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    std::thread spinner([&executor]() { executor.spin(); });

    context->shutdown("test");
    spinner.join();

    EXPECT_EQ(driver->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

    executor.remove_node(driver->get_node_base_interface());
}
