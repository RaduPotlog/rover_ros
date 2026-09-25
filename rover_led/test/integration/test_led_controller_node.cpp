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

// LedControllerNode over real topics and services, on the small catalog of
// led_controller_test_animations.yaml: the latched catalog, set/stop replies,
// panel frames, led/state and diagnostics.

#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rover_msgs/msg/led_animation_catalog.hpp"
#include "rover_msgs/msg/led_layer_state.hpp"
#include "rover_msgs/msg/led_state.hpp"
#include "rover_msgs/srv/set_led_animation.hpp"
#include "rover_msgs/srv/stop_led_animation.hpp"

#include "rover_led/infrastructure/led_controller_node.hpp"

using namespace std::chrono_literals;
using Bytes = std::vector<std::uint8_t>;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using LedLayerState = rover_msgs::msg::LedLayerState;
using LedState = rover_msgs::msg::LedState;
using SetLedAnimation = rover_msgs::srv::SetLedAnimation;
using StopLedAnimation = rover_msgs::srv::StopLedAnimation;

namespace
{

// Both panels have 46 LEDs; channel 2 folds them into 2 rows.
constexpr std::size_t kNumLed = 46;

const Bytes kRed{255, 0, 0, 255};
const Bytes kWhite{255, 255, 255, 255};

// The controller's status carries one value per segment; the key tells it apart from the
// "starting up" message and from a real rover's controller.
const std::string kFrontSegmentKey = "Segment test_front (channel 1)";
const std::string kMissing = "<missing>";

// diagnostic_updater publishes once a second, unlike everything else here, which reacts to a
// service reply or a publish at controller_frequency: allow several update cycles.
constexpr auto kDiagnosticsTimeout = 5s;

// A frame of kNumLed LEDs of the same colour.
Bytes leds(const Bytes & rgba)
{
    Bytes frame;

    for (std::size_t i = 0; i < kNumLed; i++) {
        frame.insert(frame.end(), rgba.begin(), rgba.end());
    }

    return frame;
}

std::string valueOf(const DiagnosticStatus & status, const std::string & key)
{
    for (const auto & value : status.values) {
        if (value.key == key) {
            return value.value;
        }
    }

    return kMissing;
}

// True if the STATE layer of every segment is (in)active.
bool stateLayerActive(const LedState & state, const bool active)
{
    if (state.segments.empty()) {
        return false;
    }

    for (const auto & segment : state.segments) {
        if (segment.layers.size() <= LedLayerState::STATE || segment.layers[LedLayerState::STATE].active != active) {
            return false;
        }
    }

    return true;
}

class LedControllerNodeTest : public ::testing::Test
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
        controller_ = makeController();
        helper_ = std::make_shared<rclcpp::Node>("led_controller_test_helper", ns_);

        for (const std::size_t channel : {1, 2}) {
            frame_subs_.push_back(helper_->create_subscription<sensor_msgs::msg::Image>(
                "led/channel_" + std::to_string(channel) + "_frame", 10,
                [this, channel](const sensor_msgs::msg::Image & msg) { frames_[channel] = msg; }));
        }

        const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

        catalog_sub_ = helper_->create_subscription<rover_msgs::msg::LedAnimationCatalog>(
            "led/animations", latched_qos,
            [this](const rover_msgs::msg::LedAnimationCatalog & msg) { catalog_ = msg; });
        state_sub_ = helper_->create_subscription<LedState>(
            "led/state", latched_qos, [this](const LedState & msg) { state_ = msg; });
        diagnostics_sub_ = helper_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10, [this](const diagnostic_msgs::msg::DiagnosticArray & msg) {
                for (const auto & status : msg.status) {
                    if (status.hardware_id == "Bumper Led" && valueOf(status, kFrontSegmentKey) != kMissing) {
                        controller_status_ = status;
                    }
                }
            });

        set_client_ = helper_->create_client<SetLedAnimation>("led/set_animation");
        stop_client_ = helper_->create_client<StopLedAnimation>("led/stop_animation");

        executor_.add_node(controller_);
        executor_.add_node(helper_);
    }

    void TearDown() override
    {
        executor_.remove_node(helper_);
        executor_.remove_node(controller_);
    }

    // A unique namespace keeps this test off any topics of a running robot.
    static std::shared_ptr<rover_led::LedControllerNode> makeController()
    {
        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
        options.parameter_overrides({
            {"animations_config_path",
             std::string(ROVER_LED_SOURCE_DIR) + "/test/integration/led_controller_test_animations.yaml"},
            {"controller_frequency", 50.0},
            {"state_publish_rate", 20.0},
        });

        return std::make_shared<rover_led::LedControllerNode>(options);
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

    SetLedAnimation::Response callSet(const std::uint16_t id, const std::string & param, const bool repeating)
    {
        auto request = std::make_shared<SetLedAnimation::Request>();
        request->animation.id = id;
        request->animation.param = param;
        request->repeating = repeating;

        return call<SetLedAnimation>(set_client_, request);
    }

    StopLedAnimation::Response callStop(const std::uint16_t id)
    {
        auto request = std::make_shared<StopLedAnimation::Request>();
        request->id = id;

        return call<StopLedAnimation>(stop_client_, request);
    }

    // The last frame of the panel, empty until one arrived.
    Bytes frameData(const std::size_t channel) const
    {
        const auto frame = frames_.find(channel);

        return frame == frames_.end() ? Bytes{} : frame->second.data.to_vector();
    }

    // Waits until the panels show these frames; the checks report what they show otherwise.
    void expectFrames(const Bytes & channel_1, const Bytes & channel_2)
    {
        spinUntil([&] { return frameData(1) == channel_1 && frameData(2) == channel_2; });

        EXPECT_EQ(frameData(1), channel_1);
        EXPECT_EQ(frameData(2), channel_2);
    }

    bool waitForState(const std::function<bool(const LedState &)> & done)
    {
        return spinUntil([&] { return state_ && done(*state_); });
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rover_led::LedControllerNode> controller_;
    rclcpp::Node::SharedPtr helper_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> frame_subs_;
    rclcpp::Subscription<rover_msgs::msg::LedAnimationCatalog>::SharedPtr catalog_sub_;
    rclcpp::Subscription<LedState>::SharedPtr state_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Client<SetLedAnimation>::SharedPtr set_client_;
    rclcpp::Client<StopLedAnimation>::SharedPtr stop_client_;
    std::map<std::size_t, sensor_msgs::msg::Image> frames_;
    std::optional<rover_msgs::msg::LedAnimationCatalog> catalog_;
    std::optional<LedState> state_;
    std::optional<DiagnosticStatus> controller_status_;

    inline static const std::string ns_ = "/led_controller_test_" + std::to_string(getpid());

private:

    // A failed call reports a test failure and returns a default (unsuccessful) response.
    template<typename ServiceT>
    typename ServiceT::Response call(
        const typename rclcpp::Client<ServiceT>::SharedPtr & client,
        const typename ServiceT::Request::SharedPtr & request)
    {
        if (!spinUntil([&client] { return client->service_is_ready(); })) {
            ADD_FAILURE() << client->get_service_name() << " is not available.";
            return typename ServiceT::Response();
        }

        auto future = client->async_send_request(request);

        if (executor_.spin_until_future_complete(future, 3s) != rclcpp::FutureReturnCode::SUCCESS) {
            ADD_FAILURE() << client->get_service_name() << " did not reply.";
            client->remove_pending_request(future);
            return typename ServiceT::Response();
        }

        return *future.get();
    }
};

}  // namespace

TEST_F(LedControllerNodeTest, PublishesTheCatalogSortedById)
{
    ASSERT_TRUE(spinUntil([this] { return catalog_.has_value(); }));

    std::vector<std::uint16_t> ids;
    std::vector<std::string> names;
    std::vector<std::uint8_t> priorities;

    for (const auto & animation : catalog_->animations) {
        ids.push_back(animation.id);
        names.push_back(animation.name);
        priorities.push_back(animation.priority);
    }

    EXPECT_EQ(ids, (std::vector<std::uint16_t>{2, 7, 9}));
    EXPECT_EQ(names, (std::vector<std::string>{"WHITE_FRONT", "RED", "MISSING_TYPE"}));
    EXPECT_EQ(priorities, (std::vector<std::uint8_t>{0, 3, 2}));
}

TEST_F(LedControllerNodeTest, PublishesBlankPanelFramesWhileIdle)
{
    ASSERT_TRUE(spinUntil([this] { return frames_.size() == 2; }));

    // One image row per serpentine row, data in wire order.
    const auto & front = frames_.at(1);
    EXPECT_EQ(front.encoding, "rgba8");
    EXPECT_EQ(front.height, 1u);
    EXPECT_EQ(front.width, 46u);
    EXPECT_EQ(front.step, 184u);
    EXPECT_EQ(front.data.to_vector(), Bytes(184, 0));
    EXPECT_EQ(front.header.frame_id, ns_.substr(1) + "/led_channel_1_link");

    const auto & rear = frames_.at(2);
    EXPECT_EQ(rear.encoding, "rgba8");
    EXPECT_EQ(rear.height, 2u);
    EXPECT_EQ(rear.width, 23u);
    EXPECT_EQ(rear.step, 92u);
    EXPECT_EQ(rear.data.to_vector(), Bytes(184, 0));
    EXPECT_EQ(rear.header.frame_id, ns_.substr(1) + "/led_channel_2_link");
}

TEST_F(LedControllerNodeTest, SetAnimationLightsEverySegmentItCovers)
{
    const auto response = callSet(7, "", true);
    EXPECT_TRUE(response.success);
    EXPECT_EQ(response.message, "");

    expectFrames(leds(kRed), leds(kRed));
}

TEST_F(LedControllerNodeTest, HigherPriorityLayerIsDrawnOnTop)
{
    ASSERT_TRUE(callSet(7, "", true).success);
    expectFrames(leds(kRed), leds(kRed));

    // WHITE_FRONT is on the ERROR layer, and only on the front segment.
    const auto response = callSet(2, "", false);
    EXPECT_TRUE(response.success);

    expectFrames(leds(kWhite), leds(kRed));
}

TEST_F(LedControllerNodeTest, StateReportsWhatEveryLayerPlays)
{
    ASSERT_TRUE(waitForState([](const LedState &) { return true; }));

    // Sorted by name (GetLedStateUseCase), layers ordered ERROR to STATE.
    ASSERT_EQ(state_->segments.size(), 2u);
    EXPECT_EQ(state_->segments[0].name, "test_front");
    EXPECT_EQ(state_->segments[0].channel, 1u);
    EXPECT_EQ(state_->segments[1].name, "test_rear");
    EXPECT_EQ(state_->segments[1].channel, 2u);

    for (const auto & segment : state_->segments) {
        SCOPED_TRACE(segment.name);
        ASSERT_EQ(segment.layers.size(), 4u);

        for (std::size_t i = 0; i < segment.layers.size(); i++) {
            EXPECT_EQ(segment.layers[i].priority, i);
            EXPECT_FALSE(segment.layers[i].active);
        }
    }

    ASSERT_TRUE(callSet(7, "p1", true).success);
    ASSERT_TRUE(waitForState([](const LedState & state) { return stateLayerActive(state, true); }));

    for (const auto & segment : state_->segments) {
        SCOPED_TRACE(segment.name);

        for (std::size_t i = 0; i < LedLayerState::STATE; i++) {
            EXPECT_FALSE(segment.layers[i].active) << "layer " << i;
        }

        const auto & layer = segment.layers[LedLayerState::STATE];
        EXPECT_TRUE(layer.active);
        EXPECT_EQ(layer.id, 7u);
        EXPECT_EQ(layer.name, "RED");
        EXPECT_EQ(layer.param, "p1");
        EXPECT_TRUE(layer.repeating);
        EXPECT_EQ(layer.queued, 0u);
    }
}

// Frames are not checked after the stop: a segment left without animations keeps showing
// its last frame.
TEST_F(LedControllerNodeTest, StopAnimationStopsItWhereItPlays)
{
    ASSERT_TRUE(callSet(7, "", true).success);
    ASSERT_TRUE(waitForState([](const LedState & state) { return stateLayerActive(state, true); }));

    const auto stopped = callStop(7);
    EXPECT_TRUE(stopped.success);
    EXPECT_EQ(stopped.message, "Stopped 'RED' on test_front, test_rear.");
    EXPECT_TRUE(waitForState([](const LedState & state) { return stateLayerActive(state, false); }));

    const auto again = callStop(7);
    EXPECT_FALSE(again.success);
    EXPECT_EQ(again.message, "'RED' is not playing.");
}

TEST_F(LedControllerNodeTest, UnknownIdsAreRejected)
{
    const auto set = callSet(42, "", false);
    EXPECT_FALSE(set.success);
    EXPECT_EQ(set.message, "No animation with ID: 42");

    const auto stop = callStop(42);
    EXPECT_FALSE(stop.success);
    EXPECT_EQ(stop.message, "No animation with ID: 42");
}

TEST_F(LedControllerNodeTest, AnimationOfAnUnavailableTypeIsRejected)
{
    const auto response = callSet(9, "", false);
    EXPECT_FALSE(response.success);
    EXPECT_THAT(
        response.message,
        ::testing::HasSubstr("Failed to set 'MISSING_TYPE' animation: The plugin failed to load."));
}

TEST_F(LedControllerNodeTest, DiagnosticsCountAnimationsOfAnUnavailableType)
{
    ASSERT_TRUE(spinUntil([this] { return controller_status_.has_value(); }, kDiagnosticsTimeout));

    EXPECT_EQ(controller_status_->level, DiagnosticStatus::WARN);
    EXPECT_EQ(controller_status_->message, "Some animations use an unavailable type.");
    EXPECT_EQ(valueOf(*controller_status_, "Animations loaded"), "3");
    EXPECT_EQ(valueOf(*controller_status_, "Animations with unavailable type"), "1");
    EXPECT_EQ(valueOf(*controller_status_, "Catalog warnings"), "0");
    EXPECT_EQ(valueOf(*controller_status_, kFrontSegmentKey), "idle");
}
