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

// Integration test: RoverCrsfTeleopNode on real ROS topics and services. A helper node plays
// rover_serial_driver's rover_serial_bridge_node (raw CRSF bytes on rc/raw at 50 Hz), the hardware interface (the
// three E-Stop Trigger services) and twist_mux (subscribes to the cmd_vel output).
//
// Driving the node with encoded bytes rather than decoded messages means this test now covers
// the wire format and the parser as well as the teleop rules.
//
// Synchronisation is always "spin the executor until a predicate holds, or time out" - never a
// sleep.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <rover_msgs/msg/safety_status.hpp>
#include <rover_msgs/msg/rc_calibration_state.hpp>
#include <rover_msgs/srv/set_rc_calibration.hpp>
#include <rover_msgs/srv/start_rc_calibration.hpp>

#include "rover_crsf_teleop/domain/crsf/crc8.hpp"
#include "rover_crsf_teleop/domain/crsf/crsf_protocol.hpp"
#include "rover_crsf_teleop/domain/rc_frame.hpp"
#include "rover_crsf_teleop/domain/stick_mapping.hpp"
#include "rover_crsf_teleop/infrastructure/rover_crsf_teleop_node.hpp"

namespace rover_crsf_teleop
{
namespace
{

using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::TwistStamped;
using Trigger = std_srvs::srv::Trigger;
using SafetyStatus = rover_msgs::msg::SafetyStatus;
using RcCalibrationState = rover_msgs::msg::RcCalibrationState;
using SetRcCalibration = rover_msgs::srv::SetRcCalibration;
using StartRcCalibration = rover_msgs::srv::StartRcCalibration;

constexpr int kSwitchLow = kDefaultCrsfChannelMin;
constexpr int kSwitchHigh = kDefaultCrsfChannelMax;

using Bytes = std::vector<std::uint8_t>;

// Wraps a payload in a CRSF frame: address, length, type, payload, CRC8.
Bytes buildFrame(const std::uint8_t type, const Bytes & payload)
{
    Bytes frame{crsf::kAddressFlightController, static_cast<std::uint8_t>(payload.size() + 2U), type};
    frame.insert(frame.end(), payload.begin(), payload.end());

    const crsf::Crc8 crc(crsf::kCrcPolynomial);
    frame.push_back(crc.calc(frame.data() + 2, payload.size() + 1U));

    return frame;
}

// Packs 16 channels as 11-bit little-endian fields - the encoder counterpart of the decoder
// under test, written independently here so the two have to agree.
Bytes encodeRcChannelsFrame(const RcFrame & rc_frame)
{
    Bytes payload(crsf::kRcChannelsPayloadSize, 0U);
    std::size_t bit = 0;

    for (std::size_t channel = 0; channel < RcFrame::kChannelCount; ++channel) {
        const auto value = static_cast<std::uint32_t>(rc_frame.channels[channel] & crsf::kRcChannelMask);

        for (std::size_t i = 0; i < crsf::kRcChannelBits; ++i) {
            if ((value >> i) & 1U) {
                const std::size_t index = (bit + i) / 8U;
                payload[index] = static_cast<std::uint8_t>(payload[index] | (1U << ((bit + i) % 8U)));
            }
        }

        bit += crsf::kRcChannelBits;
    }

    return buildFrame(crsf::kFrameTypeRcChannelsPacked, payload);
}

Bytes encodeLinkStatisticsFrame(const std::uint8_t uplink_link_quality)
{
    Bytes payload(crsf::kLinkStatisticsPayloadSize, 0U);
    payload[2] = uplink_link_quality;

    return buildFrame(crsf::kFrameTypeLinkStatistics, payload);
}

// Plays everything around the teleop node.
class RoverHarness
{

public:

    explicit RoverHarness(const rclcpp::Node::SharedPtr & node)
    : node_(node)
    {
        // Matches rover_serial_bridge_node's publisher QoS, which is what the node subscribes with.
        serial_pub_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("rc/raw", rclcpp::QoS(100));

        cmd_vel_sub_ = node_->create_subscription<Twist>(
            "teleop_elrs_cmd_vel_stamped", 10,
            [this](const Twist & msg) { received.push_back(msg); });

        e_stop_set_srv_ = makeService("hardware_interface/sw_user_e_stop_set", e_stop_set_calls);
        e_stop_reset_srv_ = makeService("hardware_interface/sw_user_e_stop_reset", e_stop_reset_calls);
        latch_reset_srv_ = makeService("hardware_interface/sw_e_stop_latch_reset", latch_reset_calls);

        channels_.channels.fill(kDefaultCrsfChannelMid);
        channels_.channels[3] = kSwitchHigh;   // channel 4: E-Stop latch reset
        channels_.channels[4] = kSwitchHigh;   // channel 5: E-Stop

        // The hardware interface's safety status, with its exact QoS - reliable, volatile,
        // depth 1. Miss any of the three and the node's subscription gets nothing at all.
        gpio_state_pub_ = node_->create_publisher<SafetyStatus>(
            "hardware_interface/safety_status", rclcpp::QoS(1).reliable().durability_volatile());

        // The hardware interface republishes the safety IO every cycle at 20 Hz rather than on
        // change, and the node ages a silent publisher into "unverified" after a second. A
        // harness that published once would therefore time out mid-test and prove the wrong
        // thing.
        gpio_timer_ = node_->create_wall_timer(50ms, [this]() {
            if (gpio_state_.has_value()) {
                gpio_state_pub_->publish(*gpio_state_);
            }
        });

        // The receiver's 50 Hz frame stream, while `feeding` is set.
        feed_timer_ = node_->create_wall_timer(20ms, [this]() {
            if (!feeding) {
                return;
            }

            publishBytes(encodeRcChannelsFrame(channels_));
            publishBytes(encodeLinkStatisticsFrame(100));
        });
    }

    // Channel N is channels()[N - 1].
    RcFrame & channels() { return channels_; }

    // Publishes the safety status as a real press of the physical E-Stop looks (button in, PLC
    // latched, contactor open) or as a released one. Engaged is what permits a calibration.
    // link_healthy is set because a node that is told the PLC link is down must report "cannot
    // verify" regardless of the pin values.
    void publishEStop(const bool engaged)
    {
        SafetyStatus message;
        message.hw_e_stop_user_button = engaged;
        message.latch_active = engaged;
        message.motor_contactor_engaged = !engaged;
        message.link_healthy = true;
        gpio_state_ = message;
        gpio_state_pub_->publish(message);
    }

    // The state reported from the rover: the software E-Stop (RC switch) has set the PLC latch
    // and the contactor has opened, but the physical button is released. Any Trigger call can
    // clear that latch, so it must not count as engaged.
    void publishSoftwareEStopOnly()
    {
        SafetyStatus message;
        message.hw_e_stop_user_button = false;
        message.latch_active = true;
        message.motor_contactor_engaged = false;
        message.link_healthy = true;
        gpio_state_ = message;
        gpio_state_pub_->publish(message);
    }

    // Publishes an otherwise-engaged safety status whose link the hardware interface reports as
    // down, so the values in it are last-known-good rather than current.
    void publishEStopWithUnhealthyLink()
    {
        SafetyStatus message;
        message.hw_e_stop_user_button = true;
        message.link_healthy = false;
        gpio_state_ = message;
        gpio_state_pub_->publish(message);
    }

    // Stops the 20 Hz republish, as a hardware interface that has gone away looks: the node's
    // last sample stays where it was and only ages.
    void stopSafetyStatus() { gpio_state_.reset(); }

    // Publishes an arbitrary byte chunk, so a test can split a frame across messages.
    void publishBytes(const Bytes & bytes)
    {
        std_msgs::msg::UInt8MultiArray message;
        message.data = bytes;
        serial_pub_->publish(message);
    }

    bool rcSubscribed() const { return serial_pub_->get_subscription_count() > 0; }

    bool cmdVelConnected() const { return cmd_vel_sub_->get_publisher_count() > 0; }

    std::size_t countZeros() const
    {
        std::size_t zeros = 0;
        for (const auto & msg : received) {
            if (msg.twist.linear.x == 0.0 && msg.twist.angular.z == 0.0) {
                zeros++;
            }
        }
        return zeros;
    }

    // The calibration surface, as a UI sees it: five services and one latched state topic.
    void connectCalibration()
    {
        start_client_ = node_->create_client<StartRcCalibration>("rc/calibration/start");
        sweep_client_ = node_->create_client<Trigger>("rc/calibration/sweep");
        finish_client_ = node_->create_client<Trigger>("rc/calibration/finish");
        cancel_client_ = node_->create_client<Trigger>("rc/calibration/cancel");
        apply_client_ = node_->create_client<SetRcCalibration>("rc/calibration/apply");

        calibration_state_sub_ = node_->create_subscription<RcCalibrationState>(
            "rc/calibration/state", rclcpp::QoS(1).reliable().transient_local(),
            [this](const RcCalibrationState & msg) { calibration_state = msg; });
    }

    bool calibrationServicesReady() const
    {
        return start_client_ && start_client_->service_is_ready() &&
               sweep_client_->service_is_ready() && finish_client_->service_is_ready() &&
               cancel_client_->service_is_ready() && apply_client_->service_is_ready();
    }

    rclcpp::Client<StartRcCalibration>::SharedPtr start() { return start_client_; }

    rclcpp::Client<Trigger>::SharedPtr sweep() { return sweep_client_; }

    rclcpp::Client<Trigger>::SharedPtr finishCalibration() { return finish_client_; }

    rclcpp::Client<Trigger>::SharedPtr cancelCalibration() { return cancel_client_; }

    rclcpp::Client<SetRcCalibration>::SharedPtr applyCalibration() { return apply_client_; }

    bool feeding{false};
    std::vector<Twist> received;
    int e_stop_set_calls{0};
    int e_stop_reset_calls{0};
    int latch_reset_calls{0};
    std::optional<RcCalibrationState> calibration_state;

private:

    rclcpp::Service<Trigger>::SharedPtr makeService(const std::string & name, int & counter)
    {
        return node_->create_service<Trigger>(
            name,
            [&counter](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
                counter++;
                response->success = true;
            });
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;
    rclcpp::Publisher<SafetyStatus>::SharedPtr gpio_state_pub_;
    rclcpp::TimerBase::SharedPtr gpio_timer_;
    std::optional<SafetyStatus> gpio_state_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<Trigger>::SharedPtr e_stop_set_srv_;
    rclcpp::Service<Trigger>::SharedPtr e_stop_reset_srv_;
    rclcpp::Service<Trigger>::SharedPtr latch_reset_srv_;
    rclcpp::Client<StartRcCalibration>::SharedPtr start_client_;
    rclcpp::Client<Trigger>::SharedPtr sweep_client_;
    rclcpp::Client<Trigger>::SharedPtr finish_client_;
    rclcpp::Client<Trigger>::SharedPtr cancel_client_;
    rclcpp::Client<SetRcCalibration>::SharedPtr apply_client_;
    rclcpp::Subscription<RcCalibrationState>::SharedPtr calibration_state_sub_;
    rclcpp::TimerBase::SharedPtr feed_timer_;
    RcFrame channels_;
};

}  // namespace

class TeleopNodeTest : public ::testing::Test
{

protected:

    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        // A per-test namespace keeps tests from seeing each other's lingering graph entities.
        const std::string ns = "/teleop_test_" + std::to_string(test_index_++);

        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__ns:=" + ns});
        options.parameter_overrides({
            rclcpp::Parameter("switch_settle_frames", 0),
            rclcpp::Parameter("channel_timeout_ms", 200),
            rclcpp::Parameter("diagnostic_updater.period", 0.1),
        });
        teleop_ = std::make_shared<RoverCrsfTeleopNode>("rover_crsf_teleop_node", options);

        helper_node_ = std::make_shared<rclcpp::Node>("rover_harness", ns);
        harness_ = std::make_unique<RoverHarness>(helper_node_);

        executor_.add_node(teleop_->get_node_base_interface());
        executor_.add_node(helper_node_);

        ASSERT_EQ(teleop_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        ASSERT_EQ(teleop_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

        ASSERT_TRUE(spinUntil([this]() { return harness_->rcSubscribed() && harness_->cmdVelConnected(); }))
            << "Teleop node and harness never discovered each other.";
    }

    void TearDown() override
    {
        executor_.remove_node(helper_node_);
        executor_.remove_node(teleop_->get_node_base_interface());
    }

    // Spins until `predicate` holds; false on timeout.
    bool spinUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 5s)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate()) {
                return true;
            }
            executor_.spin_some(10ms);
            executor_.spin_once(10ms);
        }
        return predicate();
    }

    // Spins for `duration` regardless - for asserting that something does NOT happen.
    void spinFor(const std::chrono::milliseconds duration)
    {
        spinUntil([]() { return false; }, duration);
    }

    bool waitForDeflectedCommand()
    {
        return spinUntil([this]() {
            return !harness_->received.empty() && harness_->received.back().twist.linear.x == 2.0;
        });
    }

    bool teleopSeesService(const std::string & name)
    {
        return spinUntil([this, name]() {
            const auto services = teleop_->get_service_names_and_types();
            return services.find(teleop_->get_namespace() + std::string("/") + name) != services.end();
        });
    }

    static inline int test_index_{0};

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<RoverCrsfTeleopNode> teleop_;
    rclcpp::Node::SharedPtr helper_node_;
    std::unique_ptr<RoverHarness> harness_;
};

TEST_F(TeleopNodeTest, DeflectedStickIsPublished)
{
    harness_->channels().channels[2] = kDefaultCrsfChannelMax;
    harness_->feeding = true;

    ASSERT_TRUE(waitForDeflectedCommand());
    EXPECT_EQ(harness_->received.back().header.frame_id, "base_link");
}

TEST_F(TeleopNodeTest, RcLinkDiagnosticFollowsTheLink)
{
    // /diagnostics is absolute, so match on this test's node name and take the latest level.
    std::optional<std::uint8_t> rc_link_level;
    std::string rc_link_message;
    auto diagnostics_sub = helper_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10,
        [&rc_link_level, &rc_link_message](const diagnostic_msgs::msg::DiagnosticArray & msg) {
            for (const auto & status : msg.status) {
                if (status.name == "rover_crsf_teleop_node: RC link") {
                    rc_link_level = status.level;
                    rc_link_message = status.message;
                }
            }
        });

    // No frames yet: waiting for the first frame.
    ASSERT_TRUE(spinUntil([&rc_link_level]() {
        return rc_link_level == diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }));

    harness_->feeding = true;
    ASSERT_TRUE(spinUntil([&rc_link_level]() {
        return rc_link_level == diagnostic_msgs::msg::DiagnosticStatus::OK;
    }));

    // Link loss warns (RC teleop is optional); the message tells it apart from "waiting".
    harness_->feeding = false;
    EXPECT_TRUE(spinUntil([&rc_link_level, &rc_link_message]() {
        return rc_link_level == diagnostic_msgs::msg::DiagnosticStatus::WARN &&
               rc_link_message.find("RC link lost") != std::string::npos;
    }));
}

TEST_F(TeleopNodeTest, ChannelsRateDiagnosticWarnsWithoutFrames)
{
    // No frames: FrequencyStatus would say ERROR ("No events recorded."), the node caps it at WARN.
    std::vector<std::uint8_t> rate_levels;
    auto diagnostics_sub = helper_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10,
        [&rate_levels](const diagnostic_msgs::msg::DiagnosticArray & msg) {
            for (const auto & status : msg.status) {
                if (status.name == "rover_crsf_teleop_node: RC channels rate") {
                    rate_levels.push_back(status.level);
                }
            }
        });

    ASSERT_TRUE(spinUntil([&rate_levels]() { return rate_levels.size() >= 3; }));
    for (const auto level : rate_levels) {
        EXPECT_EQ(level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    }
}

TEST_F(TeleopNodeTest, NoInputPublishesNothing)
{
    spinFor(300ms);

    EXPECT_TRUE(harness_->received.empty());
}

TEST_F(TeleopNodeTest, LostFramesPublishOneZeroThenSilence)
{
    harness_->channels().channels[2] = kDefaultCrsfChannelMax;
    harness_->feeding = true;
    ASSERT_TRUE(waitForDeflectedCommand());

    harness_->feeding = false;
    ASSERT_TRUE(spinUntil([this]() { return harness_->countZeros() == 1; }));

    const std::size_t count_after_zero = harness_->received.size();
    spinFor(500ms);

    EXPECT_EQ(harness_->received.size(), count_after_zero);
    EXPECT_EQ(harness_->countZeros(), 1u);
}

TEST_F(TeleopNodeTest, EStopSwitchEdgesCallTheServices)
{
    ASSERT_TRUE(teleopSeesService("hardware_interface/sw_user_e_stop_set"));
    ASSERT_TRUE(teleopSeesService("hardware_interface/sw_user_e_stop_reset"));

    harness_->feeding = true;
    // Let the switch's resting (high) position be seen before flipping it.
    ASSERT_TRUE(spinUntil([this]() { return !harness_->received.empty(); }));

    harness_->channels().channels[4] = kSwitchLow;
    ASSERT_TRUE(spinUntil([this]() { return harness_->e_stop_set_calls == 1; }));

    harness_->channels().channels[4] = kSwitchHigh;
    ASSERT_TRUE(spinUntil([this]() { return harness_->e_stop_reset_calls == 1; }));

    // A switch left in place does not re-fire.
    spinFor(200ms);
    EXPECT_EQ(harness_->e_stop_set_calls, 1);
    EXPECT_EQ(harness_->e_stop_reset_calls, 1);
    EXPECT_EQ(harness_->latch_reset_calls, 0);
}

TEST_F(TeleopNodeTest, DeactivateSendsZero)
{
    harness_->channels().channels[2] = kDefaultCrsfChannelMax;
    harness_->feeding = true;
    ASSERT_TRUE(waitForDeflectedCommand());

    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(spinUntil([this]() { return harness_->countZeros() == 1; }));

    // Inactive: frames keep coming but nothing more is commanded.
    const std::size_t count_after_zero = harness_->received.size();
    spinFor(200ms);
    EXPECT_EQ(harness_->received.size(), count_after_zero);
}

TEST_F(TeleopNodeTest, ConfigureRejectsAnInvalidChannel)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("e_stop_channel", 17)});
    auto node = std::make_shared<RoverCrsfTeleopNode>("rover_crsf_teleop_bad_config", options);

    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TeleopNodeTest, ConfigureRejectsDuplicateChannels)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("e_stop_channel", 3)});  // linear_x_channel
    auto node = std::make_shared<RoverCrsfTeleopNode>("rover_crsf_teleop_dup_channel", options);

    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TeleopNodeTest, ConfigureRejectsNegativeDeadband)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("channel_deadband", std::vector<int64_t>{-1})});
    auto node = std::make_shared<RoverCrsfTeleopNode>("rover_crsf_teleop_bad_deadband", options);

    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

// The calibrated-range check on this parameter is only a WARN, and it is skipped when there is no
// calibration - so without the hard bound a threshold off the wire domain configured silently and
// pinned both switch channels to one position for the life of the node.
TEST_F(TeleopNodeTest, ConfigureRejectsASwitchThresholdOffTheWire)
{
    for (const int threshold : {-1, 2048}) {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("channel_switch_threshold", threshold)});
        auto node = std::make_shared<RoverCrsfTeleopNode>(
            "rover_crsf_teleop_bad_threshold", options);

        EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
            << "channel_switch_threshold " << threshold << " should have failed configure";
    }
}

// --- RC calibration --------------------------------------------------------------------------

// Everything the calibration flow needs: the services connected, and teleop deactivated (which is
// the operator interlock the node enforces).
class TeleopCalibrationTest : public TeleopNodeTest
{

protected:

    void SetUp() override
    {
        TeleopNodeTest::SetUp();

        harness_->connectCalibration();
        ASSERT_TRUE(spinUntil([this]() { return harness_->calibrationServicesReady(); }))
            << "The calibration services never came up.";

        // What an operator does before calibrating. Without it every start is refused, which is
        // the point of the gate.
        engageEStop(true);
    }

    // Publishes the E-Stop state and spins until the node has taken it in, so a following
    // service call sees it rather than racing it.
    void engageEStop(const bool engaged)
    {
        harness_->publishEStop(engaged);
        ASSERT_TRUE(spinUntil([this, engaged]() {
            return harness_->calibration_state.has_value() &&
                   harness_->calibration_state->e_stop ==
                       (engaged ? RcCalibrationState::ESTOP_ENGAGED
                                : RcCalibrationState::ESTOP_RELEASED);
        })) << "the node never reported the E-Stop state the harness published";
    }

    // Calls a service and spins until the response arrives.
    template <typename ClientT, typename RequestT>
    auto call(const ClientT & client, const RequestT & request)
    {
        auto future = client->async_send_request(request);
        EXPECT_TRUE(spinUntil([&future]() {
            return future.wait_for(0s) == std::future_status::ready;
        })) << "the service never answered";
        return future.get();
    }

    auto startCalibration(const bool e_stop_confirmed)
    {
        auto request = std::make_shared<StartRcCalibration::Request>();
        request->e_stop_confirmed = e_stop_confirmed;
        return call(harness_->start(), request);
    }

    auto trigger(const rclcpp::Client<Trigger>::SharedPtr & client)
    {
        return call(client, std::make_shared<Trigger::Request>());
    }

    // Feeds `count` frames of the current channel values.
    void feedFrames(const std::size_t count)
    {
        for (std::size_t i = 0; i < count; ++i) {
            harness_->publishBytes(encodeRcChannelsFrame(harness_->channels()));
            executor_.spin_some(2ms);
        }
    }

    // The whole operator sequence: centre, sweep both sticks, finish. The sweep endpoints default
    // to the nominal CRSF ends; pass narrower ones to measure a transmitter that does not reach
    // them, which is what tells a calibrated endpoint apart from the shipped parameter.
    void measure(
        const int linear_rest, const int angular_rest,
        const int sweep_min = kDefaultCrsfChannelMin, const int sweep_max = kDefaultCrsfChannelMax)
    {
        ASSERT_TRUE(teleop_->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        ASSERT_TRUE(startCalibration(true)->success);

        harness_->channels().channels[2] = linear_rest;
        harness_->channels().channels[0] = angular_rest;
        feedFrames(kCenterSampleTarget);
        ASSERT_TRUE(trigger(harness_->sweep())->success);

        for (const int value : {sweep_min, sweep_max}) {
            harness_->channels().channels[2] = value;
            harness_->channels().channels[0] = value;
            feedFrames(3);
        }

        harness_->channels().channels[2] = linear_rest;
        harness_->channels().channels[0] = angular_rest;
        feedFrames(3);

        ASSERT_TRUE(trigger(harness_->finishCalibration())->success);
    }
};

TEST_F(TeleopCalibrationTest, StartIsRefusedWhileTheNodeIsActive)
{
    // The node is active from SetUp. A sweep at full throw would be a full-speed command.
    const auto response = startCalibration(true);

    EXPECT_FALSE(response->success);
    EXPECT_FALSE(response->message.empty());
}

TEST_F(TeleopCalibrationTest, StartIsRefusedWithoutTheEStopConfirmation)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    EXPECT_FALSE(startCalibration(false)->success);
}

TEST_F(TeleopCalibrationTest, TheStateTopicIsLatchedForWhoeverConnectsNext)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(startCalibration(true)->success);

    // A second page opening mid-session must learn at once that teleop is held off, without
    // waiting for the next heartbeat.
    auto late = helper_node_->create_subscription<RcCalibrationState>(
        "rc/calibration/state", rclcpp::QoS(1).reliable().transient_local(),
        [](const RcCalibrationState &) {});

    ASSERT_TRUE(spinUntil([this]() {
        return harness_->calibration_state.has_value() &&
               harness_->calibration_state->phase == RcCalibrationState::PHASE_CENTER;
    }));
    EXPECT_TRUE(harness_->calibration_state->teleop_inhibited);
}

TEST_F(TeleopCalibrationTest, ASweepMeasuresTheRestingPositionAndBothEndpoints)
{
    constexpr int kLinearRest = 1004;
    constexpr int kAngularRest = 987;

    measure(kLinearRest, kAngularRest);

    ASSERT_TRUE(spinUntil([this]() {
        return harness_->calibration_state.has_value() &&
               harness_->calibration_state->phase == RcCalibrationState::PHASE_REVIEW;
    }));

    const auto & measured = harness_->calibration_state->measured;
    EXPECT_EQ(measured.channel_mid[2], kLinearRest);
    EXPECT_EQ(measured.channel_mid[0], kAngularRest);
    EXPECT_EQ(measured.channel_min[2], kDefaultCrsfChannelMin);
    EXPECT_EQ(measured.channel_max[2], kDefaultCrsfChannelMax);
    EXPECT_TRUE(harness_->calibration_state->channel_moved[2]);
    EXPECT_TRUE(harness_->calibration_state->problems.empty());
}

TEST_F(TeleopCalibrationTest, SweepingTheEStopSwitchCallsNoSafetyServices)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(startCalibration(true)->success);
    feedFrames(kCenterSampleTarget);
    ASSERT_TRUE(trigger(harness_->sweep())->success);

    // Exactly what an operator does during the sweep: walk every switch end to end.
    for (const int value : {kSwitchLow, kSwitchHigh, kSwitchLow, kSwitchHigh}) {
        harness_->channels().channels[3] = value;
        harness_->channels().channels[4] = value;
        feedFrames(3);
    }

    ASSERT_TRUE(trigger(harness_->cancelCalibration())->success);

    EXPECT_EQ(harness_->e_stop_set_calls, 0);
    EXPECT_EQ(harness_->e_stop_reset_calls, 0);
    EXPECT_EQ(harness_->latch_reset_calls, 0);
}

TEST_F(TeleopCalibrationTest, ActivateIsRefusedWhileACalibrationIsRunning)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(startCalibration(true)->success);

    // Independent of the inhibit: the node refuses to become able to command at all.
    teleop_->activate();
    EXPECT_EQ(
        teleop_->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    ASSERT_TRUE(trigger(harness_->cancelCalibration())->success);
    EXPECT_EQ(teleop_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TeleopCalibrationTest, CancellingLetsTeleopCommandAgain)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(startCalibration(true)->success);
    ASSERT_TRUE(trigger(harness_->cancelCalibration())->success);
    ASSERT_EQ(teleop_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    harness_->received.clear();
    harness_->channels().channels[2] = kDefaultCrsfChannelMax;
    harness_->feeding = true;

    EXPECT_TRUE(waitForDeflectedCommand());
}

TEST_F(TeleopCalibrationTest, AnAppliedCalibrationChangesTheMappingWithoutARestart)
{
    constexpr int kLinearRest = 1004;

    measure(kLinearRest, 987);

    auto request = std::make_shared<SetRcCalibration::Request>();
    request->persist = false;   // an all-zero calibration means "apply what you measured"
    const auto response = call(harness_->applyCalibration(), request);
    ASSERT_TRUE(response->success) << response->message;

    ASSERT_EQ(teleop_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // The node is using the measured centre now: the resting count maps to exactly zero, where
    // before it was 12 counts off the nominal midpoint.
    harness_->received.clear();
    harness_->channels().channels[2] = kLinearRest;
    harness_->channels().channels[0] = 987;
    harness_->feeding = true;

    ASSERT_TRUE(spinUntil([this]() { return !harness_->received.empty(); }));
    EXPECT_DOUBLE_EQ(harness_->received.back().twist.linear.x, 0.0);

    // And the parameters agree with what the rover is actually using.
    const auto mid = teleop_->get_parameter("channel_in_mid").as_integer_array();
    ASSERT_EQ(mid.size(), RcFrame::kChannelCount);
    EXPECT_EQ(mid[2], kLinearRest);
}

// The three numbers on one diagnostic line have to come from one calibration. They used to be
// spliced: min and max from base_config_ (which deliberately keeps the *uncalibrated* endpoints,
// because applyCalibration() is always written against it) and mid from the calibration actually
// in force - so the operator read a triple that no calibration ever held.
TEST_F(TeleopCalibrationTest, TheCalibrationDiagnosticReportsOneCalibration)
{
    constexpr int kLinearRest = 1004;
    constexpr int kSweepMin = 300;    // inside the nominal 172..1811, so it differs from the
    constexpr int kSweepMax = 1700;   // shipped parameters and the splice would be visible

    measure(kLinearRest, 987, kSweepMin, kSweepMax);

    auto request = std::make_shared<SetRcCalibration::Request>();
    request->persist = false;
    ASSERT_TRUE(call(harness_->applyCalibration(), request)->success);

    std::string endpoints;
    auto diagnostics_sub = helper_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10, [&endpoints](const diagnostic_msgs::msg::DiagnosticArray & msg) {
            for (const auto & status : msg.status) {
                if (status.name != "rover_crsf_teleop_node: RC calibration") {
                    continue;
                }
                for (const auto & value : status.values) {
                    if (value.key == "linear_x endpoints") {
                        endpoints = value.value;
                    }
                }
            }
        });

    ASSERT_TRUE(spinUntil([&endpoints]() { return !endpoints.empty(); }))
        << "the RC calibration diagnostic never carried linear_x endpoints";

    // All three from the calibration that was just applied - not the shipped 172 / 1811.
    EXPECT_EQ(endpoints, std::to_string(kSweepMin) + " / " + std::to_string(kLinearRest) + " / " +
                             std::to_string(kSweepMax));
}

TEST_F(TeleopCalibrationTest, AMeasurementWaitingInReviewStillBlocksActivation)
{
    measure(1004, 987);

    // The session is not over until the measurement is applied or discarded, and activating in
    // between would have to rebuild teleop while it could command - which resets the link monitor
    // and publishes a spurious zero. So the operator has to decide first.
    teleop_->activate();
    EXPECT_EQ(
        teleop_->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    ASSERT_TRUE(trigger(harness_->cancelCalibration())->success);
    EXPECT_EQ(teleop_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TeleopCalibrationTest, NothingIsCommandedWhileACalibrationRuns)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(startCalibration(true)->success);

    harness_->received.clear();
    harness_->channels().channels[2] = kDefaultCrsfChannelMax;
    feedFrames(kCenterSampleTarget);

    EXPECT_TRUE(harness_->received.empty());
}

TEST_F(TeleopCalibrationTest, StartIsRefusedWhileTheEStopIsReleased)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    engageEStop(false);

    // The operator's confirmation does not override the rover's own answer.
    const auto response = startCalibration(true);

    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("Press the physical E-Stop"), std::string::npos);
}

// Reproduces the report from the rover: SW E-Stop on, latch set, physical button released - and
// Start went ahead. A software-set latch can be cleared remotely while the operator is standing
// next to the rover, so only the physical button may grant the permit.
TEST_F(TeleopCalibrationTest, StartIsRefusedOnASoftwareEStopWithoutThePhysicalButton)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    harness_->publishSoftwareEStopOnly();
    ASSERT_TRUE(spinUntil([this]() {
        return harness_->calibration_state.has_value() &&
               harness_->calibration_state->e_stop == RcCalibrationState::ESTOP_RELEASED;
    })) << "a software-only E-Stop was still reported as engaged";

    const auto response = startCalibration(true);

    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("Press the physical E-Stop"), std::string::npos);
}

TEST_F(TeleopCalibrationTest, ReleasingTheEStopCancelsARunningCalibration)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(startCalibration(true)->success);

    harness_->publishEStop(false);

    // The node's grace window is 1 s by default, and its watchdog re-evaluates twice a second,
    // so this resolves without anything here sleeping.
    ASSERT_TRUE(spinUntil([this]() {
        return harness_->calibration_state.has_value() &&
               harness_->calibration_state->phase == RcCalibrationState::PHASE_IDLE;
    }, 5s)) << "the session was never cancelled";

    EXPECT_FALSE(harness_->calibration_state->teleop_inhibited);
    EXPECT_EQ(harness_->calibration_state->e_stop, RcCalibrationState::ESTOP_RELEASED);
    EXPECT_NE(harness_->calibration_state->message.find("E-Stop"), std::string::npos);
}

TEST_F(TeleopCalibrationTest, TheStateTopicReportsTheVerifiedEStop)
{
    // The page draws its indicator from this, and enables Start from it.
    ASSERT_TRUE(spinUntil([this]() {
        return harness_->calibration_state.has_value() &&
               harness_->calibration_state->e_stop == RcCalibrationState::ESTOP_ENGAGED;
    }));

    engageEStop(false);
    EXPECT_EQ(harness_->calibration_state->e_stop, RcCalibrationState::ESTOP_RELEASED);
}

// A hardware interface that stops publishing leaves an "engaged" sample behind. Once it is older
// than e_stop_state_timeout_s it must read "cannot verify", never what it last said - otherwise a
// dead publisher would keep granting the permit to sweep the sticks.
TEST_F(TeleopCalibrationTest, AnEStopSampleThatStopsArrivingAgesIntoUnverified)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    harness_->stopSafetyStatus();

    // 1 s timeout by default, re-evaluated by the watchdog twice a second.
    ASSERT_TRUE(spinUntil([this]() {
        return harness_->calibration_state.has_value() &&
               harness_->calibration_state->e_stop == RcCalibrationState::ESTOP_UNKNOWN;
    }, 5s)) << "a sample that stopped arriving was never aged into unverified";

    const auto response = startCalibration(true);

    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("Cannot verify"), std::string::npos);
}

// Pin values sent while the PLC link is down are last-known-good, not current, so the node drops
// the sample rather than trusting it.
TEST_F(TeleopCalibrationTest, AnUnhealthySafetyLinkReportsTheEStopAsUnverified)
{
    harness_->publishEStopWithUnhealthyLink();

    ASSERT_TRUE(spinUntil([this]() {
        return harness_->calibration_state.has_value() &&
               harness_->calibration_state->e_stop == RcCalibrationState::ESTOP_UNKNOWN;
    })) << "an unhealthy safety link was still reported as a verified E-Stop";
}

// Without the harness publishing safety_status at all, which is how a bench or a sim looks.
class TeleopCalibrationNoSafetyIoTest : public TeleopNodeTest
{

protected:

    void SetUp() override
    {
        TeleopNodeTest::SetUp();
        harness_->connectCalibration();
        ASSERT_TRUE(spinUntil([this]() { return harness_->calibrationServicesReady(); }));
    }
};

TEST_F(TeleopCalibrationNoSafetyIoTest, StartIsRefusedWhenTheEStopWasNeverPublished)
{
    ASSERT_EQ(teleop_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    auto request = std::make_shared<StartRcCalibration::Request>();
    request->e_stop_confirmed = true;
    auto future = harness_->start()->async_send_request(request);
    ASSERT_TRUE(spinUntil([&future]() {
        return future.wait_for(0s) == std::future_status::ready;
    }));
    const auto response = future.get();

    // Refused rather than assumed safe: nobody can vouch for this rover.
    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("Cannot verify"), std::string::npos);
}

}  // namespace rover_crsf_teleop
