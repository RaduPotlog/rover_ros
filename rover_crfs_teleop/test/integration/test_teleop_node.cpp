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

// Integration test: RoverCrfsTeleopNode on real ROS topics and services. A helper node plays the
// CRSF receiver (rc/channels + rc/link at 50 Hz), the hardware interface (the three E-Stop
// Trigger services) and twist_mux (subscribes to the cmd_vel output).
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

#include "crsf_receiver_msg/msg/crsf_channels16.hpp"
#include "crsf_receiver_msg/msg/crsf_link_info.hpp"
#include "rover_crfs_teleop/domain/stick_mapping.hpp"
#include "rover_crfs_teleop/infrastructure/rover_crfs_teleop_node.hpp"

namespace rover_crfs_teleop
{
namespace
{

using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::TwistStamped;
using Trigger = std_srvs::srv::Trigger;

constexpr int kSwitchLow = kDefaultCrsfChannelMin;
constexpr int kSwitchHigh = kDefaultCrsfChannelMax;

// Plays everything around the teleop node.
class RoverHarness
{

public:

    explicit RoverHarness(const rclcpp::Node::SharedPtr & node)
    : node_(node)
    {
        const auto rc_qos = rclcpp::QoS(1).best_effort().durability_volatile();
        channels_pub_ = node_->create_publisher<crsf_receiver_msg::msg::CRSFChannels16>("rc/channels", rc_qos);
        link_pub_ = node_->create_publisher<crsf_receiver_msg::msg::CRSFLinkInfo>("rc/link", rc_qos);

        cmd_vel_sub_ = node_->create_subscription<Twist>(
            "teleop_elrs_cmd_vel_stamped", 10,
            [this](const Twist & msg) { received.push_back(msg); });

        e_stop_set_srv_ = makeService("hardware_interface/sw_user_e_stop_set", e_stop_set_calls);
        e_stop_reset_srv_ = makeService("hardware_interface/sw_user_e_stop_reset", e_stop_reset_calls);
        latch_reset_srv_ = makeService("hardware_interface/sw_e_stop_latch_reset", latch_reset_calls);

        channels_.ch1 = channels_.ch2 = channels_.ch3 = channels_.ch6 = kDefaultCrsfChannelMid;
        channels_.ch4 = channels_.ch5 = kSwitchHigh;

        // The receiver's 50 Hz frame stream, while `feeding` is set.
        feed_timer_ = node_->create_wall_timer(20ms, [this]() {
            if (!feeding) {
                return;
            }
            channels_pub_->publish(channels_);
            crsf_receiver_msg::msg::CRSFLinkInfo link;
            link.uplink_status.data = 100;
            link_pub_->publish(link);
        });
    }

    crsf_receiver_msg::msg::CRSFChannels16 & channels() { return channels_; }

    bool rcSubscribed() const
    {
        return channels_pub_->get_subscription_count() > 0 && link_pub_->get_subscription_count() > 0;
    }

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

    bool feeding{false};
    std::vector<Twist> received;
    int e_stop_set_calls{0};
    int e_stop_reset_calls{0};
    int latch_reset_calls{0};

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
    rclcpp::Publisher<crsf_receiver_msg::msg::CRSFChannels16>::SharedPtr channels_pub_;
    rclcpp::Publisher<crsf_receiver_msg::msg::CRSFLinkInfo>::SharedPtr link_pub_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<Trigger>::SharedPtr e_stop_set_srv_;
    rclcpp::Service<Trigger>::SharedPtr e_stop_reset_srv_;
    rclcpp::Service<Trigger>::SharedPtr latch_reset_srv_;
    rclcpp::TimerBase::SharedPtr feed_timer_;
    crsf_receiver_msg::msg::CRSFChannels16 channels_;
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
        teleop_ = std::make_shared<RoverCrfsTeleopNode>("rover_crfs_teleop_node", options);

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
    std::shared_ptr<RoverCrfsTeleopNode> teleop_;
    rclcpp::Node::SharedPtr helper_node_;
    std::unique_ptr<RoverHarness> harness_;
};

TEST_F(TeleopNodeTest, DeflectedStickIsPublished)
{
    harness_->channels().ch3 = kDefaultCrsfChannelMax;
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
                if (status.name == "rover_crfs_teleop_node: RC link") {
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
                if (status.name == "rover_crfs_teleop_node: RC channels rate") {
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
    harness_->channels().ch3 = kDefaultCrsfChannelMax;
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

    harness_->channels().ch5 = kSwitchLow;
    ASSERT_TRUE(spinUntil([this]() { return harness_->e_stop_set_calls == 1; }));

    harness_->channels().ch5 = kSwitchHigh;
    ASSERT_TRUE(spinUntil([this]() { return harness_->e_stop_reset_calls == 1; }));

    // A switch left in place does not re-fire.
    spinFor(200ms);
    EXPECT_EQ(harness_->e_stop_set_calls, 1);
    EXPECT_EQ(harness_->e_stop_reset_calls, 1);
    EXPECT_EQ(harness_->latch_reset_calls, 0);
}

TEST_F(TeleopNodeTest, DeactivateSendsZero)
{
    harness_->channels().ch3 = kDefaultCrsfChannelMax;
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
    auto node = std::make_shared<RoverCrfsTeleopNode>("rover_crfs_teleop_bad_config", options);

    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TeleopNodeTest, ConfigureRejectsDuplicateChannels)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("e_stop_channel", 3)});  // linear_x_channel
    auto node = std::make_shared<RoverCrfsTeleopNode>("rover_crfs_teleop_dup_channel", options);

    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TeleopNodeTest, ConfigureRejectsNegativeDeadband)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("channel_deadband", -1)});
    auto node = std::make_shared<RoverCrfsTeleopNode>("rover_crfs_teleop_bad_deadband", options);

    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

}  // namespace rover_crfs_teleop
