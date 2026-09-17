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

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

#include "rover_gps/infrastructure/rover_gps_driver_node.hpp"

using namespace std::chrono_literals;
using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
using TimeReferenceMsg = sensor_msgs::msg::TimeReference;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

namespace
{

constexpr char kNamespace[] = "/rover_gps_driver_test";

constexpr char kGga[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
constexpr char kVtg[] = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48";

/** @brief Sends one UDP datagram to the driver's loopback port. */
bool sendDatagram(uint16_t port, const std::string & payload)
{
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);

    const ssize_t sent = ::sendto(
        fd, payload.data(), payload.size(), 0, reinterpret_cast<const sockaddr *>(&address),
        sizeof(address));
    ::close(fd);
    return sent == static_cast<ssize_t>(payload.size());
}

/** @brief True when a UDP port can be bound, i.e. the driver is not holding it. */
bool canBind(uint16_t port)
{
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);

    const bool bound = ::bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0;
    ::close(fd);
    return bound;
}

}  // namespace

class RoverGpsDriverNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
    static void TearDownTestSuite() {rclcpp::shutdown();}

    void startNode(const std::vector<rclcpp::Parameter> & extra_overrides = {})
    {
        std::vector<rclcpp::Parameter> overrides{
            rclcpp::Parameter("ip", "127.0.0.1"),
            // Bind an ephemeral port so parallel test runs never collide.
            rclcpp::Parameter("port", 0),
            rclcpp::Parameter("timeout_sec", 1),
            rclcpp::Parameter("frame_id", "gps_link"),
            rclcpp::Parameter("time_ref_source", "gps"),
        };
        overrides.insert(overrides.end(), extra_overrides.begin(), extra_overrides.end());

        rclcpp::NodeOptions options;
        options.parameter_overrides(overrides);
        driver_node_ = std::make_shared<rover_gps::RoverGpsDriverNode>(
            "rover_gps_driver", kNamespace, options);

        tester_ = std::make_shared<rclcpp::Node>("driver_tester", kNamespace);
        fix_sub_ = tester_->create_subscription<NavSatFixMsg>(
            "fix", 10, [this](NavSatFixMsg::SharedPtr msg) {fixes_.push_back(*msg);});
        velocity_sub_ = tester_->create_subscription<TwistStampedMsg>(
            "vel", 10, [this](TwistStampedMsg::SharedPtr msg) {velocities_.push_back(*msg);});
        time_reference_sub_ = tester_->create_subscription<TimeReferenceMsg>(
            "time_reference", 10,
            [this](TimeReferenceMsg::SharedPtr msg) {time_references_.push_back(*msg);});

        executor_.add_node(driver_node_->get_node_base_interface());
        executor_.add_node(tester_);
    }

    void TearDown() override
    {
        if (tester_) {
            executor_.remove_node(tester_);
        }
        if (driver_node_) {
            executor_.remove_node(driver_node_->get_node_base_interface());
        }
    }

    /** @brief Spins until `done` returns true or `timeout` elapses; returns `done()`. */
    bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some(20ms);
        }
        return done();
    }

    /** @brief Drives configure + activate and waits for the subscriptions to match. */
    void activate()
    {
        ASSERT_EQ(
            driver_node_->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        ASSERT_EQ(
            driver_node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

        ASSERT_TRUE(spinUntil(
            [this] {
                return fix_sub_->get_publisher_count() > 0 &&
                       velocity_sub_->get_publisher_count() > 0;
            }, 5s)) << "publishers and subscriptions never matched";
    }

    /** @brief Sends `payload` repeatedly until `done` holds, to ride out discovery races. */
    bool sendUntil(const std::string & payload, const std::function<bool()> & done)
    {
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            EXPECT_TRUE(sendDatagram(driver_node_->boundPort(), payload));
            executor_.spin_some(20ms);
            std::this_thread::sleep_for(20ms);
        }
        return done();
    }

    std::shared_ptr<rover_gps::RoverGpsDriverNode> driver_node_;
    std::shared_ptr<rclcpp::Node> tester_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Subscription<NavSatFixMsg>::SharedPtr fix_sub_;
    rclcpp::Subscription<TwistStampedMsg>::SharedPtr velocity_sub_;
    rclcpp::Subscription<TimeReferenceMsg>::SharedPtr time_reference_sub_;

    std::vector<NavSatFixMsg> fixes_;
    std::vector<TwistStampedMsg> velocities_;
    std::vector<TimeReferenceMsg> time_references_;
};

TEST_F(RoverGpsDriverNodeTest, StartsUnconfiguredAndBindsNoPort)
{
    startNode();
    EXPECT_EQ(
        driver_node_->get_current_state().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(driver_node_->boundPort(), 0);
}

TEST_F(RoverGpsDriverNodeTest, PublishesAFixFromAUdpDatagram)
{
    startNode();
    activate();

    ASSERT_TRUE(sendUntil(std::string(kGga) + "\r\n", [this] {return !fixes_.empty();}))
        << "no NavSatFix arrived";

    const NavSatFixMsg & fix = fixes_.front();
    EXPECT_EQ(fix.header.frame_id, "gps_link");
    EXPECT_EQ(fix.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    EXPECT_EQ(fix.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
    EXPECT_NEAR(fix.latitude, 48.1173, 1e-4);
    EXPECT_NEAR(fix.longitude, 11.516667, 1e-6);
    EXPECT_NEAR(fix.altitude, 592.3, 1e-3);
    EXPECT_EQ(
        fix.position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED);
    EXPECT_NEAR(fix.position_covariance[0], (0.9 * 4.0) * (0.9 * 4.0), 1e-6);

    // GGA also carries UTC, so a time reference goes out alongside the fix.
    EXPECT_TRUE(spinUntil([this] {return !time_references_.empty();}, 2s));
}

TEST_F(RoverGpsDriverNodeTest, AppliesTfPrefixToTheFrameId)
{
    startNode({rclcpp::Parameter("tf_prefix", "rover")});
    activate();

    ASSERT_TRUE(sendUntil(std::string(kGga) + "\r\n", [this] {return !fixes_.empty();}));
    EXPECT_EQ(fixes_.front().header.frame_id, "rover/gps_link");
}

TEST_F(RoverGpsDriverNodeTest, ParsesEverySentenceOfAMultiSentenceDatagram)
{
    startNode();
    activate();

    // The GGA is followed by CRLF, so upstream's end-anchored match would have dropped it and only
    // the trailing VTG would have been seen - and without a fix, VTG is suppressed too.
    const std::string datagram = std::string(kGga) + "\r\n" + kVtg + "\r\n";

    ASSERT_TRUE(sendUntil(datagram, [this] {return !fixes_.empty() && !velocities_.empty();}))
        << "expected both a fix and a velocity from one datagram";

    EXPECT_NEAR(velocities_.front().twist.linear.x, 5.5 * 0.514444444444 * std::sin(0.954548), 1e-3);
    EXPECT_NEAR(velocities_.front().twist.linear.y, 5.5 * 0.514444444444 * std::cos(0.954548), 1e-3);
}

TEST_F(RoverGpsDriverNodeTest, IgnoresASentenceWithABadChecksum)
{
    startNode();
    activate();

    const std::string bad =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48\r\n";
    for (int i = 0; i < 10; ++i) {
        ASSERT_TRUE(sendDatagram(driver_node_->boundPort(), bad));
        executor_.spin_some(20ms);
        std::this_thread::sleep_for(10ms);
    }
    spinUntil([] {return false;}, 300ms);

    EXPECT_TRUE(fixes_.empty());
    EXPECT_GT(driver_node_->statistics().checksum_failed, 0u);
}

TEST_F(RoverGpsDriverNodeTest, DeactivateReleasesThePort)
{
    startNode();
    activate();

    const uint16_t port = driver_node_->boundPort();
    ASSERT_NE(port, 0);
    // Held while active.
    EXPECT_FALSE(canBind(port));

    ASSERT_EQ(
        driver_node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    EXPECT_TRUE(canBind(port)) << "the UDP port was not released on deactivate";
}

TEST_F(RoverGpsDriverNodeTest, SurvivesAnActivateDeactivateCycle)
{
    startNode();
    activate();

    ASSERT_EQ(
        driver_node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(driver_node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ASSERT_TRUE(spinUntil([this] {return fix_sub_->get_publisher_count() > 0;}, 5s));
    ASSERT_TRUE(sendUntil(std::string(kGga) + "\r\n", [this] {return !fixes_.empty();}))
        << "the driver stopped publishing after a deactivate/activate cycle";
}

TEST_F(RoverGpsDriverNodeTest, CleanupReturnsToUnconfigured)
{
    startNode();
    activate();

    ASSERT_EQ(
        driver_node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(
        driver_node_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}
