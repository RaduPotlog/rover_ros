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

#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rover_gps_heading/domain/geo_math.hpp"
#include "rover_gps_heading/infrastructure/rover_gps_heading_node.hpp"

using namespace std::chrono_literals;
using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
using ImuMsg = sensor_msgs::msg::Imu;
using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
using OdometryMsg = nav_msgs::msg::Odometry;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
constexpr char kNamespace[] = "/rover_gps_heading_test";

class RoverGpsHeadingNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
    static void TearDownTestSuite() {rclcpp::shutdown();}

    void startNode(const std::vector<rclcpp::Parameter> & overrides)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(overrides);
        gps_node_ = std::make_shared<rover_gps_heading::RoverGpsHeadingNode>("rover_gps_heading_node", kNamespace, options);
        gps_node_->init();

        tester_ = std::make_shared<rclcpp::Node>("tester", kNamespace);
        fix_pub_ = tester_->create_publisher<NavSatFixMsg>("gps/fix", 10);
        odom_pub_ = tester_->create_publisher<OdometryMsg>("odom", 10);
        heading_sub_ = tester_->create_subscription<ImuMsg>(
            "gps/heading_imu", 10, [this](ImuMsg::SharedPtr msg) {headings_.push_back(*msg);});
        // diagnostic_updater publishes on the absolute /diagnostics topic.
        diagnostics_sub_ = tester_->create_subscription<DiagnosticArrayMsg>(
            "/diagnostics", 10,
            [this](DiagnosticArrayMsg::SharedPtr msg) {
                for (const auto & status : msg->status) {
                    latest_statuses_[status.name] = status;
                }
            });
        reset_client_ =
            tester_->create_client<std_srvs::srv::Trigger>("gps/reset_heading_alignment");

        executor_.add_node(gps_node_);
        executor_.add_node(tester_);
    }

    void TearDown() override
    {
        // Tests that only construct a node never call startNode().
        if (tester_) {
            executor_.remove_node(tester_);
        }
        if (gps_node_) {
            executor_.remove_node(gps_node_);
        }
    }

    /** Spins until `done` returns true or `timeout` elapses; returns `done()`. */
    bool spinUntil(
        const std::function<bool()> & done, std::chrono::milliseconds timeout,
        const std::function<void()> & each_iteration = {})
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            if (each_iteration) {
                each_iteration();
            }
            executor_.spin_some(20ms);
        }
        return done();
    }

    bool waitForDiscovery()
    {
        return spinUntil(
            [this] {
                return fix_pub_->get_subscription_count() > 0 &&
                       odom_pub_->get_subscription_count() > 0 &&
                       heading_sub_->get_publisher_count() > 0 &&
                       diagnostics_sub_->get_publisher_count() > 0;
            }, 5s);
    }

    std::optional<DiagnosticStatusMsg> status(const std::string & task) const
    {
        const auto it = latest_statuses_.find("rover_gps_heading_node: " + task);
        if (it == latest_statuses_.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    /** Publishes odometry (yaw 0, 1 m/s forward) and a fix `north_m` north of the origin. */
    void publishDrivingNorth(double north_m)
    {
        OdometryMsg odom;
        odom.pose.pose.orientation.w = 1.0;
        odom.twist.twist.linear.x = 1.0;
        odom_pub_->publish(odom);

        NavSatFixMsg fix;
        fix.latitude = 45.0 + north_m / kEarthRadiusM * 180.0 / kPi;
        fix.longitude = 25.0;
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_APPROXIMATED;
        fix.position_covariance[0] = 1.0;
        fix.position_covariance[4] = 1.0;
        fix_pub_->publish(fix);
    }

    static std::vector<rclcpp::Parameter> fastAlignment()
    {
        return {
            rclcpp::Parameter("publish_heading", true),
            rclcpp::Parameter("heading_frame_id", "rover/base_link"),
            rclcpp::Parameter("alignment.min_segment_length_m", 1.0),
            rclcpp::Parameter("alignment.required_segments", 2),
            rclcpp::Parameter("alignment.max_fix_gap_s", 30.0),
            rclcpp::Parameter("alignment.max_odometry_age_s", 30.0),
        };
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rover_gps_heading::RoverGpsHeadingNode> gps_node_;
    rclcpp::Node::SharedPtr tester_;
    rclcpp::Publisher<NavSatFixMsg>::SharedPtr fix_pub_;
    rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub_;
    rclcpp::Subscription<ImuMsg>::SharedPtr heading_sub_;
    rclcpp::Subscription<DiagnosticArrayMsg>::SharedPtr diagnostics_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
    std::vector<ImuMsg> headings_;
    std::map<std::string, DiagnosticStatusMsg> latest_statuses_;
};

}  // namespace

TEST_F(RoverGpsHeadingNodeTest, WarnsWhileUnaligned)
{
    startNode({});
    ASSERT_TRUE(waitForDiscovery());

    // publish_heading defaults to true: an unaligned heading is something to act on.
    ASSERT_TRUE(spinUntil([this] {return status("Heading alignment").has_value();}, 5s));
    EXPECT_EQ(status("Heading alignment")->level, DiagnosticStatusMsg::WARN);
}

TEST_F(RoverGpsHeadingNodeTest, HeadingOutputDisabled)
{
    startNode({rclcpp::Parameter("publish_heading", false)});
    ASSERT_TRUE(waitForDiscovery());

    // No heading output while publish_heading is false, and unaligned is not a fault then.
    ASSERT_TRUE(spinUntil(
        [this] {
            const auto alignment = status("Heading alignment");
            return alignment && alignment->level == DiagnosticStatusMsg::OK;
        }, 5s, [this] {publishDrivingNorth(0.0);}));
    EXPECT_TRUE(headings_.empty());
}

TEST_F(RoverGpsHeadingNodeTest, PublishesHeadingAfterAlignment)
{
    startNode(fastAlignment());
    ASSERT_TRUE(waitForDiscovery());

    double north_m = 0.0;
    ASSERT_TRUE(spinUntil(
        [this] {return !headings_.empty();}, 20s,
        [&] {
            publishDrivingNorth(north_m);
            north_m += 0.25;
        }));

    const ImuMsg & heading = headings_.back();
    EXPECT_EQ(heading.header.frame_id, "rover/base_link");
    const double yaw = rover_gps_heading::domain::yawFromQuaternion(
        heading.orientation.x, heading.orientation.y, heading.orientation.z, heading.orientation.w);
    // Odom yaw 0 while moving north → ENU yaw +90 deg.
    EXPECT_NEAR(yaw, kPi / 2.0, 0.02);

    ASSERT_TRUE(spinUntil(
        [this] {
            const auto alignment = status("Heading alignment");
            return alignment && alignment->level == DiagnosticStatusMsg::OK;
        }, 5s));

    // Reset through the service: the node stops publishing and reports WARN again.
    ASSERT_TRUE(reset_client_->wait_for_service(5s));
    auto future = reset_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(executor_.spin_until_future_complete(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);

    ASSERT_TRUE(spinUntil(
        [this] {
            const auto alignment = status("Heading alignment");
            return alignment && alignment->level == DiagnosticStatusMsg::WARN;
        }, 5s));
}

TEST_F(RoverGpsHeadingNodeTest, RejectsInvalidAlignmentConfig)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("alignment.required_segments", 0)});
    EXPECT_THROW(
        rover_gps_heading::RoverGpsHeadingNode("rover_gps_heading_node", kNamespace, options),
        std::exception);
}
