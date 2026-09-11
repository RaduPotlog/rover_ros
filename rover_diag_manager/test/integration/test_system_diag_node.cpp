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
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_diag_manager/domain/ports/system_metrics_source_port.hpp"
#include "rover_diag_manager/infrastructure/system_diag_node.hpp"
#include "rover_msgs/msg/system_status.hpp"

using namespace std::chrono_literals;
using SystemStatusMsg = rover_msgs::msg::SystemStatus;
using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

/** @brief Deterministic SystemMetricsSourcePort double for integration tests. */
class FakeMetricsSource : public rover_diag_manager::domain::SystemMetricsSourcePort
{
public:
    rover_diag_manager::domain::SystemSample sample() override
    {
        rover_diag_manager::domain::SystemSample s;
        s.core_usages = {50.0f};
        s.cpu_mean_usage = 50.0f;
        s.cpu_temperature = 60.0f;
        s.ram_usage = 70.0f;
        s.disk_usage = 80.0f;
        return s;
    }
};

class SystemDiagNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
    static void TearDownTestSuite() {rclcpp::shutdown();}

    void startNode(const std::vector<rclcpp::Parameter> & overrides)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(overrides);
        diag_node_ = std::make_shared<rover_diag_manager::SystemDiagNode>(
            "rover_diag_manager_node", std::make_shared<FakeMetricsSource>(), options);

        tester_ = std::make_shared<rclcpp::Node>("tester");
        status_sub_ = tester_->create_subscription<SystemStatusMsg>(
            "system_status", 10,
            [this](SystemStatusMsg::SharedPtr msg) {status_msgs_.push_back(*msg);});
        diagnostics_sub_ = tester_->create_subscription<DiagnosticArrayMsg>(
            "/diagnostics", 10,
            [this](DiagnosticArrayMsg::SharedPtr msg) {diagnostic_msgs_.push_back(*msg);});

        executor_.add_node(diag_node_);
        executor_.add_node(tester_);
    }

    void TearDown() override
    {
        executor_.remove_node(tester_);
        executor_.remove_node(diag_node_);
    }

    /** Spins until `done` returns true or `timeout` elapses; returns `done()`. */
    bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some(50ms);
        }
        return done();
    }

    static const DiagnosticStatusMsg * findOsStatus(const DiagnosticArrayMsg & array)
    {
        for (const auto & status : array.status) {
            if (status.name.find("OS status") != std::string::npos) {
                return &status;
            }
        }
        return nullptr;
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rover_diag_manager::SystemDiagNode> diag_node_;
    rclcpp::Node::SharedPtr tester_;
    rclcpp::Subscription<SystemStatusMsg>::SharedPtr status_sub_;
    rclcpp::Subscription<DiagnosticArrayMsg>::SharedPtr diagnostics_sub_;
    std::vector<SystemStatusMsg> status_msgs_;
    std::vector<DiagnosticArrayMsg> diagnostic_msgs_;
};

}  // namespace

TEST_F(SystemDiagNodeTest, PublishesSystemStatusAtConfiguredFrequency)
{
    startNode({rclcpp::Parameter("publish_frequency", 10.0)});

    ASSERT_TRUE(spinUntil([this] {return !status_msgs_.empty();}, 5s));

    const auto & msg = status_msgs_.front();
    EXPECT_FLOAT_EQ(msg.avg_load_percent, 50.0f);
    EXPECT_FLOAT_EQ(msg.cpu_temp, 60.0f);
    EXPECT_FLOAT_EQ(msg.ram_usage_percent, 70.0f);
    EXPECT_FLOAT_EQ(msg.disc_usage_percent, 80.0f);
}

TEST_F(SystemDiagNodeTest, DiagnosticsReportWarnWhenBelowFakeSampleValues)
{
    startNode({
        rclcpp::Parameter("publish_frequency", 10.0),
        rclcpp::Parameter("cpu_usage_warn_threshold", 1.0),
        rclcpp::Parameter("cpu_temperature_warn_threshold", 1.0),
        rclcpp::Parameter("ram_usage_warn_threshold", 1.0),
        rclcpp::Parameter("disk_usage_warn_threshold", 1.0),
    });

    ASSERT_TRUE(spinUntil([this] {return !status_msgs_.empty();}, 5s));

    // diagnostic_updater publishes on its own ~1 Hz period; wait for it to catch up.
    ASSERT_TRUE(spinUntil(
        [this] {
            for (const auto & array : diagnostic_msgs_) {
                if (const auto * status = findOsStatus(array)) {
                    return status->level == DiagnosticStatusMsg::WARN;
                }
            }
            return false;
        }, 5s));
}

TEST(SystemDiagNodeParameters, RejectsOutOfRangeValues)
{
    rclcpp::init(0, nullptr);

    for (const auto & parameter : {rclcpp::Parameter("publish_frequency", 0.0),
                                   rclcpp::Parameter("cpu_usage_warn_threshold", -1.0)})
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({parameter});
        EXPECT_THROW(
            rover_diag_manager::SystemDiagNode(
                "rover_diag_manager_node", std::make_shared<FakeMetricsSource>(), options),
            rclcpp::exceptions::InvalidParameterValueException) << parameter.get_name();
    }

    rclcpp::shutdown();
}
