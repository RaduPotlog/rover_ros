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
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include "rover_safety/behavior_tree_utils.hpp"
#include "rover_safety/infrastructure/shutdown_command.hpp"

#include "../unit/plugins/plugin_test_utils.hpp"

using namespace std::chrono_literals;
using TriggerSrv = std_srvs::srv::Trigger;

namespace
{

/**
 * Loads RoverSafetyBT.btproj with the plugin lists of config/rover_safety.yaml, exactly as
 * rover_safety_node does, and runs the RoverShutdown tree against a fake E-Stop service. The power-off
 * command is replaced by one that records the reason.
 */
class ShutdownTreeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        bt_node_ = std::make_shared<nav2::LifecycleNode>("shutdown_tree_bt_node");
        server_node_ = std::make_shared<rclcpp::Node>("shutdown_tree_server_node");
        e_stop_service_ = server_node_->create_service<TriggerSrv>(
            "hardware_interface/sw_user_e_stop_set",
            [this](const TriggerSrv::Request::SharedPtr, TriggerSrv::Response::SharedPtr response) {
                ++e_stop_calls_;
                response->success = true;
            });
        executor_.add_node(server_node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });

        const auto params = YAML::LoadFile(ROVER_SAFETY_CONFIG)["/**"]["rover_safety_node"]["ros__parameters"];
        rover_safety::registerBehaviorTree(
            factory_, ROVER_SAFETY_BT_PROJECT,
            params["plugin_libs"].as<std::vector<std::string>>(),
            params["ros_plugin_libs"].as<std::vector<std::string>>());

        const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
        reason_file_ = std::filesystem::temp_directory_path() /
                       (std::string("rover_safety_") + test_info->name() + "_reason.txt");
        std::filesystem::remove(reason_file_);
        hosts_file_ = std::filesystem::temp_directory_path() /
                      (std::string("rover_safety_") + test_info->name() + "_hosts.yaml");
        std::filesystem::remove(hosts_file_);
    }

    void TearDown() override
    {
        executor_.cancel();
        spin_thread_.join();
        std::filesystem::remove(reason_file_);
        std::filesystem::remove(hosts_file_);
    }

    BT::Blackboard::Ptr createBlackboard(const std::string & power_off_command, const std::string & reason)
    {
        auto blackboard = BT::Blackboard::create();
        blackboard->set<nav2::LifecycleNode::SharedPtr>("node", bt_node_);
        blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
        blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 2000ms);
        blackboard->set<std::chrono::milliseconds>("server_timeout", 500ms);
        blackboard->set<std::string>("SHUTDOWN_HOSTS_FILE", ROVER_SAFETY_SHUTDOWN_HOSTS);
        blackboard->set<float>("SHUTDOWN_COMMAND_TIMEOUT", 5.0f);
        blackboard->set<std::string>(
            "SHUTDOWN_LOCALHOST_COMMAND",
            rover_safety::infrastructure::buildShutdownCommand(power_off_command, reason));
        return blackboard;
    }

    /** Ticks at the node's 10 Hz until the tree leaves RUNNING. */
    static BT::NodeStatus tickUntilDone(BT::Tree & tree)
    {
        auto status = BT::NodeStatus::RUNNING;
        for (int i = 0; i < 200 && status == BT::NodeStatus::RUNNING; ++i) {
            status = tree.tickOnce();
            tree.sleep(100ms);
        }
        return status;
    }

    std::string readReasonFile() const
    {
        std::ifstream file(reason_file_);
        return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    }

    nav2::LifecycleNode::SharedPtr bt_node_;
    rclcpp::Node::SharedPtr server_node_;
    rclcpp::Service<TriggerSrv>::SharedPtr e_stop_service_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
    std::atomic<int> e_stop_calls_{0};
    BT::BehaviorTreeFactory factory_;
    std::filesystem::path reason_file_;
    std::filesystem::path hosts_file_;
};

}  // namespace

TEST_F(ShutdownTreeTest, SafetyTreeBuildsFromProject)
{
    auto blackboard = createBlackboard("true", "unused");
    EXPECT_NO_THROW({ auto tree = factory_.createTree("RoverSafety", blackboard); });
}

TEST_F(ShutdownTreeTest, TripsEStopThenPowersOff)
{
    const std::string reason = "Battery temperature 61.0 C above fatal 60.0 C; it's hot";
    auto tree = factory_.createTree(
        "RoverShutdown",
        createBlackboard("printf '%s' \"$ROVER_SHUTDOWN_REASON\" > '" + reason_file_.string() + "'", reason));

    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(e_stop_calls_.load(), 1);
    EXPECT_EQ(readReasonFile(), reason);
}

TEST_F(ShutdownTreeTest, FailedPowerOffFailsTheTree)
{
    auto tree = factory_.createTree("RoverShutdown", createBlackboard("exit 3", "test"));

    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
    EXPECT_EQ(e_stop_calls_.load(), 1);
}

TEST_F(ShutdownTreeTest, RetryRunsTheWholeSequenceAgain)
{
    auto blackboard = createBlackboard("exit 3", "first");
    auto tree = factory_.createTree("RoverShutdown", blackboard);
    ASSERT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);

    tree.haltTree();
    blackboard->set<std::string>(
        "SHUTDOWN_LOCALHOST_COMMAND",
        rover_safety::infrastructure::buildShutdownCommand(
            "printf '%s' \"$ROVER_SHUTDOWN_REASON\" > '" + reason_file_.string() + "'", "second"));

    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(e_stop_calls_.load(), 2);
    EXPECT_EQ(readReasonFile(), "second");
}

// A remote host takes the shutdown tree through ShutdownHostsFromFile's ping. The ping to a dead
// host lasts a full second and must run beside the tick, never inside it: rover_safety_node ticks
// from the same executor thread that serves the E-Stop and battery callbacks.
TEST_F(ShutdownTreeTest, SkipsUnreachableHostWithoutBlockingTheTick)
{
    {
        std::ofstream hosts(hosts_file_);
        hosts << "hosts:\n  - ip: " << rover_safety::test::kUnreachableIp << "\n    timeout: 1.0\n";
    }
    auto blackboard = createBlackboard(
        "printf '%s' \"$ROVER_SHUTDOWN_REASON\" > '" + reason_file_.string() + "'", "remote");
    blackboard->set<std::string>("SHUTDOWN_HOSTS_FILE", hosts_file_.string());
    auto tree = factory_.createTree("RoverShutdown", blackboard);

    auto status = BT::NodeStatus::RUNNING;
    auto slowest = std::chrono::steady_clock::duration::zero();
    for (int i = 0; i < 200 && status == BT::NodeStatus::RUNNING; ++i) {
        const auto before = std::chrono::steady_clock::now();
        status = tree.tickOnce();
        slowest = std::max(slowest, std::chrono::steady_clock::now() - before);
        tree.sleep(100ms);
    }

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_EQ(readReasonFile(), "remote");
    EXPECT_LT(slowest, 200ms);
}
