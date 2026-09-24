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

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "rover_safety/plugins/action/call_set_bool_service_node.hpp"

using namespace std::chrono_literals;
using SetBoolSrv = std_srvs::srv::SetBool;

namespace
{

constexpr char kServiceName[] = "test_set_bool";

class CallSetBoolServiceTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        bt_node_ = std::make_shared<nav2::LifecycleNode>("bt_node");
        server_node_ = std::make_shared<rclcpp::Node>("server_node");
        executor_.add_node(server_node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });

        factory_.registerNodeType<rover_safety::CallSetBoolService>("CallSetBoolService");
    }

    void TearDown() override
    {
        executor_.cancel();
        spin_thread_.join();
    }

    void startServer(bool success, std::chrono::milliseconds delay = 0ms)
    {
        service_ = server_node_->create_service<SetBoolSrv>(
            kServiceName,
            [this, success, delay](
                const SetBoolSrv::Request::SharedPtr request, SetBoolSrv::Response::SharedPtr response) {
                std::this_thread::sleep_for(delay);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    received_data_ = request->data;
                }
                ++calls_;
                response->success = success;
                response->message = success ? "" : "refused";
            });
    }

    /** Ticks the tree the way SafetyNode does (fixed period) until it leaves RUNNING. */
    BT::NodeStatus tickUntilDone(const std::string & data)
    {
        auto blackboard = BT::Blackboard::create();
        blackboard->set<nav2::LifecycleNode::SharedPtr>("node", bt_node_);
        blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
        blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 2000ms);
        blackboard->set<std::chrono::milliseconds>("server_timeout", 300ms);

        auto tree = factory_.createTreeFromText(
            std::string(R"(<root BTCPP_format="4"><BehaviorTree ID="T">)") +
            R"(<CallSetBoolService service_name=")" + kServiceName + R"(" data=")" + data + R"("/>)" +
            R"(</BehaviorTree></root>)",
            blackboard);

        auto status = BT::NodeStatus::RUNNING;
        for (int i = 0; i < 50 && status == BT::NodeStatus::RUNNING; ++i) {
            status = tree.tickOnce();
            std::this_thread::sleep_for(100ms);  // SafetyNode ticks at 10 Hz
        }
        return status;
    }

    std::optional<bool> receivedData()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return received_data_;
    }

    nav2::LifecycleNode::SharedPtr bt_node_;
    rclcpp::Node::SharedPtr server_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
    rclcpp::Service<SetBoolSrv>::SharedPtr service_;
    std::atomic<int> calls_{0};
    std::mutex mutex_;
    std::optional<bool> received_data_;
    BT::BehaviorTreeFactory factory_;
};

}  // namespace

TEST_F(CallSetBoolServiceTest, SucceedsWhenServerConfirms)
{
    startServer(true);
    EXPECT_EQ(tickUntilDone("true"), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(calls_.load(), 1);
}

TEST_F(CallSetBoolServiceTest, SendsTheDataPort)
{
    startServer(true);
    ASSERT_EQ(tickUntilDone("false"), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(receivedData(), std::optional<bool>(false));
}

TEST_F(CallSetBoolServiceTest, FailsWhenServerRefuses)
{
    startServer(false);
    EXPECT_EQ(tickUntilDone("true"), BT::NodeStatus::FAILURE);
    EXPECT_EQ(calls_.load(), 1);
}

TEST_F(CallSetBoolServiceTest, FailsWhenResponseExceedsServerTimeout)
{
    startServer(true, 1000ms);
    EXPECT_EQ(tickUntilDone("true"), BT::NodeStatus::FAILURE);
}
