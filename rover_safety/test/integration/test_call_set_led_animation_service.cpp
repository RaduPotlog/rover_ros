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
#include <rover_msgs/srv/set_led_animation.hpp>

#include "rover_safety/plugins/action/call_set_led_animation_service_node.hpp"

using namespace std::chrono_literals;
using SetLedAnimationSrv = rover_msgs::srv::SetLedAnimation;

namespace
{

constexpr char kServiceName[] = "test_set_animation";

class CallSetLedAnimationServiceTest : public ::testing::Test
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

        factory_.registerNodeType<rover_safety::CallSetLedAnimationService>("CallSetLedAnimationService");
    }

    void TearDown() override
    {
        executor_.cancel();
        spin_thread_.join();
    }

    void startServer(bool success, std::chrono::milliseconds delay = 0ms)
    {
        service_ = server_node_->create_service<SetLedAnimationSrv>(
            kServiceName,
            [this, success, delay](
                const SetLedAnimationSrv::Request::SharedPtr request,
                SetLedAnimationSrv::Response::SharedPtr response) {
                std::this_thread::sleep_for(delay);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    received_ = *request;
                }
                ++calls_;
                response->success = success;
                response->message = success ? "" : "refused";
            });
    }

    /** Ticks the tree the way LedSafetyNode does (10 Hz) until it leaves RUNNING. */
    BT::NodeStatus tickUntilDone(
        const std::string & id = "7", const std::string & param = "0.5",
        const std::string & repeating = "true")
    {
        auto blackboard = BT::Blackboard::create();
        blackboard->set<nav2::LifecycleNode::SharedPtr>("node", bt_node_);
        blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
        blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 2000ms);
        blackboard->set<std::chrono::milliseconds>("server_timeout", 300ms);

        auto tree = factory_.createTreeFromText(
            std::string(R"(<root BTCPP_format="4"><BehaviorTree ID="T">)") +
            R"(<CallSetLedAnimationService service_name=")" + kServiceName + R"(" id=")" + id +
            R"(" param=")" + param + R"(" repeating=")" + repeating + R"("/>)" +
            R"(</BehaviorTree></root>)",
            blackboard);

        auto status = BT::NodeStatus::RUNNING;
        for (int i = 0; i < 50 && status == BT::NodeStatus::RUNNING; ++i) {
            status = tree.tickOnce();
            std::this_thread::sleep_for(100ms);
        }
        return status;
    }

    std::optional<SetLedAnimationSrv::Request> received()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return received_;
    }

    nav2::LifecycleNode::SharedPtr bt_node_;
    rclcpp::Node::SharedPtr server_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
    rclcpp::Service<SetLedAnimationSrv>::SharedPtr service_;
    std::atomic<int> calls_{0};
    std::mutex mutex_;
    std::optional<SetLedAnimationSrv::Request> received_;
    BT::BehaviorTreeFactory factory_;
};

}  // namespace

TEST_F(CallSetLedAnimationServiceTest, SucceedsWhenServerConfirms)
{
    startServer(true);
    EXPECT_EQ(tickUntilDone(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(calls_.load(), 1);
}

TEST_F(CallSetLedAnimationServiceTest, SendsThePorts)
{
    startServer(true);
    ASSERT_EQ(tickUntilDone("7", "0.5", "true"), BT::NodeStatus::SUCCESS);

    const auto request = received();
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->animation.id, 7u);
    EXPECT_EQ(request->animation.param, "0.5");
    EXPECT_TRUE(request->repeating);
}

// The old node fired the request and returned SUCCESS without looking at the answer, so the LED
// tree recorded animations the LED server had refused as shown.
TEST_F(CallSetLedAnimationServiceTest, FailsWhenServerRefuses)
{
    startServer(false);
    EXPECT_EQ(tickUntilDone(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(calls_.load(), 1);
}

TEST_F(CallSetLedAnimationServiceTest, FailsWhenResponseExceedsServerTimeout)
{
    startServer(true, 1000ms);
    EXPECT_EQ(tickUntilDone(), BT::NodeStatus::FAILURE);
}
