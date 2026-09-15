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
#include <cstdlib>
#include <map>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "rover_safety/plugins/action/execute_command_node.hpp"

#include "plugin_test_utils.hpp"

using rover_safety::test::singleNodeTree;
using rover_safety::test::tickUntilDone;

class ExecuteCommandTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        factory_.registerNodeType<rover_safety::ExecuteCommand>("ExecuteCommand");
    }

    BT::Tree createTree(const std::map<std::string, std::string> & ports)
    {
        return factory_.createTreeFromText(singleNodeTree("ExecuteCommand", ports));
    }

    BT::BehaviorTreeFactory factory_;
};

TEST_F(ExecuteCommandTest, WrongNodeNameThrows)
{
    EXPECT_THROW(
        { auto tree = factory_.createTreeFromText(singleNodeTree("WrongExecuteCommand", {})); },
        BT::RuntimeError);
}

TEST_F(ExecuteCommandTest, MissingCommandPortFails)
{
    auto tree = createTree({{"timeout", "1.0"}});
    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
}

TEST_F(ExecuteCommandTest, MissingTimeoutPortFails)
{
    auto tree = createTree({{"command", "echo test"}});
    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
}

TEST_F(ExecuteCommandTest, SimpleCommandSucceeds)
{
    auto tree = createTree({{"command", "echo 'Test command' &amp;&amp; echo 'Test command 2'"}, {"timeout", "1.0"}});
    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
}

TEST_F(ExecuteCommandTest, UnknownCommandFails)
{
    auto tree = createTree({{"command", "command_with_rather_impossible_name"}, {"timeout", "1.0"}});
    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
}

TEST_F(ExecuteCommandTest, NonZeroExitCodeFails)
{
    auto tree = createTree({{"command", "echo 'Test command' &amp;&amp; exit 3"}, {"timeout", "1.0"}});
    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
}

TEST_F(ExecuteCommandTest, TimeoutKillsTheWholeCommand)
{
    // A compound command makes bash fork `sleep` instead of exec'ing it.
    auto tree = createTree({{"command", "sleep 31.4159; true"}, {"timeout", "0.2"}});

    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(5));

    // The bracket keeps pgrep from matching the shell running it.
    EXPECT_NE(std::system("pgrep -f '[s]leep 31.4159' > /dev/null"), 0);
}

TEST_F(ExecuteCommandTest, HaltStopsARunningCommand)
{
    auto tree = createTree({{"command", "sleep 27.1828; true"}, {"timeout", "10.0"}});

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_NO_THROW(tree.haltTree());
    EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
    EXPECT_NE(std::system("pgrep -f '[s]leep 27.1828' > /dev/null"), 0);
}

TEST_F(ExecuteCommandTest, CommandCanRunAgain)
{
    // A retried shutdown ticks the same node again.
    auto tree = createTree({{"command", "true"}, {"timeout", "1.0"}});

    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
    tree.haltTree();
    EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
}
