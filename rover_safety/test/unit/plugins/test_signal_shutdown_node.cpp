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

#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"

#include "rover_safety/plugins/action/signal_shutdown_node.hpp"

#include "plugin_test_utils.hpp"

using rover_safety::test::singleNodeTree;

TEST(SignalShutdownTest, WrongNodeNameThrows)
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<rover_safety::SignalShutdown>("SignalShutdown");

    EXPECT_THROW(
        { auto tree = factory.createTreeFromText(singleNodeTree("WrongSignalShutdown", {})); },
        BT::RuntimeError);
}

TEST(SignalShutdownTest, WritesReasonToBlackboard)
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<rover_safety::SignalShutdown>("SignalShutdown");

    auto tree = factory.createTreeFromText(singleNodeTree("SignalShutdown", {{"reason", "Test shutdown."}}));

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

    const auto signal = tree.rootBlackboard()->get<std::pair<bool, std::string>>("signal_shutdown");
    EXPECT_TRUE(signal.first);
    EXPECT_EQ(signal.second, "Test shutdown.");
}
