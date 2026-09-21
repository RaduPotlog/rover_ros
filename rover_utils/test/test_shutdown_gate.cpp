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

#include <memory>

#include <gtest/gtest.h>

#include "rclcpp/context.hpp"

#include "rover_utils/shutdown_gate.hpp"

using rover_utils::ros::ShutdownGate;

namespace
{

std::shared_ptr<rclcpp::Context> makeContext()
{
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    return context;
}

}  // namespace

TEST(ShutdownGateTest, OpenUntilShutdown)
{
    auto context = makeContext();
    ShutdownGate gate(context);

    EXPECT_TRUE(gate.isOpen());

    context->shutdown("test");

    EXPECT_FALSE(gate.isOpen());
}

TEST(ShutdownGateTest, OnCloseRunsWhileStillOpen)
{
    auto context = makeContext();
    bool ran = false;
    bool open_during_close = false;
    std::unique_ptr<ShutdownGate> gate;

    gate = std::make_unique<ShutdownGate>(context, [&]() {
        ran = true;
        open_during_close = gate->isOpen();
    });

    context->shutdown("test");

    EXPECT_TRUE(ran);
    EXPECT_TRUE(open_during_close);
    EXPECT_FALSE(gate->isOpen());
}

TEST(ShutdownGateTest, DestructorUnregistersCallback)
{
    auto context = makeContext();
    bool ran = false;

    {
        ShutdownGate gate(context, [&]() { ran = true; });
    }

    context->shutdown("test");

    EXPECT_FALSE(ran);
}
