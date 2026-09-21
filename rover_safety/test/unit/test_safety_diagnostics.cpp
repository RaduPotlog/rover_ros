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
#include <optional>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "rover_safety/infrastructure/safety_diagnostics.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using namespace rover_safety;  // NOLINT

TEST(SafetyDiagnostics, AgeSecondsIsNulloptBeforeFirstStamp)
{
    EXPECT_FALSE(infrastructure::ageSeconds(std::nullopt, std::chrono::steady_clock::now()));
}

TEST(SafetyDiagnostics, AgeSecondsMeasuresElapsedTime)
{
    const infrastructure::SteadyTime t0{};
    const auto age = infrastructure::ageSeconds(t0, t0 + std::chrono::milliseconds(1500));

    ASSERT_TRUE(age.has_value());
    EXPECT_DOUBLE_EQ(*age, 1.5);
}

TEST(SafetyDiagnostics, StaleInputsAreErrorWithOneValuePerInput)
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    infrastructure::fillSafetyInputsStatus(
        {{"battery", 10.0, 5.0}, {"safety_status", std::nullopt, std::nullopt}}, status);

    EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
    EXPECT_EQ(status.values.size(), 2u);
}

TEST(SafetyDiagnostics, BehaviorTreeLevels)
{
    diagnostic_updater::DiagnosticStatusWrapper unconfigured;
    infrastructure::fillBehaviorTreeStatus(false, false, BT::NodeStatus::IDLE, unconfigured);
    EXPECT_EQ(unconfigured.level, DiagnosticStatus::WARN);

    diagnostic_updater::DiagnosticStatusWrapper waiting;
    infrastructure::fillBehaviorTreeStatus(true, false, BT::NodeStatus::IDLE, waiting);
    EXPECT_EQ(waiting.level, DiagnosticStatus::WARN);

    diagnostic_updater::DiagnosticStatusWrapper failed;
    infrastructure::fillBehaviorTreeStatus(true, true, BT::NodeStatus::FAILURE, failed);
    EXPECT_EQ(failed.level, DiagnosticStatus::WARN);

    diagnostic_updater::DiagnosticStatusWrapper running;
    infrastructure::fillBehaviorTreeStatus(true, true, BT::NodeStatus::RUNNING, running);
    EXPECT_EQ(running.level, DiagnosticStatus::OK);
}

TEST(SafetyDiagnostics, ShutdownLevels)
{
    using rover_safety::domain::ShutdownSequence;
    const ShutdownSequence::SteadyTime t0{};

    ShutdownSequence sequence(std::chrono::seconds(30));

    diagnostic_updater::DiagnosticStatusWrapper idle;
    infrastructure::fillShutdownStatus(sequence, idle);
    EXPECT_EQ(idle.level, DiagnosticStatus::OK);

    sequence.request("battery fatal", t0);
    diagnostic_updater::DiagnosticStatusWrapper in_progress;
    infrastructure::fillShutdownStatus(sequence, in_progress);
    EXPECT_EQ(in_progress.level, DiagnosticStatus::WARN);
    EXPECT_NE(in_progress.message.find("battery fatal"), std::string::npos);

    sequence.finish(false, "command failed", t0);
    diagnostic_updater::DiagnosticStatusWrapper failed;
    infrastructure::fillShutdownStatus(sequence, failed);
    EXPECT_EQ(failed.level, DiagnosticStatus::ERROR);
    EXPECT_NE(failed.message.find("command failed"), std::string::npos);
}
