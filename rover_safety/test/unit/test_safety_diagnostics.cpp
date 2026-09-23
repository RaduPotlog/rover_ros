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
#include <string>

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
        true, {{"battery", 10.0, 5.0}, {"safety_status", std::nullopt, std::nullopt}}, status);

    EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
    EXPECT_EQ(status.values.size(), 2u);
}

TEST(SafetyDiagnostics, UnsubscribedInputsSayNotSubscribedInsteadOfWaiting)
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    infrastructure::fillSafetyInputsStatus(
        false, {{"battery", std::nullopt, 5.0}, {"safety_status", std::nullopt, 5.0}}, status);

    EXPECT_EQ(status.level, DiagnosticStatus::WARN);
    EXPECT_NE(status.message.find("not subscribed"), std::string::npos) << status.message;
    EXPECT_TRUE(status.values.empty());
}

TEST(SafetyDiagnostics, BehaviorTreeLevels)
{
    diagnostic_updater::DiagnosticStatusWrapper unconfigured;
    infrastructure::fillBehaviorTreeStatus(false, false, BT::NodeStatus::IDLE, 0, "", unconfigured);
    EXPECT_EQ(unconfigured.level, DiagnosticStatus::WARN);

    diagnostic_updater::DiagnosticStatusWrapper waiting;
    infrastructure::fillBehaviorTreeStatus(true, false, BT::NodeStatus::IDLE, 0, "", waiting);
    EXPECT_EQ(waiting.level, DiagnosticStatus::WARN);

    diagnostic_updater::DiagnosticStatusWrapper failed;
    infrastructure::fillBehaviorTreeStatus(true, true, BT::NodeStatus::FAILURE, 0, "", failed);
    EXPECT_EQ(failed.level, DiagnosticStatus::WARN);

    diagnostic_updater::DiagnosticStatusWrapper running;
    infrastructure::fillBehaviorTreeStatus(true, true, BT::NodeStatus::RUNNING, 0, "", running);
    EXPECT_EQ(running.level, DiagnosticStatus::OK);
}

TEST(SafetyDiagnostics, FailedConfigureIsErrorWithTheCause)
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    infrastructure::fillBehaviorTreeStatus(
        false, false, BT::NodeStatus::IDLE, 3, "Service server led/set_animation not available",
        status);

    EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
    EXPECT_NE(status.message.find("led/set_animation"), std::string::npos) << status.message;

    bool has_error_value = false;
    for (const auto & value : status.values) {
        if (value.key == "Last configure error") {
            has_error_value = true;
        }
        if (value.key == "Failed configure attempts") {
            EXPECT_EQ(value.value, "3");
        }
    }
    EXPECT_TRUE(has_error_value);
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
