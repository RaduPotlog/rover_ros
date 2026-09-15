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
#include <stdexcept>

#include "rover_safety/domain/shutdown_sequence.hpp"

using namespace rover_safety::domain;  // NOLINT
using namespace std::chrono_literals;

namespace
{

const ShutdownSequence::SteadyTime kT0{};

}  // namespace

TEST(ShutdownSequence, StartsIdleAndOk)
{
    const ShutdownSequence sequence(30s);

    EXPECT_EQ(sequence.state(), ShutdownState::Idle);
    EXPECT_EQ(sequence.healthLevel(), HealthLevel::Ok);
    EXPECT_EQ(sequence.attempts(), 0u);
    EXPECT_TRUE(sequence.reason().empty());
}

TEST(ShutdownSequence, RejectsNegativeBackoff)
{
    EXPECT_THROW(ShutdownSequence(std::chrono::duration<double>(-1.0)), std::invalid_argument);
}

TEST(ShutdownSequence, FirstRequestStarts)
{
    ShutdownSequence sequence(30s);

    EXPECT_EQ(sequence.request("battery fatal", kT0), ShutdownRequestResult::Started);
    EXPECT_TRUE(sequence.inProgress());
    EXPECT_EQ(sequence.reason(), "battery fatal");
    EXPECT_EQ(sequence.attempts(), 1u);
    EXPECT_EQ(sequence.healthLevel(), HealthLevel::Warn);
}

TEST(ShutdownSequence, RequestWhileInProgressIsIdempotent)
{
    ShutdownSequence sequence(30s);
    sequence.request("first", kT0);

    EXPECT_EQ(sequence.request("second", kT0 + 1s), ShutdownRequestResult::AlreadyInProgress);
    EXPECT_EQ(sequence.reason(), "first");
    EXPECT_EQ(sequence.attempts(), 1u);
}

TEST(ShutdownSequence, SucceededIsTerminal)
{
    ShutdownSequence sequence(0s);
    sequence.request("first", kT0);
    sequence.finish(true, "", kT0 + 1s);

    EXPECT_EQ(sequence.state(), ShutdownState::Succeeded);
    EXPECT_EQ(sequence.healthLevel(), HealthLevel::Warn);
    EXPECT_EQ(sequence.request("again", kT0 + 1h), ShutdownRequestResult::AlreadySucceeded);
    EXPECT_EQ(sequence.attempts(), 1u);
}

TEST(ShutdownSequence, FailureIsErrorWithDetail)
{
    ShutdownSequence sequence(30s);
    sequence.request("first", kT0);
    sequence.finish(false, "dbus-send failed", kT0 + 1s);

    EXPECT_EQ(sequence.state(), ShutdownState::Failed);
    EXPECT_EQ(sequence.healthLevel(), HealthLevel::Error);
    EXPECT_EQ(sequence.detail(), "dbus-send failed");
    EXPECT_EQ(sequence.reason(), "first");
}

TEST(ShutdownSequence, RetryAfterFailureWaitsForBackoff)
{
    ShutdownSequence sequence(30s);
    sequence.request("first", kT0);
    sequence.finish(false, "failed", kT0 + 1s);

    EXPECT_EQ(sequence.request("retry", kT0 + 20s), ShutdownRequestResult::RetryBackoff);
    EXPECT_EQ(sequence.state(), ShutdownState::Failed);

    EXPECT_EQ(sequence.request("retry", kT0 + 31s), ShutdownRequestResult::Started);
    EXPECT_TRUE(sequence.inProgress());
    EXPECT_EQ(sequence.reason(), "retry");
    EXPECT_TRUE(sequence.detail().empty());
    EXPECT_EQ(sequence.attempts(), 2u);
}

TEST(ShutdownSequence, FinishIgnoredUnlessInProgress)
{
    ShutdownSequence sequence(30s);
    sequence.finish(true, "", kT0);
    EXPECT_EQ(sequence.state(), ShutdownState::Idle);

    sequence.request("first", kT0);
    sequence.finish(true, "", kT0 + 1s);
    sequence.finish(false, "late failure", kT0 + 2s);
    EXPECT_EQ(sequence.state(), ShutdownState::Succeeded);
}
