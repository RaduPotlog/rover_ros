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

#include <stdexcept>

#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

TEST(CheckIfJointNameContainValidSequenceTest, ExactMatch)
{
    EXPECT_TRUE(checkIfJointNameContainValidSequence("fl", "fl"));
}

TEST(CheckIfJointNameContainValidSequenceTest, SequenceAsUnderscoreDelimitedPrefix)
{
    EXPECT_TRUE(checkIfJointNameContainValidSequence("fl_wheel_joint", "fl"));
}

TEST(CheckIfJointNameContainValidSequenceTest, DifferentSequenceDoesNotMatch)
{
    EXPECT_FALSE(checkIfJointNameContainValidSequence("fr_wheel_joint", "fl"));
}

TEST(CheckIfJointNameContainValidSequenceTest, SequenceMustBeAPrefixNotJustContained)
{
    // "fl" is a substring of "left_fl_wheel" but not its leading token - must not match.
    EXPECT_FALSE(checkIfJointNameContainValidSequence("left_fl_wheel", "fl"));
}

TEST(CheckIfJointNameContainValidSequenceTest, NamespacePrefixIsSkippedBeforeMatching)
{
    EXPECT_TRUE(checkIfJointNameContainValidSequence("robot1/fl_wheel_joint", "fl"));
}

TEST(CheckIfJointNameContainValidSequenceTest, NamespaceSegmentEqualToSequenceIsNotAFalseMatch)
{
    // Regression case: a namespace segment that happens to equal `sequence` (here "fr") must not
    // be confused with the real local-name prefix ("fl_..."), unlike a plain
    // contains-as-delimited-substring scan would.
    EXPECT_FALSE(checkIfJointNameContainValidSequence("my_fr_robot/fl_wheel_joint", "fr"));
}

class OperationWithAttemptsTest : public ::testing::Test
{
protected:
    unsigned operation_calls_ = 0;
    unsigned on_error_calls_ = 0;
};

TEST_F(OperationWithAttemptsTest, SucceedsOnFirstAttemptWithoutRetrying)
{
    const bool result = operationWithAttempts(
        [this]() { ++operation_calls_; }, 3, [this]() { ++on_error_calls_; });

    EXPECT_TRUE(result);
    EXPECT_EQ(operation_calls_, 1U);
    EXPECT_EQ(on_error_calls_, 0U);
}

TEST_F(OperationWithAttemptsTest, RetriesUpToMaxAttemptsThenFails)
{
    const bool result = operationWithAttempts(
        [this]() {
            ++operation_calls_;
            throw std::runtime_error("always fails");
        },
        3, [this]() { ++on_error_calls_; });

    EXPECT_FALSE(result);
    EXPECT_EQ(operation_calls_, 3U);
    EXPECT_EQ(on_error_calls_, 3U);
}

TEST_F(OperationWithAttemptsTest, SucceedsAfterASubsetOfFailures)
{
    const bool result = operationWithAttempts(
        [this]() {
            ++operation_calls_;
            if (operation_calls_ < 2) {
                throw std::runtime_error("fails once");
            }
        },
        3, [this]() { ++on_error_calls_; });

    EXPECT_TRUE(result);
    EXPECT_EQ(operation_calls_, 2U);
    EXPECT_EQ(on_error_calls_, 1U);
}

TEST_F(OperationWithAttemptsTest, OnErrorThrowingStopsRetriesEarly)
{
    const bool result = operationWithAttempts(
        [this]() {
            ++operation_calls_;
            throw std::runtime_error("always fails");
        },
        5,
        [this]() {
            ++on_error_calls_;
            throw std::runtime_error("on_error also fails");
        });

    EXPECT_FALSE(result);
    // Stops after the first on_error() failure rather than exhausting all 5 attempts.
    EXPECT_EQ(operation_calls_, 1U);
    EXPECT_EQ(on_error_calls_, 1U);
}

}  // namespace rover_hardware_interface
