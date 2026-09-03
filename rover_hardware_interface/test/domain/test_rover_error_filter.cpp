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

#include "rover_hardware_interface/domain/rover_error_filter.hpp"

namespace rover_hardware_interface
{

TEST(ErrorFilterTest, StartsClear)
{
    ErrorFilter filter(3);
    EXPECT_FALSE(filter.isError());
}

TEST(ErrorFilterTest, LatchesOnlyAfterMaxConsecutiveErrors)
{
    ErrorFilter filter(3);

    filter.updateError(true);
    EXPECT_FALSE(filter.isError());

    filter.updateError(true);
    EXPECT_FALSE(filter.isError());

    filter.updateError(true);
    EXPECT_TRUE(filter.isError());
}

TEST(ErrorFilterTest, FalseUpdateResetsConsecutiveCountBeforeLatch)
{
    ErrorFilter filter(3);

    filter.updateError(true);
    filter.updateError(true);
    filter.updateError(false);
    filter.updateError(true);
    filter.updateError(true);

    EXPECT_FALSE(filter.isError());
}

TEST(ErrorFilterTest, FalseUpdateDoesNotUnlatchOnceErrorIsSet)
{
    ErrorFilter filter(1);

    filter.updateError(true);
    ASSERT_TRUE(filter.isError());

    filter.updateError(false);

    EXPECT_TRUE(filter.isError());
}

TEST(ErrorFilterTest, ClearErrorResetsLatchAndCount)
{
    ErrorFilter filter(1);

    filter.updateError(true);
    ASSERT_TRUE(filter.isError());

    filter.clearError();
    EXPECT_FALSE(filter.isError());

    // Confirms the consecutive-count was reset too, not just the latch.
    filter.updateError(true);
    EXPECT_TRUE(filter.isError());
}

class RoverErrorFilterTest : public ::testing::Test
{
protected:
    RoverErrorFilter filter_{1, 1, 1, 1};
};

TEST_F(RoverErrorFilterTest, StartsClear)
{
    EXPECT_FALSE(filter_.isError());

    for (const auto & [id, name] : kErrorFilterIdNames) {
        EXPECT_FALSE(filter_.isError(id)) << name;
    }
}

TEST_F(RoverErrorFilterTest, UpdateErrorSetsOnlyItsOwnCategory)
{
    filter_.updateError(ErrorsFilterIds::WRITE_CMDS, true);

    EXPECT_TRUE(filter_.isError());
    EXPECT_TRUE(filter_.isError(ErrorsFilterIds::WRITE_CMDS));
    EXPECT_FALSE(filter_.isError(ErrorsFilterIds::READ_MOTOR_STATES));
    EXPECT_FALSE(filter_.isError(ErrorsFilterIds::READ_DRIVER_STATE));
    EXPECT_FALSE(filter_.isError(ErrorsFilterIds::FAULT_FLAG));
}

TEST_F(RoverErrorFilterTest, GetErrorMapUsesFriendlyNamesAndReflectsState)
{
    filter_.updateError(ErrorsFilterIds::FAULT_FLAG, true);

    const auto error_map = filter_.getErrorMap();

    EXPECT_TRUE(error_map.at("fault_flag_error"));
    EXPECT_FALSE(error_map.at("write_cmds_error"));
    EXPECT_FALSE(error_map.at("read_motor_states_error"));
    EXPECT_FALSE(error_map.at("read_driver_state_error"));
}

TEST_F(RoverErrorFilterTest, SetClearErrorsFlagClearsAllCategoriesOnNextUpdate)
{
    filter_.updateError(ErrorsFilterIds::WRITE_CMDS, true);
    filter_.updateError(ErrorsFilterIds::FAULT_FLAG, true);
    ASSERT_TRUE(filter_.isError());

    filter_.setClearErrorsFlag();
    // The clear is only applied lazily, on the next updateError() call - here for an unrelated
    // category, to confirm the clear isn't scoped to just the category being updated.
    filter_.updateError(ErrorsFilterIds::READ_MOTOR_STATES, false);

    EXPECT_FALSE(filter_.isError());
}

}  // namespace rover_hardware_interface
