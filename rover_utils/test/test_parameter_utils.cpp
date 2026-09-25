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

#include <gtest/gtest.h>

#include "rcl_interfaces/msg/parameter_type.hpp"

#include "rover_utils/parameter_utils.hpp"

// Defined in parameter_utils_second_tu.cpp.
rcl_interfaces::msg::ParameterDescriptor describeFromSecondTranslationUnit();

namespace
{

using rcl_interfaces::msg::ParameterType;

TEST(ParameterUtilsTest, DescribeIsReadOnlyWithTheDescription)
{
    const auto descriptor = rover_utils::ros::describe("Design capacity of the pack [Ah].");

    EXPECT_EQ(descriptor.description, "Design capacity of the pack [Ah].");
    EXPECT_TRUE(descriptor.read_only);
    EXPECT_TRUE(descriptor.floating_point_range.empty());
    EXPECT_TRUE(descriptor.integer_range.empty());
    EXPECT_TRUE(descriptor.name.empty());
    EXPECT_EQ(descriptor.type, ParameterType::PARAMETER_NOT_SET);
    EXPECT_FALSE(descriptor.dynamic_typing);
    EXPECT_TRUE(descriptor.additional_constraints.empty());
}

TEST(ParameterUtilsTest, DescribePositiveAddsOneRangeFromOneMicro)
{
    const auto descriptor = rover_utils::ros::describePositive(
        "Maximum yaw change within one segment [rad].", 3.2);

    EXPECT_EQ(descriptor.description, "Maximum yaw change within one segment [rad].");
    EXPECT_TRUE(descriptor.read_only);
    ASSERT_EQ(descriptor.floating_point_range.size(), 1u);
    EXPECT_DOUBLE_EQ(descriptor.floating_point_range[0].from_value, 1.0e-6);
    EXPECT_DOUBLE_EQ(descriptor.floating_point_range[0].to_value, 3.2);
    EXPECT_DOUBLE_EQ(descriptor.floating_point_range[0].step, 0.0);
    EXPECT_TRUE(descriptor.integer_range.empty());
}

TEST(ParameterUtilsTest, IncludedFromTwoTranslationUnits)
{
    const auto descriptor = describeFromSecondTranslationUnit();

    EXPECT_TRUE(descriptor.read_only);
    ASSERT_EQ(descriptor.floating_point_range.size(), 1u);
    EXPECT_DOUBLE_EQ(descriptor.floating_point_range[0].to_value, 2.0);
}

}  // namespace
