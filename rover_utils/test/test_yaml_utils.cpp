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

#include <stdexcept>

#include "rover_utils/yaml_utils.hpp"

namespace
{

const YAML::Node kDescription = YAML::Load("period: 2.5\nname: blink\n");

TEST(YamlUtilsTest, ReadsAPresentKey)
{
    EXPECT_DOUBLE_EQ(rover_utils::getYAMLKeyValue<double>(kDescription, "period"), 2.5);
}

TEST(YamlUtilsTest, MissingKeyThrowsMissingYAMLKeyError)
{
    EXPECT_THROW(
        rover_utils::getYAMLKeyValue<double>(kDescription, "duration"),
        rover_utils::MissingYAMLKeyError);
}

TEST(YamlUtilsTest, MissingKeyWithDefaultReturnsTheDefault)
{
    EXPECT_DOUBLE_EQ(rover_utils::getYAMLKeyValue<double>(kDescription, "duration", 1.0), 1.0);
}

TEST(YamlUtilsTest, BadConversionStillThrowsWithADefault)
{
    try {
        rover_utils::getYAMLKeyValue<double>(kDescription, "name", 1.0);
        FAIL() << "expected a conversion error";
    } catch (const rover_utils::MissingYAMLKeyError &) {
        FAIL() << "a present key must not be reported as missing";
    } catch (const std::runtime_error & e) {
        EXPECT_STREQ(e.what(), "Failed to convert 'name' key.");
    }
}

}  // namespace
