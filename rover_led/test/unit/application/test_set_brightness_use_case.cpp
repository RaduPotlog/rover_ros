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

#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "rover_led/application/set_brightness_use_case.hpp"

using rover_led::SK9822FrameEncoder;

TEST(SetBrightnessUseCase, SetsEveryChannel)
{
    auto first = std::make_shared<SK9822FrameEncoder>();
    auto second = std::make_shared<SK9822FrameEncoder>();
    rover_led::SetBrightnessUseCase use_case({first, second});

    use_case.execute(0.5f);

    EXPECT_EQ(first->getGlobalBrightness(), 16);
    EXPECT_EQ(second->getGlobalBrightness(), 16);
}

TEST(SetBrightnessUseCase, OutOfRangeLeavesEveryChannelUnchanged)
{
    auto first = std::make_shared<SK9822FrameEncoder>();
    auto second = std::make_shared<SK9822FrameEncoder>();
    rover_led::SetBrightnessUseCase use_case({first, second});

    EXPECT_THROW(use_case.execute(1.5f), std::out_of_range);
    EXPECT_THROW(use_case.execute(-0.5f), std::out_of_range);

    EXPECT_EQ(first->getGlobalBrightness(), 31);
    EXPECT_EQ(second->getGlobalBrightness(), 31);
}
