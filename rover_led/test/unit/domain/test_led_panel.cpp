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

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "rover_led/domain/led_components/led_panel.hpp"

TEST(LedPanel, StartsBlank)
{
    rover_led::LedPanel panel(3);

    EXPECT_EQ(panel.getNumberOfLeds(), 3u);
    EXPECT_EQ(panel.getFrame(), std::vector<std::uint8_t>(12, 0));
}

TEST(LedPanel, WritesValuesAtTheGivenByteOffset)
{
    rover_led::LedPanel panel(3);
    panel.updateFrame(4, {1, 2, 3, 4});

    EXPECT_EQ(panel.getFrame(), (std::vector<std::uint8_t>{0, 0, 0, 0, 1, 2, 3, 4, 0, 0, 0, 0}));
}

TEST(LedPanel, RejectsValuesThatDoNotFit)
{
    rover_led::LedPanel panel(2);

    EXPECT_THROW(panel.updateFrame(0, {}), std::runtime_error);
    EXPECT_THROW(panel.updateFrame(0, std::vector<std::uint8_t>(12, 1)), std::runtime_error);
    EXPECT_THROW(panel.updateFrame(4, std::vector<std::uint8_t>(8, 1)), std::runtime_error);
    EXPECT_NO_THROW(panel.updateFrame(4, std::vector<std::uint8_t>(4, 1)));
}
