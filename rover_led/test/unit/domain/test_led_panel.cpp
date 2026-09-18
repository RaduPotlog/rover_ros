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

TEST(LedPanel, FoldsLogicalOrderIntoSerpentineRows)
{
    // 6 LEDs in 2 rows: row 0 is physical 0-2, row 1 is 3-5 running back, so
    // column k (from LED 0's end) holds physical k and 5-k.
    rover_led::LedPanel panel(6, 2);

    std::vector<std::uint8_t> logical;

    for (std::uint8_t led = 0; led < 6; led++) {
        logical.insert(logical.end(), {led, led, led, 255});
    }

    panel.updateFrame(0, logical);

    std::vector<std::uint8_t> physical_order;
    const auto frame = panel.getFrame();

    for (std::size_t i = 0; i < frame.size(); i += 4) {
        physical_order.push_back(frame[i]);
    }

    EXPECT_EQ(physical_order, (std::vector<std::uint8_t>{0, 2, 4, 5, 3, 1}));
}

TEST(LedPanel, RejectsRowsThatDoNotDivideTheStrip)
{
    EXPECT_THROW(rover_led::LedPanel(5, 2), std::runtime_error);
    EXPECT_THROW(rover_led::LedPanel(4, 0), std::runtime_error);
    EXPECT_NO_THROW(rover_led::LedPanel(4, 4));
}
