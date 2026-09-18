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

// Renders BLINKER_LEFT / BLINKER_RIGHT from the shipped rover_a1 config through
// the same use cases as rover_led_controller, and checks they light opposite
// sides of each bumper. Both panels are 2 rows x 20 LEDs in series (see
// rover_a1_animations.yaml); frames are in wire order, so sides are checked by
// physical column.

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rover_led/application/render_tick_use_case.hpp"
#include "rover_led/application/set_animation_use_case.hpp"
#include "rover_led/domain/animation/image_animation.hpp"
#include "rover_led/domain/animation/moving_image_animation.hpp"
#include "rover_led/domain/ports/animation_factory.hpp"
#include "rover_led/infrastructure/yaml_led_config.hpp"

namespace
{

constexpr std::size_t kBlinkerLeft = 14;
constexpr std::size_t kBlinkerRight = 15;
constexpr std::size_t kFrontChannel = 1;
constexpr std::size_t kRearChannel = 2;
constexpr float kControllerFrequency = 50.0f;

using LitLeds = std::map<std::size_t, std::set<std::size_t>>;

// Both panels: row 1 (LEDs 0-19) runs away from LED 0 on the robot's right,
// row 2 (LEDs 20-39) comes back. Column 0 is the robot's right.
std::size_t columnFromRobotRight(const std::size_t led)
{
    return led < 20 ? led : 39 - led;
}

// Avoids pluginlib, which needs rover_led on the ament index.
class DomainAnimationFactory : public rover_led::IAnimationFactory
{

public:

    std::shared_ptr<rover_led::Animation> create(const std::string & type) override
    {
        if (type == "rover_led::ImageAnimation") {
            return std::make_shared<rover_led::ImageAnimation>();
        }

        if (type == "rover_led::MovingImageAnimation") {
            return std::make_shared<rover_led::MovingImageAnimation>();
        }

        throw std::runtime_error("Unknown animation type: " + type);
    }
};

// The shipped config with "$(find rover_led)" pointed at the source tree.
YAML::Node loadShippedConfig()
{
    const std::string source_dir(ROVER_LED_SOURCE_DIR);
    std::ifstream file(source_dir + "/config/rover_a1_animations.yaml");
    std::stringstream buffer;
    buffer << file.rdbuf();

    std::string text = buffer.str();
    const std::string find_self = "$(find rover_led)";

    for (auto pos = text.find(find_self); pos != std::string::npos; pos = text.find(find_self, pos)) {
        text.replace(pos, find_self.size(), source_dir);
    }

    return YAML::Load(text);
}

// Plays one animation from the shipped catalog and returns, per panel
// channel, every LED that was lit at any point of one full cycle.
LitLeds playAndCollectLitLeds(const std::size_t animation_id)
{
    const auto config = loadShippedConfig();
    const auto layout = rover_led::parseLedLayout(config);

    rover_led::PanelMap panels;

    for (const auto & panel : layout.panels) {
        panels.emplace(panel.channel, std::make_shared<rover_led::LedPanel>(panel.number_of_leds, panel.rows));
    }

    rover_led::SegmentMap segments;

    for (const auto & segment : layout.segments) {
        segments.emplace(segment.name, std::make_shared<rover_led::LedSegment>(segment.config));
    }

    const auto catalog = std::make_shared<rover_led::YamlAnimationCatalog>(
        config["led_animations"], layout.segments_map);

    rover_led::SetAnimationUseCase set_animation(
        catalog, std::make_shared<DomainAnimationFactory>(), segments, kControllerFrequency);
    rover_led::RenderTickUseCase render_tick(segments, panels);

    rover_led::LedAnimationRequest request;
    request.id = animation_id;
    set_animation.execute(request);

    LitLeds lit;

    // duration 2 s at 50 Hz: one cycle is 100 ticks.
    for (int tick = 0; tick < 100; tick++) {
        const auto result = render_tick.execute();

        if (result.error) {
            ADD_FAILURE() << *result.error;
            return lit;
        }

        for (const auto & [channel, frame] : result.frames) {
            for (std::size_t led = 0; led < frame.size() / 4; led++) {
                if (frame[led * 4] || frame[led * 4 + 1] || frame[led * 4 + 2]) {
                    lit[channel].insert(led);
                }
            }
        }
    }

    return lit;
}

}  // namespace

using ::testing::IsEmpty;
using ::testing::Not;

TEST(ShippedBlinkers, LeftAndRightLightOppositeEndsOfEachBumper)
{
    const auto left = playAndCollectLitLeds(kBlinkerLeft);
    const auto right = playAndCollectLitLeds(kBlinkerRight);

    for (const auto channel : {kFrontChannel, kRearChannel}) {
        ASSERT_THAT(left.count(channel) ? left.at(channel) : std::set<std::size_t>{}, Not(IsEmpty()))
            << "BLINKER_LEFT lit nothing on channel " << channel;
        ASSERT_THAT(right.count(channel) ? right.at(channel) : std::set<std::size_t>{}, Not(IsEmpty()))
            << "BLINKER_RIGHT lit nothing on channel " << channel;

        for (const auto led : left.at(channel)) {
            EXPECT_EQ(right.at(channel).count(led), 0u)
                << "LED " << led << " on channel " << channel << " is lit by both blinkers";
        }
    }

    // Each blinker lights its own side of the robot on both rows of both panels.
    for (const auto channel : {kFrontChannel, kRearChannel}) {
        for (const auto & [lit, on_right] : {std::pair{right, true}, std::pair{left, false}}) {
            bool first_row = false;
            bool second_row = false;

            for (const auto led : lit.at(channel)) {
                EXPECT_EQ(columnFromRobotRight(led) < 10, on_right)
                    << "LED " << led << " on channel " << channel << " is on the wrong side for BLINKER_"
                    << (on_right ? "RIGHT" : "LEFT");
                (led < 20 ? first_row : second_row) = true;
            }

            EXPECT_TRUE(first_row && second_row)
                << "BLINKER_" << (on_right ? "RIGHT" : "LEFT") << " does not light both rows of channel "
                << channel;
        }
    }
}
