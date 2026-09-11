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
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

#include "rover_led/domain/led_components/led_panel.hpp"
#include "rover_led/domain/led_components/led_segment.hpp"
#include "rover_led/domain/led_components/segment_converter.hpp"

#include "../test_helpers.hpp"

using rover_led::LedPanel;
using rover_led::LedSegment;
using rover_led::LedSegmentConfig;
using rover_led::test::makeStub;
using rover_led::test::pixel;
using rover_led::test::Rgba;

namespace
{

using Segments = std::unordered_map<std::string, std::shared_ptr<LedSegment>>;
using Panels = std::unordered_map<std::size_t, std::shared_ptr<LedPanel>>;

}  // namespace

TEST(SegmentConverter, CopiesSegmentsIntoTheirPanelRangeWithOpaqueAlpha)
{
    Panels panels{{1, std::make_shared<LedPanel>(4)}};
    Segments segments{
        {"left", std::make_shared<LedSegment>(LedSegmentConfig{1, 0, 1})},
        {"right", std::make_shared<LedSegment>(LedSegmentConfig{1, 3, 2})},
    };

    segments["left"]->setAnimation(makeStub(2, {10, 20, 30, 40}), false, rover_led::STATE);
    segments["right"]->setAnimation(makeStub(2, {70, 80, 90, 255}), false, rover_led::STATE);

    for (auto & [name, segment] : segments) {
        segment->updateAnimation();
    }

    rover_led::SegmentConverter().convert(segments, panels);
    const auto frame = panels[1]->getFrame();

    // Segment colour is blended over black, then alpha is forced to 255.
    EXPECT_EQ(pixel(frame, 0), (Rgba{1, 3, 4, 255}));
    EXPECT_EQ(pixel(frame, 1), (Rgba{1, 3, 4, 255}));
    EXPECT_EQ(pixel(frame, 2), (Rgba{70, 80, 90, 255}));
    EXPECT_EQ(pixel(frame, 3), (Rgba{70, 80, 90, 255}));
}

TEST(SegmentConverter, SkipsSegmentsWithoutAnimation)
{
    Panels panels{{1, std::make_shared<LedPanel>(2)}};
    Segments segments{{"idle", std::make_shared<LedSegment>(LedSegmentConfig{1, 0, 1})}};

    rover_led::SegmentConverter().convert(segments, panels);

    EXPECT_EQ(panels[1]->getFrame(), std::vector<std::uint8_t>(8, 0));
}

TEST(SegmentConverter, FailsForAnUnknownChannel)
{
    Panels panels{{1, std::make_shared<LedPanel>(2)}};
    Segments segments{{"lost", std::make_shared<LedSegment>(LedSegmentConfig{7, 0, 1})}};
    segments["lost"]->setAnimation(makeStub(2), false, rover_led::STATE);

    EXPECT_THROW(rover_led::SegmentConverter().convert(segments, panels), std::runtime_error);
}

TEST(SegmentConverter, FailsForASegmentThatDoesNotFitItsPanel)
{
    Panels panels{{1, std::make_shared<LedPanel>(2)}};
    Segments segments{{"long", std::make_shared<LedSegment>(LedSegmentConfig{1, 1, 3})}};
    segments["long"]->setAnimation(makeStub(3), false, rover_led::STATE);

    EXPECT_THROW(rover_led::SegmentConverter().convert(segments, panels), std::runtime_error);
}
