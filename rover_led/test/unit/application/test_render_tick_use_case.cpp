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
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "rover_led/application/render_tick_use_case.hpp"

#include "../test_helpers.hpp"

using rover_led::LedPanel;
using rover_led::LedSegment;
using rover_led::LedSegmentConfig;
using rover_led::RenderTickUseCase;
using rover_led::test::makeStub;
using rover_led::test::pixel;
using rover_led::test::Rgba;

TEST(RenderTickUseCase, ProducesAFramePerPanel)
{
    rover_led::PanelMap panels{
        {1, std::make_shared<LedPanel>(2)},
        {2, std::make_shared<LedPanel>(3)},
    };
    rover_led::SegmentMap segments{{"front", std::make_shared<LedSegment>(LedSegmentConfig{1, 0, 1})}};
    segments["front"]->setAnimation(makeStub(2, {40, 0, 0, 255}), false, rover_led::STATE);

    RenderTickUseCase use_case(segments, panels);

    auto result = use_case.execute();

    ASSERT_FALSE(result.error);
    EXPECT_TRUE(result.segment_errors.empty());
    ASSERT_EQ(result.frames.size(), 2u);
    EXPECT_EQ(pixel(result.frames.at(1), 1), (Rgba{40, 0, 0, 255}));
    EXPECT_EQ(result.frames.at(2), std::vector<std::uint8_t>(12, 0));

    // Each tick advances the animation one frame.
    result = use_case.execute();
    EXPECT_EQ(pixel(result.frames.at(1), 1)[0], 41);
}

TEST(RenderTickUseCase, AFailingSegmentDoesNotStopTheOthers)
{
    rover_led::PanelMap panels{{1, std::make_shared<LedPanel>(2)}};
    rover_led::SegmentMap segments{
        {"good", std::make_shared<LedSegment>(LedSegmentConfig{1, 0, 0})},
        {"bad", std::make_shared<LedSegment>(LedSegmentConfig{1, 1, 1})},
    };
    segments["good"]->setAnimation(makeStub(1, {9, 9, 9, 255}), false, rover_led::STATE);
    auto broken = makeStub(1);
    broken->fail_update = true;
    segments["bad"]->setAnimation(broken, false, rover_led::STATE);

    const auto result = RenderTickUseCase(segments, panels).execute();

    ASSERT_EQ(result.segment_errors.size(), 1u);
    EXPECT_THAT(result.segment_errors[0], ::testing::StartsWith("bad: "));
    ASSERT_FALSE(result.error);
    EXPECT_EQ(pixel(result.frames.at(1), 0), (Rgba{9, 9, 9, 255}));
}

TEST(RenderTickUseCase, NoFramesWhenSegmentsCannotBeComposed)
{
    rover_led::PanelMap panels{{1, std::make_shared<LedPanel>(2)}};
    rover_led::SegmentMap segments{{"lost", std::make_shared<LedSegment>(LedSegmentConfig{3, 0, 1})}};
    segments["lost"]->setAnimation(makeStub(2), false, rover_led::STATE);

    const auto result = RenderTickUseCase(segments, panels).execute();

    ASSERT_TRUE(result.error);
    EXPECT_THAT(*result.error, ::testing::HasSubstr("'lost'"));
    EXPECT_TRUE(result.frames.empty());
}
