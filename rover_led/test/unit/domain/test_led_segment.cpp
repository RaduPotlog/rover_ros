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

#include "rover_led/domain/led_components/led_segment.hpp"

#include "../test_helpers.hpp"

using rover_led::LedSegment;
using rover_led::LedSegmentConfig;
using rover_led::test::makeStub;
using rover_led::test::pixel;
using rover_led::test::Rgba;

TEST(LedSegment, ForwardRange)
{
    LedSegment segment(LedSegmentConfig{1, 0, 19});

    EXPECT_EQ(segment.getChannel(), 1u);
    EXPECT_EQ(segment.getNumberOfLeds(), 20u);
    EXPECT_EQ(segment.getFirstLedPosition(), 0u);
}

TEST(LedSegment, ReversedRangeStartsAtTheLowerLed)
{
    LedSegment segment(LedSegmentConfig{2, 39, 20});

    EXPECT_EQ(segment.getNumberOfLeds(), 20u);
    EXPECT_EQ(segment.getFirstLedPosition(), 20u * 4);
}

TEST(LedSegment, SingleLed)
{
    EXPECT_EQ(LedSegment(LedSegmentConfig{1, 5, 5}).getNumberOfLeds(), 1u);
}

TEST(LedSegment, RejectsInvalidPriorityAndMismatchedAnimation)
{
    LedSegment segment(LedSegmentConfig{1, 0, 3});

    EXPECT_THROW(segment.setAnimation(makeStub(4), false, 4), std::runtime_error);
    EXPECT_THROW(segment.setAnimation(makeStub(5), false, rover_led::STATE), std::runtime_error);
    EXPECT_THROW(segment.setAnimation(nullptr, false, rover_led::STATE), std::runtime_error);
    EXPECT_FALSE(segment.hasAnimation());
}

TEST(LedSegment, AnimationGoesToItsPriorityLayer)
{
    LedSegment segment(LedSegmentConfig{1, 0, 3});

    EXPECT_TRUE(segment.setAnimation(makeStub(4), false, rover_led::INFO));

    EXPECT_TRUE(segment.hasAnimation());
    EXPECT_TRUE(segment.layerHasAnimation(rover_led::INFO));
    EXPECT_FALSE(segment.layerHasAnimation(rover_led::STATE));
    EXPECT_FALSE(segment.layerHasAnimation(rover_led::ERROR));
}

TEST(LedSegment, OpaqueHigherPriorityLayerWins)
{
    LedSegment segment(LedSegmentConfig{1, 0, 1});
    segment.setAnimation(makeStub(2, {10, 20, 30, 255}), false, rover_led::STATE);
    segment.setAnimation(makeStub(2, {90, 80, 70, 255}), false, rover_led::ERROR);
    segment.updateAnimation();

    EXPECT_EQ(pixel(segment.getAnimationFrame(), 0), (Rgba{90, 80, 70, 255}));
}

TEST(LedSegment, TranslucentLayerIsAlphaBlended)
{
    LedSegment segment(LedSegmentConfig{1, 0, 0});
    segment.setAnimation(makeStub(1, {200, 0, 100, 255}), false, rover_led::STATE);
    segment.setAnimation(makeStub(1, {0, 200, 0, 51}), false, rover_led::ALERT);
    segment.updateAnimation();

    // c = (over * a + base * (255 - a)) / 255, alpha = a + base_a * (255 - a) / 255
    EXPECT_EQ(pixel(segment.getAnimationFrame(), 0), (Rgba{160, 40, 80, 255}));
}

TEST(LedSegment, ReversedSegmentRendersLedsBackwards)
{
    LedSegment segment(LedSegmentConfig{1, 1, 0});
    auto animation = makeStub(2, {0, 0, 0, 255});
    segment.setAnimation(animation, false, rover_led::STATE);

    // Stub frames are uniform, so compare against the animation's own
    // inverted frame.
    segment.updateAnimation();
    EXPECT_EQ(segment.getAnimationFrame(), animation->getFrame(true));
}

TEST(LedSegment, NonRepeatingAnimationEndsBlank)
{
    LedSegment segment(LedSegmentConfig{1, 0, 0});
    segment.setAnimation(makeStub(1, {5, 5, 5, 255}, 0.1f), false, rover_led::STATE);

    segment.updateAnimation();
    EXPECT_EQ(pixel(segment.getAnimationFrame(), 0), (Rgba{5, 5, 5, 255}));

    segment.updateAnimation();
    EXPECT_TRUE(segment.isAnimationFinished(rover_led::STATE));
    EXPECT_EQ(segment.getAnimationFrame(), std::vector<std::uint8_t>(4, 0));
}
