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

#include "rover_led/domain/led_components/segment_layer.hpp"
#include "rover_led/domain/led_components/segment_queue_layer.hpp"

#include "../test_helpers.hpp"

using rover_led::SegmentLayer;
using rover_led::SegmentQueueLayer;
using rover_led::test::makeStub;
using rover_led::test::pixel;
using rover_led::test::Rgba;

TEST(SegmentLayer, EmptyLayerIsBlankAndRefusesQueries)
{
    SegmentLayer layer(2, false);

    EXPECT_FALSE(layer.hasAnimation());
    EXPECT_EQ(layer.getAnimationFrame(), std::vector<std::uint8_t>(8, 0));
    EXPECT_THROW(layer.updateAnimation(), std::runtime_error);
    EXPECT_THROW(layer.getAnimationProgress(), std::runtime_error);
    EXPECT_THROW(layer.resetAnimation(), std::runtime_error);
}

TEST(SegmentLayer, OneShotAnimationFinishes)
{
    SegmentLayer layer(1, false);
    EXPECT_TRUE(layer.setAnimation(makeStub(1, {1, 2, 3, 255}, 0.2f), false));

    layer.updateAnimation();
    layer.updateAnimation();
    EXPECT_FALSE(layer.isAnimationFinished());

    layer.updateAnimation();
    EXPECT_TRUE(layer.isAnimationFinished());
    EXPECT_EQ(layer.getAnimationFrame(), std::vector<std::uint8_t>(4, 0));
}

TEST(SegmentLayer, RepeatingAnimationRestarts)
{
    SegmentLayer layer(1, false);
    layer.setAnimation(makeStub(1, {0, 0, 0, 255}, 0.2f), true);

    std::vector<std::uint8_t> iterations;

    for (int i = 0; i < 5; i++) {
        layer.updateAnimation();
        iterations.push_back(pixel(layer.getAnimationFrame(), 0)[0]);
    }

    EXPECT_FALSE(layer.isAnimationFinished());
    EXPECT_EQ(iterations, (std::vector<std::uint8_t>{0, 1, 0, 1, 0}));
}

TEST(SegmentLayer, NewAnimationReplacesTheCurrentOne)
{
    SegmentLayer layer(1, false);
    layer.setAnimation(makeStub(1, {10, 0, 0, 255}), true);
    layer.updateAnimation();

    layer.setAnimation(makeStub(1, {50, 0, 0, 255}), false);
    layer.updateAnimation();

    EXPECT_EQ(pixel(layer.getAnimationFrame(), 0)[0], 50);
}

TEST(SegmentQueueLayer, PlaysAnimationsInOrderThenEmpties)
{
    SegmentQueueLayer layer(1, false);
    layer.setAnimation(makeStub(1, {10, 0, 0, 255}, 0.1f), false);
    layer.setAnimation(makeStub(1, {50, 0, 0, 255}, 0.1f), false);

    layer.updateAnimation();
    EXPECT_EQ(pixel(layer.getAnimationFrame(), 0)[0], 10);

    layer.updateAnimation();
    EXPECT_EQ(pixel(layer.getAnimationFrame(), 0)[0], 50);

    layer.updateAnimation();
    EXPECT_FALSE(layer.hasAnimation());
}

TEST(SegmentQueueLayer, IgnoresRepeating)
{
    SegmentQueueLayer layer(1, false);
    layer.setAnimation(makeStub(1, {0, 0, 0, 255}, 0.1f), true);

    layer.updateAnimation();
    layer.updateAnimation();

    EXPECT_FALSE(layer.hasAnimation());
}

TEST(SegmentQueueLayer, RejectsAnimationsWhenTheQueueIsFull)
{
    SegmentQueueLayer layer(1, false);

    // The first animation plays; the next kMaxQueueSize wait.
    for (std::size_t i = 0; i <= SegmentQueueLayer::kMaxQueueSize; i++) {
        EXPECT_TRUE(layer.setAnimation(makeStub(1), false)) << i;
    }

    EXPECT_FALSE(layer.setAnimation(makeStub(1), false));
}
