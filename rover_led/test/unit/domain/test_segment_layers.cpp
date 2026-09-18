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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
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

TEST(SegmentLayer, StatusIsEmptyWithoutAnimation)
{
    SegmentLayer layer(1, false);

    EXPECT_FALSE(layer.getStatus().has_value());
}

TEST(SegmentLayer, StatusReportsThePlayingAnimation)
{
    SegmentLayer layer(1, false);
    auto animation = makeStub(1);
    animation->setInfo({7, "BLINK", "0.5"});
    layer.setAnimation(animation, true);
    layer.updateAnimation();

    const auto status = layer.getStatus();

    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->info.id, 7u);
    EXPECT_EQ(status->info.name, "BLINK");
    EXPECT_EQ(status->info.param, "0.5");
    EXPECT_TRUE(status->repeating);
    EXPECT_FLOAT_EQ(status->progress, 0.1f);
    EXPECT_EQ(status->queued, 0u);
}

TEST(SegmentLayer, StatusIsEmptyOnceAOneShotAnimationFinishes)
{
    SegmentLayer layer(1, false);
    layer.setAnimation(makeStub(1, {0, 0, 0, 255}, 0.1f), false);

    layer.updateAnimation();
    EXPECT_TRUE(layer.getStatus().has_value());

    layer.updateAnimation();
    EXPECT_FALSE(layer.getStatus().has_value());
}

TEST(SegmentQueueLayer, StatusCountsQueuedAnimations)
{
    SegmentQueueLayer layer(1, false);
    auto first = makeStub(1, {0, 0, 0, 255}, 0.1f);
    first->setInfo({9, "FIRST", ""});
    layer.setAnimation(first, true);
    layer.setAnimation(makeStub(1, {0, 0, 0, 255}, 0.1f), false);
    layer.setAnimation(makeStub(1, {0, 0, 0, 255}, 0.1f), false);

    const auto status = layer.getStatus();

    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->info.name, "FIRST");
    EXPECT_FALSE(status->repeating);
    EXPECT_EQ(status->queued, 2u);

    layer.updateAnimation();
    layer.updateAnimation();
    EXPECT_EQ(layer.getStatus()->queued, 1u);
}

namespace
{

std::shared_ptr<rover_led::test::StubAnimation> makeStubWithId(const std::size_t id, const Rgba color = {0, 0, 0, 255})
{
    auto animation = makeStub(1, color);
    animation->setInfo({id, "STUB_" + std::to_string(id), ""});

    return animation;
}

}  // namespace

TEST(SegmentLayer, StopsTheMatchingRepeatingAnimation)
{
    SegmentLayer layer(1, false);
    layer.setAnimation(makeStubWithId(7, {9, 0, 0, 255}), true);
    layer.updateAnimation();

    EXPECT_TRUE(layer.stopAnimation(7));
    EXPECT_FALSE(layer.hasAnimation());
    EXPECT_FALSE(layer.getStatus().has_value());
    EXPECT_EQ(layer.getAnimationFrame(), std::vector<std::uint8_t>(4, 0));
}

TEST(SegmentLayer, StopIgnoresOtherAnimations)
{
    SegmentLayer layer(1, false);
    EXPECT_FALSE(layer.stopAnimation(7));

    layer.setAnimation(makeStubWithId(3), true);
    layer.updateAnimation();

    EXPECT_FALSE(layer.stopAnimation(7));
    EXPECT_TRUE(layer.hasAnimation());
}

TEST(SegmentLayer, StopIgnoresAFinishedOneShot)
{
    SegmentLayer layer(1, false);
    layer.setAnimation(makeStubWithId(7), false);

    for (int i = 0; i < 11; i++) {
        layer.updateAnimation();
    }

    ASSERT_TRUE(layer.isAnimationFinished());
    EXPECT_FALSE(layer.stopAnimation(7));
}

TEST(SegmentLayer, AcceptsANewAnimationAfterAStop)
{
    SegmentLayer layer(1, false);
    layer.setAnimation(makeStubWithId(7), true);
    layer.stopAnimation(7);

    layer.setAnimation(makeStubWithId(8, {40, 0, 0, 255}), true);
    layer.updateAnimation();

    EXPECT_EQ(pixel(layer.getAnimationFrame(), 0)[0], 40);
}

TEST(SegmentQueueLayer, StoppingTheCurrentAnimationPlaysTheNextQueuedOne)
{
    SegmentQueueLayer layer(1, false);
    layer.setAnimation(makeStubWithId(7, {10, 0, 0, 255}), false);
    layer.setAnimation(makeStubWithId(8, {50, 0, 0, 255}), false);
    layer.updateAnimation();

    EXPECT_TRUE(layer.stopAnimation(7));
    layer.updateAnimation();

    EXPECT_EQ(pixel(layer.getAnimationFrame(), 0)[0], 50);
    EXPECT_EQ(layer.getStatus()->queued, 0u);
}

TEST(SegmentQueueLayer, StopDropsQueuedCopies)
{
    SegmentQueueLayer layer(1, false);
    layer.setAnimation(makeStubWithId(8), false);
    layer.setAnimation(makeStubWithId(7), false);
    layer.setAnimation(makeStubWithId(7), false);
    layer.updateAnimation();

    EXPECT_TRUE(layer.stopAnimation(7));
    EXPECT_EQ(layer.getStatus()->info.id, 8u);
    EXPECT_EQ(layer.getStatus()->queued, 0u);
}

TEST(SegmentQueueLayer, StoppingTheLastAnimationEmptiesTheLayer)
{
    SegmentQueueLayer layer(1, false);
    layer.setAnimation(makeStubWithId(7), false);
    layer.updateAnimation();

    EXPECT_TRUE(layer.stopAnimation(7));
    EXPECT_FALSE(layer.hasAnimation());
    EXPECT_FALSE(layer.stopAnimation(7));

    // The next animation plays straight away instead of queueing.
    layer.setAnimation(makeStubWithId(8, {50, 0, 0, 255}), false);
    layer.updateAnimation();
    EXPECT_EQ(pixel(layer.getAnimationFrame(), 0)[0], 50);
}
