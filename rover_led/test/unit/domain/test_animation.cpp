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
#include "yaml-cpp/yaml.h"

#include "../test_helpers.hpp"

using rover_led::test::makeStub;
using rover_led::test::pixel;
using rover_led::test::Rgba;
using rover_led::test::StubAnimation;
using rover_led::test::stubDescription;

TEST(Animation, FinishesAfterDurationTimesFrequencyUpdates)
{
    auto animation = makeStub(3, {0, 0, 0, 255}, 1.0f);

    for (int i = 0; i < 9; i++) {
        animation->update();
        EXPECT_FALSE(animation->isFinished());
    }

    animation->update();
    EXPECT_TRUE(animation->isFinished());
    EXPECT_FLOAT_EQ(animation->getProgress(), 1.0f);
}

TEST(Animation, RepeatMultipliesTheLength)
{
    auto animation = makeStub(1, {0, 0, 0, 255}, 0.5f, 3);

    for (int i = 0; i < 3; i++) {
        animation->update();
    }

    EXPECT_FLOAT_EQ(animation->getProgress(), 0.2f);

    for (int i = 3; i < 15; i++) {
        EXPECT_FALSE(animation->isFinished());
        animation->update();
    }

    EXPECT_TRUE(animation->isFinished());
}

TEST(Animation, EachLoopRestartsTheFrameSequence)
{
    auto animation = makeStub(1, {0, 0, 0, 255}, 0.2f, 2);

    std::vector<std::uint8_t> iterations;

    for (int i = 0; i < 4; i++) {
        animation->update();
        iterations.push_back(pixel(animation->getFrame(), 0)[0]);
    }

    EXPECT_EQ(iterations, (std::vector<std::uint8_t>{0, 1, 0, 1}));
}

TEST(Animation, FrameIsBlankOnceFinished)
{
    auto animation = makeStub(2, {50, 60, 70, 255}, 0.1f);

    animation->update();
    EXPECT_EQ(pixel(animation->getFrame(), 1), (Rgba{50, 60, 70, 255}));

    animation->update();
    EXPECT_EQ(animation->getFrame(), std::vector<std::uint8_t>(8, 0));
}

TEST(Animation, ResetStartsOver)
{
    auto animation = makeStub(1, {0, 0, 0, 255}, 0.1f);
    animation->update();
    ASSERT_TRUE(animation->isFinished());

    animation->reset();

    EXPECT_FALSE(animation->isFinished());
    EXPECT_FLOAT_EQ(animation->getProgress(), 0.0f);
    EXPECT_EQ(animation->getFrame(), std::vector<std::uint8_t>(4, 0));
}

namespace
{

// LED i is painted {i, 100 + i, 200 + i, 255}.
class IndexAnimation : public rover_led::Animation
{

protected:

    std::vector<std::uint8_t> updateFrame() override
    {
        std::vector<std::uint8_t> frame;

        for (std::size_t i = 0; i < getNumberOfLeds(); i++) {
            frame.insert(frame.end(), {std::uint8_t(i), std::uint8_t(100 + i), std::uint8_t(200 + i), 255});
        }

        return frame;
    }
};

class BrokenFrameAnimation : public rover_led::Animation
{

protected:

    std::vector<std::uint8_t> updateFrame() override
    {
        return {1, 2, 3};
    }
};

}  // namespace

TEST(Animation, InvertedFrameReversesLedOrderButNotChannels)
{
    IndexAnimation animation;
    animation.initialize(stubDescription(), 3, 10.0f);
    animation.update();

    const auto frame = animation.getFrame();
    const auto inverted = animation.getFrame(true);

    EXPECT_EQ(pixel(frame, 0), (Rgba{0, 100, 200, 255}));
    EXPECT_EQ(pixel(inverted, 0), (Rgba{2, 102, 202, 255}));
    EXPECT_EQ(pixel(inverted, 1), (Rgba{1, 101, 201, 255}));
    EXPECT_EQ(pixel(inverted, 2), (Rgba{0, 100, 200, 255}));
}

TEST(Animation, RejectsFramesOfTheWrongSize)
{
    BrokenFrameAnimation animation;
    animation.initialize(stubDescription(), 3, 10.0f);

    EXPECT_THROW(animation.update(), std::runtime_error);
}

TEST(Animation, RejectsInvalidDurations)
{
    StubAnimation animation;

    EXPECT_THROW(animation.initialize(stubDescription(0.0f), 1, 10.0f), std::out_of_range);
    EXPECT_THROW(animation.initialize(stubDescription(-1.0f), 1, 10.0f), std::out_of_range);
    EXPECT_THROW(animation.initialize(stubDescription(6.0f, 2), 1, 10.0f), std::runtime_error);
    // 0.01 s at 10 Hz rounds to zero frames.
    EXPECT_THROW(animation.initialize(stubDescription(0.01f), 1, 10.0f), std::runtime_error);
    EXPECT_THROW(animation.initialize(YAML::Node(), 1, 10.0f), std::runtime_error);
}

TEST(Animation, ReportsItsLedCount)
{
    EXPECT_EQ(makeStub(17)->getNumberOfLeds(), 17u);
}
