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
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rover_led/application/stop_animation_use_case.hpp"

#include "../test_helpers.hpp"

using rover_led::AnimationDescription;
using rover_led::LedAnimationDescription;
using rover_led::LedSegment;
using rover_led::LedSegmentConfig;
using rover_led::StopAnimationUseCase;
using rover_led::test::makeStub;
using rover_led::test::stubDescription;

namespace
{

class FakeCatalog : public rover_led::IAnimationCatalog
{

public:

    std::optional<LedAnimationDescription> find(const std::size_t id) const override
    {
        const auto it = animations.find(id);

        return it == animations.end() ? std::nullopt : std::optional(it->second);
    }

    std::map<std::size_t, LedAnimationDescription> animations;
};

class StopAnimationUseCaseTest : public ::testing::Test
{

protected:

    StopAnimationUseCaseTest()
    {
        segments_["front"] = std::make_shared<LedSegment>(LedSegmentConfig{1, 0, 3});
        segments_["rear"] = std::make_shared<LedSegment>(LedSegmentConfig{2, 9, 0});

        catalog_ = std::make_shared<FakeCatalog>();
        addAnimation(1, "BLINK", rover_led::STATE, {"front", "rear"});
        addAnimation(2, "FLASH", rover_led::ALERT, {"front"});

        use_case_ = std::make_unique<StopAnimationUseCase>(catalog_, segments_);
    }

    void addAnimation(
        const std::size_t id, const std::string & name, const std::uint8_t priority,
        const std::vector<std::string> & segments)
    {
        LedAnimationDescription led_animation{id, priority, name, 120.0f, {}};
        led_animation.animations.push_back(AnimationDescription{"Stub", segments, stubDescription()});
        catalog_->animations[id] = led_animation;
    }

    // What SetAnimationUseCase does: one animation per segment, tagged with the catalog entry.
    void play(const std::size_t id, const std::string & segment_name, const bool repeating = true)
    {
        const auto & description = catalog_->animations.at(id);
        const auto & segment = segments_.at(segment_name);
        auto animation = makeStub(segment->getNumberOfLeds());
        animation->setInfo({id, description.name, ""});
        segment->setAnimation(animation, repeating, description.priority);
        segment->updateAnimation();
    }

    rover_led::SegmentMap segments_;
    std::shared_ptr<FakeCatalog> catalog_;
    std::unique_ptr<StopAnimationUseCase> use_case_;
};

}  // namespace

TEST_F(StopAnimationUseCaseTest, StopsTheAnimationOnEverySegmentItPlaysOn)
{
    play(1, "front");
    play(1, "rear");

    const auto result = use_case_->execute(1);

    EXPECT_EQ(result.name, "BLINK");
    EXPECT_EQ(result.stopped_segments, (std::vector<std::string>{"front", "rear"}));
    EXPECT_FALSE(segments_["front"]->layerHasAnimation(rover_led::STATE));
    EXPECT_FALSE(segments_["rear"]->layerHasAnimation(rover_led::STATE));
}

TEST_F(StopAnimationUseCaseTest, ReportsOnlyTheSegmentsWhereItWasPlaying)
{
    play(1, "rear");

    EXPECT_EQ(use_case_->execute(1).stopped_segments, (std::vector<std::string>{"rear"}));
}

TEST_F(StopAnimationUseCaseTest, NothingStopsWhenTheAnimationIsNotPlaying)
{
    play(2, "front");

    const auto result = use_case_->execute(1);

    EXPECT_EQ(result.name, "BLINK");
    EXPECT_TRUE(result.stopped_segments.empty());
    EXPECT_TRUE(segments_["front"]->layerHasAnimation(rover_led::ALERT));
}

TEST_F(StopAnimationUseCaseTest, StopsAnAlertAnimation)
{
    play(2, "front", false);

    EXPECT_EQ(use_case_->execute(2).stopped_segments, (std::vector<std::string>{"front"}));
    EXPECT_FALSE(segments_["front"]->layerHasAnimation(rover_led::ALERT));
}

TEST_F(StopAnimationUseCaseTest, RejectsUnknownIdsAndSegments)
{
    EXPECT_THROW(use_case_->execute(99), std::runtime_error);

    addAnimation(3, "SIDE", rover_led::INFO, {"left"});
    EXPECT_THROW(use_case_->execute(3), std::runtime_error);
}
