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

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rover_led/application/set_animation_use_case.hpp"
#include "rover_led/domain/led_components/segment_queue_layer.hpp"

#include "../test_helpers.hpp"

using rover_led::AnimationDescription;
using rover_led::LedAnimationDescription;
using rover_led::LedSegment;
using rover_led::LedSegmentConfig;
using rover_led::SetAnimationUseCase;
using rover_led::test::StubAnimation;
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

class FakeFactory : public rover_led::IAnimationFactory
{

public:

    std::shared_ptr<rover_led::Animation> create(const std::string & type) override
    {
        if (type != "Stub") {
            throw std::runtime_error("The plugin failed to load. Error: unknown " + type);
        }

        auto animation = std::make_shared<StubAnimation>();
        created.push_back(animation);

        return animation;
    }

    std::vector<std::shared_ptr<StubAnimation>> created;
};

class SetAnimationUseCaseTest : public ::testing::Test
{

protected:

    SetAnimationUseCaseTest()
    {
        segments_["front"] = std::make_shared<LedSegment>(LedSegmentConfig{1, 0, 3});
        segments_["rear"] = std::make_shared<LedSegment>(LedSegmentConfig{2, 9, 0});

        catalog_ = std::make_shared<FakeCatalog>();
        factory_ = std::make_shared<FakeFactory>();

        addAnimation(1, "BLINK", rover_led::STATE, {"front", "rear"});

        use_case_ = std::make_unique<SetAnimationUseCase>(catalog_, factory_, segments_, 10.0f);
    }

    void addAnimation(
        const std::size_t id, const std::string & name, const std::uint8_t priority,
        const std::vector<std::string> & segments, const YAML::Node & description = stubDescription(),
        const std::string & type = "Stub")
    {
        LedAnimationDescription led_animation{id, priority, name, 120.0f, {}};
        led_animation.animations.push_back(AnimationDescription{type, segments, description});
        catalog_->animations[id] = led_animation;
    }

    rover_led::SegmentMap segments_;
    std::shared_ptr<FakeCatalog> catalog_;
    std::shared_ptr<FakeFactory> factory_;
    std::unique_ptr<SetAnimationUseCase> use_case_;
};

}  // namespace

TEST_F(SetAnimationUseCaseTest, PutsTheAnimationOnEverySegmentAtItsPriority)
{
    const auto result = use_case_->execute({1, "", false});

    EXPECT_EQ(result.name, "BLINK");
    EXPECT_TRUE(result.rejected_segments.empty());
    EXPECT_TRUE(segments_["front"]->layerHasAnimation(rover_led::STATE));
    EXPECT_TRUE(segments_["rear"]->layerHasAnimation(rover_led::STATE));
    EXPECT_FALSE(segments_["front"]->layerHasAnimation(rover_led::ERROR));
}

TEST_F(SetAnimationUseCaseTest, CreatesOneAnimationPerSegmentSizedForIt)
{
    use_case_->execute({1, "0.5", true});

    ASSERT_EQ(factory_->created.size(), 2u);

    std::vector<std::size_t> sizes;

    for (const auto & animation : factory_->created) {
        sizes.push_back(animation->getNumberOfLeds());
        EXPECT_EQ(animation->param(), "0.5");
    }

    EXPECT_THAT(sizes, ::testing::UnorderedElementsAre(4u, 10u));
}

TEST_F(SetAnimationUseCaseTest, RepeatingFlagIsApplied)
{
    addAnimation(2, "SHORT", rover_led::STATE, {"front"}, stubDescription(0.1f));

    use_case_->execute({2, "", true});
    segments_["front"]->updateAnimation();
    segments_["front"]->updateAnimation();

    EXPECT_FALSE(segments_["front"]->isAnimationFinished(rover_led::STATE));
}

TEST_F(SetAnimationUseCaseTest, UnknownIdFails)
{
    EXPECT_THROW(
        {
            try {
                use_case_->execute({7, "", false});
            } catch (const std::runtime_error & e) {
                EXPECT_STREQ(e.what(), "No animation with ID: 7");
                throw;
            }
        },
        std::runtime_error);
}

TEST_F(SetAnimationUseCaseTest, UnknownSegmentFailsWithoutTouchingAnySegment)
{
    addAnimation(3, "GHOST", rover_led::STATE, {"front", "roof"});

    EXPECT_THROW(use_case_->execute({3, "", false}), std::runtime_error);
    EXPECT_FALSE(segments_["front"]->hasAnimation());
    EXPECT_TRUE(factory_->created.empty());
}

TEST_F(SetAnimationUseCaseTest, UnknownTypeFailsWithTheAnimationName)
{
    addAnimation(4, "EXOTIC", rover_led::STATE, {"front"}, stubDescription(), "Exotic");

    try {
        use_case_->execute({4, "", false});
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error & e) {
        EXPECT_THAT(e.what(), ::testing::StartsWith("Failed to set 'EXOTIC' animation: The plugin failed to load."));
    }
}

TEST_F(SetAnimationUseCaseTest, InitializationFailureLeavesCurrentAnimationsInPlace)
{
    use_case_->execute({1, "", false});

    // Second part fails to initialize: nothing may change on "front" either.
    LedAnimationDescription broken{5, rover_led::STATE, "BROKEN", 120.0f, {}};
    broken.animations.push_back(AnimationDescription{"Stub", {"front"}, stubDescription(0.1f)});
    auto failing = stubDescription();
    failing["fail_initialize"] = true;
    broken.animations.push_back(AnimationDescription{"Stub", {"rear"}, failing});
    catalog_->animations[5] = broken;

    try {
        use_case_->execute({5, "", false});
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error & e) {
        EXPECT_THAT(
            e.what(), ::testing::HasSubstr("Failed to set 'BROKEN' animation: Failed to initialize animation"));
    }

    // "front" still runs the 1 s BLINK animation, not the 0.1 s one.
    for (int i = 0; i < 5; i++) {
        segments_["front"]->updateAnimation();
    }

    EXPECT_FALSE(segments_["front"]->isAnimationFinished(rover_led::STATE));
}

TEST_F(SetAnimationUseCaseTest, ReportsSegmentsWhoseAlertQueueIsFull)
{
    addAnimation(6, "ALERT", rover_led::ALERT, {"front"});

    for (std::size_t i = 0; i <= rover_led::SegmentQueueLayer::kMaxQueueSize; i++) {
        EXPECT_TRUE(use_case_->execute({6, "", false}).rejected_segments.empty());
    }

    const auto result = use_case_->execute({6, "", false});

    EXPECT_EQ(result.rejected_segments, (std::vector<std::string>{"front"}));
}
