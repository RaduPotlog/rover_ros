// Copyright 2026 Mechatronics Academy
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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "rover_led/application/validate_animation_catalog_use_case.hpp"

#include "../test_helpers.hpp"

using rover_led::AnimationDescription;
using rover_led::LedAnimationDescription;
using rover_led::ValidateAnimationCatalogUseCase;
using rover_led::test::StubAnimation;
using rover_led::test::stubDescription;
using ::testing::ElementsAre;
using ::testing::FieldsAre;

namespace
{

// Knows only "Stub". "Broken" fails with something other than std::runtime_error.
class FakeFactory : public rover_led::IAnimationFactory
{

public:

    std::shared_ptr<rover_led::Animation> create(const std::string & type) override
    {
        requested.push_back(type);

        if (type == "Broken") {
            throw std::logic_error("broken factory");
        }

        if (type != "Stub") {
            throw std::runtime_error("The plugin failed to load. Error: unknown " + type);
        }

        return std::make_shared<StubAnimation>();
    }

    std::vector<std::string> requested;
};

class ValidateAnimationCatalogUseCaseTest : public ::testing::Test
{

protected:

    ValidateAnimationCatalogUseCaseTest()
    : factory_(std::make_shared<FakeFactory>())
    , use_case_(factory_)
    {

    }

    // One catalog entry with one animation per type.
    static LedAnimationDescription entry(
        const std::size_t id, const std::string & name, const std::vector<std::string> & types)
    {
        LedAnimationDescription led_animation{
            id, LedAnimationDescription::kDefaultPriority, name, LedAnimationDescription::kDefaultTimeout, {}};

        for (const auto & type : types) {
            led_animation.animations.push_back(AnimationDescription{type, {"front"}, stubDescription()});
        }

        return led_animation;
    }

    std::shared_ptr<FakeFactory> factory_;
    ValidateAnimationCatalogUseCase use_case_;
};

}  // namespace

TEST_F(ValidateAnimationCatalogUseCaseTest, EmptyCatalogReportsNothing)
{
    const auto result = use_case_.execute({});

    EXPECT_EQ(result.unavailable_animations, 0u);
    EXPECT_TRUE(result.unavailable_types.empty());
    EXPECT_TRUE(factory_->requested.empty());
}

TEST_F(ValidateAnimationCatalogUseCaseTest, EveryTypeAvailableReportsNothing)
{
    const auto result = use_case_.execute({entry(1, "BLINK", {"Stub", "Stub"}), entry(2, "READY", {"Stub"})});

    EXPECT_EQ(result.unavailable_animations, 0u);
    EXPECT_TRUE(result.unavailable_types.empty());
}

TEST_F(ValidateAnimationCatalogUseCaseTest, CountsAnAnimationOnceHoweverManyOfItsTypesAreUnavailable)
{
    const auto result = use_case_.execute({entry(5, "MIXED", {"Stub", "Missing", "Gone"})});

    EXPECT_EQ(result.unavailable_animations, 1u);
    EXPECT_THAT(
        result.unavailable_types,
        ElementsAre(FieldsAre(5u, "MIXED", "Missing"), FieldsAre(5u, "MIXED", "Gone")));
}

TEST_F(ValidateAnimationCatalogUseCaseTest, ReportsUnavailableTypesInCatalogOrder)
{
    const auto result = use_case_.execute(
        {entry(1, "A", {"Missing"}), entry(2, "B", {"Stub"}), entry(3, "C", {"Other"})});

    EXPECT_EQ(result.unavailable_animations, 2u);
    EXPECT_THAT(
        result.unavailable_types,
        ElementsAre(FieldsAre(1u, "A", "Missing"), FieldsAre(3u, "C", "Other")));
}

TEST_F(ValidateAnimationCatalogUseCaseTest, AsksTheFactoryForEveryAnimationOfEveryEntry)
{
    use_case_.execute({entry(1, "A", {"Stub", "Missing"}), entry(2, "B", {"Stub"})});

    EXPECT_EQ(factory_->requested, (std::vector<std::string>{"Stub", "Missing", "Stub"}));
}

TEST_F(ValidateAnimationCatalogUseCaseTest, OnlyRuntimeErrorsMarkATypeUnavailable)
{
    EXPECT_THROW(use_case_.execute({entry(1, "A", {"Stub", "Broken"})}), std::logic_error);
}
