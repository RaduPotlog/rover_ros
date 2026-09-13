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

#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rover_led/infrastructure/yaml_led_config.hpp"

using rover_led::parseLedLayout;
using rover_led::YamlAnimationCatalog;

namespace
{

const char * kLayout = R"(
panels:
  - channel: 1
    number_of_leds: 40
  - channel: 2
    number_of_leds: 20
segments:
  - name: front_1
    channel: 1
    led_range: 0-19
  - name: front_2
    channel: 1
    led_range: 39-20
segments_map:
  all: [front_1, front_2]
  left: [front_1]
)";

std::string expectThrowMessage(const std::function<void()> & call)
{
    try {
        call();
    } catch (const std::exception & e) {
        return e.what();
    }

    ADD_FAILURE() << "expected an exception";

    return "";
}

}  // namespace

TEST(ParseLedLayout, ParsesPanelsSegmentsAndGroups)
{
    const auto layout = parseLedLayout(YAML::Load(kLayout));

    ASSERT_EQ(layout.panels.size(), 2u);
    EXPECT_EQ(layout.panels[1].channel, 2u);
    EXPECT_EQ(layout.panels[1].number_of_leds, 20u);

    ASSERT_EQ(layout.segments.size(), 2u);
    EXPECT_EQ(layout.segments[1].name, "front_2");
    EXPECT_EQ(layout.segments[1].config.channel, 1u);
    EXPECT_EQ(layout.segments[1].config.first_led, 39u);
    EXPECT_EQ(layout.segments[1].config.last_led, 20u);

    EXPECT_EQ(layout.segments_map.at("all"), (std::vector<std::string>{"front_1", "front_2"}));
}

TEST(ParseLedLayout, RejectsDuplicatesAndBadRanges)
{
    auto duplicate_panel = YAML::Load(kLayout);
    duplicate_panel["panels"][1]["channel"] = 1;
    EXPECT_EQ(
        expectThrowMessage([&] { parseLedLayout(duplicate_panel); }),
        "Multiple panels with channel nr '1' found.");

    auto duplicate_segment = YAML::Load(kLayout);
    duplicate_segment["segments"][1]["name"] = "front_1";
    EXPECT_EQ(
        expectThrowMessage([&] { parseLedLayout(duplicate_segment); }),
        "Failed to initialize 'front_1' segment: Multiple segments with given name found.");

    auto no_dash = YAML::Load(kLayout);
    no_dash["segments"][0]["led_range"] = "0:19";
    EXPECT_THAT(
        expectThrowMessage([&] { parseLedLayout(no_dash); }),
        ::testing::HasSubstr("No '-' character found"));

    auto not_a_number = YAML::Load(kLayout);
    not_a_number["segments"][0]["led_range"] = "a-19";
    EXPECT_THAT(
        expectThrowMessage([&] { parseLedLayout(not_a_number); }),
        ::testing::HasSubstr("Error converting string to integer."));
}

TEST(ResolvePackageSubstitution, ResolvesFindToTheShareDirectory)
{
    const auto share = ament_index_cpp::get_package_share_path("ament_index_cpp").string();

    EXPECT_EQ(rover_led::resolvePackageSubstitution("$(find ament_index_cpp)/a/b.png"), share + "/a/b.png");
    EXPECT_EQ(rover_led::resolvePackageSubstitution("$(find  ament_index_cpp )/x"), share + "/x");
    EXPECT_EQ(rover_led::resolvePackageSubstitution("/abs/path.png"), "/abs/path.png");
    EXPECT_EQ(rover_led::resolvePackageSubstitution("plain"), "plain");
    EXPECT_FALSE(rover_led::resolvePackageSubstitution("$(find no_such_package_xyz)/x.png"));
    EXPECT_FALSE(rover_led::resolvePackageSubstitution("$(find broken"));
}

TEST(YamlAnimationCatalog, LoadsEntriesWithDefaults)
{
    const auto layout = parseLedLayout(YAML::Load(kLayout));
    const YamlAnimationCatalog catalog(YAML::Load(R"(
- id: 4
  animations:
    - type: rover_led::ImageAnimation
      segments: left
      animation: {image: $(find ament_index_cpp)/img.png, duration: 2, frames: [$(find ament_index_cpp)]}
- id: 1
  name: ERROR
  priority: 0
  timeout: 5.5
  animations:
    - type: rover_led::ImageAnimation
      segments: all
      animation: {image: /abs.png, duration: 1}
)"), layout.segments_map);

    ASSERT_FALSE(catalog.find(2));

    const auto defaults = catalog.find(4);
    ASSERT_TRUE(defaults);
    EXPECT_EQ(defaults->name, "ANIMATION_4");
    EXPECT_EQ(defaults->priority, 3);
    EXPECT_FLOAT_EQ(defaults->timeout, 120.0f);
    ASSERT_EQ(defaults->animations.size(), 1u);
    EXPECT_EQ(defaults->animations[0].segments, (std::vector<std::string>{"front_1"}));

    const auto share = ament_index_cpp::get_package_share_path("ament_index_cpp").string();
    EXPECT_EQ(defaults->animations[0].animation["image"].as<std::string>(), share + "/img.png");
    EXPECT_EQ(defaults->animations[0].animation["frames"][0].as<std::string>(), share);
    EXPECT_EQ(defaults->animations[0].animation["duration"].as<int>(), 2);

    const auto explicit_values = catalog.find(1);
    ASSERT_TRUE(explicit_values);
    EXPECT_EQ(explicit_values->name, "ERROR");
    EXPECT_EQ(explicit_values->priority, 0);
    EXPECT_FLOAT_EQ(explicit_values->timeout, 5.5f);

    const auto all = catalog.getAll();
    ASSERT_EQ(all.size(), 2u);
    EXPECT_EQ(all[0].id, 1u);
    EXPECT_TRUE(catalog.warnings().empty());
}

TEST(YamlAnimationCatalog, KeepsUnresolvableSubstitutionsAndWarns)
{
    const auto layout = parseLedLayout(YAML::Load(kLayout));
    const YamlAnimationCatalog catalog(YAML::Load(R"(
- id: 5
  name: LOW_BATTERY
  animations:
    - type: other_pkg::MovingImageAnimation
      segments: all
      animation: {image: $(find no_such_package_xyz)/low.png, duration: 1}
)"), layout.segments_map);

    ASSERT_TRUE(catalog.find(5));
    EXPECT_EQ(catalog.find(5)->animations[0].animation["image"].as<std::string>(), "$(find no_such_package_xyz)/low.png");
    ASSERT_EQ(catalog.warnings().size(), 1u);
    EXPECT_THAT(catalog.warnings()[0], ::testing::HasSubstr("LOW_BATTERY"));
}

TEST(YamlAnimationCatalog, RejectsInvalidEntries)
{
    const auto segments_map = parseLedLayout(YAML::Load(kLayout)).segments_map;
    const std::string animation = "animations: [{type: T, segments: all, animation: {duration: 1}}]";

    EXPECT_EQ(
        expectThrowMessage([&] {
            YamlAnimationCatalog(YAML::Load("[{id: 1, name: BAD, priority: 4, " + animation + "}]"), segments_map);
        }),
        "Failed to load 'BAD' animation: Invalid LED animation priority.");

    EXPECT_EQ(
        expectThrowMessage([&] {
            YamlAnimationCatalog(
                YAML::Load("[{id: 1, name: A, " + animation + "}, {id: 1, name: B, " + animation + "}]"),
                segments_map);
        }),
        "Failed to load 'B' animation: Animation with given ID already exists.");

    EXPECT_EQ(
        expectThrowMessage([&] {
            YamlAnimationCatalog(
                YAML::Load("[{id: 1, name: C, animations: [{type: T, segments: roof, animation: {}}]}]"),
                segments_map);
        }),
        "Failed to load 'C' animation: Unknown segments group 'roof'.");
}

TEST(YamlAnimationCatalog, ParsesTheShippedRoverA1Config)
{
    const auto config = YAML::LoadFile(std::string(ROVER_LED_SOURCE_DIR) + "/config/rover_a1_animations.yaml");
    const auto layout = parseLedLayout(config);
    const YamlAnimationCatalog catalog(config["led_animations"], layout.segments_map);

    EXPECT_EQ(layout.panels.size(), 2u);
    EXPECT_EQ(layout.segments.size(), 4u);
    EXPECT_TRUE(catalog.find(0));
    EXPECT_EQ(catalog.find(2)->priority, 0);
}

TEST(YamlAnimationCatalog, ShippedAnimationsUseRoverLedPluginsAndImages)
{
    const std::string source_dir(ROVER_LED_SOURCE_DIR);
    const auto config = YAML::LoadFile(source_dir + "/config/rover_a1_animations.yaml");
    const auto layout = parseLedLayout(config);
    const YamlAnimationCatalog catalog(config["led_animations"], layout.segments_map);

    // Every id rover_safety requests (rover_msgs/LedAnimation E_STOP ..
    // BATTERY_NOMINAL) must exist.
    for (std::size_t id = 0; id <= 10; id++) {
        EXPECT_TRUE(catalog.find(id)) << "missing animation id " << id;
    }

    // rover_led may not be on the ament index while testing; resolve it here.
    const std::string find_self = "$(find rover_led)";

    for (const auto & led_animation : catalog.getAll()) {
        for (const auto & animation : led_animation.animations) {
            EXPECT_THAT(
                animation.type,
                ::testing::AnyOf("rover_led::ImageAnimation", "rover_led::MovingImageAnimation"))
                << led_animation.name;

            auto image = animation.animation["image"].as<std::string>();

            if (image.rfind(find_self, 0) == 0) {
                image = source_dir + image.substr(find_self.size());
            }

            EXPECT_TRUE(std::filesystem::exists(image)) << led_animation.name << ": " << image;
        }
    }
}
