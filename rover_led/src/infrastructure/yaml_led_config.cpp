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

#include "rover_led/infrastructure/yaml_led_config.hpp"

#include <algorithm>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "yaml-cpp/yaml.h"

#include "rover_utils/yaml_utils.hpp"

namespace rover_led
{

namespace
{

constexpr const char * kFindPrefix = "$(find ";

std::string trim(const std::string & value)
{
    const auto first = value.find_first_not_of(" \t");

    if (first == std::string::npos) {
        return "";
    }

    const auto last = value.find_last_not_of(" \t");

    return value.substr(first, last - first + 1);
}

}  // namespace

LedLayoutConfig parseLedLayout(const YAML::Node & led_config)
{
    LedLayoutConfig layout;
    std::set<std::size_t> channels;

    for (const auto & panel : rover_utils::getYAMLKeyValue<std::vector<YAML::Node>>(led_config, "panels")) {
        LedPanelConfig panel_config;
        panel_config.channel = rover_utils::getYAMLKeyValue<std::size_t>(panel, "channel");
        panel_config.number_of_leds = rover_utils::getYAMLKeyValue<std::size_t>(panel, "number_of_leds");

        if (panel["rows"]) {
            panel_config.rows = rover_utils::getYAMLKeyValue<std::size_t>(panel, "rows");
        }

        if (panel_config.rows == 0 || panel_config.number_of_leds % panel_config.rows != 0) {
            throw std::runtime_error(
                "Panel with channel nr '" + std::to_string(panel_config.channel) + "' can not fold " +
                std::to_string(panel_config.number_of_leds) + " LEDs into " +
                std::to_string(panel_config.rows) + " rows.");
        }

        if (!channels.insert(panel_config.channel).second) {
            throw std::runtime_error(
                "Multiple panels with channel nr '" + std::to_string(panel_config.channel) + "' found.");
        }

        layout.panels.push_back(panel_config);
    }

    std::set<std::string> segment_names;

    for (const auto & segment : rover_utils::getYAMLKeyValue<std::vector<YAML::Node>>(led_config, "segments")) {
        const auto segment_name = rover_utils::getYAMLKeyValue<std::string>(segment, "name");

        try {
            if (!segment_names.insert(segment_name).second) {
                throw std::runtime_error("Multiple segments with given name found.");
            }

            layout.segments.push_back({segment_name, parseLedSegment(segment)});
        } catch (const std::exception & e) {
            throw std::runtime_error(
                "Failed to initialize '" + segment_name + "' segment: " + std::string(e.what()));
        }
    }

    if (led_config["segments_map"]) {
        for (const auto & key : led_config["segments_map"]) {
            layout.segments_map.emplace(key.first.as<std::string>(), key.second.as<std::vector<std::string>>());
        }
    }

    return layout;
}

LedSegmentConfig parseLedSegment(const YAML::Node & segment_description)
{
    LedSegmentConfig config;
    config.channel = rover_utils::getYAMLKeyValue<std::size_t>(segment_description, "channel");

    const auto led_range = rover_utils::getYAMLKeyValue<std::string>(segment_description, "led_range");
    const std::size_t split_char = led_range.find('-');

    if (split_char == std::string::npos) {
        throw std::invalid_argument("No '-' character found in the led_range expression.");
    }

    try {
        config.first_led = std::stoi(led_range.substr(0, split_char));
        config.last_led = std::stoi(led_range.substr(split_char + 1));
    } catch (const std::invalid_argument & e) {
        throw std::invalid_argument("Error converting string to integer.");
    }

    return config;
}

std::optional<std::string> resolvePackageSubstitution(const std::string & value)
{
    if (value.rfind(kFindPrefix, 0) != 0) {
        return value;
    }

    const auto closing = value.find(')');

    if (closing == std::string::npos) {
        return std::nullopt;
    }

    const std::string prefix(kFindPrefix);
    const auto package = trim(value.substr(prefix.size(), closing - prefix.size()));

    try {
        return ament_index_cpp::get_package_share_path(package).string() + value.substr(closing + 1);
    } catch (const ament_index_cpp::PackageNotFoundError & /*e*/) {
        return std::nullopt;
    }
}

YamlAnimationCatalog::YamlAnimationCatalog(
    const YAML::Node & animations_description,
    const SegmentsMap & segments_map)
{
    for (const auto & animation_description : animations_description.as<std::vector<YAML::Node>>()) {
        loadAnimation(animation_description, segments_map);
    }
}

std::optional<LedAnimationDescription> YamlAnimationCatalog::find(const std::size_t id) const
{
    const auto it = animations_.find(id);

    if (it == animations_.end()) {
        return std::nullopt;
    }

    return it->second;
}

std::vector<LedAnimationDescription> YamlAnimationCatalog::getAll() const
{
    std::vector<LedAnimationDescription> animations;

    for (const auto & [id, animation] : animations_) {
        animations.push_back(animation);
    }

    std::sort(animations.begin(), animations.end(), [](const auto & a, const auto & b) {
        return a.id < b.id;
    });

    return animations;
}

void YamlAnimationCatalog::loadAnimation(
    const YAML::Node & animation_description,
    const SegmentsMap & segments_map)
{
    LedAnimationDescription led_animation_desc;

    try {
        led_animation_desc.id = rover_utils::getYAMLKeyValue<std::size_t>(animation_description, "id");
        led_animation_desc.name = rover_utils::getYAMLKeyValue<std::string>(
            animation_description, "name", "ANIMATION_" + std::to_string(led_animation_desc.id));
        led_animation_desc.priority = rover_utils::getYAMLKeyValue<std::uint8_t>(
            animation_description, "priority", LedAnimationDescription::kDefaultPriority);
        led_animation_desc.timeout = rover_utils::getYAMLKeyValue<float>(
            animation_description, "timeout", LedAnimationDescription::kDefaultTimeout);

        const auto & valid_priorities = LedAnimationDescription::kValidPriorities;

        if (std::find(valid_priorities.begin(), valid_priorities.end(), led_animation_desc.priority) ==
            valid_priorities.end()) {
            throw std::runtime_error("Invalid LED animation priority.");
        }

        const auto animations =
            rover_utils::getYAMLKeyValue<std::vector<YAML::Node>>(animation_description, "animations");

        for (const auto & animation : animations) {
            AnimationDescription animation_desc;
            animation_desc.type = rover_utils::getYAMLKeyValue<std::string>(animation, "type");
            animation_desc.animation = resolveSubstitutions(
                rover_utils::getYAMLKeyValue<YAML::Node>(animation, "animation"), led_animation_desc.name);

            const auto segments_group = rover_utils::getYAMLKeyValue<std::string>(animation, "segments");
            const auto group = segments_map.find(segments_group);

            if (group == segments_map.end()) {
                throw std::runtime_error("Unknown segments group '" + segments_group + "'.");
            }

            animation_desc.segments = group->second;
            led_animation_desc.animations.push_back(animation_desc);
        }

        if (!animations_.emplace(led_animation_desc.id, led_animation_desc).second) {
            throw std::runtime_error("Animation with given ID already exists.");
        }
    } catch (const std::runtime_error & e) {
        throw std::runtime_error(
            "Failed to load '" + led_animation_desc.name + "' animation: " + std::string(e.what()));
    }
}

YAML::Node YamlAnimationCatalog::resolveSubstitutions(
    const YAML::Node & node,
    const std::string & animation_name)
{
    YAML::Node resolved = YAML::Clone(node);

    if (resolved.IsScalar()) {
        const auto value = resolved.as<std::string>();
        const auto resolved_value = resolvePackageSubstitution(value);

        if (resolved_value) {
            if (*resolved_value != value) {
                resolved = *resolved_value;
            }
        } else {
            warnings_.push_back(
                "Animation '" + animation_name + "': can't resolve '" + value + "'.");
        }
    } else if (resolved.IsMap()) {
        for (auto it = resolved.begin(); it != resolved.end(); ++it) {
            it->second = resolveSubstitutions(it->second, animation_name);
        }
    } else if (resolved.IsSequence()) {
        for (std::size_t i = 0; i < resolved.size(); ++i) {
            resolved[i] = resolveSubstitutions(resolved[i], animation_name);
        }
    }

    return resolved;
}

}  // namespace rover_led
