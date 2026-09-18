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

#ifndef ROVER_LED_INFRASTRUCTURE_YAML_LED_CONFIG_HPP_
#define ROVER_LED_INFRASTRUCTURE_YAML_LED_CONFIG_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "rover_led/domain/led_components/led_animation_description.hpp"
#include "rover_led/domain/led_components/led_segment.hpp"
#include "rover_led/domain/ports/animation_catalog.hpp"

namespace rover_led
{

// Parsers for the LED configuration file (e.g. config/rover_a1_animations.yaml).
// They throw std::runtime_error with a description of the offending entry.

struct LedPanelConfig
{
    std::size_t channel;
    std::size_t number_of_leds;
    // Serpentine rows the strip is folded into (optional, default 1), see LedPanel.
    std::size_t rows = 1;
};

struct NamedLedSegmentConfig
{
    std::string name;
    LedSegmentConfig config;
};

using SegmentsMap = std::unordered_map<std::string, std::vector<std::string>>;

struct LedLayoutConfig
{
    std::vector<LedPanelConfig> panels;
    std::vector<NamedLedSegmentConfig> segments;
    // Named segment groups referenced by animations ("all: [front_1, ...]").
    SegmentsMap segments_map;
};

// Parses the "panels", "segments" and "segments_map" sections.
LedLayoutConfig parseLedLayout(const YAML::Node & led_config);

// Parses "<first>-<last>", e.g. "39-20".
LedSegmentConfig parseLedSegment(const YAML::Node & segment_description);

// Replaces a leading "$(find <pkg>)" with the package share directory.
// Returns std::nullopt if the package can't be found; other strings are
// returned unchanged.
std::optional<std::string> resolvePackageSubstitution(const std::string & value);

// The "led_animations" section as an animation catalog. Package
// substitutions in animation descriptions are resolved while loading;
// unresolvable ones are left in place and reported by warnings().
class YamlAnimationCatalog : public IAnimationCatalog
{

public:

    YamlAnimationCatalog(const YAML::Node & animations_description, const SegmentsMap & segments_map);

    std::optional<LedAnimationDescription> find(const std::size_t id) const override;

    std::vector<LedAnimationDescription> getAll() const;

    const std::vector<std::string> & warnings() const
    {
        return warnings_;
    }

private:

    void loadAnimation(const YAML::Node & animation_description, const SegmentsMap & segments_map);

    YAML::Node resolveSubstitutions(const YAML::Node & node, const std::string & animation_name);

    std::unordered_map<std::size_t, LedAnimationDescription> animations_;
    std::vector<std::string> warnings_;
};

}  // namespace rover_led

#endif  // ROVER_LED_INFRASTRUCTURE_YAML_LED_CONFIG_HPP_
