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

#ifndef ROVER_LED_DOMAIN_LED_COMPONENTS_LED_SEGMENT_HPP_
#define ROVER_LED_DOMAIN_LED_COMPONENTS_LED_SEGMENT_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "rover_led/domain/animation/animation.hpp"
#include "rover_led/domain/led_components/segment_layer_interface.hpp"

namespace rover_led
{

enum AnimationPriority {
    ERROR = 0,
    ALERT,
    INFO,
    STATE,
};

// A virtual LED strip mapped onto a range of a panel. first_led > last_led
// means the segment runs backwards on the panel.
struct LedSegmentConfig
{
    std::size_t channel;
    std::size_t first_led;
    std::size_t last_led;
};

class LedSegment
{

public:

    explicit LedSegment(const LedSegmentConfig & config);

    ~LedSegment() {};

    // The animation has to be initialized for getNumberOfLeds() LEDs.
    // Returns false if the priority layer rejected it (full ALERT queue).
    bool setAnimation(
        const std::shared_ptr<Animation> & animation,
        const bool repeating,
        const std::uint8_t priority);

    void updateAnimation();

    bool isAnimationFinished(AnimationPriority layer) const;

    std::vector<std::uint8_t> getAnimationFrame() const;

    float getAnimationProgress(AnimationPriority layer) const;

    void resetAnimation(AnimationPriority layer);

    std::size_t getFirstLedPosition() const;

    std::size_t getChannel() const
    {
        return channel_;
    }

    std::size_t getNumberOfLeds() const
    {
        return num_led_;
    }

    bool layerHasAnimation(AnimationPriority layer) const;

    bool hasAnimation() const;

    // One entry per layer, ordered ERROR to STATE; empty where nothing plays.
    std::map<AnimationPriority, std::optional<LayerStatus>> getLayerStatuses() const;

protected:

    std::vector<std::uint8_t> mergeLayersFrames() const;

    void mergeFrames(
        std::vector<std::uint8_t> & base_frame,
        const std::vector<std::uint8_t> & overlay_frame) const;

private:

    bool invert_led_order_ = false;
    std::size_t channel_;
    std::size_t first_led_iterator_;
    std::size_t last_led_iterator_;
    std::size_t num_led_;
    std::map<AnimationPriority, std::unique_ptr<SegmentLayerInterface>> layers_;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_LED_COMPONENTS_LED_SEGMENT_HPP_
