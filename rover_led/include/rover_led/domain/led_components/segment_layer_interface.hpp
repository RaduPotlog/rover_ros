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

#ifndef ROVER_LED_DOMAIN_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_
#define ROVER_LED_DOMAIN_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "rover_led/domain/animation/animation.hpp"

namespace rover_led
{

// What a layer is playing right now.
struct LayerStatus
{
    AnimationInfo info;
    bool repeating = false;
    float progress = 0.0f;
    // Animations waiting behind the current one.
    std::size_t queued = 0;
};

// One priority layer of a segment. Layers receive animations that are already
// created and initialized for the segment (see SetAnimationUseCase).
class SegmentLayerInterface
{

public:

    SegmentLayerInterface(const std::size_t num_led, const bool invert_led_order)
    : invert_led_order_(invert_led_order)
    , num_led_(num_led)
    {

    }

    virtual ~SegmentLayerInterface() = default;

    // Returns false if the layer rejected the animation (e.g. full queue).
    virtual bool setAnimation(
        const std::shared_ptr<Animation> & animation,
        const bool repeating) = 0;

    virtual void updateAnimation() = 0;

    bool isAnimationFinished() const { return animation_finished_; }

    std::vector<std::uint8_t> getAnimationFrame() const
    {
        if (animation_finished_ || !animation_) {
            return std::vector<std::uint8_t>(4 * num_led_, 0);
        }

        return animation_->getFrame(invert_led_order_);
    }

    float getAnimationProgress() const
    {
        if (!animation_) {
            throw std::runtime_error("Segment animation not defined.");
        }

        return animation_->getProgress();
    }

    void resetAnimation()
    {
        if (!animation_) {
            throw std::runtime_error("Segment animation not defined.");
        }

        animation_->reset();
        animation_finished_ = false;
    }

    bool hasAnimation() const
    {
        return static_cast<bool>(animation_);
    }

    // Empty when nothing is playing (no animation, or a one-shot that ended).
    std::optional<LayerStatus> getStatus() const
    {
        if (!animation_ || animation_finished_) {
            return std::nullopt;
        }

        return LayerStatus{animation_->getInfo(), isRepeating(), animation_->getProgress(), getQueueSize()};
    }

protected:

    virtual bool isRepeating() const { return false; }

    virtual std::size_t getQueueSize() const { return 0; }

    std::shared_ptr<Animation> animation_;

    bool invert_led_order_ = false;
    bool animation_finished_ = true;
    std::size_t num_led_;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_
