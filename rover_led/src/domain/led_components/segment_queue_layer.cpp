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

#include "rover_led/domain/led_components/segment_queue_layer.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

namespace rover_led
{

SegmentQueueLayer::SegmentQueueLayer(const std::size_t num_led, const bool invert_led_order)
: SegmentLayerInterface(num_led, invert_led_order)
{

}

bool SegmentQueueLayer::setAnimation(
    const std::shared_ptr<Animation> & animation,
    const bool /* repeating */)
{
    bool accepted = true;

    if (!animation_finished_) {
        if (animations_queue_.size() < kMaxQueueSize) {
            animations_queue_.emplace_back(animation);
        } else {
            accepted = false;
        }
    } else {
        animation_ = animation;
    }

    animation_finished_ = false;

    return accepted;
}

void SegmentQueueLayer::updateAnimation()
{
    if (!animation_) {
        throw std::runtime_error("Segment animation not defined.");
    }

    if (animation_->isFinished()) {
        animation_finished_ = true;
    }

    if (animation_finished_) {
        if (animations_queue_.empty()) {
            animation_.reset();
            return;
        }

        animation_ = animations_queue_.front();
        animations_queue_.pop_front();
        animation_finished_ = false;
    }

    try {
        animation_->update();
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Failed to update animation: " + std::string(e.what()));
    }
}

bool SegmentQueueLayer::stopAnimation(const std::size_t id)
{
    const auto queued = animations_queue_.size();

    animations_queue_.erase(
        std::remove_if(
            animations_queue_.begin(), animations_queue_.end(),
            [id](const std::shared_ptr<Animation> & animation) { return animation->getInfo().id == id; }),
        animations_queue_.end());

    bool stopped = animations_queue_.size() != queued;

    if (isPlaying(id)) {
        stopped = true;

        if (animations_queue_.empty()) {
            animation_.reset();
            animation_finished_ = true;
        } else {
            animation_ = animations_queue_.front();
            animations_queue_.pop_front();
        }
    }

    return stopped;
}

}  // namespace rover_led
