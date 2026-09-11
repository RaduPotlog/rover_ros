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

#include "rover_led/application/set_animation_use_case.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rover_led
{

SetAnimationUseCase::SetAnimationUseCase(
    std::shared_ptr<const IAnimationCatalog> catalog,
    std::shared_ptr<IAnimationFactory> factory,
    SegmentMap segments,
    const float controller_frequency)
: catalog_(std::move(catalog))
, factory_(std::move(factory))
, segments_(std::move(segments))
, controller_frequency_(controller_frequency)
{

}

SetAnimationResult SetAnimationUseCase::execute(const LedAnimationRequest & request)
{
    const auto description = catalog_->find(request.id);

    if (!description) {
        throw std::runtime_error("No animation with ID: " + std::to_string(request.id));
    }

    for (const auto & animation : description->animations) {
        for (const auto & segment : animation.segments) {
            if (segments_.find(segment) == segments_.end()) {
                throw std::runtime_error("No segment with name: " + segment + ".");
            }
        }
    }

    struct PendingAnimation
    {
        std::string segment_name;
        std::shared_ptr<LedSegment> segment;
        std::shared_ptr<Animation> animation;
    };

    std::vector<PendingAnimation> pending;

    try {
        for (const auto & animation_description : description->animations) {
            for (const auto & segment_name : animation_description.segments) {
                const auto & segment = segments_.at(segment_name);
                auto animation = factory_->create(animation_description.type);

                try {
                    animation->initialize(
                        animation_description.animation, segment->getNumberOfLeds(), controller_frequency_);
                    animation->setParam(request.param);
                } catch (const std::runtime_error & e) {
                    throw std::runtime_error("Failed to initialize animation: " + std::string(e.what()));
                } catch (const std::out_of_range & e) {
                    throw std::runtime_error("Failed to initialize animation: " + std::string(e.what()));
                }

                pending.push_back({segment_name, segment, std::move(animation)});
            }
        }
    } catch (const std::runtime_error & e) {
        throw std::runtime_error(
            "Failed to set '" + description->name + "' animation: " + std::string(e.what()));
    }

    SetAnimationResult result;
    result.name = description->name;

    for (const auto & item : pending) {
        if (!item.segment->setAnimation(item.animation, request.repeating, description->priority)) {
            result.rejected_segments.push_back(item.segment_name);
        }
    }

    return result;
}

}  // namespace rover_led
