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

#include "rover_led/application/stop_animation_use_case.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace rover_led
{

StopAnimationUseCase::StopAnimationUseCase(
    std::shared_ptr<const IAnimationCatalog> catalog,
    SegmentMap segments)
: catalog_(std::move(catalog))
, segments_(std::move(segments))
{

}

StopAnimationResult StopAnimationUseCase::execute(const std::size_t id)
{
    const auto description = catalog_->find(id);

    if (!description) {
        throw std::runtime_error("No animation with ID: " + std::to_string(id));
    }

    StopAnimationResult result;
    result.name = description->name;

    for (const auto & animation : description->animations) {
        for (const auto & segment_name : animation.segments) {
            const auto segment = segments_.find(segment_name);

            if (segment == segments_.end()) {
                throw std::runtime_error("No segment with name: " + segment_name + ".");
            }

            // A segment can be listed by several of the description's animations.
            if (std::find(result.stopped_segments.begin(), result.stopped_segments.end(), segment_name) !=
                result.stopped_segments.end())
            {
                continue;
            }

            if (segment->second->stopAnimation(id, description->priority)) {
                result.stopped_segments.push_back(segment_name);
            }
        }
    }

    return result;
}

}  // namespace rover_led
