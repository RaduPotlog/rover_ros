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

#include "rover_crfs_teleop/domain/switch_debouncer.hpp"

namespace rover_crfs_teleop
{

SwitchDebouncer::SwitchDebouncer(const int threshold, const unsigned int settle_frames)
: threshold_(threshold), settle_frames_remaining_(settle_frames)
{
}

SwitchPosition SwitchDebouncer::classify(const int raw_value) const
{
    return (raw_value < threshold_) ? SwitchPosition::kLow : SwitchPosition::kHigh;
}

std::optional<SwitchPosition> SwitchDebouncer::update(const int raw_value)
{
    const SwitchPosition current = classify(raw_value);

    if (settle_frames_remaining_ > 0) {
        settle_frames_remaining_--;
        position_ = current;
        return std::nullopt;
    }

    // A settle period of 0 frames still must not treat the first frame as an edge.
    if (!position_.has_value()) {
        position_ = current;
        return std::nullopt;
    }

    if (current == *position_) {
        return std::nullopt;
    }

    position_ = current;
    return current;
}

}  // namespace rover_crfs_teleop
