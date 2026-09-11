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

#include "rover_led/application/get_led_state_use_case.hpp"

#include <algorithm>
#include <utility>

namespace rover_led
{

GetLedStateUseCase::GetLedStateUseCase(SegmentMap segments)
: segments_(std::move(segments))
{

}

LedStateSnapshot GetLedStateUseCase::execute() const
{
    LedStateSnapshot snapshot;

    for (const auto & [name, segment] : segments_) {
        LedSegmentSnapshot segment_snapshot{name, segment->getChannel(), {}};

        for (const auto & [priority, status] : segment->getLayerStatuses()) {
            segment_snapshot.layers.push_back({priority, status});
        }

        snapshot.segments.push_back(std::move(segment_snapshot));
    }

    std::sort(snapshot.segments.begin(), snapshot.segments.end(), [](const auto & a, const auto & b) {
        return a.name < b.name;
    });

    return snapshot;
}

}  // namespace rover_led
