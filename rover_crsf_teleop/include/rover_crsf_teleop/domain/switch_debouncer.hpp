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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_SWITCH_DEBOUNCER_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_SWITCH_DEBOUNCER_HPP_

#include <optional>

namespace rover_crsf_teleop
{

constexpr unsigned int kDefaultSwitchSettleFrames = 100;

enum class SwitchPosition
{
    kLow,
    kHigh,
};

// Turns the raw value of a two-position RC switch channel into position *changes*.
//
// Two properties matter, both because each change fires an E-Stop service call:
//
//   - Startup. The first `settle_frames` frames only record where the switch rests and never
//     emit. Otherwise the very first frame would look like an edge against an arbitrary initial
//     value and fire a spurious E-Stop set-or-reset purely from the switch's resting position.
//
//   - Jitter. Only a change of the *logical* position (raw below `threshold` = low) emits. The
//     previous implementation compared raw values, so a single count of noise on a switch channel
//     re-sent the E-Stop set or reset.
class SwitchDebouncer
{

public:

    SwitchDebouncer(int threshold, unsigned int settle_frames = kDefaultSwitchSettleFrames);

    // Feeds one frame's raw channel value. Returns the new position on a change, nullopt
    // otherwise (including every frame of the settle period).
    std::optional<SwitchPosition> update(int raw_value);

    // Last classified position, nullopt before the first frame. Includes the settle period.
    std::optional<SwitchPosition> position() const { return position_; }

private:

    SwitchPosition classify(int raw_value) const;

    int threshold_;
    unsigned int settle_frames_remaining_;
    std::optional<SwitchPosition> position_;
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_SWITCH_DEBOUNCER_HPP_
