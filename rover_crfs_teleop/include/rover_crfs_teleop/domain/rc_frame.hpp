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

#ifndef ROVER_CRFS_TELEOP_DOMAIN_RC_FRAME_HPP_
#define ROVER_CRFS_TELEOP_DOMAIN_RC_FRAME_HPP_

#include <array>
#include <cstddef>
#include <optional>

namespace rover_crfs_teleop
{

// One decoded CRSF frame: the raw values of all 16 RC channels, exactly as crsf_receiver
// republishes them on rc/channels. Kept free of crsf_receiver_msg so the teleop rules can be
// tested without ROS - the node converts at the subscription boundary.
struct RcFrame
{
    static constexpr std::size_t kChannelCount = 16;

    // Indexed 0-based: channel N lives at index N-1.
    std::array<int, kChannelCount> channels{};

    // Returns channel `channel_number` (1-16, the numbering the transmitter and the ROS
    // parameters use), or nullopt for a number outside that range - a misconfiguration the caller
    // is expected to report rather than silently read as some default value.
    std::optional<int> channel(const int channel_number) const
    {
        if (channel_number < 1 || static_cast<std::size_t>(channel_number) > kChannelCount) {
            return std::nullopt;
        }

        return channels[static_cast<std::size_t>(channel_number - 1)];
    }
};

}  // namespace rover_crfs_teleop

#endif  // ROVER_CRFS_TELEOP_DOMAIN_RC_FRAME_HPP_
