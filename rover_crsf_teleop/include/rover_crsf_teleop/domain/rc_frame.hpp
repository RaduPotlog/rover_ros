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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_RC_FRAME_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_RC_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace rover_crsf_teleop
{

// One decoded CRSF frame: the raw values of all 16 RC channels, exactly as they arrive on the
// wire. Kept free of any ROS message type so the teleop rules can be tested without ROS - the
// parser produces this directly, and the node converts it to rover_msgs only to echo it.
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

// One decoded CRSF LINK_STATISTICS frame (type 0x14).
//
// Only `uplink_link_quality` gates teleop - see LinkMonitor. The rest is carried so the node can
// echo it on rc/link for tuning and for bags; nothing in the teleop rules reads it.
//
// The two SNR fields are SIGNED. The message this used to arrive in typed uplink_snr as an
// unsigned byte, so negative uplink SNR - the interesting case, a marginal link - was published
// wrong for as long as that message existed.
struct RcLinkStats
{
    std::uint8_t uplink_rssi_ant1{0};       // dBm * -1
    std::uint8_t uplink_rssi_ant2{0};       // dBm * -1
    std::uint8_t uplink_link_quality{0};    // packet success rate, 0-100 %
    std::int8_t  uplink_snr{0};             // dB
    std::uint8_t active_antenna{0};         // 0 = antenna 1, 1 = antenna 2
    std::uint8_t rf_mode{0};                // 0 = 4 Hz, 1 = 50 Hz, 2 = 150 Hz
    std::uint8_t uplink_tx_power{0};        // enum, 0 = 0 mW .. 6 = 2000 mW
    std::uint8_t downlink_rssi{0};          // dBm * -1
    std::uint8_t downlink_link_quality{0};  // packet success rate, 0-100 %
    std::int8_t  downlink_snr{0};           // dB
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_RC_FRAME_HPP_
